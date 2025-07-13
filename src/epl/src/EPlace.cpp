#include "epl/EPlace.h"

#include <random>
#include <algorithm>

#include "gpl/placerBase.h"
#include "gpl/nesterovBase.h"
#include "sta/StaMain.hh"
#include "odb/util.h"

namespace sta {
// Tcl files encoded into strings.
extern const char* eplace_tcl_inits[];
}  // namespace sta

namespace epl {

using utl::EPL;

extern "C" {
extern int Epl_Init(Tcl_Interp* interp);
}

EPlace::EPlace()
{
}

EPlace::~EPlace()
{
  clear();
}

void EPlace::init(odb::dbDatabase* db, utl::Logger* logger)
{
  clear();
  db_ = db;
  log_ = logger;
}

bool EPlace::initPlacer()
{
  if (pbc_) {
    log_->warn(EPL, 2, "Placer already initialized.");
    return true;
  }
  // Init PlacerBaseCommon
  gpl::PlacerBaseVars pbVars;
  pbc_ = std::make_shared<gpl::PlacerBaseCommon>(db_, pbVars, log_);
  if (pbc_->placeInsts().size() == 0) {
    log_->warn(EPL, 1, "No placeable instances - skipping placement.");
    return false;
  }

  // Init PlacerBase
  pbVec_.push_back(std::make_shared<gpl::PlacerBase>(db_, pbc_, log_));
  for (auto pd : db_->getChip()->getBlock()->getPowerDomains()) {
    if (pd->getGroup()) {
      pbVec_.push_back(
          std::make_shared<gpl::PlacerBase>(db_, pbc_, log_, pd->getGroup()));
    }
  }
  return true;
}

void EPlace::clear()
{
  // clear instances
  pbc_.reset();
  pbVec_.clear();
}

void EPlace::randomPlace(int threads)
{
  debugPrint(log_,
             EPL,
             "random_place",
             2,
             "random_place: number of threads {}",
             threads);
  if (!initPlacer()) {
    return;
  }

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreRect = block->getCoreArea();

  auto insts = pbc_->placeInsts();
  int n_inst = insts.size();
  int core_x = coreRect.dx();
  int core_y = coreRect.dx();
  int core_x_min = coreRect.xMin();
  int core_y_min = coreRect.yMin();

  std::default_random_engine generator(42);
  std::uniform_int_distribution<int> distrib(0);
  std::vector<int> pos_x(n_inst), pos_y(n_inst);
  std::generate(
      pos_x.begin(), pos_x.end(), [&]() { return distrib(generator); });
  std::generate(
      pos_y.begin(), pos_y.end(), [&]() { return distrib(generator); });

  #pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n_inst; i++) {
    auto inst = insts[i];
    inst->dbSetLocation(
      core_x_min + pos_x[i]%(core_x-inst->dx()), 
      core_y_min + pos_y[i]%(core_y-inst->dy())
    );
    //inst->dbSetPlaced();
  }
}

void EPlace::simulatedAnnealingSimple(int threads, int wait_iterations, double initial_T, double alpha)
{
  debugPrint(log_,
             EPL,
             "simulatedAnnealing",
             2,
             "simulatedAnnealing: number of threads {}",
             threads);

  // Make a random placement
  randomPlace(threads);
  log_->report("initial HPWL: {}", pbc_->hpwl());

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreRect = block->getCoreArea();
  auto insts = pbc_->placeInsts();
  int n_inst = insts.size();
  std::default_random_engine generator(42);
  std::uniform_int_distribution<int> distrib(0);

  double max_disp_x = coreRect.dx();
  double max_disp_y = coreRect.dy();
  int64_t best_hpwl = pbc_->hpwl();
  int64_t last_hpwl = best_hpwl;
  int64_t current_hpwl = 0;
  int last_change = 0;
  int current_iter = 0;
  double T = initial_T;
  double log_T = log(T);

  int count = 0;
  while (current_iter++-last_change <= wait_iterations) {
    // generate new placement
    auto inst = insts[distrib(generator) % n_inst];

    // remove from hpwl only the nets that will change
    current_hpwl = last_hpwl;
    #pragma omp parallel for num_threads(threads)
    for (auto &pin : inst->pins()) {
      current_hpwl -= pin->net()->hpwl();
    }

    int m_disp_x = max_disp_x;
    int m_disp_y = max_disp_y;
    int orig_x = inst->lx(), orig_y = inst->ly();
    int min_x = std::max(inst->lx()-m_disp_x, coreRect.xMin());
    int max_x = std::min(inst->lx()+m_disp_x, coreRect.xMax()-inst->dx());
    int min_y = std::max(inst->ly()-m_disp_y, coreRect.yMin());
    int max_y = std::min(inst->ly()+m_disp_y, coreRect.yMax()-inst->dy());

    int new_x = min_x + distrib(generator) % (max_x-min_x);
    int new_y = min_y + distrib(generator) % (max_y-min_y);
    inst->dbSetLocation(new_x, new_y);

    // add back the changed nets to the hpwl
    #pragma omp parallel for num_threads(threads)
    for (auto &pin : inst->pins()) {
      auto net = pin->net();
      net->updateBox(pbc_->skipIoMode());
      current_hpwl += net->hpwl();
    }
    // debug
    //if (current_hpwl != pbc_->hpwl()) {
    //  log_->warn(EPL, 3, "iter {} | current_hpwl {} != pbc_->hpwl() {}", current_iter, current_hpwl, pbc_->hpwl());
    //}

    if (best_hpwl > current_hpwl) {
      best_hpwl = current_hpwl;
      last_hpwl = current_hpwl;

      if (max_disp_x != 1 or max_disp_y != 1) {
        last_change = current_iter;
      }
    } else {
      inst->dbSetLocation(orig_x, orig_y);
      #pragma omp parallel for num_threads(threads)
      for (auto &pin : inst->pins()) {
        auto net = pin->net();
        net->updateBox(pbc_->skipIoMode());
      }
    }

    if (++count == 100) {
      log_->report("Iter {} best HPWL: {} max_disp_x {} max_disp_y {} T {}", current_iter, block->dbuToMicrons(best_hpwl), max_disp_x, max_disp_y, T);
      count = 0;
    }

    float new_T = T*alpha;
    if (new_T <= 1) {
      log_->info(EPL, 5, "temperature is equal or less than 1 max_disp_x {} max_disp_y {}", max_disp_x, max_disp_y); 
      break;
    }
    T = new_T;
    float log_new_T = log(new_T);
    if (log_T < 1e-4) {
      log_->warn(EPL, 6, "log_T < 1e-4 | Iter {} best HPWL: {} max_disp_x {} max_disp_y {} T {}", current_iter, block->dbuToMicrons(best_hpwl), max_disp_x, max_disp_y, T);
      break;
    }
    float adjust_disp = log_new_T/log_T;
    max_disp_x = std::max(max_disp_x*adjust_disp, 1.0);
    max_disp_y = std::max(max_disp_y*adjust_disp, 1.0);
    log_T = log_new_T;
  }
  odb::WireLengthEvaluator eval(block);

  log_->report("Final HPWL: {} {}", block->dbuToMicrons(pbc_->hpwl()), block->dbuToMicrons(eval.hpwl()));
#pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n_inst; i++) {
    insts[i]->dbSetPlaced();
  }
}

void EPlace::simulatedAnnealingSimple(int threads, int wait_iterations, double initial_T, double alpha)
{
  debugPrint(log_,
             EPL,
             "simulatedAnnealing",
             2,
             "simulatedAnnealing: number of threads {}",
             threads);

  // Make a random placement
  randomPlace(threads);
  log_->report("initial HPWL: {}", pbc_->hpwl());

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreRect = block->getCoreArea();
  auto insts = pbc_->placeInsts();
  int n_inst = insts.size();
  std::default_random_engine generator(42);
  std::uniform_int_distribution<int> distrib(0);

  double max_disp_x = coreRect.dx();
  double max_disp_y = coreRect.dy();
  int64_t best_hpwl = pbc_->hpwl();
  int64_t last_hpwl = best_hpwl;
  int64_t current_hpwl = 0;
  int last_change = 0;
  int current_iter = 0;
  double T = initial_T;
  double log_T = log(T);

  int count = 0;
  while (current_iter++-last_change <= wait_iterations) {
    // generate new placement
    auto inst = insts[distrib(generator) % n_inst];

    // remove from hpwl only the nets that will change
    current_hpwl = last_hpwl;
    #pragma omp parallel for num_threads(threads)
    for (auto &pin : inst->pins()) {
      current_hpwl -= pin->net()->hpwl();
    }

    int m_disp_x = max_disp_x;
    int m_disp_y = max_disp_y;
    int orig_x = inst->lx(), orig_y = inst->ly();
    int min_x = std::max(inst->lx()-m_disp_x, coreRect.xMin());
    int max_x = std::min(inst->lx()+m_disp_x, coreRect.xMax()-inst->dx());
    int min_y = std::max(inst->ly()-m_disp_y, coreRect.yMin());
    int max_y = std::min(inst->ly()+m_disp_y, coreRect.yMax()-inst->dy());

    int new_x = min_x + distrib(generator) % (max_x-min_x);
    int new_y = min_y + distrib(generator) % (max_y-min_y);
    inst->dbSetLocation(new_x, new_y);

    // add back the changed nets to the hpwl
    #pragma omp parallel for num_threads(threads)
    for (auto &pin : inst->pins()) {
      auto net = pin->net();
      net->updateBox(pbc_->skipIoMode());
      current_hpwl += net->hpwl();
    }
    // debug
    //if (current_hpwl != pbc_->hpwl()) {
    //  log_->warn(EPL, 3, "iter {} | current_hpwl {} != pbc_->hpwl() {}", current_iter, current_hpwl, pbc_->hpwl());
    //}

    if (best_hpwl > current_hpwl) {
      best_hpwl = current_hpwl;
      last_hpwl = current_hpwl;

      if (max_disp_x != 1 or max_disp_y != 1) {
        last_change = current_iter;
      }
    } else {
      inst->dbSetLocation(orig_x, orig_y);
      #pragma omp parallel for num_threads(threads)
      for (auto &pin : inst->pins()) {
        auto net = pin->net();
        net->updateBox(pbc_->skipIoMode());
      }
    }

    if (++count == 100) {
      log_->report("Iter {} best HPWL: {} max_disp_x {} max_disp_y {} T {}", current_iter, block->dbuToMicrons(best_hpwl), max_disp_x, max_disp_y, T);
      count = 0;
    }

    float new_T = T*alpha;
    if (new_T <= 1) {
      log_->info(EPL, 5, "temperature is equal or less than 1 max_disp_x {} max_disp_y {}", max_disp_x, max_disp_y); 
      break;
    }
    T = new_T;
    float log_new_T = log(new_T);
    if (log_T < 1e-4) {
      log_->warn(EPL, 6, "log_T < 1e-4 | Iter {} best HPWL: {} max_disp_x {} max_disp_y {} T {}", current_iter, block->dbuToMicrons(best_hpwl), max_disp_x, max_disp_y, T);
      break;
    }
    float adjust_disp = log_new_T/log_T;
    max_disp_x = std::max(max_disp_x*adjust_disp, 1.0);
    max_disp_y = std::max(max_disp_y*adjust_disp, 1.0);
    log_T = log_new_T;
  }
  odb::WireLengthEvaluator eval(block);

  log_->report("Final HPWL: {} {}", block->dbuToMicrons(pbc_->hpwl()), block->dbuToMicrons(eval.hpwl()));
#pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n_inst; i++) {
    insts[i]->dbSetPlaced();
  }
}

}  // namespace epl

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

void initBinsInstDensityArea(gpl::BinGrid& grid, const std::vector<gpl::Instance *>& cells);
void binsAddRemoveInst(gpl::BinGrid& grid, const gpl::Instance* inst, bool remove);

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
             7,
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
             "simulatedAnnealingSimple",
             2,
             "simulatedAnnealingSimple: number of threads {}",
             threads);

  // Make a random placement
  randomPlace(threads);
  log_->report("initial HPWL: {}", pbc_->hpwl());

  odb::dbBlock* block = db_->getChip()->getBlock();
  auto insts = pbc_->placeInsts();
  int n_inst = insts.size();
  std::default_random_engine generator(42);
  std::uniform_int_distribution<int> distrib(0);

  // init hpwl
  odb::Rect coreRect = block->getCoreArea();
  double max_disp_x = coreRect.dx();
  double max_disp_y = coreRect.dy();

  int64_t last_hpwl = pbc_->hpwl();
  int64_t current_hpwl = 0;

   // init other vars
  int last_change = 0;
  int current_iter = 0;
  double T = initial_T;
  double log_T = log(T);

  int64_t best_hpwl = last_hpwl;

  int print_count = 0;
  while (current_iter++-last_change <= wait_iterations) {
    // generate new placement
    auto inst = insts[distrib(generator) % n_inst];

    // remove from hpwl only the nets that will change
    current_hpwl = last_hpwl;
    #pragma omp parallel for num_threads(threads)
    for (auto &pin : inst->pins()) {
      current_hpwl -= pin->net()->hpwl();
    }

    // move the inst
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
    /* debug
    if (current_hpwl != pbc_->hpwl()) {
      log_->warn(EPL, 3, "iter {} | current_hpwl {} != pbc_->hpwl() {}", current_iter, current_hpwl, pbc_->hpwl());
    }
    */

    if (best_hpwl > current_hpwl) {
      // movement accepted
      best_hpwl = current_hpwl;
      last_hpwl = current_hpwl;

      if (max_disp_x != 1 or max_disp_y != 1) {
        last_change = current_iter;
      }
    } else {
      // movement rejected (put instace back)
      inst->dbSetLocation(orig_x, orig_y);
      #pragma omp parallel for num_threads(threads)
      for (auto &pin : inst->pins()) {
        auto net = pin->net();
        net->updateBox(pbc_->skipIoMode());
      }
    }

    if (++print_count == 100) {
      log_->report("Iter {} best HPWL: {} max_disp_x {} max_disp_y {} T {}", current_iter, block->dbuToMicrons(best_hpwl), max_disp_x, max_disp_y, T);
      print_count = 0;
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

void EPlace::simulatedAnnealingDensity(int threads, int wait_iterations, double initial_T, double alpha, double density, int print_period, float swap_chance)
{
  debugPrint(log_,
             EPL,
             "simulatedAnnealingDensity",
             2,
             "simulatedAnnealingDensity: number of threads {}",
             threads);

  // Make a random placement
  randomPlace(threads);
  log_->report("initial HPWL: {}", pbc_->hpwl());

  odb::dbBlock* block = db_->getChip()->getBlock();
  auto insts = pbc_->placeInsts();
  int n_inst = insts.size();
  std::default_random_engine generator(42);
  std::uniform_int_distribution<int> distrib(0);
  std::uniform_real_distribution<double> real(0, 1);

  // init density grid
  double max_density = std::min(1.0, density*2);
  double current_density = max_density;
  int max_divisions = 10;
  int n_divisions = max_divisions;

  gpl::BinGrid density_grid(&(pbVec_[0]->die()));
  density_grid.setLogger(log_);
  density_grid.setPlacerBase(pbVec_[0]);
  density_grid.setTargetDensity(current_density);
  density_grid.setNumThreads(threads);
  density_grid.initBins();
  initBinsInstDensityArea(density_grid, insts);
  
  int64_t last_overflow = density_grid.overflowAreaUnscaled();
  int64_t last_cost_overflow = density_grid.overflowArea();
  int64_t current_overflow = 0;
  int64_t current_cost_overflow = 0;

  // init hpwl
  odb::Rect coreRect = block->getCoreArea();
  double initial_max_disp_x = coreRect.dx()/2;
  double max_disp_x = initial_max_disp_x;
  double max_disp_y = coreRect.dy()/2;

  int64_t last_hpwl = pbc_->hpwl();
  int64_t current_hpwl = 0;

  // init other vars
  int last_change = 0;
  int current_iter = 0;
  double T = initial_T;
  double log_T = log(T);

  int64_t best_hpwl = last_hpwl;
  int64_t best_cost_overflow = last_cost_overflow;
  int64_t best_overflow = last_overflow;
  int64_t best_cost = best_hpwl+last_overflow;
  int64_t last_cost = best_cost;
  int64_t current_cost = 0;

  int print_count = 0;
  int swap_count = 0;
  while (current_iter++-last_change <= wait_iterations) {
    // generate new placement
    
    gpl::Instance *inst = insts[distrib(generator) % n_inst];
    gpl::Instance *inst2 = nullptr;
    if (real(generator) < swap_chance) {
      inst2 = insts[distrib(generator) % n_inst];
      if (inst2 == inst) {inst2 = nullptr;}
    }

    // remove from hpwl only the nets that will change
    current_hpwl = last_hpwl;
    #pragma omp parallel for num_threads(threads)
    for (auto &pin : inst->pins()) {
      current_hpwl -= pin->net()->hpwl();
    }

    // remove instance from desity grid 
    binsAddRemoveInst(density_grid, inst, true);

    int orig_x = inst->lx(), orig_y = inst->ly();
    int orig2_x = 0, orig2_y = 0;
    if (inst2 != nullptr) { // swap
      // snap inside the core
      int new_pos_x = inst2->cx()-inst->dx()/2;
      int new_pos_y = inst2->cy()-inst->dy()/2;
      int new_pos2_x = inst->cx()-inst2->dx()/2;
      int new_pos2_y = inst->cy()-inst2->dy()/2;
      if (abs(new_pos_x-new_pos2_x) < max_disp_x and abs(new_pos_y-new_pos2_y) < max_disp_y) {
        swap_count++;
        // remove instance from desity grid 
        binsAddRemoveInst(density_grid, inst2, true);

        // save positions
        orig2_x = inst2->lx();
        orig2_y = inst2->ly();

        if (new_pos_x+inst->dx() > coreRect.xMax()) {
          new_pos_x = coreRect.xMax() - inst->dx();
        }
        if (new_pos_y+inst->dy() > coreRect.yMax()) {
          new_pos_x = coreRect.yMax() - inst->dy();
        }
        if (new_pos_x < coreRect.xMin()) {
          new_pos_x = coreRect.xMin();
        }
        if (new_pos_y < coreRect.yMin()) {
          new_pos_y = coreRect.yMin();
        }

        if (new_pos2_x+inst2->dx() > coreRect.xMax()) {
          new_pos2_x = coreRect.xMax() - inst2->dx();
        }
        if (new_pos2_y+inst->dy() > coreRect.yMax()) {
          new_pos2_x = coreRect.yMax() - inst2->dy();
        }
        if (new_pos2_x < coreRect.xMin()) {
          new_pos2_x = coreRect.xMin();
        }
        if (new_pos2_y < coreRect.yMin()) {
          new_pos2_y = coreRect.yMin();
        }
        

        inst->dbSetLocation(new_pos_x, new_pos_y);
        inst2->dbSetLocation(new_pos2_x, new_pos2_y);

        // add back the changed nets to the hpwl
        #pragma omp parallel for num_threads(threads)
        for (auto &pin : inst2->pins()) {
          auto net = pin->net();
          net->updateBox(pbc_->skipIoMode());
          current_hpwl += net->hpwl();
        }

        // put back the instance to the desity grid 
        binsAddRemoveInst(density_grid, inst2, false);
      } else {
        inst2 = nullptr;
      }
    }

    if (inst2 == nullptr) { // move the inst
      int m_disp_x = max_disp_x;
      int m_disp_y = max_disp_y;
      int min_x = std::max(inst->lx()-m_disp_x, coreRect.xMin());
      int max_x = std::min(inst->lx()+m_disp_x, coreRect.xMax()-inst->dx());
      int min_y = std::max(inst->ly()-m_disp_y, coreRect.yMin());
      int max_y = std::min(inst->ly()+m_disp_y, coreRect.yMax()-inst->dy());

      int new_x = min_x + distrib(generator) % (max_x-min_x);
      int new_y = min_y + distrib(generator) % (max_y-min_y);
      inst->dbSetLocation(new_x, new_y);
    }

    // add back the changed nets to the hpwl
    #pragma omp parallel for num_threads(threads)
    for (auto &pin : inst->pins()) {
      auto net = pin->net();
      net->updateBox(pbc_->skipIoMode());
      current_hpwl += net->hpwl();
    }

    // put back the instance to the desity grid 
    binsAddRemoveInst(density_grid, inst, false);
    current_overflow = density_grid.overflowAreaUnscaled();
    current_cost_overflow = density_grid.overflowArea();
    
    // update costs
    current_cost = current_hpwl+current_overflow;
    double diff = current_cost-last_cost;
    if (last_cost > current_cost or exp(-(diff*diff)/T)> real(generator)) {
      // movement accepted
      best_cost = current_cost;

      best_hpwl = current_hpwl;
      best_cost_overflow = current_cost_overflow;
      best_overflow = current_overflow;

      if (max_disp_x != 1 or max_disp_y != 1) {
        last_change = current_iter;
      }
    } else {
      // movement rejected (put instace back)
      if (inst2 !=nullptr) {
        binsAddRemoveInst(density_grid, inst2, true);
        inst2->dbSetLocation(orig2_x, orig2_y);
        #pragma omp parallel for num_threads(threads)
        for (auto &pin : inst2->pins()) {
          auto net = pin->net();
          net->updateBox(pbc_->skipIoMode());
        }
        binsAddRemoveInst(density_grid, inst2, false);
      }
      
      binsAddRemoveInst(density_grid, inst, true);
      inst->dbSetLocation(orig_x, orig_y);
      #pragma omp parallel for num_threads(threads)
      for (auto &pin : inst->pins()) {
        auto net = pin->net();
        net->updateBox(pbc_->skipIoMode());
      }
      binsAddRemoveInst(density_grid, inst, false);
    }

    if (++print_count == print_period) {
      log_->report("Iter {} best_cost: {} max_disp_x {:.1f}\tmax_disp_y {:.1f}\tT {:.1f}\thpwl {}\tcost_overflow {}\toverflow {}", current_iter, best_cost, max_disp_x, max_disp_y, T, best_hpwl, current_overflow, block->dbuAreaToMicrons(best_overflow));
      print_count = 0;
    }
    
    float new_T = T*alpha;
    if (new_T <= 1) {
      log_->info(EPL, 7, "temperature is equal or less than 1 max_disp_x {} max_disp_y {}", max_disp_x, max_disp_y); 
      break;
    }
    T = new_T;

    float log_new_T = log(new_T);
    if (log_T < 1e-4) {
      log_->warn(EPL, 8, "log_T < 1e-4 | Iter {} best HPWL: {} max_disp_x {:.1f} max_disp_y {:.1f} T {:.1f}", current_iter, block->dbuToMicrons(best_hpwl), max_disp_x, max_disp_y, T);
      break;
    }
    float adjust_disp = log_new_T/log_T;
    max_disp_x = std::max(max_disp_x*adjust_disp, 1.0);
    max_disp_y = std::max(max_disp_y*adjust_disp, 1.0);
    log_T = log_new_T;

    if (max_disp_x < initial_max_disp_x*0.8 and n_divisions>0) {
      current_density = density + (max_density-density)*--n_divisions/max_divisions;
      log_->report("Update density to {} in iter {}", current_density, current_iter);
      density_grid.setTargetDensity(current_density);
      #pragma omp parallel for num_threads(threads)
      for (auto& bin: density_grid.bins()) {
        bin.setDensity(current_density);
      }
      initBinsInstDensityArea(density_grid, insts);
      initial_max_disp_x = initial_max_disp_x*0.8;
    }
    last_cost = current_cost;
  }
  
  odb::WireLengthEvaluator eval(block);
  log_->report("N of iterations: {} | N of swaps tried {} ({:.1f}%)", current_iter, swap_count, 100.0*swap_count/current_iter);
  log_->report("Final HPWL: {} {}", block->dbuToMicrons(pbc_->hpwl()), block->dbuToMicrons(eval.hpwl()));

#pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n_inst; i++) {
    insts[i]->dbSetPlaced();
  }
}

static double getOverlapDensityArea(const gpl::Bin& bin, const gpl::Instance * cell)
{
  const int rectLx = std::max(bin.lx(), cell->lx());
  const int rectLy = std::max(bin.ly(), cell->ly());
  const int rectUx = std::min(bin.ux(), cell->ux());
  const int rectUy = std::min(bin.uy(), cell->uy());

  if (rectLx >= rectUx || rectLy >= rectUy) {
    return 0;
  }
  return static_cast<double>(rectUx - rectLx)
         * static_cast<double>(rectUy - rectLy);
}

void initBinsInstDensityArea(gpl::BinGrid& grid, const std::vector<gpl::Instance *>& cells)
{
  std::vector<gpl::Bin>& bins_ = grid.bins();
  int binCntX_ = grid.binCntX();

  // clear the Bin-area info
  for (gpl::Bin& bin : bins_) {
    bin.setInstPlacedAreaUnscaled(0);
  }

  for (auto& cell : cells) {
    std::pair<int, int> pairX = grid.getMinMaxIdxX(cell);
    std::pair<int, int> pairY = grid.getMinMaxIdxY(cell);

    // The following function is critical runtime hotspot
    // for global placer.
    //
    if (cell->isInstance()) {
      // macro should have
      // scale-down with target-density
      if (cell->isMacro()) {
        for (int y = pairY.first; y < pairY.second; y++) {
          for (int x = pairX.first; x < pairX.second; x++) {
            gpl::Bin& bin = bins_[y * binCntX_ + x];

            const double scaledArea = double(getOverlapDensityArea(bin, cell))
                                     * bin.targetDensity();
            bin.addInstPlacedAreaUnscaled(scaledArea);
          }
        }
      }
      // normal cells
      else {
        for (int y = pairY.first; y < pairY.second; y++) {
          for (int x = pairX.first; x < pairX.second; x++) {
            gpl::Bin& bin = bins_[y * binCntX_ + x];
            const double scaledArea = getOverlapDensityArea(bin, cell);
            bin.addInstPlacedAreaUnscaled(scaledArea);
          }
        }
      }
    }
  } 
  int64_t sumOverflowAreaCost_ = 0;
  int64_t sumOverflowAreaUnscaled_ = 0;
  // update density and overflowArea
#pragma omp parallel for num_threads(grid.getNumThreads()) \
    reduction(+ : sumOverflowAreaCost_, sumOverflowAreaUnscaled_)
  for (auto it = bins_.begin(); it < bins_.end(); ++it) {
    gpl::Bin& bin = *it;  // old-style loop for old OpenMP
    
    int64_t binArea = bin.binArea();
    const double scaledBinArea
        = static_cast<double>(binArea * bin.targetDensity());
    const double overflowAreaUnscaled = std::max(
        0.0,
        static_cast<double>(bin.instPlacedAreaUnscaled())
            + static_cast<double>(bin.nonPlaceAreaUnscaled()) - scaledBinArea);

    int64_t cost = overflowAreaUnscaled*overflowAreaUnscaled;
    bin.setInstPlacedArea(overflowAreaUnscaled); // overflow
    bin.setFillerArea(cost); // cost

    sumOverflowAreaUnscaled_ += overflowAreaUnscaled;
    sumOverflowAreaCost_ += cost;
    
  }
  grid.setoverflowArea(sumOverflowAreaCost_);
  grid.setoverflowAreaUnscaled(sumOverflowAreaUnscaled_);
}

void binsAddRemoveInst(gpl::BinGrid& grid, const gpl::Instance* inst, bool remove=false)
{
  std::vector<gpl::Bin>& bins_ = grid.bins();
  int binCntX_ = grid.binCntX();

  int64_t sumOverflowAreaCost_ = grid.overflowArea();
  int64_t sumOverflowAreaUnscaled_ = grid.overflowAreaUnscaled();

  std::pair<int, int> pairX = grid.getMinMaxIdxX(inst);
  std::pair<int, int> pairY = grid.getMinMaxIdxY(inst);

  // add/remove the area of the instance from the bins
  if (inst->isInstance()) {
    for (int y = pairY.first; y < pairY.second; y++) {
      for (int x = pairX.first; x < pairX.second; x++) {
        gpl::Bin& bin = bins_[y * binCntX_ + x];

        // remove this bin old overflow and cost
        sumOverflowAreaUnscaled_ -= bin.instPlacedArea(); // overflow
        sumOverflowAreaCost_ -= bin.fillerArea(); // cost

        double scaledArea = getOverlapDensityArea(bin, inst);
        if (inst->isMacro()) {
          // macro should have
          // scale-down with target-density
          scaledArea *= bin.targetDensity();
        }
        if (remove) {
          bin.addInstPlacedAreaUnscaled(-int(scaledArea));
        } else {
          bin.addInstPlacedAreaUnscaled(scaledArea);
        }

        int64_t binArea = bin.binArea();
        const double scaledBinArea
            = static_cast<double>(binArea * bin.targetDensity());
        const double overflowAreaUnscaled = std::max(
            0.0,
            static_cast<double>(bin.instPlacedAreaUnscaled())
                + static_cast<double>(bin.nonPlaceAreaUnscaled()) - scaledBinArea);

        int64_t cost = overflowAreaUnscaled*overflowAreaUnscaled;
        bin.setInstPlacedArea(overflowAreaUnscaled); // overflow
        bin.setFillerArea(cost); // cost

        // adds back this bin overflow and cost
        sumOverflowAreaUnscaled_ += overflowAreaUnscaled;// overflow
        sumOverflowAreaCost_ += cost; // cost
      }
    }
  } else {
    assert(false);
  }

  grid.setoverflowArea(sumOverflowAreaCost_);
  grid.setoverflowAreaUnscaled(sumOverflowAreaUnscaled_);
}


}  // namespace epl

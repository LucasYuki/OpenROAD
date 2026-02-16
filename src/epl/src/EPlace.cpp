#include "epl/EPlace.h"

#include <random>

#include "EDensity.h"
#include "gpl/placerBase.h"
#include "sta/StaMain.hh"

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

bool EPlace::initEPlace(float density, bool uniform_density)
{
  if (wa_wirelength_) {
    log_->warn(EPL, 3, "EPlacer already initialized.");
    return true;
  }

  if (!initPlacer()) {
    return false;
  }

  // Init wa_wirelength_
  WAwirelengthVars waVars;
  int threads = 1;
  wa_wirelength_ = std::make_shared<WAwirelength>(
      waVars, pbc_, log_, threads, gpl::Clusters());

  // Init e_density
  EDensityVars edVars;
  edVars.target_density = density;
  edVars.uniform_density = uniform_density;
  for (auto pb : pbVec_) {
    e_density_vec_.push_back(std::make_shared<EDensity>(
        edVars, pb, wa_wirelength_, log_));
  }
  return true;
}

bool EPlace::initPlacer()
{
  if (pbc_) {
    log_->warn(EPL, 2, "Placer already initialized.");
    return true;
  }
  // Init PlacerBaseCommon
  gpl::PlacerBaseVars pbVars;
  // pbVars.padLeft = padLeft_;
  // pbVars.padRight = padRight_;
  // pbVars.skipIoMode = skipIoMode_;
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
  wa_wirelength_.reset();
  e_density_vec_.clear();
  pbc_.reset();
  pbVec_.clear();
}

void EPlace::place(int threads, float density, bool uniform_density)
{
  debugPrint(log_,
             EPL,
             "place",
             1,
             "place: number of threads {}",
             threads);
  if (!initEPlace(density, uniform_density)) {
    return;
  }

  bool debug = true;
  /*
  std::unique_ptr<gpl::Graphics> graphics = nullptr;
  if (debug && gpl::Graphics::guiActive()) {
    graphics = std::make_unique<gpl::Graphics>(log_, pbc_, pbVec_);
  }

  if (debug && gpl::Graphics::guiActive()) {
    graphics->cellPlot(true);
  }
  */
}

void EPlace::randomPlace(int threads)
{
  debugPrint(log_,
             EPL,
             "random_place",
             1,
             "random_place: number of threads {}",
             threads);
  if (!initPlacer()) {
    return;
  }

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreRect = block->getCoreArea();

  auto insts = pbc_->placeInsts();
  int n_inst = insts.size();
  std::default_random_engine generator(42);
  std::uniform_int_distribution<int> distrib_x(coreRect.xMin(),
                                               coreRect.xMax());
  std::uniform_int_distribution<int> distrib_y(coreRect.yMin(),
                                               coreRect.yMax());
  std::vector<int> pos_x(n_inst), pos_y(n_inst);
  std::generate(
      pos_x.begin(), pos_x.end(), [&]() { return distrib_x(generator); });
  std::generate(
      pos_y.begin(), pos_y.end(), [&]() { return distrib_y(generator); });

#pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n_inst; i++) {
    auto inst = insts[i];
    inst->dbSetLocation(pos_x[i], pos_y[i]);
    inst->dbSetPlaced();
  }
}

}  // namespace epl

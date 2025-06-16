#include "epl/EPlace.h"

#include "EDensity.h"
#include "gpl/placerBase.h"
#include "sta/StaMain.hh"

#include <random>

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

bool EPlace::init_placer()
{  
  // Init PlacerBaseCommon
  gpl::PlacerBaseVars pbVars;
  //pbVars.padLeft = padLeft_;
  //pbVars.padRight = padRight_;
  //pbVars.skipIoMode = skipIoMode_;
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

  // Init e_density
  for (auto pb : pbVec_) {
    e_density_vec_.push_back(std::make_shared<EDensity>(db_, log_, pb));
  }
  return true;
}

void EPlace::clear()
{
  // clear instances
  pbc_.reset();
  pbVec_.clear();
  e_density_vec_.clear();
}

void EPlace::random_place(int threads)
{
  if (!init_placer()) {
    return;
  }

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreRect = block->getCoreArea();

  std::default_random_engine generator(42);
  std::uniform_int_distribution<int> distrib_x(coreRect.xMin(), coreRect.xMax());
  std::uniform_int_distribution<int> distrib_y(coreRect.yMin(), coreRect.yMax());
//#pragma omp parallel for num_threads(threads)
  for (auto inst: pbc_->placeInsts()) {
    if (!inst->isPlaceInstance()) {
      continue;
    }

    int x = distrib_x(generator);
    int y = distrib_y(generator);
    inst->dbSetLocation(x, y);
  }
}

}  // namespace epl

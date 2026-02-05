#include "EDensity.h"

#include <cmath>

namespace epl {

EDensity::EDensity(EDensityVars edVars,
                   std::shared_ptr<gpl::PlacerBase> pb,
                   std::shared_ptr<WAwirelength> wa_wirelength,
                   utl::Logger* log)
    : edVars_(edVars),
      pb_(std::move(pb)),
      wa_wirelength_(std::move(wa_wirelength)),
      log_(log)
{
  init();
}

void EDensity::clear()
{
  return;
}

void EDensity::init()
{
  clear();

  // Set seed
  srand(42);
  int dbu_per_micron = pb_->db()->getChip()->getBlock()->getDbUnitsPerMicron();

  // update gFillerCells
  initFillers();

  /*
  nb_gcells_.reserve(pb_->getInsts().size() + fillerStor_.size());

  // add place instances
  for (auto& pb_inst : pb_->placeInsts()) {
    int x_offset = rand() % (2 * dbu_per_micron) - dbu_per_micron;
    int y_offset = rand() % (2 * dbu_per_micron) - dbu_per_micron;

    GCell* gCell = nbc_->pbToNb(pb_inst);
    if (pb_inst != gCell->insts()[0]) {
      // Only process the first cluster once
      continue;
    }

    for (Instance* inst : gCell->insts()) {
      inst->setLocation(pb_inst->lx() + x_offset, pb_inst->ly() + y_offset);
    }
    gCell->updateLocations();
    nb_gcells_.emplace_back(nbc_.get(), nbc_->getGCellIndex(gCell));
    size_t gcells_index = nb_gcells_.size() - 1;
    db_inst_to_nb_index_[pb_inst->dbInst()] = gcells_index;
  }

  // add filler cells to gCells_
  for (size_t i = 0; i < fillerStor_.size(); ++i) {
    nb_gcells_.emplace_back(this, i);
    filler_stor_index_to_nb_index_[i] = nb_gcells_.size() - 1;
  }

  log_->debug(utl::EPL,
              "Init",
              "{:27} {:10}",
              "FillerInit:NumGCells:",
              nb_gcells_.size());
  debugPrint(log_,
             GPL,
             "FillerInit",
             1,
             format_label_int,
             "FillerInit:NumGNets:",
             nbc_->getGNets().size());
  debugPrint(log_,
             GPL,
             "FillerInit",
             1,
             format_label_int,
             "FillerInit:NumGPins:",
             nbc_->getGPins().size());

  // initialize bin grid structure
  // send param into binGrid structure
  if (nbVars_.isSetBinCnt) {
    bg_.setBinCnt(nbVars_.binCntX, nbVars_.binCntY);
  }

  bg_.setPlacerBase(pb_);
  bg_.setLogger(log_);
  bg_.setCorePoints(&(pb_->getDie()));
  bg_.setTargetDensity(targetDensity_);

  // update binGrid info
  bg_.initBins();

  // initialize fft structrue based on bins
  std::unique_ptr<FFT> fft(new FFT(bg_.getBinCntX(),
                                   bg_.getBinCntY(),
                                   bg_.getBinSizeX(),
                                   bg_.getBinSizeY()));

  fft_ = std::move(fft);

  // update densitySize and densityScale in each gCell
  updateDensitySize();
  */
}

void EDensity::initFillers()
{
  const int64_t place_area = pb_->placeInstsArea();
  const int64_t fixed_area = pb_->nonPlaceInstsArea();
  const int64_t core_area = pb_->getDie().coreArea();

  if (edVars_.uniform_density) {
    edVars_.target_density = double(place_area+fixed_area) / core_area;
  }
}

}  // namespace epl

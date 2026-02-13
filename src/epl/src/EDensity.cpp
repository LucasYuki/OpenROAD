#include "EDensity.h"

#include <algorithm>
#include <cmath>
#include <random>

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
  grid_.reset();
  filler_area_ = 0;
  fillers_.clear();
  return;
}

void EDensity::init()
{
  clear();

  // Set seed
  srand(42);
  //int dbu_per_micron = pb_->db()->getChip()->getBlock()->getDbUnitsPerMicron();

  initFillers();

  initGrid();
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
  // Calculate filler area
  const int64_t place_area = pb_->placeInstsArea();
  const int64_t fixed_area = pb_->nonPlaceInstsArea();
  const int64_t core_area = pb_->getDie().coreArea();

  double curr_density
      = static_cast<double>(place_area + fixed_area) / core_area;
  if (edVars_.uniform_density) {
    log_->info(utl::EPL, 4, "Using uniform density ({})", curr_density);
    target_density_ = curr_density;
  }
  if (curr_density > edVars_.target_density) {
    log_->warn(utl::EPL,
               5,
               "Target density ({}) is lower than the minimum possible. Target "
               "density set to {}",
               target_density_,
               curr_density);
    target_density_ = curr_density;
  }

  filler_area_ = (target_density_ - curr_density) * core_area;
  if (filler_area_ == 0) {
    return;
  }

  // Calculate filler size
  // Filler size is "the average size of the mid 80% of the movable cells"
  auto insts = pb_->placeInsts();
  int n_insts = insts.size();
  int size_x[n_insts], size_y[n_insts];
  int curr_idx = 0;
  for (auto& inst : insts) {
    size_x[curr_idx] = inst->dx();
    size_y[curr_idx] = inst->dy();
    curr_idx++;
  }
  std::sort(size_x, size_x + n_insts);
  std::sort(size_y, size_y + n_insts);

  int64_t filler_size_x = 0, filler_size_y = 0;
  int start_idx = n_insts * 0.1, end_idx = n_insts * 0.9;
  if (start_idx == end_idx) {
    start_idx = 0;
    end_idx = n_insts;
  }
  for (curr_idx = start_idx; curr_idx < end_idx; curr_idx++) {
    filler_size_x += size_x[curr_idx];
    filler_size_y += size_y[curr_idx];
  }
  filler_size_x = std::round(filler_size_x / (end_idx - start_idx));
  filler_size_y = std::round(filler_size_y / (end_idx - start_idx));

  // Create fillers
  int n_fillers = filler_area_ / (filler_size_x * filler_size_y);
  std::mt19937 randVal(0);
  std::uniform_int_distribution<> distr_x(
      pb_->getDie().coreLx(), pb_->getDie().coreUx() - filler_size_x);
  std::uniform_int_distribution<> distr_y(
      pb_->getDie().coreLy(), pb_->getDie().coreUy() - filler_size_y);
  for (int i = 0; i < n_fillers; i++) {
    int pos_x = distr_x(randVal), pos_y = distr_y(randVal);
    fillers_.push_back(gpl::Instance(
        pos_x, pos_y, pos_x + filler_size_x, pos_y + filler_size_y));
  }
  log_->info(utl::EPL,
             6,
             "{} fillers inserted of size ({}, {})",
             n_fillers,
             pb_->db()->getChip()->getBlock()->dbuToMicrons(filler_size_x),
             pb_->db()->getChip()->getBlock()->dbuToMicrons(filler_size_y));

  // Re-update the target density
  filler_area_ = n_fillers * (filler_size_x * filler_size_y);
  target_density_ = (place_area + fixed_area + filler_area_) / core_area;
  log_->info(utl::EPL,
             7,
             "Filler area: {}. Trget density adjusted: {}",
             filler_area_,
             target_density_);
}

void EDensity::initGrid()
{
  // Number of bins in the grid should be approximately the number of instances
  double ratio = pb_->getDie().coreDx() / pb_->getDie().coreDy();
  int n_movable_intances = pb_->placeInsts().size() + fillers_.size();
  double y = std::sqrt(n_movable_intances / ratio);
  double x = y * ratio;

  int binCntX = std::ceil(x), binCntY = std::ceil(y);
  float binSizeX = pb_->getDie().coreDx() / static_cast<float>(binCntX);
  float binSizeY = pb_->getDie().coreDy() / static_cast<float>(binCntY);

  grid_ = std::make_unique<Grid>(binCntX, binCntY, binSizeX, binSizeY);
}
}  // namespace epl

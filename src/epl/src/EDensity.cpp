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
  target_density_ = edVars_.target_density;

  initFillers();
  initGrid();

  updateDensity();
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
    log_->info(utl::EPL, 4, "Using uniform density", curr_density);
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
  log_->info(utl::EPL,
             12,
             "{:27} {:10.3f}",
             "Original target density:",
             target_density_);

  filler_area_ = (target_density_ - curr_density) * core_area;
  if (filler_area_ == 0) {
    return;
  }
  log_->info(utl::EPL,
             6,
             "{:27} {:10.3f}",
             "Target filler area:",
             pb_->db()->getChip()->getBlock()->dbuAreaToMicrons(filler_area_));

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
             7,
             "{} fillers inserted of size ({}, {}) um",
             n_fillers,
             pb_->db()->getChip()->getBlock()->dbuToMicrons(filler_size_x),
             pb_->db()->getChip()->getBlock()->dbuToMicrons(filler_size_y));

  // Re-update the target density
  filler_area_ = n_fillers * (filler_size_x * filler_size_y);
  target_density_ = (place_area + fixed_area + filler_area_)
                    / static_cast<double>(core_area);
  log_->info(utl::EPL,
             8,
             "{:27} {:10.3f} um^2",
             "Filler area:",
             pb_->db()->getChip()->getBlock()->dbuAreaToMicrons(filler_area_));
  log_->info(utl::EPL,
             9,
             "{:27} {:10.3f}",
             "Target density adjusted:",
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

  grid_ = std::make_unique<Grid>(
      log_, binCntX, binCntY, binSizeX, binSizeY, &pb_->getDie());

  for (auto inst : pb_->nonPlaceInsts()) {
    grid_->addFixedInst(inst);
  }
  grid_->clearMovable();
}

void EDensity::updateDensity()
{
  grid_->clearMovable();
  for (auto inst : pb_->placeInsts()) {
    grid_->addMovableInst(inst);
  }
  for (auto inst : fillers_) {
    grid_->addMovableInst(&inst);
  }
}

}  // namespace epl

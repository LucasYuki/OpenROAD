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
  place_instances_.clear();
  return;
}

void EDensity::init()
{
  clear();
  target_density_ = edVars_.target_density;

  initFillers();
  initGrid();

  std::mt19937 randVal(0);
  std::uniform_int_distribution<> distr_x(
      pb_->getRegionBBox().xCenter()-500, pb_->getRegionBBox().xCenter()+500);
  std::uniform_int_distribution<> distr_y(
      pb_->getRegionBBox().yCenter()-500, pb_->getRegionBBox().yCenter()+500);
  for (auto& inst : pb_->placeInsts()) {
    place_instances_.push_back(inst);
    int pos_x = distr_x(randVal), pos_y = distr_y(randVal);
    inst->setCenterLocation(pos_x, pos_y);
  }
  for (auto& inst : fillers_) {
    place_instances_.push_back(&inst);
  }

  updateDensity();
}

void EDensity::initFillers()
{
  // Calculate filler area
  const int64_t place_area = pb_->placeInstsArea();
  const int64_t fixed_area = pb_->nonPlaceInstsArea();
  const int64_t region_area = pb_->getRegionArea();

  double curr_density
      = static_cast<double>(place_area + fixed_area) / region_area;
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

  filler_area_ = (target_density_ - curr_density) * region_area;
  log_->info(utl::EPL,
             6,
             "{:27} {:10.3f}",
             "Target filler area:",
             pb_->db()->getChip()->getBlock()->dbuAreaToMicrons(filler_area_));

  // Calculate filler size
  // Filler size is "the average size of the mid 80% of the movable cells"
  auto insts = pb_->placeInsts();
  int n_insts = insts.size();
  std::vector<int> size_x, size_y;
  int curr_idx = 0;
  for (auto& inst : insts) {
    if (inst->isMacro()) {
      continue;
    }
    size_x.push_back(inst->dx());
    size_y.push_back(inst->dy());
  }
  n_insts = size_x.size();
  std::sort(size_x.begin(), size_x.end());
  std::sort(size_y.begin(), size_y.end());

  int start_idx = n_insts * 0.1, end_idx = n_insts * 0.9;
  if (start_idx == end_idx) {
    start_idx = 0;
    end_idx = n_insts;
  }
  for (curr_idx = start_idx; curr_idx < end_idx; curr_idx++) {
    filler_size_x_ += size_x[curr_idx];
    filler_size_y_ += size_y[curr_idx];
  }
  filler_size_x_ = std::round(filler_size_x_ / (end_idx - start_idx));
  filler_size_y_ = std::round(filler_size_y_ / (end_idx - start_idx));

  if (filler_area_ == 0) {
    return;
  }

  // Create fillers
  int n_fillers = filler_area_ / (filler_size_x_ * filler_size_y_);
  std::mt19937 randVal(0);
  std::uniform_int_distribution<> distr_x(
      pb_->getRegionBBox().xMin(), pb_->getRegionBBox().xMax() - filler_size_x_);
  std::uniform_int_distribution<> distr_y(
      pb_->getRegionBBox().yMin(), pb_->getRegionBBox().xMax() - filler_size_y_);
  for (int i = 0; i < n_fillers; i++) {
    int pos_x = distr_x(randVal), pos_y = distr_y(randVal);
    fillers_.push_back(gpl::Instance(
        pos_x, pos_y, pos_x + filler_size_x_, pos_y + filler_size_y_, false));
  }
  log_->info(utl::EPL,
             7,
             "{} fillers inserted of size ({}, {}) um",
             n_fillers,
             pb_->db()->getChip()->getBlock()->dbuToMicrons(filler_size_x_),
             pb_->db()->getChip()->getBlock()->dbuToMicrons(filler_size_y_));

  // Re-update the target density
  filler_area_ = n_fillers * (filler_size_x_ * filler_size_y_);
  target_density_ = (place_area + fixed_area + filler_area_)
                    / static_cast<double>(region_area);
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
  double ratio = pb_->getRegionBBox().dx() / pb_->getRegionBBox().dy();
  int n_movable_intances = pb_->placeInsts().size() + fillers_.size();
  double y = std::sqrt(n_movable_intances / ratio);
  double x = y * ratio;

  // Change this approximation to a better one later
  int binCntX = std::pow(2, std::floor(std::log2(x)));
  int binCntY = std::pow(2, std::floor(std::log2(y)));
  float binSizeX = pb_->getRegionBBox().dx() / static_cast<float>(binCntX);
  float binSizeY = pb_->getRegionBBox().dy() / static_cast<float>(binCntY);

  grid_ = std::make_unique<Grid>(
      log_, binCntX, binCntY, binSizeX, binSizeY, pb_->getRegionBBox());

  for (auto inst : pb_->nonPlaceInsts()) {
    grid_->addFixedInst(inst);
  }
  grid_->setTargetDensity(target_density_);
}

void EDensity::updateDensity()
{
  grid_->clearMovable();
  for (auto inst : place_instances_) {
    grid_->addMovableInst(inst);
  }
}

void EDensity::updateForce()
{
  grid_->doFFT();
}

void EDensity::printInfo()
{
  pb_->printInfo(false);
  log_->info(utl::EPL, 13, "Number of fillers: {}", fillers_.size());

  for (auto inst : place_instances_) {
    debugPrint(log_,
               utl::EPL,
               "initEPlace",
               1,
               "Instance : ({} {}) {}",
               inst->cx(),
               inst->cy(),
               inst->isDummy() ? "Dummy" : inst->dbInst()->getDebugName());
  }
}
}  // namespace epl

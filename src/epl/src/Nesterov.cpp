#include "Nesterov.h"

#include "utl/Logger.h"
namespace epl {

NesterovOptimizer::NesterovOptimizer(
    const std::shared_ptr<WAwirelength>& wa_wirelength,
    const std::vector<std::shared_ptr<EDensity>>& e_density_vec,
    utl::Logger* log)
    : wa_wirelength_(std::move(wa_wirelength)), log_(log)
{
  for (auto ed : e_density_vec) {
    e_density_vec_.push_back(std::move(ed));
  }
  init();
}

void NesterovOptimizer::init()
{
  for (auto ed : e_density_vec_) {
    auto gpl_insts = ed->placeInsts();
    std::vector<NesterovInst> instances;
    instances.reserve(gpl_insts.size());
    for (auto inst : gpl_insts) {
      instances.push_back(NesterovInst(inst));
    }
    inst_ed_vec_.push_back(instances);
  }
}

std::pair<float, float> NesterovOptimizer::snapPosition(
    const odb::Rect& region,
    float x,
    float y,
    gpl::Instance* inst) const
{
  float xlo = std::max(x - inst->dx() / 2., static_cast<double>(region.xMin()));
  float xhi = xlo + inst->dx();
  if (xhi > region.xMax()) {
    xlo = region.xMax() - inst->dx();
    xhi = region.xMax();
  }

  float ylo = std::max(y - inst->dy() / 2., static_cast<double>(region.yMin()));
  float yhi = ylo + inst->dy();
  if (yhi > region.yMax()) {
    ylo = region.yMax() - inst->dy();
    yhi = region.yMax();
  }
  return std::make_pair((xlo + xhi) / 2, (ylo + yhi) / 2);
}

float NesterovOptimizer::stepLength(float pos_diff)
{
  float grad_diff = 0;
  for (auto& ed_insts : inst_ed_vec_) {
    for (auto& inst : ed_insts) {
      auto [x, y] = inst.getPos();
      auto [x_old, y_old] = inst.getOldPos();
      pos_diff += std::abs(x - x_old) + std::abs(y - y_old);
      auto [gx, gy] = inst.getGradient();
      auto [gx_old, gy_old] = inst.getOldGradient();
      grad_diff += std::abs(gx - gx_old) + std::abs(gy - gy_old);
    }
  }
  std::cout << "pos_diff: " << pos_diff << " grad_diff: " << grad_diff
            << std::endl;
  return pos_diff / grad_diff;
}

int NesterovOptimizer::step(int curr_iter)
{
  curr_a_ = (1 + std::sqrt(1 + lst_a_ * lst_a_)) / 2;

  float epsilon = 0.95;
  curr_step_length_ = stepLength();
  if (lst_iter_ == -1) {
    // initialize
    lst_step_length_ = 0;  // shouldn't backtrack in the first pass
    lst_a_ = 1;
    curr_step_length_ = stepLength(500);
  }
  bool backtrack = lst_step_length_ > (epsilon * curr_step_length_);
  std::cout << lst_step_length_ << " " << curr_step_length_ << " " << backtrack
            << std::endl;

  // Update the location
  int idx = 0;
  for (auto& ed_insts : inst_ed_vec_) {
    auto bbox = e_density_vec_[idx++]->getRegionBBox();
    for (auto& inst : ed_insts) {
      if (!backtrack) {
        inst.commitValues();
      }
      auto [x, y] = inst.getPos();
      auto [fx, fy] = inst.getGradient();
      auto [x_ref_new_, y_ref_new_] = snapPosition(bbox,
                                                   x - curr_step_length_ * fx,
                                                   y - curr_step_length_ * fy,
                                                   inst.gplInst());
      inst.setRef(x_ref_new_, y_ref_new_);
      auto [x_ref_old, y_ref_old] = inst.getOldRef();
      auto [x_new_, y_new_] = snapPosition(
          bbox,
          x_ref_new_ + (lst_a_ - 1) * (x_ref_new_ - x_ref_old) / curr_a_,
          y_ref_new_ + (lst_a_ - 1) * (y_ref_new_ - y_ref_old) / curr_a_,
          inst.gplInst());
      inst.setPos(x_new_, y_new_);
    }
  }

  if (backtrack) {
    return curr_iter;
  }
  lst_step_length_ = curr_step_length_;
  lst_a_ = curr_a_;
  lst_iter_ = curr_iter;
  return curr_iter + 1;
}

}  // namespace epl
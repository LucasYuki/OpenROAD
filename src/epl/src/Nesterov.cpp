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

bool NesterovOptimizer::step()
{
  // update the force on each instance
  int idx = 0;
  for (auto& ed : e_density_vec_) {
    for (auto& inst : inst_ed_vec_[idx++]) {
      // Density Force
      auto [force_e_x, force_e_y] = ed->getElectroForce(inst.gplInst());
      /*
      debugPrint(log_,
                 utl::EPL,
                 "initEPlace",
                 1,
                 "force ({:g} {:g})",
                 force_e_x,
                 force_e_y);
      debugPrint(log_,
                 utl::EPL,
                 "initEPlace",
                 1,
                 "Instance : ({} {}) {}",
                 inst.gplInst()->cx(),
                 inst.gplInst()->cy(),
                 inst.gplInst()->isDummy()
                     ? "Dummy"
                     : inst.gplInst()->dbInst()->getDebugName());
      */

      // WA force
      float force_wa_x = 0, force_wa_y = 0;
      for (auto* pin : inst.gplInst()->getPins()) {
        auto [tmp_x, tmp_y] = wa_wirelength_->getGradient(pin);
        force_wa_x -= tmp_x;
        force_wa_y -= tmp_y;
      }

      // Compute the total force
      float force_x = force_wa_x; //force_e_x;
      float force_y = force_wa_y; //force_e_y;
      //std::cout << force_x << " " << force_y << std::endl;
      inst.setForce(force_x, force_y);
    }
  }

  // Update the location
  idx = 0;
  for (auto& ed_insts : inst_ed_vec_) {
    auto bbox = e_density_vec_[idx++]->getRegionBBox();
    for (auto& inst : ed_insts) {
      auto [x, y] = inst.getPos();
      auto [fx, fy] = inst.getForce();
      float step_size = 10;
      auto [new_x, new_y] = snapPosition(
          bbox, x + step_size * fx, y + step_size * fy, inst.gplInst());
      inst.setPos(new_x, new_y);
    }
  }
  return true;
}

}  // namespace epl
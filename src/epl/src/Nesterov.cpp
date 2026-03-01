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
  debugPrint(log_, utl::EPL, "initEPlace", 1, "NesterovOptimizer::Init");
  for (auto ed : e_density_vec_) {
    auto gpl_insts = ed->placeInsts();
    debugPrint(log_, utl::EPL, "initEPlace", 1, "A?");
    std::vector<NesterovInst> instances;
    debugPrint(log_, utl::EPL, "initEPlace", 1, "B?");
    instances.reserve(gpl_insts.size());
    for (auto inst : gpl_insts) {
      instances.push_back(NesterovInst(inst));
    }
    debugPrint(log_, utl::EPL, "initEPlace", 1, "C?");
    inst_ed_vec_.push_back(instances);
    debugPrint(log_, utl::EPL, "initEPlace", 1, "AAAAAAAAA");
  }
  debugPrint(log_, utl::EPL, "initEPlace", 1, "Init NesterovOptimizer end");
}

bool NesterovOptimizer::step()
{
  // eDensity force calc
  for (auto ed : e_density_vec_) {
    ed->updateDensity();
  }

  // update the force on each instance
  int idx = 0;
  for (auto ed : e_density_vec_) {
    for (auto& inst : inst_ed_vec_[idx++]) {
      // Density Force
      auto [force_e_x, force_e_y] = ed->getElectroForce(inst.gplInst());

      // WA force
      // auto [force_wa_x, force_wa_y] =

      // Compute the total force
      float force_x = force_e_x;
      float force_y = force_e_y;
      inst.setForce(force_x, force_y);
    }
  }

  // Update the location
  for (auto ed_insts : inst_ed_vec_) {
    for (auto& inst : ed_insts) {
      auto [x, y] = inst.getPos();
      auto [dx, dy] = inst.getForce();
      inst.setPos(x + dx, y + dy);
    }
  }
  return true;
}

}  // namespace epl
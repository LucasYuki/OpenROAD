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
      // auto [force_wa_x, force_wa_y] =

      // Compute the total force
      float force_x = force_e_x;
      float force_y = force_e_y;
      inst.setForce(force_x, force_y);
    }
  }

  // Update the location
  int a = 0;
  for (auto& ed_insts : inst_ed_vec_) {
    for (auto& inst : ed_insts) {
      auto [x, y] = inst.getPos();
      auto [dx, dy] = inst.getForce();
      float density_penalty = 1;
      inst.setPos(x + dx * density_penalty, y + dy * density_penalty);
      if (a++ == 0) {
        std::cout << x << " + " << dx << ", " << y << " + " << dy << " : "
                  << std::log10(std::abs(dx + dy)) << std::endl;
        auto [l, m] = inst.getPos();
        std::cout << l << ", " << m << std::endl;
      }
    }
  }
  return true;
}

}  // namespace epl
#include "EDensity.h"

#include <cmath>

namespace epl {

EDensity::EDensity(gpl::NesterovBaseVars nbVars,
                   std::shared_ptr<gpl::PlacerBase> pb,
                   std::shared_ptr<WAwirelength> nbc,
                   utl::Logger* log)
    : gpl::NesterovBase(
          nbVars,
          pb,
          std::dynamic_pointer_cast<gpl::NesterovBaseCommon>(nbc),
          log),
      wa_wirelength_(std::move(nbc))
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

  //odb::dbBlock* block = pb_->db()->getChip()->getBlock();
  //odb::Rect coreRect = block->getCoreArea();

  int n_intances = pb_->insts().size();
  int m = std::sqrt(n_intances);
  n_intances = m;
}

}  // namespace epl

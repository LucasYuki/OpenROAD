#include "EDensity.h"

#include <cmath>

#include "GridDensity.h"

namespace epl {

EDensity::EDensity(odb::dbDatabase* db, utl::Logger* logger, std::shared_ptr<gpl::PlacerBase> pb)
    : db_(db), log_(logger), pb_(pb)
{
  init();
}

void EDensity::clear()
{
  grid_density_.clear();
}

void EDensity::init()
{
  clear();

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreRect = block->getCoreArea();

  int n_intances = pb_->insts().size();
  int m = std::sqrt(n_intances);
  grid_density_.init(coreRect, m, m);
}

}  // namespace epl

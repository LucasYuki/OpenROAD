#include "EDensity.h"

#include <cmath>

#include "GridDensity.h"

namespace epl {

EDensity::EDensity(odb::dbDatabase* db, utl::Logger* logger)
    : db_(db), log_(logger)
{
  init();
}

void EDensity::init()
{
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreRect = block->getCoreArea();

  int n_intances = 10000;
  // iterate over all instances
  // get the number of fixes instances
  // get the number of movable instances
  // get the number of macros
  // generate dark nodes
  // generate fillers

  int m = std::sqrt(n_intances);
  grid_density_.init(coreRect, m, m);
}

}  // namespace epl

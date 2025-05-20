#include "GridDensity.h"

#include <vector>

#include "odb/db.h"

namespace epl {

void GridDensity::init(odb::Rect coreRect, int n_rows, int n_columns)
{
  coreRect_ = coreRect;
  n_rows_ = n_rows;
  n_columns_ = n_columns;
}

}  // namespace epl
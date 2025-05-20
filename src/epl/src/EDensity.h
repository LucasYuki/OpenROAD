#pragma once

#include "GridDensity.h"
#include "odb/db.h"

namespace epl {
class GridDensity;

class EDensity
{
 public:
  EDensity(odb::dbDatabase* db, utl::Logger* logger);

  ~EDensity() = default;

  void init();

 private:
  odb::dbDatabase* db_;
  utl::Logger* log_;
  GridDensity grid_density_;
};
}  // namespace epl
#pragma once

#include "GridDensity.h"
#include "odb/db.h"

namespace epl {
template <typename TArea, typename TDensity>
class GridDensity;

template <typename TArea, typename TDensity>
class EDensity
{
 public:
  EDensity(odb::dbDatabase* db) : db_(db) {}

  ~EDensity() = default;

  void init();

 private:
  odb::dbDatabase* db_;
  GridDensity<TArea, TDensity>* grid_density_;
};
}  // namespace epl
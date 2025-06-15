#pragma once

#include "GridDensity.h"
#include "odb/db.h"
#include "gpl/placerBase.h"

namespace gpl {
class PlacerBase;
class PlacerBaseCommon;
class PlacerBaseVars;
class Instance;
}

namespace epl {

class EDensity
{
 public:
  EDensity(odb::dbDatabase* db, utl::Logger* logger, std::shared_ptr<gpl::PlacerBase> pb);
  ~EDensity() = default;

  void clear();
  void init();

 private:
  odb::dbDatabase* db_;
  utl::Logger* log_;
  std::shared_ptr<gpl::PlacerBase> pb_;
  GridDensity grid_density_;
};
}  // namespace epl
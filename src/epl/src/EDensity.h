#pragma once

#include "WAwirelength.h"
#include "gpl/placerBase.h"
#include "odb/db.h"

namespace gpl {
class PlacerBase;
class PlacerBaseCommon;
class PlacerBaseVars;
class Instance;
class BinGrid;
}  // namespace gpl

namespace epl {
class WAwirelength;

class EDensityVars
{
 public:
  double target_density;
  bool uniform_density = false;
};

class EDensity
{
 public:
  EDensity(EDensityVars edVars,
           std::shared_ptr<gpl::PlacerBase> pb,
           std::shared_ptr<WAwirelength> wa_wirelength,
           utl::Logger* log);
  ~EDensity() = default;

  void clear();
  void init();

 private:
  void initFillers();

 private:
  EDensityVars edVars_;
  std::shared_ptr<gpl::PlacerBase> pb_;
  std::shared_ptr<WAwirelength> wa_wirelength_;
  utl::Logger* log_ = nullptr;

  gpl::BinGrid bg_;
};
}  // namespace epl
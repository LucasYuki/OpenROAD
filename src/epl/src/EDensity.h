#pragma once

#include <vector>

#include "Grid.h"
#include "WAwirelength.h"
#include "gpl/placerBase.h"
#include "odb/db.h"

namespace gpl {
class PlacerBase;
class PlacerBaseCommon;
class PlacerBaseVars;
class Instance;
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
  void initGrid();

 private:
  EDensityVars edVars_;
  std::shared_ptr<gpl::PlacerBase> pb_;
  std::shared_ptr<WAwirelength> wa_wirelength_;
  utl::Logger* log_ = nullptr;

  std::unique_ptr<Grid> grid_;

  double target_density_;
  int64_t filler_area_;
  std::vector<gpl::Instance> fillers_;
};
}  // namespace epl
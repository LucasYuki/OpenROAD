#pragma once

#include <vector>

#include "Grid.h"
#include "WAwirelength.h"
#include "gpl/placerBase.h"
#include "odb/db.h"

namespace gpl {
class PlacerBase;
class Instance;
}  // namespace gpl

namespace epl {
class WAwirelength;

class EDensityVars
{
 public:
  double target_density = 0;
  bool uniform_density = false;
  bool debug = true;
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
  void updateDensity();
  std::pair<float, float> getElectroForce(gpl::Instance* inst) const
  {
    return grid_->getElectroForce(inst);
  };

  std::vector<gpl::Instance*> placeInsts() { return place_instances_; };

 private:
  void init();
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
  std::vector<gpl::Instance*> place_instances_;
};
}  // namespace epl
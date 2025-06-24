#pragma once

#include "gpl/nesterovBase.h"
#include "gpl/placerBase.h"
#include "odb/db.h"
#include "WAwirelength.h"

namespace gpl {
class PlacerBase;
class PlacerBaseCommon;
class PlacerBaseVars;
class Instance;
class NesterovBase;
}  // namespace gpl

namespace epl {
class WAwirelength;

class EDensity : gpl::NesterovBase
{
 public:
  EDensity(gpl::NesterovBaseVars nbVars,
           std::shared_ptr<gpl::PlacerBase> pb,
           std::shared_ptr<WAwirelength> nbc,
           utl::Logger* log);
  ~EDensity() = default;

  void clear();
  void init();

 private:
  std::shared_ptr<WAwirelength> wa_wirelength_;
};
}  // namespace epl
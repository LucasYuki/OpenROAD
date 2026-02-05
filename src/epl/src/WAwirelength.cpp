#include "WAwirelength.h"

namespace epl {

WAwirelength::WAwirelength(WAwirelengthVars waVars,
                           std::shared_ptr<gpl::PlacerBaseCommon> pb,
                           utl::Logger* log,
                           int num_threads,
                           const gpl::Clusters& clusters)
    : waVars_(waVars),
      pb_(std::move(pb)),
      log_(log),
      num_threads_(num_threads_),
      clusters_(clusters_)
{
}

}  // namespace epl
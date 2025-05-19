#pragma once

#include <Eigen/Core>

#include "odb/db.h"

namespace epl {

template <typename TArea = u_int64_t, typename TDensity = double>
class GridDensity
{
 public:
  GridDensity(odb::dbDatabase* db) : db_(db) {}

  ~GridDensity() = default;

  void init();

 private:
  odb::dbDatabase* db_;
  Eigen::Matrix<TArea, Eigen::Dynamic, Eigen::Dynamic> fixed_area_, bin_area_;
  Eigen::Matrix<TDensity, Eigen::Dynamic, Eigen::Dynamic> density_;
};
}  // namespace epl
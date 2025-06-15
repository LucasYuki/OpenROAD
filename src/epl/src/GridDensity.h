#pragma once

#include <Eigen/Core>

#include "odb/db.h"

namespace epl {

class GridDensity
{
 public:
  GridDensity() = default;

  void init(odb::Rect coreRect, int n_rows, int n_columns);
  void clear();

 private:
  odb::Rect coreRect_;
  int n_rows_;
  int n_columns_;

  Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> fixed_area_, bin_area_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> density_;
};
}  // namespace epl
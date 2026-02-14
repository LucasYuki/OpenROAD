// Copyright (c) 2021, The Regents of the University of California
// All rights reserved.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <unordered_map>

#include "gpl/Replace.h"
#include "gpl/placerBase.h"
#include "odb/db.h"

namespace gpl {
class PlacerBaseCommon;
}  // namespace gpl

namespace epl {
class WAwirelengthVars
{
 public:
  double target_density;
  bool uniform_density = false;
};
class WAwirelength
{
 public:
  WAwirelength(WAwirelengthVars waVars,
               std::shared_ptr<gpl::PlacerBaseCommon> pb,
               utl::Logger* log,
               int num_threads,
               const gpl::Clusters& clusters);

  /*
    void clear();
    void update(int threads);
  */
  WAwirelengthVars waVars_;
  std::shared_ptr<gpl::PlacerBaseCommon> pb_;
  utl::Logger* log_;
  int num_threads_;
  const gpl::Clusters& clusters_;
};

}  // namespace epl

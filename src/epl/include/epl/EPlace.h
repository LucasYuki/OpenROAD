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

#include <vector>

#include "odb/db.h"
#include "gpl/Replace.h"

namespace epl {

class EPlace
{
 public:
  EPlace();
  ~EPlace();
  
  void clear();
  void init(odb::dbDatabase* db, utl::Logger* logger);
  bool initPlacer();
  bool initEPlace();
  void randomPlace(int threads);
  void simulatedAnnealingSimple(int threads, int wait_iterations, double initial_T, double alpha);

 private:

  odb::dbDatabase* db_;
  utl::Logger* log_;

  std::shared_ptr<gpl::PlacerBaseCommon> pbc_;
  std::vector<std::shared_ptr<gpl::PlacerBase>> pbVec_;
};

}  // namespace epl

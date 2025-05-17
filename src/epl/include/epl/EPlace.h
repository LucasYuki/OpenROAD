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

#include <tcl.h>
#include <vector>

#include "odb/db.h"
#include "epl/Cell.h"
#include "epl/FlipFlop.h"

namespace epl {
class FlipFlop;
class Cell;

class EPlace
{
public:
  EPlace();
  ~EPlace() = default;
  void init(Tcl_Interp *tcl_interp,
	    odb::dbDatabase *db);
  void orderFlops();
  void clusterFlops();

private:
  odb::dbDatabase *db_;
  std::vector<FlipFlop> flipflops_;
  std::vector<Cell> cells_;
};

}

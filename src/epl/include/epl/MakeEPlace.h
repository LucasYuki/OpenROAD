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

#ifndef MAKE_EPLACE_H
#define MAKE_EPLACE_H

namespace odb {
class dbDatabase;
}
namespace utl {
class Logger;
}

namespace epl {
class EPlace;

class OpenRoad;

epl::EPlace* makeEPlace();

void deleteEPlace(epl::EPlace* eplace);

void initEPlace(epl::EPlace* eplace,
                odb::dbDatabase* db,
                utl::Logger* logger,
                Tcl_Interp* tcl_interp);

}  // namespace epl
#endif

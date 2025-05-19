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

#include "epl/MakeEPlace.h"

#include "epl/EPlace.h"
#include "odb/db.h"
#include "utl/decode.h"

extern "C" {
extern int Epl_Init(Tcl_Interp* interp);
}

namespace epl {

// Tcl files encoded into strings.
extern const char* epl_tcl_inits[];

epl::EPlace* makeEPlace()
{
  return new epl::EPlace;
}

void deleteEPlace(epl::EPlace* eplace)
{
  delete eplace;
}

void initEPlace(epl::EPlace* eplace,
                odb::dbDatabase* db,
                utl::Logger* logger,
                Tcl_Interp* tcl_interp)
{
  Epl_Init(tcl_interp);
  utl::evalTclInit(tcl_interp, epl::epl_tcl_inits);
  eplace->init(db, logger);
}

}  // namespace epl

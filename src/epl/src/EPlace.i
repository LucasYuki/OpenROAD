// Copyright (c) 2021, The Regents of the Federal University of Santa Catarina
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

%{
#include "ord/OpenRoad.hh"
#include "epl/EPlace.h"
#include "odb/db.h"

namespace ord {
OpenRoad*
getOpenRoad();

epl::EPlace*
getEPlace();
}

using ord::getOpenRoad;
using ord::getEPlace;
using epl::EPlace;

%}

%inline %{

void
eplace_random_placement_cmd()
{
  EPlace* eplace = getEPlace();
  int threads = ord::OpenRoad::openRoad()->getThreadCount();
  eplace->randomPlace(threads);
}

void
eplace_simulated_annealing_simple_cmd(int wait_iterations, double initial_T, double alpha)
{
  EPlace* eplace = getEPlace();
  int threads = ord::OpenRoad::openRoad()->getThreadCount();
  eplace->simulatedAnnealingSimple(threads, wait_iterations, initial_T, alpha);
}

void
eplace_simulated_annealing_density_cmd(int wait_iterations, double initial_T, double alpha, double density)
{
  EPlace* eplace = getEPlace();
  int threads = ord::OpenRoad::openRoad()->getThreadCount();
  eplace->simulatedAnnealingDensity(threads, wait_iterations, initial_T, alpha, density);
}

%} // inline

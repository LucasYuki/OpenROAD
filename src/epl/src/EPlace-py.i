// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

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

%include "../../Exception-py.i"
%include "epl/EPlace.h"

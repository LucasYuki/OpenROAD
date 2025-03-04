// *****************************************************************************
// *****************************************************************************
// ATTENTION: THIS IS AN AUTO-GENERATED FILE. DO NOT CHANGE IT!
// *****************************************************************************
// *****************************************************************************
// Copyright 2012, Cadence Design Systems
//
// This  file  is  part  of  the  Cadence  LEF/DEF  Open   Source
// Distribution,  Product Version 5.8.
//
// Licensed under the Apache License, Version 2.0 (the \"License\");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an \"AS IS\" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
//    implied. See the License for the specific language governing
//    permissions and limitations under the License.
//
//
// For updates, support, or to become part of the LEF/DEF Community,
// check www.openeda.org for details.
//
//  $Author: xxx $
//  $Revision: xxx $
//  $Date: xxx $
//  $State: xxx $
// *****************************************************************************
// *****************************************************************************

#define EXTERN extern "C"

#include "defiSite.h"
#include "defiSite.hpp"

union udefiPoints
{
  DefParser::defiPoints cpp;
  ::defiPoints c;
};

// Wrappers definitions.
double defiSite_x_num(const ::defiSite* obj)
{
  return ((DefParser::defiSite*) obj)->x_num();
}

double defiSite_y_num(const ::defiSite* obj)
{
  return ((DefParser::defiSite*) obj)->y_num();
}

double defiSite_x_step(const ::defiSite* obj)
{
  return ((DefParser::defiSite*) obj)->x_step();
}

double defiSite_y_step(const ::defiSite* obj)
{
  return ((DefParser::defiSite*) obj)->y_step();
}

double defiSite_x_orig(const ::defiSite* obj)
{
  return ((DefParser::defiSite*) obj)->x_orig();
}

double defiSite_y_orig(const ::defiSite* obj)
{
  return ((DefParser::defiSite*) obj)->y_orig();
}

int defiSite_orient(const ::defiSite* obj)
{
  return ((DefParser::defiSite*) obj)->orient();
}

const char* defiSite_orientStr(const ::defiSite* obj)
{
  return ((const DefParser::defiSite*) obj)->orientStr();
}

const char* defiSite_name(const ::defiSite* obj)
{
  return ((const DefParser::defiSite*) obj)->name();
}

void defiSite_print(const ::defiSite* obj, FILE* f)
{
  ((DefParser::defiSite*) obj)->print(f);
}

int defiBox_xl(const ::defiBox* obj)
{
  return ((DefParser::defiBox*) obj)->xl();
}

int defiBox_yl(const ::defiBox* obj)
{
  return ((DefParser::defiBox*) obj)->yl();
}

int defiBox_xh(const ::defiBox* obj)
{
  return ((DefParser::defiBox*) obj)->xh();
}

int defiBox_yh(const ::defiBox* obj)
{
  return ((DefParser::defiBox*) obj)->yh();
}

::defiPoints defiBox_getPoint(const ::defiBox* obj)
{
  udefiPoints tmp;
  tmp.cpp = ((DefParser::defiBox*) obj)->getPoint();
  return tmp.c;
}

void defiBox_print(const ::defiBox* obj, FILE* f)
{
  ((DefParser::defiBox*) obj)->print(f);
}

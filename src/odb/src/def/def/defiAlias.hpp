// *****************************************************************************
// *****************************************************************************
// Copyright 2013 - 2014, Cadence Design Systems
//
// This  file  is  part  of  the  Cadence  LEF/DEF  Open   Source
// Distribution,  Product Version 5.8.
//
// Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
//    implied. See the License for the specific language governing
//    permissions and limitations under the License.
//
// For updates, support, or to become part of the LEF/DEF Community,
// check www.openeda.org for details.
//
//  $Author: dell $
//  $Revision: #1 $
//  $Date: 2020/09/29 $
//  $State:  $
// *****************************************************************************
// *****************************************************************************

#ifndef defiALIAS_h
#define defiALIAS_h

#include "defiKRDefs.hpp"

BEGIN_DEF_PARSER_NAMESPACE

class defAliasIterator;
class defrData;

class defiAlias_itr
{
 public:
  defiAlias_itr(defrData* defData = nullptr);
  void Init();

  void Destroy();
  ~defiAlias_itr();

  int Next();
  const char* Key();
  const char* Data();
  int Marked();

 protected:
  int first{1};
  defAliasIterator* iterator{nullptr};
  defrData* defData;
};

END_DEF_PARSER_NAMESPACE

#endif

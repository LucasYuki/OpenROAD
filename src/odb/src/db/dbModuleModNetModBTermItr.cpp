///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2019, Nefelus Inc
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Generator Code Begin Cpp
#include "dbModuleModNetModBTermItr.h"

#include "dbModBTerm.h"
#include "dbModNet.h"
#include "dbModule.h"
#include "dbTable.h"
#include "dbTable.hpp"

namespace odb {

////////////////////////////////////////////////////////////////////
//
// dbModuleModNetModBTermItr - Methods
//
////////////////////////////////////////////////////////////////////

bool dbModuleModNetModBTermItr::reversible()
{
  return true;
}

bool dbModuleModNetModBTermItr::orderReversed()
{
  return true;
}

void dbModuleModNetModBTermItr::reverse(dbObject* parent)
{
}

uint dbModuleModNetModBTermItr::sequential()
{
  return 0;
}

uint dbModuleModNetModBTermItr::size(dbObject* parent)
{
  uint id;
  uint cnt = 0;

  for (id = dbModuleModNetModBTermItr::begin(parent);
       id != dbModuleModNetModBTermItr::end(parent);
       id = dbModuleModNetModBTermItr::next(id)) {
    ++cnt;
  }

  return cnt;
}

uint dbModuleModNetModBTermItr::begin(dbObject* parent)
{
  // User Code Begin begin
  _dbModNet* mod_net = (_dbModNet*) parent;
  return mod_net->_modbterms;
  // User Code End begin
}

uint dbModuleModNetModBTermItr::end(dbObject* /* unused: parent */)
{
  return 0;
}

uint dbModuleModNetModBTermItr::next(uint id, ...)
{
  // User Code Begin next
  _dbModBTerm* _modbterm = _modbterm_tbl->getPtr(id);
  return _modbterm->_next_net_modbterm;
  // User Code End next
}

dbObject* dbModuleModNetModBTermItr::getObject(uint id, ...)
{
  return _modbterm_tbl->getPtr(id);
}
}  // namespace odb
   // Generator Code End Cpp

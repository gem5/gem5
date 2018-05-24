/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

    sc_concatref.cpp -- Concatenation support.

    Original Author: Andy Goodrich, Forte Design Systems, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_concatref.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:54:01  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include "systemc/ext/dt/misc/sc_concatref.hh"
#include "systemc/ext/dt/sc_temporary.hh"

// STORAGE POOLS USED BY sc_concatref:
namespace sc_core
{

template class sc_vpool<sc_dt::sc_concatref>;
template class sc_vpool<sc_dt::sc_concat_bool>;
sc_byte_heap sc_temp_heap(0x300000);

} // namespace sc_core

namespace sc_dt
{

sc_core::sc_vpool<sc_concat_bool> sc_concat_bool::m_pool(9);
sc_core::sc_vpool<sc_concatref> sc_concatref::m_pool(9);

} // namespace sc_dt

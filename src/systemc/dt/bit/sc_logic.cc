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

  sc_logic.cpp -- C++ implementation of logic type. Behaves
                  pretty much the same way as HDLs logic type.

  Original Author: Stan Y. Liao, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: sc_logic.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include <sstream>

#include "systemc/ext/dt/bit/messages.hh"
#include "systemc/ext/dt/bit/sc_logic.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_dt
{

// ----------------------------------------------------------------------------
//  CLASS : sc_logic
//
//  Four-valued logic type.
// ----------------------------------------------------------------------------

// support methods
void
sc_logic::invalid_value(sc_logic_value_t v)
{
    invalid_value((int)v);
}

void
sc_logic::invalid_value(char c)
{
    std::stringstream msg;
    msg << "sc_logic('" << c << "')";
    SC_REPORT_ERROR(sc_core::SC_ID_VALUE_NOT_VALID_, msg.str().c_str());
}

void
sc_logic::invalid_value(int i)
{
    std::stringstream msg;
    msg << "sc_logic(" << i << ")";
    SC_REPORT_ERROR(sc_core::SC_ID_VALUE_NOT_VALID_, msg.str().c_str());
}


void
sc_logic::invalid_01() const
{
    if ((int)m_val == Log_Z)
        SC_REPORT_WARNING(sc_core::SC_ID_LOGIC_Z_TO_BOOL_, 0);
    else
        SC_REPORT_WARNING(sc_core::SC_ID_LOGIC_X_TO_BOOL_, 0);
}


// conversion tables
const sc_logic_value_t sc_logic::char_to_logic[128] = {
    Log_0, Log_1, Log_Z, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_0, Log_1, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_Z, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X, Log_X,
    Log_X, Log_X, Log_Z, Log_X, Log_X, Log_X, Log_X, Log_X
};

const char sc_logic::logic_to_char[4] = { '0', '1', 'Z', 'X' };

const sc_logic_value_t sc_logic::and_table[4][4] = {
    { Log_0, Log_0, Log_0, Log_0 },
    { Log_0, Log_1, Log_X, Log_X },
    { Log_0, Log_X, Log_X, Log_X },
    { Log_0, Log_X, Log_X, Log_X }
};

const sc_logic_value_t sc_logic::or_table[4][4] = {
    { Log_0, Log_1, Log_X, Log_X },
    { Log_1, Log_1, Log_1, Log_1 },
    { Log_X, Log_1, Log_X, Log_X },
    { Log_X, Log_1, Log_X, Log_X }
};

const sc_logic_value_t sc_logic::xor_table[4][4] = {
    { Log_0, Log_1, Log_X, Log_X },
    { Log_1, Log_0, Log_X, Log_X },
    { Log_X, Log_X, Log_X, Log_X },
    { Log_X, Log_X, Log_X, Log_X }
};

const sc_logic_value_t sc_logic::not_table[4] = {
    Log_1, Log_0, Log_X, Log_X
};

// other methods
void
sc_logic::scan(::std::istream &is)
{
    char c;
    is >> c;
    *this = c;
}

// #ifdef SC_DT_DEPRECATED
const sc_logic sc_logic_0(Log_0);
const sc_logic sc_logic_1(Log_1);
const sc_logic sc_logic_Z(Log_Z);
const sc_logic sc_logic_X(Log_X);
// #endif

const sc_logic SC_LOGIC_0(Log_0);
const sc_logic SC_LOGIC_1(Log_1);
const sc_logic SC_LOGIC_Z(Log_Z);
const sc_logic SC_LOGIC_X(Log_X);

} // namespace sc_dt

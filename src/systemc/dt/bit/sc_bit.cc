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

  sc_bit.cpp -- Bit class.

  Original Author: Gene Bushuyev, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_bit.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.6  2006/04/12 20:17:52  acg
//  Andy Goodrich: enabled deprecation message for sc_bit.
//
// Revision 1.5  2006/01/25 00:31:15  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.4  2006/01/24 20:50:55  acg
// Andy Goodrich: added warnings indicating that sc_bit is deprecated and that
// the C bool data type should be used in its place.
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include <sstream>

#include "systemc/ext/dt/bit/messages.hh"
#include "systemc/ext/dt/bit/sc_bit.hh"
#include "systemc/ext/dt/bit/sc_logic.hh"
#include "systemc/ext/utils/messages.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_dt
{

// ----------------------------------------------------------------------------
//  CLASS : sc_bit
//
//  Bit class.
//  Note: VSIA compatibility indicated.
// ----------------------------------------------------------------------------

// support methods
void
sc_bit::invalid_value(char c)
{
    std::stringstream msg;
    msg << "sc_bit( '" << c << "' )";
    SC_REPORT_ERROR(sc_core::SC_ID_VALUE_NOT_VALID_, msg.str().c_str());
    sc_core::sc_abort(); // can't recover from here
}

void
sc_bit::invalid_value(int i)
{
    std::stringstream msg;
    msg << "sc_bit( " << i << " )";
    SC_REPORT_ERROR(sc_core::SC_ID_VALUE_NOT_VALID_, msg.str().c_str());
    sc_core::sc_abort(); // can't recover from here
}

// constructors
sc_bit::sc_bit(const sc_logic &a) : m_val(a.to_bool()) // non-VSIA
{
    sc_deprecated_sc_bit();
}

// assignment operators
sc_bit &
sc_bit::operator=(const sc_logic &b) // non-VSIA
{
    return (*this = sc_bit(b));
}

// other methods
void
sc_bit::scan(::std::istream &is)
{
    bool b;
    is >> b;
    *this = b;
}

void
sc_deprecated_sc_bit()
{
    static bool warn_sc_bit_deprecated = true;
    if (warn_sc_bit_deprecated) {
        warn_sc_bit_deprecated = false;
        SC_REPORT_INFO(sc_core::SC_ID_IEEE_1666_DEPRECATION_,
                       "sc_bit is deprecated, use bool instead");
    }
}

} // namespace sc_dt

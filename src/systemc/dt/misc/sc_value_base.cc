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

  sc_value_base.cpp -- Base class for all SystemC data values.

  Original Author: Andy Goodrich, Forte Design Systems

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_value_base.cpp,v $
// Revision 1.2  2011/08/15 16:43:24  acg
//  Torsten Maehne: changes to remove unused argument warnings.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:54:01  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include <cctype>
#include <cstdio>
#include <cstdlib>

#include "systemc/ext/dt/int/messages.hh"
#include "systemc/ext/dt/misc/sc_value_base.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_dt
{

void
sc_value_base::concat_clear_data(bool /* to_ones */)
{
    static const char error_message[] =
        "concat_clear_data method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
}

bool
sc_value_base::concat_get_ctrl(sc_digit * /*dst_p*/, int /*low_i*/) const
{
    static const char error_message[] =
        "concat_get_ctrl method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
    return false;
}

bool
sc_value_base::concat_get_data(sc_digit * /*dst_p*/, int /*low_i*/) const
{
    static const char error_message[] =
        "concat_get_data method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
    return false;
}

sc_dt::uint64
sc_value_base::concat_get_uint64() const
{
    static const char error_message[] =
        "concat_get_uint64 method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
    return 0;
}

int
sc_value_base::concat_length(bool * /*xz_present_p*/) const
{
    static const char error_message[] =
        "concat_length method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
    return 0;
}

void
sc_value_base::concat_set(int64 /*src*/, int /*low_i*/)
{
    static const char error_message[] =
        "concat_set(int64) method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
}

void
sc_value_base::concat_set(const sc_signed & /*src*/, int /*low_i*/)
{
    static const char error_message[] =
        "concat_set(sc_signed) method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
}

void
sc_value_base::concat_set(const sc_unsigned & /*src*/, int /*low_i*/)
{
    static const char error_message[] =
        "concat_set(sc_unsigned) method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
}

void
sc_value_base::concat_set(uint64 /*src*/, int /*low_i*/)
{
    static const char error_message[] =
        "concat_set(uint64) method not supported by this type";
    SC_REPORT_ERROR(sc_core::SC_ID_OPERATION_FAILED_, error_message);
}

} // namespace sc_dt

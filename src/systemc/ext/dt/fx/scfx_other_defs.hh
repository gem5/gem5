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

  scfx_other_defs.h -

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: scfx_other_defs.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_FX_SCFX_OTHER_DEFS_HH__
#define __SYSTEMC_EXT_DT_FX_SCFX_OTHER_DEFS_HH__

#include "../int/sc_int_base.hh"
#include "../int/sc_signed.hh"
#include "../int/sc_uint_base.hh"
#include "../int/sc_unsigned.hh"
#include "messages.hh"

namespace sc_dt
{

// ----------------------------------------------------------------------------
//  CLASS : sc_signed
// ----------------------------------------------------------------------------

// assignment operators
inline const sc_signed &
sc_signed::operator = (const sc_fxval &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_signed::operator = ( const sc_fxval& )");
        return *this;
    }
    for (int i = 0; i < length(); ++i)
        (*this)[i] = v.get_bit(i);

    return *this;
}

inline const sc_signed &
sc_signed::operator = (const sc_fxval_fast &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_signed::operator = ( const sc_fxval_fast& )");
        return *this;
    }

    for (int i = 0; i < length(); ++i)
        (*this)[i] = v.get_bit(i);

    return *this;
}

inline const sc_signed &
sc_signed::operator = (const sc_fxnum &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_signed::operator = ( const sc_fxnum& )");
        return *this;
    }

    for (int i = 0; i < length(); ++i)
        (*this)[i] = v.get_bit(i);

    return *this;
}

inline const sc_signed &
sc_signed::operator = (const sc_fxnum_fast &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_signed::operator = ( const sc_fxnum_fast& )");
        return *this;
    }

    for (int i = 0; i < length(); ++i)
        (*this)[i] = v.get_bit(i);

    return *this;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned
// ----------------------------------------------------------------------------

// assignment operators

inline const sc_unsigned &
sc_unsigned::operator = (const sc_fxval &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_unsigned::operator = ( const sc_fxval& )");
        return *this;
    }

    for (int i = 0; i < length(); ++i)
        (*this)[i] = v.get_bit(i);

    return *this;
}

inline const sc_unsigned &
sc_unsigned::operator = (const sc_fxval_fast &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_unsigned::operator = ( const sc_fxval_fast& )");
        return *this;
    }

    for (int i = 0; i < length(); ++i)
        (*this)[i] = v.get_bit(i);

    return *this;
}

inline const sc_unsigned &
sc_unsigned::operator = (const sc_fxnum &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_unsigned::operator = ( const sc_fxnum& )" );
        return *this;
    }

    for (int i = 0; i < length(); ++i)
        (*this)[i] = v.get_bit(i);

    return *this;
}

inline const sc_unsigned &
sc_unsigned::operator = (const sc_fxnum_fast &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_unsigned::operator = ( const sc_fxnum_fast& )" );
        return *this;
    }

    for (int i = 0; i < length(); ++i)
        (*this)[i] = v.get_bit(i);

    return *this;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_int_base
// ----------------------------------------------------------------------------

// assignment operators

inline sc_int_base &
sc_int_base::operator = (const sc_fxval &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_int_base::operator = ( const sc_fxval& )");
        return *this;
    }
    for (int i = 0; i < m_len; ++i) {
        set(i, v.get_bit(i));
    }
    extend_sign();
    return *this;
}

inline sc_int_base &
sc_int_base::operator = (const sc_fxval_fast &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_int_base::operator = ( const sc_fxval_fast& )");
        return *this;
    }
    for (int i = 0; i < m_len; ++i) {
        set(i, v.get_bit(i));
    }
    extend_sign();
    return *this;
}

inline sc_int_base &
sc_int_base::operator = (const sc_fxnum &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_int_base::operator = ( const sc_fxnum& )");
        return *this;
    }
    for (int i = 0; i < m_len; ++i) {
        set(i, v.get_bit(i));
    }
    extend_sign();
    return *this;
}

inline sc_int_base &
sc_int_base::operator = (const sc_fxnum_fast &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_int_base::operator = ( const sc_fxnum_fast& )");
        return *this;
    }
    for (int i = 0; i < m_len; ++i) {
        set (i, v.get_bit(i));
    }
    extend_sign();
    return *this;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_uint_base
// ----------------------------------------------------------------------------

// assignment operators
inline sc_uint_base &
sc_uint_base::operator = (const sc_fxval &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_uint_base::operator = ( const sc_fxval& )");
        return *this;
    }
    for (int i = 0; i < m_len; ++i) {
        set(i, v.get_bit(i));
    }
    extend_sign();
    return *this;
}

inline sc_uint_base &
sc_uint_base::operator = (const sc_fxval_fast &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_uint_base::operator = ( const sc_fxval_fast& )");
        return *this;
    }
    for (int i = 0; i < m_len; ++i) {
        set(i, v.get_bit(i));
    }
    extend_sign();
    return *this;
}

inline sc_uint_base &
sc_uint_base::operator = (const sc_fxnum &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_uint_base::operator = ( const sc_fxnum& )");
        return *this;
    }
    for (int i = 0; i < m_len; ++i) {
        set(i, v.get_bit(i));
    }
    extend_sign();
    return *this;
}

inline sc_uint_base &
sc_uint_base::operator = (const sc_fxnum_fast &v)
{
    if (!v.is_normal()) { /* also triggers OBSERVER_READ call */
        SC_REPORT_ERROR(sc_core::SC_ID_INVALID_FX_VALUE_,
                        "sc_uint_base::operator = ( const sc_fxnum_fast& )");
        return *this;
    }
    for (int i = 0; i < m_len; ++i) {
        set(i, v.get_bit(i));
    }
    extend_sign();
    return *this;
}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_FX_SCFX_OTHER_DEFS_HH__

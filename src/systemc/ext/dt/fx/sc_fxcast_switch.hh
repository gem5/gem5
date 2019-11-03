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

  sc_fxcast_switch.h -

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_fxcast_switch.h,v $
// Revision 1.2  2011/08/24 22:05:43  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:57  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_FX_SC_FXCAST_SWITCH_HH__
#define __SYSTEMC_EXT_DT_FX_SC_FXCAST_SWITCH_HH__

#include <iostream>

#include "sc_context.hh"
#include "sc_fxdefs.hh"

namespace sc_dt
{

// classes defined in this module
class sc_fxcast_switch;


// ----------------------------------------------------------------------------
//  CLASS : sc_fxcast_switch
//
//  Fixed-point cast switch class.
// ----------------------------------------------------------------------------

class sc_fxcast_switch
{
  public:
    sc_fxcast_switch();
    sc_fxcast_switch(sc_switch);
    sc_fxcast_switch(const sc_fxcast_switch &);
    explicit sc_fxcast_switch(sc_without_context);

    sc_fxcast_switch &operator = (const sc_fxcast_switch &);

    friend bool operator == (const sc_fxcast_switch &,
                             const sc_fxcast_switch &);
    friend bool operator != (const sc_fxcast_switch &,
                             const sc_fxcast_switch &);

    const std::string to_string() const;

    void print(::std::ostream & =::std::cout) const;
    void dump(::std::ostream & =::std::cout) const;

  private:
    sc_switch m_sw;
};

} // namespace sc_dt

// ----------------------------------------------------------------------------
//  TYPEDEF : sc_fxcast_context
//
//  Context type for the fixed-point cast switch parameter.
// ----------------------------------------------------------------------------

namespace sc_dt
{

extern template class sc_global<sc_fxcast_switch>;
extern template class sc_context<sc_fxcast_switch>;

typedef sc_context<sc_fxcast_switch> sc_fxcast_context;


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline sc_fxcast_switch::sc_fxcast_switch() : m_sw()
{
    *this = sc_fxcast_context::default_value();
}

inline sc_fxcast_switch::sc_fxcast_switch(sc_switch sw_) : m_sw( sw_ ) {}

inline sc_fxcast_switch::sc_fxcast_switch(const sc_fxcast_switch &a) :
        m_sw(a.m_sw)
{}

inline sc_fxcast_switch::sc_fxcast_switch(sc_without_context) :
        m_sw(SC_DEFAULT_CAST_SWITCH_)
{}

inline sc_fxcast_switch &
sc_fxcast_switch::operator = (const sc_fxcast_switch &a)
{
    if (&a != this) {
        m_sw = a.m_sw;
    }
    return *this;
}

inline bool
operator == (const sc_fxcast_switch &a, const sc_fxcast_switch &b)
{
    return (a.m_sw == b.m_sw);
}

inline bool
operator != (const sc_fxcast_switch &a, const sc_fxcast_switch &b)
{
    return (a.m_sw != b.m_sw);
}

inline ::std::ostream &
operator << (::std::ostream &os, const sc_fxcast_switch &a)
{
    a.print(os);
    return os;
}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_FX_SC_FXCAST_SWITCH_HH__

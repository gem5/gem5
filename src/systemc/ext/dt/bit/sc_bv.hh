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

  sc_bv.h -- Arbitrary size bit vector class.

  Original Author: Gene Bushuyev, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_bv.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_BIT_SC_BV_HH__
#define __SYSTEMC_EXT_DT_BIT_SC_BV_HH__

#include "sc_bv_base.hh"

namespace sc_dt
{

// classes defined in this module
template <int W>
class sc_bv;


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_bv<W>
//
//  Arbitrary size bit vector class.
// ----------------------------------------------------------------------------

template <int W>
class sc_bv : public sc_bv_base
{
  public:
    // constructors
    sc_bv() :sc_bv_base(W) {}

    explicit sc_bv(bool init_value) : sc_bv_base(init_value, W) {}

    explicit sc_bv(char init_value) : sc_bv_base((init_value != '0'), W) {}

    sc_bv(const char *a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(const bool *a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(const sc_logic *a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(const sc_unsigned &a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(const sc_signed &a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(const sc_uint_base &a) : sc_bv_base(W)
    {
        sc_bv_base::operator = (a);
    }
    sc_bv(const sc_int_base &a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(unsigned long a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(long a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(unsigned int a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(int a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(uint64 a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(int64 a) : sc_bv_base(W) { sc_bv_base::operator = (a); }

    template <class X>
    sc_bv(const sc_proxy<X> &a) : sc_bv_base(W) { sc_bv_base::operator = (a); }
    sc_bv(const sc_bv<W> &a) : sc_bv_base(a) {}

    // assignment operators
    template <class X>
    sc_bv<W> &
    operator = (const sc_proxy<X> &a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (const sc_bv<W> &a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (const char *a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (const bool *a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (const sc_logic *a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (const sc_unsigned &a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (const sc_signed &a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (const sc_uint_base &a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (const sc_int_base &a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (unsigned long a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (long a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (unsigned int a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (int a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (uint64 a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }

    sc_bv<W> &
    operator = (int64 a)
    {
        sc_bv_base::operator = (a);
        return *this;
    }
};

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_BIT_SC_BV_HH__

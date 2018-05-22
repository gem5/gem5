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

  sc_int.h -- A signed integer whose length is less than 64 bits.

              Unlike arbitrary precision, arithmetic and bitwise operations
              are performed using the native types (hence capped at 64 bits).
              The sc_int integer is useful when the user does not need
              arbitrary precision and the performance is superior to
              sc_bigint/sc_biguint.

  Original Author: Amit Rao, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Ali Dasdan, Synopsys, Inc.
  Description of Modification: - Resolved ambiguity with sc_(un)signed.
                               - Merged the code for 64- and 32-bit versions
                                 via the constants in sc_nbdefs.h.
                               - Eliminated redundant file inclusions.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_int.h,v $
// Revision 1.2  2011/02/18 20:19:14  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:49:31  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#ifndef __SYSTEMC_EXT_DT_INT_SC_INT_HH__
#define __SYSTEMC_EXT_DT_INT_SC_INT_HH__

#include "sc_int_base.hh"

namespace sc_dt
{

// classes defined in this module
template <int W>
class sc_int;


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_int<W>
//
//  Template class sc_int<W> is the interface that the user sees. It is
//  derived from sc_int_base and most of its methods are just wrappers
//  that call the corresponding method in the parent class. Note that
//  the length of sc_int datatype is specified as a template parameter.
// ----------------------------------------------------------------------------

template <int W>
class sc_int : public sc_int_base
{
  public:
    // constructors
    sc_int() : sc_int_base(W) {}
    sc_int(int_type v) : sc_int_base(v, W) {}
    sc_int(const sc_int<W> &a) : sc_int_base(a) {}

    sc_int(const sc_int_base &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(const sc_int_subref_r &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    template< class T >
    sc_int(const sc_generic_base<T> &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a->to_int64());
    }
    sc_int(const sc_signed &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(const sc_unsigned &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    explicit sc_int(const sc_fxval &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    explicit sc_int(const sc_fxval_fast &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    explicit sc_int(const sc_fxnum &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    explicit sc_int(const sc_fxnum_fast &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(const sc_bv_base &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(const sc_lv_base &a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(const char *a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(unsigned long a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(long a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(unsigned int a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(int a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(uint64 a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }
    sc_int(double a) : sc_int_base(W)
    {
        sc_int_base::operator = (a);
    }

    // assignment operators
    sc_int<W> &
    operator = (int_type v)
    {
        sc_int_base::operator = (v);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_int_base &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_int_subref_r &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_int<W> &a)
    {
        m_val = a.m_val;
        return *this;
    }
    template< class T >
    sc_int<W> &
    operator = (const sc_generic_base<T> &a)
    {
        sc_int_base::operator = (a->to_int64());
        return *this;
    }
    sc_int<W> &
    operator = (const sc_signed &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_unsigned &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_fxval &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_fxval_fast &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_fxnum &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &operator = (const sc_fxnum_fast &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_bv_base &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const sc_lv_base &a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (const char *a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (unsigned long a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (long a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (unsigned int a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (int a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (uint64 a)
    {
        sc_int_base::operator = (a);
        return *this;
    }
    sc_int<W> &
    operator = (double a)
    {
        sc_int_base::operator = (a);
        return *this;
    }

    // arithmetic assignment operators
    sc_int<W> &
    operator += (int_type v)
    {
        sc_int_base::operator += (v);
        return *this;
    }
    sc_int<W> &
    operator -= (int_type v)
    {
        sc_int_base::operator -= (v);
        return *this;
    }
    sc_int<W> &
    operator *= (int_type v)
    {
        sc_int_base::operator *= (v);
        return *this;
    }
    sc_int<W> &
    operator /= (int_type v)
    {
        sc_int_base::operator /= (v);
        return *this;
    }
    sc_int<W> &
    operator %= (int_type v)
    {
        sc_int_base::operator %= (v);
        return *this;
    }

    // bitwise assignment operators
    sc_int<W> &
    operator &= (int_type v)
    {
        sc_int_base::operator &= (v);
        return *this;
    }
    sc_int<W> &
    operator |= (int_type v)
    {
        sc_int_base::operator |= (v);
        return *this;
    }
    sc_int<W> &
    operator ^= (int_type v)
    {
        sc_int_base::operator ^= (v);
        return *this;
    }
    sc_int<W> &
    operator <<= (int_type v)
    {
        sc_int_base::operator <<= (v);
        return *this;
    }
    sc_int<W> &
    operator >>= (int_type v)
    {
        sc_int_base::operator >>= (v);
        return *this;
    }

    // prefix and postfix increment and decrement operators
    sc_int<W> &
    operator ++ () // prefix
    {
        sc_int_base::operator ++ ();
        return *this;
    }
    const sc_int<W>
    operator ++ (int) // postfix
    {
        return sc_int<W>(sc_int_base::operator ++ (0));
    }
    sc_int<W> &
    operator -- () // prefix
    {
        sc_int_base::operator -- ();
        return *this;
    }
    const sc_int<W>
    operator -- (int) // postfix
    {
        return sc_int<W>(sc_int_base::operator -- (0));
    }
};

} // namespace sc_dt


#endif // __SYSTEMC_EXT_DT_INT_SC_INT_HH__

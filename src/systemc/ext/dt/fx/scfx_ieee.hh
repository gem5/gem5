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

  scfx_ieee.h -

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: scfx_ieee.h,v $
// Revision 1.3  2011/08/24 22:05:43  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.2  2011/08/07 18:55:24  acg
//  Philipp A. Hartmann: added guard for __clang__ to get the clang platform
//  working.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_FX_SCFX_IEEE_HH__
#define __SYSTEMC_EXT_DT_FX_SCFX_IEEE_HH__

#include "../../utils/endian.hh"
#include "sc_fxdefs.hh"

namespace sc_dt
{

// classes defined in this module
union ieee_double;
class scfx_ieee_double;
union ieee_float;
class scfx_ieee_float;

#define SCFX_MASK_(Size) ((1u << (Size))-1u)

// ----------------------------------------------------------------------------
//  UNION : ieee_double
//
//  IEEE 754 double-precision format.
// ----------------------------------------------------------------------------

union ieee_double
{
    double d;

    struct
    {
#if defined(SC_BOOST_BIG_ENDIAN)
        unsigned negative:1;
        unsigned exponent:11;
        unsigned mantissa0:20;
        unsigned mantissa1:32;
#elif defined(SC_BOOST_LITTLE_ENDIAN)
        unsigned mantissa1:32;
        unsigned mantissa0:20;
        unsigned exponent:11;
        unsigned negative:1;
#endif
    } s;
};


const unsigned int SCFX_IEEE_DOUBLE_BIAS = 1023U;

const int SCFX_IEEE_DOUBLE_E_MAX = 1023;
const int SCFX_IEEE_DOUBLE_E_MIN = -1022;

const unsigned int SCFX_IEEE_DOUBLE_M_SIZE = 52;
const unsigned int SCFX_IEEE_DOUBLE_M0_SIZE = 20;
const unsigned int SCFX_IEEE_DOUBLE_M1_SIZE = 32;
const unsigned int SCFX_IEEE_DOUBLE_E_SIZE = 11;


// ----------------------------------------------------------------------------
//  CLASS : scfx_ieee_double
//
//  Convenient interface to union ieee_double.
// ----------------------------------------------------------------------------

class scfx_ieee_double
{
    ieee_double m_id;
  public:
    scfx_ieee_double();
    scfx_ieee_double(double);
    scfx_ieee_double(const scfx_ieee_double &);

    scfx_ieee_double &operator = (double);
    scfx_ieee_double &operator = (const scfx_ieee_double &);

    operator double() const;

    unsigned int negative() const;
    void negative(unsigned int);
    int exponent() const;
    void exponent(int);
    unsigned int mantissa0() const;
    void mantissa0(unsigned int);
    unsigned int mantissa1() const;
    void mantissa1(unsigned int);

    bool is_zero() const;
    bool is_subnormal() const;
    bool is_normal() const;
    bool is_inf() const;
    bool is_nan() const;

    void set_inf();
    void set_nan();

    int msb() const; // most significant non-zero bit
    int lsb() const; // least significant non-zero bit

    static const scfx_ieee_double nan();
    static const scfx_ieee_double inf(int);
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline scfx_ieee_double::scfx_ieee_double() : m_id()
{
    m_id.d = 0.0;
}

inline scfx_ieee_double::scfx_ieee_double(double d) : m_id()
{
    m_id.d = d;
}

inline scfx_ieee_double::scfx_ieee_double(const scfx_ieee_double &a) :
        m_id(a.m_id)
{
    // m_id.d = a.m_id.d;
}

inline scfx_ieee_double &
scfx_ieee_double::operator = (double d)
{
    m_id.d = d;
    return *this;
}

inline scfx_ieee_double &
scfx_ieee_double::operator = (const scfx_ieee_double &a)
{
    m_id.d = a.m_id.d;
    return *this;
}

inline scfx_ieee_double::operator double() const
{
    return m_id.d;
}

inline unsigned int
scfx_ieee_double::negative() const
{
    return m_id.s.negative;
}

inline void
scfx_ieee_double::negative(unsigned int a)
{
    m_id.s.negative = a & SCFX_MASK_(1);
}

inline int
scfx_ieee_double::exponent() const
{
    return m_id.s.exponent - SCFX_IEEE_DOUBLE_BIAS;
}

inline void
scfx_ieee_double::exponent(int a)
{
    m_id.s.exponent = (SCFX_IEEE_DOUBLE_BIAS + a)
                      & SCFX_MASK_(SCFX_IEEE_DOUBLE_E_SIZE);
}

inline unsigned int
scfx_ieee_double::mantissa0() const
{
    return m_id.s.mantissa0;
}

inline void
scfx_ieee_double::mantissa0(unsigned int a)
{
    m_id.s.mantissa0 = a & SCFX_MASK_(SCFX_IEEE_DOUBLE_M0_SIZE);
}

inline unsigned int
scfx_ieee_double::mantissa1() const
{
    return m_id.s.mantissa1;
}

inline void
scfx_ieee_double::mantissa1(unsigned int a)
{
    m_id.s.mantissa1 = a; // & SCFX_MASK_(SCFX_IEEE_DOUBLE_M1_SIZE);
}

inline bool
scfx_ieee_double::is_zero() const
{
    return (exponent() == SCFX_IEEE_DOUBLE_E_MIN - 1 &&
            mantissa0() == 0U && mantissa1() == 0U);
}

inline bool
scfx_ieee_double::is_subnormal() const
{
    return (exponent() == SCFX_IEEE_DOUBLE_E_MIN - 1 &&
            (mantissa0() != 0U || mantissa1() != 0U));
}

inline bool
scfx_ieee_double::is_normal() const
{
    return (exponent() >= SCFX_IEEE_DOUBLE_E_MIN &&
            exponent() <= SCFX_IEEE_DOUBLE_E_MAX);
}

inline bool
scfx_ieee_double::is_inf() const
{
    return (exponent() == SCFX_IEEE_DOUBLE_E_MAX + 1 &&
            mantissa0() == 0U && mantissa1() == 0U);
}

inline bool
scfx_ieee_double::is_nan() const
{
    return (exponent() == SCFX_IEEE_DOUBLE_E_MAX + 1 &&
             (mantissa0() != 0U || mantissa1() != 0U));
}

inline void
scfx_ieee_double::set_inf()
{
    exponent(SCFX_IEEE_DOUBLE_E_MAX + 1);
    mantissa0(0U);
    mantissa1(0U);
}

inline void
scfx_ieee_double::set_nan()
{
    exponent(SCFX_IEEE_DOUBLE_E_MAX + 1);
    mantissa0((unsigned int)-1);
    mantissa1((unsigned int)-1);
}

#define MSB_STATEMENT(x,n) if ( x >> n ) { x >>= n; i += n; }

inline int
scfx_ieee_double::msb() const
{
    unsigned int m0 = mantissa0();
    unsigned int m1 = mantissa1();
    if (m0 != 0) {
        int i = 0;
        MSB_STATEMENT(m0, 16);
        MSB_STATEMENT(m0, 8);
        MSB_STATEMENT(m0, 4);
        MSB_STATEMENT(m0, 2);
        MSB_STATEMENT(m0, 1);
        return (i - 20);
    } else if (m1 != 0) {
        int i = 0;
        MSB_STATEMENT(m1, 16);
        MSB_STATEMENT(m1, 8);
        MSB_STATEMENT(m1, 4);
        MSB_STATEMENT(m1, 2);
        MSB_STATEMENT(m1, 1);
        return (i - 52);
    } else {
        return 0;
    }
}

#undef MSB_STATEMENT

#define LSB_STATEMENT(x,n) if ( x << n ) { x <<= n; i -= n; }

inline int
scfx_ieee_double::lsb() const
{
    unsigned int m0 = mantissa0();
    unsigned int m1 = mantissa1();
    if (m1 != 0) {
        int i = 31;
        LSB_STATEMENT(m1, 16);
        LSB_STATEMENT(m1, 8);
        LSB_STATEMENT(m1, 4);
        LSB_STATEMENT(m1, 2);
        LSB_STATEMENT(m1, 1);
        return (i - 52);
    } else if (m0 != 0) {
        int i = 31;
        LSB_STATEMENT(m0, 16);
        LSB_STATEMENT(m0, 8);
        LSB_STATEMENT(m0, 4);
        LSB_STATEMENT(m0, 2);
        LSB_STATEMENT(m0, 1);
        return (i - 20);
    } else {
        return 0;
    }
}

#undef LSB_STATEMENT

inline const scfx_ieee_double
scfx_ieee_double::nan()
{
    scfx_ieee_double id;
    id.set_nan();
    return id;
}

inline const scfx_ieee_double
scfx_ieee_double::inf(int sign)
{
    scfx_ieee_double id(sign);
    id.set_inf();
    return id;
}


// ----------------------------------------------------------------------------
//  UNION : ieee_float
//
//  IEEE 754 single-precision format.
// ----------------------------------------------------------------------------

union ieee_float
{
    float f;
    struct
    {
#if defined(SC_BOOST_BIG_ENDIAN)
        unsigned negative:1;
        unsigned exponent:8;
        unsigned mantissa:23;
#elif defined(SC_BOOST_LITTLE_ENDIAN)
        unsigned mantissa:23;
        unsigned exponent:8;
        unsigned negative:1;
#endif
    } s;
};


const unsigned int SCFX_IEEE_FLOAT_BIAS = 127U;

const int SCFX_IEEE_FLOAT_E_MAX = 127;
const int SCFX_IEEE_FLOAT_E_MIN = -126;

const unsigned int SCFX_IEEE_FLOAT_M_SIZE = 23;
const unsigned int SCFX_IEEE_FLOAT_E_SIZE = 8;


// ----------------------------------------------------------------------------
//  CLASS : scfx_ieee_float
//
// Convenient wrapper to union ieee_float.
// ----------------------------------------------------------------------------

class scfx_ieee_float
{
    ieee_float m_if;

  public:
    scfx_ieee_float();
    scfx_ieee_float(float);
    scfx_ieee_float(const scfx_ieee_float &);

    scfx_ieee_float &operator = (float);
    scfx_ieee_float &operator = (const scfx_ieee_float &);

    operator float() const;

    unsigned int negative() const;
    void negative(unsigned int);
    int exponent() const;
    void exponent(int);
    unsigned int mantissa() const;
    void mantissa(unsigned int);

    bool is_zero() const;
    bool is_subnormal() const;
    bool is_normal() const;
    bool is_inf() const;
    bool is_nan() const;

    void set_inf();
    void set_nan();
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline scfx_ieee_float::scfx_ieee_float() : m_if()
{
    m_if.f = 0.0;
}

inline scfx_ieee_float::scfx_ieee_float(float f) : m_if()
{
    m_if.f = f;
}

inline scfx_ieee_float::scfx_ieee_float(const scfx_ieee_float &a) :
        m_if(a.m_if)
{
    // m_if.f = a.m_if.f;
}


inline scfx_ieee_float &
scfx_ieee_float::operator = (float f)
{
    m_if.f = f;
    return *this;
}

inline scfx_ieee_float &
scfx_ieee_float::operator = (const scfx_ieee_float &a)
{
    m_if.f = a.m_if.f;
    return *this;
}

inline scfx_ieee_float::operator float() const
{
    return m_if.f;
}

inline unsigned int
scfx_ieee_float::negative() const
{
    return m_if.s.negative;
}

inline void
scfx_ieee_float::negative(unsigned int a)
{
    m_if.s.negative = a & SCFX_MASK_(1);
}

inline int
scfx_ieee_float::exponent() const
{
    return m_if.s.exponent - SCFX_IEEE_FLOAT_BIAS;
}

inline void
scfx_ieee_float::exponent(int a)
{
    m_if.s.exponent = (SCFX_IEEE_FLOAT_BIAS + a) &
                      SCFX_MASK_(SCFX_IEEE_FLOAT_E_SIZE);
}

inline unsigned int
scfx_ieee_float::mantissa() const
{
    return m_if.s.mantissa;
}

inline void
scfx_ieee_float::mantissa(unsigned int a)
{
    m_if.s.mantissa = a & SCFX_MASK_(SCFX_IEEE_FLOAT_M_SIZE);
}


inline bool
scfx_ieee_float::is_zero() const
{
    return (exponent() == SCFX_IEEE_FLOAT_E_MIN - 1 && mantissa() == 0U);
}

inline bool
scfx_ieee_float::is_subnormal() const
{
    return (exponent() == SCFX_IEEE_FLOAT_E_MIN - 1 && mantissa() != 0U);
}

inline bool
scfx_ieee_float::is_normal() const
{
    return (exponent() >= SCFX_IEEE_FLOAT_E_MIN &&
            exponent() <= SCFX_IEEE_FLOAT_E_MAX);
}

inline bool
scfx_ieee_float::is_inf() const
{
    return (exponent() == SCFX_IEEE_FLOAT_E_MAX + 1 && mantissa() == 0U);
}

inline bool
scfx_ieee_float::is_nan() const
{
    return (exponent() == SCFX_IEEE_FLOAT_E_MAX + 1 && mantissa() != 0U);
}

inline void
scfx_ieee_float::set_inf()
{
    exponent(SCFX_IEEE_FLOAT_E_MAX + 1);
    mantissa(0U);
}

inline void
scfx_ieee_float::set_nan()
{
    exponent(SCFX_IEEE_FLOAT_E_MAX + 1);
    mantissa((unsigned int)-1);
}


// ----------------------------------------------------------------------------
//  FUNCTION : scfx_pow2
//
//  Computes 2.0**exp in double-precision.
// ----------------------------------------------------------------------------

inline double
scfx_pow2(int exp)
{
    scfx_ieee_double r;
    if (exp < SCFX_IEEE_DOUBLE_E_MIN) {
        r = 0.0;
        // handle subnormal case
        exp -= SCFX_IEEE_DOUBLE_E_MIN;
        if ((exp += 20) >= 0) {
            r.mantissa0(1U << exp);
        } else if ((exp += 32) >= 0) {
            r.mantissa1(1U << exp);
        }
    } else if (exp > SCFX_IEEE_DOUBLE_E_MAX) {
        r.set_inf();
    } else {
        r = 1.0;
        r.exponent(exp);
    }
    return r;
}


// ----------------------------------------------------------------------------
//  FUNCTION : uint64_to_double
//
//  Platform independent conversion from double uint64 to double.
//  Needed because VC++6 doesn't support this conversion.
// ----------------------------------------------------------------------------

inline double
uint64_to_double(uint64 a)
{
#if defined(__clang__)
    // conversion from uint64 to double not implemented; use int64
    double tmp = static_cast<double>(static_cast<int64>(a));
    return (tmp >= 0) ? tmp : tmp + sc_dt::scfx_pow2(64);
#else
    return static_cast<double>(a);
#endif
}

} // namespace sc_dt

#undef SCFX_MASK_

#endif // __SYSTEMC_EXT_DT_FX_SCFX_IEEE_HH__

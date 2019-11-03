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

  sc_bit.h -- Bit class.

  Original Author: Stan Y. Liao, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_bit.h,v $
// Revision 1.2  2011/08/07 18:54:19  acg
//  Philipp A. Hartmann: remove friend function declarations that implement
//  code, and clean up how bit and logic operators are defined in general.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.6  2006/05/08 17:49:59  acg
//   Andy Goodrich: Added David Long's declarations for friend operators,
//   functions, and methods, to keep the Microsoft compiler happy.
//
// Revision 1.5  2006/04/12 20:17:52  acg
//  Andy Goodrich: enabled deprecation message for sc_bit.
//
// Revision 1.4  2006/01/24 20:50:55  acg
// Andy Goodrich: added warnings indicating that sc_bit is deprecated and that
// the C bool data type should be used in its place.
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_BIT_SC_BIT_HH__
#define __SYSTEMC_EXT_DT_BIT_SC_BIT_HH__

#include <iostream>

#include "../int/sc_nbdefs.hh"

namespace sc_dt
{

// classes defined in this module
class sc_bit;

// forward class declarations
class sc_logic;

extern void sc_deprecated_sc_bit();

// ----------------------------------------------------------------------------
//  CLASS : sc_bit
//
//  Bit class.
//  Note: VSIA compatibility indicated.
// ----------------------------------------------------------------------------

class sc_bit
{
    // support methods

    static void invalid_value(char);
    static void invalid_value(int);

    static bool
    to_value(char c)
    {
        if (c != '0' && c != '1') {
            invalid_value(c);
        }
        return (c == '0' ? false : true);
    }

    static bool
    to_value(int i)
    {
        if (i != 0 && i != 1) {
            invalid_value(i);
        }
        return (i == 0 ? false : true);
    }
    static bool to_value(bool b) { return b; }

#define DEFN_TO_VALUE_T(tp) \
    static bool to_value(tp i) { return to_value((int)i); }

    DEFN_TO_VALUE_T(unsigned)
    DEFN_TO_VALUE_T(long)
    DEFN_TO_VALUE_T(unsigned long)
    DEFN_TO_VALUE_T(int64)
    DEFN_TO_VALUE_T(uint64)

#undef DEFN_TO_VALUE_T

  public:
    // constructors
    // MANDATORY
    sc_bit() : m_val(false) { sc_deprecated_sc_bit(); }

#define DEFN_CTOR_T(tp)              \
    explicit sc_bit(tp a) : m_val(to_value(a)) { sc_deprecated_sc_bit(); }

    DEFN_CTOR_T(bool)
    DEFN_CTOR_T(char)
    DEFN_CTOR_T(int)
    DEFN_CTOR_T(unsigned)
    DEFN_CTOR_T(long)
    DEFN_CTOR_T(unsigned long)
    DEFN_CTOR_T(int64)
    DEFN_CTOR_T(uint64)

#undef DEFN_CTOR_T

    explicit sc_bit(const sc_logic &a);  // non-VSIA

    // copy constructor
    // MANDATORY
    sc_bit(const sc_bit &a) : m_val(a.m_val) {}

    // destructor
    // MANDATORY
    ~sc_bit() {}

    // assignment operators
    // MANDATORY
    sc_bit &
    operator = (const sc_bit &b)
    {
        m_val = b.m_val;
        return *this;
    }

#define DEFN_ASN_OP_T(op, tp) \
    sc_bit &operator op(tp b) { return (*this op sc_bit(b)); }
#define DEFN_ASN_OP(op) \
    DEFN_ASN_OP_T(op,int) \
    DEFN_ASN_OP_T(op,bool) \
    DEFN_ASN_OP_T(op,char)

    DEFN_ASN_OP(=)
    DEFN_ASN_OP_T(=,int64)
    DEFN_ASN_OP_T(=,uint64)
    DEFN_ASN_OP_T(=,long)
    DEFN_ASN_OP_T(=,unsigned long)

    sc_bit &operator = (const sc_logic &b); // non-VSIA

    // bitwise assignment operators
    sc_bit &
    operator &= (const sc_bit &b)
    {
        m_val = (m_val && b.m_val);
        return *this;
    }

    sc_bit &
    operator |= (const sc_bit &b)
    {
        m_val = (m_val || b.m_val);
        return *this;
    }

    sc_bit &
    operator ^= (const sc_bit &b)
    {
        m_val = (m_val != b.m_val);
        return *this;
    }

    DEFN_ASN_OP(&=)
    DEFN_ASN_OP(|=)
    DEFN_ASN_OP(^=)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP

    // conversions
    // MANDATORY

    // implicit conversion to bool
    operator bool () const { return m_val; }

    // non-VSIA
    bool operator ! () const { return !m_val; }


    // explicit conversions - non-VSIA
    bool to_bool() const { return m_val; }
    char to_char() const { return (m_val ? '1' : '0'); }

    // relational operators and functions
    // MANDATORY
    friend bool operator == (const sc_bit &a, const sc_bit &b);
    friend bool operator != (const sc_bit &a, const sc_bit &b);

    // bitwise operators and functions

    // bitwise complement
    // MANDATORY
    friend const sc_bit operator ~ (const sc_bit &a);

    // RECOMMENDED
    sc_bit &
    b_not()
    {
        m_val = (!m_val);
        return *this;
    }

    // binary bit-wise operations
    friend const sc_bit operator | (const sc_bit &a, const sc_bit &b);
    friend const sc_bit operator & (const sc_bit &a, const sc_bit &b);
    friend const sc_bit operator ^ (const sc_bit &a, const sc_bit &b);

    // other methods
    void print(::std::ostream &os=::std::cout) const { os << to_bool(); }
    void scan(::std::istream & =::std::cin);

  private:
    bool m_val;
};

// ----------------------------------------------------------------------------
// relational operators and functions

#define DEFN_BIN_FUN_T(ret,fun,tp) \
    inline ret fun(const sc_bit& a, tp b) { return fun(a, sc_bit(b)); } \
    inline ret fun(tp b, const sc_bit &a) { return fun(sc_bit(a), b); }

#define DEFN_BIN_FUN(ret,fun) \
      DEFN_BIN_FUN_T(ret,fun,bool) \
      DEFN_BIN_FUN_T(ret,fun,char) \
      DEFN_BIN_FUN_T(ret,fun,int)

// MANDATORY
inline bool
operator == (const sc_bit &a, const sc_bit &b)
{
    return (a.m_val == b.m_val);
}

inline bool
operator != (const sc_bit &a, const sc_bit &b)
{
    return (a.m_val != b.m_val);
}

DEFN_BIN_FUN(bool, operator ==)
DEFN_BIN_FUN(bool, operator !=)

// OPTIONAL

inline bool equal(const sc_bit &a, const sc_bit &b) { return (a == b); }

inline bool not_equal(const sc_bit &a, const sc_bit &b) { return (a != b); }

DEFN_BIN_FUN(bool,equal)
DEFN_BIN_FUN(bool,not_equal)

// ----------------------------------------------------------------------------
// bitwise operators and functions

// bitwise complement

// MANDATORY
inline const sc_bit operator ~ (const sc_bit &a) { return sc_bit(!a.m_val); }

// OPTIONAL
inline const sc_bit b_not(const sc_bit &a) { return (~a); }

// RECOMMENDED
inline void b_not(sc_bit &r, const sc_bit &a) { r = (~a); }

// binary bit-wise operations
// MANDATORY
inline const sc_bit
operator & (const sc_bit &a, const sc_bit &b)
{
    return sc_bit(a.m_val && b.m_val);
}

inline const sc_bit
operator | (const sc_bit &a, const sc_bit &b)
{
    return sc_bit(a.m_val || b.m_val);
}

inline const sc_bit
operator ^ (const sc_bit &a, const sc_bit &b)
{
    return sc_bit(a.m_val != b.m_val);
}

DEFN_BIN_FUN(const sc_bit,operator&)
DEFN_BIN_FUN(const sc_bit,operator|)
DEFN_BIN_FUN(const sc_bit,operator^)

// OPTIONAL
inline const sc_bit b_and(const sc_bit &a, const sc_bit &b) { return a & b; }
inline const sc_bit b_or(const sc_bit &a, const sc_bit &b) { return a | b; }
inline const sc_bit b_xor(const sc_bit &a, const sc_bit &b) { return a ^ b; }

DEFN_BIN_FUN(const sc_bit,b_and)
DEFN_BIN_FUN(const sc_bit,b_or)
DEFN_BIN_FUN(const sc_bit,b_xor)

// RECOMMENDED

#define DEFN_TRN_FUN_T(fun,tp) \
    inline void \
    fun(sc_bit &r, const sc_bit &a, tp b) \
    { r = fun(a, sc_bit(b)); } \
    inline void \
    fun(sc_bit &r, tp a, const sc_bit &b) \
    { r = fun(sc_bit(a), b); }

#define DEFN_TRN_FUN(fun) \
    inline void \
    fun(sc_bit &r, const sc_bit &a, const sc_bit &b) { r = fun(a , b); } \
    DEFN_TRN_FUN_T(fun, int) \
    DEFN_TRN_FUN_T(fun, bool) \
    DEFN_TRN_FUN_T(fun, char)

    DEFN_TRN_FUN(b_and)
    DEFN_TRN_FUN(b_or)
    DEFN_TRN_FUN(b_xor)

#undef DEFN_BIN_FUN_T
#undef DEFN_BIN_FUN
#undef DEFN_TRN_FUN_T
#undef DEFN_TRN_FUN


// ----------------------------------------------------------------------------

inline ::std::ostream &
operator << (::std::ostream &os, const sc_bit &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_bit &a)
{
    a.scan(is);
    return is;
}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_BIT_SC_BIT_HH__

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

  sc_logic.h -- C++ implementation of logic type. Behaves
                pretty much the same way as HDLs except with 4 values.

  Original Author: Stan Y. Liao, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_logic.h,v $
// Revision 1.3  2011/08/07 18:54:19  acg
//  Philipp A. Hartmann: remove friend function declarations that implement
//  code, and clean up how bit and logic operators are defined in general.
//
// Revision 1.2  2011/01/25 20:50:37  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.5  2006/12/02 21:00:57  acg
//  Andy Goodrich: fixes for concatenation support.
//
// Revision 1.4  2006/05/08 17:49:59  acg
//   Andy Goodrich: Added David Long's declarations for friend operators,
//   functions, and methods, to keep the Microsoft compiler happy.
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_BIT_SC_LOGIC_HH__
#define __SYSTEMC_EXT_DT_BIT_SC_LOGIC_HH__

#include <cstdio>
#include <iostream>

#include "../sc_mempool.hh"
#include "sc_bit.hh"

namespace sc_dt
{

// classes defined in this module
class sc_logic;


// ----------------------------------------------------------------------------
//  ENUM : sc_logic_value_t
//
//  Enumeration of values for sc_logic.
// ----------------------------------------------------------------------------

enum sc_logic_value_t
{
    Log_0 = 0,
    Log_1,
    Log_Z,
    Log_X
};

// ----------------------------------------------------------------------------
//  CLASS : sc_logic
//
//  Four-valued logic type.
// ----------------------------------------------------------------------------

class sc_logic
{
  private:
    // support methods
    static void invalid_value(sc_logic_value_t);
    static void invalid_value(char);
    static void invalid_value(int);

    static sc_logic_value_t
    to_value(sc_logic_value_t v)
    {
        if (v < Log_0 || v > Log_X) {
            invalid_value(v);
            // may continue, if suppressed
            v = Log_X;
        }
        return v;
    }

    static sc_logic_value_t to_value(bool b) { return (b ? Log_1 : Log_0); }

    static sc_logic_value_t
    to_value(char c)
    {
        unsigned int index = (int)c;
        if (index > 127) {
            invalid_value(c);
            // may continue, if suppressed
            index = 127; // aka Log_X
        }
        return char_to_logic[index];
    }

    static sc_logic_value_t
    to_value(int i)
    {
        if (i < Log_0 || i > Log_X) {
            invalid_value(i);
            // may continue, if suppressed
            i = Log_X;
        }
        return sc_logic_value_t(i);
    }

    void invalid_01() const;

  public:
    // conversion tables
    static const sc_logic_value_t char_to_logic[128];
    static const char logic_to_char[4];
    static const sc_logic_value_t and_table[4][4];
    static const sc_logic_value_t or_table[4][4];
    static const sc_logic_value_t xor_table[4][4];
    static const sc_logic_value_t not_table[4];

    // constructors
    sc_logic() : m_val(Log_X) {}
    sc_logic(const sc_logic &a) : m_val(a.m_val) {}
    sc_logic(sc_logic_value_t v) : m_val(to_value(v)) {}
    explicit sc_logic(bool a) : m_val(to_value(a)) {}
    explicit sc_logic(char a) : m_val(to_value(a)) {}
    explicit sc_logic(int a) : m_val(to_value(a)) {}
    explicit sc_logic(const sc_bit &a) : m_val(to_value(a.to_bool())) {}

    // destructor
    ~sc_logic() {}

    // (bitwise) assignment operators
#define DEFN_ASN_OP_T(op,tp) \
    sc_logic & \
    operator op (tp v) \
    { \
        *this op sc_logic(v); \
        return *this; \
    }

#define DEFN_ASN_OP(op) \
    DEFN_ASN_OP_T(op, sc_logic_value_t) \
    DEFN_ASN_OP_T(op, bool) \
    DEFN_ASN_OP_T(op, char) \
    DEFN_ASN_OP_T(op, int) \
    DEFN_ASN_OP_T(op, const sc_bit &)

    sc_logic &
    operator = (const sc_logic &a)
    {
        m_val = a.m_val;
        return *this;
    }

    sc_logic &
    operator &= (const sc_logic &b)
    {
        m_val = and_table[m_val][b.m_val];
        return *this;
    }

    sc_logic &
    operator |= (const sc_logic &b)
    {
        m_val = or_table[m_val][b.m_val];
        return *this;
    }

    sc_logic &
    operator ^= (const sc_logic &b)
    {
        m_val = xor_table[m_val][b.m_val];
        return *this;
    }

    DEFN_ASN_OP(=)
    DEFN_ASN_OP(&=)
    DEFN_ASN_OP(|=)
    DEFN_ASN_OP(^=)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP

    // bitwise operators and functions
    friend const sc_logic operator & (const sc_logic &, const sc_logic &);
    friend const sc_logic operator | (const sc_logic &, const sc_logic &);
    friend const sc_logic operator ^ (const sc_logic &, const sc_logic &);

    // relational operators
    friend bool operator == (const sc_logic &, const sc_logic &);
    friend bool operator != (const sc_logic &, const sc_logic &);

    // bitwise complement
    const sc_logic operator ~ () const { return sc_logic(not_table[m_val]); }
    sc_logic &
    b_not()
    {
        m_val = not_table[m_val];
        return *this;
    }

    // explicit conversions
    sc_logic_value_t value() const { return m_val; }

    bool is_01() const { return ((int)m_val == Log_0 || (int)m_val == Log_1); }
    bool
    to_bool() const
    {
        if (!is_01()) {
            invalid_01();
        }
        return ((int)m_val != Log_0);
    }

    char to_char() const { return logic_to_char[m_val]; }

    // other methods
    void print(::std::ostream &os=::std::cout) const { os << to_char(); }
    void scan(::std::istream &is=::std::cin);

    // memory (de)allocation
    // placement new
    static void *operator new (std::size_t, void *p) { return p; }
    static void *
    operator new (std::size_t sz)
    {
        return sc_core::sc_mempool::allocate(sz);
    }
    static void
    operator delete (void *p, std::size_t sz)
    {
        sc_core::sc_mempool::release(p, sz);
    }
    static void *
    operator new [] (std::size_t sz)
    {
        return sc_core::sc_mempool::allocate(sz);
    }
    static void
    operator delete [] (void *p, std::size_t sz)
    {
        sc_core::sc_mempool::release(p, sz);
    }

  private:
    sc_logic_value_t m_val;

  private:
    // Disabled
    explicit sc_logic(const char *);
    sc_logic &operator = (const char *);
};

// ----------------------------------------------------------------------------

// bitwise operators
inline const sc_logic
operator & (const sc_logic &a, const sc_logic &b)
{
    return sc_logic(sc_logic::and_table[a.m_val][b.m_val]);
}

inline const sc_logic
operator | (const sc_logic &a, const sc_logic &b)
{
    return sc_logic(sc_logic::or_table[a.m_val][b.m_val]);
}

inline const sc_logic
operator ^ (const sc_logic &a, const sc_logic &b)
{
    return sc_logic(sc_logic::xor_table[a.m_val][b.m_val]);
}

#define DEFN_BIN_OP_T(ret,op,tp) \
    inline ret \
    operator op (const sc_logic &a, tp b) \
    { \
        return (a op sc_logic(b)); \
    } \
    inline ret \
    operator op (tp a, const sc_logic &b) \
    { \
        return (sc_logic(a) op b); \
    }

#define DEFN_BIN_OP(ret, op) \
    DEFN_BIN_OP_T(ret, op, sc_logic_value_t) \
    DEFN_BIN_OP_T(ret, op, bool) \
    DEFN_BIN_OP_T(ret, op, char) \
    DEFN_BIN_OP_T(ret, op, int)

DEFN_BIN_OP(const sc_logic, &)
DEFN_BIN_OP(const sc_logic, |)
DEFN_BIN_OP(const sc_logic, ^)

// relational operators and functions

inline bool
operator == (const sc_logic &a, const sc_logic &b)
{
    return ((int)a.m_val == b.m_val);
}

inline bool
operator != (const sc_logic &a, const sc_logic &b)
{
    return ((int)a.m_val != b.m_val);
}

DEFN_BIN_OP(bool, ==)
DEFN_BIN_OP(bool, !=)

#undef DEFN_BIN_OP_T
#undef DEFN_BIN_OP

// ----------------------------------------------------------------------------

inline ::std::ostream &
operator << (::std::ostream &os, const sc_logic &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_logic &a)
{
    a.scan(is);
    return is;
}


extern const sc_logic SC_LOGIC_0;
extern const sc_logic SC_LOGIC_1;
extern const sc_logic SC_LOGIC_Z;
extern const sc_logic SC_LOGIC_X;

// #ifdef SC_DT_DEPRECATED
extern const sc_logic sc_logic_0;
extern const sc_logic sc_logic_1;
extern const sc_logic sc_logic_Z;
extern const sc_logic sc_logic_X;
// #endif

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_BIT_SC_LOGIC_HH__

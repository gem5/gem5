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

  sc_proxy.h -- Proxy base class for vector data types.

                This class is created for several purposes:
                1) hiding operators from the global namespace that would be
                   otherwise found by Koenig lookup
                2) avoiding repeating the same operations in every class
                   including proxies that could also be achieved by common
                   base class, but this method allows
                3) improve performance by using non-virtual functions

  Original Author: Gene Bushuyev, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_proxy.h,v $
// Revision 1.3  2010/12/07 20:09:07  acg
// Andy Goodrich: Fix for returning enough data
//
// Revision 1.2  2009/02/28 00:26:14  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.1.1.1  2006/12/15 20:31:36  acg
// SystemC 2.2
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_BIT_SC_PROXY_HH__
#define __SYSTEMC_EXT_DT_BIT_SC_PROXY_HH__

#include <iostream>

#include "../../utils/functions.hh"
#include "../int/sc_int_base.hh"
#include "../int/sc_signed.hh"
#include "../int/sc_uint_base.hh"
#include "../int/sc_unsigned.hh"
#include "messages.hh"
#include "sc_bit.hh"
#include "sc_logic.hh"

namespace sc_dt
{

// classes defined in this module
template <class X>
class sc_proxy;

// forward class declarations
class sc_bv_base;
class sc_lv_base;
template <class X>
class sc_bitref_r;
template <class X>
class sc_bitref;
template <class X>
class sc_subref_r;
template <class X>
class sc_subref;
template <class X, class Y>
class sc_concref_r;
template <class X, class Y>
class sc_concref;

const int SC_DIGIT_SIZE = BITS_PER_BYTE * sizeof(sc_digit);

const sc_digit SC_DIGIT_ZERO = (sc_digit)0;
const sc_digit SC_DIGIT_ONE = (sc_digit)1;
const sc_digit SC_DIGIT_TWO = (sc_digit)2;

void sc_proxy_out_of_bounds(const char *msg=NULL, int64 val=0);

// assignment functions; forward declarations

template <class X, class Y>
inline void assign_p_(sc_proxy<X> &px, const sc_proxy<Y> &py);

// Vector types that are not derived from sc_proxy must have a length()
// function and an operator [].

template <class X, class T>
inline void assign_v_(sc_proxy<X> &px, const T &a);

// other functions; forward declarations
const std::string convert_to_bin(const char *s);
const std::string convert_to_fmt(const std::string &s, sc_numrep numrep, bool);

// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_proxy_traits
//
// Template traits helper to select the correct bit/value/vector_types for
// sc_proxy-based vector classes.
//
// All types derived from/based on a bit-vector contain typedef to a plain
// bool, all others point to the sc_logic_value_t/sc_logic/sc_lv_base types.
// ----------------------------------------------------------------------------

template <typename X>
struct sc_proxy_traits;

template <>
struct sc_proxy_traits<sc_bv_base>
{
    typedef sc_proxy_traits<sc_bv_base> traits_type;
    typedef bool value_type;
    typedef sc_logic bit_type; // sc_logic needed for mixed expressions
    typedef sc_bv_base vector_type;
};

template <>
struct sc_proxy_traits<sc_lv_base>
{
    typedef sc_proxy_traits<sc_lv_base> traits_type;
    typedef sc_logic_value_t value_type;
    typedef sc_logic bit_type;
    typedef sc_lv_base vector_type;
};

template <typename X>
struct sc_proxy_traits<sc_bitref_r<X> > : sc_proxy_traits<X> {};

template <typename X>
struct sc_proxy_traits<sc_bitref<X> > : sc_proxy_traits<X> {};

template <typename X>
struct sc_proxy_traits<sc_subref_r<X> > : sc_proxy_traits<X> {};

template <typename X>
struct sc_proxy_traits<sc_subref<X> > : sc_proxy_traits<X> {};

template <typename X>
struct sc_proxy_traits<sc_proxy<X> > : sc_proxy_traits<X> {};


template <typename X, typename Y>
struct sc_mixed_proxy_traits_helper : sc_proxy_traits<sc_lv_base>
{}; // logic vector by default

template <typename X>
struct sc_mixed_proxy_traits_helper<X, X> : X {};

template <typename X, typename Y>
struct sc_proxy_traits<sc_concref_r<X, Y> > :
        sc_mixed_proxy_traits_helper<
            typename X::traits_type, typename Y::traits_type>
{};

template <typename X, typename Y>
struct sc_proxy_traits<sc_concref<X, Y> > :
        sc_mixed_proxy_traits_helper<
            typename X::traits_type, typename Y::traits_type>
{};


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_proxy
//
//  Base class template for bit/logic vector classes.
//  (Barton/Nackmann implementation)
// ----------------------------------------------------------------------------

template <class X>
class sc_proxy // #### : public sc_value_base
{
  public:
    typedef typename sc_proxy_traits<X>::traits_type traits_type;
    typedef typename traits_type::bit_type bit_type;
    typedef typename traits_type::value_type value_type;

    // virtual destructor
    virtual ~sc_proxy() {}

    // casts
    X &back_cast() { return static_cast<X &>(*this); }

    const X &back_cast() const { return static_cast<const X &>(*this); }

    // assignment operators
    template <class Y>
    X &
    assign_(const sc_proxy<Y> &a)
    {
        assign_p_(*this, a);
        return back_cast();
    }

    X &assign_(const char *a);
    X &assign_(const bool *a);
    X &assign_(const sc_logic *a);

    X &
    assign_(const sc_unsigned &a)
    {
        assign_v_(*this, a);
        return back_cast();
    }

    X &
    assign_(const sc_signed &a)
    {
        assign_v_(*this, a);
        return back_cast();
    }

    X &assign_(const sc_uint_base &a) { return assign_((uint64)a); }
    X &assign_(const sc_int_base &a) { return assign_((int64)a); }
    X &assign_(unsigned int a);
    X &assign_(int a);
    X &assign_(unsigned long a);
    X &assign_(long a);
    X &assign_(uint64 a);
    X &assign_(int64 a);

    // bitwise operators and functions

    // bitwise complement
    X &b_not();

    const sc_lv_base operator ~ () const;

    // bitwise and
    X &operator &= (const char *b);
    X &operator &= (const bool *b);
    X &operator &= (const sc_logic *b);
    X &operator &= (const sc_unsigned &b);
    X &operator &= (const sc_signed &b);
    X &operator &= (const sc_uint_base &b) { return operator &= ((uint64)b); }
    X &operator &= (const sc_int_base &b) { return operator &= ((int64)b); }
    X &operator &= (unsigned long b);
    X &operator &= (long b);
    X &operator &= (unsigned int b) { return operator &= ((unsigned long)b); }
    X &operator &= (int b) { return operator &= ((long)b); }
    X &operator &= (uint64 b);
    X &operator &= (int64 b);

    const sc_lv_base operator & (const char *b) const;
    const sc_lv_base operator & (const bool *b) const;
    const sc_lv_base operator & (const sc_logic *b) const;
    const sc_lv_base operator & (const sc_unsigned &b) const;
    const sc_lv_base operator & (const sc_signed &b) const;
    const sc_lv_base operator & (const sc_uint_base &b) const;
    const sc_lv_base operator & (const sc_int_base &b) const;
    const sc_lv_base operator & (unsigned long b) const;
    const sc_lv_base operator & (long b) const;
    const sc_lv_base operator & (unsigned int b) const;
    const sc_lv_base operator & (int b) const;
    const sc_lv_base operator & (uint64 b) const;
    const sc_lv_base operator & (int64 b) const;

    // bitwise or
    X &operator |= (const char *b);
    X &operator |= (const bool *b);
    X &operator |= (const sc_logic *b);
    X &operator |= (const sc_unsigned &b);
    X &operator |= (const sc_signed &b);
    X &operator |= (const sc_uint_base &b) { return operator |= ((uint64)b); }
    X &operator |= (const sc_int_base &b) { return operator |= ((int64)b); }
    X &operator |= (unsigned long b);
    X &operator |= (long b);
    X &operator |= (unsigned int b) { return operator |= ((unsigned long)b); }
    X &operator |= (int b) { return operator |= ((long)b); }
    X &operator |= (uint64 b);
    X &operator |= (int64 b);

    const sc_lv_base operator | (const char *b) const;
    const sc_lv_base operator | (const bool *b) const;
    const sc_lv_base operator | (const sc_logic *b) const;
    const sc_lv_base operator | (const sc_unsigned &b) const;
    const sc_lv_base operator | (const sc_signed &b) const;
    const sc_lv_base operator | (const sc_uint_base &b) const;
    const sc_lv_base operator | (const sc_int_base &b) const;
    const sc_lv_base operator | (unsigned long b) const;
    const sc_lv_base operator | (long b) const;
    const sc_lv_base operator | (unsigned int b) const;
    const sc_lv_base operator | (int b) const;
    const sc_lv_base operator | (uint64 b) const;
    const sc_lv_base operator | (int64 b) const;

    // bitwise xor
    X &operator ^= (const char *b);
    X &operator ^= (const bool *b);
    X &operator ^= (const sc_logic *b);
    X &operator ^= (const sc_unsigned &b);
    X &operator ^= (const sc_signed &b);
    X &operator ^= (const sc_uint_base &b) { return operator ^= ((uint64)b); }
    X &operator ^= (const sc_int_base &b) { return operator ^= ((int64)b); }
    X &operator ^= (unsigned long b);
    X &operator ^= (long b);
    X &operator ^= (unsigned int b) { return operator ^= ((unsigned long)b); }
    X &operator ^= (int b) { return operator ^= ((long)b); }
    X &operator ^= (uint64 b);
    X &operator ^= (int64 b);

    const sc_lv_base operator ^ (const char *b) const;
    const sc_lv_base operator ^ (const bool *b) const;
    const sc_lv_base operator ^ (const sc_logic *b) const;
    const sc_lv_base operator ^ (const sc_unsigned &b) const;
    const sc_lv_base operator ^ (const sc_signed &b) const;
    const sc_lv_base operator ^ (const sc_uint_base &b) const;
    const sc_lv_base operator ^ (const sc_int_base &b) const;
    const sc_lv_base operator ^ (unsigned long b) const;
    const sc_lv_base operator ^ (long b) const;
    const sc_lv_base operator ^ (unsigned int b) const;
    const sc_lv_base operator ^ (int b) const;
    const sc_lv_base operator ^ (uint64 b) const;
    const sc_lv_base operator ^ (int64 b) const;

    // bitwise left shift
    X &operator <<= (int n);
    const sc_lv_base operator << (int n) const;

    // bitwise right shift
    X &operator >>= (int n);
    const sc_lv_base operator >> (int n) const;

    // bitwise left rotate
    X &lrotate(int n);

    // bitwise right rotate
    X &rrotate(int n);

    // bitwise reverse
    X &reverse();

    // bit selection
    sc_bitref<X> operator [] (int i) { return sc_bitref<X>(back_cast(), i); }
    sc_bitref_r<X>
    operator [] (int i) const
    {
        return sc_bitref_r<X>(back_cast(), i);
    }
    sc_bitref<X> bit(int i) { return sc_bitref<X>(back_cast(), i); }
    sc_bitref_r<X> bit(int i) const { return sc_bitref_r<X>(back_cast(), i); }

    // part selection
    sc_subref<X>
    operator () (int hi, int lo)
    {
        return sc_subref<X>(back_cast(), hi, lo);
    }
    sc_subref_r<X>
    operator () (int hi, int lo) const
    {
        return sc_subref_r<X>(back_cast(), hi, lo);
    }
    sc_subref<X>
    range(int hi, int lo)
    {
        return sc_subref<X>(back_cast(), hi, lo);
    }
    sc_subref_r<X>
    range(int hi, int lo) const
    {
        return sc_subref_r<X>(back_cast(), hi, lo);
    }

    // reduce functions
    value_type and_reduce() const;
    value_type
    nand_reduce() const
    {
        return sc_logic::not_table[and_reduce()];
    }
    value_type or_reduce() const;
    value_type nor_reduce() const { return sc_logic::not_table[or_reduce()]; }
    value_type xor_reduce() const;
    value_type
    xnor_reduce() const
    {
        return sc_logic::not_table[xor_reduce()];
    }

    // relational operators
    bool operator == (const char *b) const;
    bool operator == (const bool *b) const;
    bool operator == (const sc_logic *b) const;
    bool operator == (const sc_unsigned &b) const;
    bool operator == (const sc_signed &b) const;
    bool operator == (const sc_uint_base &b) const;
    bool operator == (const sc_int_base &b) const;
    bool operator == (unsigned long b) const;
    bool operator == (long b) const;
    bool operator == (unsigned int b) const;
    bool operator == (int b) const;
    bool operator == (uint64 b) const;
    bool operator == (int64 b) const;

    // explicit conversions to character string
    const std::string to_string() const;
    const std::string to_string(sc_numrep) const;
    const std::string to_string(sc_numrep, bool) const;

    // explicit conversions
    inline int64 to_int64() const { return to_anything_signed(); }
    inline uint64 to_uint64() const;
    int to_int() const { return (int)to_anything_signed(); }

    unsigned int
    to_uint() const
    {
        return (unsigned int)to_anything_unsigned();
    }

    long to_long() const { return (long)to_anything_signed(); }

    unsigned long
    to_ulong() const
    {
        return (unsigned long)to_anything_unsigned();
    }

    // other methods
    void
    print(::std::ostream &os=::std::cout) const
    {
        // The test below will force printing in binary if decimal is
        // specified.
        if (sc_io_base(os, SC_DEC) == SC_DEC)
            os << to_string();
        else
            os << to_string(sc_io_base(os, SC_BIN), sc_io_show_base(os));
    }

    void scan(::std::istream &is=::std::cin);

  protected:
    void check_bounds(int n) const; // check if bit n accessible
    void check_wbounds(int n) const; // check if word n accessible

    sc_digit to_anything_unsigned() const;
    int64 to_anything_signed() const;
};


// ----------------------------------------------------------------------------

// bitwise operators and functions

// bitwise and

template <class X, class Y>
inline X &operator &= (sc_proxy<X> &px, const sc_proxy<Y> &py);


template <class X, class Y>
inline const sc_lv_base operator & (
        const sc_proxy<X> &px, const sc_proxy<Y> &py);


#define DECL_BITWISE_AND_OP_T(tp) \
template <class X> \
inline const sc_lv_base operator & (tp b, const sc_proxy<X> &px);

DECL_BITWISE_AND_OP_T(const char *)
DECL_BITWISE_AND_OP_T(const bool *)
DECL_BITWISE_AND_OP_T(const sc_logic *)
DECL_BITWISE_AND_OP_T(const sc_unsigned &)
DECL_BITWISE_AND_OP_T(const sc_signed &)
DECL_BITWISE_AND_OP_T(const sc_uint_base &)
DECL_BITWISE_AND_OP_T(const sc_int_base &)
DECL_BITWISE_AND_OP_T(unsigned long)
DECL_BITWISE_AND_OP_T(long)
DECL_BITWISE_AND_OP_T(unsigned int)
DECL_BITWISE_AND_OP_T(int)
DECL_BITWISE_AND_OP_T(uint64)
DECL_BITWISE_AND_OP_T(int64)

#undef DECL_BITWISE_AND_OP_T

// bitwise or
template <class X, class Y>
inline X &operator |= (sc_proxy<X> &px, const sc_proxy<Y> &py);

template <class X, class Y>
inline const sc_lv_base operator | (
        const sc_proxy<X> &px, const sc_proxy<Y> &py);


#define DECL_BITWISE_OR_OP_T(tp) \
template <class X> \
inline const sc_lv_base operator | (tp a, const sc_proxy<X> &px);

DECL_BITWISE_OR_OP_T(const char *)
DECL_BITWISE_OR_OP_T(const bool *)
DECL_BITWISE_OR_OP_T(const sc_logic *)
DECL_BITWISE_OR_OP_T(const sc_unsigned &)
DECL_BITWISE_OR_OP_T(const sc_signed &)
DECL_BITWISE_OR_OP_T(const sc_uint_base &)
DECL_BITWISE_OR_OP_T(const sc_int_base &)
DECL_BITWISE_OR_OP_T(unsigned long)
DECL_BITWISE_OR_OP_T(long)
DECL_BITWISE_OR_OP_T(unsigned int)
DECL_BITWISE_OR_OP_T(int)
DECL_BITWISE_OR_OP_T(uint64)
DECL_BITWISE_OR_OP_T(int64)

#undef DECL_BITWISE_OR_OP_T

// bitwise xor
template <class X, class Y>
inline X &operator ^= (sc_proxy<X> &px, const sc_proxy<Y> &py);

template <class X, class Y>
inline const sc_lv_base operator ^ (
        const sc_proxy<X> &px, const sc_proxy<Y> &py);

#define DECL_BITWISE_XOR_OP_T(tp) \
template <class X> \
inline const sc_lv_base operator ^ (tp a, const sc_proxy<X> &px);

DECL_BITWISE_XOR_OP_T(const char *)
DECL_BITWISE_XOR_OP_T(const bool *)
DECL_BITWISE_XOR_OP_T(const sc_logic *)
DECL_BITWISE_XOR_OP_T(const sc_unsigned &)
DECL_BITWISE_XOR_OP_T(const sc_signed &)
DECL_BITWISE_XOR_OP_T(const sc_uint_base &)
DECL_BITWISE_XOR_OP_T(const sc_int_base &)
DECL_BITWISE_XOR_OP_T(unsigned long)
DECL_BITWISE_XOR_OP_T(long)
DECL_BITWISE_XOR_OP_T(unsigned int)
DECL_BITWISE_XOR_OP_T(int)
DECL_BITWISE_XOR_OP_T(uint64)
DECL_BITWISE_XOR_OP_T(int64)

#undef DECL_BITWISE_XOR_OP_T

// relational operators
template <class X, class Y>
inline bool operator == (const sc_proxy<X> &px, const sc_proxy<Y> &py);

template <class X, class Y>
inline bool operator != (const sc_proxy<X> &px, const sc_proxy<Y> &py);

#define DECL_REL_OP_T(tp) \
template <class X> \
inline bool operator == (tp b, const sc_proxy<X> &px); \
 \
template <class X> \
inline bool operator != (const sc_proxy<X> &px, tp b); \
 \
template <class X> \
inline bool operator != (tp b, const sc_proxy<X> &px);

DECL_REL_OP_T(const char *)
DECL_REL_OP_T(const bool *)
DECL_REL_OP_T(const sc_logic *)
DECL_REL_OP_T(const sc_unsigned &)
DECL_REL_OP_T(const sc_signed &)
DECL_REL_OP_T(const sc_uint_base &)
DECL_REL_OP_T(const sc_int_base &)
DECL_REL_OP_T(unsigned long)
DECL_REL_OP_T(long)
DECL_REL_OP_T(unsigned int)
DECL_REL_OP_T(int)
DECL_REL_OP_T(uint64)
DECL_REL_OP_T(int64)

#undef DECL_REL_OP_T

// l-value concatenation

// Due to the fact that temporary objects cannot be passed to non-const
// references, we have to enumerate, use call by value, and use dynamic
// memory allocation (and deallocation).


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

template <class X>
inline void
get_words_(const X &x, int wi, sc_digit &x_dw, sc_digit &x_cw)
{
    x_dw = x.get_word(wi);
    x_cw = x.get_cword(wi);
}

template <class X>
inline void
set_words_(X &x, int wi, sc_digit x_dw, sc_digit x_cw)
{
    x.set_word(wi, x_dw);
    x.set_cword(wi, x_cw);
}

template <class X>
inline void
extend_sign_w_(X &x, int wi, bool sign)
{
    int sz = x.size();
    unsigned int sgn = (sign ? ~SC_DIGIT_ZERO : SC_DIGIT_ZERO);
    for (int i = wi; i < sz; ++i) {
        set_words_(x, i, sgn, SC_DIGIT_ZERO);
    }
}

// assignment functions
template <class X, class Y>
inline void
assign_p_(sc_proxy<X> &px, const sc_proxy<Y> &py)
{
    if ((void *)&px != (void *)&py) {
        X &x = px.back_cast();
        const Y &y = py.back_cast();
        int sz = x.size();
        int min_sz = sc_min(sz, y.size());
        int i = 0;
        for (; i < min_sz; ++i) {
            set_words_(x, i, y.get_word(i), y.get_cword(i));
        }
        // extend with zeros
        extend_sign_w_(x, i, false);
        x.clean_tail();
    }
}

// Vector types that are not derived from sc_proxy, sc_int_base,
// sc_uint_base, sc_signed, or sc_unsigned, must have a length()
// function and an operator []. The vector argument type must support
// accessing bits that are beyond the msb. The vector argument type
// decides what to do there (e.g. sign extension or zero padding).

template <class X, class T>
inline void
assign_v_(sc_proxy<X> &px, const T &a)
{
    X &x = px.back_cast();
    int i;
    int len_x = x.length();
    int len_a = a.length();
    if (len_a > len_x)
        len_a = len_x;
    for (i = 0; i < len_a; ++i) {
        x.set_bit(i, sc_logic_value_t((bool)a[i]));
    }
    for (; i < len_x; ++i) {
        x.set_bit(i, sc_logic_value_t(false));
    }
}

template <class X>
inline void
assign_v_(sc_proxy<X> &px, const sc_int_base &a)
{
    X &x = px.back_cast();
    int i;
    bool sign = a < 0;
    int len_x = x.length();
    int len_a = a.length();
    if ( len_a > len_x ) len_a = len_x;
    for (i = 0; i < len_a; ++i) {
        x.set_bit(i, sc_logic_value_t((bool)a[i]));
    }
    for (; i < len_x; ++i) {
        x.set_bit(i, sc_logic_value_t(sign));
    }
}

template <class X>
inline void
assign_v_(sc_proxy<X> &px, const sc_signed &a)
{
    X &x = px.back_cast();
    int i;
    bool sign = a < 0;
    int len_x = x.length();
    int len_a = a.length();
    if (len_a > len_x)
        len_a = len_x;
    for (i = 0; i < len_a; ++i) {
        x.set_bit(i, sc_logic_value_t((bool)a[i]));
    }
    for (; i < len_x; ++i) {
        x.set_bit(i, sc_logic_value_t(sign));
    }
}

template <class X>
inline void
assign_v_(sc_proxy<X> &px, const sc_uint_base &a)
{
    X &x = px.back_cast();
    int i;
    int len_x = x.length();
    int len_a = a.length();
    if (len_a > len_x)
        len_a = len_x;
    for (i = 0; i < len_a; ++i) {
        x.set_bit(i, sc_logic_value_t((bool)a[i]));
    }
    for (; i < len_x; ++i) {
        x.set_bit(i, sc_logic_value_t(false));
    }
}

template <class X>
inline void
assign_v_(sc_proxy<X> &px, const sc_unsigned &a)
{
    X &x = px.back_cast();
    int i;
    int len_x = x.length();
    int len_a = a.length();
    if (len_a > len_x)
        len_a = len_x;
    for (i = 0; i < len_a; ++i) {
        x.set_bit(i, sc_logic_value_t((bool)a[i]));
    }
    for (; i < len_x; ++i) {
        x.set_bit(i, sc_logic_value_t(false));
    }
}

// assignment operators
template <class X>
inline X &
sc_proxy<X>::assign_(const char *a)
{
    X &x = back_cast();
    std::string s = convert_to_bin(a);
    int len = x.length();
    int s_len = s.length() - 1;
    int min_len = sc_min(len, s_len);
    int i = 0;
    for (; i < min_len; ++i) {
        char c = s[s_len - i - 1];
        x.set_bit(i, sc_logic::char_to_logic[(int)c]);
    }
    // if formatted, fill the rest with sign(s), otherwise fill with zeros
    sc_logic_value_t fill = (s[s_len] == 'F' ? sc_logic_value_t(s[0] - '0')
                                             : sc_logic_value_t(0));
    for (; i < len; ++i) {
        x.set_bit(i, fill);
    }
    return x;
}

template <class X>
inline X &
sc_proxy<X>::assign_(const bool *a)
{
    // the length of 'a' must be larger than or equal to the length of 'this'
    X &x = back_cast();
    int len = x.length();
    for (int i = 0; i < len; ++i) {
        x.set_bit(i, sc_logic_value_t(a[i]));
    }
    return x;
}

template <class X>
inline X &
sc_proxy<X>::assign_(const sc_logic *a)
{
    // the length of 'a' must be larger than or equal to the length of 'this'
    X &x = back_cast();
    int len = x.length();
    for (int i = 0; i < len; ++i) {
        x.set_bit(i, a[i].value());
    }
    return x;
}

template <class X>
inline X &
sc_proxy<X>::assign_(unsigned int a)
{
    X &x = back_cast();
    set_words_(x, 0, (sc_digit)a, SC_DIGIT_ZERO);
    // extend with zeros
    extend_sign_w_(x, 1, false);
    x.clean_tail();
    return x;
}

template <class X>
inline X &
sc_proxy<X>::assign_(int a)
{
    X &x = back_cast();
    set_words_(x, 0, (sc_digit)a, SC_DIGIT_ZERO);
    // extend with sign(a)
    extend_sign_w_(x, 1, (a < 0));
    x.clean_tail();
    return x;
}

#if SC_LONG_64
template <class X>
inline X &
sc_proxy<X>::assign_(unsigned long a)
{
    X &x = back_cast();
    set_words_(x, 0, ((sc_digit)a & ~SC_DIGIT_ZERO), SC_DIGIT_ZERO);
    if (x.size() > 1) {
        set_words_(x, 1, ((sc_digit)(a >> SC_DIGIT_SIZE) & ~SC_DIGIT_ZERO),
                   SC_DIGIT_ZERO);
        // extend with zeros
        extend_sign_w_(x, 2, false);
    }
    x.clean_tail();
    return x;
}

template <class X>
inline X &
sc_proxy<X>::assign_(long a)
{
    X &x = back_cast();
    set_words_(x, 0, ((sc_digit)a & ~SC_DIGIT_ZERO), SC_DIGIT_ZERO);
    if (x.size() > 1) {
        set_words_(x, 1,
                   ((sc_digit)((uint64)a >> SC_DIGIT_SIZE) & ~SC_DIGIT_ZERO),
                   SC_DIGIT_ZERO);
        // extend with sign(a)
        extend_sign_w_(x, 2, (a < 0));
    }
    x.clean_tail();
    return x;
}

#else

template <class X>
inline X &
sc_proxy<X>::assign_(unsigned long a)
{
    X &x = back_cast();
    set_words_(x, 0, (sc_digit)a, SC_DIGIT_ZERO);
    // extend with zeros
    extend_sign_w_(x, 1, false);
    x.clean_tail();
    return x;
}

template <class X>
inline X &
sc_proxy<X>::assign_(long a)
{
    X &x = back_cast();
    set_words_(x, 0, (sc_digit)a, SC_DIGIT_ZERO);
    // extend with sign(a)
    extend_sign_w_(x, 1, (a < 0));
    x.clean_tail();
    return x;
}

#endif

template <class X>
inline X &
sc_proxy<X>::assign_(uint64 a)
{
    X &x = back_cast();
    set_words_(x, 0, ((sc_digit)a & ~SC_DIGIT_ZERO), SC_DIGIT_ZERO);
    if (x.size() > 1) {
        set_words_(x, 1, ((sc_digit) (a >> SC_DIGIT_SIZE) & ~SC_DIGIT_ZERO),
                   SC_DIGIT_ZERO );
        // extend with zeros
        extend_sign_w_(x, 2, false);
    }
    x.clean_tail();
    return x;
}

template <class X>
inline X &
sc_proxy<X>::assign_(int64 a)
{
    X &x = back_cast();
    set_words_(x, 0, ((sc_digit)a & ~SC_DIGIT_ZERO), SC_DIGIT_ZERO);
    if (x.size() > 1) {
        set_words_(x, 1,
                   ((sc_digit)((uint64)a >> SC_DIGIT_SIZE) & ~SC_DIGIT_ZERO),
                   SC_DIGIT_ZERO );
        // extend with sign(a)
        extend_sign_w_(x, 2, (a < 0));
    }
    x.clean_tail();
    return x;
}

// bitwise operators and functions

// bitwise complement
template <class X>
inline X &
sc_proxy<X>::b_not()
{
    X &x = back_cast();
    int sz = x.size();
    for (int i = 0; i < sz; ++i) {
        sc_digit x_dw, x_cw;
        get_words_(x, i, x_dw, x_cw);
        x.set_word(i, x_cw | ~x_dw);
    }
    x.clean_tail();
    return x;
}

// bitwise and
template <class X, class Y>
inline X &
b_and_assign_(sc_proxy<X> &px, const sc_proxy<Y> &py)
{
    X &x = px.back_cast();
    const Y &y = py.back_cast();
    sc_assert(x.length() == y.length());
    int sz = x.size();
    for (int i = 0; i < sz; ++i) {
        sc_digit x_dw, x_cw, y_dw, y_cw;
        get_words_(x, i, x_dw, x_cw);
        get_words_(y, i, y_dw, y_cw);
        sc_digit cw = (x_dw & y_cw) | (x_cw & y_dw) | (x_cw & y_cw);
        sc_digit dw = cw | (x_dw & y_dw);
        set_words_(x, i, dw, cw);
    }
    // tail cleaning not needed
    return x;
}

// bitwise or
template <class X, class Y>
inline X &
b_or_assign_(sc_proxy<X> &px, const sc_proxy<Y> &py)
{
    X &x = px.back_cast();
    const Y &y = py.back_cast();
    sc_assert(x.length() == y.length());
    int sz = x.size();
    for (int i = 0; i < sz; ++i) {
        sc_digit x_dw, x_cw, y_dw, y_cw;
        get_words_(x, i, x_dw, x_cw);
        get_words_(y, i, y_dw, y_cw);
        sc_digit cw = (x_cw & y_cw) | (x_cw & ~y_dw) | (~x_dw & y_cw);
        sc_digit dw = cw | x_dw | y_dw;
        set_words_(x, i, dw, cw);
    }
    // tail cleaning not needed
    return x;
}

// bitwise xor
template <class X, class Y>
inline X &
b_xor_assign_(sc_proxy<X> &a, const sc_proxy<Y> &b)
{
    X &x = a.back_cast();
    const Y &y = b.back_cast();
    sc_assert(x.length() == y.length());
    int sz = x.size();
    for (int i = 0; i < sz; ++i) {
        sc_digit x_dw, x_cw, y_dw, y_cw;
        get_words_(x, i, x_dw, x_cw);
        get_words_(y, i, y_dw, y_cw);
        sc_digit cw = x_cw | y_cw;
        sc_digit dw = cw | (x_dw ^ y_dw);
        set_words_( x, i, dw, cw );
    }
    // tail cleaning not needed
    return x;
}

// bitwise left shift
template <class X>
inline X &
sc_proxy<X>::operator <<= (int n)
{
    X &x = back_cast();
    if (n < 0) {
        sc_proxy_out_of_bounds("left shift operation is only allowed with "
                               "positive shift values, shift value = ", n);
        return x;
    }
    if (n >= x.length()) {
        extend_sign_w_(x, 0, false);
        // no tail cleaning needed
        return x;
    }
    int sz = x.size();
    int wn = n / SC_DIGIT_SIZE;
    int bn = n % SC_DIGIT_SIZE;
    if (wn != 0) {
        // shift words
        int i = sz - 1;
        for (; i >= wn; --i) {
            set_words_(x, i, x.get_word(i - wn), x.get_cword(i - wn));
        }
        for (; i >= 0; --i) {
            set_words_(x, i, SC_DIGIT_ZERO, SC_DIGIT_ZERO);
        }
    }
    if (bn != 0) {
        // shift bits
        for (int i = sz - 1; i >= 1; --i) {
            sc_digit x_dw, x_cw;
            get_words_(x, i, x_dw, x_cw);
            x_dw <<= bn;
            x_dw |= x.get_word(i - 1) >> (SC_DIGIT_SIZE - bn);
            x_cw <<= bn;
            x_cw |= x.get_cword(i - 1) >> (SC_DIGIT_SIZE - bn);
            set_words_(x, i, x_dw, x_cw);
        }
        sc_digit x_dw, x_cw;
        get_words_(x, 0, x_dw, x_cw);
        x_dw <<= bn;
        x_cw <<= bn;
        set_words_(x, 0, x_dw, x_cw);
    }
    x.clean_tail();
    return x;
}

// bitwise right shift
template <class X>
inline X &
sc_proxy<X>::operator >>= (int n)
{
    X &x = back_cast();
    if (n < 0) {
        sc_proxy_out_of_bounds("right shift operation is only allowed with "
                               "positive shift values, shift value = ", n);
        return x;
    }
    if (n >= x.length()) {
        extend_sign_w_(x, 0, false);
        // no tail cleaning needed
        return x;
    }
    int sz = x.size();
    int wn = n / SC_DIGIT_SIZE;
    int bn = n % SC_DIGIT_SIZE;
    if (wn != 0) {
        // shift words
        int i = 0;
        for (; i < (sz - wn); ++i) {
            set_words_(x, i, x.get_word(i + wn), x.get_cword(i + wn));
        }
        for (; i < sz; ++i) {
            set_words_(x, i, SC_DIGIT_ZERO, SC_DIGIT_ZERO);
        }
    }
    if (bn != 0) {
        // shift bits
        for (int i = 0; i < (sz - 1); ++i) {
            sc_digit x_dw, x_cw;
            get_words_(x, i, x_dw, x_cw);
            x_dw >>= bn;
            x_dw |= x.get_word(i + 1) << (SC_DIGIT_SIZE - bn);
            x_cw >>= bn;
            x_cw |= x.get_cword(i + 1) << (SC_DIGIT_SIZE - bn);
            set_words_(x, i, x_dw, x_cw);
        }
        sc_digit x_dw, x_cw;
        get_words_(x, sz - 1, x_dw, x_cw);
        x_dw >>= bn;
        x_cw >>= bn;
        set_words_(x, sz - 1, x_dw, x_cw);
    }
    x.clean_tail();
    return x;
}

// bitwise left rotate
template <class X>
inline const sc_lv_base lrotate(const sc_proxy<X> &x, int n);

// bitwise right rotate
template <class X>
inline const sc_lv_base rrotate(const sc_proxy<X>& x, int n);

// bitwise reverse
template <class X>
inline X &
sc_proxy<X>::reverse()
{
    X &x = back_cast();
    int len = x.length();
    int half_len = len / 2;
    for (int i = 0, j = len - 1; i < half_len; ++ i, --j) {
        value_type t = x.get_bit(i);
        x.set_bit(i, x.get_bit(j));
        x.set_bit(j, t);
    }
    return x;
}

template <class X>
inline const sc_lv_base reverse(const sc_proxy<X> &a);

// reduce functions
template <class X>
inline typename sc_proxy<X>::value_type
sc_proxy<X>::and_reduce() const
{
    const X &x = back_cast();
    value_type result = value_type(1);
    int len = x.length();
    for (int i = 0; i < len; ++i) {
        result = sc_logic::and_table[result][x.get_bit(i)];
    }
    return result;
}

template <class X>
inline typename sc_proxy<X>::value_type
sc_proxy<X>::or_reduce() const
{
    const X &x = back_cast();
    value_type result = value_type(0);
    int len = x.length();
    for (int i = 0; i < len; ++i) {
        result = sc_logic::or_table[result][x.get_bit(i)];
    }
    return result;
}

template <class X>
inline typename sc_proxy<X>::value_type
sc_proxy<X>::xor_reduce() const
{
    const X &x = back_cast();
    value_type result = value_type(0);
    int len = x.length();
    for (int i = 0; i < len; ++i) {
        result = sc_logic::xor_table[result][x.get_bit(i)];
    }
    return result;
}

// relational operators
template <class X, class Y>
inline bool
operator != (const sc_proxy<X> &px, const sc_proxy<Y> &py)
{
    return !(px == py);
}


#define DEFN_REL_OP_T(tp) \
template <class X> \
inline bool operator == (tp b, const sc_proxy<X> &px) { return (px == b); } \
 \
template <class X> \
inline bool operator != (const sc_proxy<X> &px, tp b) { return !(px == b); } \
 \
template <class X> \
inline bool operator != (tp b, const sc_proxy<X> &px) { return !(px == b); }

DEFN_REL_OP_T(const char *)
DEFN_REL_OP_T(const bool *)
DEFN_REL_OP_T(const sc_logic *)
DEFN_REL_OP_T(const sc_unsigned &)
DEFN_REL_OP_T(const sc_signed &)
DEFN_REL_OP_T(const sc_uint_base &)
DEFN_REL_OP_T(const sc_int_base &)
DEFN_REL_OP_T(unsigned long)
DEFN_REL_OP_T(long)
DEFN_REL_OP_T(unsigned int)
DEFN_REL_OP_T(int)
DEFN_REL_OP_T(uint64)
DEFN_REL_OP_T(int64)

#undef DEFN_REL_OP_T

// explicit conversions to character string
template <class X>
inline const std::string
sc_proxy<X>::to_string() const
{
    const X &x = back_cast();
    int len = x.length();
    std::string s; // (len + 1);
    for (int i = 0; i < len; ++i) {
        s += sc_logic::logic_to_char[x.get_bit(len - i - 1)];
    }
    return s;
}

template <class X>
inline const std::string
sc_proxy<X>::to_string(sc_numrep numrep) const
{
    return convert_to_fmt(to_string(), numrep, true);
}

template <class X>
inline const std::string
sc_proxy<X>::to_string(sc_numrep numrep, bool w_prefix) const
{
    return convert_to_fmt(to_string(), numrep, w_prefix);
}

// other methods
template <class X>
inline void
sc_proxy<X>::scan(::std::istream &is)
{
    std::string s;
    is >> s;
    back_cast() = s.c_str();
}

template <class X>
inline void
sc_proxy<X>::check_bounds(int n) const // check if bit n accessible
{
    if (n < 0 || n >= back_cast().length()) {
        sc_proxy_out_of_bounds(NULL, n);
        sc_core::sc_abort(); // can't recover from here
    }
}

template <class X>
inline void
sc_proxy<X>::check_wbounds(int n) const // check if word n accessible
{
    if (n < 0 || n >= back_cast().size()) {
        sc_proxy_out_of_bounds(NULL, n);
        sc_core::sc_abort(); // can't recover from here
    }
}

template <class X>
inline sc_digit
sc_proxy<X>::to_anything_unsigned() const
{
    // only 0 word is returned
    // can't convert logic values other than 0 and 1
    const X &x = back_cast();
    int len = x.length();
    if (x.get_cword(0) != SC_DIGIT_ZERO) {
        SC_REPORT_WARNING(sc_core::SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, 0);
    }
    sc_digit w = x.get_word(0);
    if (len >= SC_DIGIT_SIZE) {
        return w;
    }
    return (w & (~SC_DIGIT_ZERO >> (SC_DIGIT_SIZE - len)));
}

template <class X>
inline uint64
sc_proxy<X>::to_uint64() const
{
    // words 1 and 0 returned.
    // can't convert logic values other than 0 and 1
    const X &x = back_cast();
    int len = x.length();
    if (x.get_cword(0) != SC_DIGIT_ZERO) {
        SC_REPORT_WARNING(sc_core::SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, 0);
    }
    uint64 w = x.get_word(0);
    if (len > SC_DIGIT_SIZE) {
        if (x.get_cword(1) != SC_DIGIT_ZERO) {
            SC_REPORT_WARNING(sc_core::SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, 0);
        }
        uint64 w1 = x.get_word(1);
        w = w | (w1 << SC_DIGIT_SIZE);
        return w;
    } else if (len == SC_DIGIT_SIZE) {
        return w;
    } else {
        return (w & (~SC_DIGIT_ZERO >> (SC_DIGIT_SIZE - len)));
    }
}

template <class X>
inline int64
sc_proxy<X>::to_anything_signed() const
{
    const X &x = back_cast();
    int len = x.length();
    int64 w = 0;

    if (len > SC_DIGIT_SIZE) {
        if (x.get_cword(1) != SC_DIGIT_ZERO)
            SC_REPORT_WARNING(sc_core::SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, 0);
        w = x.get_word(1);
    }
    if (x.get_cword(0) != SC_DIGIT_ZERO)
        SC_REPORT_WARNING(sc_core::SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, 0);
    w = (w << SC_DIGIT_SIZE) | x.get_word(0);
    if (len >= 64) {
        return w;
    }

    uint64 zero = 0;
    value_type sgn = x.get_bit(len - 1);
    if (sgn == 0) {
        return (int64)(w & (~zero >> (64 - len)));
    } else {
        return (int64)(w | (~zero << len));
    }
}


// ----------------------------------------------------------------------------

// functional notation for the reduce methods
template <class X>
inline typename sc_proxy<X>::value_type
and_reduce(const sc_proxy<X> &a)
{
    return a.and_reduce();
}

template <class X>
inline typename sc_proxy<X>::value_type
nand_reduce(const sc_proxy<X> &a)
{
    return a.nand_reduce();
}

template <class X>
inline typename sc_proxy<X>::value_type
or_reduce(const sc_proxy<X> &a)
{
    return a.or_reduce();
}

template <class X>
inline typename sc_proxy<X>::value_type
nor_reduce(const sc_proxy<X> &a)
{
    return a.nor_reduce();
}

template <class X>
inline typename sc_proxy<X>::value_type
xor_reduce(const sc_proxy<X> &a)
{
    return a.xor_reduce();
}

template <class X>
inline typename sc_proxy<X>::value_type
xnor_reduce(const sc_proxy<X> &a)
{
    return a.xnor_reduce();
}

// ----------------------------------------------------------------------------

template <class X>
inline ::std::ostream &
operator << (::std::ostream &os, const sc_proxy<X> &a)
{
    a.print(os);
    return os;
}

template <class X>
inline ::std::istream &
operator >> (::std::istream &is, sc_proxy<X> &a)
{
    a.scan(is);
    return is;
}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_BIT_SC_PROXY_HH__

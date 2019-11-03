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

  sc_fxnum.h -

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_fxnum.h,v $
// Revision 1.5  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.4  2011/08/24 22:05:43  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.3  2011/01/19 18:57:40  acg
//  Andy Goodrich: changes for IEEE_1666_2011.
//
// Revision 1.2  2009/03/09 17:26:46  acg
//  Andy Goodrich: removed ; from namespace { }
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_FX_SC_FXNUM_HH__
#define __SYSTEMC_EXT_DT_FX_SC_FXNUM_HH__

#include <iostream>

#include "../bit/sc_lv_base.hh"
#include "messages.hh"
#include "sc_fxnum_observer.hh"
#include "sc_fxval.hh"
#include "scfx_params.hh"

namespace sc_gem5
{

template <typename T, typename B>
class TraceValFxnumBase;

} // namespace sc_core


namespace sc_dt
{

// classes defined in this module
class sc_fxnum_bitref;
class sc_fxnum_fast_bitref;
class sc_fxnum_subref;
class sc_fxnum_fast_subref;
class sc_fxnum;
class sc_fxnum_fast;


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_bitref
//
// Proxy class for bit-selection in class sc_fxnum, behaves like sc_bit.
// ----------------------------------------------------------------------------

class sc_fxnum_bitref
{
    friend class sc_fxnum;
    friend class sc_fxnum_fast_bitref;

    bool get() const;
    void set(bool);

    // constructor
    sc_fxnum_bitref(sc_fxnum &, int);

  public:
    // copy constructor
    sc_fxnum_bitref(const sc_fxnum_bitref &);

    // assignment operators
#define DECL_ASN_OP_T(op, tp) \
    sc_fxnum_bitref &operator op (tp);

#define DECL_ASN_OP(op) \
    DECL_ASN_OP_T(op, const sc_fxnum_bitref &) \
    DECL_ASN_OP_T(op, const sc_fxnum_fast_bitref &) \
    DECL_ASN_OP_T(op, const sc_bit &) \
    DECL_ASN_OP_T(op, bool)

    DECL_ASN_OP(=)

    DECL_ASN_OP(&=)
    DECL_ASN_OP(|=)
    DECL_ASN_OP(^=)

#undef DECL_ASN_OP_T
#undef DECL_ASN_OP

    // implicit conversion
    operator bool() const;

    // print or dump content
    void print(::std::ostream & =::std::cout) const;
    void scan(::std::istream & =::std::cin);
    void dump(::std::ostream & =::std::cout) const;

  private:
    sc_fxnum &m_num;
    int m_idx;

  private:
    // disabled
    sc_fxnum_bitref();
};


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_fast_bitref
//
// Proxy class for bit-selection in class sc_fxnum_fast, behaves like sc_bit.
// ----------------------------------------------------------------------------

class sc_fxnum_fast_bitref
{
    friend class sc_fxnum_fast;
    friend class sc_fxnum_bitref;

    bool get() const;
    void set(bool);

    // constructor
    sc_fxnum_fast_bitref(sc_fxnum_fast &, int);

  public:
    // copy constructor
    sc_fxnum_fast_bitref(const sc_fxnum_fast_bitref &);

    // assignment operators
#define DECL_ASN_OP_T(op, tp) sc_fxnum_fast_bitref &operator op (tp);

#define DECL_ASN_OP(op) \
    DECL_ASN_OP_T(op, const sc_fxnum_bitref &) \
    DECL_ASN_OP_T(op, const sc_fxnum_fast_bitref &) \
    DECL_ASN_OP_T(op, const sc_bit &) \
    DECL_ASN_OP_T(op, bool)

    DECL_ASN_OP(=)

    DECL_ASN_OP(&=)
    DECL_ASN_OP(|=)
    DECL_ASN_OP(^=)

#undef DECL_ASN_OP_T
#undef DECL_ASN_OP

    // implicit conversion
    operator bool() const;

    // print or dump content
    void print(::std::ostream & =::std::cout) const;
    void scan(::std::istream & =::std::cin);
    void dump(::std::ostream & =::std::cout) const;

  private:
    sc_fxnum_fast &m_num;
    int m_idx;

  private:
    // Disabled
    sc_fxnum_fast_bitref();
};


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_subref
//
// Proxy class for part-selection in class sc_fxnum,
// behaves like sc_bv_base.
// ----------------------------------------------------------------------------

class sc_fxnum_subref
{
    friend class sc_fxnum;
    friend class sc_fxnum_fast_subref;

    bool get() const;
    bool set();

    // constructor
    sc_fxnum_subref(sc_fxnum &, int, int);

  public:
    // copy constructor
    sc_fxnum_subref(const sc_fxnum_subref &);

    // destructor
    ~sc_fxnum_subref();

    // assignment operators
#define DECL_ASN_OP_T(tp) \
    sc_fxnum_subref &operator = (tp);

    DECL_ASN_OP_T(const sc_fxnum_subref &)
    DECL_ASN_OP_T(const sc_fxnum_fast_subref &)
    DECL_ASN_OP_T(const sc_bv_base &)
    DECL_ASN_OP_T(const sc_lv_base &)
    DECL_ASN_OP_T(const char *)
    DECL_ASN_OP_T(const bool *)
    DECL_ASN_OP_T(const sc_signed &)
    DECL_ASN_OP_T(const sc_unsigned &)
    DECL_ASN_OP_T(const sc_int_base &)
    DECL_ASN_OP_T(const sc_uint_base &)
    DECL_ASN_OP_T(int64)
    DECL_ASN_OP_T(uint64)
    DECL_ASN_OP_T(int)
    DECL_ASN_OP_T(unsigned int)
    DECL_ASN_OP_T(long)
    DECL_ASN_OP_T(unsigned long)
    DECL_ASN_OP_T(char)

#undef DECL_ASN_OP_T

#define DECL_ASN_OP_T_A(op, tp) \
    sc_fxnum_subref &operator op ## = (tp);

#define DECL_ASN_OP_A(op) \
    DECL_ASN_OP_T_A(op, const sc_fxnum_subref &) \
    DECL_ASN_OP_T_A(op, const sc_fxnum_fast_subref &) \
    DECL_ASN_OP_T_A(op, const sc_bv_base &) \
    DECL_ASN_OP_T_A(op, const sc_lv_base &)

    DECL_ASN_OP_A( &)
    DECL_ASN_OP_A(|)
    DECL_ASN_OP_A(^)

#undef DECL_ASN_OP_T_A
#undef DECL_ASN_OP_A

    // relational operators
#define DECL_REL_OP_T(op, tp) \
    friend bool operator op (const sc_fxnum_subref &, tp); \
    friend bool operator op (tp, const sc_fxnum_subref &);

#define DECL_REL_OP(op) \
    friend bool operator op (const sc_fxnum_subref &, \
                             const sc_fxnum_subref &); \
    friend bool operator op (const sc_fxnum_subref &, \
                             const sc_fxnum_fast_subref &); \
    DECL_REL_OP_T(op, const sc_bv_base &) \
    DECL_REL_OP_T(op, const sc_lv_base &) \
    DECL_REL_OP_T(op, const char *) \
    DECL_REL_OP_T(op, const bool *) \
    DECL_REL_OP_T(op, const sc_signed &) \
    DECL_REL_OP_T(op, const sc_unsigned &) \
    DECL_REL_OP_T(op, int) \
    DECL_REL_OP_T(op, unsigned int) \
    DECL_REL_OP_T(op, long) \
    DECL_REL_OP_T(op, unsigned long)

    DECL_REL_OP(==)
    DECL_REL_OP(!=)

#undef DECL_REL_OP_T
#undef DECL_REL_OP

    // reduce functions
    bool and_reduce() const;
    bool nand_reduce() const;
    bool or_reduce() const;
    bool nor_reduce() const;
    bool xor_reduce() const;
    bool xnor_reduce() const;

    // query parameter
    int length() const;

    // explicit conversions
    int to_int() const;
    unsigned int to_uint() const;
    long to_long() const;
    unsigned long to_ulong() const;
    int64 to_int64() const;
    uint64 to_uint64() const;

    const std::string to_string() const;
    const std::string to_string(sc_numrep) const;
    const std::string to_string(sc_numrep, bool) const;

    // implicit conversion
    operator sc_bv_base() const;

    // print or dump content
    void print(::std::ostream & =::std::cout) const;
    void scan(::std::istream & =::std::cin);
    void dump(::std::ostream & =::std::cout) const;

  private:
    sc_fxnum &m_num;
    int m_from;
    int m_to;

    sc_bv_base &m_bv;

  private:
    // Disabled
    sc_fxnum_subref();
};


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_fast_subref
//
// Proxy class for part-selection in class sc_fxnum_fast,
// behaves like sc_bv_base.
// ----------------------------------------------------------------------------

class sc_fxnum_fast_subref
{
    friend class sc_fxnum_fast;
    friend class sc_fxnum_subref;

    bool get() const;
    bool set();

    // constructor
    sc_fxnum_fast_subref(sc_fxnum_fast &, int, int);

  public:
    // copy constructor
    sc_fxnum_fast_subref(const sc_fxnum_fast_subref &);

    // destructor
    ~sc_fxnum_fast_subref();

    // assignment operators
#define DECL_ASN_OP_T(tp) \
    sc_fxnum_fast_subref &operator = (tp);

    DECL_ASN_OP_T(const sc_fxnum_subref &)
    DECL_ASN_OP_T(const sc_fxnum_fast_subref &)
    DECL_ASN_OP_T(const sc_bv_base &)
    DECL_ASN_OP_T(const sc_lv_base &)
    DECL_ASN_OP_T(const char *)
    DECL_ASN_OP_T(const bool *)
    DECL_ASN_OP_T(const sc_signed &)
    DECL_ASN_OP_T(const sc_unsigned &)
    DECL_ASN_OP_T(const sc_int_base &)
    DECL_ASN_OP_T(const sc_uint_base &)
    DECL_ASN_OP_T(int64)
    DECL_ASN_OP_T(uint64)
    DECL_ASN_OP_T(int)
    DECL_ASN_OP_T(unsigned int)
    DECL_ASN_OP_T(long)
    DECL_ASN_OP_T(unsigned long)
    DECL_ASN_OP_T(char)

#undef DECL_ASN_OP_T

#define DECL_ASN_OP_T_A(op, tp) sc_fxnum_fast_subref &operator op ## = (tp);

#define DECL_ASN_OP_A(op) \
    DECL_ASN_OP_T_A(op, const sc_fxnum_subref &) \
    DECL_ASN_OP_T_A(op, const sc_fxnum_fast_subref &) \
    DECL_ASN_OP_T_A(op, const sc_bv_base &) \
    DECL_ASN_OP_T_A(op, const sc_lv_base &)

    DECL_ASN_OP_A(&)
    DECL_ASN_OP_A(|)
    DECL_ASN_OP_A(^)

#undef DECL_ASN_OP_T_A
#undef DECL_ASN_OP_A

    // relational operators
#define DECL_REL_OP_T(op, tp) \
    friend bool operator op (const sc_fxnum_fast_subref &, tp); \
    friend bool operator op (tp, const sc_fxnum_fast_subref &);

#define DECL_REL_OP(op) \
    friend bool operator op (const sc_fxnum_fast_subref &, \
                             const sc_fxnum_fast_subref &); \
    friend bool operator op (const sc_fxnum_fast_subref &, \
                             const sc_fxnum_subref &); \
    DECL_REL_OP_T(op, const sc_bv_base &) \
    DECL_REL_OP_T(op, const sc_lv_base &) \
    DECL_REL_OP_T(op, const char *) \
    DECL_REL_OP_T(op, const bool *) \
    DECL_REL_OP_T(op, const sc_signed &) \
    DECL_REL_OP_T(op, const sc_unsigned &) \
    DECL_REL_OP_T(op, int) \
    DECL_REL_OP_T(op, unsigned int) \
    DECL_REL_OP_T(op, long) \
    DECL_REL_OP_T(op, unsigned long)

    DECL_REL_OP(==)
    DECL_REL_OP(!=)

#undef DECL_REL_OP_T
#undef DECL_REL_OP

    // reduce functions
    bool and_reduce() const;
    bool nand_reduce() const;
    bool or_reduce() const;
    bool nor_reduce() const;
    bool xor_reduce() const;
    bool xnor_reduce() const;

    // query parameter
    int length() const;

    // explicit conversions
    int to_int() const;
    unsigned int to_uint() const;
    long to_long() const;
    unsigned long to_ulong() const;
    int64 to_int64() const;
    uint64 to_uint64() const;

    const std::string to_string() const;
    const std::string to_string(sc_numrep) const;
    const std::string to_string(sc_numrep, bool) const;

    // implicit conversion
    operator sc_bv_base() const;

    // print or dump content
    void print(::std::ostream & =::std::cout) const;
    void scan(::std::istream & =::std::cin);
    void dump(::std::ostream & =::std::cout) const;

  private:
    sc_fxnum_fast &m_num;
    int m_from;
    int m_to;

    sc_bv_base &m_bv;

  private:
    // Disabled
    sc_fxnum_fast_subref();
};


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum
//
// Base class for the fixed-point types; arbitrary precision.
// ----------------------------------------------------------------------------

class sc_fxnum
{
    friend class sc_fxval;

    friend class sc_fxnum_bitref;
    friend class sc_fxnum_subref;
    friend class sc_fxnum_fast_bitref;
    friend class sc_fxnum_fast_subref;

    template <typename T, typename B>
    friend class sc_gem5::TraceValFxnumBase;

  protected:
    sc_fxnum_observer *observer() const;

    void cast();

    // constructors
    sc_fxnum(const sc_fxtype_params &, sc_enc, const sc_fxcast_switch &,
             sc_fxnum_observer *);

#define DECL_CTOR_T(tp) \
    sc_fxnum(tp, const sc_fxtype_params &, sc_enc, const sc_fxcast_switch &, \
             sc_fxnum_observer *);

    DECL_CTOR_T(int)
    DECL_CTOR_T(unsigned int)
    DECL_CTOR_T(long)
    DECL_CTOR_T(unsigned long)
    DECL_CTOR_T(float)
    DECL_CTOR_T(double)
    DECL_CTOR_T(const char *)
    DECL_CTOR_T(const sc_fxval &)
    DECL_CTOR_T(const sc_fxval_fast &)
    DECL_CTOR_T(const sc_fxnum &)
    DECL_CTOR_T(const sc_fxnum_fast &)

    DECL_CTOR_T(int64)
    DECL_CTOR_T(uint64)
    DECL_CTOR_T(const sc_int_base &)
    DECL_CTOR_T(const sc_uint_base &)
    DECL_CTOR_T(const sc_signed &)
    DECL_CTOR_T(const sc_unsigned &)

#undef DECL_CTOR_T

    ~sc_fxnum();

    // internal use only;
    const scfx_rep *get_rep() const;

  public:
    // unary operators
    const sc_fxval operator - () const;
    const sc_fxval operator + () const;

    // unary functions
    friend void neg(sc_fxval &, const sc_fxnum &);
    friend void neg(sc_fxnum &, const sc_fxnum &);

    // binary operators
#define DECL_BIN_OP_T(op, tp) \
    friend const sc_fxval operator op (const sc_fxnum &, tp); \
    friend const sc_fxval operator op (tp, const sc_fxnum &);

#define DECL_BIN_OP_OTHER(op) \
    DECL_BIN_OP_T(op, int64) \
    DECL_BIN_OP_T(op, uint64) \
    DECL_BIN_OP_T(op, const sc_int_base &) \
    DECL_BIN_OP_T(op, const sc_uint_base &) \
    DECL_BIN_OP_T(op, const sc_signed &) \
    DECL_BIN_OP_T(op, const sc_unsigned &)

#define DECL_BIN_OP(op, dummy) \
    friend const sc_fxval operator op (const sc_fxnum &, const sc_fxnum &); \
    DECL_BIN_OP_T(op, int) \
    DECL_BIN_OP_T(op, unsigned int) \
    DECL_BIN_OP_T(op, long) \
    DECL_BIN_OP_T(op, unsigned long) \
    DECL_BIN_OP_T(op, float) \
    DECL_BIN_OP_T(op, double) \
    DECL_BIN_OP_T(op, const char *) \
    DECL_BIN_OP_T(op, const sc_fxval &) \
    DECL_BIN_OP_T(op, const sc_fxval_fast &) \
    DECL_BIN_OP_T(op, const sc_fxnum_fast &) \
    DECL_BIN_OP_OTHER(op)

    DECL_BIN_OP(*, mult)
    DECL_BIN_OP(+, add)
    DECL_BIN_OP(-, sub)
// don't use macros
// DECL_BIN_OP(/, div)
    friend const sc_fxval operator / (const sc_fxnum &, const sc_fxnum &);
    DECL_BIN_OP_T(/, int)
    DECL_BIN_OP_T(/, unsigned int)
    DECL_BIN_OP_T(/, long)
    DECL_BIN_OP_T(/, unsigned long)
    DECL_BIN_OP_T(/, float)
    DECL_BIN_OP_T(/, double)
    DECL_BIN_OP_T(/, const char *)
    DECL_BIN_OP_T(/, const sc_fxval &)
    DECL_BIN_OP_T(/, const sc_fxval_fast &)
    DECL_BIN_OP_T(/, const sc_fxnum_fast &)
// DECL_BIN_OP_OTHER(op)

    DECL_BIN_OP_T(/, int64)
    DECL_BIN_OP_T(/, uint64)
    DECL_BIN_OP_T(/, const sc_int_base &)
    DECL_BIN_OP_T(/, const sc_uint_base &)
    DECL_BIN_OP_T(/, const sc_signed &)
    DECL_BIN_OP_T(/, const sc_unsigned &)

#undef DECL_BIN_OP_T
#undef DECL_BIN_OP_OTHER
#undef DECL_BIN_OP

    friend const sc_fxval operator << (const sc_fxnum &, int);
    friend const sc_fxval operator >> (const sc_fxnum &, int);

    // binary functions
#define DECL_BIN_FNC_T(fnc, tp) \
    friend void fnc (sc_fxval &, const sc_fxnum &, tp); \
    friend void fnc (sc_fxval &, tp, const sc_fxnum &); \
    friend void fnc (sc_fxnum &, const sc_fxnum &, tp); \
    friend void fnc (sc_fxnum &, tp, const sc_fxnum &);

#define DECL_BIN_FNC_OTHER(fnc) \
    DECL_BIN_FNC_T(fnc, int64) \
    DECL_BIN_FNC_T(fnc, uint64) \
    DECL_BIN_FNC_T(fnc, const sc_int_base &) \
    DECL_BIN_FNC_T(fnc, const sc_uint_base &) \
    DECL_BIN_FNC_T(fnc, const sc_signed &) \
    DECL_BIN_FNC_T(fnc, const sc_unsigned &)

#define DECL_BIN_FNC(fnc) \
    friend void fnc (sc_fxval &, const sc_fxnum &, const sc_fxnum &); \
    friend void fnc (sc_fxnum &, const sc_fxnum &, const sc_fxnum &); \
    DECL_BIN_FNC_T(fnc, int) \
    DECL_BIN_FNC_T(fnc, unsigned int) \
    DECL_BIN_FNC_T(fnc, long) \
    DECL_BIN_FNC_T(fnc, unsigned long) \
    DECL_BIN_FNC_T(fnc, float) \
    DECL_BIN_FNC_T(fnc, double) \
    DECL_BIN_FNC_T(fnc, const char *) \
    DECL_BIN_FNC_T(fnc, const sc_fxval &) \
    DECL_BIN_FNC_T(fnc, const sc_fxval_fast &) \
    DECL_BIN_FNC_T(fnc, const sc_fxnum_fast &) \
    DECL_BIN_FNC_OTHER(fnc)

    DECL_BIN_FNC(mult)
    DECL_BIN_FNC(div)
    DECL_BIN_FNC(add)
    DECL_BIN_FNC(sub)

#undef DECL_BIN_FNC_T
#undef DECL_BIN_FNC_OTHER
#undef DECL_BIN_FNC

    friend void lshift(sc_fxval &, const sc_fxnum &, int);
    friend void rshift(sc_fxval &, const sc_fxnum &, int);
    friend void lshift(sc_fxnum &, const sc_fxnum &, int);
    friend void rshift(sc_fxnum &, const sc_fxnum &, int);

    // relational (including equality) operators
#define DECL_REL_OP_T(op, tp) \
    friend bool operator op (const sc_fxnum &, tp); \
    friend bool operator op (tp, const sc_fxnum &);

#define DECL_REL_OP_OTHER(op) \
    DECL_REL_OP_T(op, int64) \
    DECL_REL_OP_T(op, uint64) \
    DECL_REL_OP_T(op, const sc_int_base &) \
    DECL_REL_OP_T(op, const sc_uint_base &) \
    DECL_REL_OP_T(op, const sc_signed &) \
    DECL_REL_OP_T(op, const sc_unsigned &)

#define DECL_REL_OP(op) \
    friend bool operator op (const sc_fxnum &, const sc_fxnum &); \
    DECL_REL_OP_T(op, int) \
    DECL_REL_OP_T(op, unsigned int) \
    DECL_REL_OP_T(op, long) \
    DECL_REL_OP_T(op, unsigned long) \
    DECL_REL_OP_T(op, float) \
    DECL_REL_OP_T(op, double) \
    DECL_REL_OP_T(op, const char *) \
    DECL_REL_OP_T(op, const sc_fxval &) \
    DECL_REL_OP_T(op, const sc_fxval_fast &) \
    DECL_REL_OP_T(op, const sc_fxnum_fast &) \
    DECL_REL_OP_OTHER(op)

    DECL_REL_OP(<)
    DECL_REL_OP(<=)
    DECL_REL_OP(>)
    DECL_REL_OP(>=)
    DECL_REL_OP(==)
    DECL_REL_OP(!=)

#undef DECL_REL_OP_T
#undef DECL_REL_OP_OTHER
#undef DECL_REL_OP

    // assignment operators
#define DECL_ASN_OP_T(op, tp) \
    sc_fxnum &operator op(tp);

#define DECL_ASN_OP_OTHER(op) \
    DECL_ASN_OP_T(op, int64) \
    DECL_ASN_OP_T(op, uint64) \
    DECL_ASN_OP_T(op, const sc_int_base &) \
    DECL_ASN_OP_T(op, const sc_uint_base &) \
    DECL_ASN_OP_T(op, const sc_signed &) \
    DECL_ASN_OP_T(op, const sc_unsigned &)

#define DECL_ASN_OP(op) \
    DECL_ASN_OP_T(op, int) \
    DECL_ASN_OP_T(op, unsigned int) \
    DECL_ASN_OP_T(op, long) \
    DECL_ASN_OP_T(op, unsigned long) \
    DECL_ASN_OP_T(op, float) \
    DECL_ASN_OP_T(op, double) \
    DECL_ASN_OP_T(op, const char *) \
    DECL_ASN_OP_T(op, const sc_fxval &) \
    DECL_ASN_OP_T(op, const sc_fxval_fast &) \
    DECL_ASN_OP_T(op, const sc_fxnum &) \
    DECL_ASN_OP_T(op, const sc_fxnum_fast &) \
    DECL_ASN_OP_OTHER(op)

    DECL_ASN_OP(=)

    DECL_ASN_OP(*=)
    DECL_ASN_OP(/=)
    DECL_ASN_OP(+=)
    DECL_ASN_OP(-=)

    DECL_ASN_OP_T(<<=, int)
    DECL_ASN_OP_T(>>=, int)

#undef DECL_ASN_OP_T
#undef DECL_ASN_OP_OTHER
#undef DECL_ASN_OP

    // auto-increment and auto-decrement
    const sc_fxval operator ++ (int);
    const sc_fxval operator -- (int);

    sc_fxnum &operator ++ ();
    sc_fxnum &operator -- ();

    // bit selection
    const sc_fxnum_bitref operator [] (int) const;
    sc_fxnum_bitref operator [] (int);

    const sc_fxnum_bitref bit(int) const;
    sc_fxnum_bitref bit(int);

    // part selection
    const sc_fxnum_subref operator () (int, int) const;
    sc_fxnum_subref operator () (int, int);

    const sc_fxnum_subref range(int, int) const;
    sc_fxnum_subref range(int, int);

    const sc_fxnum_subref operator () () const;
    sc_fxnum_subref operator () ();

    const sc_fxnum_subref range() const;
    sc_fxnum_subref range();

    // implicit conversion
    operator double() const; // necessary evil!

    // explicit conversion to primitive types
    short to_short() const;
    unsigned short to_ushort() const;
    int to_int() const;
    unsigned int to_uint() const;
    long to_long() const;
    unsigned long to_ulong() const;
    int64 to_int64() const;
    uint64 to_uint64() const;
    float to_float() const;
    double to_double() const;

    // explicit conversion to character string
    const std::string to_string() const;
    const std::string to_string(sc_numrep) const;
    const std::string to_string(sc_numrep, bool) const;
    const std::string to_string(sc_fmt) const;
    const std::string to_string(sc_numrep, sc_fmt) const;
    const std::string to_string(sc_numrep, bool, sc_fmt) const;

    const std::string to_dec() const;
    const std::string to_bin() const;
    const std::string to_oct() const;
    const std::string to_hex() const;

    // query value
    bool is_neg() const;
    bool is_zero() const;

    // internal use only;
    bool is_normal() const;

    bool quantization_flag() const;
    bool overflow_flag() const;

    const sc_fxval value() const;

    // query parameters
    int wl() const;
    int iwl() const;
    sc_q_mode q_mode() const;
    sc_o_mode o_mode() const;
    int n_bits() const;

    const sc_fxtype_params &type_params() const;

    const sc_fxcast_switch &cast_switch() const;

    // print or dump content
    void print(::std::ostream & =::std::cout) const;
    void scan(::std::istream & =::std::cin);
    void dump(::std::ostream & =::std::cout) const;

    // internal use only;
    void observer_read() const;

    // internal use only;
    bool get_bit(int) const;

  protected:
    bool set_bit(int, bool);

    bool get_slice(int, int, sc_bv_base &) const;
    bool set_slice(int, int, const sc_bv_base &);

    sc_fxnum_observer *lock_observer() const;
    void unlock_observer(sc_fxnum_observer *) const;

  private:
    scfx_rep *m_rep;

    scfx_params m_params;
    bool m_q_flag;
    bool m_o_flag;

    mutable sc_fxnum_observer *m_observer;

  private:
    // disabled
    sc_fxnum();
    sc_fxnum(const sc_fxnum &);
};


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_fast
//
// Base class for the fixed-point types; limited precision.
// ----------------------------------------------------------------------------

class sc_fxnum_fast
{
    friend class sc_fxval_fast;

    friend class sc_fxnum_bitref;
    friend class sc_fxnum_subref;
    friend class sc_fxnum_fast_bitref;
    friend class sc_fxnum_fast_subref;

    template <typename T, typename B>
    friend class sc_gem5::TraceValFxnumBase;

  protected:
    sc_fxnum_fast_observer *observer() const;

    void cast();

    // constructors
    sc_fxnum_fast(const sc_fxtype_params &, sc_enc, const sc_fxcast_switch &,
                  sc_fxnum_fast_observer *);

#define DECL_CTOR_T(tp) \
    sc_fxnum_fast(tp, const sc_fxtype_params &, sc_enc, \
                  const sc_fxcast_switch &, sc_fxnum_fast_observer *);

    DECL_CTOR_T(int)
    DECL_CTOR_T(unsigned int)
    DECL_CTOR_T(long)
    DECL_CTOR_T(unsigned long)
    DECL_CTOR_T(float)
    DECL_CTOR_T(double)
    DECL_CTOR_T(const char *)
    DECL_CTOR_T(const sc_fxval &)
    DECL_CTOR_T(const sc_fxval_fast &)
    DECL_CTOR_T(const sc_fxnum &)
    DECL_CTOR_T(const sc_fxnum_fast &)

    DECL_CTOR_T(int64)
    DECL_CTOR_T(uint64)
    DECL_CTOR_T(const sc_int_base &)
    DECL_CTOR_T(const sc_uint_base &)
    DECL_CTOR_T(const sc_signed &)
    DECL_CTOR_T(const sc_unsigned &)

#undef DECL_CTOR_T
    ~sc_fxnum_fast();

    // internal use only;
    double get_val() const;

  public:
    // unary operators
    const sc_fxval_fast operator - () const;
    const sc_fxval_fast operator + () const;

    // unary functions
    friend void neg(sc_fxval_fast &, const sc_fxnum_fast &);
    friend void neg(sc_fxnum_fast &, const sc_fxnum_fast &);


    // binary operators
#define DECL_BIN_OP_T(op, tp) \
    friend const sc_fxval_fast operator op (const sc_fxnum_fast &, tp); \
    friend const sc_fxval_fast operator op (tp, const sc_fxnum_fast &);

#define DECL_BIN_OP_OTHER(op) \
    DECL_BIN_OP_T(op, int64) \
    DECL_BIN_OP_T(op, uint64) \
    DECL_BIN_OP_T(op, const sc_int_base &) \
    DECL_BIN_OP_T(op, const sc_uint_base &) \
    DECL_BIN_OP_T(op, const sc_signed &) \
    DECL_BIN_OP_T(op, const sc_unsigned &)

#define DECL_BIN_OP(op, dummy) \
    friend const sc_fxval_fast operator op (const sc_fxnum_fast &, \
                                            const sc_fxnum_fast &); \
    DECL_BIN_OP_T(op, int) \
    DECL_BIN_OP_T(op, unsigned int) \
    DECL_BIN_OP_T(op, long) \
    DECL_BIN_OP_T(op, unsigned long) \
    DECL_BIN_OP_T(op, float) \
    DECL_BIN_OP_T(op, double) \
    DECL_BIN_OP_T(op, const char *) \
    DECL_BIN_OP_T(op, const sc_fxval_fast &) \
    DECL_BIN_OP_OTHER(op)

    DECL_BIN_OP(*, mult)
    DECL_BIN_OP(+, add)
    DECL_BIN_OP(-, sub)
// DECL_BIN_OP(/, div)
    friend const sc_fxval_fast operator / (const sc_fxnum_fast &,
                                           const sc_fxnum_fast &);
    DECL_BIN_OP_T(/, int)
    DECL_BIN_OP_T(/, unsigned int)
    DECL_BIN_OP_T(/, long)
    DECL_BIN_OP_T(/, unsigned long)
    DECL_BIN_OP_T(/, float)
    DECL_BIN_OP_T(/, double)
    DECL_BIN_OP_T(/, const char *)
    DECL_BIN_OP_T(/, const sc_fxval_fast &)
// DECL_BIN_OP_OTHER(op)

    DECL_BIN_OP_T(/, int64) \
    DECL_BIN_OP_T(/, uint64) \
    DECL_BIN_OP_T(/, const sc_int_base &) \
    DECL_BIN_OP_T(/, const sc_uint_base &) \
    DECL_BIN_OP_T(/, const sc_signed &) \
    DECL_BIN_OP_T(/, const sc_unsigned &)

#undef DECL_BIN_OP_T
#undef DECL_BIN_OP_OTHER
#undef DECL_BIN_OP

    friend const sc_fxval_fast operator << (const sc_fxnum_fast &, int);
    friend const sc_fxval_fast operator >> (const sc_fxnum_fast &, int);

    // binary functions
#define DECL_BIN_FNC_T(fnc, tp) \
    friend void fnc (sc_fxval_fast &, const sc_fxnum_fast &, tp); \
    friend void fnc (sc_fxval_fast &, tp, const sc_fxnum_fast &); \
    friend void fnc (sc_fxnum_fast &, const sc_fxnum_fast &, tp); \
    friend void fnc (sc_fxnum_fast &, tp, const sc_fxnum_fast &);

#define DECL_BIN_FNC_OTHER(fnc) \
    DECL_BIN_FNC_T(fnc, int64) \
    DECL_BIN_FNC_T(fnc, uint64) \
    DECL_BIN_FNC_T(fnc, const sc_int_base &) \
    DECL_BIN_FNC_T(fnc, const sc_uint_base &) \
    DECL_BIN_FNC_T(fnc, const sc_signed &) \
    DECL_BIN_FNC_T(fnc, const sc_unsigned &)

#define DECL_BIN_FNC(fnc) \
    friend void fnc (sc_fxval_fast &, const sc_fxnum_fast &, \
                                      const sc_fxnum_fast &); \
    friend void fnc (sc_fxnum_fast &, const sc_fxnum_fast &, \
                                      const sc_fxnum_fast &); \
    DECL_BIN_FNC_T(fnc, int) \
    DECL_BIN_FNC_T(fnc, unsigned int) \
    DECL_BIN_FNC_T(fnc, long) \
    DECL_BIN_FNC_T(fnc, unsigned long) \
    DECL_BIN_FNC_T(fnc, float) \
    DECL_BIN_FNC_T(fnc, double) \
    DECL_BIN_FNC_T(fnc, const char *) \
    DECL_BIN_FNC_T(fnc, const sc_fxval &) \
    DECL_BIN_FNC_T(fnc, const sc_fxval_fast &) \
    DECL_BIN_FNC_T(fnc, const sc_fxnum &) \
    DECL_BIN_FNC_OTHER(fnc)

    DECL_BIN_FNC(mult)
    DECL_BIN_FNC(div)
    DECL_BIN_FNC(add)
    DECL_BIN_FNC(sub)

#undef DECL_BIN_FNC_T
#undef DECL_BIN_FNC_OTHER
#undef DECL_BIN_FNC

    friend void lshift(sc_fxval_fast &, const sc_fxnum_fast &, int);
    friend void rshift(sc_fxval_fast &, const sc_fxnum_fast &, int);
    friend void lshift(sc_fxnum_fast &, const sc_fxnum_fast &, int);
    friend void rshift(sc_fxnum_fast &, const sc_fxnum_fast &, int);

    // relational (including equality) operators
#define DECL_REL_OP_T(op, tp) \
    friend bool operator op (const sc_fxnum_fast &, tp); \
    friend bool operator op (tp, const sc_fxnum_fast &);

#define DECL_REL_OP_OTHER(op) \
    DECL_REL_OP_T(op, int64) \
    DECL_REL_OP_T(op, uint64) \
    DECL_REL_OP_T(op, const sc_int_base &) \
    DECL_REL_OP_T(op, const sc_uint_base &) \
    DECL_REL_OP_T(op, const sc_signed &) \
    DECL_REL_OP_T(op, const sc_unsigned &)

#define DECL_REL_OP(op) \
    friend bool operator op (const sc_fxnum_fast &, const sc_fxnum_fast &); \
    DECL_REL_OP_T(op, int) \
    DECL_REL_OP_T(op, unsigned int) \
    DECL_REL_OP_T(op, long) \
    DECL_REL_OP_T(op, unsigned long) \
    DECL_REL_OP_T(op, float) \
    DECL_REL_OP_T(op, double) \
    DECL_REL_OP_T(op, const char *) \
    DECL_REL_OP_T(op, const sc_fxval_fast &) \
    DECL_REL_OP_OTHER(op)

    DECL_REL_OP(<)
    DECL_REL_OP(<=)
    DECL_REL_OP(>)
    DECL_REL_OP(>=)
    DECL_REL_OP(==)
    DECL_REL_OP(!=)

#undef DECL_REL_OP_T
#undef DECL_REL_OP_OTHER
#undef DECL_REL_OP

    // assignment operators
#define DECL_ASN_OP_T(op, tp) sc_fxnum_fast &operator op(tp);

#define DECL_ASN_OP_OTHER(op) \
    DECL_ASN_OP_T(op, int64) \
    DECL_ASN_OP_T(op, uint64) \
    DECL_ASN_OP_T(op, const sc_int_base &) \
    DECL_ASN_OP_T(op, const sc_uint_base &) \
    DECL_ASN_OP_T(op, const sc_signed &) \
    DECL_ASN_OP_T(op, const sc_unsigned &)

#define DECL_ASN_OP(op) \
    DECL_ASN_OP_T(op, int) \
    DECL_ASN_OP_T(op, unsigned int) \
    DECL_ASN_OP_T(op, long) \
    DECL_ASN_OP_T(op, unsigned long) \
    DECL_ASN_OP_T(op, float) \
    DECL_ASN_OP_T(op, double) \
    DECL_ASN_OP_T(op, const char *) \
    DECL_ASN_OP_T(op, const sc_fxval &) \
    DECL_ASN_OP_T(op, const sc_fxval_fast &) \
    DECL_ASN_OP_T(op, const sc_fxnum &) \
    DECL_ASN_OP_T(op, const sc_fxnum_fast &) \
    DECL_ASN_OP_OTHER(op)

    DECL_ASN_OP(=)

    DECL_ASN_OP(*=)
    DECL_ASN_OP(/=)
    DECL_ASN_OP(+=)
    DECL_ASN_OP(-=)

    DECL_ASN_OP_T(<<=, int)
    DECL_ASN_OP_T(>>=, int)

#undef DECL_ASN_OP_T
#undef DECL_ASN_OP_OTHER
#undef DECL_ASN_OP

    // auto-increment and auto-decrement
    const sc_fxval_fast operator ++ (int);
    const sc_fxval_fast operator -- (int);

    sc_fxnum_fast &operator ++ ();
    sc_fxnum_fast &operator -- ();

    // bit selection
    const sc_fxnum_fast_bitref operator [] (int) const;
    sc_fxnum_fast_bitref operator [] (int);

    const sc_fxnum_fast_bitref bit(int) const;
    sc_fxnum_fast_bitref bit(int);

    // part selection
    const sc_fxnum_fast_subref operator () (int, int) const;
    sc_fxnum_fast_subref operator () (int, int);

    const sc_fxnum_fast_subref range(int, int) const;
    sc_fxnum_fast_subref range(int, int);


    const sc_fxnum_fast_subref operator () () const;
    sc_fxnum_fast_subref operator () ();

    const sc_fxnum_fast_subref range() const;
    sc_fxnum_fast_subref range();

    // implicit conversion
    operator double() const; // necessary evil!

    // explicit conversion to primitive types
    short to_short() const;
    unsigned short to_ushort() const;
    int to_int() const;
    unsigned int to_uint() const;
    long to_long() const;
    unsigned long to_ulong() const;
    int64 to_int64() const;
    uint64 to_uint64() const;
    float to_float() const;
    double to_double() const;

    // explicit conversion to character string
    const std::string to_string() const;
    const std::string to_string(sc_numrep) const;
    const std::string to_string(sc_numrep, bool) const;
    const std::string to_string(sc_fmt) const;
    const std::string to_string(sc_numrep, sc_fmt) const;
    const std::string to_string(sc_numrep, bool, sc_fmt) const;

    const std::string to_dec() const;
    const std::string to_bin() const;
    const std::string to_oct() const;
    const std::string to_hex() const;

    // query value
    bool is_neg() const;
    bool is_zero() const;

    // internal use only;
    bool is_normal() const;

    bool quantization_flag() const;
    bool overflow_flag() const;

    const sc_fxval_fast value() const;

    // query parameters
    int wl() const;
    int iwl() const;
    sc_q_mode q_mode() const;
    sc_o_mode o_mode() const;
    int n_bits() const;

    const sc_fxtype_params &type_params() const;

    const sc_fxcast_switch &cast_switch() const;

    // print or dump content
    void print(::std::ostream & =::std::cout) const;
    void scan(::std::istream & =::std::cin);
    void dump(::std::ostream & =::std::cout) const;

    // internal use only;
    void observer_read() const;

    // internal use only;
    bool get_bit(int) const;

  protected:
    bool set_bit(int, bool);

    bool get_slice(int, int, sc_bv_base &) const;
    bool set_slice(int, int, const sc_bv_base &);

    sc_fxnum_fast_observer *lock_observer() const;
    void unlock_observer(sc_fxnum_fast_observer *) const;

  private:
    double m_val;

    scfx_params m_params;
    bool m_q_flag;
    bool m_o_flag;

    mutable sc_fxnum_fast_observer *m_observer;

  private:
    // Disabled
    sc_fxnum_fast();
    sc_fxnum_fast(const sc_fxnum_fast &);
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_bitref
//
// Proxy class for bit-selection in class sc_fxnum, behaves like sc_bit.
// ----------------------------------------------------------------------------

// constructor

inline
sc_fxnum_bitref::sc_fxnum_bitref(sc_fxnum &num_, int idx_) :
        m_num(num_), m_idx(idx_)
{}

// copy constructor
inline sc_fxnum_bitref::sc_fxnum_bitref(const sc_fxnum_bitref &a) :
        m_num(a.m_num), m_idx(a.m_idx)
{}

// assignment operators
inline sc_fxnum_bitref &
sc_fxnum_bitref::operator = (const sc_fxnum_bitref &a)
{
    if (&a != this) {
        SC_FXNUM_OBSERVER_READ_(a.m_num)
        set(a.get());
        SC_FXNUM_OBSERVER_WRITE_(m_num)
    }
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator = (const sc_fxnum_fast_bitref &a)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a.m_num)
    set(a.get());
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator = (const sc_bit &a)
{
    set(static_cast<bool>(a));
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator = (bool a)
{
    set(a);
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator &= (const sc_fxnum_bitref &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    SC_FXNUM_OBSERVER_READ_(b.m_num)
    set(get() && b.get());
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator &= (const sc_fxnum_fast_bitref &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    SC_FXNUM_FAST_OBSERVER_READ_(b.m_num)
    set(get() && b.get());
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator &= (const sc_bit &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    set(get() && static_cast<bool>(b));
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator &= (bool b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    set(get() && b);
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}


inline sc_fxnum_bitref &
sc_fxnum_bitref::operator |= (const sc_fxnum_bitref &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    SC_FXNUM_OBSERVER_READ_(b.m_num)
    set(get() || b.get());
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator |= (const sc_fxnum_fast_bitref &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    SC_FXNUM_FAST_OBSERVER_READ_(b.m_num)
    set(get() || b.get());
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator |= (const sc_bit &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    set(get() || static_cast<bool>(b));
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator |= (bool b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    set(get() || b);
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}


inline sc_fxnum_bitref &
sc_fxnum_bitref::operator ^= (const sc_fxnum_bitref &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    SC_FXNUM_OBSERVER_READ_(b.m_num)
    set(get() != b.get());
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator ^= (const sc_fxnum_fast_bitref &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    SC_FXNUM_FAST_OBSERVER_READ_(b.m_num)
    set(get() != b.get());
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator ^= (const sc_bit &b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    set(get() != static_cast<bool>(b));
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_bitref &
sc_fxnum_bitref::operator ^= (bool b)
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    set(get() != b);
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

// implicit conversion
inline sc_fxnum_bitref::operator bool() const
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    return get();
}

inline ::std::ostream &
operator << (::std::ostream &os, const sc_fxnum_bitref &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_fxnum_bitref &a)
{
    a.scan(is);
    return is;
}


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_fast_bitref
//
// Proxy class for bit-selection in class sc_fxnum_fast, behaves like sc_bit.
// ----------------------------------------------------------------------------

// constructor
inline sc_fxnum_fast_bitref::sc_fxnum_fast_bitref(
        sc_fxnum_fast &num_, int idx_) : m_num(num_), m_idx(idx_)
{}

// copy constructor
inline sc_fxnum_fast_bitref::sc_fxnum_fast_bitref(
        const sc_fxnum_fast_bitref &a) : m_num(a.m_num), m_idx(a.m_idx)
{}

// assignment operators
inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator = (const sc_fxnum_bitref &a)
{
    SC_FXNUM_OBSERVER_READ_(a.m_num)
    set(a.get());
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator = (const sc_fxnum_fast_bitref &a)
{
    if (&a != this) {
        SC_FXNUM_FAST_OBSERVER_READ_(a.m_num)
        set(a.get());
        SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    }
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator = (const sc_bit &a)
{
    set(static_cast<bool>(a));
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator = (bool a)
{
    set(a);
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}


inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator &= (const sc_fxnum_bitref &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    SC_FXNUM_OBSERVER_READ_(b.m_num)
    set(get() && b.get());
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator &= (const sc_fxnum_fast_bitref &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    SC_FXNUM_FAST_OBSERVER_READ_(b.m_num)
    set(get() && b.get());
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator &= (const sc_bit &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    set(get() && static_cast<bool>(b));
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator &= (bool b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    set(get() && b);
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}


inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator |= (const sc_fxnum_bitref &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    SC_FXNUM_OBSERVER_READ_(b.m_num)
    set(get() || b.get());
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator |= (const sc_fxnum_fast_bitref &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    SC_FXNUM_FAST_OBSERVER_READ_(b.m_num)
    set(get() || b.get());
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator |= (const sc_bit &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    set(get() || static_cast<bool>(b));
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator |= (bool b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    set(get() || b);
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}


inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator ^= (const sc_fxnum_bitref &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    SC_FXNUM_OBSERVER_READ_(b.m_num)
    set(get() != b.get());
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator ^= (const sc_fxnum_fast_bitref &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    SC_FXNUM_FAST_OBSERVER_READ_(b.m_num)
    set(get() != b.get());
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator ^= (const sc_bit &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    set(get() != static_cast<bool>(b));
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_bitref &
sc_fxnum_fast_bitref::operator ^= (bool b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    set(get() != b);
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}


// implicit conversion
inline sc_fxnum_fast_bitref::operator bool() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    return get();
}

inline ::std::ostream &
operator << (::std::ostream &os, const sc_fxnum_fast_bitref &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_fxnum_fast_bitref &a)
{
    a.scan(is);
    return is;
}


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_subref
//
// Proxy class for part-selection in class sc_fxnum,
// behaves like sc_bv_base.
// ----------------------------------------------------------------------------

// constructor
inline sc_fxnum_subref::sc_fxnum_subref(sc_fxnum &num_, int from_, int to_) :
        m_num(num_), m_from(from_), m_to(to_),
        m_bv(*new sc_bv_base(sc_max(m_from, m_to) - sc_min(m_from, m_to) + 1))
{}

// copy constructor
inline sc_fxnum_subref::sc_fxnum_subref(const sc_fxnum_subref &a) :
        m_num(a.m_num), m_from(a.m_from), m_to(a.m_to),
        m_bv(*new sc_bv_base(a.m_bv))
{}

// destructor
inline sc_fxnum_subref::~sc_fxnum_subref()
{
    delete &m_bv;
}

// assignment operators
inline sc_fxnum_subref &
sc_fxnum_subref::operator = (const sc_fxnum_subref &a)
{
    if (&a != this) {
        m_bv = static_cast<sc_bv_base>(a);
        set();
        SC_FXNUM_OBSERVER_WRITE_(m_num)
    }
    return *this;
}

inline sc_fxnum_subref &
sc_fxnum_subref::operator = (const sc_fxnum_fast_subref &a)
{
    m_bv = static_cast<sc_bv_base>(a);
    set();
    SC_FXNUM_OBSERVER_WRITE_(m_num)
    return *this;
}

#define DEFN_ASN_OP_T(tp) \
inline sc_fxnum_subref & \
sc_fxnum_subref::operator = (tp a) \
{ \
    m_bv = a; \
    set(); \
    SC_FXNUM_OBSERVER_WRITE_(m_num) \
    return *this; \
}

DEFN_ASN_OP_T(const sc_bv_base &)
DEFN_ASN_OP_T(const sc_lv_base &)
DEFN_ASN_OP_T(const char *)
DEFN_ASN_OP_T(const bool *)
DEFN_ASN_OP_T(const sc_signed &)
DEFN_ASN_OP_T(const sc_unsigned &)
DEFN_ASN_OP_T(const sc_int_base &)
DEFN_ASN_OP_T(const sc_uint_base &)
DEFN_ASN_OP_T(int64)
DEFN_ASN_OP_T(uint64)
DEFN_ASN_OP_T(int)
DEFN_ASN_OP_T(unsigned int)
DEFN_ASN_OP_T(long)
DEFN_ASN_OP_T(unsigned long)
DEFN_ASN_OP_T(char)

#undef DEFN_ASN_OP_T

#define DEFN_ASN_OP_T(op, tp) \
inline sc_fxnum_subref & \
sc_fxnum_subref::operator op ## = (tp a) \
{ \
    SC_FXNUM_OBSERVER_READ_(m_num) \
    get(); \
    m_bv = m_bv op a; \
    set(); \
    SC_FXNUM_OBSERVER_WRITE_(m_num) \
    return *this; \
}

#define DEFN_ASN_OP(op) \
inline sc_fxnum_subref & \
sc_fxnum_subref::operator op ## = (const sc_fxnum_subref &a) \
{ \
    SC_FXNUM_OBSERVER_READ_(m_num) \
    get(); \
    m_bv = m_bv op static_cast<sc_bv_base>(a); \
    set(); \
    SC_FXNUM_OBSERVER_WRITE_(m_num) \
    return *this; \
} \
 \
inline sc_fxnum_subref & \
sc_fxnum_subref::operator op ## = (const sc_fxnum_fast_subref &a) \
{ \
    SC_FXNUM_OBSERVER_READ_(m_num) \
    get(); \
    m_bv = m_bv op static_cast<sc_bv_base>(a); \
    set(); \
    SC_FXNUM_OBSERVER_WRITE_(m_num) \
    return *this; \
} \
 \
DEFN_ASN_OP_T(op, const sc_bv_base &) \
DEFN_ASN_OP_T(op, const sc_lv_base &)

DEFN_ASN_OP( &)
DEFN_ASN_OP(|)
DEFN_ASN_OP(^)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP

// relational operators
#define DEFN_REL_OP_T(op, tp) \
inline bool \
operator op (const sc_fxnum_subref &a, tp b) \
{ \
    return (static_cast<sc_bv_base>(a) op b); \
} \
 \
inline bool \
operator op (tp a, const sc_fxnum_subref &b) \
{ \
    return (static_cast<sc_bv_base>(b) op a); \
}

#define DEFN_REL_OP(op) \
inline bool \
operator op (const sc_fxnum_subref &a, const sc_fxnum_subref &b) \
{ \
    return (static_cast<sc_bv_base>(a) op static_cast<sc_bv_base>(b)); \
} \
 \
inline bool \
operator op (const sc_fxnum_subref &a, const sc_fxnum_fast_subref &b) \
{ \
    return (static_cast<sc_bv_base>(a) op static_cast<sc_bv_base>(b)); \
} \
 \
DEFN_REL_OP_T(op, const sc_bv_base &) \
DEFN_REL_OP_T(op, const sc_lv_base &) \
DEFN_REL_OP_T(op, const char *) \
DEFN_REL_OP_T(op, const bool *) \
DEFN_REL_OP_T(op, const sc_signed &) \
DEFN_REL_OP_T(op, const sc_unsigned &) \
DEFN_REL_OP_T(op, int) \
DEFN_REL_OP_T(op, unsigned int) \
DEFN_REL_OP_T(op, long) \
DEFN_REL_OP_T(op, unsigned long)

DEFN_REL_OP(==)
DEFN_REL_OP(!=)

#undef DEFN_REL_OP_T
#undef DEFN_REL_OP


// reduce functions

#define DEFN_RED_FNC(fnc) \
inline bool \
sc_fxnum_subref::fnc() const \
{ \
    SC_FXNUM_OBSERVER_READ_(m_num) \
    get(); \
    return static_cast<bool>(m_bv.fnc()); \
}

DEFN_RED_FNC(and_reduce)
DEFN_RED_FNC(nand_reduce)
DEFN_RED_FNC(or_reduce)
DEFN_RED_FNC(nor_reduce)
DEFN_RED_FNC(xor_reduce)
DEFN_RED_FNC(xnor_reduce)

#undef DEFN_RED_FNC

// query parameter
inline int
sc_fxnum_subref::length() const
{
    return m_bv.length();
}

// explicit conversions
inline int
sc_fxnum_subref::to_int() const
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_int();
}

inline int64
sc_fxnum_subref::to_int64() const
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_int64();
}

inline unsigned int
sc_fxnum_subref::to_uint() const
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_uint();
}

inline uint64
sc_fxnum_subref::to_uint64() const
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_uint64();
}

inline long
sc_fxnum_subref::to_long() const
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_long();
}

inline unsigned long
sc_fxnum_subref::to_ulong() const
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_ulong();
}


inline const std::string
sc_fxnum_subref::to_string() const
{
    get();
    return m_bv.to_string();
}

inline const std::string
sc_fxnum_subref::to_string(sc_numrep numrep) const
{
    get();
    return m_bv.to_string(numrep);
}

inline const std::string
sc_fxnum_subref::to_string(sc_numrep numrep, bool w_prefix) const
{
    get();
    return m_bv.to_string(numrep, w_prefix);
}


// implicit conversion
inline sc_fxnum_subref::operator sc_bv_base () const
{
    SC_FXNUM_OBSERVER_READ_(m_num)
    get();
    return m_bv;
}


inline ::std::ostream &
operator << (::std::ostream &os, const sc_fxnum_subref &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_fxnum_subref &a)
{
    a.scan(is);
    return is;
}


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_fast_subref
//
// Proxy class for part-selection in class sc_fxnum_fast,
// behaves like sc_bv_base.
// ----------------------------------------------------------------------------

// constructor

inline sc_fxnum_fast_subref::sc_fxnum_fast_subref(
        sc_fxnum_fast &num_, int from_, int to_) :
    m_num(num_), m_from(from_), m_to(to_),
    m_bv(*new sc_bv_base(sc_max(m_from, m_to) - sc_min(m_from, m_to) + 1))
{}


// copy constructor
inline sc_fxnum_fast_subref::sc_fxnum_fast_subref(
        const sc_fxnum_fast_subref &a) :
    m_num(a.m_num), m_from(a.m_from), m_to(a.m_to),
    m_bv(*new sc_bv_base(a.m_bv))
{}


// destructor
inline sc_fxnum_fast_subref::~sc_fxnum_fast_subref()
{
    delete &m_bv;
}


// assignment operators
inline sc_fxnum_fast_subref &
sc_fxnum_fast_subref::operator = (const sc_fxnum_subref &a)
{
    m_bv = static_cast<sc_bv_base>(a);
    set();
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    return *this;
}

inline sc_fxnum_fast_subref &
sc_fxnum_fast_subref::operator = (const sc_fxnum_fast_subref &a)
{
    if (&a != this) {
        m_bv = static_cast<sc_bv_base>(a);
        set();
        SC_FXNUM_FAST_OBSERVER_WRITE_(m_num)
    }
    return *this;
}

#define DEFN_ASN_OP_T(tp) \
inline sc_fxnum_fast_subref & \
sc_fxnum_fast_subref::operator = (tp a) \
{ \
    m_bv = a; \
    set(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num) \
    return *this; \
}

DEFN_ASN_OP_T(const sc_bv_base &)
DEFN_ASN_OP_T(const sc_lv_base &)
DEFN_ASN_OP_T(const char *)
DEFN_ASN_OP_T(const bool *)
DEFN_ASN_OP_T(const sc_signed &)
DEFN_ASN_OP_T(const sc_unsigned &)
DEFN_ASN_OP_T(const sc_int_base &)
DEFN_ASN_OP_T(const sc_uint_base &)
DEFN_ASN_OP_T(int64)
DEFN_ASN_OP_T(uint64)
DEFN_ASN_OP_T(int)
DEFN_ASN_OP_T(unsigned int)
DEFN_ASN_OP_T(long)
DEFN_ASN_OP_T(unsigned long)
DEFN_ASN_OP_T(char)

#undef DEFN_ASN_OP_T


#define DEFN_ASN_OP_T(op, tp) \
inline sc_fxnum_fast_subref & \
sc_fxnum_fast_subref::operator op ## = (tp a) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(m_num) \
    get(); \
    m_bv = m_bv op a; \
    set(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num) \
    return *this; \
}

#define DEFN_ASN_OP(op) \
inline sc_fxnum_fast_subref & \
sc_fxnum_fast_subref::operator op ## = (const sc_fxnum_subref &a) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(m_num) \
    get(); \
    m_bv = m_bv op static_cast<sc_bv_base>(a); \
    set(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num) \
    return *this; \
} \
 \
inline sc_fxnum_fast_subref & \
sc_fxnum_fast_subref::operator op ## = (const sc_fxnum_fast_subref &a) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(m_num) \
    get(); \
    m_bv = m_bv op static_cast<sc_bv_base>(a); \
    set(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(m_num) \
    return *this; \
} \
 \
DEFN_ASN_OP_T(op, const sc_bv_base &) \
DEFN_ASN_OP_T(op, const sc_lv_base &)

DEFN_ASN_OP(&)
DEFN_ASN_OP(|)
DEFN_ASN_OP(^)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP


// relational operators

#define DEFN_REL_OP_T(op, tp) \
inline bool \
operator op (const sc_fxnum_fast_subref &a, tp b) \
{ \
    return (static_cast<sc_bv_base>(a) op b); \
} \
 \
inline bool \
operator op (tp a, const sc_fxnum_fast_subref &b) \
{ \
    return (static_cast<sc_bv_base>(b) op a); \
}

#define DEFN_REL_OP(op) \
inline bool \
operator op (const sc_fxnum_fast_subref &a, const sc_fxnum_fast_subref &b) \
{ \
    return (static_cast<sc_bv_base>(a) op static_cast<sc_bv_base>(b)); \
} \
 \
inline bool \
operator op (const sc_fxnum_fast_subref &a, const sc_fxnum_subref &b) \
{ \
    return (static_cast<sc_bv_base>(a) op static_cast<sc_bv_base>(b)); \
} \
 \
DEFN_REL_OP_T(op, const sc_bv_base &) \
DEFN_REL_OP_T(op, const sc_lv_base &) \
DEFN_REL_OP_T(op, const char *) \
DEFN_REL_OP_T(op, const bool *) \
DEFN_REL_OP_T(op, const sc_signed &) \
DEFN_REL_OP_T(op, const sc_unsigned &) \
DEFN_REL_OP_T(op, int) \
DEFN_REL_OP_T(op, unsigned int) \
DEFN_REL_OP_T(op, long) \
DEFN_REL_OP_T(op, unsigned long)

DEFN_REL_OP(==)
DEFN_REL_OP(!=)

#undef DEFN_REL_OP_T
#undef DEFN_REL_OP

// reduce functions
#define DEFN_RED_FNC(fnc) \
inline bool \
sc_fxnum_fast_subref::fnc() const \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(m_num) \
    get(); \
    return static_cast<bool>(m_bv.fnc()); \
}

DEFN_RED_FNC(and_reduce)
DEFN_RED_FNC(nand_reduce)
DEFN_RED_FNC(or_reduce)
DEFN_RED_FNC(nor_reduce)
DEFN_RED_FNC(xor_reduce)
DEFN_RED_FNC(xnor_reduce)

#undef DEFN_RED_FNC

// query parameter
inline int
sc_fxnum_fast_subref::length() const
{
    return m_bv.length();
}

// explicit conversions
inline int
sc_fxnum_fast_subref::to_int() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_int();
}

inline int64
sc_fxnum_fast_subref::to_int64() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_int64();
}

inline unsigned int
sc_fxnum_fast_subref::to_uint() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_uint();
}

inline uint64
sc_fxnum_fast_subref::to_uint64() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_uint64();
}

inline long
sc_fxnum_fast_subref::to_long() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_long();
}

inline unsigned long
sc_fxnum_fast_subref::to_ulong() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    get();
    return m_bv.to_ulong();
}

inline const std::string
sc_fxnum_fast_subref::to_string() const
{
    get();
    return m_bv.to_string();
}

inline const std::string
sc_fxnum_fast_subref::to_string(sc_numrep numrep) const
{
    get();
    return m_bv.to_string(numrep);
}

inline const std::string
sc_fxnum_fast_subref::to_string(sc_numrep numrep, bool w_prefix) const
{
    get();
    return m_bv.to_string(numrep, w_prefix);
}


// implicit conversion
inline sc_fxnum_fast_subref::operator sc_bv_base () const
{
    SC_FXNUM_FAST_OBSERVER_READ_(m_num)
    get();
    return m_bv;
}

inline ::std::ostream &
operator << (::std::ostream &os, const sc_fxnum_fast_subref &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_fxnum_fast_subref &a)
{
    a.scan(is);
    return is;
}


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum
//
// Base class for the fixed-point types; arbitrary precision.
// ----------------------------------------------------------------------------

inline sc_fxnum_observer *
sc_fxnum::observer() const
{
    return m_observer;
}

inline void
sc_fxnum::cast()
{
    SC_ERROR_IF_(!m_rep->is_normal(), sc_core::SC_ID_INVALID_FX_VALUE_);

    if (m_params.cast_switch() == SC_ON)
        m_rep->cast(m_params, m_q_flag, m_o_flag);
}

// constructors
inline sc_fxnum::sc_fxnum(const sc_fxtype_params &type_params_,
                          sc_enc enc_, const sc_fxcast_switch &cast_sw,
                          sc_fxnum_observer *observer_) :
    m_rep(new scfx_rep), m_params(type_params_, enc_, cast_sw),
    m_q_flag(false), m_o_flag(false), m_observer(observer_)
{
    SC_FXNUM_OBSERVER_DEFAULT_
    SC_FXNUM_OBSERVER_CONSTRUCT_(*this)
}

#define DEFN_CTOR_T(tp, arg) \
inline sc_fxnum::sc_fxnum(tp a, const sc_fxtype_params &type_params_, \
                          sc_enc enc_, const sc_fxcast_switch &cast_sw, \
                          sc_fxnum_observer *observer_) : \
    m_rep(new scfx_rep(arg)), m_params(type_params_, enc_, cast_sw), \
    m_q_flag(false), m_o_flag(false), m_observer(observer_) \
{ \
    SC_FXNUM_OBSERVER_DEFAULT_ \
    cast(); \
    SC_FXNUM_OBSERVER_CONSTRUCT_(*this) \
    SC_FXNUM_OBSERVER_WRITE_(*this) \
}

#define DEFN_CTOR_T_A(tp) DEFN_CTOR_T(tp, a)
#define DEFN_CTOR_T_B(tp) DEFN_CTOR_T(tp, *a.m_rep)
#define DEFN_CTOR_T_C(tp) DEFN_CTOR_T(tp, a.to_double())
#define DEFN_CTOR_T_D(tp) DEFN_CTOR_T(tp, a.value())

DEFN_CTOR_T_A(int)
DEFN_CTOR_T_A(unsigned int)
DEFN_CTOR_T_A(long)
DEFN_CTOR_T_A(unsigned long)
DEFN_CTOR_T_A(float)
DEFN_CTOR_T_A(double)
DEFN_CTOR_T_A(const char *)
DEFN_CTOR_T_B(const sc_fxval &)
DEFN_CTOR_T_C(const sc_fxval_fast &)
DEFN_CTOR_T_B(const sc_fxnum &)
DEFN_CTOR_T_C(const sc_fxnum_fast &)
#ifndef SC_FX_EXCLUDE_OTHER
DEFN_CTOR_T_A(int64)
DEFN_CTOR_T_A(uint64)
DEFN_CTOR_T_D(const sc_int_base &)
DEFN_CTOR_T_D(const sc_uint_base &)
DEFN_CTOR_T_A(const sc_signed &)
DEFN_CTOR_T_A(const sc_unsigned &)
#endif

#undef DEFN_CTOR_T
#undef DEFN_CTOR_T_A
#undef DEFN_CTOR_T_B
#undef DEFN_CTOR_T_C
#undef DEFN_CTOR_T_D

inline sc_fxnum::~sc_fxnum()
{
    SC_FXNUM_OBSERVER_DESTRUCT_(*this)
    delete m_rep;
}

// internal use only;
inline const scfx_rep *
sc_fxnum::get_rep() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return m_rep;
}

// unary operators
inline const sc_fxval
sc_fxnum::operator - () const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return sc_fxval(sc_dt::neg_scfx_rep(*m_rep));
}

inline const sc_fxval
sc_fxnum::operator + () const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return sc_fxval(new scfx_rep(*m_rep));
}

// unary functions
inline void
neg(sc_fxval &c, const sc_fxnum &a)
{
    SC_FXNUM_OBSERVER_READ_(a)
    c.set_rep(sc_dt::neg_scfx_rep(*a.m_rep));
}

inline void
neg(sc_fxnum &c, const sc_fxnum &a)
{
    SC_FXNUM_OBSERVER_READ_(a)
    delete c.m_rep;
    c.m_rep = sc_dt::neg_scfx_rep(*a.m_rep);
    c.cast();
    SC_FXNUM_OBSERVER_WRITE_(c)
}

// binary operators
#define DEFN_BIN_OP_T(op, fnc, tp) \
inline const sc_fxval \
operator op (const sc_fxnum &a, tp b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    sc_fxval tmp(b); \
    return sc_fxval(sc_dt::fnc ## _scfx_rep(*a.m_rep, *tmp.get_rep())); \
} \
 \
inline const sc_fxval \
operator op (tp a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(b) \
    sc_fxval tmp(a); \
    return sc_fxval(sc_dt::fnc ## _scfx_rep(*tmp.get_rep(), *b.m_rep)); \
}

#ifndef SC_FX_EXCLUDE_OTHER
#define DEFN_BIN_OP_OTHER(op, fnc) \
DEFN_BIN_OP_T(op, fnc, int64) \
DEFN_BIN_OP_T(op, fnc, uint64) \
DEFN_BIN_OP_T(op, fnc, const sc_int_base &) \
DEFN_BIN_OP_T(op, fnc, const sc_uint_base &) \
DEFN_BIN_OP_T(op, fnc, const sc_signed &) \
DEFN_BIN_OP_T(op, fnc, const sc_unsigned &)
#else
#define DEFN_BIN_OP_OTHER(op, fnc)
#endif

#define DEFN_BIN_OP(op, fnc) \
inline const sc_fxval \
operator op (const sc_fxnum &a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    SC_FXNUM_OBSERVER_READ_(b) \
    return sc_fxval(sc_dt::fnc ## _scfx_rep(*a.m_rep, *b.m_rep)); \
} \
 \
inline const sc_fxval \
operator op (const sc_fxnum &a, const sc_fxval &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    return sc_fxval(sc_dt::fnc ## _scfx_rep(*a.m_rep, *b.get_rep())); \
} \
 \
inline const sc_fxval \
operator op (const sc_fxval &a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(b) \
    return sc_fxval(sc_dt::fnc ## _scfx_rep(*a.get_rep(), *b.m_rep)); \
} \
 \
DEFN_BIN_OP_T(op, fnc, int) \
DEFN_BIN_OP_T(op, fnc, unsigned int) \
DEFN_BIN_OP_T(op, fnc, long) \
DEFN_BIN_OP_T(op, fnc, unsigned long) \
DEFN_BIN_OP_T(op, fnc, float) \
DEFN_BIN_OP_T(op, fnc, double) \
DEFN_BIN_OP_T(op, fnc, const char *) \
DEFN_BIN_OP_T(op, fnc, const sc_fxval_fast &) \
DEFN_BIN_OP_T(op, fnc, const sc_fxnum_fast &) \
DEFN_BIN_OP_OTHER(op, fnc)

DEFN_BIN_OP(*, mult)
DEFN_BIN_OP(+, add)
DEFN_BIN_OP(-, sub)
// don't use macros
//DEFN_BIN_OP(/, div)
inline const sc_fxval
operator / (const sc_fxnum &a, const sc_fxnum &b)
{
    SC_FXNUM_OBSERVER_READ_(a)
    SC_FXNUM_OBSERVER_READ_(b)
    return sc_fxval(sc_dt::div_scfx_rep(*a.m_rep, *b.m_rep));
}

inline const sc_fxval
operator / (const sc_fxnum &a, const sc_fxval &b)
{
    SC_FXNUM_OBSERVER_READ_(a)
    return sc_fxval(sc_dt::div_scfx_rep(*a.m_rep, *b.get_rep()));
}

inline const sc_fxval
operator / (const sc_fxval &a, const sc_fxnum &b)
{
    SC_FXNUM_OBSERVER_READ_(b)
    return sc_fxval(sc_dt::div_scfx_rep(*a.get_rep(), *b.m_rep));
}

DEFN_BIN_OP_T(/, div, int)
DEFN_BIN_OP_T(/, div, unsigned int)
DEFN_BIN_OP_T(/, div, long)
DEFN_BIN_OP_T(/, div, unsigned long)
DEFN_BIN_OP_T(/, div, float)
DEFN_BIN_OP_T(/, div, double)
DEFN_BIN_OP_T(/, div, const char *)
DEFN_BIN_OP_T(/, div, const sc_fxval_fast &)
DEFN_BIN_OP_T(/, div, const sc_fxnum_fast &)
//DEFN_BIN_OP_OTHER(/, div)

DEFN_BIN_OP_T(/, div, int64)
DEFN_BIN_OP_T(/, div, uint64)
DEFN_BIN_OP_T(/, div, const sc_int_base &)
DEFN_BIN_OP_T(/, div, const sc_uint_base &)
DEFN_BIN_OP_T(/, div, const sc_signed &)
DEFN_BIN_OP_T(/, div, const sc_unsigned &)

#undef DEFN_BIN_OP_T
#undef DEFN_BIN_OP_OTHER
#undef DEFN_BIN_OP

inline const sc_fxval
operator << (const sc_fxnum &a, int b)
{
    SC_FXNUM_OBSERVER_READ_(a)
    return sc_fxval(sc_dt::lsh_scfx_rep(*a.m_rep, b));
}

inline const sc_fxval
operator >> (const sc_fxnum &a, int b)
{
    SC_FXNUM_OBSERVER_READ_(a)
    return sc_fxval(sc_dt::rsh_scfx_rep(*a.m_rep, b));
}

// binary functions
#define DEFN_BIN_FNC_T(fnc, tp) \
inline void \
fnc (sc_fxval &c, const sc_fxnum &a, tp b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    sc_fxval tmp(b); \
    c.set_rep(sc_dt::fnc ## _scfx_rep(*a.m_rep, *tmp.get_rep())); \
} \
 \
inline void \
fnc (sc_fxval &c, tp a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(b) \
    sc_fxval tmp(a); \
    c.set_rep(sc_dt::fnc ## _scfx_rep(*tmp.get_rep(), *b.m_rep)); \
} \
 \
inline void \
fnc (sc_fxnum &c, const sc_fxnum &a, tp b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    sc_fxval tmp(b); \
    delete c.m_rep; \
    c.m_rep = sc_dt::fnc ## _scfx_rep(*a.m_rep, *tmp.get_rep()); \
    c.cast(); \
    SC_FXNUM_OBSERVER_WRITE_(c) \
} \
 \
inline void \
fnc (sc_fxnum &c, tp a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(b) \
    sc_fxval tmp(a); \
    delete c.m_rep; \
    c.m_rep = sc_dt::fnc ## _scfx_rep(*tmp.get_rep(), *b.m_rep); \
    c.cast(); \
    SC_FXNUM_OBSERVER_WRITE_(c) \
}

#define DEFN_BIN_FNC_OTHER(fnc) \
DEFN_BIN_FNC_T(fnc, int64) \
DEFN_BIN_FNC_T(fnc, uint64) \
DEFN_BIN_FNC_T(fnc, const sc_int_base &) \
DEFN_BIN_FNC_T(fnc, const sc_uint_base &) \
DEFN_BIN_FNC_T(fnc, const sc_signed &) \
DEFN_BIN_FNC_T(fnc, const sc_unsigned &)

#define DEFN_BIN_FNC(fnc) \
inline void \
fnc (sc_fxval &c, const sc_fxnum &a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    SC_FXNUM_OBSERVER_READ_(b) \
    c.set_rep(sc_dt::fnc ## _scfx_rep(*a.m_rep, *b.m_rep)); \
} \
 \
inline void \
fnc (sc_fxnum &c, const sc_fxnum &a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    SC_FXNUM_OBSERVER_READ_(b) \
    delete c.m_rep; \
    c.m_rep = sc_dt::fnc ## _scfx_rep(*a.m_rep, *b.m_rep); \
    c.cast(); \
    SC_FXNUM_OBSERVER_WRITE_(c) \
} \
 \
inline void \
fnc (sc_fxval &c, const sc_fxnum &a, const sc_fxval &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    c.set_rep(sc_dt::fnc ## _scfx_rep(*a.m_rep, *b.get_rep())); \
} \
 \
inline void \
fnc (sc_fxval &c, const sc_fxval &a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(b) \
    c.set_rep(sc_dt::fnc ## _scfx_rep(*a.get_rep(), *b.m_rep)); \
} \
 \
inline void \
fnc (sc_fxnum &c, const sc_fxnum &a, const sc_fxval &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    delete c.m_rep; \
    c.m_rep = sc_dt::fnc ## _scfx_rep(*a.m_rep, *b.get_rep()); \
    c.cast(); \
    SC_FXNUM_OBSERVER_WRITE_(c) \
} \
 \
inline void \
fnc (sc_fxnum &c, const sc_fxval &a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(b) \
    delete c.m_rep; \
    c.m_rep = sc_dt::fnc ## _scfx_rep(*a.get_rep(), *b.m_rep); \
    c.cast(); \
    SC_FXNUM_OBSERVER_WRITE_(c) \
} \
 \
DEFN_BIN_FNC_T(fnc, int) \
DEFN_BIN_FNC_T(fnc, unsigned int) \
DEFN_BIN_FNC_T(fnc, long) \
DEFN_BIN_FNC_T(fnc, unsigned long) \
DEFN_BIN_FNC_T(fnc, float) \
DEFN_BIN_FNC_T(fnc, double) \
DEFN_BIN_FNC_T(fnc, const char *) \
DEFN_BIN_FNC_T(fnc, const sc_fxval_fast &) \
DEFN_BIN_FNC_T(fnc, const sc_fxnum_fast &) \
DEFN_BIN_FNC_OTHER(fnc)

DEFN_BIN_FNC(mult)
DEFN_BIN_FNC(div)
DEFN_BIN_FNC(add)
DEFN_BIN_FNC(sub)

#undef DEFN_BIN_FNC_T
#undef DEFN_BIN_FNC_OTHER
#undef DEFN_BIN_FNC

inline void
lshift(sc_fxval &c, const sc_fxnum &a, int b)
{
    SC_FXNUM_OBSERVER_READ_(a)
    c.set_rep(sc_dt::lsh_scfx_rep(*a.m_rep, b));
}

inline void
rshift(sc_fxval &c, const sc_fxnum &a, int b)
{
    SC_FXNUM_OBSERVER_READ_(a)
    c.set_rep(sc_dt::rsh_scfx_rep(*a.m_rep, b));
}

inline void
lshift(sc_fxnum &c, const sc_fxnum &a, int b)
{
    SC_FXNUM_OBSERVER_READ_(a)
    delete c.m_rep;
    c.m_rep = sc_dt::lsh_scfx_rep(*a.m_rep, b);
    c.cast();
    SC_FXNUM_OBSERVER_WRITE_(c)
}

inline void
rshift(sc_fxnum &c, const sc_fxnum &a, int b)
{
    SC_FXNUM_OBSERVER_READ_(a)
    delete c.m_rep;
    c.m_rep = sc_dt::rsh_scfx_rep(*a.m_rep, b);
    c.cast();
    SC_FXNUM_OBSERVER_WRITE_(c)
}

// relational (including equality) operators
#define DEFN_REL_OP_T(op, ret, tp) \
inline bool \
operator op (const sc_fxnum &a, tp b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    sc_fxval tmp(b); \
    int result = sc_dt::cmp_scfx_rep(*a.m_rep, *tmp.get_rep()); \
    return (ret); \
} \
 \
inline bool \
operator op (tp a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(b) \
    sc_fxval tmp(a); \
    int result = sc_dt::cmp_scfx_rep(*tmp.get_rep(), *b.m_rep); \
    return (ret); \
}

#define DEFN_REL_OP_OTHER(op, ret) \
DEFN_REL_OP_T(op, ret, int64) \
DEFN_REL_OP_T(op, ret, uint64) \
DEFN_REL_OP_T(op, ret, const sc_int_base &) \
DEFN_REL_OP_T(op, ret, const sc_uint_base &) \
DEFN_REL_OP_T(op, ret, const sc_signed &) \
DEFN_REL_OP_T(op, ret, const sc_unsigned &)

#define DEFN_REL_OP(op, ret) \
inline bool \
operator op (const sc_fxnum &a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    SC_FXNUM_OBSERVER_READ_(b) \
    int result = sc_dt::cmp_scfx_rep(*a.m_rep, *b.m_rep); \
    return (ret); \
} \
 \
inline bool \
operator op (const sc_fxnum &a, const sc_fxval &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(a) \
    int result = sc_dt::cmp_scfx_rep(*a.m_rep, *b.get_rep()); \
    return (ret); \
} \
 \
inline bool \
operator op (const sc_fxval &a, const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(b) \
    int result = sc_dt::cmp_scfx_rep(*a.get_rep(), *b.m_rep); \
    return (ret); \
} \
 \
DEFN_REL_OP_T(op, ret, int) \
DEFN_REL_OP_T(op, ret, unsigned int) \
DEFN_REL_OP_T(op, ret, long) \
DEFN_REL_OP_T(op, ret, unsigned long) \
DEFN_REL_OP_T(op, ret, float) \
DEFN_REL_OP_T(op, ret, double) \
DEFN_REL_OP_T(op, ret, const char *) \
DEFN_REL_OP_T(op, ret, const sc_fxval_fast &) \
DEFN_REL_OP_T(op, ret, const sc_fxnum_fast &) \
DEFN_REL_OP_OTHER(op, ret)

DEFN_REL_OP(<, result < 0)
DEFN_REL_OP(<=, result <= 0)
DEFN_REL_OP(>, result > 0 && result != 2)
DEFN_REL_OP(>=, result >= 0 && result != 2)
DEFN_REL_OP(==, result == 0)
DEFN_REL_OP(!=, result != 0)

#undef DEFN_REL_OP_T
#undef DEFN_REL_OP_OTHER
#undef DEFN_REL_OP

// assignment operators
inline sc_fxnum &
sc_fxnum::operator = (const sc_fxnum &a)
{
    if (&a != this) {
        SC_FXNUM_OBSERVER_READ_(a)
        *m_rep = *a.m_rep;
        cast();
        SC_FXNUM_OBSERVER_WRITE_(*this)
    }
    return *this;
}

inline sc_fxnum &
sc_fxnum::operator = (const sc_fxval &a)
{
    *m_rep = *a.get_rep();
    cast();
    SC_FXNUM_OBSERVER_WRITE_(*this)
    return *this;
}

#define DEFN_ASN_OP_T(tp) \
inline sc_fxnum & \
sc_fxnum::operator = (tp a) \
{ \
    sc_fxval tmp(a); \
    *m_rep = *tmp.get_rep(); \
    cast(); \
    SC_FXNUM_OBSERVER_WRITE_(*this) \
    return *this; \
}

DEFN_ASN_OP_T(int)
DEFN_ASN_OP_T(unsigned int)
DEFN_ASN_OP_T(long)
DEFN_ASN_OP_T(unsigned long)
DEFN_ASN_OP_T(float)
DEFN_ASN_OP_T(double)
DEFN_ASN_OP_T(const char *)
DEFN_ASN_OP_T(const sc_fxval_fast &)
DEFN_ASN_OP_T(const sc_fxnum_fast &)

DEFN_ASN_OP_T(int64)
DEFN_ASN_OP_T(uint64)
DEFN_ASN_OP_T(const sc_int_base &)
DEFN_ASN_OP_T(const sc_uint_base &)
DEFN_ASN_OP_T(const sc_signed &)
DEFN_ASN_OP_T(const sc_unsigned &)

#undef DEFN_ASN_OP_T


#define DEFN_ASN_OP_T(op, fnc, tp) \
inline sc_fxnum & \
sc_fxnum::operator op (tp b) \
{ \
    SC_FXNUM_OBSERVER_READ_(*this) \
    sc_fxval tmp(b); \
    scfx_rep *new_rep = sc_dt::fnc ## _scfx_rep(*m_rep, *tmp.get_rep()); \
    delete m_rep; \
    m_rep = new_rep; \
    cast(); \
    SC_FXNUM_OBSERVER_WRITE_(*this) \
    return *this; \
}

#define DEFN_ASN_OP_OTHER(op, fnc) \
DEFN_ASN_OP_T(op, fnc, int64) \
DEFN_ASN_OP_T(op, fnc, uint64) \
DEFN_ASN_OP_T(op, fnc, const sc_int_base &) \
DEFN_ASN_OP_T(op, fnc, const sc_uint_base &) \
DEFN_ASN_OP_T(op, fnc, const sc_signed &) \
DEFN_ASN_OP_T(op, fnc, const sc_unsigned &)

#define DEFN_ASN_OP(op, fnc) \
inline sc_fxnum & \
sc_fxnum::operator op (const sc_fxnum &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(*this) \
    SC_FXNUM_OBSERVER_READ_(b) \
    scfx_rep *new_rep = sc_dt::fnc ## _scfx_rep(*m_rep, *b.m_rep); \
    delete m_rep; \
    m_rep = new_rep; \
    cast(); \
    SC_FXNUM_OBSERVER_WRITE_(*this) \
    return *this; \
} \
 \
inline sc_fxnum & \
sc_fxnum::operator op (const sc_fxval &b) \
{ \
    SC_FXNUM_OBSERVER_READ_(*this) \
    scfx_rep *new_rep = sc_dt::fnc ## _scfx_rep(*m_rep, *b.get_rep()); \
    delete m_rep; \
    m_rep = new_rep; \
    cast(); \
    SC_FXNUM_OBSERVER_WRITE_(*this) \
    return *this; \
} \
 \
DEFN_ASN_OP_T(op, fnc, int) \
DEFN_ASN_OP_T(op, fnc, unsigned int) \
DEFN_ASN_OP_T(op, fnc, long) \
DEFN_ASN_OP_T(op, fnc, unsigned long) \
DEFN_ASN_OP_T(op, fnc, float) \
DEFN_ASN_OP_T(op, fnc, double) \
DEFN_ASN_OP_T(op, fnc, const char *) \
DEFN_ASN_OP_T(op, fnc, const sc_fxval_fast &) \
DEFN_ASN_OP_T(op, fnc, const sc_fxnum_fast &) \
DEFN_ASN_OP_OTHER(op, fnc)

DEFN_ASN_OP(*=, mult)
DEFN_ASN_OP(/=, div)
DEFN_ASN_OP(+=, add)
DEFN_ASN_OP(-=, sub)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP_OTHER
#undef DEFN_ASN_OP


inline sc_fxnum &
sc_fxnum::operator <<= (int b)
{
    SC_FXNUM_OBSERVER_READ_(*this)
    m_rep->lshift(b);
    cast();
    SC_FXNUM_OBSERVER_WRITE_(*this)
    return *this;
}

inline sc_fxnum &
sc_fxnum::operator >>= (int b)
{
    SC_FXNUM_OBSERVER_READ_(*this)
    m_rep->rshift(b);
    cast();
    SC_FXNUM_OBSERVER_WRITE_(*this)
    return *this;
}

// auto-increment and auto-decrement
inline const sc_fxval
sc_fxnum::operator ++ (int)
{
    sc_fxval c(*this);
    (*this) += 1;
    return c;
}

inline const sc_fxval
sc_fxnum::operator -- (int)
{
    sc_fxval c(*this);
    (*this) -= 1;
    return c;
}

inline sc_fxnum &
sc_fxnum::operator ++ ()
{
    (*this) += 1;
    return *this;
}

inline sc_fxnum &
sc_fxnum::operator -- ()
{
    (*this) -= 1;
    return *this;
}

// bit selection
inline const sc_fxnum_bitref
sc_fxnum::operator [] (int i) const
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    return sc_fxnum_bitref(const_cast<sc_fxnum &>(*this),
                           i - m_params.fwl());
}

inline sc_fxnum_bitref
sc_fxnum::operator [] (int i)
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    return sc_fxnum_bitref(*this, i - m_params.fwl());
}

inline const sc_fxnum_bitref
sc_fxnum::bit(int i) const
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    return sc_fxnum_bitref(const_cast<sc_fxnum &>(*this),
                            i - m_params.fwl());
}

inline sc_fxnum_bitref
sc_fxnum::bit(int i)
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    return sc_fxnum_bitref(*this, i - m_params.fwl());
}

// part selection

inline const sc_fxnum_subref
sc_fxnum::operator () (int i, int j) const
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    SC_ERROR_IF_(j < 0 || j >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);

    return sc_fxnum_subref(const_cast<sc_fxnum &>(*this),
                           i - m_params.fwl(), j - m_params.fwl());
}

inline sc_fxnum_subref
sc_fxnum::operator () (int i, int j)
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    SC_ERROR_IF_(j < 0 || j >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);

    return sc_fxnum_subref(*this, i - m_params.fwl(), j - m_params.fwl());
}

inline const sc_fxnum_subref
sc_fxnum::range(int i, int j) const
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    SC_ERROR_IF_(j < 0 || j >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);

    return sc_fxnum_subref(const_cast<sc_fxnum &>(*this),
                           i - m_params.fwl(), j - m_params.fwl());
}

inline sc_fxnum_subref
sc_fxnum::range(int i, int j)
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    SC_ERROR_IF_(j < 0 || j >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);

    return sc_fxnum_subref(*this, i - m_params.fwl(), j - m_params.fwl());
}


inline const sc_fxnum_subref
sc_fxnum::operator () () const
{
    return this->operator () (m_params.wl() - 1, 0);
}

inline sc_fxnum_subref
sc_fxnum::operator () ()
{
    return this->operator () (m_params.wl() - 1, 0);
}

inline const sc_fxnum_subref
sc_fxnum::range() const
{
    return this->range(m_params.wl() - 1, 0);
}

inline sc_fxnum_subref
sc_fxnum::range()
{
    return this->range(m_params.wl() - 1, 0);
}

// implicit conversion
inline sc_fxnum::operator double() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return m_rep->to_double();
}

// explicit conversion to primitive types
inline short
sc_fxnum::to_short() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return static_cast<short>(m_rep->to_uint64());
}

inline unsigned short
sc_fxnum::to_ushort() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return static_cast<unsigned short>(m_rep->to_uint64());
}

inline int
sc_fxnum::to_int() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return static_cast<int>(m_rep->to_uint64());
}

inline int64
sc_fxnum::to_int64() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return static_cast<int64>(m_rep->to_uint64());
}

inline unsigned int
sc_fxnum::to_uint() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return static_cast<unsigned int>(m_rep->to_uint64());
}

inline uint64
sc_fxnum::to_uint64() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return m_rep->to_uint64();
}

inline long
sc_fxnum::to_long() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return static_cast<long>(m_rep->to_uint64());
}

inline unsigned long
sc_fxnum::to_ulong() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return static_cast<unsigned long>(m_rep->to_uint64());
}

inline float
sc_fxnum::to_float() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return static_cast<float>(m_rep->to_double());
}

inline double
sc_fxnum::to_double() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return m_rep->to_double();
}

// query value
inline bool
sc_fxnum::is_neg() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return m_rep->is_neg();
}

inline bool
sc_fxnum::is_zero() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return m_rep->is_zero();
}

// internal use only;
inline bool
sc_fxnum::is_normal() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return m_rep->is_normal();
}

inline bool
sc_fxnum::quantization_flag() const
{
    return m_q_flag;
}

inline bool
sc_fxnum::overflow_flag() const
{
    return m_o_flag;
}


inline const sc_fxval
sc_fxnum::value() const
{
    SC_FXNUM_OBSERVER_READ_(*this)
    return sc_fxval(new scfx_rep(*m_rep));
}

// query parameters
inline int
sc_fxnum::wl() const
{
    return m_params.wl();
}

inline int
sc_fxnum::iwl() const
{
    return m_params.iwl();
}

inline sc_q_mode
sc_fxnum::q_mode() const
{
    return m_params.q_mode();
}

inline sc_o_mode
sc_fxnum::o_mode() const
{
    return m_params.o_mode();
}

inline int
sc_fxnum::n_bits() const
{
    return m_params.n_bits();
}

inline const sc_fxtype_params &
sc_fxnum::type_params() const
{
    return m_params.type_params();
}

inline const sc_fxcast_switch &
sc_fxnum::cast_switch() const
{
    return m_params.cast_switch();
}

// internal use only;
inline void
sc_fxnum::observer_read() const
{
    SC_FXNUM_OBSERVER_READ_(*this);
}

// internal use only;
inline bool
sc_fxnum::get_bit(int i) const
{
    return m_rep->get_bit(i);
}

// protected methods and friend functions
inline bool
sc_fxnum::set_bit(int i, bool high)
{
    if (high)
        return m_rep->set(i, m_params);
    else
        return m_rep->clear(i, m_params);
}

inline bool
sc_fxnum::get_slice(int i, int j, sc_bv_base &bv) const
{
    return m_rep->get_slice(i, j, m_params, bv);
}

inline bool
sc_fxnum::set_slice(int i, int j, const sc_bv_base &bv)
{
    return m_rep->set_slice(i, j, m_params, bv);
}

inline ::std::ostream &
operator << (::std::ostream &os, const sc_fxnum &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_fxnum &a)
{
    a.scan(is);
    return is;
}


// ----------------------------------------------------------------------------
// CLASS : sc_fxnum_fast
//
// Base class for the fixed-point types; limited precision.
// ----------------------------------------------------------------------------

inline sc_fxnum_fast_observer *
sc_fxnum_fast::observer() const
{
    return m_observer;
}


// constructors
inline sc_fxnum_fast::sc_fxnum_fast(const sc_fxtype_params &type_params_,
                                    sc_enc enc_,
                                    const sc_fxcast_switch &cast_sw,
                                    sc_fxnum_fast_observer *observer_) :
    m_val(0.0), m_params(type_params_, enc_, cast_sw), m_q_flag(false),
    m_o_flag(false), m_observer(observer_)
{
    SC_FXNUM_FAST_OBSERVER_DEFAULT_
    SC_FXNUM_FAST_OBSERVER_CONSTRUCT_(*this)
}

inline sc_fxnum_fast::sc_fxnum_fast(const sc_fxnum_fast &a,
                                    const sc_fxtype_params &type_params_,
                                    sc_enc enc_,
                                    const sc_fxcast_switch &cast_sw,
                                    sc_fxnum_fast_observer *observer_) :
    m_val(a.m_val), m_params(type_params_, enc_, cast_sw), m_q_flag(false),
    m_o_flag(false), m_observer(observer_)
{
    SC_FXNUM_FAST_OBSERVER_DEFAULT_
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    cast();
    SC_FXNUM_FAST_OBSERVER_CONSTRUCT_(*this)
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
}

#define DEFN_CTOR_T(tp, arg) \
inline sc_fxnum_fast::sc_fxnum_fast( \
        tp a, const sc_fxtype_params &type_params_, sc_enc enc_, \
        const sc_fxcast_switch &cast_sw, \
        sc_fxnum_fast_observer *observer_) : \
    m_val(arg), m_params(type_params_, enc_, cast_sw), m_q_flag(false), \
    m_o_flag(false), m_observer(observer_) \
{ \
    SC_FXNUM_FAST_OBSERVER_DEFAULT_ \
    cast(); \
    SC_FXNUM_FAST_OBSERVER_CONSTRUCT_(*this) \
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this) \
}

#define DEFN_CTOR_T_A(tp) DEFN_CTOR_T(tp, static_cast<double>(a))
#define DEFN_CTOR_T_B(tp) DEFN_CTOR_T(tp, sc_fxval_fast::from_string(a))
#define DEFN_CTOR_T_C(tp) DEFN_CTOR_T(tp, a.to_double())

DEFN_CTOR_T_A(int)
DEFN_CTOR_T_A(unsigned int)
DEFN_CTOR_T_A(long)
DEFN_CTOR_T_A(unsigned long)
DEFN_CTOR_T_A(float)
DEFN_CTOR_T_A(double)
DEFN_CTOR_T_B(const char *)
DEFN_CTOR_T_C(const sc_fxval &)
DEFN_CTOR_T_C(const sc_fxval_fast &)
DEFN_CTOR_T_C(const sc_fxnum &)

DEFN_CTOR_T_A(int64)
DEFN_CTOR_T_A(uint64)
DEFN_CTOR_T_C(const sc_int_base &)
DEFN_CTOR_T_C(const sc_uint_base &)
DEFN_CTOR_T_C(const sc_signed &)
DEFN_CTOR_T_C(const sc_unsigned &)

#undef DEFN_CTOR_T
#undef DEFN_CTOR_T_A
#undef DEFN_CTOR_T_B
#undef DEFN_CTOR_T_C
#undef DEFN_CTOR_T_D
#undef DEFN_CTOR_T_E

inline sc_fxnum_fast::~sc_fxnum_fast()
{
    SC_FXNUM_FAST_OBSERVER_DESTRUCT_(*this)
}

// internal use only;
inline double
sc_fxnum_fast::get_val() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    return m_val;
}

// unary operators
inline const sc_fxval_fast
sc_fxnum_fast::operator - () const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    return sc_fxval_fast(- m_val);
}

inline const sc_fxval_fast
sc_fxnum_fast::operator + () const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    return sc_fxval_fast(m_val);
}

// unary functions
inline void
neg(sc_fxval_fast &c, const sc_fxnum_fast &a)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    c.set_val(- a.m_val);
}

inline void
neg(sc_fxnum_fast &c, const sc_fxnum_fast &a)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    c.m_val = - a.m_val;
    c.cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(c)
}

// binary operators
#define DEFN_BIN_OP_T(op, tp) \
inline const sc_fxval_fast \
operator op (const sc_fxnum_fast &a, tp b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    sc_fxval_fast tmp(b); \
    return sc_fxval_fast(a.m_val op tmp.get_val()); \
} \
 \
inline const sc_fxval_fast \
operator op (tp a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    sc_fxval_fast tmp(a); \
    return sc_fxval_fast(tmp.get_val() op b.m_val); \
}

#define DEFN_BIN_OP_OTHER(op) \
DEFN_BIN_OP_T(op, int64) \
DEFN_BIN_OP_T(op, uint64) \
DEFN_BIN_OP_T(op, const sc_int_base &) \
DEFN_BIN_OP_T(op, const sc_uint_base &) \
DEFN_BIN_OP_T(op, const sc_signed &) \
DEFN_BIN_OP_T(op, const sc_unsigned &)

#define DEFN_BIN_OP(op, dummy) \
inline const sc_fxval_fast \
operator op (const sc_fxnum_fast &a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    return sc_fxval_fast(a.m_val op b.m_val); \
} \
 \
inline const sc_fxval_fast \
operator op (const sc_fxnum_fast &a, const sc_fxval_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    return sc_fxval_fast(a.m_val op b.get_val()); \
} \
 \
inline const sc_fxval_fast \
operator op (const sc_fxval_fast &a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    return sc_fxval_fast(a.get_val() op b.m_val); \
} \
 \
DEFN_BIN_OP_T(op, int) \
DEFN_BIN_OP_T(op, unsigned int) \
DEFN_BIN_OP_T(op, long) \
DEFN_BIN_OP_T(op, unsigned long) \
DEFN_BIN_OP_T(op, float) \
DEFN_BIN_OP_T(op, double) \
DEFN_BIN_OP_T(op, const char *) \
DEFN_BIN_OP_OTHER(op)

DEFN_BIN_OP(*, mult)
DEFN_BIN_OP(+, add)
DEFN_BIN_OP(-, sub)
//DEFN_BIN_OP(/, div)
inline const sc_fxval_fast
operator / (const sc_fxnum_fast &a, const sc_fxnum_fast &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    SC_FXNUM_FAST_OBSERVER_READ_(b)
    return sc_fxval_fast(a.m_val / b.m_val);
}

inline const sc_fxval_fast
operator / (const sc_fxnum_fast &a, const sc_fxval_fast &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    return sc_fxval_fast(a.m_val / b.get_val());
}

inline const sc_fxval_fast
operator / (const sc_fxval_fast &a, const sc_fxnum_fast &b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(b)
    return sc_fxval_fast(a.get_val() / b.m_val);
}

DEFN_BIN_OP_T(/, int)
DEFN_BIN_OP_T(/, unsigned int)
DEFN_BIN_OP_T(/, long)
DEFN_BIN_OP_T(/, unsigned long)
DEFN_BIN_OP_T(/, float)
DEFN_BIN_OP_T(/, double)
DEFN_BIN_OP_T(/, const char *)
//DEFN_BIN_OP_OTHER(/)

DEFN_BIN_OP_T(/, int64)
DEFN_BIN_OP_T(/, uint64)
DEFN_BIN_OP_T(/, const sc_int_base &)
DEFN_BIN_OP_T(/, const sc_uint_base &)
DEFN_BIN_OP_T(/, const sc_signed &)
DEFN_BIN_OP_T(/, const sc_unsigned &)

#undef DEFN_BIN_OP_T
#undef DEFN_BIN_OP_OTHER
#undef DEFN_BIN_OP

inline const sc_fxval_fast
operator << (const sc_fxnum_fast &a, int b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    return sc_fxval_fast(a.m_val  *scfx_pow2(b));
}

inline const sc_fxval_fast
operator >> (const sc_fxnum_fast &a, int b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    return sc_fxval_fast(a.m_val  *scfx_pow2(-b));
}

// binary functions
#define DEFN_BIN_FNC_T(fnc, op, tp) \
inline void \
fnc (sc_fxval_fast &c, const sc_fxnum_fast &a, tp b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    sc_fxval_fast tmp(b); \
    c.set_val(a.m_val op tmp.get_val()); \
} \
 \
inline void \
fnc (sc_fxval_fast &c, tp a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    sc_fxval_fast tmp(a); \
    c.set_val(tmp.get_val() op b.m_val); \
} \
 \
inline void \
fnc (sc_fxnum_fast &c, const sc_fxnum_fast &a, tp b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    sc_fxval_fast tmp(b); \
    c.m_val = a.m_val op tmp.get_val(); \
    c.cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(c) \
} \
 \
inline void \
fnc (sc_fxnum_fast &c, tp a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    sc_fxval_fast tmp(a); \
    c.m_val = tmp.get_val() op b.m_val; \
    c.cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(c) \
}

#define DEFN_BIN_FNC_OTHER(fnc, op) \
DEFN_BIN_FNC_T(fnc, op, int64) \
DEFN_BIN_FNC_T(fnc, op, uint64) \
DEFN_BIN_FNC_T(fnc, op, const sc_int_base &) \
DEFN_BIN_FNC_T(fnc, op, const sc_uint_base &) \
DEFN_BIN_FNC_T(fnc, op, const sc_signed &) \
DEFN_BIN_FNC_T(fnc, op, const sc_unsigned &)

#define DEFN_BIN_FNC(fnc, op) \
inline void \
fnc (sc_fxval_fast &c, const sc_fxnum_fast &a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    c.set_val(a.m_val op b.m_val); \
} \
 \
inline void \
fnc (sc_fxnum_fast &c, const sc_fxnum_fast &a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    c.m_val = a.m_val op b.m_val; \
    c.cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(c) \
} \
 \
inline void \
fnc (sc_fxval_fast &c, const sc_fxnum_fast &a, const sc_fxval_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    c.set_val(a.m_val op b.get_val()); \
} \
 \
inline void \
fnc (sc_fxval_fast &c, const sc_fxval_fast &a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    c.set_val(a.get_val() op b.m_val); \
} \
 \
inline void \
fnc (sc_fxnum_fast &c, const sc_fxnum_fast &a, const sc_fxval_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    c.m_val = a.m_val op b.get_val(); \
    c.cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(c) \
} \
 \
inline void \
fnc (sc_fxnum_fast &c, const sc_fxval_fast &a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    c.m_val = a.get_val() op b.m_val; \
    c.cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(c) \
} \
 \
DEFN_BIN_FNC_T(fnc, op, int) \
DEFN_BIN_FNC_T(fnc, op, unsigned int) \
DEFN_BIN_FNC_T(fnc, op, long) \
DEFN_BIN_FNC_T(fnc, op, unsigned long) \
DEFN_BIN_FNC_T(fnc, op, float) \
DEFN_BIN_FNC_T(fnc, op, double) \
DEFN_BIN_FNC_T(fnc, op, const char *) \
DEFN_BIN_FNC_T(fnc, op, const sc_fxval &) \
DEFN_BIN_FNC_T(fnc, op, const sc_fxnum &) \
DEFN_BIN_FNC_OTHER(fnc, op)

DEFN_BIN_FNC(mult, *)
DEFN_BIN_FNC(div, /)
DEFN_BIN_FNC(add, +)
DEFN_BIN_FNC(sub, -)

#undef DEFN_BIN_FNC_T
#undef DEFN_BIN_FNC_OTHER
#undef DEFN_BIN_FNC

inline void
lshift(sc_fxval_fast &c, const sc_fxnum_fast &a, int b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    c.set_val(a.m_val * scfx_pow2(b));
}

inline void
rshift(sc_fxval_fast &c, const sc_fxnum_fast &a, int b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    c.set_val(a.m_val * scfx_pow2(-b));
}

inline void
lshift(sc_fxnum_fast &c, const sc_fxnum_fast &a, int b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    c.m_val = a.m_val * scfx_pow2(b);
    c.cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(c)
}

inline void
rshift(sc_fxnum_fast &c, const sc_fxnum_fast &a, int b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(a)
    c.m_val = a.m_val * scfx_pow2(-b);
    c.cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(c)
}

// relational (including equality) operators
#define DEFN_REL_OP_T(op, tp) \
inline bool \
operator op (const sc_fxnum_fast &a, tp b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    sc_fxval_fast tmp(b); \
    return (a.m_val op tmp.get_val()); \
} \
 \
inline bool \
operator op (tp a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    sc_fxval_fast tmp(a); \
    return (tmp.get_val() op b.m_val); \
}

#define DEFN_REL_OP_OTHER(op) \
DEFN_REL_OP_T(op, int64) \
DEFN_REL_OP_T(op, uint64) \
DEFN_REL_OP_T(op, const sc_int_base &) \
DEFN_REL_OP_T(op, const sc_uint_base &) \
DEFN_REL_OP_T(op, const sc_signed &) \
DEFN_REL_OP_T(op, const sc_unsigned &)

#define DEFN_REL_OP(op) \
inline bool \
operator op (const sc_fxnum_fast &a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    return (a.m_val op b.m_val); \
} \
 \
inline bool \
operator op (const sc_fxnum_fast &a, const sc_fxval_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(a) \
    return (a.m_val op b.get_val()); \
} \
 \
inline bool \
operator op (const sc_fxval_fast &a, const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    return (a.get_val() op b.m_val); \
} \
 \
DEFN_REL_OP_T(op, int) \
DEFN_REL_OP_T(op, unsigned int) \
DEFN_REL_OP_T(op, long) \
DEFN_REL_OP_T(op, unsigned long) \
DEFN_REL_OP_T(op, float) \
DEFN_REL_OP_T(op, double) \
DEFN_REL_OP_T(op, const char *) \
DEFN_REL_OP_OTHER(op)

DEFN_REL_OP(<)
DEFN_REL_OP(<=)
DEFN_REL_OP(>)
DEFN_REL_OP(>=)
DEFN_REL_OP(==)
DEFN_REL_OP(!=)

#undef DEFN_REL_OP_T
#undef DEFN_REL_OP_OTHER
#undef DEFN_REL_OP

// assignment operators

inline sc_fxnum_fast &
sc_fxnum_fast::operator = (const sc_fxnum_fast &a)
{
    if (&a != this) {
        SC_FXNUM_FAST_OBSERVER_READ_(a)
        m_val = a.m_val;
        cast();
        SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
    }
    return *this;
}

inline sc_fxnum_fast &
sc_fxnum_fast::operator = (const sc_fxval_fast &a)
{
    m_val = a.get_val();
    cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
    return *this;
}

#define DEFN_ASN_OP_T(tp) \
inline sc_fxnum_fast & \
sc_fxnum_fast::operator = (tp a) \
{ \
    sc_fxval_fast tmp(a); \
    m_val = tmp.get_val(); \
    cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this) \
    return *this; \
}

DEFN_ASN_OP_T(int)
DEFN_ASN_OP_T(unsigned int)
DEFN_ASN_OP_T(long)
DEFN_ASN_OP_T(unsigned long)
DEFN_ASN_OP_T(float)
DEFN_ASN_OP_T(double)
DEFN_ASN_OP_T(const char *)
DEFN_ASN_OP_T(const sc_fxval &)
DEFN_ASN_OP_T(const sc_fxnum &)

DEFN_ASN_OP_T(int64)
DEFN_ASN_OP_T(uint64)
DEFN_ASN_OP_T(const sc_int_base &)
DEFN_ASN_OP_T(const sc_uint_base &)
DEFN_ASN_OP_T(const sc_signed &)
DEFN_ASN_OP_T(const sc_unsigned &)

#undef DEFN_ASN_OP_T

#define DEFN_ASN_OP_T(op, tp) \
inline sc_fxnum_fast & \
sc_fxnum_fast::operator op (tp b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(*this) \
    sc_fxval_fast tmp(b); \
    m_val op tmp.get_val(); \
    cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this) \
    return *this; \
}

#define DEFN_ASN_OP_OTHER(op) \
DEFN_ASN_OP_T(op, int64) \
DEFN_ASN_OP_T(op, uint64) \
DEFN_ASN_OP_T(op, const sc_int_base &) \
DEFN_ASN_OP_T(op, const sc_uint_base &) \
DEFN_ASN_OP_T(op, const sc_signed &) \
DEFN_ASN_OP_T(op, const sc_unsigned &)

#define DEFN_ASN_OP(op) \
inline sc_fxnum_fast & \
sc_fxnum_fast::operator op (const sc_fxnum_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(*this) \
    SC_FXNUM_FAST_OBSERVER_READ_(b) \
    m_val op b.m_val; \
    cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this) \
    return *this; \
} \
 \
inline sc_fxnum_fast & \
sc_fxnum_fast::operator op (const sc_fxval_fast &b) \
{ \
    SC_FXNUM_FAST_OBSERVER_READ_(*this) \
    m_val op b.get_val(); \
    cast(); \
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this) \
    return *this; \
} \
 \
DEFN_ASN_OP_T(op, int) \
DEFN_ASN_OP_T(op, unsigned int) \
DEFN_ASN_OP_T(op, long) \
DEFN_ASN_OP_T(op, unsigned long) \
DEFN_ASN_OP_T(op, float) \
DEFN_ASN_OP_T(op, double) \
DEFN_ASN_OP_T(op, const char *) \
DEFN_ASN_OP_T(op, const sc_fxval &) \
DEFN_ASN_OP_T(op, const sc_fxnum &) \
DEFN_ASN_OP_OTHER(op)

DEFN_ASN_OP(*=)
DEFN_ASN_OP(/=)
DEFN_ASN_OP(+=)
DEFN_ASN_OP(-=)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP_OTHER
#undef DEFN_ASN_OP

inline sc_fxnum_fast &
sc_fxnum_fast::operator <<= (int b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    m_val *= scfx_pow2(b);
    cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
    return *this;
}

inline sc_fxnum_fast &
sc_fxnum_fast::operator >>= (int b)
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    m_val *= scfx_pow2(-b);
    cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
    return *this;
}

// auto-increment and auto-decrement
inline const sc_fxval_fast
sc_fxnum_fast::operator ++ (int)
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    double c = m_val;
    m_val = m_val + 1;
    cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
    return sc_fxval_fast(c);
}

inline const sc_fxval_fast
sc_fxnum_fast::operator -- (int)
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    double c = m_val;
    m_val = m_val - 1;
    cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
    return sc_fxval_fast(c);
}

inline sc_fxnum_fast &
sc_fxnum_fast::operator ++ ()
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    m_val = m_val + 1;
    cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
    return *this;
}

inline sc_fxnum_fast &
sc_fxnum_fast::operator -- ()
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    m_val = m_val - 1;
    cast();
    SC_FXNUM_FAST_OBSERVER_WRITE_(*this)
    return *this;
}

// bit selection
inline const sc_fxnum_fast_bitref
sc_fxnum_fast::operator [] (int i) const
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    return sc_fxnum_fast_bitref(const_cast<sc_fxnum_fast &>(*this),
                                i - m_params.fwl());
}

inline sc_fxnum_fast_bitref
sc_fxnum_fast::operator [] (int i)
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    return sc_fxnum_fast_bitref(*this, i - m_params.fwl());
}

inline const sc_fxnum_fast_bitref
sc_fxnum_fast::bit(int i) const
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    return sc_fxnum_fast_bitref(const_cast<sc_fxnum_fast &>(*this),
                                i - m_params.fwl());
}

inline sc_fxnum_fast_bitref
sc_fxnum_fast::bit(int i)
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    return sc_fxnum_fast_bitref(*this, i - m_params.fwl());
}

// part selection
inline const sc_fxnum_fast_subref
sc_fxnum_fast::operator () (int i, int j) const
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    SC_ERROR_IF_(j < 0 || j >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);

    return sc_fxnum_fast_subref(const_cast<sc_fxnum_fast &>(*this),
                                i - m_params.fwl(), j - m_params.fwl());
}

inline sc_fxnum_fast_subref
sc_fxnum_fast::operator () (int i, int j)
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    SC_ERROR_IF_(j < 0 || j >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);

    return sc_fxnum_fast_subref(*this, i - m_params.fwl(), j - m_params.fwl());
}

inline const sc_fxnum_fast_subref
sc_fxnum_fast::range(int i, int j) const
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    SC_ERROR_IF_(j < 0 || j >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);

    return sc_fxnum_fast_subref(const_cast<sc_fxnum_fast &>(*this),
                                i - m_params.fwl(), j - m_params.fwl());
}

inline sc_fxnum_fast_subref
sc_fxnum_fast::range(int i, int j)
{
    SC_ERROR_IF_(i < 0 || i >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);
    SC_ERROR_IF_(j < 0 || j >= m_params.wl(), sc_core::SC_ID_OUT_OF_RANGE_);

    return sc_fxnum_fast_subref(*this, i - m_params.fwl(), j - m_params.fwl());
}

inline const sc_fxnum_fast_subref
sc_fxnum_fast::operator () () const
{
    return this->operator () (m_params.wl() - 1, 0);
}

inline sc_fxnum_fast_subref
sc_fxnum_fast::operator () ()
{
    return this->operator () (m_params.wl() - 1, 0);
}

inline const sc_fxnum_fast_subref
sc_fxnum_fast::range() const
{
    return this->range(m_params.wl() - 1, 0);
}

inline sc_fxnum_fast_subref
sc_fxnum_fast::range()
{
    return this->range(m_params.wl() - 1, 0);
}

// implicit conversion
inline sc_fxnum_fast::operator double() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    return m_val;
}

// explicit conversion to primitive types
inline short
sc_fxnum_fast::to_short() const
{
    // SC_FXNUM_FAST_OBSERVER_READ_ in to_uint64
    return static_cast<short>(to_uint64());
}

inline unsigned short
sc_fxnum_fast::to_ushort() const
{
    // SC_FXNUM_FAST_OBSERVER_READ_ in to_uint64
    return static_cast<unsigned short>(to_uint64());
}

inline int
sc_fxnum_fast::to_int() const
{
    // SC_FXNUM_FAST_OBSERVER_READ_ in to_uint64
    return static_cast<int>(to_uint64());
}

inline int64
sc_fxnum_fast::to_int64() const
{
    // SC_FXNUM_FAST_OBSERVER_READ_ in to_uint64
    return static_cast<int64>(to_uint64());
}

inline unsigned int
sc_fxnum_fast::to_uint() const
{
    // SC_FXNUM_FAST_OBSERVER_READ_ in to_uint64
    return static_cast<unsigned int>(to_uint64());
}

inline uint64
sc_fxnum_fast::to_uint64() const
{
    // SC_FXNUM_FAST_OBSERVER_READ_ in is_normal
    if (!is_normal()) {
        return 0;
    }

    int exponent;
    double mantissa_dbl = frexp(m_val, &exponent);

    uint64 mantissa = static_cast<uint64>(fabs(mantissa_dbl) *
                                          (UINT64_ONE << 53));
    exponent -= 53;

    if (!(-64 < exponent && exponent < 64)) {
        return 0;
    }

    mantissa = exponent >= 0 ? mantissa << exponent : mantissa >> -exponent;
    return mantissa_dbl >= 0 ? mantissa : -mantissa;
}

inline long
sc_fxnum_fast::to_long() const
{
    // SC_FXNUM_FAST_OBSERVER_READ_ in to_uint64
    return static_cast<long>(to_uint64());
}

inline unsigned long
sc_fxnum_fast::to_ulong() const
{
    // SC_FXNUM_FAST_OBSERVER_READ_ in to_uint64
    return static_cast<unsigned long>(to_uint64());
}

inline float
sc_fxnum_fast::to_float() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    return static_cast<float>(m_val);
}

inline double
sc_fxnum_fast::to_double() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    return m_val;
}

// query value
inline bool
sc_fxnum_fast::is_neg() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    scfx_ieee_double id(m_val);
    return (id.negative() != 0);
}

inline bool
sc_fxnum_fast::is_zero() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    scfx_ieee_double id(m_val);
    return id.is_zero();
}

// internal use only;
inline bool
sc_fxnum_fast::is_normal() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    scfx_ieee_double id(m_val);
    return (id.is_normal() || id.is_subnormal() || id.is_zero());
}

inline bool
sc_fxnum_fast::quantization_flag() const
{
    return m_q_flag;
}

inline bool
sc_fxnum_fast::overflow_flag() const
{
    return m_o_flag;
}

inline const sc_fxval_fast
sc_fxnum_fast::value() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this)
    return sc_fxval_fast(m_val);
}

// query parameters
inline int
sc_fxnum_fast::wl() const
{
    return m_params.wl();
}

inline int
sc_fxnum_fast::iwl() const
{
    return m_params.iwl();
}

inline sc_q_mode
sc_fxnum_fast::q_mode() const
{
    return m_params.q_mode();
}

inline sc_o_mode
sc_fxnum_fast::o_mode() const
{
    return m_params.o_mode();
}

inline int
sc_fxnum_fast::n_bits() const
{
    return m_params.n_bits();
}

inline const sc_fxtype_params &
sc_fxnum_fast::type_params() const
{
    return m_params.type_params();
}

inline const sc_fxcast_switch &
sc_fxnum_fast::cast_switch() const
{
    return m_params.cast_switch();
}

// internal use only;
inline void
sc_fxnum_fast::observer_read() const
{
    SC_FXNUM_FAST_OBSERVER_READ_(*this);
}

inline ::std::ostream &
operator << (::std::ostream &os, const sc_fxnum_fast &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_fxnum_fast &a)
{
    a.scan(is);
    return is;
}


// ----------------------------------------------------------------------------
// CLASS : sc_fxval
//
// Fixed-point value type; arbitrary precision.
// ----------------------------------------------------------------------------

// public constructors
inline sc_fxval::sc_fxval(const sc_fxnum &a, sc_fxval_observer *observer_) :
        m_rep(new scfx_rep(*a.get_rep())), m_observer(observer_)
{
    SC_FXVAL_OBSERVER_DEFAULT_
    SC_FXVAL_OBSERVER_CONSTRUCT_(*this)
    SC_FXVAL_OBSERVER_WRITE_(*this)
}

inline sc_fxval::sc_fxval(const sc_fxnum_fast &a,
                          sc_fxval_observer *observer_) :
    m_rep(new scfx_rep(a.to_double())), m_observer(observer_)
{
    SC_FXVAL_OBSERVER_DEFAULT_
    SC_FXVAL_OBSERVER_CONSTRUCT_(*this)
    SC_FXVAL_OBSERVER_WRITE_(*this)
}

// binary operators
#define DEFN_BIN_OP_T(op, fnc, tp) \
inline const sc_fxval \
operator op (const sc_fxval &a, tp b) \
{ \
    SC_FXVAL_OBSERVER_READ_(a) \
    sc_fxval tmp(b); \
    return sc_fxval(sc_dt::fnc ## _scfx_rep(*a.m_rep, *tmp.m_rep)); \
} \
 \
inline const sc_fxval \
operator op (tp a, const sc_fxval &b) \
{ \
    SC_FXVAL_OBSERVER_READ_(b) \
    sc_fxval tmp(a); \
    return sc_fxval(sc_dt::fnc ## _scfx_rep(*tmp.m_rep, *b.m_rep)); \
}

#define DEFN_BIN_OP(op, fnc) \
DEFN_BIN_OP_T(op, fnc, const sc_fxnum_fast &)

DEFN_BIN_OP(*, mult)
DEFN_BIN_OP(+, add)
DEFN_BIN_OP(-, sub)
//DEFN_BIN_OP(/, div)
DEFN_BIN_OP_T(/, div, const sc_fxnum_fast &)

#undef DEFN_BIN_OP_T
#undef DEFN_BIN_OP


// binary functions
#define DEFN_BIN_FNC_T(fnc, tp) \
inline void \
fnc (sc_fxval &c, const sc_fxval &a, tp b) \
{ \
    SC_FXVAL_OBSERVER_READ_(a) \
    sc_fxval tmp(b); \
    delete c.m_rep; \
    c.m_rep = sc_dt::fnc ## _scfx_rep(*a.m_rep, *tmp.m_rep); \
    SC_FXVAL_OBSERVER_WRITE_(c) \
} \
 \
inline void \
fnc (sc_fxval &c, tp a, const sc_fxval &b) \
{ \
    SC_FXVAL_OBSERVER_READ_(b) \
    sc_fxval tmp(a); \
    delete c.m_rep; \
    c.m_rep = sc_dt::fnc ## _scfx_rep(*tmp.m_rep, *b.m_rep); \
    SC_FXVAL_OBSERVER_WRITE_(c) \
}

#define DEFN_BIN_FNC(fnc) \
DEFN_BIN_FNC_T(fnc, const sc_fxnum_fast &)

DEFN_BIN_FNC(mult)
DEFN_BIN_FNC(div)
DEFN_BIN_FNC(add)
DEFN_BIN_FNC(sub)

#undef DEFN_BIN_FNC_T
#undef DEFN_BIN_FNC


// relational (including equality) operators
#define DEFN_REL_OP_T(op, ret, tp) \
inline bool \
operator op (const sc_fxval &a, tp b) \
{ \
    SC_FXVAL_OBSERVER_READ_(a) \
    sc_fxval tmp(b); \
    int result = sc_dt::cmp_scfx_rep(*a.m_rep, *tmp.m_rep); \
    return (ret); \
} \
 \
inline bool \
operator op (tp a, const sc_fxval &b) \
{ \
    SC_FXVAL_OBSERVER_READ_(b) \
    sc_fxval tmp(a); \
    int result = sc_dt::cmp_scfx_rep(*tmp.m_rep, *b.m_rep); \
    return (ret); \
}

#define DEFN_REL_OP(op, ret) \
DEFN_REL_OP_T(op, ret, const sc_fxnum_fast &)

DEFN_REL_OP(<, result < 0)
DEFN_REL_OP(<=, result <= 0)
DEFN_REL_OP(>, result > 0 && result != 2)
DEFN_REL_OP(>=, result >= 0 && result != 2)
DEFN_REL_OP(==, result == 0)
DEFN_REL_OP(!=, result != 0)

#undef DEFN_REL_OP_T
#undef DEFN_REL_OP

// assignment operators
inline sc_fxval &
sc_fxval::operator = (const sc_fxnum &a)
{
    *m_rep = *a.get_rep();
    SC_FXVAL_OBSERVER_WRITE_(*this)
    return *this;
}

#define DEFN_ASN_OP_T(tp) \
inline sc_fxval & \
sc_fxval::operator = (tp b) \
{ \
    sc_fxval tmp(b); \
    *m_rep = *tmp.m_rep; \
    SC_FXVAL_OBSERVER_WRITE_(*this) \
    return *this; \
}

DEFN_ASN_OP_T(const sc_fxnum_fast &)

#undef DEFN_ASN_OP_T

#define DEFN_ASN_OP_T(op, fnc, tp) \
inline sc_fxval & \
sc_fxval::operator op (tp b) \
{ \
    SC_FXVAL_OBSERVER_READ_(*this) \
    sc_fxval tmp(b); \
    scfx_rep *new_rep = sc_dt::fnc ## _scfx_rep(*m_rep, *tmp.m_rep); \
    delete m_rep; \
    m_rep = new_rep; \
    SC_FXVAL_OBSERVER_WRITE_(*this) \
    return *this; \
}

#define DEFN_ASN_OP(op, fnc) \
inline sc_fxval & \
sc_fxval::operator op (const sc_fxnum &b) \
{ \
    SC_FXVAL_OBSERVER_READ_(*this) \
    scfx_rep *new_rep = sc_dt::fnc ## _scfx_rep(*m_rep, *b.get_rep()); \
    delete m_rep; \
    m_rep = new_rep; \
    SC_FXVAL_OBSERVER_WRITE_(*this) \
    return *this; \
} \
 \
DEFN_ASN_OP_T(op, fnc, const sc_fxnum_fast &)

DEFN_ASN_OP(*=, mult)
DEFN_ASN_OP(/=, div)
DEFN_ASN_OP(+=, add)
DEFN_ASN_OP(-=, sub)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP


// ----------------------------------------------------------------------------
// CLASS : sc_fxval_fast
//
// Fixed-point value types; limited precision.
// ----------------------------------------------------------------------------

// public constructors

inline
sc_fxval_fast::sc_fxval_fast(const sc_fxnum &a,
                              sc_fxval_fast_observer *observer_) :
    m_val(a.to_double()), m_observer(observer_)
{
    SC_FXVAL_FAST_OBSERVER_DEFAULT_
    SC_FXVAL_FAST_OBSERVER_CONSTRUCT_(*this)
    SC_FXVAL_FAST_OBSERVER_WRITE_(*this)
}

inline sc_fxval_fast::sc_fxval_fast(const sc_fxnum_fast &a,
                                    sc_fxval_fast_observer *observer_) :
    m_val(a.get_val()), m_observer(observer_)
{
    SC_FXVAL_FAST_OBSERVER_DEFAULT_
    SC_FXVAL_FAST_OBSERVER_CONSTRUCT_(*this)
    SC_FXVAL_FAST_OBSERVER_WRITE_(*this)
}


// binary functions
#define DEFN_BIN_FNC_T(fnc, op, tp) \
inline void \
fnc (sc_fxval_fast &c, const sc_fxval_fast &a, tp b) \
{ \
    SC_FXVAL_FAST_OBSERVER_READ_(a) \
    sc_fxval_fast tmp(b); \
    c.m_val = a.m_val op tmp.m_val; \
    SC_FXVAL_FAST_OBSERVER_WRITE_(c) \
} \
 \
inline void \
fnc (sc_fxval_fast &c, tp a, const sc_fxval_fast &b) \
{ \
    SC_FXVAL_FAST_OBSERVER_READ_(b) \
    sc_fxval_fast tmp(a); \
    c.m_val = tmp.m_val op b.m_val; \
    SC_FXVAL_FAST_OBSERVER_WRITE_(c) \
}

#define DEFN_BIN_FNC(fnc, op) \
DEFN_BIN_FNC_T(fnc, op, const sc_fxval &) \
DEFN_BIN_FNC_T(fnc, op, const sc_fxnum &)

DEFN_BIN_FNC(mult, *)
DEFN_BIN_FNC(div, /)
DEFN_BIN_FNC(add, +)
DEFN_BIN_FNC(sub, -)

#undef DEFN_BIN_FNC_T
#undef DEFN_BIN_FNC


// assignment operators
inline sc_fxval_fast &
sc_fxval_fast::operator = (const sc_fxnum_fast &a)
{
    m_val = a.get_val();
    SC_FXVAL_FAST_OBSERVER_WRITE_(*this)
    return *this;
}

#define DEFN_ASN_OP_T(tp) \
inline sc_fxval_fast & \
sc_fxval_fast::operator = (tp a) \
{ \
    sc_fxval_fast tmp(a); \
    m_val = tmp.m_val; \
    SC_FXVAL_FAST_OBSERVER_WRITE_(*this) \
    return *this; \
}

DEFN_ASN_OP_T(const sc_fxnum &)

#undef DEFN_ASN_OP_T

#define DEFN_ASN_OP_T(op, tp) \
inline sc_fxval_fast & \
sc_fxval_fast::operator op (tp b) \
{ \
    SC_FXVAL_FAST_OBSERVER_READ_(*this) \
    sc_fxval_fast tmp(b); \
    m_val op tmp.m_val; \
    SC_FXVAL_FAST_OBSERVER_WRITE_(*this) \
    return *this; \
}

#define DEFN_ASN_OP(op) \
inline sc_fxval_fast & \
sc_fxval_fast::operator op (const sc_fxnum_fast &b) \
{ \
    SC_FXVAL_FAST_OBSERVER_READ_(*this) \
    m_val op b.get_val(); \
    SC_FXVAL_FAST_OBSERVER_WRITE_(*this) \
    return *this; \
} \
 \
DEFN_ASN_OP_T(op, const sc_fxnum &)

DEFN_ASN_OP(*=)
DEFN_ASN_OP(/=)
DEFN_ASN_OP(+=)
DEFN_ASN_OP(-=)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_FX_SC_FXNUM_HH__

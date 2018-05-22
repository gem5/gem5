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

  sc_unsigned.h -- Arbitrary precision unsigned arithmetic.

    This file includes the definitions of sc_unsigned_bitref,
    sc_unsigned_subref, and sc_unsigned classes. The first two classes
    are proxy classes to reference one bit and a range of bits of a
    sc_unsigned number, respectively.

    An sc_signed number has the sign-magnitude representation
    internally. However, its interface guarantees a 2's-complement
    representation. The sign-magnitude representation is chosen
    because of its efficiency: The sc_signed and sc_unsigned types are
    optimized for arithmetic rather than bitwise operations. For
    arithmetic operations, the sign-magnitude representation performs
    better.

    It is also important to note that an sc_unsigned number with n
    bits is equivalent to an sc_signed non-negative number with n + 1
    bits.

    The implementations of sc_signed and sc_unsigned classes are
    almost identical: Most of the member and friend functions are
    defined in sc_nbcommon.cpp and sc_nbfriends.cpp so that they can
    be shared by both of these classes. These functions are chosed by
    defining a few macros before including them such as IF_SC_SIGNED
    and CLASS_TYPE. Our implementation choices are mostly dictated by
    performance considerations in that we tried to provide the most
    efficient sc_signed and sc_unsigned types without compromising
    their interface.

    For the behavior of operators, we have two semantics: the old and
    new. The most important difference between these two semantics is
    that the old semantics is closer to C/C++ semantics in that the
    result type of a binary operator on unsigned and signed arguments
    is unsigned; the new semantics, on the other hand, requires the
    result type be signed. The new semantics is required by the VSIA
    C/C++ data types standard. We have implemented the new semantics.

  Original Author: Ali Dasdan, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_unsigned.h,v $
// Revision 1.4  2011/08/24 22:05:46  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.3  2011/02/18 20:19:15  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.2  2009/02/28 00:26:26  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.5  2006/05/08 17:50:02  acg
//   Andy Goodrich: Added David Long's declarations for friend operators,
//   functions, and methods, to keep the Microsoft compiler happy.
//
// Revision 1.4  2006/03/13 20:25:27  acg
//  Andy Goodrich: Addition of function declarations, e.g., xor_signed_friend()
//  to keep gcc 4.x happy.
//
// Revision 1.3  2006/01/13 18:49:32  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#ifndef __SYSTEMC_EXT_DT_INT_SC_UNSIGNED_HH__
#define __SYSTEMC_EXT_DT_INT_SC_UNSIGNED_HH__

#include <iostream>

#include "../misc/sc_value_base.hh"
#include "../sc_temporary.hh"
#include "sc_length_param.hh"
#include "sc_nbdefs.hh"
#include "sc_nbexterns.hh"
#include "sc_nbutils.hh"

namespace sc_dt
{

// classes defined in this module
class sc_unsigned_bitref_r;
class sc_unsigned_bitref;
class sc_unsigned_subref_r;
class sc_unsigned_subref;
class sc_concatref;
class sc_unsigned;

// forward class declarations
class sc_bv_base;
class sc_lv_base;
class sc_int_base;
class sc_uint_base;
class sc_int_subref_r;
class sc_uint_subref_r;
class sc_signed;
class sc_signed_subref_r;
class sc_fxval;
class sc_fxval_fast;
class sc_fxnum;
class sc_fxnum_fast;

} // namespace sc_dt

// extern template instantiations
namespace sc_core
{

extern template class sc_vpool<sc_dt::sc_unsigned_bitref>;
extern template class sc_vpool<sc_dt::sc_unsigned_subref>;
extern template class sc_vpool<sc_dt::sc_unsigned>;

} // namespace sc_core

namespace sc_dt
{

// Helper function declarions
int compare_unsigned(small_type us, int unb, int und, const sc_digit *ud,
                     small_type vs, int vnb, int vnd, const sc_digit *vd,
                     small_type if_u_signed=0, small_type if_v_signed=0);

sc_unsigned add_unsigned_friend(
        small_type us, int unb, int und, const sc_digit *ud,
        small_type vs, int vnb, int vnd, const sc_digit *vd);

sc_unsigned sub_unsigned_friend(
        small_type us, int unb, int und, const sc_digit *ud,
        small_type vs, int vnb, int vnd, const sc_digit *vd);

sc_unsigned mul_unsigned_friend(
        small_type s, int unb, int und, const sc_digit *ud,
        int vnb, int vnd, const sc_digit *vd);

sc_unsigned div_unsigned_friend(
        small_type s, int unb, int und, const sc_digit *ud,
        int vnb, int vnd, const sc_digit *vd);

sc_unsigned mod_unsigned_friend(
        small_type us, int unb, int und, const sc_digit *ud,
        int vnb, int vnd, const sc_digit *vd);

sc_unsigned and_unsigned_friend(
        small_type us, int unb, int und, const sc_digit *ud,
        small_type vs, int vnb, int vnd, const sc_digit *vd);


sc_unsigned or_unsigned_friend(
        small_type us, int unb, int und, const sc_digit *ud,
        small_type vs, int vnb, int vnd, const sc_digit *vd);

sc_unsigned xor_unsigned_friend(
        small_type us, int unb, int und, const sc_digit *ud,
        small_type vs, int vnb, int vnd, const sc_digit *vd);


/*
 * friend operator declarations
 */

// ARITHMETIC OPERATORS:

// ADDition operators:
sc_signed operator + (const sc_unsigned &u, const sc_signed &v);
sc_signed operator + (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator + (const sc_unsigned &u, const sc_unsigned &v);
sc_signed operator + (const sc_unsigned &u, int64 v);
sc_unsigned operator + (const sc_unsigned &u, uint64 v);
sc_signed operator + (const sc_unsigned &u, long v);
sc_unsigned operator + (const sc_unsigned &u, unsigned long v);
sc_signed operator + (const sc_unsigned &u, int v);
inline sc_unsigned operator + (const sc_unsigned &u, unsigned int v);

sc_signed operator + (int64 u, const sc_unsigned &v);
sc_unsigned operator + (uint64 u, const sc_unsigned &v);
sc_signed operator + (long u, const sc_unsigned &v);
sc_unsigned operator + (unsigned long u, const sc_unsigned &v);
sc_signed operator + (int u, const sc_unsigned &v);
inline sc_unsigned operator + (unsigned int u, const sc_unsigned &v);

sc_unsigned operator + (const sc_unsigned &u, const sc_uint_base &v);
sc_signed operator + (const sc_unsigned &u, const sc_int_base &v);
sc_unsigned operator + (const sc_uint_base &u, const sc_unsigned &v);
sc_signed operator + (const sc_int_base &u, const sc_unsigned &v);

// SUBtraction operators:
sc_signed operator - (const sc_unsigned &u, const sc_signed &v);
sc_signed operator - (const sc_signed &u, const sc_unsigned &v);

sc_signed operator - (const sc_unsigned &u, const sc_unsigned &v);
sc_signed operator - (const sc_unsigned &u, int64 v);
sc_signed operator - (const sc_unsigned &u, uint64 v);
sc_signed operator - (const sc_unsigned &u, long v);
sc_signed operator - (const sc_unsigned &u, unsigned long v);
sc_signed operator - (const sc_unsigned &u, int v);
sc_signed operator - (const sc_unsigned &u, unsigned int v);

sc_signed operator - (int64 u, const sc_unsigned &v);
sc_signed operator - (uint64 u, const sc_unsigned &v);
sc_signed operator - (long u, const sc_unsigned &v);
sc_signed operator - (unsigned long u, const sc_unsigned &v);
sc_signed operator - (int u, const sc_unsigned &v);
sc_signed operator - (unsigned int u, const sc_unsigned &v);

sc_signed operator - (const sc_unsigned &u, const sc_uint_base &v);
sc_signed operator - (const sc_unsigned &u, const sc_int_base &v);
sc_signed operator - (const sc_uint_base &u, const sc_unsigned &v);
sc_signed operator - (const sc_int_base &u, const sc_unsigned &v);

// MULtiplication operators:
sc_signed operator * (const sc_unsigned &u, const sc_signed &v);
sc_signed operator * (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator * (const sc_unsigned &u, const sc_unsigned &v);
sc_signed operator * (const sc_unsigned &u, int64 v);
sc_unsigned operator * (const sc_unsigned &u, uint64 v);
sc_signed operator * (const sc_unsigned &u, long v);
sc_unsigned operator * (const sc_unsigned &u, unsigned long v);
sc_signed operator * (const sc_unsigned &u, int v);
inline sc_unsigned operator * (const sc_unsigned &u, unsigned int v);

sc_signed operator * (int64 u, const sc_unsigned &v);
sc_unsigned operator * (uint64 u, const sc_unsigned &v);
sc_signed operator * (long u, const sc_unsigned &v);
sc_unsigned operator * (unsigned long u, const sc_unsigned &v);
sc_signed operator * (int u, const sc_unsigned &v);
inline sc_unsigned operator * (unsigned int u, const sc_unsigned &v);

sc_unsigned operator * (const sc_unsigned &u, const sc_uint_base &v);
sc_signed operator * (const sc_unsigned &u, const sc_int_base &v);
sc_unsigned operator * (const sc_uint_base &u, const sc_unsigned &v);
sc_signed operator * (const sc_int_base &u, const sc_unsigned &v);

// DIVision operators:
sc_signed operator / (const sc_unsigned &u, const sc_signed &v);
sc_signed operator / (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator / (const sc_unsigned &u, const sc_unsigned &v);
sc_signed operator / (const sc_unsigned &u, int64 v);
sc_unsigned operator / (const sc_unsigned &u, uint64 v);
sc_signed operator / (const sc_unsigned &u, long v);
sc_unsigned operator / (const sc_unsigned &u, unsigned long v);
sc_signed operator / (const sc_unsigned &u, int v);
inline sc_unsigned operator / (const sc_unsigned &u, unsigned int v);

sc_signed operator / (int64 u, const sc_unsigned &v);
sc_unsigned operator / (uint64 u, const sc_unsigned &v);
sc_signed operator / (long u, const sc_unsigned &v);
sc_unsigned operator / (unsigned long u, const sc_unsigned &v);
sc_signed operator / (int u, const sc_unsigned &v);
inline sc_unsigned operator / (unsigned int u, const sc_unsigned &v);

sc_unsigned operator / (const sc_unsigned &u, const sc_uint_base &v);
sc_signed operator / (const sc_unsigned &u, const sc_int_base &v);
sc_unsigned operator / (const sc_uint_base &u, const sc_unsigned &v);
sc_signed operator / (const sc_int_base &u, const sc_unsigned &v);

// MODulo operators:
sc_signed operator % (const sc_unsigned &u, const sc_signed &v);
sc_signed operator % (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator % (const sc_unsigned &u, const sc_unsigned &v);
sc_signed operator % (const sc_unsigned &u, int64 v);
sc_unsigned operator % (const sc_unsigned &u, uint64 v);
sc_signed operator % (const sc_unsigned &u, long v);
sc_unsigned operator % (const sc_unsigned &u, unsigned long v);
sc_signed operator % (const sc_unsigned &u, int v);
inline sc_unsigned operator % (const sc_unsigned &u, unsigned int v);

sc_signed operator % (int64 u, const sc_unsigned &v);
sc_unsigned operator % (uint64 u, const sc_unsigned &v);
sc_signed operator % (long u, const sc_unsigned &v);
sc_unsigned operator % (unsigned long u, const sc_unsigned &v);
sc_signed operator % (int u, const sc_unsigned &v);
inline sc_unsigned operator % (unsigned int u, const sc_unsigned &v);

sc_unsigned operator % (const sc_unsigned &u, const sc_uint_base &v);
sc_signed operator % (const sc_unsigned &u, const sc_int_base &v);
sc_unsigned operator % (const sc_uint_base &u, const sc_unsigned &v);
sc_signed operator % (const sc_int_base &u, const sc_unsigned &v);

// BITWISE OPERATORS:

// Bitwise AND operators:
sc_signed operator & (const sc_unsigned &u, const sc_signed &v);
sc_signed operator & (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator & (const sc_unsigned &u, const sc_unsigned &v);
sc_signed operator & (const sc_unsigned &u, int64 v);
sc_unsigned operator & (const sc_unsigned &u, uint64 v);
sc_signed operator & (const sc_unsigned &u, long v);
sc_unsigned operator & (const sc_unsigned &u, unsigned long v);
sc_signed operator & (const sc_unsigned &u, int v);
inline sc_unsigned operator & (const sc_unsigned &u, unsigned int v);

sc_signed operator & (int64 u, const sc_unsigned &v);
sc_unsigned operator & (uint64 u, const sc_unsigned &v);
sc_signed operator & (long u, const sc_unsigned &v);
sc_unsigned operator & (unsigned long u, const sc_unsigned &v);
sc_signed operator & (int u, const sc_unsigned &v);
inline sc_unsigned operator & (unsigned int u, const sc_unsigned &v);

sc_unsigned operator & (const sc_unsigned &u, const sc_uint_base &v);
sc_signed operator & (const sc_unsigned &u, const sc_int_base &v);
sc_unsigned operator & (const sc_uint_base &u, const sc_unsigned &v);
sc_signed operator & (const sc_int_base &u, const sc_unsigned &v);

// Bitwise OR operators:
sc_signed operator | (const sc_unsigned &u, const sc_signed &v);
sc_signed operator | (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator | (const sc_unsigned &u, const sc_unsigned &v);
sc_signed operator | (const sc_unsigned &u, int64 v);
sc_unsigned operator | (const sc_unsigned &u, uint64 v);
sc_signed operator | (const sc_unsigned &u, long v);
sc_unsigned operator | (const sc_unsigned &u, unsigned long v);
sc_signed operator | (const sc_unsigned &u, int v);
inline sc_unsigned operator | (const sc_unsigned &u, unsigned int v);

sc_signed operator | (int64 u, const sc_unsigned &v);
sc_unsigned operator | (uint64 u, const sc_unsigned &v);
sc_signed operator | (long u, const sc_unsigned &v);
sc_unsigned operator | (unsigned long u, const sc_unsigned &v);
sc_signed operator | (int u, const sc_unsigned &v);
inline sc_unsigned operator | (unsigned int u, const sc_unsigned &v);

sc_unsigned operator | (const sc_unsigned &u, const sc_uint_base &v);
sc_signed operator | (const sc_unsigned &u, const sc_int_base &v);
sc_unsigned operator | (const sc_uint_base &u, const sc_unsigned &v);
sc_signed operator | (const sc_int_base &u, const sc_unsigned &v);

// Bitwise XOR operators:
sc_signed operator ^ (const sc_unsigned &u, const sc_signed &v);
sc_signed operator ^ (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator ^ (const sc_unsigned &u, const sc_unsigned &v);
sc_signed operator ^ (const sc_unsigned &u, int64 v);
sc_unsigned operator ^ (const sc_unsigned &u, uint64 v);
sc_signed operator ^ (const sc_unsigned &u, long v);
sc_unsigned operator ^ (const sc_unsigned &u, unsigned long v);
sc_signed operator ^ (const sc_unsigned &u, int v);
inline sc_unsigned operator ^ (const sc_unsigned &u, unsigned int v);

sc_signed operator ^ (int64 u, const sc_unsigned &v);
sc_unsigned operator ^ (uint64 u, const sc_unsigned &v);
sc_signed operator ^ (long u, const sc_unsigned &v);
sc_unsigned operator ^ (unsigned long u, const sc_unsigned &v);
sc_signed operator ^ (int u, const sc_unsigned &v);
inline sc_unsigned operator ^ (unsigned int u, const sc_unsigned &v);

sc_unsigned operator ^ (const sc_unsigned &u, const sc_uint_base &v);
sc_signed operator ^ (const sc_unsigned &u, const sc_int_base &v);
sc_unsigned operator ^ (const sc_uint_base &u, const sc_unsigned &v);
sc_signed operator ^ (const sc_int_base &u, const sc_unsigned &v);

// SHIFT OPERATORS:

// LEFT SHIFT operators:
sc_unsigned operator << (const sc_unsigned &u, const sc_signed &v);
sc_signed operator << (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator << (const sc_unsigned &u, const sc_unsigned &v);
sc_unsigned operator << (const sc_unsigned &u, int64 v);
sc_unsigned operator << (const sc_unsigned &u, uint64 v);
sc_unsigned operator << (const sc_unsigned &u, long v);
sc_unsigned operator << (const sc_unsigned &u, unsigned long v);
inline sc_unsigned operator << (const sc_unsigned &u, int v);
inline sc_unsigned operator << (const sc_unsigned &u, unsigned int v);

sc_unsigned operator << (const sc_unsigned &u, const sc_uint_base &v);
sc_unsigned operator << (const sc_unsigned &u, const sc_int_base &v);

// RIGHT SHIFT operators:
sc_unsigned operator >> (const sc_unsigned &u, const sc_signed &v);
sc_signed operator >> (const sc_signed &u, const sc_unsigned &v);

sc_unsigned operator >> (const sc_unsigned &u, const sc_unsigned &v);
sc_unsigned operator >> (const sc_unsigned &u, int64 v);
sc_unsigned operator >> (const sc_unsigned &u, uint64 v);
sc_unsigned operator >> (const sc_unsigned &u, long v);
sc_unsigned operator >> (const sc_unsigned &u, unsigned long v);
inline sc_unsigned operator >> (const sc_unsigned &u, int v);
inline sc_unsigned operator >> (const sc_unsigned &u, unsigned int v);

sc_unsigned operator >> ( const sc_unsigned &, const sc_uint_base &);
sc_unsigned operator >> ( const sc_unsigned&, const sc_int_base &);

// Unary arithmetic operators
sc_unsigned operator + (const sc_unsigned &u);
sc_signed operator - (const sc_unsigned &u);

// LOGICAL OPERATORS:

// Logical EQUAL operators:
bool operator == (const sc_unsigned &u, const sc_signed &v);
bool operator == (const sc_signed &u, const sc_unsigned &v);

bool operator == (const sc_unsigned &u, const sc_unsigned &v);
bool operator == (const sc_unsigned &u, int64 v);
bool operator == (const sc_unsigned &u, uint64 v);
bool operator == (const sc_unsigned &u, long v);
bool operator == (const sc_unsigned &u, unsigned long v);
inline bool operator == (const sc_unsigned &u, int v);
inline bool operator == (const sc_unsigned &u, unsigned int v);

bool operator == (int64 u, const sc_unsigned &v);
bool operator == (uint64 u, const sc_unsigned &v);
bool operator == (long u, const sc_unsigned &v);
bool operator == (unsigned long u, const sc_unsigned &v);
inline bool operator == (int u, const sc_unsigned &v);
inline bool operator == (unsigned int u, const sc_unsigned &v) ;

bool operator == (const sc_unsigned &u, const sc_uint_base &v);
bool operator == (const sc_unsigned &u, const sc_int_base &v);
bool operator == (const sc_uint_base &u, const sc_unsigned &v);
bool operator == (const sc_int_base &u, const sc_unsigned &v);

// Logical NOT_EQUAL operators:
bool operator != (const sc_unsigned &u, const sc_signed &v);
bool operator != (const sc_signed &u, const sc_unsigned &v);

bool operator != (const sc_unsigned &u, const sc_unsigned &v);
bool operator != (const sc_unsigned &u, int64 v);
bool operator != (const sc_unsigned &u, uint64 v);
bool operator != (const sc_unsigned &u, long v);
bool operator != (const sc_unsigned &u, unsigned long v);
inline bool operator != (const sc_unsigned &u, int v);
inline bool operator != (const sc_unsigned &u, unsigned int v);

bool operator != (int64 u, const sc_unsigned &v);
bool operator != (uint64 u, const sc_unsigned &v);
bool operator != (long u, const sc_unsigned &v);
bool operator != (unsigned long u, const sc_unsigned &v);
inline bool operator != (int u, const sc_unsigned &v);
inline bool operator != (unsigned int u, const sc_unsigned &v);

bool operator != (const sc_unsigned &u, const sc_uint_base &v);
bool operator != (const sc_unsigned &u, const sc_int_base &v);
bool operator != (const sc_uint_base &u, const sc_unsigned &v);
bool operator != (const sc_int_base &u, const sc_unsigned &v);

// Logical LESS_THAN operators:
bool operator < (const sc_unsigned &u, const sc_signed &v);
bool operator < (const sc_signed &u, const sc_unsigned &v);

bool operator < (const sc_unsigned &u, const sc_unsigned &v);
bool operator < (const sc_unsigned &u, int64 v);
bool operator < (const sc_unsigned &u, uint64 v);
bool operator < (const sc_unsigned &u, long v);
bool operator < (const sc_unsigned &u, unsigned long v);
inline bool operator < (const sc_unsigned &u, int v);
inline bool operator < (const sc_unsigned &u, unsigned int v);

bool operator < (int64 u, const sc_unsigned &v);
bool operator < (uint64 u, const sc_unsigned &v);
bool operator < (long u, const sc_unsigned &v);
bool operator < (unsigned long u, const sc_unsigned &v);
inline bool operator < (int u, const sc_unsigned &v);
inline bool operator < (unsigned int u, const sc_unsigned &v);

bool operator < (const sc_unsigned &u, const sc_uint_base &v);
bool operator < (const sc_unsigned &u, const sc_int_base &v);
bool operator < (const sc_uint_base &u, const sc_unsigned &v);
bool operator < (const sc_int_base &u, const sc_unsigned &v);

// Logical LESS_THAN_AND_EQUAL operators:
bool operator <= (const sc_unsigned &u, const sc_signed &v);
bool operator <= (const sc_signed &u, const sc_unsigned &v);

bool operator <= (const sc_unsigned &u, const sc_unsigned &v);
bool operator <= (const sc_unsigned &u, int64 v);
bool operator <= (const sc_unsigned &u, uint64 v);
bool operator <= (const sc_unsigned &u, long v);
bool operator <= (const sc_unsigned &u, unsigned long v);
inline bool operator <= (const sc_unsigned &u, int v);
inline bool operator <= (const sc_unsigned &u, unsigned int v);

bool operator <= (int64 u, const sc_unsigned &v);
bool operator <= (uint64 u, const sc_unsigned &v);
bool operator <= (long u, const sc_unsigned &v);
bool operator <= (unsigned long u, const sc_unsigned &v);
inline bool operator <= (int u, const sc_unsigned &v);
inline bool operator <= (unsigned int u, const sc_unsigned &v);

bool operator <= (const sc_unsigned &u, const sc_uint_base &v);
bool operator <= (const sc_unsigned &u, const sc_int_base &v);
bool operator <= (const sc_uint_base &u, const sc_unsigned &v);
bool operator <= (const sc_int_base &u, const sc_unsigned &v);

// Logical GREATER_THAN operators:
bool operator > (const sc_unsigned &u, const sc_signed &v);
bool operator > (const sc_signed &u, const sc_unsigned &v);

bool operator > (const sc_unsigned &u, const sc_unsigned &v);
bool operator > (const sc_unsigned &u, int64 v);
bool operator > (const sc_unsigned &u, uint64 v);
bool operator > (const sc_unsigned &u, long v);
bool operator > (const sc_unsigned &u, unsigned long v);
inline bool operator > (const sc_unsigned &u, int v);
inline bool operator > (const sc_unsigned &u, unsigned int v);

bool operator > (int64 u, const sc_unsigned &v);
bool operator > (uint64 u, const sc_unsigned &v);
bool operator > (long u, const sc_unsigned &v);
bool operator > (unsigned long u, const sc_unsigned &v);
inline bool operator > (int u, const sc_unsigned &v);
inline bool operator > (unsigned int u, const sc_unsigned &v);

bool operator > (const sc_unsigned &u, const sc_uint_base &v);
bool operator > (const sc_unsigned &u, const sc_int_base &v);
bool operator > (const sc_uint_base &u, const sc_unsigned &v);
bool operator > (const sc_int_base &u, const sc_unsigned &v);

// Logical GREATER_THAN_AND_EQUAL operators:
bool operator >= (const sc_unsigned &u, const sc_signed &v);
bool operator >= (const sc_signed &u, const sc_unsigned &v);

bool operator >= (const sc_unsigned &u, const sc_unsigned &v);
bool operator >= (const sc_unsigned &u, int64 v);
bool operator >= (const sc_unsigned &u, uint64 v);
bool operator >= (const sc_unsigned &u, long v);
bool operator >= (const sc_unsigned &u, unsigned long v);
inline bool operator >= (const sc_unsigned &u, int v);
inline bool operator >= (const sc_unsigned &u, unsigned int v);

bool operator >= (int64 u, const sc_unsigned &v);
bool operator >= (uint64 u, const sc_unsigned &v);
bool operator >= (long u, const sc_unsigned &v);
bool operator >= (unsigned long u, const sc_unsigned &v);
inline bool operator >= (int u, const sc_unsigned &v);
inline bool operator >= (unsigned int u, const sc_unsigned &v);

bool operator >= (const sc_unsigned &u, const sc_uint_base &v);
bool operator >= (const sc_unsigned &u, const sc_int_base &v);
bool operator >= (const sc_uint_base &u, const sc_unsigned &v);
bool operator >= (const sc_int_base &u, const sc_unsigned &v);

// Bitwise NOT operator (unary).
sc_unsigned operator ~ (const sc_unsigned &u);

// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_bitref_r
//
//  Proxy class for sc_unsigned bit selection (r-value only).
// ----------------------------------------------------------------------------

class sc_unsigned_bitref_r : public sc_value_base
{
    friend class sc_unsigned;

  protected:
    // construction and initialization:
    sc_unsigned_bitref_r() : sc_value_base(), m_index(0), m_obj_p(0) {}

    void
    initialize(const sc_unsigned *obj_p, int index_)
    {
        m_obj_p = const_cast<sc_unsigned *>(obj_p);
        m_index = index_;
    }

  public:
    // destructor
    virtual ~sc_unsigned_bitref_r() {}

    // copy constructor
    sc_unsigned_bitref_r(const sc_unsigned_bitref_r &a) :
        sc_value_base(a), m_index(a.m_index), m_obj_p(a.m_obj_p)
    {}

    // capacity
    int length() const { return 1; }

    // implicit conversion to bool
    operator uint64 () const;
    bool operator ! () const;
    bool operator ~ () const;

    // explicit conversions
    uint64 value() const { return operator uint64(); }
    bool to_bool() const { return operator uint64(); }

    // concatenation support
    virtual int
    concat_length(bool *xz_present_p) const
    {
        if (xz_present_p)
            *xz_present_p = false;
        return 1;
    }
    virtual uint64
    concat_get_uint64() const
    {
        return (uint64)operator uint64();
    }
    virtual bool
    concat_get_ctrl(sc_digit *dst_p, int low_i) const
    {
        int bit_mask = 1 << (low_i % BITS_PER_DIGIT);
        int word_i = low_i / BITS_PER_DIGIT;
        dst_p[word_i] &= ~bit_mask;
        return false;
    }
    virtual bool
    concat_get_data(sc_digit *dst_p, int low_i) const
    {
        int bit_mask = 1 << (low_i % BITS_PER_DIGIT);
        bool result; // True if non-zero.
        int word_i = low_i / BITS_PER_DIGIT;
        if (operator uint64())
        {
            dst_p[word_i] |= bit_mask;
            result = true;
        } else {
            dst_p[word_i] &= ~bit_mask;
            result = false;
        }
        return result;
    }

    // other methods
    void print(::std::ostream &os=::std::cout) const { os << to_bool(); }

  protected:
    int m_index;
    sc_unsigned *m_obj_p;

  private:
    // Disabled
    const sc_unsigned_bitref_r &operator = (const sc_unsigned_bitref_r &);
};

inline ::std::ostream &operator << (
        ::std::ostream &, const sc_unsigned_bitref_r &);


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_bitref
//
//  Proxy class for sc_unsigned bit selection (r-value and l-value).
// ----------------------------------------------------------------------------

class sc_unsigned_bitref : public sc_unsigned_bitref_r
{
    friend class sc_unsigned;
    friend class sc_core::sc_vpool<sc_unsigned_bitref>;

  protected: // construction
    sc_unsigned_bitref() : sc_unsigned_bitref_r() {}

  public:
    // copy constructor
    sc_unsigned_bitref(const sc_unsigned_bitref &a) : sc_unsigned_bitref_r(a)
    {}

    // assignment operators
    const sc_unsigned_bitref &operator = (const sc_unsigned_bitref_r &);
    const sc_unsigned_bitref &operator = (const sc_unsigned_bitref &);
    const sc_unsigned_bitref &operator = (bool);

    const sc_unsigned_bitref &operator &= (bool);
    const sc_unsigned_bitref &operator |= (bool);
    const sc_unsigned_bitref &operator ^= (bool);

    // concatenation methods
    virtual void concat_set(int64 src, int low_i);
    virtual void concat_set(const sc_signed &src, int low_i);
    virtual void concat_set(const sc_unsigned &src, int low_i);
    virtual void concat_set(uint64 src, int low_i);

    // other methods
    void scan(::std::istream &is=::std::cin);

  protected:
    static sc_core::sc_vpool<sc_unsigned_bitref> m_pool;
};

inline ::std::istream &operator >> (::std::istream &, sc_unsigned_bitref &);


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_subref_r
//
//  Proxy class for sc_unsigned part selection (r-value only).
// ----------------------------------------------------------------------------

class sc_unsigned_subref_r : public sc_value_base
{
    friend class sc_signed;
    friend class sc_unsigned;
    friend class sc_unsigned_signal;

  protected:
    // constructor
    sc_unsigned_subref_r() : sc_value_base(), m_left(0), m_obj_p(0), m_right(0)
    {}

    void
    initialize(const sc_unsigned *obj_p, int left_, int right_)
    {
        m_obj_p = const_cast<sc_unsigned *>(obj_p);
        m_left = left_;
        m_right = right_;
    }

  public:
    // destructor
    virtual ~sc_unsigned_subref_r() {}

    // copy constructor
    sc_unsigned_subref_r(const sc_unsigned_subref_r &a) :
            sc_value_base(a), m_left(a.m_left), m_obj_p(a.m_obj_p),
            m_right(a.m_right)
    {}

    // capacity
    int
    length() const
    {
        if (m_left >= m_right)
            return m_left - m_right + 1;
        else
            return m_right - m_left + 1;
    }

    // implicit conversion to sc_unsigned
    operator sc_unsigned () const;

    // explicit conversions
    int to_int() const;
    unsigned int to_uint() const;
    long to_long() const;
    unsigned long to_ulong() const;
    int64 to_int64() const;
    uint64 to_uint64() const;
    double to_double() const;

    // explicit conversion to character string
    const std::string to_string(sc_numrep numrep=SC_DEC) const;
    const std::string to_string(sc_numrep numrep, bool w_prefix) const;

    // concatenation support
    virtual int concat_length(bool *xz_present_p) const
    {
        if (xz_present_p)
            *xz_present_p = false;
        return m_left - m_right + 1;
    }
    virtual uint64 concat_get_uint64() const;
    virtual bool concat_get_ctrl(sc_digit *dst_p, int low_i) const;
    virtual bool concat_get_data(sc_digit *dst_p, int low_i) const;

    // reduce methods
    bool and_reduce() const;
    bool nand_reduce() const;
    bool or_reduce() const;
    bool nor_reduce() const;
    bool xor_reduce() const ;
    bool xnor_reduce() const;

    // other methods
    void
    print(::std::ostream &os=::std::cout) const
    {
        os << to_string(sc_io_base(os, SC_DEC), sc_io_show_base(os));
    }

  protected:
    int m_left; // Left-most bit in this part selection.
    sc_unsigned *m_obj_p; // Target of this part selection.
    int m_right; // Right-most bit in this part selection.

  private:
    // Disabled
    const sc_unsigned_subref_r &operator = (const sc_unsigned_subref_r &);
};

inline ::std::ostream &operator << (
        ::std::ostream &, const sc_unsigned_subref_r &);


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_subref
//
//  Proxy class for sc_unsigned part selection (r-value and l-value).
// ----------------------------------------------------------------------------

class sc_unsigned_subref : public sc_unsigned_subref_r
{
    friend class sc_unsigned;
    friend class sc_core::sc_vpool<sc_unsigned_subref>;

    // constructor
  protected:
    sc_unsigned_subref() : sc_unsigned_subref_r() {}

  public:
    // copy constructor
    sc_unsigned_subref(const sc_unsigned_subref &a) : sc_unsigned_subref_r(a)
    {}

    // assignment operators
    const sc_unsigned_subref &operator = (const sc_unsigned_subref_r &a);
    const sc_unsigned_subref &operator = (const sc_unsigned_subref &a);
    const sc_unsigned_subref &operator = (const sc_unsigned &a);

    template<class T>
    const sc_unsigned_subref &operator = (const sc_generic_base<T> &a);
    const sc_unsigned_subref &operator = (const sc_signed_subref_r &a);
    const sc_unsigned_subref &operator = (const sc_signed &a);

    const sc_unsigned_subref &operator = (const char *a);
    const sc_unsigned_subref &operator = (unsigned long a);
    const sc_unsigned_subref &operator = (long a);

    const sc_unsigned_subref &
    operator = (unsigned int a)
    {
        return operator = ((unsigned long)a);
    }

    const sc_unsigned_subref &
    operator = (int a)
    {
        return operator = ((long)a);
    }

    const sc_unsigned_subref &operator = (uint64 a);
    const sc_unsigned_subref &operator = (int64 a);
    const sc_unsigned_subref &operator = (double a);
    const sc_unsigned_subref &operator = (const sc_int_base &a);
    const sc_unsigned_subref &operator = (const sc_uint_base &a);

    // concatenation methods
    virtual void concat_set(int64 src, int low_i);
    virtual void concat_set(const sc_signed &src, int low_i);
    virtual void concat_set(const sc_unsigned &src, int low_i);
    virtual void concat_set(uint64 src, int low_i);

    // other methods
    void scan(::std::istream &is=::std::cin);

  protected:
    static sc_core::sc_vpool<sc_unsigned_subref> m_pool;
};

inline ::std::istream &operator >> (::std::istream &, sc_unsigned_subref &);


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned
//
//  Arbitrary precision unsigned number.
// ----------------------------------------------------------------------------

class sc_unsigned : public sc_value_base
{
    friend class sc_concatref;
    friend class sc_unsigned_bitref_r;
    friend class sc_unsigned_bitref;
    friend class sc_unsigned_subref_r;
    friend class sc_unsigned_subref;
    friend class sc_signed;
    friend class sc_signed_subref;
    friend class sc_signed_subref_r;

    // Needed for types using sc_unsigned.
    typedef bool elemtype;

    void invalid_init(const char *type_name, int nb) const;

  public:
    // constructors
    explicit sc_unsigned(int nb=sc_length_param().len());
    sc_unsigned(const sc_unsigned &v);
    sc_unsigned(const sc_signed &v);
    template<class T>
    explicit sc_unsigned(const sc_generic_base<T> &v);
    explicit sc_unsigned(const sc_bv_base &v);
    explicit sc_unsigned(const sc_lv_base &v);
    explicit sc_unsigned(const sc_int_subref_r &v);
    explicit sc_unsigned(const sc_uint_subref_r &v);
    explicit sc_unsigned(const sc_signed_subref_r &v);
    explicit sc_unsigned(const sc_unsigned_subref_r &v);

    // assignment operators
    const sc_unsigned &operator = (const sc_unsigned &v);
    const sc_unsigned &operator = (const sc_unsigned_subref_r &a);

    template<class T>
    const sc_unsigned &
    operator = (const sc_generic_base<T> &a)
    {
        a->to_sc_unsigned(*this);
        return *this;
    }

    const sc_unsigned &operator = (const sc_signed &v);
    const sc_unsigned &operator = (const sc_signed_subref_r &a);

    const sc_unsigned &operator = (const char *v);
    const sc_unsigned &operator = (int64 v);
    const sc_unsigned &operator = (uint64 v);
    const sc_unsigned &operator = (long v);
    const sc_unsigned &operator = (unsigned long v);

    const sc_unsigned &
    operator = (int v)
    {
        return operator = ((long)v);
    }

    const sc_unsigned &
    operator = (unsigned int v)
    {
        return operator = ((unsigned long)v);
    }

    const sc_unsigned &operator = (double v);
    const sc_unsigned &operator = (const sc_int_base &v);
    const sc_unsigned &operator = (const sc_uint_base &v);

    const sc_unsigned &operator = (const sc_bv_base &);
    const sc_unsigned &operator = (const sc_lv_base &);

    const sc_unsigned &operator = (const sc_fxval &);
    const sc_unsigned &operator = (const sc_fxval_fast &);
    const sc_unsigned &operator = (const sc_fxnum &);
    const sc_unsigned &operator = (const sc_fxnum_fast &);

    // destructor
    virtual ~sc_unsigned()
    {
#       ifndef SC_MAX_NBITS
            delete [] digit;
#       endif
    }

    // Concatenation support:
    sc_digit *get_raw() const { return digit; }
    virtual int
    concat_length(bool *xz_present_p) const
    {
        if (xz_present_p)
            *xz_present_p = false;
        return nbits - 1;
    }
    virtual bool concat_get_ctrl(sc_digit *dst_p, int low_i) const;
    virtual bool concat_get_data(sc_digit *dst_p, int low_i) const;
    virtual uint64 concat_get_uint64() const;
    virtual void concat_set(int64 src, int low_i);
    virtual void concat_set(const sc_signed &src, int low_i);
    virtual void concat_set(const sc_unsigned &src, int low_i);
    virtual void concat_set(uint64 src, int low_i);

    // Increment operators.
    sc_unsigned &operator ++ ();
    const sc_unsigned operator ++ (int);

    // Decrement operators.
    sc_unsigned &operator -- ();
    const sc_unsigned operator -- (int);

    // bit selection
    inline void
    check_index(int i) const
    {
        if ((i < 0) || (i >= nbits - 1))
            invalid_index(i);
    }

    void invalid_index(int i) const;

    sc_unsigned_bitref &
    operator [] (int i)
    {
        check_index(i);
        sc_unsigned_bitref *result_p = sc_unsigned_bitref::m_pool.allocate();
        result_p->initialize(this, i);
        return *result_p;
    }

    const sc_unsigned_bitref_r &
    operator [] (int i) const
    {
        check_index(i);
        sc_unsigned_bitref *result_p = sc_unsigned_bitref::m_pool.allocate();
        result_p->initialize(this, i);
        return *result_p;
    }

    sc_unsigned_bitref &
    bit(int i)
    {
        check_index(i);
        sc_unsigned_bitref *result_p = sc_unsigned_bitref::m_pool.allocate();
        result_p->initialize(this, i);
        return *result_p;
    }

    const sc_unsigned_bitref_r &
    bit(int i) const
    {
        check_index(i);
        sc_unsigned_bitref *result_p = sc_unsigned_bitref::m_pool.allocate();
        result_p->initialize(this, i);
        return *result_p;
    }

    // part selection

    // Subref operators. Help access the range of bits from the ith to
    // jth. These indices have arbitrary precedence with respect to each
    // other, i.e., we can have i <= j or i > j. Note the equivalence
    // between range(i, j) and operator (i, j). Also note that
    // operator (i, i) returns an unsigned number that corresponds to the
    // bit operator [i], so these two forms are not the same.
    inline void
    check_range(int l, int r) const
    {
        if (l < r) {
            if ((l < 0) || (r >= nbits - 1))
                invalid_range(l, r);
        } else {
            if ((r < 0) || (l >= nbits - 1))
                invalid_range(l, r);
        }
    }

    void invalid_range(int l, int r) const;

    sc_unsigned_subref &
    range(int i, int j)
    {
        check_range(i, j);
        sc_unsigned_subref *result_p = sc_unsigned_subref::m_pool.allocate();
        result_p->initialize(this, i, j);
        return *result_p;
    }

    const sc_unsigned_subref_r &
    range(int i, int j) const
    {
        check_range(i, j);
        sc_unsigned_subref *result_p = sc_unsigned_subref::m_pool.allocate();
        result_p->initialize(this, i, j);
        return *result_p;
    }

    sc_unsigned_subref &
    operator () (int i, int j)
    {
        check_range(i,j);
        sc_unsigned_subref *result_p = sc_unsigned_subref::m_pool.allocate();
        result_p->initialize(this, i, j);
        return *result_p;
    }

    const sc_unsigned_subref_r &
    operator () (int i, int j) const
    {
        check_range(i,j);
        sc_unsigned_subref *result_p = sc_unsigned_subref::m_pool.allocate();
        result_p->initialize(this, i, j);
        return *result_p;
    }

    // explicit conversions
    int to_int() const;
    unsigned int to_uint() const;
    long to_long() const;
    unsigned long to_ulong() const;
    int64 to_int64() const;
    uint64 to_uint64() const;
    double to_double() const;

    // explicit conversion to character string
    const std::string to_string(sc_numrep numrep=SC_DEC) const;
    const std::string to_string(sc_numrep numrep, bool w_prefix) const;

    // Print functions. dump prints the internals of the class.
    void
    print(::std::ostream &os=::std::cout) const
    {
        os << to_string(sc_io_base(os, SC_DEC), sc_io_show_base(os));
    }

    void scan(::std::istream &is=::std::cin);
    void dump(::std::ostream &os=::std::cout) const;

    // Functions to find various properties.
    int length() const { return nbits - 1; } // Bit width.
    bool iszero() const; // Is the number zero?
    bool sign() const { return 0; } // Sign.

    // reduce methods
    bool and_reduce() const;
    bool nand_reduce() const { return !and_reduce(); }
    bool or_reduce() const;
    bool nor_reduce() const { return !or_reduce(); }
    bool xor_reduce() const;
    bool xnor_reduce() const { return !xor_reduce(); }

    // Functions to access individual bits.
    bool test(int i) const; // Is the ith bit 0 or 1?
    void set(int i); // Set the ith bit to 1.
    void clear(int i); // Set the ith bit to 0.
    void
    set(int i, bool v) // Set the ith bit to v.
    {
        if (v)
            set(i);
        else
            clear(i);
    }
    void
    invert(int i)           // Negate the ith bit.
    {
        if (test(i))
            clear(i);
        else
            set(i);
    }

    // Make the number equal to its mirror image.
    void reverse();

    // Get/set a packed bit representation of the number.
    void get_packed_rep(sc_digit *buf) const;
    void set_packed_rep(sc_digit *buf);

    /*
      The comparison of the old and new semantics are as follows:

      Let s = sc_signed,
          u = sc_unsigned,
          un = { uint64, unsigned long, unsigned int },
          sn = { int64, long, int, char* }, and
          OP = { +, -, *, /, % }.

      Old semantics:                     New semantics:
        u OP u -> u                        u OP u -> u
        s OP u -> u                        s OP u -> s
        u OP s -> u                        u OP s -> s
        s OP s -> s                        s OP s -> s

        u OP un = un OP u -> u             u OP un = un OP u -> u
        u OP sn = sn OP u -> u             u OP sn = sn OP u -> s

        s OP un = un OP s -> s             s OP un = un OP s -> s
        s OP sn = sn OP s -> s             s OP sn = sn OP s -> s

      In the new semantics, the result is u if both operands are u; the
      result is s otherwise. The only exception is subtraction. The result
      of a subtraction is always s.

      The old semantics is like C/C++ semantics on integer types; the
      new semantics is due to the VSIA C/C++ data types standard.
    */

    // ARITHMETIC OPERATORS:

    // ADDition operators:
    friend sc_signed operator + (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator + (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator + (const sc_unsigned &u, const sc_unsigned &v);
    friend sc_signed operator + (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator + (const sc_unsigned &u, uint64 v);
    friend sc_signed operator + (const sc_unsigned &u, long v);
    friend sc_unsigned operator + (const sc_unsigned &u, unsigned long v);
    friend sc_signed operator + (const sc_unsigned &u, int v);
    friend sc_unsigned
    operator + (const sc_unsigned &u, unsigned int v)
    {
        return operator + (u, (unsigned long)v);
    }

    friend sc_signed operator + (int64 u, const sc_unsigned &v);
    friend sc_unsigned operator + (uint64 u, const sc_unsigned &v);
    friend sc_signed operator + (long u, const sc_unsigned &v);
    friend sc_unsigned operator + (unsigned long u, const sc_unsigned &v);
    friend sc_signed operator + (int u, const sc_unsigned &v);
    friend sc_unsigned
    operator + (unsigned int u, const sc_unsigned &v)
    {
        return operator + ((unsigned long)u, v);
    }

    const sc_unsigned &operator += (const sc_signed &v);
    const sc_unsigned &operator += (const sc_unsigned &v);
    const sc_unsigned &operator += (int64 v);
    const sc_unsigned &operator += (uint64 v);
    const sc_unsigned &operator += (long v);
    const sc_unsigned &operator += (unsigned long v);
    const sc_unsigned &
    operator += (int v)
    {
        return operator += ((long)v);
    }
    const sc_unsigned &
    operator += (unsigned int v)
    {
        return operator += ((unsigned long)v);
    }

    friend sc_unsigned operator + (
            const sc_unsigned &u, const sc_uint_base &v);
    friend sc_signed operator + (const sc_unsigned &u, const sc_int_base &v);
    friend sc_unsigned operator + (
            const sc_uint_base &u, const sc_unsigned &v);
    friend sc_signed operator + (const sc_int_base &u, const sc_unsigned &v);
    const sc_unsigned &operator += (const sc_int_base &v);
    const sc_unsigned &operator += (const sc_uint_base &v);

    // SUBtraction operators:
    friend sc_signed operator - (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator - (const sc_signed &u, const sc_unsigned &v);

    friend sc_signed operator - (const sc_unsigned &u, const sc_unsigned &v);
    friend sc_signed operator - (const sc_unsigned &u, int64 v);
    friend sc_signed operator - (const sc_unsigned &u, uint64 v);
    friend sc_signed operator - (const sc_unsigned &u, long v);
    friend sc_signed operator - (const sc_unsigned &u, unsigned long v);
    friend sc_signed operator - (const sc_unsigned &u, int v);
    friend sc_signed operator - (const sc_unsigned &u, unsigned int v);

    friend sc_signed operator - (int64 u, const sc_unsigned &v);
    friend sc_signed operator - (uint64 u, const sc_unsigned &v);
    friend sc_signed operator - (long u, const sc_unsigned &v);
    friend sc_signed operator - (unsigned long u, const sc_unsigned &v);
    friend sc_signed operator - (int u, const sc_unsigned &v);
    friend sc_signed operator - (unsigned int u, const sc_unsigned &v);

    const sc_unsigned &operator -= (const sc_signed &v);
    const sc_unsigned &operator -= (const sc_unsigned &v);
    const sc_unsigned &operator -= (int64 v);
    const sc_unsigned &operator -= (uint64 v);
    const sc_unsigned &operator -= (long v);
    const sc_unsigned &operator -= (unsigned long v);
    const sc_unsigned &
    operator -= (int v)
    {
        return operator -= ((long)v);
    }
    const sc_unsigned &
    operator -= (unsigned int v)
    {
        return operator -= ((unsigned long)v);
    }

    friend sc_signed operator - (const sc_unsigned &u, const sc_uint_base &v);
    friend sc_signed operator - (const sc_unsigned &u, const sc_int_base &v);
    friend sc_signed operator - (const sc_uint_base &u, const sc_unsigned &v);
    friend sc_signed operator - (const sc_int_base &u, const sc_unsigned &v);
    const sc_unsigned &operator -= (const sc_int_base &v);
    const sc_unsigned &operator -= (const sc_uint_base &v);

    // MULtiplication operators:
    friend sc_signed operator * (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator * (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator * (const sc_unsigned &u, const sc_unsigned &v);
    friend sc_signed operator * (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator * (const sc_unsigned &u, uint64 v);
    friend sc_signed operator * (const sc_unsigned &u, long v);
    friend sc_unsigned operator * (const sc_unsigned &u, unsigned long v);
    friend sc_signed operator * (const sc_unsigned &u, int v);
    friend sc_unsigned
    operator * (const sc_unsigned &u, unsigned int v)
    {
        return operator * (u, (unsigned long)v);
    }

    friend sc_signed operator * (int64 u, const sc_unsigned &v);
    friend sc_unsigned operator * (uint64 u, const sc_unsigned &v);
    friend sc_signed operator * (long u, const sc_unsigned &v);
    friend sc_unsigned operator * (unsigned long u, const sc_unsigned &v);
    friend sc_signed operator * (int u, const sc_unsigned &v);
    friend sc_unsigned
    operator * (unsigned int u, const sc_unsigned &v)
    {
        return operator * ((unsigned long)u, v);
    }

    const sc_unsigned &operator *= (const sc_signed &v);
    const sc_unsigned &operator *= (const sc_unsigned &v);
    const sc_unsigned &operator *= (int64 v);
    const sc_unsigned &operator *= (uint64 v);
    const sc_unsigned &operator *= (long v);
    const sc_unsigned &operator *= (unsigned long v);
    const sc_unsigned &operator *= (int v) { return operator *= ((long)v); }
    const sc_unsigned &
    operator *= (unsigned int v)
    {
        return operator *= ((unsigned long)v);
    }

    friend sc_unsigned operator * (
            const sc_unsigned &u, const sc_uint_base &v);
    friend sc_signed operator * (const sc_unsigned &u, const sc_int_base &v);
    friend sc_unsigned operator * (
            const sc_uint_base &u, const sc_unsigned &v);
    friend sc_signed operator * (const sc_int_base &u, const sc_unsigned &v);
    const sc_unsigned &operator *= (const sc_int_base &v);
    const sc_unsigned &operator *= (const sc_uint_base &v);

    // DIVision operators:
    friend sc_signed operator / (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator / (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator / (const sc_unsigned &u, const sc_unsigned &v);
    friend sc_signed operator / (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator / (const sc_unsigned &u, uint64 v);
    friend sc_signed operator / (const sc_unsigned &u, long v);
    friend sc_unsigned operator / (const sc_unsigned &u, unsigned long v);
    friend sc_signed operator / (const sc_unsigned &u, int v);
    friend sc_unsigned
    operator / (const sc_unsigned &u, unsigned int v)
    {
        return operator / (u, (unsigned long)v);
    }

    friend sc_signed operator / (int64 u, const sc_unsigned &v);
    friend sc_unsigned operator / (uint64 u, const sc_unsigned &v);
    friend sc_signed operator / (long u, const sc_unsigned &v);
    friend sc_unsigned operator / (unsigned long u, const sc_unsigned &v);
    friend sc_signed operator / (int u, const sc_unsigned &v);
    friend sc_unsigned
    operator / (unsigned int u, const sc_unsigned &v)
    {
        return operator / ((unsigned long)u, v);
    }

    const sc_unsigned &operator /= (const sc_signed &v);
    const sc_unsigned &operator /= (const sc_unsigned &v);
    const sc_unsigned &operator /= (int64 v);
    const sc_unsigned &operator /= (uint64 v);
    const sc_unsigned &operator /= (long v);
    const sc_unsigned &operator /= (unsigned long v);
    const sc_unsigned &operator /= (int v) { return operator /= ((long)v); }
    const sc_unsigned &
    operator /= (unsigned int v)
    {
        return operator /= ((unsigned long)v);
    }

    friend sc_unsigned operator / (
            const sc_unsigned &u, const sc_uint_base &v);
    friend sc_signed operator / (const sc_unsigned &u, const sc_int_base &v);
    friend sc_unsigned operator / (
            const sc_uint_base &u, const sc_unsigned &v);
    friend sc_signed operator / (const sc_int_base &u, const sc_unsigned &v);
    const sc_unsigned &operator /= (const sc_int_base &v);
    const sc_unsigned &operator /= (const sc_uint_base &v);

    // MODulo operators:
    friend sc_signed operator % (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator % (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator % (const sc_unsigned &u, const sc_unsigned &v);
    friend sc_signed operator % (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator % (const sc_unsigned &u, uint64 v);
    friend sc_signed operator % (const sc_unsigned &u, long v);
    friend sc_unsigned operator % (const sc_unsigned &u, unsigned long v);
    friend sc_signed operator % (const sc_unsigned &u, int v);
    friend sc_unsigned
    operator % (const sc_unsigned &u, unsigned int v)
    {
        return operator % (u, (unsigned long)v);
    }

    friend sc_signed operator % (int64 u, const sc_unsigned &v);
    friend sc_unsigned operator % (uint64 u, const sc_unsigned &v);
    friend sc_signed operator % (long u, const sc_unsigned &v);
    friend sc_unsigned operator % (unsigned long u, const sc_unsigned &v);
    friend sc_signed operator % (int u, const sc_unsigned &v);
    friend sc_unsigned
    operator % (unsigned int u, const sc_unsigned &v)
    {
        return operator % ((unsigned long)u, v);
    }

    const sc_unsigned &operator %= (const sc_signed &v);
    const sc_unsigned &operator %= (const sc_unsigned &v);
    const sc_unsigned &operator %= (int64 v);
    const sc_unsigned &operator %= (uint64 v);
    const sc_unsigned &operator %= (long v);
    const sc_unsigned &operator %= (unsigned long v);
    const sc_unsigned &operator %= (int v) { return operator %= ((long)v); }
    const sc_unsigned &
    operator %= (unsigned int v)
    {
        return operator %= ((unsigned long)v);
    }

    friend sc_unsigned operator % (
            const sc_unsigned &u, const sc_uint_base &v);
    friend sc_signed operator % (const sc_unsigned &u, const sc_int_base &v);
    friend sc_unsigned operator % (
            const sc_uint_base &u, const sc_unsigned &v);
    friend sc_signed operator % (const sc_int_base &u, const sc_unsigned &v);
    const sc_unsigned &operator %= (const sc_int_base &v);
    const sc_unsigned &operator %= (const sc_uint_base &v);

    // BITWISE OPERATORS:

    // Bitwise AND operators:
    friend sc_signed operator & (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator & (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator & (const sc_unsigned &u, const sc_unsigned &v);
    friend sc_signed operator & (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator & (const sc_unsigned &u, uint64 v);
    friend sc_signed operator & (const sc_unsigned &u, long v);
    friend sc_unsigned operator & (const sc_unsigned &u, unsigned long v);
    friend sc_signed operator & (const sc_unsigned &u, int v);
    friend sc_unsigned
    operator & (const sc_unsigned &u, unsigned int v)
    {
        return operator & (u, (unsigned long)v);
    }

    friend sc_signed operator & (int64 u, const sc_unsigned &v);
    friend sc_unsigned operator & (uint64 u, const sc_unsigned &v);
    friend sc_signed operator & (long u, const sc_unsigned &v);
    friend sc_unsigned operator & (unsigned long u, const sc_unsigned &v);
    friend sc_signed operator & (int u, const sc_unsigned &v);
    friend sc_unsigned
    operator & (unsigned int u, const sc_unsigned &v)
    {
        return operator & ((unsigned long)u, v);
    }

    const sc_unsigned &operator &= (const sc_signed &v);
    const sc_unsigned &operator &= (const sc_unsigned &v);
    const sc_unsigned &operator &= (int64 v);
    const sc_unsigned &operator &= (uint64 v);
    const sc_unsigned &operator &= (long v);
    const sc_unsigned &operator &= (unsigned long v);
    const sc_unsigned &operator &= (int v) { return operator&=((long) v); }
    const sc_unsigned &
    operator &= (unsigned int v)
    {
        return operator &= ((unsigned long)v);
    }

    friend sc_unsigned operator & (
            const sc_unsigned &u, const sc_uint_base &v);
    friend sc_signed operator & (const sc_unsigned &u, const sc_int_base &v);
    friend sc_unsigned operator & (
            const sc_uint_base &u, const sc_unsigned &v);
    friend sc_signed operator & (const sc_int_base &u, const sc_unsigned &v);
    const sc_unsigned &operator &= (const sc_int_base &v);
    const sc_unsigned &operator &= (const sc_uint_base &v);

    // Bitwise OR operators:
    friend sc_signed operator | (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator | (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator | (const sc_unsigned &u, const sc_unsigned &v);
    friend sc_signed operator | (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator | (const sc_unsigned &u, uint64 v);
    friend sc_signed operator | (const sc_unsigned &u, long v);
    friend sc_unsigned operator | (const sc_unsigned &u, unsigned long v);
    friend sc_signed operator | (const sc_unsigned &u, int v);
    friend sc_unsigned
    operator | (const sc_unsigned &u, unsigned int v)
    {
        return operator | (u, (unsigned long)v);
    }

    friend sc_signed operator | (int64 u, const sc_unsigned &v);
    friend sc_unsigned operator | (uint64 u, const sc_unsigned &v);
    friend sc_signed operator | (long u, const sc_unsigned &v);
    friend sc_unsigned operator | (unsigned long u, const sc_unsigned &v);
    friend sc_signed operator | (int u, const sc_unsigned &v);
    friend sc_unsigned
    operator | (unsigned int u, const sc_unsigned &v)
    {
        return operator | ((unsigned long)u, v);
    }

    const sc_unsigned &operator |= (const sc_signed &v);
    const sc_unsigned &operator |= (const sc_unsigned &v);
    const sc_unsigned &operator |= (int64 v);
    const sc_unsigned &operator |= (uint64 v);
    const sc_unsigned &operator |= (long v);
    const sc_unsigned &operator |= (unsigned long v);
    const sc_unsigned &operator |= (int v) { return operator|=((long) v); }
    const sc_unsigned &
    operator |= (unsigned int v)
    {
        return operator |= ((unsigned long)v);
    }

    friend sc_unsigned operator | (
            const sc_unsigned &u, const sc_uint_base &v);
    friend sc_signed operator | (const sc_unsigned &u, const sc_int_base &v);
    friend sc_unsigned operator | (
            const sc_uint_base &u, const sc_unsigned &v);
    friend sc_signed operator | (const sc_int_base &u, const sc_unsigned &v);
    const sc_unsigned &operator |= (const sc_int_base &v);
    const sc_unsigned &operator |= (const sc_uint_base &v);

    // Bitwise XOR operators:
    friend sc_signed operator ^ (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator ^ (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator ^ (const sc_unsigned &u, const sc_unsigned &v);
    friend sc_signed operator ^ (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator ^ (const sc_unsigned &u, uint64 v);
    friend sc_signed operator ^ (const sc_unsigned &u, long v);
    friend sc_unsigned operator ^ (const sc_unsigned &u, unsigned long v);
    friend sc_signed operator ^ (const sc_unsigned &u, int v);
    friend sc_unsigned
    operator ^ (const sc_unsigned &u, unsigned int v)
    {
        return operator ^ (u, (unsigned long)v);
    }

    friend sc_signed operator ^ (int64 u, const sc_unsigned &v);
    friend sc_unsigned operator ^ (uint64 u, const sc_unsigned &v);
    friend sc_signed operator ^ (long u, const sc_unsigned &v);
    friend sc_unsigned operator ^ (unsigned long u, const sc_unsigned &v);
    friend sc_signed operator ^ (int u, const sc_unsigned &v);
    friend sc_unsigned
    operator ^ (unsigned int u, const sc_unsigned &v)
    {
        return operator ^ ((unsigned long)u, v);
    }

    const sc_unsigned &operator ^= (const sc_signed &v);
    const sc_unsigned &operator ^= (const sc_unsigned &v);
    const sc_unsigned &operator ^= (int64 v);
    const sc_unsigned &operator ^= (uint64 v);
    const sc_unsigned &operator ^= (long v);
    const sc_unsigned &operator ^= (unsigned long v);
    const sc_unsigned &
    operator ^= (int v)
    {
        return operator ^= ((long)v);
    }
    const sc_unsigned &
    operator ^= (unsigned int v)
    {
        return operator ^= ((unsigned long)v);
    }

    friend sc_unsigned operator ^ (
            const sc_unsigned &u, const sc_uint_base &v);
    friend sc_signed operator ^ (const sc_unsigned &u, const sc_int_base &v);
    friend sc_unsigned operator ^ (
            const sc_uint_base &u, const sc_unsigned &v);
    friend sc_signed operator ^ (const sc_int_base &u, const sc_unsigned &v);
    const sc_unsigned &operator ^= (const sc_int_base &v);
    const sc_unsigned &operator ^= (const sc_uint_base &v);

    // SHIFT OPERATORS:

    // LEFT SHIFT operators:
    friend sc_unsigned operator << (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator << (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator << (
            const sc_unsigned &u, const sc_unsigned &v);
    friend sc_unsigned operator << (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator << (const sc_unsigned &u, uint64 v);
    friend sc_unsigned operator << (const sc_unsigned &u, long v);
    friend sc_unsigned operator << (const sc_unsigned &u, unsigned long v);
    friend sc_unsigned
    operator << (const sc_unsigned &u, int v)
    {
        return operator << (u, (long)v);
    }
    friend sc_unsigned
    operator << (const sc_unsigned &u, unsigned int v)
    {
        return operator << (u, (unsigned long)v);
    }

    const sc_unsigned &operator <<= (const sc_signed &v);
    const sc_unsigned &operator <<= (const sc_unsigned &v);
    const sc_unsigned &operator <<= (int64 v);
    const sc_unsigned &operator <<= (uint64 v);
    const sc_unsigned &operator <<= (long v);
    const sc_unsigned &operator <<= (unsigned long v);
    const sc_unsigned &operator <<= (int v) { return operator <<= ((long)v); }
    const sc_unsigned &
    operator <<= (unsigned int v)
    {
        return operator <<= ((unsigned long)v);
    }

    friend sc_unsigned operator << (
            const sc_unsigned &u, const sc_uint_base &v);
    friend sc_unsigned operator << (
            const sc_unsigned &u, const sc_int_base &v);
    const sc_unsigned &operator <<= (const sc_int_base &v);
    const sc_unsigned &operator <<= (const sc_uint_base &v);

    // RIGHT SHIFT operators:
    friend sc_unsigned operator >> (const sc_unsigned &u, const sc_signed &v);
    friend sc_signed operator >> (const sc_signed &u, const sc_unsigned &v);

    friend sc_unsigned operator >> (
            const sc_unsigned &u, const sc_unsigned &v);
    friend sc_unsigned operator >> (const sc_unsigned &u, int64 v);
    friend sc_unsigned operator >> (const sc_unsigned &u, uint64 v);
    friend sc_unsigned operator >> (const sc_unsigned &u, long v);
    friend sc_unsigned operator >> (const sc_unsigned &u, unsigned long v);
    friend sc_unsigned
    operator >> (const sc_unsigned &u, int v)
    {
        return operator >> (u, (long)v);
    }
    friend sc_unsigned
    operator >> (const sc_unsigned &u, unsigned int v)
    {
        return operator >> (u, (unsigned long)v);
    }

    const sc_unsigned &operator >>= (const sc_signed &v);
    const sc_unsigned &operator >>= (const sc_unsigned &v);
    const sc_unsigned &operator >>= (int64 v);
    const sc_unsigned &operator >>= (uint64 v);
    const sc_unsigned &operator >>= (long v);
    const sc_unsigned &operator >>= (unsigned long v);
    const sc_unsigned &operator >>= (int v) { return operator >>= ((long)v); }
    const sc_unsigned &
    operator >>= (unsigned int v)
    {
        return operator >>= ((unsigned long)v);
    }

    friend sc_unsigned operator >> (const sc_unsigned &, const sc_uint_base &);
    friend sc_unsigned operator >> (const sc_unsigned&, const sc_int_base &);
    const sc_unsigned &operator >>= (const sc_int_base &v);
    const sc_unsigned &operator >>= (const sc_uint_base &v);

    // Unary arithmetic operators
    friend sc_unsigned operator + (const sc_unsigned &u);
    friend sc_signed operator - (const sc_unsigned &u);

    // LOGICAL OPERATORS:

    // Logical EQUAL operators:
    friend bool operator == (const sc_unsigned &u, const sc_signed &v);
    friend bool operator == (const sc_signed &u, const sc_unsigned &v);

    friend bool operator == (const sc_unsigned &u, const sc_unsigned &v);
    friend bool operator == (const sc_unsigned &u, int64 v);
    friend bool operator == (const sc_unsigned &u, uint64 v);
    friend bool operator == (const sc_unsigned &u, long v);
    friend bool operator == (const sc_unsigned &u, unsigned long v);
    friend bool
    operator == (const sc_unsigned &u, int v)
    {
        return operator == (u, (long)v);
    }
    friend bool
    operator == (const sc_unsigned &u, unsigned int v)
    {
        return operator == (u, (unsigned long)v);
    }

    friend bool operator == (int64 u, const sc_unsigned &v);
    friend bool operator == (uint64 u, const sc_unsigned &v);
    friend bool operator == (long u, const sc_unsigned &v);
    friend bool operator == (unsigned long u, const sc_unsigned &v);
    friend bool
    operator == (int u, const sc_unsigned &v)
    {
        return operator == ((long)u, v);
    }
    friend bool
    operator == (unsigned int u, const sc_unsigned &v)
    {
        return operator == ((unsigned long)u, v);
    }

    friend bool operator == (const sc_unsigned &u, const sc_uint_base &v);
    friend bool operator == (const sc_unsigned &u, const sc_int_base &v);
    friend bool operator == (const sc_uint_base &u, const sc_unsigned &v);
    friend bool operator == (const sc_int_base &u, const sc_unsigned &v);

    // Logical NOT_EQUAL operators:
    friend bool operator != (const sc_unsigned &u, const sc_signed &v);
    friend bool operator != (const sc_signed &u, const sc_unsigned &v);

    friend bool operator != (const sc_unsigned &u, const sc_unsigned &v);
    friend bool operator != (const sc_unsigned &u, int64 v);
    friend bool operator != (const sc_unsigned &u, uint64 v);
    friend bool operator != (const sc_unsigned &u, long v);
    friend bool operator != (const sc_unsigned &u, unsigned long v);
    friend bool
    operator != (const sc_unsigned &u, int v)
    {
        return operator != (u, (long)v);
    }
    friend bool
    operator != (const sc_unsigned &u, unsigned int v)
    {
        return operator != (u, (unsigned long)v);
    }

    friend bool operator != (int64 u, const sc_unsigned &v);
    friend bool operator != (uint64 u, const sc_unsigned &v);
    friend bool operator != (long u, const sc_unsigned &v);
    friend bool operator != (unsigned long u, const sc_unsigned &v);
    friend bool
    operator != (int u, const sc_unsigned &v)
    {
        return operator != ((long)u, v);
    }
    friend bool
    operator != (unsigned int u, const sc_unsigned &v)
    {
        return operator != ((unsigned long)u, v);
    }

    friend bool operator != (const sc_unsigned &u, const sc_uint_base &v);
    friend bool operator != (const sc_unsigned &u, const sc_int_base &v);
    friend bool operator != (const sc_uint_base &u, const sc_unsigned &v);
    friend bool operator != (const sc_int_base &u, const sc_unsigned &v);

    // Logical LESS_THAN operators:
    friend bool operator < (const sc_unsigned &u, const sc_signed &v);
    friend bool operator < (const sc_signed &u, const sc_unsigned &v);

    friend bool operator < (const sc_unsigned &u, const sc_unsigned &v);
    friend bool operator < (const sc_unsigned &u, int64 v);
    friend bool operator < (const sc_unsigned &u, uint64 v);
    friend bool operator < (const sc_unsigned &u, long v);
    friend bool operator < (const sc_unsigned &u, unsigned long v);
    friend bool
    operator < (const sc_unsigned &u, int v)
    {
        return operator < (u, (long)v);
    }
    friend bool
    operator < (const sc_unsigned &u, unsigned int v)
    {
        return operator < (u, (unsigned long)v);
    }

    friend bool operator < (int64 u, const sc_unsigned &v);
    friend bool operator < (uint64 u, const sc_unsigned &v);
    friend bool operator < (long u, const sc_unsigned &v);
    friend bool operator < (unsigned long u, const sc_unsigned &v);
    friend bool
    operator < (int u, const sc_unsigned &v)
    {
        return operator < ((long)u, v);
    }
    friend bool
    operator < (unsigned int u, const sc_unsigned &v)
    {
        return operator < ((unsigned long)u, v);
    }

    friend bool operator < (const sc_unsigned &u, const sc_uint_base &v);
    friend bool operator < (const sc_unsigned &u, const sc_int_base &v);
    friend bool operator < (const sc_uint_base &u, const sc_unsigned &v);
    friend bool operator < (const sc_int_base &u, const sc_unsigned &v);

    // Logical LESS_THAN_AND_EQUAL operators:
    friend bool operator <= (const sc_unsigned &u, const sc_signed &v);
    friend bool operator <= (const sc_signed &u, const sc_unsigned &v);

    friend bool operator <= (const sc_unsigned &u, const sc_unsigned &v);
    friend bool operator <= (const sc_unsigned &u, int64 v);
    friend bool operator <= (const sc_unsigned &u, uint64 v);
    friend bool operator <= (const sc_unsigned &u, long v);
    friend bool operator <= (const sc_unsigned &u, unsigned long v);
    friend bool
    operator <= (const sc_unsigned &u, int v)
    {
        return operator <= (u, (long)v);
    }
    friend bool
    operator <= (const sc_unsigned &u, unsigned int v)
    {
        return operator <= (u, (unsigned long)v);
    }

    friend bool operator <= (int64 u, const sc_unsigned &v);
    friend bool operator <= (uint64 u, const sc_unsigned &v);
    friend bool operator <= (long u, const sc_unsigned &v);
    friend bool operator <= (unsigned long u, const sc_unsigned &v);
    friend bool
    operator <= (int u, const sc_unsigned &v)
    {
        return operator <= ((long)u, v);
    }
    friend bool
    operator <= (unsigned int u, const sc_unsigned &v)
    {
        return operator <= ((unsigned long)u, v);
    }

    friend bool operator <= (const sc_unsigned &u, const sc_uint_base &v);
    friend bool operator <= (const sc_unsigned &u, const sc_int_base &v);
    friend bool operator <= (const sc_uint_base &u, const sc_unsigned &v);
    friend bool operator <= (const sc_int_base &u, const sc_unsigned &v);

    // Logical GREATER_THAN operators:
    friend bool operator > (const sc_unsigned &u, const sc_signed &v);
    friend bool operator > (const sc_signed &u, const sc_unsigned &v);

    friend bool operator > (const sc_unsigned &u, const sc_unsigned &v);
    friend bool operator > (const sc_unsigned &u, int64 v);
    friend bool operator > (const sc_unsigned &u, uint64 v);
    friend bool operator > (const sc_unsigned &u, long v);
    friend bool operator > (const sc_unsigned &u, unsigned long v);
    friend bool
    operator > (const sc_unsigned &u, int v)
    {
        return operator > (u, (long)v);
    }
    friend bool
    operator > (const sc_unsigned &u, unsigned int v)
    {
        return operator > (u, (unsigned long)v);
    }

    friend bool operator > (int64 u, const sc_unsigned &v);
    friend bool operator > (uint64 u, const sc_unsigned &v);
    friend bool operator > (long u, const sc_unsigned &v);
    friend bool operator > (unsigned long u, const sc_unsigned &v);
    friend bool
    operator > (int u, const sc_unsigned &v)
    {
        return operator > ((long)u, v);
    }
    friend bool
    operator > (unsigned int u, const sc_unsigned &v)
    {
        return operator > ((unsigned long)u, v);
    }

    friend bool operator > (const sc_unsigned &u, const sc_uint_base &v);
    friend bool operator > (const sc_unsigned &u, const sc_int_base &v);
    friend bool operator > (const sc_uint_base &u, const sc_unsigned &v);
    friend bool operator > (const sc_int_base &u, const sc_unsigned &v);

    // Logical GREATER_THAN_AND_EQUAL operators:
    friend bool operator >= (const sc_unsigned &u, const sc_signed &v);
    friend bool operator >= (const sc_signed &u, const sc_unsigned &v);

    friend bool operator >= (const sc_unsigned &u, const sc_unsigned &v);
    friend bool operator >= (const sc_unsigned &u, int64 v);
    friend bool operator >= (const sc_unsigned &u, uint64 v);
    friend bool operator >= (const sc_unsigned &u, long v);
    friend bool operator >= (const sc_unsigned &u, unsigned long v);
    friend bool
    operator >= (const sc_unsigned &u, int v)
    {
        return operator >= (u, (long)v);
    }
    friend bool
    operator >= (const sc_unsigned &u, unsigned int v)
    {
        return operator >= (u, (unsigned long)v);
    }

    friend bool operator >= (int64 u, const sc_unsigned &v);
    friend bool operator >= (uint64 u, const sc_unsigned &v);
    friend bool operator >= (long u, const sc_unsigned &v);
    friend bool operator >= (unsigned long u, const sc_unsigned &v);
    friend bool
    operator >= (int u, const sc_unsigned &v)
    {
        return operator >= ((long)u, v);
    }
    friend bool
    operator >= (unsigned int u, const sc_unsigned &v)
    {
        return operator >= ((unsigned long)u, v);
    }

    friend bool operator >= (const sc_unsigned &u, const sc_uint_base &v);
    friend bool operator >= (const sc_unsigned &u, const sc_int_base &v);
    friend bool operator >= (const sc_uint_base &u, const sc_unsigned &v);
    friend bool operator >= (const sc_int_base &u, const sc_unsigned &v);

    // Bitwise NOT operator (unary).
    friend sc_unsigned operator ~ (const sc_unsigned &u);

    // Helper functions.
    friend int compare_unsigned(
            small_type us, int unb, int und, const sc_digit *ud,
            small_type vs, int vnb, int vnd, const sc_digit *vd,
            small_type if_u_signed, small_type if_v_signed);

    friend sc_unsigned add_unsigned_friend(
            small_type us, int unb, int und, const sc_digit *ud,
            small_type vs, int vnb, int vnd, const sc_digit *vd);

    friend sc_unsigned sub_unsigned_friend(
            small_type us, int unb, int und, const sc_digit *ud,
            small_type vs, int vnb, int vnd, const sc_digit *vd);

    friend sc_unsigned mul_unsigned_friend(
            small_type s, int unb, int und, const sc_digit *ud,
            int vnb, int vnd, const sc_digit *vd);

    friend sc_unsigned div_unsigned_friend(
            small_type s, int unb, int und, const sc_digit *ud,
            int vnb, int vnd, const sc_digit *vd);

    friend sc_unsigned mod_unsigned_friend(
            small_type us, int unb, int und, const sc_digit *ud,
            int vnb, int vnd, const sc_digit *vd);

    friend sc_unsigned and_unsigned_friend(
            small_type us, int unb, int und, const sc_digit *ud,
            small_type vs, int vnb, int vnd, const sc_digit *vd);

    friend sc_unsigned or_unsigned_friend(
            small_type us, int unb, int und, const sc_digit *ud,
            small_type vs, int vnb, int vnd, const sc_digit *vd);

    friend sc_unsigned xor_unsigned_friend(
            small_type us, int unb, int und, const sc_digit *ud,
            small_type vs, int vnb, int vnd, const sc_digit *vd);

  public:
    static sc_core::sc_vpool<sc_unsigned> m_pool;

  private:
    small_type sgn; // Shortened as s.
    int nbits; // Shortened as nb.
    int ndigits; // Shortened as nd.

#ifdef SC_MAX_NBITS
    sc_digit digit[DIV_CEIL(SC_MAX_NBITS)]; // Shortened as d.
#else
    sc_digit *digit; // Shortened as d.
#endif

    // Private constructors:

    // Create a copy of v with sign s.
    sc_unsigned(const sc_unsigned &v, small_type s);
    sc_unsigned(const sc_signed &v, small_type s);

    // Create an unsigned number with the given attributes.
    sc_unsigned(small_type s, int nb, int nd, sc_digit *d, bool alloc=true);

    // Create an unsigned number using the bits u[l..r].
    sc_unsigned(const sc_signed *u, int l, int r);
    sc_unsigned(const sc_unsigned *u, int l, int r);

    // Private member functions. The called functions are inline functions.

    small_type default_sign() const { return SC_POS; }

    int num_bits(int nb) const { return nb + 1; }

    bool check_if_outside(int bit_num) const;

    void
    copy_digits(int nb, int nd, const sc_digit *d)
    {
        copy_digits_unsigned(sgn, nbits, ndigits, digit, nb, nd, d);
    }

    void makezero() { sgn = make_zero(ndigits, digit); }

    // Conversion functions between 2's complement (2C) and
    // sign-magnitude (SM):
    void
    convert_2C_to_SM()
    {
        sgn = convert_unsigned_2C_to_SM(nbits, ndigits, digit);
    }

    void
    convert_SM_to_2C_to_SM()
    {
        sgn = convert_unsigned_SM_to_2C_to_SM(sgn, nbits, ndigits, digit);
    }

    void convert_SM_to_2C() { convert_unsigned_SM_to_2C(sgn, ndigits, digit); }
};

inline ::std::ostream &operator << (::std::ostream &, const sc_unsigned &);

inline ::std::istream &operator >> (::std::istream &, sc_unsigned &);


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_bitref_r
//
//  Proxy class for sc_unsigned bit selection (r-value only).
// ----------------------------------------------------------------------------


inline ::std::ostream &
operator << (::std::ostream &os, const sc_unsigned_bitref_r &a)
{
    a.print(os);
    return os;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_bitref
//
//  Proxy class for sc_unsigned bit selection (r-value and l-value).
// ----------------------------------------------------------------------------

template<class T>
inline const sc_unsigned_subref &
sc_unsigned_subref::operator = (const sc_generic_base<T> &a)
{
    sc_unsigned temp(length());
    a->to_sc_unsigned(temp);
    return *this = temp;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_unsigned_bitref &a)
{
    a.scan(is);
    return is;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_subref_r
//
//  Proxy class for sc_unsigned part selection (r-value only).
// ----------------------------------------------------------------------------

// reduce methods

inline bool
sc_unsigned_subref_r::and_reduce() const
{
    const sc_unsigned *target_p = m_obj_p;
    for (int i = m_right; i <= m_left; i++)
        if (!target_p->test(i))
            return false;
    return true;
}

inline bool
sc_unsigned_subref_r::nand_reduce() const
{
    return !and_reduce();
}

inline bool
sc_unsigned_subref_r::or_reduce() const
{
    const sc_unsigned *target_p = m_obj_p;
    for (int i = m_right; i <= m_left; i++)
        if (target_p->test(i))
            return true;
    return false;
}

inline bool
sc_unsigned_subref_r::nor_reduce() const
{
    return !or_reduce();
}

inline bool
sc_unsigned_subref_r::xor_reduce() const
{
    int odd;
    const sc_unsigned *target_p = m_obj_p;
    odd = 0;
    for (int i = m_right; i <= m_left; i++)
        if (target_p->test(i)) odd = ~odd;
    return odd ? true : false;
}

inline bool sc_unsigned_subref_r::xnor_reduce() const { return !xor_reduce(); }

inline ::std::ostream &
operator << (::std::ostream &os, const sc_unsigned_subref_r &a)
{
    a.print(os);
    return os;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_subref
//
//  Proxy class for sc_unsigned part selection (r-value and l-value).
// ----------------------------------------------------------------------------

// assignment operators

inline const sc_unsigned_subref &
sc_unsigned_subref::operator = (const char *a)
{
    sc_unsigned aa(length());
    return (*this = aa = a);
}


inline ::std::istream &
operator >> (::std::istream &is, sc_unsigned_subref &a)
{
    a.scan(is);
    return is;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned
//
//  Arbitrary precision signed number.
// ----------------------------------------------------------------------------

template<class T>
sc_unsigned::sc_unsigned( const sc_generic_base<T> &v)
{
    int nb = v->length();
    sgn = default_sign();
    if (nb > 0) {
        nbits = num_bits(nb);
    } else {
        invalid_init("sc_generic_base<T>", nb);
        sc_core::sc_abort(); // can't recover from here
    }
    ndigits = DIV_CEIL(nbits);
#   ifdef SC_MAX_NBITS
        test_bound(nb);
#   else
        digit = new sc_digit[ndigits];
#   endif
    makezero();
    v->to_sc_unsigned(*this);
}

inline ::std::ostream &
operator << (::std::ostream &os, const sc_unsigned &a)
{
    a.print(os);
    return os;
}

inline ::std::istream &
operator >> (::std::istream &is, sc_unsigned &a)
{
    a.scan(is);
    return is;
}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_INT_SC_UNSIGNED_HH__

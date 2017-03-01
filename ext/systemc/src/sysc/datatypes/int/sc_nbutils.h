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

  sc_nbutils.h -- External and friend functions for both sc_signed and
                  sc_unsigned classes.

  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_nbutils.h,v $
// Revision 1.6  2011/09/08 16:12:15  acg
//  Philipp A. Hartmann: fix issue with Sun machines wrt real math libraries.
//
// Revision 1.5  2011/08/26 23:00:01  acg
//  Torsten Maehne: remove use of ieeefp.h.
//
// Revision 1.4  2011/08/15 16:43:24  acg
//  Torsten Maehne: changes to remove unused argument warnings.
//
// Revision 1.3  2011/02/18 20:19:15  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.2  2010/09/06 16:35:48  acg
//  Andy Goodrich: changed i386 to __i386__ in ifdef's.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:49:32  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#ifndef SC_NBUTILS_H
#define SC_NBUTILS_H

#include <cmath>
#include <limits>

#include "sysc/datatypes/bit/sc_bit_ids.h"
#include "sysc/datatypes/int/sc_int_ids.h"
#include "sysc/datatypes/int/sc_nbdefs.h"
#include "sysc/utils/sc_report.h"


namespace sc_dt
{

//-----------------------------------------------------------------------------
//"sc_io_base"
//
// This inline function returns the type of an i/o stream's base as a SystemC
// base designator.
//   stream_object = reference to the i/o stream whose base is to be returned.
//
//"sc_io_show_base"
//
// This inline function returns true if the base should be shown when a SystemC
// value is displayed via the supplied stream operator.
//   stream_object = reference to the i/o stream to return showbase value for.
//-----------------------------------------------------------------------------
#if defined(__GNUC__) || defined(_MSC_VER) || defined(__SUNPRO_CC)
    inline sc_numrep
    sc_io_base( systemc_ostream& os, sc_numrep def_base )
    {
        std::ios::fmtflags flags = os.flags() & std::ios::basefield;
        if ( flags & ::std::ios::dec ) return  SC_DEC;
        if ( flags & ::std::ios::hex ) return  SC_HEX;
        if ( flags & ::std::ios::oct ) return  SC_OCT;
        return def_base;
    }

    inline bool
    sc_io_show_base( systemc_ostream& os )
    {
        return (os.flags() & ::std::ios::showbase) != 0 ;
    }
#else   // Other
    inline sc_numrep
    sc_io_base( systemc_ostream& /*unused*/, sc_numrep /*unused*/ )
    {
        return SC_DEC;
    }
    inline bool
    sc_io_show_base( systemc_ostream& /*unused*/ )
    {
        return false;
    }
#endif

const std::string to_string( sc_numrep );

inline
systemc_ostream&
operator << ( systemc_ostream& os, sc_numrep numrep )
{
    os << to_string( numrep );
    return os;
}

// only used within vec_from_str (non-standard, deprecated)
inline void
is_valid_base(sc_numrep base)
{
  switch (base) {
    case SC_NOBASE: case SC_BIN: 
    case SC_OCT: case SC_DEC: 
    case SC_HEX: 
        break;
    case SC_BIN_US: case SC_BIN_SM: 
    case SC_OCT_US: case SC_OCT_SM:
    case SC_HEX_US: case SC_HEX_SM:
    case SC_CSD:
      SC_REPORT_ERROR( sc_core::SC_ID_NOT_IMPLEMENTED_,
		       "is_valid_base( sc_numrep base ) : "
		       "bases SC_CSD, or ending in _US and _SM are not supported" );
      break;
    default:
      char msg[BUFSIZ];
      std::sprintf( msg, "is_valid_base( sc_numrep base ) : "
	       "base = %s is not valid",
	       to_string( base ).c_str() );
      SC_REPORT_ERROR( sc_core::SC_ID_VALUE_NOT_VALID_, msg );
  }
}

// ----------------------------------------------------------------------------

// One transition of the FSM to find base and sign of a number.
extern
small_type 
fsm_move(char c, small_type &b, small_type &s, small_type &state);

// Parse a character string into its equivalent binary bits.
extern
void parse_binary_bits( 
    const char* src_p, int dst_n, sc_digit* data_p, sc_digit* ctrl_p=0
);


// Parse a character string into its equivalent hexadecimal bits.
extern
void parse_hex_bits( 
    const char* src_p, int dst_n, sc_digit* data_p, sc_digit* ctrl_p=0
);


// Find the base and sign of a number in v.
extern 
const char *
get_base_and_sign(const char *v, small_type &base, small_type &sign);

// Create a number out of v in base.
extern 
small_type 
vec_from_str(int unb, int und, sc_digit *u, 
             const char *v, sc_numrep base = SC_NOBASE) ;


// ----------------------------------------------------------------------------
//  Naming convention for the vec_ functions below:
//    vec_OP(u, v, w)  : computes w = u OP v.
//    vec_OP_on(u, v)  : computes u = u OP v if u has more digits than v.
//    vec_OP_on2(u, v) : computes u = u OP v if u has fewer digits than v.
//    _large           : parameters are vectors.
//    _small           : one of the parameters is a single digit.
//    Xlen             : the number of digits in X.
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//  Functions for vector addition: w = u + v or u += v.
// ----------------------------------------------------------------------------

extern 
void 
vec_add(int ulen, const sc_digit *u, 
        int vlen, const sc_digit *v, sc_digit *w);

extern 
void 
vec_add_on(int ulen, sc_digit *u, 
           int vlen, const sc_digit *v);

extern 
void 
vec_add_on2(int ulen, sc_digit *u, 
            int vlen, const sc_digit *v);

extern 
void 
vec_add_small(int ulen, const sc_digit *u,
              sc_digit v, sc_digit *w);

extern 
void 
vec_add_small_on(int ulen, sc_digit *u, sc_digit v);


// ----------------------------------------------------------------------------
//  Functions for vector subtraction: w = u - v, u -= v, or u = v - u.
// ----------------------------------------------------------------------------

extern 
void 
vec_sub(int ulen, const sc_digit *u, 
        int vlen, const sc_digit *v, sc_digit *w);

extern 
void 
vec_sub_on(int ulen, sc_digit *u, 
           int vlen, const sc_digit *v);

extern 
void 
vec_sub_on2(int ulen, sc_digit *u,
            int vlen, const sc_digit *v);

extern 
void 
vec_sub_small(int ulen, const sc_digit *u,
              sc_digit v, sc_digit *w);

extern 
void 
vec_sub_small_on(int ulen, sc_digit *u, sc_digit v);


// ----------------------------------------------------------------------------
//  Functions for vector multiplication: w = u * v or u *= v.
// ----------------------------------------------------------------------------

extern 
void 
vec_mul(int ulen, const sc_digit *u, 
        int vlen, const sc_digit *v, sc_digit *w);

extern 
void 
vec_mul_small(int ulen, const sc_digit *u,
              sc_digit v, sc_digit *w);

extern 
void 
vec_mul_small_on(int ulen, sc_digit *u, sc_digit v);


// ----------------------------------------------------------------------------
//  Functions for vector division: w = u / v.
// ----------------------------------------------------------------------------

extern 
void 
vec_div_large(int ulen, const sc_digit *u, 
              int vlen, const sc_digit *v, sc_digit *w);

extern 
void 
vec_div_small(int ulen, const sc_digit *u, 
              sc_digit v, sc_digit *w);


// ----------------------------------------------------------------------------
//  Functions for vector remainder: w = u % v or u %= v.
// ----------------------------------------------------------------------------

extern 
void 
vec_rem_large(int ulen, const sc_digit *u, 
              int vlen, const sc_digit *v, sc_digit *w);

extern 
sc_digit 
vec_rem_small(int ulen, const sc_digit *u, sc_digit v);

extern 
sc_digit 
vec_rem_on_small(int ulen, sc_digit *u, sc_digit v);


// ----------------------------------------------------------------------------
//  Functions to convert between vectors of char and sc_digit.
// ----------------------------------------------------------------------------

extern 
int 
vec_to_char(int ulen, const sc_digit *u, 
            int vlen, uchar *v);

extern 
void 
vec_from_char(int ulen, const uchar *u,
              int vlen, sc_digit *v);


// ----------------------------------------------------------------------------
//  Functions to shift left or right, or to create a mirror image of vectors.
// ----------------------------------------------------------------------------

extern 
void 
vec_shift_left(int ulen, sc_digit *u, int nsl);

extern 
void 
vec_shift_right(int vlen, sc_digit *u, int nsr, sc_digit fill = 0);

extern
void 
vec_reverse(int unb, int und, sc_digit *ud, 
            int l, int r = 0);


// ----------------------------------------------------------------------------
//  Various utility functions. 
// ----------------------------------------------------------------------------

// Return the low half part of d.
inline 
sc_digit 
low_half(sc_digit d) 
{
  return (d & HALF_DIGIT_MASK);
}

// Return the high half part of d. The high part of the digit may have
// more bits than BITS_PER_HALF_DIGIT due to, e.g., overflow in the
// multiplication. Hence, in other functions that use high_half(),
// make sure that the result contains BITS_PER_HALF_DIGIT if
// necessary. This is done by high_half_masked().
inline 
sc_digit 
high_half(sc_digit d) 
{
  return (d >> BITS_PER_HALF_DIGIT);
}

inline
sc_digit
high_half_masked(sc_digit d)
{
  return (high_half(d) & HALF_DIGIT_MASK);
}

// Concatenate the high part h and low part l. Assumes that h and l
// are less than or equal to HALF_DIGIT_MASK;
inline 
sc_digit 
concat(sc_digit h, sc_digit l) 
{
  return ((h << BITS_PER_HALF_DIGIT) | l);
}

// Create a number with n 1's.
inline
sc_digit
one_and_ones(int n)
{
  return (((sc_digit) 1 << n) - 1);
}

// Create a number with one 1 and n 0's.
inline
sc_digit
one_and_zeros(int n)
{
  return ((sc_digit) 1 << n);
}


// ----------------------------------------------------------------------------

// Find the digit that bit i is in.
inline
int 
digit_ord(int i)
{
  return (i / BITS_PER_DIGIT);
}

// Find the bit in digit_ord(i) that bit i corressponds to.
inline
int 
bit_ord(int i)
{
  return (i % BITS_PER_DIGIT);
}


// ----------------------------------------------------------------------------
//  Functions to compare, zero, complement vector(s).
// ----------------------------------------------------------------------------

// Compare u and v and return r
//  r = 0 if u == v
//  r < 0 if u < v
//  r > 0 if u > v
// - Assume that all the leading zero digits are already skipped.
// - ulen and/or vlen can be zero.
// - Every digit is less than or equal to DIGIT_MASK;
inline 
int
vec_cmp(int ulen, const sc_digit *u, 
        int vlen, const sc_digit *v)
{

#ifdef DEBUG_SYSTEMC
  // assert((ulen <= 0) || (u != NULL));
  // assert((vlen <= 0) || (v != NULL));

  // ulen and vlen can be equal to 0 because vec_cmp can be called
  // after vec_skip_leading_zeros.
  assert((ulen >= 0) && (u != NULL));
  assert((vlen >= 0) && (v != NULL));
  // If ulen > 0, then the leading digit of u must be non-zero.
  assert((ulen <= 0) || (u[ulen - 1] != 0));
  assert((vlen <= 0) || (v[vlen - 1] != 0));
#endif

  if (ulen != vlen)
    return (ulen - vlen);

  // ulen == vlen >= 1
  while ((--ulen >= 0) && (u[ulen] == v[ulen]))
    ;

  if (ulen < 0)
    return 0;

#ifdef DEBUG_SYSTEMC
  // Test to see if the result is wrong due to the presence of
  // overflow bits.
  assert((u[ulen] & DIGIT_MASK) != (v[ulen] & DIGIT_MASK));
#endif

  return (int) (u[ulen] - v[ulen]);

}

// Find the index of the first non-zero digit.
// - ulen (before) = the number of digits in u.
// - the returned value = the index of the first non-zero digit. 
// A negative value of -1 indicates that every digit in u is zero.
inline
int
vec_find_first_nonzero(int ulen, const sc_digit *u)
{

#ifdef DEBUG_SYSTEMC
  // assert((ulen <= 0) || (u != NULL));
  assert((ulen > 0) && (u != NULL));
#endif

  while ((--ulen >= 0) && (! u[ulen]))
    ;

  return ulen;
  
}

// Skip all the leading zero digits.  
// - ulen (before) = the number of digits in u.
// - the returned value = the number of non-zero digits in u.
// - the returned value is non-negative.
inline 
int
vec_skip_leading_zeros(int ulen, const sc_digit *u)
{

#ifdef DEBUG_SYSTEMC
  // assert((ulen <= 0) || (u != NULL));
  assert((ulen > 0) && (u != NULL));
#endif

  return (1 + vec_find_first_nonzero(ulen, u));

}

// Compare u and v and return r
//  r = 0 if u == v
//  r < 0 if u < v
//  r > 0 if u > v
inline 
int 
vec_skip_and_cmp(int ulen, const sc_digit *u, 
                 int vlen, const sc_digit *v)
{

#ifdef DEBUG_SYSTEMC
  assert((ulen > 0) && (u != NULL));
  assert((vlen > 0) && (v != NULL));
#endif

  ulen = vec_skip_leading_zeros(ulen, u);
  vlen = vec_skip_leading_zeros(vlen, v);
  // ulen and/or vlen can be equal to zero here.
  return vec_cmp(ulen, u, vlen, v);

}

// Set u[i] = 0 where i = from ... (ulen - 1).
inline
void
vec_zero(int from, int ulen, sc_digit *u)
{

#ifdef DEBUG_SYSTEMC
  assert((ulen > 0) && (u != NULL));
#endif

  for(int i = from; i < ulen; i++)
    u[i] = 0;

}

// Set u[i] = 0 where i = 0 .. (ulen - 1).
inline
void
vec_zero(int ulen, sc_digit *u)
{
  vec_zero(0, ulen, u);
}

// Copy n digits from v to u.
inline
void
vec_copy(int n, sc_digit *u, const sc_digit *v)
{

#ifdef DEBUG_SYSTEMC
  assert((n > 0) && (u != NULL) && (v != NULL));
#endif

  for (int i = 0; i < n; ++i)
    u[i] = v[i];
}

// Copy v to u, where ulen >= vlen, and zero the rest of the digits in u.
inline
void
vec_copy_and_zero(int ulen, sc_digit *u, 
                  int vlen, const sc_digit *v)
{

#ifdef DEBUG_SYSTEMC
  assert((ulen > 0) && (u != NULL));
  assert((vlen > 0) && (v != NULL));
  assert(ulen >= vlen);
#endif

  vec_copy(vlen, u, v);
  vec_zero(vlen, ulen, u);

}

// 2's-complement the digits in u.
inline
void
vec_complement(int ulen, sc_digit *u)
{

#ifdef DEBUG_SYSTEMC
  assert((ulen > 0) && (u != NULL));
#endif

  sc_digit carry = 1;

  for (int i = 0; i < ulen; ++i) {
    carry += (~u[i] & DIGIT_MASK);
    u[i] = carry & DIGIT_MASK;
    carry >>= BITS_PER_DIGIT;
  }
  
}


// ----------------------------------------------------------------------------
//  Functions to handle built-in types or signs.
// ----------------------------------------------------------------------------

// u = v
// - v is an unsigned long or uint64, and positive integer.
template< class Type >
inline
void
from_uint(int ulen, sc_digit *u, Type v)
{

#ifdef DEBUG_SYSTEMC
  // assert((ulen <= 0) || (u != NULL));
  assert((ulen > 0) && (u != NULL));
  assert(v >= 0);
#endif

  int i = 0;

  while (v && (i < ulen)) {
#ifndef _WIN32
    u[i++] = static_cast<sc_digit>( v & DIGIT_MASK );
#else
    u[i++] = ((sc_digit) v) & DIGIT_MASK;
#endif
    v >>= BITS_PER_DIGIT;
  }

  vec_zero(i, ulen, u);

}


// Get u's sign and return its absolute value.
// u can be long, unsigned long, int64, or uint64.
template< class Type >
inline
small_type
get_sign(Type &u) 
{
  if (u > 0)
    return SC_POS;

  if (u == 0)
    return SC_ZERO;

  // no positive number representable for minimum value,
  // leave as is to avoid Undefined Behaviour
  if( SC_LIKELY_( u > (std::numeric_limits<Type>::min)() ) )
    u = -u;

  return SC_NEG;
}


// Return us * vs:
// - Return SC_ZERO if either sign is SC_ZERO.
// - Return SC_POS if us == vs
// - Return SC_NEG if us != vs.
inline
small_type
mul_signs(small_type us, small_type vs)
{
  if ((us == SC_ZERO) || (vs == SC_ZERO))
    return SC_ZERO;

  if (us == vs)
    return SC_POS;

  return SC_NEG;
}


// ----------------------------------------------------------------------------
//  Functions to test for errors and print out error messages.
// ----------------------------------------------------------------------------

#ifdef SC_MAX_NBITS

inline
void
test_bound(int nb)
{
  if (nb > SC_MAX_NBITS) {
      char msg[BUFSIZ];
      std::sprintf( msg, "test_bound( int nb ) : "
	       "nb = %d > SC_MAX_NBITS = %d is not valid",
	       nb, SC_MAX_NBITS );
      SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, msg );
  }
}

#endif

template< class Type >
inline
void
div_by_zero(Type s)
{
  if (s == 0) {
      SC_REPORT_ERROR( sc_core::SC_ID_OPERATION_FAILED_,
		       "div_by_zero<Type>( Type ) : division by zero" );
  }
}


// ----------------------------------------------------------------------------
//  Functions to check if a given vector is zero or make one.
// ----------------------------------------------------------------------------

// If u[i] is zero for every i = 0,..., ulen - 1, return SC_ZERO,
// else return s.
inline
small_type 
check_for_zero(small_type s, int ulen, const sc_digit *u)
{

#ifdef DEBUG_SYSTEMC
  // assert(ulen >= 0);
  assert((ulen > 0) && (u != NULL));
#endif

  if (vec_find_first_nonzero(ulen, u) < 0)
    return SC_ZERO;  

  return s;

}

// If u[i] is zero for every i = 0,..., ulen - 1, return true,
// else return false.
inline
bool
check_for_zero(int ulen, const sc_digit *u)
{

#ifdef DEBUG_SYSTEMC
  // assert(ulen >= 0);
  assert((ulen > 0) && (u != NULL));
#endif

  if (vec_find_first_nonzero(ulen, u) < 0)
    return true;

  return false;

}

inline
small_type
make_zero(int nd, sc_digit *d)
{
  vec_zero(nd, d);
  return SC_ZERO;
}


// ----------------------------------------------------------------------------
//  Functions for both signed and unsigned numbers to convert sign-magnitude
//  (SM) and 2's complement (2C) representations.
//  added = 1 => for signed.
//  added = 0 => for unsigned.
//  IF_SC_SIGNED can be used as 'added'.
// ----------------------------------------------------------------------------

// Trim the extra leading bits of a signed or unsigned number.
inline
void
trim(small_type added, int nb, int nd, sc_digit *d)
{
#ifdef DEBUG_SYSTEMC
  assert((nb > 0) && (nd > 0) && (d != NULL));
#endif

  d[nd - 1] &= one_and_ones(bit_ord(nb - 1) + added);    
}

// Convert an (un)signed number from sign-magnitude representation to
// 2's complement representation and trim the extra bits.
inline
void
convert_SM_to_2C_trimmed(small_type added, 
                         small_type s, int nb, int nd, sc_digit *d)
{
  if (s == SC_NEG) {
    vec_complement(nd, d);
    trim(added, nb, nd, d);
  }
}

// Convert an (un)signed number from sign-magnitude representation to
// 2's complement representation but do not trim the extra bits.
inline
void
convert_SM_to_2C(small_type s, int nd, sc_digit *d)
{
  if (s == SC_NEG)
    vec_complement(nd, d);
}


// ----------------------------------------------------------------------------
//  Functions to convert between sign-magnitude (SM) and 2's complement
//  (2C) representations of signed numbers.
// ----------------------------------------------------------------------------

// Trim the extra leading bits off a signed number.
inline
void
trim_signed(int nb, int nd, sc_digit *d)
{
#ifdef DEBUG_SYSTEMC
  assert((nb > 0) && (nd > 0) && (d != NULL));
#endif

  d[nd - 1] &= one_and_ones(bit_ord(nb - 1) + 1);
}

// Convert a signed number from 2's complement representation to
// sign-magnitude representation, and return its sign. nd is d's
// actual size, without zeros eliminated.
inline
small_type
convert_signed_2C_to_SM(int nb, int nd, sc_digit *d)
{

#ifdef DEBUG_SYSTEMC
  assert((nb > 0) && (nd > 0) && (d != NULL));
#endif

  small_type s;

  int xnb = bit_ord(nb - 1) + 1;

  // Test the sign bit.  
  if (d[nd - 1] & one_and_zeros(xnb - 1)) {
    s = SC_NEG;
    vec_complement(nd, d);
  }
  else
    s = SC_POS;

  // Trim the last digit.
  d[nd - 1] &= one_and_ones(xnb);

  // Check if the new number is zero.
  if (s == SC_POS)
    return check_for_zero(s, nd, d);

  return s;

}

// Convert a signed number from sign-magnitude representation to 2's
// complement representation, get its sign, convert back to
// sign-magnitude representation, and return its sign. nd is d's
// actual size, without zeros eliminated.
inline
small_type 
convert_signed_SM_to_2C_to_SM(small_type s, int nb, int nd, sc_digit *d)
{
  convert_SM_to_2C(s, nd, d);
  return convert_signed_2C_to_SM(nb, nd, d);
}

// Convert a signed number from sign-magnitude representation to 2's
// complement representation and trim the extra bits.
inline
void
convert_signed_SM_to_2C_trimmed(small_type s, int nb, int nd, sc_digit *d)
{
  convert_SM_to_2C_trimmed(1, s, nb, nd, d);
}

// Convert a signed number from sign-magnitude representation to 2's
// complement representation but do not trim the extra bits.
inline
void
convert_signed_SM_to_2C(small_type s, int nd, sc_digit *d)
{
  convert_SM_to_2C(s, nd, d);
}


// ----------------------------------------------------------------------------
//  Functions to convert between sign-magnitude (SM) and 2's complement
//  (2C) representations of unsigned numbers.
// ----------------------------------------------------------------------------

// Trim the extra leading bits off an unsigned number.
inline
void
trim_unsigned(int nb, int nd, sc_digit *d)
{
#ifdef DEBUG_SYSTEMC
  assert((nb > 0) && (nd > 0) && (d != NULL));
#endif

  d[nd - 1] &= one_and_ones(bit_ord(nb - 1));    
}

// Convert an unsigned number from 2's complement representation to
// sign-magnitude representation, and return its sign. nd is d's
// actual size, without zeros eliminated.
inline
small_type
convert_unsigned_2C_to_SM(int nb, int nd, sc_digit *d)
{
  trim_unsigned(nb, nd, d);
  return check_for_zero(SC_POS, nd, d);
}

// Convert an unsigned number from sign-magnitude representation to
// 2's complement representation, get its sign, convert back to
// sign-magnitude representation, and return its sign. nd is d's
// actual size, without zeros eliminated.
inline
small_type
convert_unsigned_SM_to_2C_to_SM(small_type s, int nb, int nd, sc_digit *d)
{
  convert_SM_to_2C(s, nd, d);
  return convert_unsigned_2C_to_SM(nb, nd, d);
}

// Convert an unsigned number from sign-magnitude representation to
// 2's complement representation and trim the extra bits.
inline
void
convert_unsigned_SM_to_2C_trimmed(small_type s, int nb, int nd, sc_digit *d)
{
  convert_SM_to_2C_trimmed(0, s, nb, nd, d);
}

// Convert an unsigned number from sign-magnitude representation to
// 2's complement representation but do not trim the extra bits.
inline
void
convert_unsigned_SM_to_2C(small_type s, int nd, sc_digit *d)
{
  convert_SM_to_2C(s, nd, d);
}


// ----------------------------------------------------------------------------
//  Functions to copy one (un)signed number to another.
// ----------------------------------------------------------------------------

// Copy v to u.
inline
void
copy_digits_signed(small_type &us, 
                   int unb, int und, sc_digit *ud,
                   int vnb, int vnd, const sc_digit *vd)
{

  if (und <= vnd) {

    vec_copy(und, ud, vd);

    if (unb <= vnb)
      us = convert_signed_SM_to_2C_to_SM(us, unb, und, ud);

  }
  else // und > vnd
    vec_copy_and_zero(und, ud, vnd, vd);

}

// Copy v to u.
inline
void
copy_digits_unsigned(small_type &us, 
                     int unb, int und, sc_digit *ud,
                     int /* vnb */, int vnd, const sc_digit *vd)
{

  if (und <= vnd)
    vec_copy(und, ud, vd);

  else // und > vnd
    vec_copy_and_zero(und, ud, vnd, vd);

  us = convert_unsigned_SM_to_2C_to_SM(us, unb, und, ud);

}


// ----------------------------------------------------------------------------
//  Faster set(i, v), without bound checking.
// ----------------------------------------------------------------------------

// A version of set(i, v) without bound checking.
inline
void
safe_set(int i, bool v, sc_digit *d)
{

#ifdef DEBUG_SYSTEMC
  assert((i >= 0) && (d != NULL));
#endif

  int bit_num = bit_ord(i);
  int digit_num = digit_ord(i);    
  
  if (v)
    d[digit_num] |= one_and_zeros(bit_num);      
  else
    d[digit_num] &= ~(one_and_zeros(bit_num));
  
}


// ----------------------------------------------------------------------------
//  Function to check if a double number is bad (NaN or infinite).
// ----------------------------------------------------------------------------

inline
bool
is_nan( double v )
{
    return std::numeric_limits<double>::has_quiet_NaN && (v != v);
}

inline
bool
is_inf( double v )
{
    return v ==  std::numeric_limits<double>::infinity()
        || v == -std::numeric_limits<double>::infinity();
}

inline
void
is_bad_double(double v)
{
// Windows throws exception.
    if( is_nan(v) || is_inf(v) )
        SC_REPORT_ERROR( sc_core::SC_ID_VALUE_NOT_VALID_,
		         "is_bad_double( double v ) : "
		         "v is not finite - NaN or Inf" );
}

} // namespace sc_dt


#endif

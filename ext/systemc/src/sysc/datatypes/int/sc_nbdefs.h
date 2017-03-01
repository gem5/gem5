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

  sc_nbdefs.h -- Top level header file for arbitrary precision signed/unsigned
                 arithmetic. This file defines all the constants needed.

  Original Author: Ali Dasdan, Synopsys, Inc.
  
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_nbdefs.h,v $
// Revision 1.7  2011/02/18 20:19:15  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.6  2011/02/18 20:09:34  acg
//  Philipp A. Hartmann: added alternative #define for Windows to guard.
//
// Revision 1.5  2011/01/20 16:52:20  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.4  2010/02/08 18:35:55  acg
//  Andy Goodrich: Philipp Hartmann's changes for Solaris and Linux 64.
//
// Revision 1.2  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:49:32  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#ifndef SC_NBDEFS_H
#define SC_NBDEFS_H


#include "sysc/kernel/sc_cmnhdr.h"

#include <climits>

#if defined(__sun) || defined(__sun__)
#  include <inttypes.h>
#elif !defined(WIN32) && !defined(_WIN32)
#  include <stdint.h>
#endif

#include "sysc/utils/sc_iostream.h"
#include "sysc/kernel/sc_constants.h"   // For SC_MAX_NBITS

// Activate support mixed operands for concatenation via the comma operator
#define SC_DT_MIXED_COMMA_OPERATORS


namespace sc_dt
{

// ----------------------------------------------------------------------------
//  ENUM : sc_numrep
//
//  Enumeration of number representations for character string conversion.
// ----------------------------------------------------------------------------

enum sc_numrep
{
    SC_NOBASE = 0,
    SC_BIN    = 2,
    SC_OCT    = 8,
    SC_DEC    = 10,
    SC_HEX    = 16,
    SC_BIN_US,
    SC_BIN_SM,
    SC_OCT_US,
    SC_OCT_SM,
    SC_HEX_US,
    SC_HEX_SM,
    SC_CSD
};


// Sign of a number:
#define SC_NEG       -1     // Negative number
#define SC_ZERO      0      // Zero
#define SC_POS       1      // Positive number
#define SC_NOSIGN    2      // Uninitialized sc_signed number

typedef unsigned char uchar;

// A small_type number is at least a char. Defining an int is probably
// better for alignment.
typedef int small_type;

// Attributes of a byte.
#define BITS_PER_BYTE   8
#define BYTE_RADIX      256
#define BYTE_MASK       255

// LOG2_BITS_PER_BYTE = log2(BITS_PER_BYTE), assuming that
// BITS_PER_BYTE is a power of 2.
#define LOG2_BITS_PER_BYTE 3

// Attributes of the unsigned long. These definitions are used mainly in
// the functions that are aware of the internal representation of
// digits, e.g., get/set_packed_rep().
#define BYTES_PER_DIGIT_TYPE  4
#define BITS_PER_DIGIT_TYPE   32

// Attributes of a digit, i.e., unsigned long less the overflow bits.
#define BYTES_PER_DIGIT 4
#define BITS_PER_DIGIT  30
#define DIGIT_RADIX     (1ul << BITS_PER_DIGIT)
#define DIGIT_MASK      (DIGIT_RADIX - 1)
// Make sure that BYTES_PER_DIGIT = ceil(BITS_PER_DIGIT / BITS_PER_BYTE).

// Similar attributes for the half of a digit. Note that
// HALF_DIGIT_RADIX is equal to the square root of DIGIT_RADIX. These
// definitions are used mainly in the multiplication routines.
#define BITS_PER_HALF_DIGIT (BITS_PER_DIGIT / 2)
#define HALF_DIGIT_RADIX    (1ul << BITS_PER_HALF_DIGIT)
#define HALF_DIGIT_MASK     (HALF_DIGIT_RADIX - 1)

// DIV_CEIL2(x, y) = ceil(x / y). x and y are positive numbers.
#define DIV_CEIL2(x, y) (((x) - 1) / (y) + 1)

// DIV_CEIL(x) = ceil(x / BITS_PER_DIGIT) = the number of digits to
// store x bits. x is a positive number.
#define DIV_CEIL(x) DIV_CEIL2(x, BITS_PER_DIGIT)

#ifdef SC_MAX_NBITS
extern const int MAX_NDIGITS;
// Consider a number with x bits another with y bits. The maximum
// number of bits happens when we multiply them. The result will have
// (x + y) bits. Assume that x + y <= SC_MAX_NBITS. Then, DIV_CEIL(x) +
// DIV_CEIL(y) <= DIV_CEIL(SC_MAX_NBITS) + 2. This is the reason for +2
// above. With this change, MAX_NDIGITS must be enough to hold the
// result of any operation.
#endif

// Support for "digit" vectors used to hold the values of sc_signed,
// sc_unsigned, sc_bv_base,  and sc_lv_base data types. This type is also used 
// in the concatenation support. An sc_digit is currently an unsigned 32-bit 
// quantity. The typedef used is an unsigned int, rather than an unsigned long,
// since the unsigned long data type varies in size between 32-bit and 64-bit 
// machines.

typedef unsigned int sc_digit;	// 32-bit unsigned integer

// Support for the long long type. This type is not in the standard
// but is usually supported by compilers.
#ifndef _WIN32
#   if defined(__x86_64__)
        typedef long long          int64;
        typedef unsigned long long uint64;
#   else
        typedef int64_t            int64;
        typedef uint64_t           uint64;
#   endif
    extern const uint64        UINT64_ZERO;
    extern const uint64        UINT64_ONE;
    extern const uint64        UINT64_32ONES;
#else
    typedef __int64            int64;
    typedef unsigned __int64   uint64;
    extern const uint64        UINT64_ZERO;
    extern const uint64        UINT64_ONE;
    extern const uint64        UINT64_32ONES;
#endif


// Bits per ...
// will be deleted in the future. Use numeric_limits instead
#define BITS_PER_CHAR    8
#define BITS_PER_INT    (sizeof(int) * BITS_PER_CHAR)
#define BITS_PER_LONG   (sizeof(long) * BITS_PER_CHAR)
#define BITS_PER_INT64  (sizeof(::sc_dt::int64) * BITS_PER_CHAR)
#define BITS_PER_UINT   (sizeof(unsigned int) * BITS_PER_CHAR)
#define BITS_PER_ULONG  (sizeof(unsigned long) * BITS_PER_CHAR)
#define BITS_PER_UINT64 (sizeof(::sc_dt::uint64) * BITS_PER_CHAR)

// Digits per ...
#define DIGITS_PER_CHAR   1
#define DIGITS_PER_INT    ((BITS_PER_INT+29)/30)
#define DIGITS_PER_LONG   ((BITS_PER_LONG+29)/30)
#define DIGITS_PER_INT64  ((BITS_PER_INT64+29)/30)
#define DIGITS_PER_UINT   ((BITS_PER_UINT+29)/30)
#define DIGITS_PER_ULONG  ((BITS_PER_ULONG+29)/30)
#define DIGITS_PER_UINT64 ((BITS_PER_UINT64+29)/30)

// Above, BITS_PER_X is mainly used for sc_signed, and BITS_PER_UX is
// mainly used for sc_unsigned.

#if defined( _WIN32 ) || defined( __HP_aCC )
typedef unsigned long fmtflags;
#else
typedef ::std::ios::fmtflags fmtflags;
#endif

extern const small_type NB_DEFAULT_BASE ;

// For sc_int code:
#define LLWIDTH  BITS_PER_INT64
#define INTWIDTH BITS_PER_INT

#ifndef _32BIT_

typedef int64 int_type;
typedef uint64 uint_type;
#define SC_INTWIDTH 64
extern const uint64 UINT_ZERO;
extern const uint64 UINT_ONE;

#else

typedef int int_type;
typedef unsigned int uint_type;
#define SC_INTWIDTH 32
extern const unsigned int UINT_ZERO;
extern const unsigned int UINT_ONE;

#endif


#if defined(_MSC_VER) && ( _MSC_VER < 1300 )
    // VC++6 bug
    ::std::ostream& operator << ( ::std::ostream&, int64 );
    ::std::ostream& operator << ( ::std::ostream&, uint64 );
#endif

} // namespace sc_dt


#if defined(_MSC_VER) && ( _MSC_VER < 1300 )

    inline
    ::std::ostream&
    operator << ( ::std::ostream& os, sc_dt::int64 a )
    {
	sc_dt::operator << ( os, a );
	return os;
    }

    inline
    ::std::ostream&
    operator << ( ::std::ostream& os, sc_dt::uint64 a )
    {
	sc_dt::operator << ( os, a );
	return os;
    }

#endif


#endif

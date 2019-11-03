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

  sc_nbutils.cpp -- External and friend functions for both sc_signed and
                    sc_unsigned classes.

  Original Author: Ali Dasdan, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: sc_nbutils.cpp,v $
// Revision 1.4  2011/08/24 22:05:46  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.3  2011/02/18 20:19:15  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.2  2007/11/04 21:26:40  acg
//  Andy Goodrich: added a buffer to the allocation of the q array to address
//  an issue with references outside the array by 1 byte detected by valgrind.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:49:32  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#include <cctype>
#include <cstdio>
#include <cstring>
#include <sstream>

#include "systemc/ext/dt/bit/messages.hh"
#include "systemc/ext/dt/int/messages.hh"
#include "systemc/ext/dt/int/sc_nbutils.hh"
#include "systemc/ext/utils/functions.hh"

namespace sc_dt
{

// only used within vec_from_str (non-standard, deprecated)
static inline void
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
        SC_REPORT_ERROR("not implemented",
                        "is_valid_base( sc_numrep base ) : "
                        "bases SC_CSD, or ending in _US and _SM are "
                        "not supported");
        break;
      default:
        std::stringstream msg;
        msg << "is_valid_base( sc_numrep base ) : base = " << base <<
               " is not valid";
        SC_REPORT_ERROR(sc_core::SC_ID_VALUE_NOT_VALID_, msg.str().c_str());
    }
}

// ----------------------------------------------------------------------------
//  ENUM : sc_numrep
//
//  Enumeration of number representations for character string conversion.
// ----------------------------------------------------------------------------

const std::string
to_string(sc_numrep numrep)
{
    switch (numrep) {
#define CASE_ENUM2STR(Value) case Value: return #Value

      CASE_ENUM2STR(SC_DEC);

      CASE_ENUM2STR(SC_BIN);
      CASE_ENUM2STR(SC_BIN_US);
      CASE_ENUM2STR(SC_BIN_SM);

      CASE_ENUM2STR(SC_OCT);
      CASE_ENUM2STR(SC_OCT_US);
      CASE_ENUM2STR(SC_OCT_SM);

      CASE_ENUM2STR(SC_HEX);
      CASE_ENUM2STR(SC_HEX_US);
      CASE_ENUM2STR(SC_HEX_SM);

      CASE_ENUM2STR(SC_CSD);

#undef CASE_ENUM2STR

      default:
        return "unknown";
    }
}

// ----------------------------------------------------------------------------
//  SECTION: General utility functions.
// ----------------------------------------------------------------------------

// Return the number of characters to advance the source of c.  This
// function implements one move of the FSM to parse the following
// regular expressions. Error checking is done in the caller.

small_type
fsm_move(char c, small_type &b, small_type &s, small_type &state)
{
    // Possible regular expressions (REs):
    // Let N = any digit depending on the base.
    //    1. [0|1|..|9]N*
    //    2. [+|-][0|1|..|9]N*
    //    3. 0[b|B|d|D|o|O|x|X][0|1|..|F]N*
    //    4. [+|-]?0[b|B|d|D|o|O|x|X][0|1|..|F]N*
    //
    // The finite state machine (FMS) to parse these regular expressions
    // has 4 states, 0 to 3. 0 is the initial state and 3 is the final
    // state.
    //
    // Default sign = SC_POS, default base = NB_DEFAULT_BASE.

    switch (state) {
      case 0: // The initial state.
        switch (c) {
          case '0': s = SC_POS; state = 1; return 0; // RE 1 or 3
          case '+': s = SC_POS; state = 2; return 1; // RE 2
          case '-': s = SC_NEG; state = 2; return 1; // RE 2
          default:
            s = SC_POS; b = NB_DEFAULT_BASE; state = 3; return 0; // RE 1
        }
        // break; //unreachable code
      case 1: // 0...
        switch (c) {
          case 'x': case 'X': b = SC_HEX; state = 3; return 2; // RE 3 or 4
          case 'd': case 'D': b = SC_DEC; state = 3; return 2; // RE 3 or 4
          case 'o': case 'O': b = SC_OCT; state = 3; return 2; // RE 3 or 4
          case 'b': case 'B': b = SC_BIN; state = 3; return 2; // RE 3 or 4
          default: b = NB_DEFAULT_BASE; state = 3; return 0; // RE 1
        }
        // break; //unreachable code
      case 2: // +... or -...
        switch (c) {
          case '0': state = 1; return 0; // RE 2 or 4
          default: b = NB_DEFAULT_BASE; state = 3; return 0; // RE 2
        }
        // break; //unreachable code
      case 3: // The final state.
        break;
      default:
        // Any other state is not possible.
        sc_assert((0 <= state) && (state <= 3));
    } // switch
    return 0;
}


// Get base b and sign s of the number in the char string v. Return a
// pointer to the first char after the point where b and s are
// determined or where the end of v is reached. The input string v has
// to be null terminated.
const char *
get_base_and_sign(const char *v, small_type &b, small_type &s)
{
#ifdef DEBUG_SYSTEMC
    sc_assert(v != NULL);
#endif
    const small_type STATE_START = 0;
    const small_type STATE_FINISH = 3;

    // Default sign = SC_POS, default base = 10.
    s = SC_POS;
    b = NB_DEFAULT_BASE;

    small_type state = STATE_START;
    small_type nskip = 0; // Skip that many chars.
    const char *u = v;

    while (*u) {
        if (isspace(*u)) { // Skip white space.
            ++u;
        } else {
            nskip += fsm_move(*u, b, s, state);
            if (state == STATE_FINISH)
              break;
            else
              ++u;
        }
    }

    // Test to see if the above loop executed more than it should
    // have. The max number of skipped chars is equal to the length of
    // the longest format specifier, e.g., "-0x".
    sc_assert(nskip <= 3);

    v += nskip;

    // Handles empty strings or strings without any digits after the
    // base or base and sign specifier.
    if (*v == '\0') {
        static const char msg[] =
            "get_base_and_sign( const char* v, small_type&, small_type& ) : "
            "v = \"\" is not valid";
        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_, msg);
    }
    return v;
}

//-----------------------------------------------------------------------------
//"parse_binary_bits"
//
// This function parses the supplied string into the supplied vector as a
// right justified bit value.
//    src_p  -> character string representing the bits to be parsed.
//    dst_n  =  number of words in data_p and ctrl_p.
//    data_p -> words w/BITS_PER_DIGIT bits to receive the value's data bits.
//    ctrl_p -> words w/BITS_PER_DIGIT bits to receive the value's control
//              bits, or zero.
// Result is true if value was non-zero.
//-----------------------------------------------------------------------------
void
parse_binary_bits(const char *src_p, int dst_n,
                  sc_digit *data_p, sc_digit *ctrl_p)
{
    int bit_i; // Number of bit now processing.
    sc_digit ctrl; // Control word now assembling.
    sc_digit data; // Data word now assembling.
    int delta_n; // src_n - dst_n*BITS_PER_DIGIT.
    int src_i; // Index in src_p now accessing (left to right).
    int src_n; // Length of source that is left in bits.
    int word_i; // Bit within word now accessing (left to right).

    // MAKE SURE WE HAVE A STRING TO PARSE:
    if (src_p == 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                        "character string is zero");
        return;
    }
    if (*src_p == 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                        "character string is empty");
        return;
    }


    // INDEX INTO THE SOURCE TO A DEPTH THAT WILL ACCOMODATE OUR SIZE:
    //
    // If the source is smaller than our value initialize our value to zero.

    src_n = strlen(src_p);
    delta_n = src_n - (dst_n*BITS_PER_DIGIT);
    if (delta_n > 0) {
        src_p = &src_p[delta_n];
        src_n -= delta_n;
    } else {
        for (word_i = 0; word_i < dst_n; word_i++)
            data_p[word_i] = 0;
        if (ctrl_p)
            for (word_i = 0; word_i < dst_n; word_i++)
                ctrl_p[word_i] = 0;
    }

    // LOOP OVER THE SOURCE ASSEMBLING WORDS AND PLACING THEM IN OUR VALUE:
    //
    // We stride right to left through the source in BITS_PER_DIGIT chunks.
    // Each of those chunks is processed from left to right a bit at a time.
    // We process the high order word specially, since there are less bits.
    src_n = src_n - BITS_PER_DIGIT;
    for (word_i=0; word_i < dst_n; word_i++) {
        src_i = src_n;

        // PARTIAL LAST WORD TO ASSEMBLE:
        if (src_i < 0) {
            src_n += BITS_PER_DIGIT;
            data = 0;
            ctrl = 0;
            for (src_i = 0; src_i < src_n; src_i++) {
                ctrl = ctrl << 1;
                data = data << 1;
                switch (src_p[src_i]) {
                  case 'X':
                  case 'x': ctrl = ctrl | 1; data = data | 1; break;
                  case '1': data = data | 1; break;
                  case 'Z':
                  case 'z': ctrl = ctrl | 1; break;
                  case '0': break;
                  default:
                    {
                        std::stringstream msg;
                        msg << "character string '" << src_p <<
                               "' is not valid";
                        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                                        msg.str().c_str());
                        return;
                    }
                    break;
                }
            }
            if (ctrl_p)
                ctrl_p[word_i] = ctrl;
            data_p[word_i] = data;
            break;
        }

        // FULL WORD TO BE ASSEMBLED:
        ctrl = 0;
        data = 0;
        for (bit_i = 0; bit_i < BITS_PER_DIGIT; bit_i++) {
            ctrl = ctrl << 1;
            data = data << 1;
            switch (src_p[src_i++]) {
              case 'X':
              case 'x': ctrl = ctrl | 1; data = data | 1; break;
              case '1': data = data | 1; break;
              case 'Z':
              case 'z': ctrl = ctrl | 1; break;
              case '0': break;
              default:
                {
                    std::stringstream msg;
                    msg << "character string '" << src_p <<
                           "' is not valid";
                    SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                                    msg.str().c_str());
                    return;
                }
                break;
            }
        }
        if (ctrl_p)
            ctrl_p[word_i] = ctrl;
        data_p[word_i] = data;
        src_n = src_n - BITS_PER_DIGIT;
    }
}


//-----------------------------------------------------------------------------
//"parse_hex_bits"
//
// This function parses the supplied string into the supplied vector as a
// right justified bit value.
//    src_p  -> character string representing the bits to be parsed.
//    dst_n  =  number of words in data_p and ctrl_p.
//    data_p -> words w/32 bits to receive the value's data bits.
//    ctrl_p -> words w/32 bits to receive the value's control bits,
//              or zero.
// Result is true if value was non-zero.
//-----------------------------------------------------------------------------
void
parse_hex_bits(const char *src_p, int dst_n,
               sc_digit *data_p, sc_digit *ctrl_p)
{
    sc_digit ctrl; // Control word now assembling.
    sc_digit data; // Data word now assembling.
    int delta_n; // src_n - dst_n*BITS_PER_DIGIT.
    int digit_i; // Number of digit now processing.
    int src_i; // Index in src_p now accessing (left to right).
    int src_n; // Length of source that is left in bits.
    int word_i; // Bit within word now accessing (left to right).

    // MAKE SURE WE HAVE A STRING TO PARSE:
    if (src_p == 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                        "character string is zero");
        return;
    }
    if (*src_p == 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                        "character string is empty");
        return;
    }

    // INDEX INTO THE SOURCE TO A DEPTH THAT WILL ACCOMODATE OUR SIZE:
    //
    // If the source is smaller than our value initialize our value to zero.
    src_n = strlen(src_p);
    delta_n = src_n - (dst_n*8);
    if (delta_n > 0) {
        src_p = &src_p[delta_n];
        src_n -= delta_n;
    } else {
        for (word_i = 0; word_i < dst_n; word_i++)
            data_p[word_i] = 0;
        if (ctrl_p)
            for (word_i = 0; word_i < dst_n; word_i++)
                ctrl_p[word_i] = 0;
    }

    // LOOP OVER THE SOURCE ASSEMBLING WORDS AND PLACING THEM IN OUR VALUE:
    //
    // We stride right to left through the source in BITS_PER_DIGIT chunks.
    // Each of those chunks is processed from left to right a bit at a time.
    // We process the high order word specially, since there are less bits.
    src_n = src_n - 8;
    for (word_i = 0; word_i < dst_n; word_i++) {
        src_i = src_n;

        // PARTIAL LAST WORD TO ASSEMBLE:
        if (src_i < 0) {
            src_n += 8;
            data = 0;
            ctrl = 0;
            for (src_i = 0; src_i < src_n; src_i++) {
                ctrl = ctrl << 4;
                data = data << 4;
                switch (src_p[src_i]) {
                  case 'X':
                  case 'x': ctrl = ctrl | 15; data = data | 15; break;
                  case 'F':
                  case 'f': data = data | 15; break;
                  case 'E':
                  case 'e': data = data | 14; break;
                  case 'D':
                  case 'd': data = data | 13; break;
                  case 'C':
                  case 'c': data = data | 12; break;
                  case 'B':
                  case 'b': data = data | 11; break;
                  case 'A':
                  case 'a': data = data | 10; break;
                  case '9': data = data |  9; break;
                  case '8': data = data |  8; break;
                  case '7': data = data |  7; break;
                  case '6': data = data |  6; break;
                  case '5': data = data |  5; break;
                  case '4': data = data |  4; break;
                  case '3': data = data |  3; break;
                  case '2': data = data |  2; break;
                  case '1': data = data |  1; break;
                  case '0': break;
                  case 'Z':
                  case 'z': ctrl = ctrl | 15; break;
                  default:
                    {
                        std::stringstream msg;
                        msg << "character string '" << src_p <<
                               "' is not valid";
                        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                                        msg.str().c_str());
                        return;
                    }
                    break;
                }
            }
            if (ctrl_p)
                ctrl_p[word_i] = ctrl;
            data_p[word_i] = data;
            break;
        }

        // FULL WORD TO BE ASSEMBLED:
        ctrl = 0;
        data = 0;
        for (digit_i = 0; digit_i < 8; digit_i++) {
            ctrl = ctrl << 4;
            data = data << 4;
            switch (src_p[src_i++]) {
              case 'X':
              case 'x': ctrl = ctrl | 15; data = data | 15; break;
              case 'F':
              case 'f': data = data | 15; break;
              case 'E':
              case 'e': data = data | 14; break;
              case 'D':
              case 'd': data = data | 13; break;
              case 'C':
              case 'c': data = data | 12; break;
              case 'B':
              case 'b': data = data | 11; break;
              case 'A':
              case 'a': data = data | 10; break;
              case '9': data = data |  9; break;
              case '8': data = data |  8; break;
              case '7': data = data |  7; break;
              case '6': data = data |  6; break;
              case '5': data = data |  5; break;
              case '4': data = data |  4; break;
              case '3': data = data |  3; break;
              case '2': data = data |  2; break;
              case '1': data = data |  1; break;
              case '0': break;
              case 'Z':
              case 'z': ctrl = ctrl | 15; break;
              default:
                {
                    std::stringstream msg;
                    msg << "character string '" << src_p << "' is not valid";
                    SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                                    msg.str().c_str() );
                    return;
                }
                break;
            }
        }
        if (ctrl_p)
            ctrl_p[word_i] = ctrl;
        data_p[word_i] = data;
        src_n = src_n - BITS_PER_DIGIT;
    }
}


// ----------------------------------------------------------------------------
//  SECTION: Utility functions involving unsigned vectors.
// ----------------------------------------------------------------------------

// Read u from a null terminated char string v. Note that operator>>
// in sc_nbcommon.cpp is similar to this function.
small_type
vec_from_str(int unb, int und, sc_digit *u, const char *v, sc_numrep base)
{

#ifdef DEBUG_SYSTEMC
    sc_assert((unb > 0) && (und > 0) && (u != NULL));
    sc_assert(v != NULL);
#endif
    is_valid_base(base);

    small_type b, s; // base and sign.

    v = get_base_and_sign(v, b, s);

    if (base != SC_NOBASE) {
        if (b == NB_DEFAULT_BASE) {
            b = base;
        } else {
            std::stringstream msg;
            msg << "vec_from_str( int, int, sc_digit*, const char*, " <<
                   "sc_numrep base ) : base = " << base <<
                   " does not match the default base";
            SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                            msg.str().c_str());
            return 0;
        }
    }

    vec_zero(und, u);

    char c;
    for (; (c = *v); ++v) {
        if (isalnum(c)) {
            small_type val;  // Numeric value of a char.

            if (isalpha(c)) // Hex digit.
                val = toupper(c) - 'A' + 10;
            else
                val = c - '0';

            if (val >= b) {
                std::stringstream msg;
                msg << "vec_from_str( int, int, sc_digit*, const char*, " <<
                       "sc_numrep base ) : '" << *v << "' is not a valid " <<
                       "digit in base " << b;
                SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                                msg.str().c_str());
                return 0;
            }

            // digit = digit * b + val;
            vec_mul_small_on(und, u, b);

            if (val)
                vec_add_small_on(und, u, val);
        } else {
            std::stringstream msg;
            msg << "vec_from_str( int, int, sc_digit*, const char*, " <<
                   "sc_numrep base ) : '" << *v << "' is not a valid " <<
                   "digit in base " << b;
            SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                            msg.str().c_str());
            return 0;
        }
    }

    return convert_signed_SM_to_2C_to_SM(s, unb, und, u);
}


// All vec_ functions assume that the vector to hold the result,
// called w, has sufficient length to hold the result. For efficiency
// reasons, we do not test whether or not we are out of bounds.

// Compute w = u + v, where w, u, and v are vectors.
// - ulen >= vlen
// - wlen >= sc_max(ulen, vlen) + 1
void
vec_add(int ulen, const sc_digit *u, int vlen, const sc_digit *v, sc_digit *w)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((vlen > 0) && (v != NULL));
    sc_assert(w != NULL);
    sc_assert(ulen >= vlen);
#endif

    const sc_digit *uend = (u + ulen);
    const sc_digit *vend = (v + vlen);

    sc_digit carry = 0; // Also used as sum to save space.

    // Add along the shorter v.
    while (v < vend) {
        carry += (*u++) + (*v++);
        (*w++) = carry & DIGIT_MASK;
        carry >>= BITS_PER_DIGIT;
    }

    // Propagate the carry.
    while (carry && (u < uend)) {
        carry = (*u++) + 1;
        (*w++) = carry & DIGIT_MASK;
        carry >>= BITS_PER_DIGIT;
    }

    // Copy the rest of u to the result.
    while (u < uend)
        (*w++) = (*u++);

    // Propagate the carry if it is still 1.
    if (carry)
        (*w) = 1;
}


// Compute u += v, where u and v are vectors.
// - ulen >= vlen
void
vec_add_on(int ulen, sc_digit *ubegin, int vlen, const sc_digit *v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (ubegin != NULL));
    sc_assert((vlen > 0) && (v != NULL));
    sc_assert(ulen >= vlen);
#endif

    sc_digit *u = ubegin;
    const sc_digit *uend = (u + ulen);
    const sc_digit *vend = (v + vlen);

    sc_digit carry = 0; // Also used as sum to save space.

    // Add along the shorter v.
    while (v < vend) {
        carry += (*u) + (*v++);
        (*u++) = carry & DIGIT_MASK;
        carry >>= BITS_PER_DIGIT;
    }

    // Propagate the carry.
    while (carry && (u < uend)) {
        carry = (*u) + 1;
        (*u++) = carry & DIGIT_MASK;
        carry >>= BITS_PER_DIGIT;
    }

#ifdef   DEBUG_SYSTEMC
    if (carry != 0) {
        SC_REPORT_WARNING(sc_core::SC_ID_WITHOUT_MESSAGE_,
                          "vec_add_on( int, sc_digit*, int, const "
                          "sc_digit* ) : "
                          "result of addition is wrapped around");
    }
#endif
}


// Compute u += v, where u and v are vectors.
// - ulen < vlen
void
vec_add_on2(int ulen, sc_digit *ubegin, int,
#ifdef DEBUG_SYSTEMC
            vlen,
#endif
            const sc_digit *v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (ubegin != NULL));
    sc_assert((vlen > 0) && (v != NULL));
    sc_assert(ulen < vlen);
#endif

    sc_digit *u = ubegin;
    const sc_digit *uend = (u + ulen);

    sc_digit carry = 0; // Also used as sum to save space.

    // Add along the shorter u.
    while (u < uend) {
        carry += (*u) + (*v++);
        (*u++) = carry & DIGIT_MASK;
        carry >>= BITS_PER_DIGIT;
    }

#ifdef   DEBUG_SYSTEMC
    if (carry != 0) {
        SC_REPORT_WARNING(sc_core::SC_ID_WITHOUT_MESSAGE_,
                          "vec_add_on2( int, sc_digit*, int, const "
                          "sc_digit* ) : "
                          "result of addition is wrapped around");
    }
#endif
}


// Compute w = u + v, where w and u are vectors, and v is a scalar.
void
vec_add_small(int ulen, const sc_digit *u, sc_digit v, sc_digit *w)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert(w != NULL);
#endif

    const sc_digit *uend = (u + ulen);

    // Add along the shorter v.
    sc_digit carry = (*u++) + v;
    (*w++) = carry & DIGIT_MASK;
    carry >>= BITS_PER_DIGIT;

    // Propagate the carry.
    while (carry && (u < uend)) {
        carry = (*u++) + 1;
        (*w++) = carry & DIGIT_MASK;
        carry >>= BITS_PER_DIGIT;
    }

    // Copy the rest of u to the result.
    while (u < uend)
        (*w++) = (*u++);

    // Propagate the carry if it is still 1.
    if (carry)
        (*w) = 1;
}

// Compute u += v, where u is vectors, and v is a scalar.
void
vec_add_small_on(int ulen, sc_digit *u, sc_digit v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
#endif

    int i = 0;

    while (v && (i < ulen)) {
        v += u[i];
        u[i++] = v & DIGIT_MASK;
        v >>= BITS_PER_DIGIT;
    }

#ifdef   DEBUG_SYSTEMC
    if (v != 0) {
        SC_REPORT_WARNING(sc_core::SC_ID_WITHOUT_MESSAGE_,
                          "vec_add_small_on( int, sc_digit*, unsigned "
                          "long ) : "
                          "result of addition is wrapped around");
    }
#endif
}

// Compute w = u - v, where w, u, and v are vectors.
// - ulen >= vlen
// - wlen >= sc_max(ulen, vlen)
void
vec_sub(int ulen, const sc_digit *u, int vlen, const sc_digit *v, sc_digit *w)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((vlen > 0) && (v != NULL));
    sc_assert(w != NULL);
    sc_assert(ulen >= vlen);
#endif

    const sc_digit *uend = (u + ulen);
    const sc_digit *vend = (v + vlen);

    sc_digit borrow = 0; // Also used as diff to save space.

    // Subtract along the shorter v.
    while (v < vend) {
        borrow = ((*u++) + DIGIT_RADIX) - (*v++) - borrow;
        (*w++) = borrow & DIGIT_MASK;
        borrow = 1 - (borrow >> BITS_PER_DIGIT);
    }

    // Propagate the borrow.
    while (borrow && (u < uend)) {
        borrow = ((*u++) + DIGIT_RADIX) - 1;
        (*w++) = borrow & DIGIT_MASK;
        borrow = 1 - (borrow >> BITS_PER_DIGIT);
    }

#ifdef DEBUG_SYSTEMC
    sc_assert(borrow == 0);
#endif

    // Copy the rest of u to the result.
    while (u < uend)
        (*w++) = (*u++);
}

// Compute u = u - v, where u and v are vectors.
// - u > v
// - ulen >= vlen
void
vec_sub_on(int ulen, sc_digit *ubegin, int vlen, const sc_digit *v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (ubegin != NULL));
    sc_assert((vlen > 0) && (v != NULL));
    sc_assert(ulen >= vlen);
#endif

    sc_digit *u = ubegin;
    const sc_digit *uend = (u + ulen);
    const sc_digit *vend = (v + vlen);

    sc_digit borrow = 0;   // Also used as diff to save space.

    // Subtract along the shorter v.
    while (v < vend) {
        borrow = ((*u) + DIGIT_RADIX) - (*v++) - borrow;
        (*u++) = borrow & DIGIT_MASK;
        borrow = 1 - (borrow >> BITS_PER_DIGIT);
    }

    // Propagate the borrow.
    while (borrow && (u < uend)) {
        borrow = ((*u) + DIGIT_RADIX) - 1;
        (*u++) = borrow & DIGIT_MASK;
        borrow = 1 - (borrow >> BITS_PER_DIGIT);
    }

#ifdef DEBUG_SYSTEMC
    sc_assert(borrow == 0);
#endif
}

// Compute u = v - u, where u and v are vectors.
// - v > u
// - ulen <= vlen or ulen > ulen
void
vec_sub_on2(int ulen, sc_digit *ubegin, int vlen, const sc_digit *v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (ubegin != NULL));
    sc_assert((vlen > 0) && (v != NULL));
#endif

    sc_digit *u = ubegin;
    const sc_digit *uend = (u + sc_min(ulen, vlen));

    sc_digit borrow = 0;   // Also used as diff to save space.

    // Subtract along the shorter u.
    while (u < uend) {
        borrow = ((*v++) + DIGIT_RADIX) - (*u) - borrow;
        (*u++) = borrow & DIGIT_MASK;
        borrow = 1 - (borrow >> BITS_PER_DIGIT);
    }

#ifdef DEBUG_SYSTEMC
    if (borrow != 0) {
        SC_REPORT_WARNING(sc_core::SC_ID_WITHOUT_MESSAGE_,
                          "vec_sub_on2( int, sc_digit*, int, const "
                          "sc_digit* ) : "
                          "result of subtraction is wrapped around");
    }
#endif
}

// Compute w = u - v, where w and u are vectors, and v is a scalar.
void
vec_sub_small(int ulen, const sc_digit *u, sc_digit v, sc_digit *w)
{
#ifdef DEBUG_SYSTEMC
    sc_assert(ulen > 0);
    sc_assert(u != NULL);
#endif

    const sc_digit *uend = (u + ulen);

    // Add along the shorter v.
    sc_digit borrow = ((*u++) + DIGIT_RADIX) - v;
    (*w++) = borrow & DIGIT_MASK;
    borrow = 1 - (borrow >> BITS_PER_DIGIT);

    // Propagate the borrow.
    while (borrow && (u < uend)) {
        borrow = ((*u++) + DIGIT_RADIX) - 1;
        (*w++) = borrow & DIGIT_MASK;
        borrow = 1 - (borrow >> BITS_PER_DIGIT);
    }

#ifdef   DEBUG_SYSTEMC
    sc_assert(borrow == 0);
#endif

    // Copy the rest of u to the result.
    while (u < uend)
        (*w++) = (*u++);
}


// Compute u -= v, where u is vectors, and v is a scalar.
void
vec_sub_small_on(int ulen, sc_digit *u, sc_digit v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
#endif

    for (int i = 0; i < ulen; ++i) {
      v = (u[i] + DIGIT_RADIX) - v;
      u[i] = v & DIGIT_MASK;
      v = 1 - (v >> BITS_PER_DIGIT);
    }

#ifdef DEBUG_SYSTEMC
    sc_assert(v == 0);
#endif
}

// Compute w = u * v, where w, u, and v are vectors.
void
vec_mul(int ulen, const sc_digit *u, int vlen, const sc_digit *vbegin,
        sc_digit *wbegin)
{

  /* Consider u = Ax + B and v = Cx + D where x is equal to
     HALF_DIGIT_RADIX. In other words, A is the higher half of u and
     B is the lower half of u. The interpretation for v is
     similar. Then, we have the following picture:

              u_h     u_l
     u: -------- --------
               A        B

              v_h     v_l
     v: -------- --------
               C        D

     result (d):
     carry_before:                           -------- --------
                                              carry_h  carry_l
     result_before:        -------- -------- -------- --------
                               R1_h     R1_l     R0_h     R0_l
                                             -------- --------
                                                 BD_h     BD_l
                                    -------- --------
                                        AD_h     AD_l
                                    -------- --------
                                        BC_h     BC_l
                           -------- --------
                               AC_h     AC_l
     result_after:         -------- -------- -------- --------
                              R1_h'    R1_l'    R0_h'    R0_l'

     prod_l = R0_h|R0_l + B * D  + 0|carry_l
            = R0_h|R0_l + BD_h|BD_l + 0|carry_l

     prod_h = A * D + B * C + high_half(prod_l) + carry_h
            = AD_h|AD_l + BC_h|BC_l + high_half(prod_l) + 0|carry_h

     carry = A * C + high_half(prod_h)
           = AC_h|AC_l + high_half(prod_h)

     R0_l' = low_half(prod_l)

     R0_h' = low_half(prod_h)

     R0 = high_half(prod_h)|low_half(prod_l)

     where '|' is the concatenation operation and the suffixes 0 and 1
     show the iteration number, i.e., 0 is the current iteration and 1
     is the next iteration.

     NOTE: sc_max(prod_l, prod_h, carry) <= 2 * x^2 - 1, so any
     of these numbers can be stored in a digit.

     NOTE: low_half(u) returns the lower BITS_PER_HALF_DIGIT of u,
     whereas high_half(u) returns the rest of the bits, which may
     contain more bits than BITS_PER_HALF_DIGIT.
  */

#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((vlen > 0) && (vbegin != NULL));
    sc_assert(wbegin != NULL);
#endif

#define prod_h carry
    const sc_digit *uend = (u + ulen);
    const sc_digit *vend = (vbegin + vlen);

    while (u < uend) {
        sc_digit u_h = (*u++); // A|B
        sc_digit u_l = low_half(u_h); // B
        u_h = high_half(u_h); // A

#ifdef DEBUG_SYSTEMC
        // The overflow bits must be zero.
        sc_assert(u_h == (u_h & HALF_DIGIT_MASK));
#endif
        sc_digit carry = 0;
        sc_digit *w = (wbegin++);
        const sc_digit *v = vbegin;

        while (v < vend) {
            sc_digit v_h = (*v++); // C|D
            sc_digit v_l = low_half(v_h); // D

            v_h = high_half(v_h); // C

#ifdef   DEBUG_SYSTEMC
            // The overflow bits must be zero.
            sc_assert(v_h == (v_h & HALF_DIGIT_MASK));
#endif

            sc_digit prod_l = (*w) + u_l * v_l + low_half(carry);
            prod_h = u_h * v_l + u_l * v_h +
                high_half(prod_l) + high_half(carry);
            (*w++) = concat(low_half(prod_h), low_half(prod_l));
            carry = u_h * v_h + high_half(prod_h);
        }
        (*w) = carry;
    }
#undef prod_h
}

// Compute w = u * v, where w and u are vectors, and v is a scalar.
// - 0 < v < HALF_DIGIT_RADIX.
void
vec_mul_small(int ulen, const sc_digit *u, sc_digit v, sc_digit *w)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert(w != NULL);
    sc_assert((0 < v) && (v < HALF_DIGIT_RADIX));
#endif

#define prod_h carry

    const sc_digit *uend = (u + ulen);
    sc_digit carry = 0;
    while (u < uend) {
        sc_digit u_AB = (*u++);
#ifdef DEBUG_SYSTEMC
        // The overflow bits must be zero.
        sc_assert(high_half(u_AB) == high_half_masked(u_AB));
#endif
        sc_digit prod_l = v * low_half(u_AB) + low_half(carry);
        prod_h = v * high_half(u_AB) + high_half(prod_l) + high_half(carry);
        (*w++) = concat(low_half(prod_h), low_half(prod_l));
        carry = high_half(prod_h);
    }
    (*w) = carry;
#undef prod_h
}

// Compute u = u * v, where u is a vector, and v is a scalar.
// - 0 < v < HALF_DIGIT_RADIX.
void
vec_mul_small_on(int ulen, sc_digit *u, sc_digit v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((0 < v) && (v < HALF_DIGIT_RADIX));
#endif

#define   prod_h carry
    sc_digit carry = 0;
    for (int i = 0; i < ulen; ++i) {
#ifdef DEBUG_SYSTEMC
        // The overflow bits must be zero.
        sc_assert(high_half(u[i]) == high_half_masked(u[i]));
#endif
        sc_digit prod_l = v * low_half(u[i]) + low_half(carry);
        prod_h = v * high_half(u[i]) + high_half(prod_l) + high_half(carry);
        u[i] = concat(low_half(prod_h), low_half(prod_l));
        carry = high_half(prod_h);
    }
#undef   prod_h

#ifdef   DEBUG_SYSTEMC
    if (carry != 0) {
        SC_REPORT_WARNING(sc_core::SC_ID_WITHOUT_MESSAGE_,
                          "vec_mul_small_on( int, sc_digit*, unsigned "
                          "long ) : "
                          "result of multiplication is wrapped around");
    }
#endif
}

// Compute w = u / v, where w, u, and v are vectors.
// - u and v are assumed to have at least two digits as uchars.
void
vec_div_large(int ulen, const sc_digit *u, int vlen, const sc_digit *v,
              sc_digit *w)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((vlen > 0) && (v != NULL));
    sc_assert(w != NULL);
    sc_assert(BITS_PER_DIGIT >= 3 * BITS_PER_BYTE);
#endif

    // We will compute q = x / y where x = u and y = v. The reason for
    // using x and y is that x and y are BYTE_RADIX copies of u and v,
    // respectively. The use of BYTE_RADIX radix greatly simplifies the
    // complexity of the division operation. These copies are also
    // needed even when we use DIGIT_RADIX representation.

    int xlen = BYTES_PER_DIGIT * ulen + 1;
    int ylen = BYTES_PER_DIGIT * vlen;

#ifdef SC_MAX_NBITS
    uchar x[DIV_CEIL2(SC_MAX_NBITS, BITS_PER_BYTE)];
    uchar y[DIV_CEIL2(SC_MAX_NBITS, BITS_PER_BYTE)];
    uchar q[DIV_CEIL2(SC_MAX_NBITS, BITS_PER_BYTE)];
#else
    uchar *x = new uchar[xlen];
    uchar *y = new uchar[ylen];
    // valgrind complains about us accessing too far to so leave a buffer.
    uchar *q = new uchar[(xlen - ylen) + 10];
#endif

    // q corresponds to w.

    // Set (uchar) x = (sc_digit) u.
    xlen = vec_to_char(ulen, u, xlen, x);

    // Skip all the leading zeros in x.
    while ((--xlen >= 0) && (! x[xlen]))
        continue;
    xlen++;

    // Set (uchar) y = (sc_digit) v.
    ylen = vec_to_char(vlen, v, ylen, y);

    // Skip all the leading zeros in y.
    while ((--ylen >= 0) && (! y[ylen]))
        continue;
    ylen++;

#ifdef DEBUG_SYSTEMC
    sc_assert(xlen > 1);
    sc_assert(ylen > 1);
#endif

    // At this point, all the leading zeros are eliminated from x and y.

    // Zero the last digit of x.
    x[xlen] = 0;

    // The first two digits of y.
    sc_digit y2 = (y[ylen - 1] << BITS_PER_BYTE) + y[ylen - 2];

    const sc_digit DOUBLE_BITS_PER_BYTE = 2 * BITS_PER_BYTE;

    // Find each q[k].
    for (int k = (xlen - ylen); k >= 0; --k) {
        // qk is a guess for q[k] such that q[k] = qk or qk - 1.
        sc_digit qk;

        // Find qk by just using 2 digits of y and 3 digits of x. The
        // following code assumes that sizeof(sc_digit) >= 3 BYTEs.
        int k2 = k + ylen;

        qk = ((x[k2] << DOUBLE_BITS_PER_BYTE) +
              (x[k2 - 1] << BITS_PER_BYTE) + x[k2 - 2]) / y2;

        if (qk >= BYTE_RADIX) // qk cannot be larger than the largest
            qk = BYTE_RADIX - 1; // digit in BYTE_RADIX.

        // q[k] = qk or qk - 1. The following if-statement determines which:
        if (qk) {
            uchar *xk = (x + k);  // A shortcut for x[k].

            // x = x - y * qk :
            sc_digit carry = 0;

            for (int i = 0; i < ylen; ++i) {
                carry += y[i] * qk;
                sc_digit diff = (xk[i] + BYTE_RADIX) - (carry & BYTE_MASK);
                xk[i] = (uchar)(diff & BYTE_MASK);
                carry = (carry >> BITS_PER_BYTE) +
                    (1 - (diff >> BITS_PER_BYTE));
            }

            // If carry, qk may be one too large.
            if (carry) {
                // 2's complement the last digit.
                carry = (xk[ylen] + BYTE_RADIX) - carry;
                xk[ylen] = (uchar)(carry & BYTE_MASK);
                carry = 1 - (carry >> BITS_PER_BYTE);

                if (carry) {

                  // qk was one too large, so decrement it.
                  --qk;

                  // Since qk was decreased by one, y must be added to x:
                  // x = x - y * (qk - 1) = x - y * qk + y = x_above + y.
                  carry = 0;

                  for (int i = 0; i < ylen; ++i) {
                      carry += xk[i] + y[i];
                      xk[i] = (uchar)(carry & BYTE_MASK);
                      carry >>= BITS_PER_BYTE;
                  }

                  if (carry)
                      xk[ylen] = (uchar)((xk[ylen] + 1) & BYTE_MASK);

                }  // second if carry
            }  // first if carry
        }  // if qk
        q[k] = (uchar)qk;
    }  // for k

    // Set (sc_digit) w = (uchar) q.
    vec_from_char(xlen - ylen + 1, q, ulen, w);

#ifndef SC_MAX_NBITS
    delete [] x;
    delete [] y;
    delete [] q;
#endif

}

// Compute w = u / v, where u and w are vectors, and v is a scalar.
// - 0 < v < HALF_DIGIT_RADIX. Below, we rename w to q.
void
vec_div_small(int ulen, const sc_digit *u, sc_digit v, sc_digit *q)
{
    // Given (u = u_1u_2...u_n)_b = (q = q_1q_2...q_n) * v + r, where b
    // is the base, and 0 <= r < v. Then, the algorithm is as follows:
    //
    // r = 0;
    // for (j = 1; j <= n; j++) {
    //   q_j = (r * b + u_j) / v;
    //   r = (r * b + u_j) % v;
    // }
    //
    // In our case, b = DIGIT_RADIX, and u = Ax + B and q = Cx + D where
    // x = HALF_DIGIT_RADIX. Note that r < v < x and b = x^2. Then, a
    // typical situation is as follows:
    //
    // ---- ----
    // 0    r
    //           ---- ----
    //           A    B
    //      ---- ---- ----
    //      r    A    B     = r * b + u
    //
    // Hence, C = (r|A) / v.
    //        D = (((r|A) % v)|B) / v
    //        r = (((r|A) % v)|B) % v

#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert(q != NULL);
    sc_assert((0 < v) && (v < HALF_DIGIT_RADIX));
#endif

#define q_h r
    sc_digit r = 0;
    const sc_digit *ubegin = u;

    u += ulen;
    q += ulen;

    while (ubegin < u) {
        sc_digit u_AB = (*--u); // A|B

#ifdef DEBUG_SYSTEMC
        // The overflow bits must be zero.
        sc_assert(high_half(u_AB) == high_half_masked(u_AB));
#endif

        sc_digit num = concat(r, high_half(u_AB)); // num = r|A
        q_h = num / v; // C
        num = concat((num % v), low_half(u_AB)); // num = (((r|A) % v)|B)
        (*--q) = concat(q_h, num / v); // q = C|D
        r = num % v;
    }
#undef q_h
}

// Compute w = u % v, where w, u, and v are vectors.
// - u and v are assumed to have at least two digits as uchars.
void
vec_rem_large(int ulen, const sc_digit *u, int vlen, const sc_digit *v,
              sc_digit *w)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((vlen > 0) && (v != NULL));
    sc_assert(w != NULL);
    sc_assert(BITS_PER_DIGIT >= 3 * BITS_PER_BYTE);
#endif

    // This function is adapted from vec_div_large.
    int xlen = BYTES_PER_DIGIT * ulen + 1;
    int ylen = BYTES_PER_DIGIT * vlen;

#ifdef SC_MAX_NBITS
    uchar x[DIV_CEIL2(SC_MAX_NBITS, BITS_PER_BYTE)];
    uchar y[DIV_CEIL2(SC_MAX_NBITS, BITS_PER_BYTE)];
#else
    uchar *x = new uchar[xlen];
    uchar *y = new uchar[ylen];
#endif

    // r corresponds to w.

    // Set (uchar) x = (sc_digit) u.
    xlen = vec_to_char(ulen, u, xlen, x);

    // Skip all the leading zeros in x.
    while ((--xlen >= 0) && (!x[xlen]))
        continue;
    xlen++;

    // Set (uchar) y = (sc_digit) v.
    ylen = vec_to_char(vlen, v, ylen, y);

    // Skip all the leading zeros in y.
    while ((--ylen >= 0) && (!y[ylen]))
        continue;
    ylen++;

#ifdef DEBUG_SYSTEMC
    sc_assert(xlen > 1);
    sc_assert(ylen > 1);
#endif

    // At this point, all the leading zeros are eliminated from x and y.

    // Zero the last digit of x.
    x[xlen] = 0;

    // The first two digits of y.
    sc_digit y2 = (y[ylen - 1] << BITS_PER_BYTE) + y[ylen - 2];

    const sc_digit DOUBLE_BITS_PER_BYTE = 2 * BITS_PER_BYTE;

    // Find each q[k].
    for (int k = xlen - ylen; k >= 0; --k) {
        // qk is a guess for q[k] such that q[k] = qk or qk - 1.
        sc_digit qk;

        // Find qk by just using 2 digits of y and 3 digits of x. The
        // following code assumes that sizeof(sc_digit) >= 3 BYTEs.
        int k2 = k + ylen;

        qk = ((x[k2] << DOUBLE_BITS_PER_BYTE) +
            (x[k2 - 1] << BITS_PER_BYTE) + x[k2 - 2]) / y2;

        if (qk >= BYTE_RADIX) // qk cannot be larger than the largest
            qk = BYTE_RADIX - 1; // digit in BYTE_RADIX.

        // q[k] = qk or qk - 1. The following if-statement determines which.
        if (qk) {
            uchar *xk = (x + k);  // A shortcut for x[k].

            // x = x - y * qk;
            sc_digit carry = 0;

            for (int i = 0; i < ylen; ++i) {
                carry += y[i] * qk;
                sc_digit diff = (xk[i] + BYTE_RADIX) - (carry & BYTE_MASK);
                xk[i] = (uchar)(diff & BYTE_MASK);
                carry = (carry >> BITS_PER_BYTE) +
                    (1 - (diff >> BITS_PER_BYTE));
            }

            if (carry) {
                // 2's complement the last digit.
                carry = (xk[ylen] + BYTE_RADIX) - carry;
                xk[ylen] = (uchar)(carry & BYTE_MASK);
                carry = 1 - (carry >> BITS_PER_BYTE);

                if (carry) {
                  // qk was one too large, so decrement it.
                  // --qk;

                  // x = x - y * (qk - 1) = x - y * qk + y = x_above + y.
                  carry = 0;

                  for (int i = 0; i < ylen; ++i) {
                      carry += xk[i] + y[i];
                      xk[i] = (uchar)(carry & BYTE_MASK);
                      carry >>= BITS_PER_BYTE;
                  }

                  if (carry)
                      xk[ylen] = (uchar)((xk[ylen] + 1) & BYTE_MASK);
                }  // second if carry
            } // first if carry
        }  // if qk
    }  // for k

    // Set (sc_digit) w = (uchar) x for the remainder.
    vec_from_char(ylen, x, ulen, w);

#ifndef SC_MAX_NBITS
    delete [] x;
    delete [] y;
#endif

}

// Compute r = u % v, where u is a vector, and r and v are scalars.
// - 0 < v < HALF_DIGIT_RADIX.
// - The remainder r is returned.
sc_digit
vec_rem_small(int ulen, const sc_digit *u, sc_digit v)
{

#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((0 < v) && (v < HALF_DIGIT_RADIX));
#endif

    // This function is adapted from vec_div_small().

    sc_digit r = 0;
    const sc_digit *ubegin = u;

    u += ulen;

    while (ubegin < u) {
        sc_digit u_AB = (*--u); // A|B
#ifdef DEBUG_SYSTEMC
        // The overflow bits must be zero.
        sc_assert(high_half(u_AB) == high_half_masked(u_AB));
#endif
        // r = (((r|A) % v)|B) % v
        r = (concat(((concat(r, high_half(u_AB))) % v), low_half(u_AB))) % v;
    }

    return r;
}

// u = u / v, r = u % v.
sc_digit
vec_rem_on_small(int ulen, sc_digit *u, sc_digit v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert(v > 0);
#endif

#define q_h r
    sc_digit r = 0;
    const sc_digit *ubegin = u;

    u += ulen;
    while (ubegin < u) {
        sc_digit u_AB = (*--u); // A|B
#ifdef DEBUG_SYSTEMC
        // The overflow bits must be zero.
        sc_assert(high_half(u_AB) == high_half_masked(u_AB));
#endif
        sc_digit num = concat(r, high_half(u_AB)); // num = r|A
        q_h = num / v; // C
        num = concat((num % v), low_half(u_AB)); // num = (((r|A) % v)|B)
        (*u) = concat(q_h, num / v); // q = C|D
        r = num % v;
    }
#undef q_h
    return r;
}

// Set (uchar) v = (sc_digit) u. Return the new vlen.
int
vec_to_char(int ulen, const sc_digit *u, int vlen, uchar *v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((vlen > 0) && (v != NULL));
#endif

    int nbits = ulen * BITS_PER_DIGIT;
    int right = 0;
    int left = right + BITS_PER_BYTE - 1;

    vlen = 0;
    while (nbits > 0) {
        int left_digit = left / BITS_PER_DIGIT;
        int right_digit = right / BITS_PER_DIGIT;
        int nsr = ((vlen << LOG2_BITS_PER_BYTE) % BITS_PER_DIGIT);
        int d = u[right_digit] >> nsr;

        if (left_digit != right_digit) {
            if (left_digit < ulen)
                d |= u[left_digit] << (BITS_PER_DIGIT - nsr);
        }

        v[vlen++] = (uchar)(d & BYTE_MASK);

        left += BITS_PER_BYTE;
        right += BITS_PER_BYTE;
        nbits -= BITS_PER_BYTE;
    }
    return vlen;
}

// Set (sc_digit) v = (uchar) u.
// - sizeof(uchar) <= sizeof(sc_digit),
void
vec_from_char(int ulen, const uchar *u, int vlen, sc_digit *v)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
    sc_assert((vlen > 0) && (v != NULL));
    sc_assert(sizeof(uchar) <= sizeof(sc_digit));
#endif

    sc_digit *vend = (v + vlen);

    const int nsr = BITS_PER_DIGIT - BITS_PER_BYTE;
    const sc_digit mask = one_and_ones(nsr);

    (*v) = (sc_digit) u[ulen - 1];

    for (int i = ulen - 2; i >= 0; --i) {
        // Manual inlining of vec_shift_left().
        sc_digit *viter = v;
        sc_digit carry = 0;
        while (viter < vend) {
            sc_digit vval = (*viter);
            (*viter++) = (((vval & mask) << BITS_PER_BYTE) | carry);
            carry = vval >> nsr;
        }

        if (viter < vend)
            (*viter) = carry;

        (*v) |= (sc_digit)u[i];
    }
}

// Set u <<= nsl.
// If nsl is negative, it is ignored.
void
vec_shift_left(int ulen, sc_digit *u, int nsl)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
#endif

    if (nsl <= 0)
        return;

    // Shift left whole digits if nsl is large enough.
    if (nsl >= (int) BITS_PER_DIGIT) {
        int nd;
        if (nsl % BITS_PER_DIGIT == 0) {
            nd = nsl / BITS_PER_DIGIT; // No need to use DIV_CEIL(nsl).
            nsl = 0;
        } else {
            nd = DIV_CEIL(nsl) - 1;
            nsl -= nd * BITS_PER_DIGIT;
        }

        if (nd) {
            // Shift left for nd digits.
            for (int j = ulen - 1; j >= nd; --j)
                u[j] = u[j - nd];

            vec_zero(sc_min(nd, ulen), u);
        }
        if (nsl == 0)
            return;
    }

    // Shift left if nsl < BITS_PER_DIGIT.
    sc_digit *uiter = u;
    sc_digit *uend = uiter + ulen;

    int nsr = BITS_PER_DIGIT - nsl;
    sc_digit mask = one_and_ones(nsr);

    sc_digit carry = 0;

    while (uiter < uend) {
        sc_digit uval = (*uiter);
        (*uiter++) = (((uval & mask) << nsl) | carry);
        carry = uval >> nsr;
    }

    if (uiter < uend)
        (*uiter) = carry;
}

// Set u >>= nsr.
// If nsr is negative, it is ignored.
void
vec_shift_right(int ulen, sc_digit *u, int nsr, sc_digit fill)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((ulen > 0) && (u != NULL));
#endif

    // fill is usually either 0 or DIGIT_MASK; it can be any value.
    if (nsr <= 0)
        return;

    // Shift right whole digits if nsr is large enough.
    if (nsr >= (int) BITS_PER_DIGIT) {
        int nd;
        if (nsr % BITS_PER_DIGIT == 0) {
            nd = nsr / BITS_PER_DIGIT;
            nsr = 0;
        } else {
            nd = DIV_CEIL(nsr) - 1;
            nsr -= nd * BITS_PER_DIGIT;
        }

        if (nd) {
            // Shift right for nd digits.
            for (int j = 0; j < (ulen - nd); ++j)
                u[j] = u[j + nd];

            if (fill) {
                for (int j = ulen - sc_min( nd, ulen ); j < ulen; ++j)
                    u[j] = fill;
            } else {
                vec_zero(ulen - sc_min( nd, ulen ), ulen, u);
            }
        }
        if (nsr == 0)
          return;
    }

    // Shift right if nsr < BITS_PER_DIGIT.
    sc_digit *ubegin = u;
    sc_digit *uiter = (ubegin + ulen);

    int nsl = BITS_PER_DIGIT - nsr;
    sc_digit mask = one_and_ones(nsr);

    sc_digit carry = (fill & mask) << nsl;

    while (ubegin < uiter) {
        sc_digit uval = (*--uiter);
        (*uiter) = (uval >> nsr) | carry;
        carry = (uval & mask) << nsl;
    }
}


// Let u[l..r], where l and r are left and right bit positions
// respectively, be equal to its mirror image.
void
vec_reverse(int unb, int und, sc_digit *ud, int l, int r)
{
#ifdef DEBUG_SYSTEMC
    sc_assert((unb > 0) && (und > 0) && (ud != NULL));
    sc_assert((0 <= r) && (r <= l) && (l < unb));
#endif

    if (l < r) {
        std::stringstream msg;
        msg << "vec_reverse( int, int, sc_digit*, int l, int r ) : " <<
               "l = " << l << " < r = " << r << " is not valid",
        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_, msg.str().c_str());
        return;
    }

    // Make sure that l and r are within bounds.
    r = sc_max(r, 0);
    l = sc_min(l, unb - 1);

    // Allocate memory for processing.
#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS];
#else
    sc_digit *d = new sc_digit[und];
#endif

    // d is a copy of ud.
    vec_copy(und, d, ud);

    // Based on the value of the ith in d, find the value of the jth bit
    // in ud.
    for (int i = l, j = r; i >= r; --i, ++j) {
        if ((d[digit_ord(i)] & one_and_zeros(bit_ord(i))) != 0) // Test.
            ud[digit_ord(j)] |= one_and_zeros(bit_ord(j)); // Set.
        else
            ud[digit_ord(j)] &= ~(one_and_zeros(bit_ord(j))); // Clear.
    }

#ifndef SC_MAX_NBITS
    delete [] d;
#endif
}

#ifdef SC_MAX_NBITS
void test_bound_failed(int nb)
{
    std::stringstream msg;
    msg << "test_bound( int nb ) : "
           "nb = " << nb << " > SC_MAX_NBITS = " << SC_MAX_NBITS <<
           "  is not valid";
    SC_REPORT_ERROR(sc_core::SC_ID_OUT_OF_BOUNDS_, msg.str().c_str());
}
#endif // SC_MAX_NBITS

} // namespace sc_dt

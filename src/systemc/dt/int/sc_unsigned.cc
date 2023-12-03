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

  sc_unsigned.cpp -- Arbitrary precision signed arithmetic.

    This file includes the definitions of sc_unsigned_bitref,
    sc_unsigned_subref, and sc_unsigned classes. The first two classes
    are proxy classes to reference one bit and a range of bits of a
    sc_unsigned number, respectively. This file also includes
    sc_nbcommon.cpp and sc_nbfriends.cpp, which contain the
    definitions shared by sc_unsigned.

  Original Author: Ali Dasdan, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_unsigned.cpp,v $
// Revision 1.7  2011/02/18 20:19:15  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.6  2008/12/10 20:38:45  acg
//  Andy Goodrich: fixed conversion of double values to the digits vector.
//  The bits above the radix were not being masked off.
//
// Revision 1.5  2008/06/19 17:47:57  acg
//  Andy Goodrich: fixes for bugs. See 2.2.1 RELEASENOTES.
//
// Revision 1.4  2008/06/19 16:57:57  acg
//  Andy Goodrich: added case for negative unsigned values to the support in
//  concate_get_data().
//
// Revision 1.3  2007/11/04 21:27:00  acg
//  Andy Goodrich: changes to make sure the proper value is returned from
//  concat_get_data().
//
// Revision 1.2  2007/02/22 21:35:05  acg
//  Andy Goodrich: cleaned up comments in concat_get_ctrl and concat_get_data.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.4  2006/08/29 23:36:54  acg
//  Andy Goodrich: fixed and_reduce and optimized or_reduce.
//
// Revision 1.3  2006/01/13 18:49:32  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#include <cctype>
#include <cmath>
#include <sstream>

#include "systemc/ext/dt/bit/sc_bv_base.hh"
#include "systemc/ext/dt/bit/sc_lv_base.hh"
#include "systemc/ext/dt/fx/sc_ufix.hh"
#include "systemc/ext/dt/fx/scfx_other_defs.hh"
#include "systemc/ext/dt/int/messages.hh"
#include "systemc/ext/dt/int/sc_int_base.hh"
#include "systemc/ext/dt/int/sc_signed.hh"
#include "systemc/ext/dt/int/sc_uint_base.hh"
#include "systemc/ext/dt/int/sc_unsigned.hh"
#include "systemc/ext/dt/misc/sc_concatref.hh"
#include "systemc/ext/utils/messages.hh"

// explicit template instantiations
namespace sc_core
{

template class sc_vpool<sc_dt::sc_unsigned_bitref>;
template class sc_vpool<sc_dt::sc_unsigned_subref>;
template class sc_vpool<sc_dt::sc_unsigned>;

} // namespace sc_core

namespace sc_dt
{

// Pool of temporary instances:
//   The sc_unsigned pool is used by the concatenation support.
//   The bit and part reference pools allow references to be returned.

sc_core::sc_vpool<sc_unsigned> sc_unsigned::m_pool(8);
sc_core::sc_vpool<sc_unsigned_bitref> sc_unsigned_bitref::m_pool(9);
sc_core::sc_vpool<sc_unsigned_subref> sc_unsigned_subref::m_pool(9);

void
sc_unsigned::invalid_init(const char *type_name, int nb) const
{
    std::stringstream msg;
    msg << "sc_unsigned(" << type_name << ") : nb = " << nb << " is not valid";
    SC_REPORT_ERROR(sc_core::SC_ID_INIT_FAILED_, msg.str().c_str());
}

// ----------------------------------------------------------------------------
// SECTION: Public members - Invalid selections.
// ----------------------------------------------------------------------------

void
sc_unsigned::invalid_index(int i) const
{
    std::stringstream msg;
    msg << "sc_biguint bit selection: index = " << i
        << " violates "
           "0 <= index <= "
        << (nbits - 2);
    SC_REPORT_ERROR(sc_core::SC_ID_OUT_OF_BOUNDS_, msg.str().c_str());
    sc_core::sc_abort(); // can't recover from here
}

void
sc_unsigned::invalid_range(int l, int r) const
{
    std::stringstream msg;
    msg << "sc_biguint part selection: left = " << l << ", right = " << r
        << " \n"
           "  violates either ("
        << (nbits - 2)
        << " >= left >= 0) or "
           "("
        << (nbits - 2) << " >= right >= 0)";
    SC_REPORT_ERROR(sc_core::SC_ID_OUT_OF_BOUNDS_, msg.str().c_str());
    sc_core::sc_abort(); // can't recover from here
}

// ----------------------------------------------------------------------------
//  SECTION: Public members - Concatenation support.
// ----------------------------------------------------------------------------

// Most public members are included from sc_nbcommon.inc. However, some
// concatenation support appears here to optimize between the signed and
// unsigned cases.

// Insert this object's value at the specified place in a vector of big style
// values.

bool
sc_unsigned::concat_get_ctrl(sc_digit *dst_p, int low_i) const
{
    int dst_i;      // Index to next word to set in dst_p.
    int end_i;      // Index of high order word to set.
    int left_shift; // Amount to shift value left.
    sc_digit mask;  // Mask for partial word sets.

    // CALCULATE METRICS FOR DATA MOVEMENT:
    dst_i = low_i / BITS_PER_DIGIT;
    end_i = (low_i + nbits - 2) / BITS_PER_DIGIT;
    left_shift = low_i % BITS_PER_DIGIT;

    // MOVE FIRST WORD (IT MAY BE PARTIAL) AND THEN ANY OTHERS:
    //
    // We may "clobber" upper bits, but they will be written at some point
    // anyway.

    mask = ~(~0U << left_shift);
    dst_p[dst_i] = (dst_p[dst_i] & ~mask);
    dst_i++;

    for (; dst_i <= end_i; dst_i++)
        dst_p[dst_i] = 0;

    return false;
}

bool
sc_unsigned::concat_get_data(sc_digit *dst_p, int low_i) const
{
    sc_digit carry;      // Carry for negating value.
    int dst_i;           // Index to next word to set in dst_p.
    int end_i;           // Index of high order word to set.
    int high_i;          // Index w/in word of high order bit.
    int left_shift;      // Amount to shift value left.
    sc_digit left_word;  // High word component for set.
    sc_digit mask;       // Mask for partial word sets.
    bool result;         // True if inserting non-zero data.
    int right_shift;     // Amount to shift value right.
    sc_digit right_word; // Low word component for set.
    int real_bits;       // nbits - 1.
    int src_i;           // Index to next word to get from digit.

    // CALCULATE METRICS FOR DATA MOVEMENT:
    real_bits = nbits - 1; // Remove that extra sign bit.
    dst_i = low_i / BITS_PER_DIGIT;
    high_i = low_i + real_bits - 1;
    end_i = high_i / BITS_PER_DIGIT;
    left_shift = low_i % BITS_PER_DIGIT;

    switch (sgn) {
    // POSITIVE SOURCE VALUE:
    case SC_POS:
        result = true;

        // ALL DATA TO BE MOVED IS IN A SINGLE WORD:
        if (dst_i == end_i) {
            mask = ~(~0U << left_shift);
            dst_p[dst_i] = ((dst_p[dst_i] & mask) | (digit[0] << left_shift)) &
                           DIGIT_MASK;

            // DATA IS IN MORE THAN ONE WORD, BUT IS WORD ALIGNED:
        } else if (left_shift == 0) {
            for (src_i = 0; dst_i < end_i; dst_i++, src_i++) {
                dst_p[dst_i] = digit[src_i];
            }
            high_i = high_i % BITS_PER_DIGIT;
            mask = ~(~1U << high_i) & DIGIT_MASK;
            dst_p[dst_i] = digit[src_i] & mask;

            // DATA IS IN MORE THAN ONE WORD, AND NOT WORD ALIGNED:
        } else {
            high_i = high_i % BITS_PER_DIGIT;
            right_shift = BITS_PER_DIGIT - left_shift;
            mask = ~(~0U << left_shift);
            right_word = digit[0];
            dst_p[dst_i] = (dst_p[dst_i] & mask) |
                           ((right_word << left_shift) & DIGIT_MASK);
            for (src_i = 1, dst_i++; dst_i < end_i; dst_i++, src_i++) {
                left_word = digit[src_i];
                dst_p[dst_i] = ((left_word << left_shift) & DIGIT_MASK) |
                               (right_word >> right_shift);
                right_word = left_word;
            }
            left_word = (src_i < ndigits) ? digit[src_i] : 0;
            mask = ~(~1U << high_i) & DIGIT_MASK;
            dst_p[dst_i] =
                ((left_word << left_shift) | (right_word >> right_shift)) &
                mask;
        }
        break;

    // SOURCE VALUE IS NEGATIVE:
    case SC_NEG:
        // ALL DATA TO BE MOVED IS IN A SINGLE WORD:
        result = true;
        if (dst_i == end_i) {
            mask = ~(~0U << nbits);
            right_word = ((digit[0] ^ DIGIT_MASK) + 1) & mask;
            mask = ~(~0U << left_shift);
            dst_p[dst_i] =
                ((dst_p[dst_i] & mask) | (right_word << left_shift)) &
                DIGIT_MASK;

            // DATA IS IN MORE THAN ONE WORD, BUT IS WORD ALIGNED:

        } else if (left_shift == 0) {
            carry = 1;
            for (src_i = 0; dst_i < end_i; dst_i++, src_i++) {
                right_word = (digit[src_i] ^ DIGIT_MASK) + carry;
                dst_p[dst_i] = right_word & DIGIT_MASK;
                carry = right_word >> BITS_PER_DIGIT;
            }
            high_i = high_i % BITS_PER_DIGIT;
            mask = (~(~1U << high_i)) & DIGIT_MASK;
            right_word = (src_i < ndigits) ?
                             (digit[src_i] ^ DIGIT_MASK) + carry :
                             DIGIT_MASK + carry;
            dst_p[dst_i] = right_word & mask;

            // DATA IS IN MORE THAN ONE WORD, AND NOT WORD ALIGNED:
        } else {
            high_i = high_i % BITS_PER_DIGIT;
            right_shift = BITS_PER_DIGIT - left_shift;
            mask = ~(~0U << left_shift);
            carry = 1;
            right_word = (digit[0] ^ DIGIT_MASK) + carry;
            dst_p[dst_i] = (dst_p[dst_i] & mask) |
                           ((right_word << left_shift) & DIGIT_MASK);
            carry = right_word >> BITS_PER_DIGIT;
            right_word &= DIGIT_MASK;
            for (src_i = 1, dst_i++; dst_i < end_i; dst_i++, src_i++) {
                left_word = (digit[src_i] ^ DIGIT_MASK) + carry;
                dst_p[dst_i] = ((left_word << left_shift) & DIGIT_MASK) |
                               (right_word >> right_shift);
                carry = left_word >> BITS_PER_DIGIT;
                right_word = left_word & DIGIT_MASK;
            }
            left_word = (src_i < ndigits) ?
                            (digit[src_i] ^ DIGIT_MASK) + carry :
                            carry;
            mask = ~(~1U << high_i) & DIGIT_MASK;
            dst_p[dst_i] =
                ((left_word << left_shift) | (right_word >> right_shift)) &
                mask;
        }
        break;
    // VALUE IS ZERO:
    default:
        result = false;
        // ALL DATA TO BE MOVED IS IN A SINGLE WORD:
        if (dst_i == end_i) {
            mask = ~(~0U << real_bits) << left_shift;
            dst_p[dst_i] = dst_p[dst_i] & ~mask;

            // DATA IS IN MORE THAN ONE WORD, BUT IS WORD ALIGNED:

        } else if (left_shift == 0) {
            for (src_i = 0; dst_i < end_i; dst_i++, src_i++) {
                dst_p[dst_i] = 0;
            }
            dst_p[dst_i] = 0;

            // DATA IS IN MORE THAN ONE WORD, AND NOT WORD ALIGNED:
        } else {
            mask = ~(~0U << left_shift);
            dst_p[dst_i] = (dst_p[dst_i] & mask);
            for (dst_i++; dst_i <= end_i; dst_i++) {
                dst_p[dst_i] = 0;
            }
        }
        break;
    }
    return result;
}

// Return this object instance's bits as a uint64 without sign extension.
uint64
sc_unsigned::concat_get_uint64() const
{
    uint64 result;

    switch (sgn) {
    case SC_POS:
        result = 0;
        if (ndigits > 2)
            result = digit[2];
        if (ndigits > 1)
            result = (result << BITS_PER_DIGIT) | digit[1];
        result = (result << BITS_PER_DIGIT) | digit[0];
        break;
    default:
        result = 0;
        break;
    }
    return result;
}

// #### OPTIMIZE
void
sc_unsigned::concat_set(int64 src, int low_i)
{
    *this = (low_i < 64) ? src >> low_i : src >> 63;
}

void
sc_unsigned::concat_set(const sc_signed &src, int low_i)
{
    if (low_i < src.length())
        *this = src >> low_i;
    else
        *this = (src < 0) ? (int_type)-1 : 0;
}

void
sc_unsigned::concat_set(const sc_unsigned &src, int low_i)
{
    if (low_i < src.length())
        *this = src >> low_i;
    else
        *this = 0;
}

void
sc_unsigned::concat_set(uint64 src, int low_i)
{
    *this = (low_i < 64) ? src >> low_i : 0;
}

// ----------------------------------------------------------------------------
//  SECTION: Public members - Reduction methods.
// ----------------------------------------------------------------------------

bool
sc_unsigned::and_reduce() const
{
    int i; // Digit examining.

    if (sgn == SC_ZERO)
        return false;
    for (i = 0; i < ndigits - 1; i++)
        if ((digit[i] & DIGIT_MASK) != DIGIT_MASK)
            return false;
    if ((digit[i] & ~(~0U << ((nbits - 1) % BITS_PER_DIGIT))) ==
        static_cast<sc_digit>(~(~0U << ((nbits - 1) % BITS_PER_DIGIT)))) {
        return true;
    }
    return false;
}

bool
sc_unsigned::or_reduce() const
{
    return (sgn == SC_ZERO) ? false : true;
}

bool
sc_unsigned::xor_reduce() const
{
    int i;   // Digit examining.
    int odd; // Flag for odd number of digits.

    odd = 0;
    for (i = 0; i < nbits - 1; i++)
        if (test(i))
            odd = ~odd;
    return odd ? true : false;
}

// ----------------------------------------------------------------------------
//  SECTION: Public members - Assignment operators.
// ----------------------------------------------------------------------------

// assignment operators
const sc_unsigned &
sc_unsigned::operator=(const char *a)
{
    if (a == 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                        "character string is zero");
    } else if (*a == 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                        "character string is empty");
    } else
        try {
            int len = length();
            sc_ufix aa(a, len, len, SC_TRN, SC_WRAP, 0, SC_ON);
            return this->operator=(aa);
        } catch (const sc_core::sc_report &) {
            std::stringstream msg;
            msg << "character string '" << a << "' is not valid";
            SC_REPORT_ERROR(sc_core::SC_ID_CONVERSION_FAILED_,
                            msg.str().c_str());
        }
    return *this;
}

const sc_unsigned &
sc_unsigned::operator=(int64 v)
{
    sgn = get_sign(v);
    if (sgn == SC_ZERO) {
        vec_zero(ndigits, digit);
    } else {
        from_uint(ndigits, digit, (uint64)v);
        convert_SM_to_2C_to_SM();
    }
    return *this;
}

const sc_unsigned &
sc_unsigned::operator=(uint64 v)
{
    if (v == 0) {
        sgn = SC_ZERO;
        vec_zero(ndigits, digit);
    } else {
        sgn = SC_POS;
        from_uint(ndigits, digit, v);
        convert_SM_to_2C_to_SM();
    }
    return *this;
}

const sc_unsigned &
sc_unsigned::operator=(long v)
{
    sgn = get_sign(v);
    if (sgn == SC_ZERO) {
        vec_zero(ndigits, digit);
    } else {
        from_uint(ndigits, digit, (unsigned long)v);
        convert_SM_to_2C_to_SM();
    }
    return *this;
}

const sc_unsigned &
sc_unsigned::operator=(unsigned long v)
{
    if (v == 0) {
        sgn = SC_ZERO;
        vec_zero(ndigits, digit);
    } else {
        sgn = SC_POS;
        from_uint(ndigits, digit, v);
        convert_SM_to_2C_to_SM();
    }
    return *this;
}

const sc_unsigned &
sc_unsigned::operator=(double v)
{
    is_bad_double(v);
    sgn = SC_POS;
    int i = 0;
    while (std::floor(v) && (i < ndigits)) {
        digit[i++] =
            ((sc_digit)std::floor(remainder(v, DIGIT_RADIX))) & DIGIT_MASK;
        v /= DIGIT_RADIX;
    }
    vec_zero(i, ndigits, digit);
    convert_SM_to_2C_to_SM();
    return *this;
}

// ----------------------------------------------------------------------------

const sc_unsigned &
sc_unsigned::operator=(const sc_bv_base &v)
{
    int minlen = sc_min(nbits, v.length());
    int i = 0;
    for (; i < minlen; ++i) {
        safe_set(i, v.get_bit(i), digit);
    }
    for (; i < nbits; ++i) {
        safe_set(i, 0, digit); // zero-extend
    }
    convert_2C_to_SM();
    return *this;
}

const sc_unsigned &
sc_unsigned::operator=(const sc_lv_base &v)
{
    int minlen = sc_min(nbits, v.length());
    int i = 0;
    for (; i < minlen; ++i) {
        safe_set(i, sc_logic(v.get_bit(i)).to_bool(), digit);
    }
    for (; i < nbits; ++i) {
        safe_set(i, 0, digit); // zero-extend
    }
    convert_2C_to_SM();
    return *this;
}

// explicit conversion to character string
const std::string
sc_unsigned::to_string(sc_numrep numrep) const
{
    int len = length();
    sc_ufix aa(*this, len, len, SC_TRN, SC_WRAP, 0, SC_ON);
    return aa.to_string(numrep);
}

const std::string
sc_unsigned::to_string(sc_numrep numrep, bool w_prefix) const
{
    int len = length();
    sc_ufix aa(*this, len, len, SC_TRN, SC_WRAP, 0, SC_ON);
    return aa.to_string(numrep, w_prefix);
}

// ----------------------------------------------------------------------------
//  SECTION: Interfacing with sc_int_base
// ----------------------------------------------------------------------------

const sc_unsigned &
sc_unsigned::operator=(const sc_int_base &v)
{
    return operator=((int64)v);
}

const sc_unsigned &
sc_unsigned::operator+=(const sc_int_base &v)
{
    return operator+=((int64)v);
}

const sc_unsigned &
sc_unsigned::operator-=(const sc_int_base &v)
{
    return operator-=((int64)v);
}

const sc_unsigned &
sc_unsigned::operator*=(const sc_int_base &v)
{
    return operator*=((int64)v);
}

const sc_unsigned &
sc_unsigned::operator/=(const sc_int_base &v)
{
    return operator/=((int64)v);
}

const sc_unsigned &
sc_unsigned::operator%=(const sc_int_base &v)
{
    return operator%=((int64)v);
}

const sc_unsigned &
sc_unsigned::operator&=(const sc_int_base &v)
{
    return operator&=((int64)v);
}

const sc_unsigned &
sc_unsigned::operator|=(const sc_int_base &v)
{
    return operator|=((int64)v);
}

const sc_unsigned &
sc_unsigned::operator^=(const sc_int_base &v)
{
    return operator^=((int64)v);
}

sc_unsigned
operator<<(const sc_unsigned &u, const sc_int_base &v)
{
    return operator<<(u, (int64)v);
}

const sc_unsigned &
sc_unsigned::operator<<=(const sc_int_base &v)
{
    return operator<<=((int64)v);
}

sc_unsigned
operator>>(const sc_unsigned &u, const sc_int_base &v)
{
    return operator>>(u, (int64)v);
}

const sc_unsigned &
sc_unsigned::operator>>=(const sc_int_base &v)
{
    return operator>>=((int64)v);
}

bool
operator==(const sc_unsigned &u, const sc_int_base &v)
{
    return operator==(u, (int64)v);
}

bool
operator==(const sc_int_base &u, const sc_unsigned &v)
{
    return operator==((int64)u, v);
}

bool
operator!=(const sc_unsigned &u, const sc_int_base &v)
{
    return operator!=(u, (int64)v);
}

bool
operator!=(const sc_int_base &u, const sc_unsigned &v)
{
    return operator!=((int64)u, v);
}

bool
operator<(const sc_unsigned &u, const sc_int_base &v)
{
    return operator<(u, (int64)v);
}

bool
operator<(const sc_int_base &u, const sc_unsigned &v)
{
    return operator<((int64)u, v);
}

bool
operator<=(const sc_unsigned &u, const sc_int_base &v)
{
    return operator<=(u, (int64)v);
}

bool
operator<=(const sc_int_base &u, const sc_unsigned &v)
{
    return operator<=((int64)u, v);
}

bool
operator>(const sc_unsigned &u, const sc_int_base &v)
{
    return operator>(u, (int64)v);
}

bool
operator>(const sc_int_base &u, const sc_unsigned &v)
{
    return operator>((int64)u, v);
}

bool
operator>=(const sc_unsigned &u, const sc_int_base &v)
{
    return operator>=(u, (int64)v);
}

bool
operator>=(const sc_int_base &u, const sc_unsigned &v)
{
    return operator>=((int64)u, v);
}

// ----------------------------------------------------------------------------
//  SECTION: Interfacing with sc_uint_base
// ----------------------------------------------------------------------------

const sc_unsigned &
sc_unsigned::operator=(const sc_uint_base &v)
{
    return operator=((uint64)v);
}

sc_unsigned
operator+(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator+(u, (uint64)v);
}

sc_unsigned
operator+(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator+((uint64)u, v);
}

const sc_unsigned &
sc_unsigned::operator+=(const sc_uint_base &v)
{
    return operator+=((uint64)v);
}

const sc_unsigned &
sc_unsigned::operator-=(const sc_uint_base &v)
{
    return operator-=((uint64)v);
}

sc_unsigned
operator*(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator*(u, (uint64)v);
}

sc_unsigned
operator*(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator*((uint64)u, v);
}

const sc_unsigned &
sc_unsigned::operator*=(const sc_uint_base &v)
{
    return operator*=((uint64)v);
}

sc_unsigned
operator/(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator/(u, (uint64)v);
}

sc_unsigned
operator/(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator/((uint64)u, v);
}

const sc_unsigned &
sc_unsigned::operator/=(const sc_uint_base &v)
{
    return operator/=((uint64)v);
}

sc_unsigned
operator%(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator%(u, (uint64)v);
}

sc_unsigned
operator%(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator%((uint64)u, v);
}

const sc_unsigned &
sc_unsigned::operator%=(const sc_uint_base &v)
{
    return operator%=((uint64)v);
}

sc_unsigned
operator&(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator&(u, (uint64)v);
}

sc_unsigned
operator&(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator&((uint64)u, v);
}

const sc_unsigned &
sc_unsigned::operator&=(const sc_uint_base &v)
{
    return operator&=((uint64)v);
}

sc_unsigned
operator|(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator|(u, (uint64)v);
}

sc_unsigned
operator|(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator|((uint64)u, v);
}

const sc_unsigned &
sc_unsigned::operator|=(const sc_uint_base &v)
{
    return operator|=((uint64)v);
}

sc_unsigned
operator^(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator^(u, (uint64)v);
}

sc_unsigned
operator^(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator^((uint64)u, v);
}

const sc_unsigned &
sc_unsigned::operator^=(const sc_uint_base &v)
{
    return operator^=((uint64)v);
}

sc_unsigned
operator<<(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator<<(u, (uint64)v);
}

const sc_unsigned &
sc_unsigned::operator<<=(const sc_uint_base &v)
{
    return operator<<=((uint64)v);
}

sc_unsigned
operator>>(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator>>(u, (uint64)v);
}

const sc_unsigned &
sc_unsigned::operator>>=(const sc_uint_base &v)
{
    return operator>>=((uint64)v);
}

bool
operator==(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator==(u, (uint64)v);
}

bool
operator==(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator==((uint64)u, v);
}

bool
operator!=(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator!=(u, (uint64)v);
}

bool
operator!=(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator!=((uint64)u, v);
}

bool
operator<(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator<(u, (uint64)v);
}

bool
operator<(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator<((uint64)u, v);
}

bool
operator<=(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator<=(u, (uint64)v);
}

bool
operator<=(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator<=((uint64)u, v);
}

bool
operator>(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator>(u, (uint64)v);
}

bool
operator>(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator>((uint64)u, v);
}

bool
operator>=(const sc_unsigned &u, const sc_uint_base &v)
{
    return operator>=(u, (uint64)v);
}

bool
operator>=(const sc_uint_base &u, const sc_unsigned &v)
{
    return operator>=((uint64)u, v);
}

// ----------------------------------------------------------------------------
//  SECTION: Input and output operators
// ----------------------------------------------------------------------------

// The operators in this section are included from sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: Operator macros.
// ----------------------------------------------------------------------------

#define CONVERT_LONG(u)                                                       \
    small_type u##s = get_sign(u);                                            \
    sc_digit u##d[DIGITS_PER_ULONG];                                          \
    from_uint(DIGITS_PER_ULONG, u##d, (unsigned long)u);

#define CONVERT_LONG_2(u)                                                     \
    sc_digit u##d[DIGITS_PER_ULONG];                                          \
    from_uint(DIGITS_PER_ULONG, u##d, (unsigned long)u);

#define CONVERT_INT(u)                                                        \
    small_type u##s = get_sign(u);                                            \
    sc_digit u##d[DIGITS_PER_UINT];                                           \
    from_uint(DIGITS_PER_UINT, u##d, (unsigned int)u);

#define CONVERT_INT_2(u)                                                      \
    sc_digit u##d[DIGITS_PER_UINT];                                           \
    from_uint(DIGITS_PER_UINT, u##d, (unsigned int)u);

#define CONVERT_INT64(u)                                                      \
    small_type u##s = get_sign(u);                                            \
    sc_digit u##d[DIGITS_PER_UINT64];                                         \
    from_uint(DIGITS_PER_UINT64, u##d, (uint64)u);

#define CONVERT_INT64_2(u)                                                    \
    sc_digit u##d[DIGITS_PER_UINT64];                                         \
    from_uint(DIGITS_PER_UINT64, u##d, (uint64)u);

// ----------------------------------------------------------------------------
//  SECTION: PLUS operators: +, +=, ++
// ----------------------------------------------------------------------------

// Cases to consider when computing u + v:
// 1. 0 + v = v
// 2. u + 0 = u
// 3. if sgn(u) == sgn(v)
//    3.1 u + v = +(u + v) = sgn(u) * (u + v)
//    3.2 (-u) + (-v) = -(u + v) = sgn(u) * (u + v)
// 4. if sgn(u) != sgn(v)
//    4.1 u + (-v) = u - v = sgn(u) * (u - v)
//    4.2 (-u) + v = -(u - v) ==> sgn(u) * (u - v)
//
// Specialization of above cases for computing ++u or u++:
// 1. 0 + 1 = 1
// 3. u + 1 = u + 1 = sgn(u) * (u + 1)
// 4. (-u) + 1 = -(u - 1) = sgn(u) * (u - 1)

sc_unsigned
operator+(const sc_unsigned &u, const sc_unsigned &v)
{
    if (u.sgn == SC_ZERO) // case 1
        return sc_unsigned(v);

    if (v.sgn == SC_ZERO) // case 2
        return sc_unsigned(u);

    // cases 3 and 4
    return add_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn,
                               v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator+(const sc_unsigned &u, uint64 v)
{
    if (v == 0) // case 2
        return sc_unsigned(u);

    CONVERT_INT64(v);

    if (u.sgn == SC_ZERO) // case 1
        return sc_unsigned(vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd, false);

    // cases 3 and 4
    return add_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                               BITS_PER_UINT64, DIGITS_PER_UINT64, vd);
}

sc_unsigned
operator+(uint64 u, const sc_unsigned &v)
{
    if (u == 0) // case 1
        return sc_unsigned(v);

    CONVERT_INT64(u);

    if (v.sgn == SC_ZERO) // case 2
        return sc_unsigned(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud, false);

    // cases 3 and 4
    return add_unsigned_friend(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud,
                               v.sgn, v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator+(const sc_unsigned &u, unsigned long v)
{
    if (v == 0) // case 2
        return sc_unsigned(u);

    CONVERT_LONG(v);

    if (u.sgn == SC_ZERO) // case 1
        return sc_unsigned(vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd, false);

    // cases 3 and 4
    return add_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                               BITS_PER_ULONG, DIGITS_PER_ULONG, vd);
}

sc_unsigned
operator+(unsigned long u, const sc_unsigned &v)
{
    if (u == 0) // case 1
        return sc_unsigned(v);

    CONVERT_LONG(u);

    if (v.sgn == SC_ZERO) // case 2
        return sc_unsigned(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, false);

    // cases 3 and 4
    return add_unsigned_friend(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, v.sgn,
                               v.nbits, v.ndigits, v.digit);
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: MINUS operators: -, -=, --
// ----------------------------------------------------------------------------

// Cases to consider when computing u + v:
// 1. u - 0 = u
// 2. 0 - v = -v
// 3. if sgn(u) != sgn(v)
//    3.1 u - (-v) = u + v = sgn(u) * (u + v)
//    3.2 (-u) - v = -(u + v) ==> sgn(u) * (u + v)
// 4. if sgn(u) == sgn(v)
//    4.1 u - v = +(u - v) = sgn(u) * (u - v)
//    4.2 (-u) - (-v) = -(u - v) = sgn(u) * (u - v)
//
// Specialization of above cases for computing --u or u--:
// 1. 0 - 1 = -1
// 3. (-u) - 1 = -(u + 1) = sgn(u) * (u + 1)
// 4. u - 1 = u - 1 = sgn(u) * (u - 1)

// The operators in this section are included from sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: MULTIPLICATION operators: *, *=
// ----------------------------------------------------------------------------

// Cases to consider when computing u * v:
// 1. u * 0 = 0 * v = 0
// 2. 1 * v = v and -1 * v = -v
// 3. u * 1 = u and u * -1 = -u
// 4. u * v = u * v

sc_unsigned
operator*(const sc_unsigned &u, const sc_unsigned &v)
{
    small_type s = mul_signs(u.sgn, v.sgn);

    if (s == SC_ZERO) // case 1
        return sc_unsigned();

    // cases 2-4
    return mul_unsigned_friend(s, u.nbits, u.ndigits, u.digit, v.nbits,
                               v.ndigits, v.digit);
}

sc_unsigned
operator*(const sc_unsigned &u, uint64 v)
{
    small_type s = mul_signs(u.sgn, get_sign(v));

    if (s == SC_ZERO) // case 1
        return sc_unsigned();

    CONVERT_INT64_2(v);

    // cases 2-4
    return mul_unsigned_friend(s, u.nbits, u.ndigits, u.digit, BITS_PER_UINT64,
                               DIGITS_PER_UINT64, vd);
}

sc_unsigned
operator*(uint64 u, const sc_unsigned &v)
{
    small_type s = mul_signs(v.sgn, get_sign(u));

    if (s == SC_ZERO) // case 1
        return sc_unsigned();

    CONVERT_INT64_2(u);

    // cases 2-4
    return mul_unsigned_friend(s, BITS_PER_UINT64, DIGITS_PER_UINT64, ud,
                               v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator*(const sc_unsigned &u, unsigned long v)
{
    small_type s = mul_signs(u.sgn, get_sign(v));

    if (s == SC_ZERO) // case 1
        return sc_unsigned();

    CONVERT_LONG_2(v);

    // else cases 2-4
    return mul_unsigned_friend(s, u.nbits, u.ndigits, u.digit, BITS_PER_ULONG,
                               DIGITS_PER_ULONG, vd);
}

sc_unsigned
operator*(unsigned long u, const sc_unsigned &v)
{
    small_type s = mul_signs(v.sgn, get_sign(u));

    if (s == SC_ZERO) // case 1
        return sc_unsigned();

    CONVERT_LONG_2(u);

    // cases 2-4
    return mul_unsigned_friend(s, BITS_PER_ULONG, DIGITS_PER_ULONG, ud,
                               v.nbits, v.ndigits, v.digit);
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: DIVISION operators: /, /=
// ----------------------------------------------------------------------------

// Cases to consider when finding the quotient q = floor(u/v):
// Note that u = q * v + r for r < q.
// 1. 0 / 0 or u / 0 => error
// 2. 0 / v => 0 = 0 * v + 0
// 3. u / v & &u = v => u = 1 * u + 0  - u or v can be 1 or -1
// 4. u / v & &u < v => u = 0 * v + u  - u can be 1 or -1
// 5. u / v & &u > v => u = q * v + r  - v can be 1 or -1

sc_unsigned
operator/(const sc_unsigned &u, const sc_unsigned &v)
{
    small_type s = mul_signs(u.sgn, v.sgn);

    if (s == SC_ZERO) {
        div_by_zero(v.sgn);   // case 1
        return sc_unsigned(); // case 2
    }

    // other cases
    return div_unsigned_friend(s, u.nbits, u.ndigits, u.digit, v.nbits,
                               v.ndigits, v.digit);
}

sc_unsigned
operator/(const sc_unsigned &u, uint64 v)
{
    small_type s = mul_signs(u.sgn, get_sign(v));

    if (s == SC_ZERO) {
        div_by_zero(v);       // case 1
        return sc_unsigned(); // case 2
    }

    CONVERT_INT64_2(v);

    // other cases
    return div_unsigned_friend(s, u.nbits, u.ndigits, u.digit, BITS_PER_UINT64,
                               DIGITS_PER_UINT64, vd);
}

sc_unsigned
operator/(uint64 u, const sc_unsigned &v)
{
    small_type s = mul_signs(v.sgn, get_sign(u));

    if (s == SC_ZERO) {
        div_by_zero(v.sgn);   // case 1
        return sc_unsigned(); // case 2
    }

    CONVERT_INT64_2(u);

    // other cases
    return div_unsigned_friend(s, BITS_PER_UINT64, DIGITS_PER_UINT64, ud,
                               v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator/(const sc_unsigned &u, unsigned long v)
{
    small_type s = mul_signs(u.sgn, get_sign(v));

    if (s == SC_ZERO) {
        div_by_zero(v);       // case 1
        return sc_unsigned(); // case 2
    }

    CONVERT_LONG_2(v);

    // other cases
    return div_unsigned_friend(s, u.nbits, u.ndigits, u.digit, BITS_PER_ULONG,
                               DIGITS_PER_ULONG, vd);
}

sc_unsigned
operator/(unsigned long u, const sc_unsigned &v)
{
    small_type s = mul_signs(v.sgn, get_sign(u));

    if (s == SC_ZERO) {
        div_by_zero(v.sgn);   // case 1
        return sc_unsigned(); // case 2
    }

    CONVERT_LONG_2(u);

    // other cases
    return div_unsigned_friend(s, BITS_PER_ULONG, DIGITS_PER_ULONG, ud,
                               v.nbits, v.ndigits, v.digit);
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: MOD operators: %, %=.
// ----------------------------------------------------------------------------

// Cases to consider when finding the remainder r = u % v:
// Note that u = q * v + r for r < q.
// 1. 0 % 0 or u % 0 => error
// 2. 0 % v => 0 = 0 * v + 0
// 3. u % v & &u = v => u = 1 * u + 0  - u or v can be 1 or -1
// 4. u % v & &u < v => u = 0 * v + u  - u can be 1 or -1
// 5. u % v & &u > v => u = q * v + r  - v can be 1 or -1

sc_unsigned
operator%(const sc_unsigned &u, const sc_unsigned &v)
{
    if ((u.sgn == SC_ZERO) || (v.sgn == SC_ZERO)) {
        div_by_zero(v.sgn);   // case 1
        return sc_unsigned(); // case 2
    }

    // other cases
    return mod_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, v.nbits,
                               v.ndigits, v.digit);
}

sc_unsigned
operator%(const sc_unsigned &u, uint64 v)
{
    if ((u.sgn == SC_ZERO) || (v == 0)) {
        div_by_zero(v);       // case 1
        return sc_unsigned(); // case 2
    }

    CONVERT_INT64_2(v);

    // other cases
    return mod_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit,
                               BITS_PER_UINT64, DIGITS_PER_UINT64, vd);
}

sc_unsigned
operator%(uint64 u, const sc_unsigned &v)
{
    if ((u == 0) || (v.sgn == SC_ZERO)) {
        div_by_zero(v.sgn);   // case 1
        return sc_unsigned(); // case 2
    }

    CONVERT_INT64(u);

    // other cases
    return mod_unsigned_friend(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud,
                               v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator%(const sc_unsigned &u, unsigned long v)
{
    if ((u.sgn == SC_ZERO) || (v == 0)) {
        div_by_zero(v);       // case 1
        return sc_unsigned(); // case 2
    }

    CONVERT_LONG_2(v);

    // other cases
    return mod_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit,
                               BITS_PER_ULONG, DIGITS_PER_ULONG, vd);
}

sc_unsigned
operator%(unsigned long u, const sc_unsigned &v)
{
    if ((u == 0) || (v.sgn == SC_ZERO)) {
        div_by_zero(v.sgn);   // case 1
        return sc_unsigned(); // case 2
    }

    CONVERT_LONG(u);

    // other cases
    return mod_unsigned_friend(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud,
                               v.nbits, v.ndigits, v.digit);
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: Bitwise AND operators: &, &=
// ----------------------------------------------------------------------------

// Cases to consider when computing u  &v:
// 1. u & 0 = 0  &v = 0
// 2. u  &v => sgn = +
// 3. (-u) & (-v) => sgn = -
// 4. u & (-v) => sgn = +
// 5. (-u)  &v => sgn = +

sc_unsigned
operator&(const sc_unsigned &u, const sc_unsigned &v)
{
    if ((u.sgn == SC_ZERO) || (v.sgn == SC_ZERO)) // case 1
        return sc_unsigned();

    // other cases
    return and_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn,
                               v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator&(const sc_unsigned &u, uint64 v)
{
    if ((u.sgn == SC_ZERO) || (v == 0)) // case 1
        return sc_unsigned();

    CONVERT_INT64(v);

    // other cases
    return and_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                               BITS_PER_UINT64, DIGITS_PER_UINT64, vd);
}

sc_unsigned
operator&(uint64 u, const sc_unsigned &v)
{
    if ((u == 0) || (v.sgn == SC_ZERO)) // case 1
        return sc_unsigned();

    CONVERT_INT64(u);

    // other cases
    return and_unsigned_friend(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud,
                               v.sgn, v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator&(const sc_unsigned &u, unsigned long v)
{
    if ((u.sgn == SC_ZERO) || (v == 0)) // case 1
        return sc_unsigned();

    CONVERT_LONG(v);

    // other cases
    return and_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                               BITS_PER_ULONG, DIGITS_PER_ULONG, vd);
}

sc_unsigned
operator&(unsigned long u, const sc_unsigned &v)
{
    if ((u == 0) || (v.sgn == SC_ZERO)) // case 1
        return sc_unsigned();

    CONVERT_LONG(u);

    // other cases
    return and_unsigned_friend(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, v.sgn,
                               v.nbits, v.ndigits, v.digit);
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: Bitwise OR operators: |, |=
// ----------------------------------------------------------------------------

// Cases to consider when computing u | v:
// 1. u | 0 = u
// 2. 0 | v = v
// 3. u | v => sgn = +
// 4. (-u) | (-v) => sgn = -
// 5. u | (-v) => sgn = -
// 6. (-u) | v => sgn = -

sc_unsigned
operator|(const sc_unsigned &u, const sc_unsigned &v)
{
    if (v.sgn == SC_ZERO) // case 1
        return sc_unsigned(u);

    if (u.sgn == SC_ZERO) // case 2
        return sc_unsigned(v);

    // other cases
    return or_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn,
                              v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator|(const sc_unsigned &u, uint64 v)
{
    if (v == 0) // case 1
        return sc_unsigned(u);

    CONVERT_INT64(v);

    if (u.sgn == SC_ZERO) // case 2
        return sc_unsigned(vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd, false);

    // other cases
    return or_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                              BITS_PER_UINT64, DIGITS_PER_UINT64, vd);
}

sc_unsigned
operator|(uint64 u, const sc_unsigned &v)
{
    if (u == 0)
        return sc_unsigned(v);

    CONVERT_INT64(u);

    if (v.sgn == SC_ZERO)
        return sc_unsigned(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud, false);

    // other cases
    return or_unsigned_friend(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud,
                              v.sgn, v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator|(const sc_unsigned &u, unsigned long v)
{
    if (v == 0) // case 1
        return sc_unsigned(u);

    CONVERT_LONG(v);

    if (u.sgn == SC_ZERO) // case 2
        return sc_unsigned(vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd, false);

    // other cases
    return or_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                              BITS_PER_ULONG, DIGITS_PER_ULONG, vd);
}

sc_unsigned
operator|(unsigned long u, const sc_unsigned &v)
{
    if (u == 0)
        return sc_unsigned(v);

    CONVERT_LONG(u);

    if (v.sgn == SC_ZERO)
        return sc_unsigned(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, false);

    // other cases
    return or_unsigned_friend(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, v.sgn,
                              v.nbits, v.ndigits, v.digit);
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: Bitwise XOR operators: ^, ^=
// ----------------------------------------------------------------------------

// Cases to consider when computing u ^ v:
// Note that  u ^ v = (~u  &v) | (u & ~v).
// 1. u ^ 0 = u
// 2. 0 ^ v = v
// 3. u ^ v => sgn = +
// 4. (-u) ^ (-v) => sgn = -
// 5. u ^ (-v) => sgn = -
// 6. (-u) ^ v => sgn = +

sc_unsigned
operator^(const sc_unsigned &u, const sc_unsigned &v)
{
    if (v.sgn == SC_ZERO) // case 1
        return sc_unsigned(u);

    if (u.sgn == SC_ZERO) // case 2
        return sc_unsigned(v);

    // other cases
    return xor_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn,
                               v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator^(const sc_unsigned &u, uint64 v)
{
    if (v == 0) // case 1
        return sc_unsigned(u);

    CONVERT_INT64(v);

    if (u.sgn == SC_ZERO) // case 2
        return sc_unsigned(vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd, false);

    // other cases
    return xor_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                               BITS_PER_UINT64, DIGITS_PER_UINT64, vd);
}

sc_unsigned
operator^(uint64 u, const sc_unsigned &v)
{
    if (u == 0)
        return sc_unsigned(v);

    CONVERT_INT64(u);

    if (v.sgn == SC_ZERO)
        return sc_unsigned(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud, false);

    // other cases
    return xor_unsigned_friend(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud,
                               v.sgn, v.nbits, v.ndigits, v.digit);
}

sc_unsigned
operator^(const sc_unsigned &u, unsigned long v)
{
    if (v == 0) // case 1
        return sc_unsigned(u);

    CONVERT_LONG(v);

    if (u.sgn == SC_ZERO) // case 2
        return sc_unsigned(vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd, false);

    // other cases
    return xor_unsigned_friend(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                               BITS_PER_ULONG, DIGITS_PER_ULONG, vd);
}

sc_unsigned
operator^(unsigned long u, const sc_unsigned &v)
{
    if (u == 0)
        return sc_unsigned(v);

    CONVERT_LONG(u);

    if (v.sgn == SC_ZERO)
        return sc_unsigned(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, false);

    // other cases
    return xor_unsigned_friend(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, v.sgn,
                               v.nbits, v.ndigits, v.digit);
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: Bitwise NOT operator: ~
// ----------------------------------------------------------------------------

// Operators in this section are included from sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: LEFT SHIFT operators: <<, <<=
// ----------------------------------------------------------------------------

sc_unsigned
operator<<(const sc_unsigned &u, const sc_signed &v)
{
    if ((v.sgn == SC_ZERO) || (v.sgn == SC_NEG))
        return sc_unsigned(u);

    return operator<<(u, v.to_ulong());
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//    SECTION: RIGHT SHIFT operators: >>, >>=
// ----------------------------------------------------------------------------

sc_unsigned
operator>>(const sc_unsigned &u, const sc_signed &v)
{
    if ((v.sgn == SC_ZERO) || (v.sgn == SC_NEG))
        return sc_unsigned(u);

    return operator>>(u, v.to_long());
}

// The rest of the operators in this section are included from
// sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//    SECTION: Unary arithmetic operators.
// ----------------------------------------------------------------------------

sc_unsigned
operator+(const sc_unsigned &u)
{
    return sc_unsigned(u);
}

// ----------------------------------------------------------------------------
//    SECTION: EQUAL operator: ==
// ----------------------------------------------------------------------------

bool
operator==(const sc_unsigned &u, const sc_unsigned &v)
{
    if (&u == &v)
        return true;
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn, v.nbits,
                         v.ndigits, v.digit) != 0) {
        return false;
    }
    return true;
}

bool
operator==(const sc_unsigned &u, const sc_signed &v)
{
    if (v.sgn == SC_NEG)
        return false;
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn, v.nbits,
                         v.ndigits, v.digit, 0, 1) != 0) {
        return false;
    }
    return true;
}

bool
operator==(const sc_signed &u, const sc_unsigned &v)
{
    if (u.sgn == SC_NEG)
        return false;
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn, v.nbits,
                         v.ndigits, v.digit, 1, 0) != 0) {
        return false;
    }
    return true;
}

bool
operator==(const sc_unsigned &u, int64 v)
{
    if (v < 0)
        return false;
    CONVERT_INT64(v);
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                         BITS_PER_UINT64, DIGITS_PER_UINT64, vd) != 0) {
        return false;
    }
    return true;
}

bool
operator==(int64 u, const sc_unsigned &v)
{
    if (u < 0)
        return false;
    CONVERT_INT64(u);
    if (compare_unsigned(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud, v.sgn,
                         v.nbits, v.ndigits, v.digit) != 0) {
        return false;
    }
    return true;
}

bool
operator==(const sc_unsigned &u, uint64 v)
{
    CONVERT_INT64(v);
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                         BITS_PER_UINT64, DIGITS_PER_UINT64, vd) != 0)
        return false;
    return true;
}

bool
operator==(uint64 u, const sc_unsigned &v)
{
    CONVERT_INT64(u);
    if (compare_unsigned(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud, v.sgn,
                         v.nbits, v.ndigits, v.digit) != 0)
        return false;
    return true;
}

bool
operator==(const sc_unsigned &u, long v)
{
    if (v < 0)
        return false;
    CONVERT_LONG(v);
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                         BITS_PER_ULONG, DIGITS_PER_ULONG, vd) != 0)
        return false;
    return true;
}

bool
operator==(long u, const sc_unsigned &v)
{
    if (u < 0)
        return false;
    CONVERT_LONG(u);
    if (compare_unsigned(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, v.sgn,
                         v.nbits, v.ndigits, v.digit) != 0)
        return false;
    return true;
}

bool
operator==(const sc_unsigned &u, unsigned long v)
{
    CONVERT_LONG(v);
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                         BITS_PER_ULONG, DIGITS_PER_ULONG, vd) != 0)
        return false;
    return true;
}

bool
operator==(unsigned long u, const sc_unsigned &v)
{
    CONVERT_LONG(u);
    if (compare_unsigned(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, v.sgn,
                         v.nbits, v.ndigits, v.digit) != 0)
        return false;
    return true;
}

// ----------------------------------------------------------------------------
//  SECTION: NOT_EQUAL operator: !=
// ----------------------------------------------------------------------------

bool
operator!=(const sc_unsigned &u, const sc_signed &v)
{
    return (!operator==(u, v));
}

bool
operator!=(const sc_signed &u, const sc_unsigned &v)
{
    return (!operator==(u, v));
}

// The rest of the operators in this section are included from sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: LESS THAN operator: <
// ----------------------------------------------------------------------------

bool
operator<(const sc_unsigned &u, const sc_unsigned &v)
{
    if (&u == &v)
        return false;
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn, v.nbits,
                         v.ndigits, v.digit) < 0) {
        return true;
    }
    return false;
}

bool
operator<(const sc_unsigned &u, const sc_signed &v)
{
    if (v.sgn == SC_NEG)
        return false;
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn, v.nbits,
                         v.ndigits, v.digit, 0, 1) < 0) {
        return true;
    }
    return false;
}

bool
operator<(const sc_signed &u, const sc_unsigned &v)
{
    if (u.sgn == SC_NEG)
        return true;
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, v.sgn, v.nbits,
                         v.ndigits, v.digit, 1, 0) < 0) {
        return true;
    }
    return false;
}

bool
operator<(const sc_unsigned &u, int64 v)
{
    if (v < 0)
        return false;
    CONVERT_INT64(v);
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                         BITS_PER_UINT64, DIGITS_PER_UINT64, vd) < 0) {
        return true;
    }
    return false;
}

bool
operator<(int64 u, const sc_unsigned &v)
{
    if (u < 0)
        return true;
    CONVERT_INT64(u);
    if (compare_unsigned(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud, v.sgn,
                         v.nbits, v.ndigits, v.digit) < 0) {
        return true;
    }
    return false;
}

bool
operator<(const sc_unsigned &u, uint64 v)
{
    CONVERT_INT64(v);
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                         BITS_PER_UINT64, DIGITS_PER_UINT64, vd) < 0) {
        return true;
    }
    return false;
}

bool
operator<(uint64 u, const sc_unsigned &v)
{
    CONVERT_INT64(u);
    if (compare_unsigned(us, BITS_PER_UINT64, DIGITS_PER_UINT64, ud, v.sgn,
                         v.nbits, v.ndigits, v.digit) < 0) {
        return true;
    }
    return false;
}

bool
operator<(const sc_unsigned &u, long v)
{
    if (v < 0)
        return false;
    CONVERT_LONG(v);
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                         BITS_PER_ULONG, DIGITS_PER_ULONG, vd) < 0) {
        return true;
    }
    return false;
}

bool
operator<(long u, const sc_unsigned &v)
{
    if (u < 0)
        return true;
    CONVERT_LONG(u);
    if (compare_unsigned(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, v.sgn,
                         v.nbits, v.ndigits, v.digit) < 0) {
        return true;
    }
    return false;
}

bool
operator<(const sc_unsigned &u, unsigned long v)
{
    CONVERT_LONG(v);
    if (compare_unsigned(u.sgn, u.nbits, u.ndigits, u.digit, vs,
                         BITS_PER_ULONG, DIGITS_PER_ULONG, vd) < 0) {
        return true;
    }
    return false;
}

bool
operator<(unsigned long u, const sc_unsigned &v)
{
    CONVERT_LONG(u);
    if (compare_unsigned(us, BITS_PER_ULONG, DIGITS_PER_ULONG, ud, v.sgn,
                         v.nbits, v.ndigits, v.digit) < 0) {
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
//  SECTION: LESS THAN or EQUAL operator: <=
// ----------------------------------------------------------------------------

bool
operator<=(const sc_unsigned &u, const sc_signed &v)
{
    return (operator<(u, v) || operator==(u, v));
}

bool
operator<=(const sc_signed &u, const sc_unsigned &v)
{
    return (operator<(u, v) || operator==(u, v));
}

// The rest of the operators in this section are included from sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: GREATER THAN operator: >
// ----------------------------------------------------------------------------

bool
operator>(const sc_unsigned &u, const sc_signed &v)
{
    return (!(operator<=(u, v)));
}

bool
operator>(const sc_signed &u, const sc_unsigned &v)
{
    return (!(operator<=(u, v)));
}

// The rest of the operators in this section are included from sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: GREATER THAN or EQUAL operator: >=
// ----------------------------------------------------------------------------

bool
operator>=(const sc_unsigned &u, const sc_signed &v)
{
    return (!(operator<(u, v)));
}

bool
operator>=(const sc_signed &u, const sc_unsigned &v)
{
    return (!(operator<(u, v)));
}

// The rest of the operators in this section are included from sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: Friends
// ----------------------------------------------------------------------------

// Compare u and v as unsigned and return r
//  r = 0 if u == v
//  r < 0 if u < v
//  r > 0 if u > v

int
compare_unsigned(small_type us, int unb, int und, const sc_digit *ud,
                 small_type vs, int vnb, int vnd, const sc_digit *vd,
                 small_type if_u_signed, small_type if_v_signed)
{
    if (us == vs) {
        if (us == SC_ZERO) {
            return 0;
        } else {
            int cmp_res = vec_skip_and_cmp(und, ud, vnd, vd);
            if (us == SC_POS)
                return cmp_res;
            else
                return -cmp_res;
        }
    } else {
        if (us == SC_ZERO)
            return -vs;
        if (vs == SC_ZERO)
            return us;

        int cmp_res;
        int nd = (us == SC_NEG ? und : vnd);

#ifdef SC_MAX_NBITS
        sc_digit d[MAX_NDIGITS];
#else
        sc_digit *d = new sc_digit[nd];
#endif

        if (us == SC_NEG) {
            vec_copy(nd, d, ud);
            vec_complement(nd, d);
            trim(if_u_signed, unb, nd, d);
            cmp_res = vec_skip_and_cmp(nd, d, vnd, vd);
        } else {
            vec_copy(nd, d, vd);
            vec_complement(nd, d);
            trim(if_v_signed, vnb, nd, d);
            cmp_res = vec_skip_and_cmp(und, ud, nd, d);
        }

#ifndef SC_MAX_NBITS
        delete[] d;
#endif

        return cmp_res;
    }
}

// ----------------------------------------------------------------------------
//  SECTION: Public members - Other utils.
// ----------------------------------------------------------------------------

bool
sc_unsigned::iszero() const
{
    if (sgn == SC_ZERO) {
        return true;
    } else if (sgn == SC_NEG) {
        // A negative unsigned number can be zero, e.g., -16 in 4 bits, so
        // check that.

#ifdef SC_MAX_NBITS
        sc_digit d[MAX_NDIGITS];
#else
        sc_digit *d = new sc_digit[ndigits];
#endif

        vec_copy(ndigits, d, digit);
        vec_complement(ndigits, d);
        trim_unsigned(nbits, ndigits, d);

        bool res = check_for_zero(ndigits, d);

#ifndef SC_MAX_NBITS
        delete[] d;
#endif

        return res;
    } else {
        return false;
    }
}

// The rest of the utils in this section are included from sc_nbcommon.cpp.

// ----------------------------------------------------------------------------
//  SECTION: Private members.
// ----------------------------------------------------------------------------

// The private members in this section are included from
// sc_nbcommon.cpp.

#define CLASS_TYPE sc_unsigned
#define CLASS_TYPE_STR "sc_unsigned"

#define ADD_HELPER add_unsigned_friend
#define SUB_HELPER sub_unsigned_friend
#define MUL_HELPER mul_unsigned_friend
#define DIV_HELPER div_unsigned_friend
#define MOD_HELPER mod_unsigned_friend
#define AND_HELPER and_unsigned_friend
#define OR_HELPER or_unsigned_friend
#define XOR_HELPER xor_unsigned_friend

#include "sc_nbfriends.inc"

#undef SC_SIGNED
#define SC_UNSIGNED
#define IF_SC_SIGNED 0 // 0 = sc_unsigned
#define CLASS_TYPE_SUBREF sc_unsigned_subref_r
#define OTHER_CLASS_TYPE sc_signed
#define OTHER_CLASS_TYPE_SUBREF sc_signed_subref_r

#define MUL_ON_HELPER mul_on_help_unsigned
#define DIV_ON_HELPER div_on_help_unsigned
#define MOD_ON_HELPER mod_on_help_unsigned

#include "sc_nbcommon.inc"

#undef MOD_ON_HELPER
#undef DIV_ON_HELPER
#undef MUL_ON_HELPER

#undef OTHER_CLASS_TYPE_SUBREF
#undef OTHER_CLASS_TYPE
#undef CLASS_TYPE_SUBREF
#undef IF_SC_SIGNED
#undef SC_UNSIGNED

#undef XOR_HELPER
#undef OR_HELPER
#undef AND_HELPER
#undef MOD_HELPER
#undef DIV_HELPER
#undef MUL_HELPER
#undef SUB_HELPER
#undef ADD_HELPER

#undef CLASS_TYPE
#undef CLASS_TYPE_STR

#include "sc_unsigned_bitref.inc"
#include "sc_unsigned_subref.inc"

#undef CONVERT_LONG
#undef CONVERT_LONG_2
#undef CONVERT_INT64
#undef CONVERT_INT64_2

} // namespace sc_dt

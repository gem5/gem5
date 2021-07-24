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

  scfx_rep.cpp -

  Original Author: Robert Graulich, Synopsys, Inc.
                   Martin Janssen,  Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: scfx_rep.cpp,v $
// Revision 1.4  2011/08/24 22:05:43  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.3  2011/08/15 16:43:24  acg
//  Torsten Maehne: changes to remove unused argument warnings.
//
// Revision 1.2  2009/02/28 00:26:20  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.2  2008/11/06 17:22:47  acg
//  Andy Goodrich: bug fixes for 2.2.1.
//
// Revision 1.1.1.1  2006/12/15 20:31:36  acg
// SystemC 2.2
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "base/compiler.hh"
#include "systemc/ext/dt/bit/sc_bv_base.hh"
#include "systemc/ext/dt/bit/sc_lv_base.hh"
#include "systemc/ext/dt/fx/messages.hh"
#include "systemc/ext/dt/fx/scfx_ieee.hh"
#include "systemc/ext/dt/fx/scfx_pow10.hh"
#include "systemc/ext/dt/fx/scfx_rep.hh"
#include "systemc/ext/dt/fx/scfx_utils.hh"
#include "systemc/ext/utils/endian.hh"
#include "systemc/ext/utils/messages.hh"

namespace sc_dt
{

// ----------------------------------------------------------------------------
//  some utilities
// ----------------------------------------------------------------------------

static scfx_pow10 pow10_fx;

static const int mantissa0_size = SCFX_IEEE_DOUBLE_M_SIZE - bits_in_int;

static inline int
n_word(int x)
{
    return (x + bits_in_word - 1) / bits_in_word;
}


// ----------------------------------------------------------------------------
//  CONSTRUCTORS
// ----------------------------------------------------------------------------

scfx_rep::scfx_rep() :
    m_mant(min_mant), m_wp(), m_sign(), m_state(), m_msw(), m_lsw(),
    m_r_flag(false)
{
    set_zero();
}

scfx_rep::scfx_rep(int a) : m_mant(min_mant), m_wp(), m_sign(), m_state(),
    m_msw(), m_lsw(), m_r_flag(false)
{
    if (a != 0) {
        m_mant.clear();
        m_wp = m_msw = m_lsw = 2;
        m_state = normal;
        if (a > 0) {
            m_mant[2] = a;
            m_sign = 1;
        } else {
            m_mant[2] = -a;
            m_sign = -1;
        }
    } else {
        set_zero();
    }
}

scfx_rep::scfx_rep(unsigned int a) : m_mant(min_mant), m_wp(), m_sign(),
    m_state(), m_msw(), m_lsw(), m_r_flag(false)
{
    if (a != 0) {
        m_mant.clear();
        m_wp = m_msw = m_lsw = 2;
        m_state = normal;
        m_mant[2] = a;
        m_sign = 1;
    } else {
        set_zero();
    }
}

scfx_rep::scfx_rep(long a) :
    m_mant(min_mant), m_wp(), m_sign(), m_state(), m_msw(), m_lsw(),
    m_r_flag(false)
{
    if (a != 0) {
        m_mant.clear();
        m_state = normal;
        if (a > 0) {
            m_sign = 1;
        } else {
            a = -a;
            m_sign = -1;
        }
#       if SC_LONG_64
            m_wp = 1;
            m_mant[1] = static_cast<word>(a);
            m_mant[2] = static_cast<word>(a >> bits_in_word);
            find_sw();
#       else
            m_wp = 2;
            m_msw = 2;
            m_lsw = 2;
            m_mant[2] = a;
#       endif
    } else {
        set_zero();
    }
}

scfx_rep::scfx_rep(unsigned long a) :
    m_mant(min_mant), m_wp(), m_sign(), m_state(), m_msw(), m_lsw(),
    m_r_flag(false)
{
    if (a != 0) {
        m_mant.clear();
        m_wp = m_msw = m_lsw = 2;
        m_state = normal;
#       if SC_LONG_64
            m_wp = 1;
            m_mant[1] = static_cast<word>(a);
            m_mant[2] = static_cast<word>(a >> bits_in_word);
            find_sw();
#       else
            m_wp = 2;
            m_msw = 2;
            m_lsw = 2;
            m_mant[2] = a;
#       endif
        m_sign = 1;
    }
    else
        set_zero();
}

scfx_rep::scfx_rep(double a) :
    m_mant(min_mant), m_wp(0), m_sign(), m_state(normal), m_msw(0),
    m_lsw(0), m_r_flag(false)
{
    m_mant.clear();

    scfx_ieee_double id(a);

    m_sign = id.negative() ? -1 : 1;

    if (id.is_nan()) {
        m_state = not_a_number;
    } else if (id.is_inf()) {
        m_state = infinity;
    } else if (id.is_subnormal()) {
        m_mant[0] = id.mantissa1();
        m_mant[1] = id.mantissa0();
        normalize(id.exponent() + 1 - SCFX_IEEE_DOUBLE_M_SIZE);
    } else if (id.is_normal()) {
        m_mant[0] = id.mantissa1();
        m_mant[1] = id.mantissa0() | (1 << mantissa0_size);
        normalize(id.exponent() - SCFX_IEEE_DOUBLE_M_SIZE);
    }
}

scfx_rep::scfx_rep(int64 a) :
    m_mant(min_mant), m_wp(), m_sign(), m_state(), m_msw(), m_lsw(),
    m_r_flag(false)
{
    if (a != 0) {
        m_mant.clear();
        m_wp = 1;
        m_state = normal;
        if (a > 0) {
            m_mant[1] = static_cast<word>(a);
            m_mant[2] = static_cast<word>(a >> bits_in_word);
            m_sign = 1;
        } else {
            m_mant[1] = static_cast<word>(-a);
            m_mant[2] = static_cast<word>((-a) >> bits_in_word);
            m_sign = -1;
        }
        find_sw();
    } else {
        set_zero();
    }
}

scfx_rep::scfx_rep(uint64 a) :
    m_mant(min_mant), m_wp(), m_sign(), m_state(), m_msw(), m_lsw(),
    m_r_flag(false)
{
    if (a != 0) {
        m_mant.clear();
        m_wp = 1;
        m_state = normal;
        m_mant[1] = static_cast<word>(a);
        m_mant[2] = static_cast<word>(a >> bits_in_word);
        m_sign = 1;
        find_sw();
    } else {
        set_zero();
    }
}

scfx_rep::scfx_rep(const sc_signed &a) :
    m_mant(min_mant), m_wp(), m_sign(), m_state(), m_msw(), m_lsw(),
    m_r_flag(false)
{
    if (a.iszero()) {
        set_zero();
    } else {
        int words = n_word(a.length());
        if (words > size())
            resize_to(words);
        m_mant.clear();
        m_wp = 0;
        m_state = normal;
        if (a.sign()) {
            sc_signed a2 = -a;
            for (int i = 0; i < a2.length(); ++i) {
                if (a2[i]) {
                    scfx_index x = calc_indices(i);
                    m_mant[x.wi()] |= 1 << x.bi();
                }
            }
            m_sign = -1;
        } else {
            for (int i = 0; i < a.length(); ++i) {
                if (a[i]) {
                    scfx_index x = calc_indices(i);
                    m_mant[x.wi()] |= 1 << x.bi();
                }
            }
            m_sign = 1;
        }
        find_sw();
    }
}

scfx_rep::scfx_rep(const sc_unsigned &a) :
    m_mant(min_mant), m_wp(), m_sign(), m_state(), m_msw(), m_lsw(),
    m_r_flag(false)
{
    if (a.iszero()) {
        set_zero();
    } else {
        int words = n_word(a.length());
        if (words > size())
            resize_to(words);
        m_mant.clear();
        m_wp = 0;
        m_state = normal;
        for (int i = 0; i < a.length(); ++i) {
            if (a[i]) {
                scfx_index x = calc_indices(i);
                m_mant[x.wi()] |= 1 << x.bi();
            }
        }
        m_sign = 1;
        find_sw();
    }
}

// copy constructor
scfx_rep::scfx_rep(const scfx_rep &a) :
    m_mant(a.m_mant), m_wp(a.m_wp), m_sign(a.m_sign), m_state(a.m_state),
    m_msw(a.m_msw), m_lsw(a.m_lsw), m_r_flag(false)
{}


// ----------------------------------------------------------------------------
//  OPERATORS : new, delete
//
//  Memory management for class scfx_rep.
// ----------------------------------------------------------------------------

union scfx_rep_node
{
    char data[sizeof(scfx_rep)];
    scfx_rep_node *next;
};

static scfx_rep_node *list = 0;

void *
scfx_rep::operator new(std::size_t size)
{
    const int ALLOC_SIZE = 1024;

    if (size != sizeof(scfx_rep))
        return ::operator new(size);

    if (!list) {
        list = new scfx_rep_node[ALLOC_SIZE];
        for (int i = 0; i < ALLOC_SIZE - 1; i++)
            list[i].next = list + i + 1;
        list[ALLOC_SIZE - 1].next = 0;
    }

    scfx_rep *ptr = reinterpret_cast<scfx_rep *>(list->data);
    list = list->next;

    return ptr;
}

void
scfx_rep::operator delete(void *ptr, std::size_t size)
{
    if (size != sizeof(scfx_rep)) {
        ::operator delete(ptr);
        return;
    }

    scfx_rep_node *node = static_cast<scfx_rep_node *>(ptr);
    node->next = list;
    list = node;
}


// ----------------------------------------------------------------------------
//  METHOD : from_string
//
//  Convert from character string to sc_fxrep.
// ----------------------------------------------------------------------------

#define SCFX_FAIL_IF_(cnd) \
{ \
    if ((cnd)) { \
        m_state = not_a_number; \
        m_mant.clear(); /* to avoid Purify UMRs during assignment */ \
        return; \
    } \
}

void
scfx_rep::from_string(const char *s, int cte_wl)
{
    SCFX_FAIL_IF_(s == 0 || *s == 0);

    scfx_string s2;
    s2 += s;
    s2 += '\0';

    bool sign_char;
    m_sign = scfx_parse_sign(s, sign_char);

    sc_numrep numrep = scfx_parse_prefix(s);

    int base = 0;

    switch (numrep) {
      case SC_DEC:
        {
            base = 10;
            if (scfx_is_nan(s)) {   // special case: NaN
                m_state = not_a_number;
                m_mant.clear(); /* to avoid Purify UMRs during assignment */
                return;
            }
            if (scfx_is_inf(s)) {   // special case: Infinity
                m_state = infinity;
                m_mant.clear(); /* to avoid Purify UMRs during assignment */
                return;
            }
            break;
        }
      case SC_BIN:
      case SC_BIN_US:
        {
            SCFX_FAIL_IF_(sign_char);
            base = 2;
            break;
        }

      case SC_BIN_SM:
        {
            base = 2;
            break;
        }
      case SC_OCT:
      case SC_OCT_US:
        {
            SCFX_FAIL_IF_(sign_char);
            base = 8;
            break;
        }
      case SC_OCT_SM:
        {
            base = 8;
            break;
        }
      case SC_HEX:
      case SC_HEX_US:
        {
            SCFX_FAIL_IF_(sign_char);
            base = 16;
            break;
        }
      case SC_HEX_SM:
        {
            base = 16;
            break;
        }
      case SC_CSD:
        {
            SCFX_FAIL_IF_(sign_char);
            base = 2;
            scfx_csd2tc(s2);
            s = (const char *)s2 + 4;
            numrep = SC_BIN;
            break;
        }
      default:
        ;
    }

    //
    // find end of mantissa and count the digits and points
    //

    const char *end = s;
    bool based_point = false;
    int int_digits = 0;
    int frac_digits = 0;

    while (*end) {
        if (scfx_exp_start(end))
            break;

        if (*end == '.') {
            SCFX_FAIL_IF_(based_point);
            based_point = true;
        } else {
            SCFX_FAIL_IF_(!scfx_is_digit(*end, numrep));
            if (based_point)
                frac_digits++;
            else
                int_digits++;
        }

        ++end;
    }

    SCFX_FAIL_IF_(int_digits == 0 && frac_digits == 0);

    // [ exponent ]
    int exponent = 0;

    if (*end) {
        for (const char *e = end + 2; *e; ++e)
            SCFX_FAIL_IF_(!scfx_is_digit(*e, SC_DEC));
        exponent = std::atoi(end + 1);
    }

    //
    // check if the mantissa is negative
    //
    bool mant_is_neg = false;
    switch (numrep) {
      case SC_BIN:
      case SC_OCT:
      case SC_HEX:
        {
            const char *p = s;
            if (*p == '.')
                ++p;

            mant_is_neg = (scfx_to_digit(* p, numrep) >= (base >> 1));
            break;
        }
      default:
            ;
    }

    //
    // convert the mantissa
    //

    switch (base) {
      case 2:
        {
            int bit_offset = exponent % bits_in_word;
            int word_offset = exponent / bits_in_word;

            int_digits += bit_offset;
            frac_digits -= bit_offset;

            int words = n_word(int_digits) + n_word(frac_digits);
            if (words > size())
                resize_to(words);
            m_mant.clear();

            int j = n_word(frac_digits) * bits_in_word + int_digits - 1;

            for (; s < end; s++) {
                switch (*s) {
                  case '1':
                    set_bin(j);
                    [[fallthrough]];
                  case '0':
                    j--;
                    [[fallthrough]];
                  case '.':
                    break;
                  default:
                    SCFX_FAIL_IF_(true); // should not happen
                }
            }

            m_wp = n_word(frac_digits) - word_offset;
            break;
        }
      case 8:
        {
            exponent *= 3;
            int_digits *= 3;
            frac_digits *= 3;

            int bit_offset = exponent % bits_in_word;
            int word_offset = exponent / bits_in_word;

            int_digits += bit_offset;
            frac_digits -= bit_offset;

            int words = n_word(int_digits) + n_word(frac_digits);
            if (words > size())
                resize_to(words);
            m_mant.clear();

            int j = n_word(frac_digits) * bits_in_word + int_digits - 3;

            for (; s < end; s++) {
                switch (*s) {
                  case '7': case '6': case '5': case '4':
                  case '3': case '2': case '1':
                    set_oct(j, *s - '0');
                    [[fallthrough]];
                  case '0':
                    j -= 3;
                    [[fallthrough]];
                  case '.':
                    break;
                  default:
                    SCFX_FAIL_IF_(true); // should not happen
                }
            }

            m_wp = n_word(frac_digits) - word_offset;
            break;
        }
      case 10:
        {
            word carry, temp;
            int length = int_digits + frac_digits;
            resize_to(sc_max(min_mant, n_word(4 * length)));

            m_mant.clear();
            m_msw = m_lsw = 0;

            for (; s < end; s++) {
                switch (*s) {
                  case '9': case '8': case '7': case '6': case '5':
                  case '4': case '3': case '2': case '1': case '0':
                    multiply_by_ten();
                    carry = *s - '0';
                    for (int i = 0; carry && i < m_mant.size(); i++) {
                        temp = m_mant[i];
                        temp += carry;
                        carry = temp < m_mant[i];
                        m_mant[i] = temp;
                    }
                  case '.':
                    break;
                  default:
                    SCFX_FAIL_IF_(true); // should not happen
                }
            }

            m_wp = 0;
            find_sw();

            int denominator = frac_digits - exponent;

            if (denominator) {
                scfx_rep frac_num = pow10_fx(denominator);
                scfx_rep *temp_num =
                    div_scfx_rep(const_cast<const scfx_rep &>(*this),
                                   frac_num, cte_wl);
                *this = *temp_num;
                delete temp_num;
            }

            break;
        }
      case 16:
        {
            exponent *= 4;
            int_digits *= 4;
            frac_digits *= 4;

            int bit_offset = exponent % bits_in_word;
            int word_offset = exponent / bits_in_word;

            int_digits += bit_offset;
            frac_digits -= bit_offset;

            int words = n_word(int_digits) + n_word(frac_digits);
            if (words > size())
                resize_to(words);
            m_mant.clear();

            int j = n_word(frac_digits) * bits_in_word + int_digits - 4;

            for (; s < end; s ++) {
                switch (*s) {
                  case 'f': case 'e': case 'd': case 'c': case 'b': case 'a':
                    set_hex(j, *s - 'a' + 10);
                    j -= 4;
                    break;
                  case 'F': case 'E': case 'D': case 'C': case 'B': case 'A':
                    set_hex(j, *s - 'A' + 10);
                    j -= 4;
                    break;
                  case '9': case '8': case '7': case '6': case '5':
                  case '4': case '3': case '2': case '1':
                    set_hex(j, *s - '0');
                    [[fallthrough]];
                  case '0':
                    j -= 4;
                    [[fallthrough]];
                  case '.':
                    break;
                  default:
                    SCFX_FAIL_IF_(true); // should not happen
                }
            }

            m_wp = n_word(frac_digits) - word_offset;
            break;
        }
    }

    m_state = normal;
    find_sw();

    //
    // two's complement of mantissa if it is negative
    //
    if (mant_is_neg) {
        m_mant[m_msw] |=  ~0U << scfx_find_msb(m_mant[m_msw]);
        for (int i = m_msw + 1; i < m_mant.size(); ++i)
            m_mant[i] = static_cast<word>(-1);
        complement(m_mant, m_mant, m_mant.size());
        inc(m_mant);
        m_sign *= -1;
        find_sw();
    }
}

#undef SCFX_FAIL_IF_

// ----------------------------------------------------------------------------
//  METHOD : to_double
//
//  Convert from scfx_rep to double.
// ----------------------------------------------------------------------------

double
scfx_rep::to_double() const
{
    scfx_ieee_double id;

    // handle special cases
    if (is_nan()) {
        id.set_nan();
        return id;
    }

    if (is_inf()) {
        id.set_inf();
        id.negative(m_sign < 0);
        return id;
    }

    if (is_zero()) {
        id = 0.;
        id.negative(m_sign < 0);
        return id;
    }

    int msb = scfx_find_msb(m_mant[m_msw]);

    int exp = (m_msw - m_wp) * bits_in_word + msb;

    if (exp > SCFX_IEEE_DOUBLE_E_MAX) {
        id.set_inf();
        id.negative(m_sign < 0);
        return id;
    }

    if (exp < SCFX_IEEE_DOUBLE_E_MIN -
        static_cast<int>(SCFX_IEEE_DOUBLE_M_SIZE))
    {
        id = 0.;
        return id;
    }

    int shift = mantissa0_size - msb;

    unsigned int m0;
    unsigned int m1 = 0;
    unsigned int guard = 0;

    if (shift == 0) {
        m0 = m_mant[m_msw] & ~(1 << mantissa0_size);
        if (m_msw > m_lsw) {
            m1 = m_mant[m_msw - 1];
            if (m_msw - 1 > m_lsw)
                guard = m_mant[m_msw - 2] >> (bits_in_word - 1);
        }
    } else if (shift < 0) {
        m0 = (m_mant[m_msw] >> -shift) & ~(1 << mantissa0_size);
        m1 = m_mant[m_msw] << (bits_in_word + shift);
        if (m_msw > m_lsw) {
            m1 |= m_mant[m_msw - 1] >> -shift;
            guard = (m_mant[m_msw - 1] >> (-shift - 1)) & 1;
        }
    } else {
        m0 = (m_mant[m_msw] << shift) & ~(1 << mantissa0_size);
        if (m_msw > m_lsw) {
            m0 |= m_mant[m_msw - 1] >> (bits_in_word - shift);
            m1 = m_mant[m_msw - 1] << shift;
            if (m_msw - 1 > m_lsw) {
                m1 |= m_mant[m_msw - 2] >> (bits_in_word - shift);
                guard = (m_mant[m_msw - 2] >> (bits_in_word - shift - 1)) & 1;
            }
        }
    }

    if (exp < SCFX_IEEE_DOUBLE_E_MIN) {
        m0 |= (1 << mantissa0_size);

        int subnormal_shift = SCFX_IEEE_DOUBLE_E_MIN - exp;

        if (subnormal_shift < bits_in_word) {
            m1 = m1 >> subnormal_shift |
                m0 << (bits_in_word - subnormal_shift);
            m0 = m0 >> subnormal_shift;
        } else {
            m1 = m0 >> (subnormal_shift - bits_in_word);
            m0 = 0;
        }

        guard = 0;

        exp = SCFX_IEEE_DOUBLE_E_MIN - 1;
    }

    id.mantissa0(m0);
    id.mantissa1(m1);
    id.exponent(exp);
    id.negative(m_sign < 0);

    double result = id;

    if (guard != 0)
        result += m_sign * scfx_pow2(exp - SCFX_IEEE_DOUBLE_M_SIZE);

    return result;
}


// ----------------------------------------------------------------------------
//  METHOD : to_uint64
//
//  Convert from scfx_rep to uint64.
//  Truncates towards 0 _then_ wraps; infinities and NaN go to zero.
// ----------------------------------------------------------------------------

uint64
scfx_rep::to_uint64() const
{
    if (!is_normal() || is_zero()) {
        return 0;
    }

    uint64 result = 0;
    int shift = 0;
    int idx = m_wp;

    // Ignore bits off the top; they modulo out.
    // Ignore bits off the bottom; we're truncating.
    while (shift < 64 && m_msw >= idx && idx >= m_lsw) {
        result += static_cast<uint64>(m_mant[idx]) << shift;
        shift += bits_in_word;
        idx += 1;
    }

    return m_sign > 0 ? result : -result;
}


// ----------------------------------------------------------------------------
//  METHOD : to_string
//
//  Convert from scfx_rep to character string.
// ----------------------------------------------------------------------------

void
print_dec(scfx_string &s, const scfx_rep &num, int w_prefix, sc_fmt fmt)
{
    if (num.is_neg())
        s += '-';

    if (w_prefix == 1) {
        scfx_print_prefix(s, SC_DEC);
    }

    if (num.is_zero()) {
        s += '0';
        return;
    }

    // split 'num' into its integer and fractional part
    scfx_rep int_part = num;
    scfx_rep frac_part = num;

    int i;

    for (i = int_part.m_lsw; i <= int_part.m_msw && i < int_part.m_wp; i++)
        int_part.m_mant[i] = 0;
    int_part.find_sw();
    if (int_part.m_wp < int_part.m_lsw)
        int_part.resize_to(int_part.size() - int_part.m_wp, -1);

    for (i = frac_part.m_msw;
            i >= frac_part.m_lsw && i >= frac_part.m_wp; i--)
        frac_part.m_mant[i] = 0;
    frac_part.find_sw();
    if (frac_part.m_msw == frac_part.size() - 1)
        frac_part.resize_to(frac_part.size() + 1, 1);

    // print integer part
    int int_digits = 0;
    int int_zeros  = 0;

    if (!int_part.is_zero()) {
        double int_wl = (int_part.m_msw - int_part.m_wp) * bits_in_word +
                         scfx_find_msb(int_part.m_mant[int_part.m_msw]) + 1;
        int_digits = (int)std::ceil(int_wl * std::log10(2.));

        int len = s.length();
        s.append(int_digits);

        bool zero_digits = (frac_part.is_zero() && fmt != SC_F);

        for (i = int_digits + len - 1; i >= len; i--) {
            unsigned int remainder = int_part.divide_by_ten();
            s[i] = static_cast<char>('0' + remainder);

            if (zero_digits) {
                if (remainder == 0)
                    int_zeros++;
                else
                    zero_digits = false;
            }
        }

        // discard trailing zeros from int_part
        s.discard(int_zeros);

        if (s[len] == '0') {
            // int_digits was overestimated by one
            s.remove(len);
            --int_digits;
        }
    }

    // print fractional part
    int frac_digits = 0;
    int frac_zeros  = 0;

    if (!frac_part.is_zero()) {
        s += '.';

        bool zero_digits = (int_digits == 0 && fmt != SC_F);

        double frac_wl = (frac_part.m_wp - frac_part.m_msw) * bits_in_word -
                          scfx_find_msb(frac_part.m_mant[frac_part.m_msw]) - 1;
        frac_zeros = (int)std::floor(frac_wl * std::log10(2.));

        scfx_rep temp;
        sc_dt::multiply(temp, frac_part, pow10_fx(frac_zeros));
        frac_part = temp;
        if (frac_part.m_msw == frac_part.size() - 1)
            frac_part.resize_to(frac_part.size() + 1, 1);

        frac_digits = frac_zeros;
        if (!zero_digits) {
            for (i = 0; i < frac_zeros; i++)
                s += '0';
            frac_zeros = 0;
        }

        while (!frac_part.is_zero()) {
            frac_part.multiply_by_ten();
            int n = frac_part.m_mant[frac_part.m_msw + 1];

            if (zero_digits) {
                if (n == 0)
                    frac_zeros++;
                else
                    zero_digits = false;
            }

            if (! zero_digits)
                s += static_cast<char>('0' + n);

            frac_part.m_mant[frac_part.m_msw + 1] = 0;
            frac_digits++;
        }
    }

    // print exponent
    if (fmt != SC_F) {
        if (frac_digits == 0)
            scfx_print_exp(s, int_zeros);
        else if (int_digits == 0)
            scfx_print_exp(s, -frac_zeros);
    }
}

void
print_other(scfx_string &s, const scfx_rep &a, sc_numrep numrep, int w_prefix,
            sc_fmt fmt, const scfx_params *params)
{
    scfx_rep b = a;

    sc_numrep numrep2 = numrep;

    bool numrep_is_sm = (numrep == SC_BIN_SM ||
                         numrep == SC_OCT_SM ||
                         numrep == SC_HEX_SM);

    if (numrep_is_sm) {
        if (b.is_neg()) {
            s += '-';
            b = *neg_scfx_rep(a);
        }
        switch (numrep) {
          case SC_BIN_SM:
            numrep2 = SC_BIN_US;
            break;
          case SC_OCT_SM:
            numrep2 = SC_OCT_US;
            break;
          case SC_HEX_SM:
            numrep2 = SC_HEX_US;
            break;
          default:
            ;
        }
    }

    if (w_prefix != 0) {
        scfx_print_prefix(s, numrep);
    }

    numrep = numrep2;

    int msb, lsb;

    if (params != 0) {
        msb = params->iwl() - 1;
        lsb = params->iwl() - params->wl();

        if (params->enc() == SC_TC_ &&
            (numrep == SC_BIN_US ||
              numrep == SC_OCT_US ||
              numrep == SC_HEX_US) &&
            !numrep_is_sm &&
            params->wl() > 1) {
            --msb;
        } else if (params->enc() == SC_US_ &&
            (numrep == SC_BIN ||
              numrep == SC_OCT ||
              numrep == SC_HEX ||
              numrep == SC_CSD)) {
            ++msb;
        }
    } else {
        if (b.is_zero()) {
            msb = 0;
            lsb = 0;
        } else {
            msb = (b.m_msw - b.m_wp) * bits_in_word
                + scfx_find_msb(b.m_mant[ b.m_msw ]) + 1;
            while (b.get_bit(msb) == b.get_bit(msb - 1))
                --msb;

            if (numrep == SC_BIN_US ||
                numrep == SC_OCT_US ||
                numrep == SC_HEX_US) {
                --msb;
            }

            lsb = (b.m_lsw - b.m_wp) * bits_in_word +
                scfx_find_lsb(b.m_mant[b.m_lsw]);

        }
    }

    int step;

    switch (numrep) {
      case SC_BIN:
      case SC_BIN_US:
      case SC_CSD:
        step = 1;
       break;
      case SC_OCT:
      case SC_OCT_US:
        step = 3;
        break;
      case SC_HEX:
      case SC_HEX_US:
        step = 4;
        break;
      default:
        SC_REPORT_FATAL(sc_core::SC_ID_ASSERTION_FAILED_,
                "unexpected sc_numrep");
        sc_core::sc_abort();
    }

    msb = (int)std::ceil(double(msb + 1) / step) * step - 1;

    lsb = (int)std::floor(double(lsb) / step) * step;

    if (msb < 0) {
        s += '.';
        if (fmt == SC_F) {
            int sign = (b.is_neg()) ? (1 << step) - 1 : 0;
            for (int i = (msb + 1) / step; i < 0; i++) {
                if (sign < 10)
                    s += static_cast<char>(sign + '0');
                else
                    s += static_cast<char>(sign + 'a' - 10);
            }
        }
    }

    int i = msb;
    while (i >= lsb) {
        int value = 0;
        for (int j = step - 1; j >= 0; --j) {
            value += static_cast<int>(b.get_bit(i)) << j;
            --i;
        }
        if (value < 10)
            s += static_cast<char>(value + '0');
        else
            s += static_cast<char>(value + 'a' - 10);
        if (i == -1)
            s += '.';
    }

    if (lsb > 0 && fmt == SC_F) {
        for (int i = lsb / step; i > 0; i--)
            s += '0';
    }

    if (s[s.length() - 1] == '.')
        s.discard(1);

    if (fmt != SC_F) {
        if (msb < 0)
            scfx_print_exp(s, (msb + 1) / step);
        else if (lsb > 0)
            scfx_print_exp(s, lsb / step);
    }

    if (numrep == SC_CSD)
        scfx_tc2csd(s, w_prefix);
}

const char *
scfx_rep::to_string(sc_numrep numrep, int w_prefix,
                    sc_fmt fmt, const scfx_params *params) const
{
    static scfx_string s;

    s.clear();

    if (is_nan()) {
        scfx_print_nan(s);
    } else if (is_inf()) {
        scfx_print_inf(s, is_neg());
    } else if (is_neg() && !is_zero() &&
               (numrep == SC_BIN_US ||
                numrep == SC_OCT_US ||
                numrep == SC_HEX_US)) {
        s += "negative";
    } else if (numrep == SC_DEC || numrep == SC_NOBASE) {
        sc_dt::print_dec(s, *this, w_prefix, fmt);
    } else {
        sc_dt::print_other(s, *this, numrep, w_prefix, fmt, params);
    }

    return s;
}


// ----------------------------------------------------------------------------
//  ADD
//
//  add two mantissas of the same size
//  result has the same size
//  returns carry of operation
// ----------------------------------------------------------------------------

static inline int
add_mants(int size, scfx_mant &result, const scfx_mant &a, const scfx_mant &b)
{
    unsigned int carry = 0;

    int index = 0;

    do {
        word x = a[index];
        word y = b[index];

        y += carry;
        carry = y < carry;
        y += x;
        carry += y < x;
        result[index] = y;
    } while (++index < size);

    return (carry ? 1 : 0);
}

static inline int
sub_mants(int size, scfx_mant &result, const scfx_mant &a, const scfx_mant &b)
{
    unsigned carry = 0;

    int index = 0;

    do {
        word x = a[index];
        word y = b[index];

        y += carry;
        carry = y < carry;
        y = x - y;
        carry += y > x;
        result[index] = y;
    } while (++index < size);

    return (carry ? 1 : 0);
}

scfx_rep *
add_scfx_rep(const scfx_rep &lhs, const scfx_rep &rhs, int max_wl)
{
    scfx_rep &result = *new scfx_rep;

    //
    // check for special cases
    //
    if (lhs.is_nan() || rhs.is_nan() ||
        (lhs.is_inf() && rhs.is_inf() && lhs.m_sign != rhs.m_sign)) {
        result.set_nan();
        return &result;
    }

    if (lhs.is_inf()) {
        result.set_inf(lhs.m_sign);
        return &result;
    }

    if (rhs.is_inf()) {
        result.set_inf(rhs.m_sign);
        return &result;
    }

    //
    // align operands if needed
    //
    scfx_mant_ref lhs_mant;
    scfx_mant_ref rhs_mant;

    int len_mant = lhs.size();
    int new_wp = lhs.m_wp;

    align(lhs, rhs, new_wp, len_mant, lhs_mant, rhs_mant);

    //
    // size the result mantissa
    //
    result.resize_to(len_mant);
    result.m_wp = new_wp;

    //
    // do it
    //
    if (lhs.m_sign == rhs.m_sign) {
        add_mants(len_mant, result.m_mant, lhs_mant, rhs_mant);
        result.m_sign = lhs.m_sign;
    } else {
        int cmp = compare_abs(lhs, rhs);

        if (cmp == 1) {
            sub_mants(len_mant, result.m_mant, lhs_mant, rhs_mant);
            result.m_sign = lhs.m_sign;
        } else if (cmp == -1) {
            sub_mants(len_mant, result.m_mant, rhs_mant, lhs_mant);
            result.m_sign = rhs.m_sign;
        } else {
            result.m_mant.clear();
            result.m_sign = 1;
        }
    }

    result.find_sw();
    result.round(max_wl);

    return &result;
}


// ----------------------------------------------------------------------------
//  SUB
//
//  sub two word's of the same size
//  result has the same size
//  returns carry of operation
// ----------------------------------------------------------------------------

static inline int
sub_with_index(scfx_mant &a, int a_msw, int /*a_lsw*/,
               const scfx_mant &b, int b_msw, int b_lsw)
{
    unsigned carry = 0;

    int size = b_msw - b_lsw;
    int a_index = a_msw - size;
    int b_index = b_msw - size;

    do {
        word x = a[a_index];
        word y = b[b_index];

        y += carry;
        carry = y < carry;
        y = x - y;
        carry += y > x;
        a[a_index] = y;

        a_index++;
        b_index++;
    } while (size--);

    if (carry) {
        // special case: a[a_msw + 1] == 1
        a[a_msw + 1] = 0;
    }

    return (carry ? 1 : 0);
}

scfx_rep *
sub_scfx_rep(const scfx_rep &lhs, const scfx_rep &rhs, int max_wl)
{
    scfx_rep &result = *new scfx_rep;

    //
    // check for special cases
    //
    if (lhs.is_nan() || rhs.is_nan() ||
        (lhs.is_inf() && rhs.is_inf() && lhs.m_sign == rhs.m_sign)) {
        result.set_nan();
        return &result;
    }

    if (lhs.is_inf()) {
        result.set_inf(lhs.m_sign);
        return &result;
    }

    if (rhs.is_inf()) {
        result.set_inf(-1 * rhs.m_sign);
        return &result;
    }

    //
    // align operands if needed
    //
    scfx_mant_ref lhs_mant;
    scfx_mant_ref rhs_mant;

    int len_mant = lhs.size();
    int new_wp = lhs.m_wp;

    align(lhs, rhs, new_wp, len_mant, lhs_mant, rhs_mant);

    //
    // size the result mantissa
    //
    result.resize_to(len_mant);
    result.m_wp = new_wp;

    //
    // do it
    //
    if (lhs.m_sign != rhs.m_sign) {
        add_mants(len_mant, result.m_mant, lhs_mant, rhs_mant);
        result.m_sign = lhs.m_sign;
    } else {
        int cmp = compare_abs(lhs, rhs);

        if (cmp == 1) {
            sub_mants(len_mant, result.m_mant, lhs_mant, rhs_mant);
            result.m_sign = lhs.m_sign;
        } else if (cmp == -1) {
            sub_mants(len_mant, result.m_mant, rhs_mant, lhs_mant);
            result.m_sign = -rhs.m_sign;
        } else {
            result.m_mant.clear();
            result.m_sign = 1;
        }
    }

    result.find_sw();
    result.round(max_wl);

    return &result;
}


// ----------------------------------------------------------------------------
//  MUL
// ----------------------------------------------------------------------------

union word_short
{
    word l;
    struct
    {
#if defined(SC_BOOST_BIG_ENDIAN)
        half_word u;
        half_word l;
#elif defined(SC_BOOST_LITTLE_ENDIAN)
        half_word l;
        half_word u;
#endif
    } s;
};

#if defined(SC_BOOST_BIG_ENDIAN)
static const int half_word_incr = -1;
#elif defined(SC_BOOST_LITTLE_ENDIAN)
static const int half_word_incr = 1;
#endif

void
multiply(scfx_rep &result, const scfx_rep &lhs, const scfx_rep &rhs,
         int max_wl)
{
    //
    // check for special cases
    //
    if (lhs.is_nan() || rhs.is_nan() ||
        (lhs.is_inf() && rhs.is_zero()) ||
        (lhs.is_zero() && rhs.is_inf())) {
        result.set_nan();
        return;
    }

    if (lhs.is_inf() || rhs.is_inf()) {
        result.set_inf(lhs.m_sign * rhs.m_sign);
        return;
    }

    if (lhs.is_zero() || rhs.is_zero()) {
        result.set_zero(lhs.m_sign * rhs.m_sign);
        return;
    }

    //
    // do it
    //
    int len_lhs = lhs.m_msw - lhs.m_lsw + 1;
    int len_rhs = rhs.m_msw - rhs.m_lsw + 1;

    int new_size = sc_max(min_mant, len_lhs + len_rhs);
    int new_wp = (lhs.m_wp - lhs.m_lsw) + (rhs.m_wp - rhs.m_lsw);
    int new_sign = lhs.m_sign * rhs.m_sign;

    result.resize_to(new_size);
    result.m_mant.clear();
    result.m_wp = new_wp;
    result.m_sign = new_sign;
    result.m_state = scfx_rep::normal;

    half_word *s1 = lhs.m_mant.half_addr(lhs.m_lsw);
    half_word *s2 = rhs.m_mant.half_addr(rhs.m_lsw);

    half_word *t = result.m_mant.half_addr();

    len_lhs <<= 1;
    len_rhs <<= 1;

    int i1, i2;

    for (i1 = 0; i1 * half_word_incr < len_lhs; i1 += half_word_incr) {
        word_short ls;
        ls.l = 0;

        half_word v1 = s1[i1];

        for (i2  = 0; i2 * half_word_incr < len_rhs; i2 += half_word_incr) {
            ls.l  += v1 * s2[i2];
            ls.s.l = ls.s.u + ((t[i2] += ls.s.l) < ls.s.l);
            ls.s.u = 0;
        }

        t[i2] = ls.s.l;
        t += half_word_incr;
    }

    result.find_sw();
    result.round(max_wl);
}


// ----------------------------------------------------------------------------
//  DIV
// ----------------------------------------------------------------------------

scfx_rep *
div_scfx_rep(const scfx_rep &lhs, const scfx_rep &rhs, int div_wl)
{
    scfx_rep &result = *new scfx_rep;

    //
    // check for special cases
    //
    if (lhs.is_nan() || rhs.is_nan() || (lhs.is_inf() && rhs.is_inf()) ||
        (lhs.is_zero() && rhs.is_zero())) {
        result.set_nan();
        return &result;
    }

    if (lhs.is_inf() || rhs.is_zero()) {
        result.set_inf(lhs.m_sign * rhs.m_sign);
        return &result;
    }

    if (lhs.is_zero() || rhs.is_inf()) {
        result.set_zero(lhs.m_sign * rhs.m_sign);
        return &result;
    }

    //
    // do it
    //

    // compute one bit more for rounding
    div_wl++;

    result.resize_to(sc_max(n_word(div_wl) + 1, min_mant));
    result.m_mant.clear();
    result.m_sign = lhs.m_sign * rhs.m_sign;

    int msb_lhs = scfx_find_msb(lhs.m_mant[lhs.m_msw]) +
                  (lhs.m_msw - lhs.m_wp) * bits_in_word;
    int msb_rhs = scfx_find_msb(rhs.m_mant[rhs.m_msw]) +
                  (rhs.m_msw - rhs.m_wp) * bits_in_word;

    int msb_res = msb_lhs - msb_rhs;
    int to_shift = -msb_res % bits_in_word;
    int result_index;

    int c = (msb_res % bits_in_word >= 0) ? 1 : 0;

    result_index = (result.size() - c) * bits_in_word + msb_res % bits_in_word;
    result.m_wp = (result.size() - c) - msb_res / bits_in_word;

    scfx_rep remainder = lhs;

    // align msb from remainder to msb from rhs
    remainder.lshift(to_shift);

    // make sure msw(remainder) < size - 1
    if (remainder.m_msw == remainder.size() - 1)
        remainder.resize_to(remainder.size() + 1, 1);

    // make sure msw(remainder) >= msw(rhs)!
    int msw_diff = rhs.m_msw - remainder.m_msw;
    if (msw_diff > 0)
        remainder.resize_to(remainder.size() + msw_diff, -1);

    int counter;

    for (counter = div_wl; counter && !remainder.is_zero(); counter--) {
        if (compare_msw_ff(rhs, remainder) <= 0) {
            result.set_bin(result_index);
            sub_with_index(remainder.m_mant, remainder.m_msw, remainder.m_lsw,
                           rhs.m_mant, rhs.m_msw, rhs.m_lsw);
        }
        result_index--;
        remainder.shift_left(1);
        remainder.m_lsw = remainder.find_lsw();
    }

    // perform convergent rounding, if needed
    if (counter == 0) {
        int index = result_index + 1 - result.m_wp * bits_in_word;

        scfx_index x = result.calc_indices(index);
        scfx_index x1 = result.calc_indices(index + 1);

        if (result.o_bit_at(x) && result.o_bit_at(x1))
            result.q_incr(x);

        result.m_r_flag = true;
    }

    result.find_sw();

    return &result;
}

// ----------------------------------------------------------------------------
//  destructive shift mantissa to the left
// ----------------------------------------------------------------------------

void
scfx_rep::lshift(int n)
{
    if (n == 0)
        return;

    if (n < 0) {
        rshift(-n);
        return;
    }

    if (is_normal()) {
        int shift_bits = n % bits_in_word;
        int shift_words = n / bits_in_word;

        // resize if needed
        if (m_msw == size() - 1 &&
            scfx_find_msb(m_mant[m_msw]) >= bits_in_word - shift_bits)
            resize_to(size() + 1, 1);

        // do it
        m_wp -= shift_words;
        shift_left(shift_bits);
        find_sw();
    }
}

// ----------------------------------------------------------------------------
//  destructive shift mantissa to the right
// ----------------------------------------------------------------------------

void
scfx_rep::rshift(int n)
{
    if (n == 0)
        return;

    if (n < 0) {
        lshift(-n);
        return;
    }

    if (is_normal()) {
        int shift_bits = n % bits_in_word;
        int shift_words = n / bits_in_word;

        // resize if needed
        if (m_lsw == 0 && scfx_find_lsb(m_mant[m_lsw]) < shift_bits)
            resize_to(size() + 1, -1);

        // do it
        m_wp += shift_words;
        shift_right(shift_bits);
        find_sw();
    }
}


// ----------------------------------------------------------------------------
//  FRIEND FUNCTION : compare_abs
//
//  Compares the absolute values of two scfx_reps, excluding the special cases.
// ----------------------------------------------------------------------------

int
compare_abs(const scfx_rep &a, const scfx_rep &b)
{
    // check for zero
    word a_word = a.m_mant[a.m_msw];
    word b_word = b.m_mant[b.m_msw];

    if (a_word == 0 || b_word == 0) {
        if (a_word != 0)
            return 1;
        if (b_word != 0)
            return -1;
        return 0;
    }

    // compare msw index
    int a_msw = a.m_msw - a.m_wp;
    int b_msw = b.m_msw - b.m_wp;

    if (a_msw > b_msw)
        return 1;

    if (a_msw < b_msw)
        return -1;

    // compare content
    int a_i = a.m_msw;
    int b_i = b.m_msw;

    while (a_i >= a.m_lsw && b_i >= b.m_lsw) {
        a_word = a.m_mant[a_i];
        b_word = b.m_mant[b_i];
        if (a_word > b_word)
            return 1;
        if (a_word < b_word)
            return -1;
        --a_i;
        --b_i;
    }

    bool a_zero = true;
    while (a_i >= a.m_lsw) {
        a_zero = a_zero && (a.m_mant[a_i] == 0);
        --a_i;
    }

    bool b_zero = true;
    while (b_i >= b.m_lsw) {
        b_zero = b_zero && (b.m_mant[b_i] == 0);
        --b_i;
    }

    // assertion: a_zero || b_zero

    if (!a_zero && b_zero)
        return 1;

    if (a_zero && !b_zero)
        return -1;

    return 0;
}

// ----------------------------------------------------------------------------
//  FRIEND FUNCTION : cmp_scfx_rep
//
//  Compares the values of two scfx_reps, including the special cases.
// ----------------------------------------------------------------------------

int
cmp_scfx_rep(const scfx_rep &a, const scfx_rep &b)
{
    // handle special cases

    if (a.is_nan() || b.is_nan()) {
        return 2;
    }

    if (a.is_inf() || b.is_inf()) {
        if (a.is_inf()) {
            if (!a.is_neg()) {
                if (b.is_inf() && !b.is_neg()) {
                    return 0;
                } else {
                    return 1;
                }
            } else {
                if (b.is_inf() && b.is_neg()) {
                    return 0;
                } else {
                    return -1;
                }
            }
        }
        if (b.is_inf()) {
            if (!b.is_neg()) {
                return -1;
            } else {
                return 1;
            }
        }
    }

    if (a.is_zero() && b.is_zero()) {
        return 0;
    }

    // compare sign
    if (a.m_sign != b.m_sign) {
        return a.m_sign;
    }

    return (a.m_sign * compare_abs(a, b));
}


// ----------------------------------------------------------------------------
//  PRIVATE METHOD : quantization
//
//  Performs destructive quantization.
// ----------------------------------------------------------------------------

void
scfx_rep::quantization(const scfx_params &params, bool &q_flag)
{
    scfx_index x = calc_indices(params.iwl() - params.wl());

    if (x.wi() < 0)
        return;

    if (x.wi() >= size())
        resize_to(x.wi() + 1, 1);

    bool qb = q_bit(x);
    bool qz = q_zero(x);

    q_flag = (qb || ! qz);

    if (q_flag) {
        switch (params.q_mode()) {
          case SC_TRN: // truncation
            {
                if (is_neg())
                    q_incr(x);
                break;
            }
          case SC_RND: // rounding to plus infinity
            {
                if (!is_neg()) {
                    if (qb)
                        q_incr(x);
                } else {
                    if (qb && !qz)
                        q_incr(x);
                }
                break;
            }
          case SC_TRN_ZERO: // truncation to zero
            {
                break;
            }
          case SC_RND_INF: // rounding to infinity
            {
                if (qb)
                    q_incr(x);
                break;
            }
          case SC_RND_CONV: // convergent rounding
            {
                if ((qb && !qz) || (qb && qz && q_odd(x)))
                    q_incr(x);
                break;
            }
          case SC_RND_ZERO: // rounding to zero
            {
                if (qb && !qz)
                    q_incr(x);
                break;
            }
          case SC_RND_MIN_INF: // rounding to minus infinity
            {
                if (!is_neg()) {
                    if (qb && !qz)
                        q_incr(x);
                } else {
                    if (qb)
                        q_incr(x);
                }
                break;
            }
          default:
            ;
        }
        q_clear(x);

        find_sw();
    }
}


// ----------------------------------------------------------------------------
//  PRIVATE METHOD : overflow
//
//  Performs destructive overflow handling.
// ----------------------------------------------------------------------------

void
scfx_rep::overflow(const scfx_params &params, bool &o_flag)
{
    scfx_index x = calc_indices(params.iwl() - 1);

    if (x.wi() >= size())
        resize_to(x.wi() + 1, 1);

    if (x.wi() < 0) {
        resize_to(size() - x.wi(), -1);
        x.wi(0);
    }

    bool zero_left = o_zero_left(x);
    bool bit_at = o_bit_at(x);
    bool zero_right = o_zero_right(x);

    bool under = false;
    bool over = false;

    sc_enc enc = params.enc();

    if (enc == SC_TC_) {
        if (is_neg()) {
            if (params.o_mode() == SC_SAT_SYM)
                under = (!zero_left || bit_at);
            else
                under = (!zero_left || (zero_left && bit_at && ! zero_right));
        } else {
            over = (! zero_left || bit_at);
        }
    } else {
        if (is_neg())
            under = (!is_zero());
        else
            over = (!zero_left);
    }

    o_flag = (under || over);

    if (o_flag) {
        scfx_index x2 = calc_indices(params.iwl() - params.wl());

        if (x2.wi() < 0) {
            resize_to(size() - x2.wi(), -1);
            x.wi(x.wi() - x2.wi());
            x2.wi(0);
        }

        switch (params.o_mode()) {
          case SC_WRAP: // wrap-around
            {
                int n_bits = params.n_bits();

                if (n_bits == 0) {
                    // wrap-around all 'wl' bits
                    toggle_tc();
                    o_extend(x, enc);
                    toggle_tc();
                } else if (n_bits < params.wl()) {
                    scfx_index x3 = calc_indices(params.iwl() - 1 - n_bits);

                    // wrap-around least significant 'wl - n_bits' bits;
                    // saturate most significant 'n_bits' bits
                    toggle_tc();
                    o_set(x, x3, enc, under);
                    o_extend(x, enc);
                    toggle_tc();
                } else {
                    // saturate all 'wl' bits
                    if (under)
                        o_set_low(x, enc);
                    else
                        o_set_high(x, x2, enc);
                }
                break;
            }
          case SC_SAT: // saturation
            {
                if (under)
                    o_set_low(x, enc);
                else
                    o_set_high(x, x2, enc);
                break;
            }
          case SC_SAT_SYM: // symmetrical saturation
            {
                if (under) {
                    if (enc == SC_TC_)
                        o_set_high(x, x2, SC_TC_, -1);
                    else
                        o_set_low(x, SC_US_);
                } else {
                    o_set_high(x, x2, enc);
                }
                break;
            }
          case SC_SAT_ZERO: // saturation to zero
            {
                set_zero();
                break;
            }
          case SC_WRAP_SM: // sign magnitude wrap-around
            {
                SC_ERROR_IF_(enc == SC_US_,
                             sc_core::SC_ID_WRAP_SM_NOT_DEFINED_);

                int n_bits = params.n_bits();

                if (n_bits == 0) {
                    scfx_index x4 = calc_indices(params.iwl());

                    if (x4.wi() >= size())
                        resize_to(x4.wi() + 1, 1);

                    toggle_tc();
                    if (o_bit_at(x4) != o_bit_at(x))
                        o_invert(x2);
                    o_extend(x, SC_TC_);
                    toggle_tc();
                } else if (n_bits == 1) {
                    toggle_tc();
                    if (is_neg() != o_bit_at(x))
                        o_invert(x2);
                    o_extend(x, SC_TC_);
                    toggle_tc();
                } else if (n_bits < params.wl()) {
                    scfx_index x3 = calc_indices(params.iwl() - 1 - n_bits);
                    scfx_index x4 = calc_indices(params.iwl() - n_bits);

                    // wrap-around least significant 'wl - n_bits' bits;
                    // saturate most significant 'n_bits' bits
                    toggle_tc();
                    if (is_neg() == o_bit_at(x4))
                        o_invert(x2);
                    o_set(x, x3, SC_TC_, under);
                    o_extend(x, SC_TC_);
                    toggle_tc();
                } else {
                    if (under)
                        o_set_low(x, SC_TC_);
                    else
                        o_set_high(x, x2, SC_TC_);
                }
                break;
            }
          default:
            ;
        }

        find_sw();
    }
}


// ----------------------------------------------------------------------------
//  PUBLIC METHOD : cast
//
//  Performs a destructive cast operation on a scfx_rep.
// ----------------------------------------------------------------------------

void
scfx_rep::cast(const scfx_params &params, bool &q_flag, bool &o_flag)
{
    q_flag = false;
    o_flag = false;

    // check for special cases
    if (is_zero()) {
        if (is_neg())
            m_sign = 1;
        return;
    }

    // perform casting
    quantization(params, q_flag);
    overflow(params, o_flag);

    // check for special case: -0
    if (is_zero() && is_neg())
        m_sign = 1;
}


// ----------------------------------------------------------------------------
//  make sure, the two mantissas are aligned
// ----------------------------------------------------------------------------

void
align(const scfx_rep &lhs, const scfx_rep &rhs, int &new_wp,
      int &len_mant, scfx_mant_ref &lhs_mant, scfx_mant_ref &rhs_mant)
{
    bool need_lhs = true;
    bool need_rhs = true;

    if (lhs.m_wp != rhs.m_wp || lhs.size() != rhs.size()) {
        int lower_bound_lhs = lhs.m_lsw - lhs.m_wp;
        int upper_bound_lhs = lhs.m_msw - lhs.m_wp;
        int lower_bound_rhs = rhs.m_lsw - rhs.m_wp;
        int upper_bound_rhs = rhs.m_msw - rhs.m_wp;

        int lower_bound = sc_min(lower_bound_lhs, lower_bound_rhs);
        int upper_bound = sc_max(upper_bound_lhs, upper_bound_rhs);

        new_wp = -lower_bound;
        len_mant = sc_max(min_mant, upper_bound - lower_bound + 1);

        if (new_wp != lhs.m_wp || len_mant != lhs.size()) {
            lhs_mant = lhs.resize(len_mant, new_wp);
            need_lhs = false;
        }

        if (new_wp != rhs.m_wp || len_mant != rhs.size()) {
            rhs_mant = rhs.resize(len_mant, new_wp);
            need_rhs = false;
        }
    }

    if (need_lhs) {
        lhs_mant = lhs.m_mant;
    }

    if (need_rhs) {
        rhs_mant = rhs.m_mant;
    }
}


// ----------------------------------------------------------------------------
//  compare two mantissas
// ----------------------------------------------------------------------------

int
compare_msw_ff(const scfx_rep &lhs, const scfx_rep &rhs)
{
    // special case: rhs.m_mant[rhs.m_msw + 1] == 1
    if (rhs.m_msw < rhs.size() - 1 && rhs.m_mant[rhs.m_msw + 1 ] != 0) {
        return -1;
    }

    int lhs_size = lhs.m_msw - lhs.m_lsw + 1;
    int rhs_size = rhs.m_msw - rhs.m_lsw + 1;

    int size = sc_min(lhs_size, rhs_size);

    int lhs_index = lhs.m_msw;
    int rhs_index = rhs.m_msw;

    int i;

    for (i = 0;
         i < size && lhs.m_mant[lhs_index] == rhs.m_mant[rhs_index];
         i++) {
        lhs_index--;
        rhs_index--;
    }

    if (i == size) {
        if (lhs_size == rhs_size) {
            return 0;
        }

        if (lhs_size < rhs_size) {
            return -1;
        } else {
            return 1;
        }
    }

    if (lhs.m_mant[lhs_index] < rhs.m_mant[rhs_index]) {
        return -1;
    } else {
        return 1;
    }
}


// ----------------------------------------------------------------------------
//  divide the mantissa by ten
// ----------------------------------------------------------------------------

unsigned int
scfx_rep::divide_by_ten()
{
#if defined(SC_BOOST_BIG_ENDIAN)
    half_word *hw = (half_word *)&m_mant[m_msw];
#elif defined(SC_BOOST_LITTLE_ENDIAN)
    half_word *hw = ((half_word *)&m_mant[m_msw]) + 1;
#endif

    unsigned int remainder = 0;

    word_short ls;
    ls.l = 0;

#if defined(SC_BOOST_BIG_ENDIAN)
    for (int i = 0, end = (m_msw - m_wp + 1) * 2; i < end; i++) {
#elif defined(SC_BOOST_LITTLE_ENDIAN)
    for (int i = 0, end = -(m_msw - m_wp + 1) * 2; i > end; i--) {
#endif
        ls.s.u = static_cast<half_word>(remainder);
        ls.s.l = hw[i];
        remainder = ls.l % 10;
        ls.l /= 10;
        hw[i] = ls.s.l;
    }

    return remainder;
}


// ----------------------------------------------------------------------------
//  multiply the mantissa by ten
// ----------------------------------------------------------------------------

void
scfx_rep::multiply_by_ten()
{
    int size = m_mant.size() + 1;

    scfx_mant mant8(size);
    scfx_mant mant2(size);

    size--;

    mant8[size] = (m_mant[size - 1] >> (bits_in_word - 3));
    mant2[size] = (m_mant[size - 1] >> (bits_in_word - 1));

    while (--size) {
        mant8[size] = (m_mant[size] << 3) |
                      (m_mant[size - 1] >> (bits_in_word - 3));
        mant2[size] = (m_mant[size] << 1) |
                      (m_mant[size - 1] >> (bits_in_word - 1));
    }

    mant8[0] = (m_mant[0] << 3);
    mant2[0] = (m_mant[0] << 1);

    add_mants(m_mant.size(), m_mant, mant8, mant2);
}


// ----------------------------------------------------------------------------
//  normalize
// ----------------------------------------------------------------------------

void
scfx_rep::normalize(int exponent)
{
    int shift = exponent % bits_in_word;
    if (shift < 0) {
        shift += bits_in_word;
    }

    if (shift) {
        shift_left(shift);
    }

    find_sw();

    m_wp = (shift - exponent) / bits_in_word;
}


// ----------------------------------------------------------------------------
//  return a new mantissa that is aligned and resized
// ----------------------------------------------------------------------------

scfx_mant *
scfx_rep::resize(int new_size, int new_wp) const
{
    scfx_mant *result = new scfx_mant(new_size);

    result->clear();

    int shift = new_wp - m_wp;

    for (int j = m_lsw; j <= m_msw; j++) {
        (*result)[j + shift] = m_mant[j];
    }

    return result;
}


// ----------------------------------------------------------------------------
//  set a single bit
// ----------------------------------------------------------------------------

void
scfx_rep::set_bin(int i)
{
    m_mant[i >> 5] |= 1 << (i & 31);
}


// ----------------------------------------------------------------------------
//  set three bits
// ----------------------------------------------------------------------------

void
scfx_rep::set_oct(int i, int n)
{
    if (n & 1) {
        m_mant[i >> 5] |= 1 << (i & 31);
    }
    i++;
    if (n & 2) {
        m_mant[i >> 5] |= 1 << (i & 31);
    }
    i++;
    if (n & 4) {
        m_mant[i >> 5] |= 1 << (i & 31);
    }
}


// ----------------------------------------------------------------------------
//  set four bits
// ----------------------------------------------------------------------------

void
scfx_rep::set_hex(int i, int n)
{
    if (n & 1) {
        m_mant[i >> 5] |= 1 << (i & 31);
    }
    i++;
    if (n & 2) {
        m_mant[i >> 5] |= 1 << (i & 31);
    }
    i++;
    if (n & 4) {
        m_mant[i >> 5] |= 1 << (i & 31);
    }
    i++;
    if (n & 8) {
        m_mant[i >> 5] |= 1 << (i & 31);
    }
}


// ----------------------------------------------------------------------------
//  PRIVATE METHOD : shift_left
//
//  Shifts a scfx_rep to the left by a MAXIMUM of bits_in_word - 1 bits.
// ----------------------------------------------------------------------------

void
scfx_rep::shift_left(int n)
{
    if (n != 0) {
        int shift_left = n;
        int shift_right = bits_in_word - n;

        SC_ASSERT_(!(m_mant[size() - 1] >> shift_right),
                   "shift_left overflow");

        for (int i = size() - 1; i > 0; i--) {
            m_mant[i] = (m_mant[i] << shift_left) |
                        (m_mant[i - 1] >> shift_right);
        }
        m_mant[0] <<= shift_left;
    }
}


// ----------------------------------------------------------------------------
//  PRIVATE METHOD : shift_right
//
//  Shifts a scfx_rep to the right by a MAXIMUM of bits_in_word - 1 bits.
// ----------------------------------------------------------------------------

void
scfx_rep::shift_right(int n)
{
    if (n != 0) {
        int shift_left = bits_in_word - n;
        int shift_right = n;

        SC_ASSERT_(!(m_mant[0] << shift_left), "shift_right overflow");

        for (int i = 0; i < size() - 1; i++) {
            m_mant[i] = (m_mant[i] >> shift_right) |
                        (m_mant[i + 1] << shift_left);
        }
        m_mant[size() - 1] >>= shift_right;
    }
}


// ----------------------------------------------------------------------------
//  METHOD : get_bit
//
//  Tests a bit, in two's complement.
// ----------------------------------------------------------------------------

bool
scfx_rep::get_bit(int i) const
{
    if (!is_normal())
        return false;

    scfx_index x = calc_indices(i);

    if (x.wi() >= size())
        return is_neg();

    if (x.wi() < 0)
        return false;

    const_cast<scfx_rep*>(this)->toggle_tc();

    bool result = (m_mant[x.wi()] & (1 << x.bi())) != 0;

    const_cast<scfx_rep *>(this)->toggle_tc();

    return result;
}


// ----------------------------------------------------------------------------
//  METHOD : set
//
//  Sets a bit, in two's complement, between iwl-1 and -fwl.
// ----------------------------------------------------------------------------

bool
scfx_rep::set(int i, const scfx_params &params)
{
    if (!is_normal())
        return false;

    scfx_index x = calc_indices(i);

    if (x.wi() >= size()) {
        if (is_neg())
            return true;
        else
            resize_to(x.wi() + 1, 1);
    } else if (x.wi() < 0) {
        resize_to(size() - x.wi(), -1);
        x.wi(0);
    }

    toggle_tc();

    m_mant[x.wi()] |= 1 << x.bi();

    if (i == params.iwl() - 1)
        o_extend(x, params.enc()); // sign extension

    toggle_tc();

    find_sw();

    return true;
}


// ----------------------------------------------------------------------------
//  METHOD : clear
//
//  Clears a bit, in two's complement, between iwl-1 and -fwl.
// ----------------------------------------------------------------------------

bool
scfx_rep::clear(int i, const scfx_params &params)
{
    if (!is_normal())
        return false;

    scfx_index x = calc_indices(i);

    if (x.wi() >= size()) {
        if (!is_neg())
            return true;
        else
            resize_to(x.wi() + 1, 1);
    } else if (x.wi() < 0) {
        return true;
    }

    toggle_tc();

    m_mant[x.wi()] &= ~(1 << x.bi());

    if (i == params.iwl() - 1)
        o_extend(x, params.enc()); // sign extension

    toggle_tc();

    find_sw();

    return true;
}


// ----------------------------------------------------------------------------
//  METHOD : get_slice
// ----------------------------------------------------------------------------

bool
scfx_rep::get_slice(int i, int j, const scfx_params &, sc_bv_base &bv) const
{
    if (is_nan() || is_inf())
        return false;

    // get the bits

    int l = j;
    for (int k = 0; k < bv.length(); ++k) {
        bv[k] = get_bit(l);

        if (i >= j)
            ++l;
        else
            --l;
    }

    return true;
}

bool
scfx_rep::set_slice(int i, int j, const scfx_params &params,
                    const sc_bv_base &bv)
{
    if (is_nan() || is_inf())
        return false;

    // set the bits
    int l = j;
    for (int k = 0; k < bv.length(); ++k) {
        if (bv[k].to_bool())
            set(l, params);
        else
            clear(l, params);

        if (i >= j)
            ++l;
        else
            --l;
    }

    return true;
}


// ----------------------------------------------------------------------------
//  METHOD : print
// ----------------------------------------------------------------------------

void
scfx_rep::print(::std::ostream &os) const
{
    os << to_string(SC_DEC, -1, SC_E);
}


// ----------------------------------------------------------------------------
//  METHOD : dump
// ----------------------------------------------------------------------------

void
scfx_rep::dump(::std::ostream &os) const
{
    os << "scfx_rep" << ::std::endl;
    os << "(" << ::std::endl;

    os << "mant  =" << ::std::endl;
    for (int i = size() - 1; i >= 0; i--) {
        char buf[BUFSIZ];
        std::sprintf(buf, " %d: %10u (%8x)", i,
                     (int)m_mant[i], (int)m_mant[i]);
        os << buf << ::std::endl;
    }

    os << "wp = " << m_wp << ::std::endl;
    os << "sign = " << m_sign << ::std::endl;

    os << "state = ";
    switch (m_state) {
      case normal:
        os << "normal";
        break;
      case infinity:
        os << "infinity";
        break;
      case not_a_number:
        os << "not_a_number";
        break;
      default:
        os << "unknown";
    }
    os << ::std::endl;

    os << "msw = " << m_msw << ::std::endl;
    os << "lsw = " << m_lsw << ::std::endl;

    os << ")" << ::std::endl;
}


// ----------------------------------------------------------------------------
//  METHOD : get_type
// ----------------------------------------------------------------------------

void
scfx_rep::get_type(int &wl, int &iwl, sc_enc &enc) const
{
    if (is_nan() || is_inf()) {
        wl  = 0;
        iwl = 0;
        enc = SC_TC_;
        return;
    }

    if (is_zero()) {
        wl  = 1;
        iwl = 1;
        enc = SC_US_;
        return;
    }

    int msb = (m_msw - m_wp) * bits_in_word +
              scfx_find_msb(m_mant[ m_msw ]) + 1;
    while (get_bit(msb) == get_bit(msb - 1)) {
        --msb;
    }

    int lsb = (m_lsw - m_wp) * bits_in_word +
              scfx_find_lsb(m_mant[m_lsw]);

    if (is_neg()) {
        wl  = msb - lsb + 1;
        iwl = msb + 1;
        enc = SC_TC_;
    } else {
        wl  = msb - lsb;
        iwl = msb;
        enc = SC_US_;
    }
}


// ----------------------------------------------------------------------------
//  PRIVATE METHOD : round
//
//  Performs convergent rounding (rounding to even) as in floating-point.
// ----------------------------------------------------------------------------

void
scfx_rep::round(int wl)
{
    // check for special cases

    if (is_nan() || is_inf() || is_zero())
        return;

    // estimate effective wordlength and compare
    int wl_effective;
    wl_effective = (m_msw - m_lsw + 1) * bits_in_word;
    if (wl_effective <= wl)
        return;

    // calculate effective wordlength and compare
    int msb = scfx_find_msb(m_mant[m_msw]);
    int lsb = scfx_find_lsb(m_mant[m_lsw]);
    wl_effective = (m_msw * bits_in_word + msb) -
                   (m_lsw * bits_in_word + lsb) + 1;
    if (wl_effective <= wl)
        return;

    // perform rounding
    int wi = m_msw - (wl - 1) / bits_in_word;
    int bi = msb - (wl - 1) % bits_in_word;
    if (bi < 0) {
        --wi;
        bi += bits_in_word;
    }

    scfx_index x(wi, bi);

    if ((q_bit(x) && ! q_zero(x)) || (q_bit(x) && q_zero(x) && q_odd(x))) {
        q_incr(x);
    }
    q_clear(x);

    find_sw();

    m_r_flag = true;
}

} // namespace sc_dt

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

  sc_fxval.cpp -

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: sc_fxval.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdlib>

#include "systemc/ext/dt/fx/sc_fxval.hh"
#include "systemc/ext/utils/messages.hh"

namespace sc_dt
{

// ----------------------------------------------------------------------------
//  CLASS : sc_fxval
//
//  Fixed-point value type; arbitrary precision.
// ----------------------------------------------------------------------------

// explicit conversion to character string

const std::string
sc_fxval::to_string() const
{
    return std::string(m_rep->to_string(SC_DEC, -1, SC_E));
}

const std::string
sc_fxval::to_string(sc_numrep numrep) const
{
    return std::string(m_rep->to_string(numrep, -1, SC_E));
}

const std::string
sc_fxval::to_string(sc_numrep numrep, bool w_prefix) const
{
    return std::string(m_rep->to_string(numrep, (w_prefix ? 1 : 0), SC_E));
}

const std::string
sc_fxval::to_string(sc_fmt fmt) const
{
    return std::string(m_rep->to_string(SC_DEC, -1, fmt));
}

const std::string
sc_fxval::to_string(sc_numrep numrep, sc_fmt fmt) const
{
    return std::string(m_rep->to_string(numrep, -1, fmt));
}

const std::string
sc_fxval::to_string(sc_numrep numrep, bool w_prefix, sc_fmt fmt) const
{
    return std::string(m_rep->to_string(numrep, (w_prefix ? 1 : 0), fmt));
}


const std::string
sc_fxval::to_dec() const
{
    return std::string(m_rep->to_string(SC_DEC, -1, SC_E));
}

const std::string
sc_fxval::to_bin() const
{
    return std::string(m_rep->to_string(SC_BIN, -1, SC_E));
}

const std::string
sc_fxval::to_oct() const
{
    return std::string(m_rep->to_string(SC_OCT, -1, SC_E));
}

const std::string
sc_fxval::to_hex() const
{
    return std::string(m_rep->to_string(SC_HEX, -1, SC_E));
}


// print or dump content

void sc_fxval::print(::std::ostream &os) const { m_rep->print(os); }

void
sc_fxval::scan(::std::istream &is)
{
    std::string s;
    is >> s;
    *this = s.c_str();
}

void
sc_fxval::dump(::std::ostream &os) const
{
    os << "sc_fxval" << ::std::endl;
    os << "(" << ::std::endl;
    os << "rep = ";
    m_rep->dump(os);
    // TO BE COMPLETED
    // os << "r_flag   = " << m_r_flag << ::std::endl;
    // os << "observer = ";
    // if (m_observer != 0)
    //     m_observer->dump(os);
    // else
    //     os << "0" << ::std::endl;
    os << ")" << ::std::endl;
}

// protected methods and friend functions
sc_fxval_observer *
sc_fxval::lock_observer() const
{
    SC_ASSERT_(m_observer != 0, "lock observer failed");
    sc_fxval_observer *tmp = m_observer;
    m_observer = 0;
    return tmp;
}

void
sc_fxval::unlock_observer(sc_fxval_observer *observer_) const
{
    SC_ASSERT_(observer_ != 0, "unlock observer failed");
    m_observer = observer_;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_fxval_fast
//
//  Fixed-point value types; limited precision.
// ----------------------------------------------------------------------------

static void
print_dec(scfx_string &s, scfx_ieee_double id, int w_prefix, sc_fmt fmt)
{
    if (id.negative() != 0) {
        id.negative(0);
        s += '-';
    }

    if (w_prefix == 1) {
        scfx_print_prefix(s, SC_DEC);
    }

    if (id.is_zero()) {
        s += '0';
        return;
    }

    // split 'id' into its integer and fractional part
    double int_part;
    double frac_part = std::modf(static_cast<double>(id), &int_part);

    int i;

    // print integer part
    int int_digits = 0;
    int int_zeros  = 0;

    if (int_part != 0.0) {
        int_digits = (int)std::ceil(std::log10(int_part + 1.0));

        int len = s.length();
        s.append(int_digits);

        bool zero_digits = (frac_part == 0.0 && fmt != SC_F);

        for (i = int_digits + len - 1; i >= len; i--) {
            unsigned int remainder = (unsigned int)std::fmod(int_part, 10.0);
            s[i] = static_cast<char>('0' + remainder);

            if (zero_digits) {
                if (remainder == 0)
                    int_zeros++;
                else
                    zero_digits = false;
            }

            int_part /= 10.0;
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

    if (frac_part != 0.0) {
        s += '.';

        bool zero_digits = (int_digits == 0 && fmt != SC_F);

        frac_zeros = (int)std::floor(-std::log10(frac_part + DBL_EPSILON));

        frac_part *= std::pow(10.0, frac_zeros);

        frac_digits = frac_zeros;
        if (!zero_digits) {
            for (i = 0; i < frac_zeros; i++)
                s += '0';
            frac_zeros = 0;
        }

        while (frac_part != 0.0) {
            frac_part *= 10.0;
            int n = static_cast<int>(frac_part);

            if (zero_digits) {
                if (n == 0)
                    frac_zeros++;
                else
                    zero_digits = false;
            }

            if (!zero_digits)
                s += static_cast<char>('0' + n);

            frac_part -= n;
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

static void
print_other(scfx_string &s, const scfx_ieee_double &id, sc_numrep numrep,
             int w_prefix, sc_fmt fmt, const scfx_params *params)
{
    scfx_ieee_double id2 = id;

    sc_numrep numrep2 = numrep;

    bool numrep_is_sm = (numrep == SC_BIN_SM ||
                         numrep == SC_OCT_SM ||
                         numrep == SC_HEX_SM);

    if (numrep_is_sm) {
        if (id2.negative() != 0) {
            s += '-';
            id2.negative(0);
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

    sc_fxval_fast a(id2);

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
        if (a.is_zero()) {
            msb = 0;
            lsb = 0;
        } else {
            msb = id2.exponent() + 1;
            while (a.get_bit(msb) == a.get_bit(msb - 1))
                --msb;

            if (numrep == SC_BIN_US ||
                numrep == SC_OCT_US ||
                numrep == SC_HEX_US) {
                --msb;
            }

            lsb = id2.exponent() - 52;
            while (!a.get_bit(lsb))
                ++lsb;
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
            int sign = (id2.negative() != 0) ? (1 << step) - 1 : 0;
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
            value += static_cast<int>(a.get_bit(i)) << j;
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
to_string(const scfx_ieee_double &id, sc_numrep numrep, int w_prefix,
          sc_fmt fmt, const scfx_params *params=0)
{
    static scfx_string s;

    s.clear();

    if (id.is_nan()) {
        scfx_print_nan(s);
    } else if (id.is_inf()) {
        scfx_print_inf(s, static_cast<bool>(id.negative()));
    } else if (id.negative() && !id.is_zero() &&
             (numrep == SC_BIN_US ||
               numrep == SC_OCT_US ||
               numrep == SC_HEX_US)) {
        s += "negative";
    } else if (numrep == SC_DEC) {
        sc_dt::print_dec(s, id, w_prefix, fmt);
    } else {
        sc_dt::print_other(s, id, numrep, w_prefix, fmt, params);
    }

    return s;
}


// explicit conversion to character string
const std::string
sc_fxval_fast::to_string() const
{
    return std::string(sc_dt::to_string(m_val, SC_DEC, -1, SC_E));
}

const std::string
sc_fxval_fast::to_string(sc_numrep numrep) const
{
    return std::string(sc_dt::to_string(m_val, numrep, -1, SC_E));
}

const std::string
sc_fxval_fast::to_string(sc_numrep numrep, bool w_prefix) const
{
    return std::string(sc_dt::to_string(m_val, numrep, (w_prefix ? 1 : 0),
                       SC_E));
}

const std::string
sc_fxval_fast::to_string(sc_fmt fmt) const
{
    return std::string(sc_dt::to_string(m_val, SC_DEC, -1, fmt));
}

const std::string
sc_fxval_fast::to_string(sc_numrep numrep, sc_fmt fmt) const
{
    return std::string(sc_dt::to_string(m_val, numrep, -1, fmt));
}

const std::string
sc_fxval_fast::to_string(sc_numrep numrep, bool w_prefix, sc_fmt fmt) const
{
    return std::string(sc_dt::to_string(m_val, numrep, (w_prefix ? 1 : 0),
                                        fmt));
}

const std::string
sc_fxval_fast::to_dec() const
{
    return std::string(sc_dt::to_string(m_val, SC_DEC, -1, SC_E));
}

const std::string
sc_fxval_fast::to_bin() const
{
    return std::string(sc_dt::to_string(m_val, SC_BIN, -1, SC_E));
}

const std::string
sc_fxval_fast::to_oct() const
{
    return std::string(sc_dt::to_string(m_val, SC_OCT, -1, SC_E));
}

const std::string
sc_fxval_fast::to_hex() const
{
    return std::string(sc_dt::to_string(m_val, SC_HEX, -1, SC_E));
}


// print or dump content

void
sc_fxval_fast::print(::std::ostream &os) const
{
    os << sc_dt::to_string(m_val, SC_DEC, -1, SC_E);
}

void
sc_fxval_fast::scan(::std::istream &is)
{
    std::string s;
    is >> s;
    *this = s.c_str();
}

void
sc_fxval_fast::dump(::std::ostream &os) const
{
    os << "sc_fxval_fast" << ::std::endl;
    os << "(" << ::std::endl;
    os << "val = " << m_val << ::std::endl;
    // TO BE COMPLETED
    // os << "r_flag   = " << m_r_flag << ::std::endl;
    // os << "observer = ";
    // if (m_observer != 0)
    //     m_observer->dump(os);
    // else
    //     os << "0" << ::std::endl;
    os << ")" << ::std::endl;
}


// internal use only;
bool
sc_fxval_fast::get_bit(int i) const
{
    scfx_ieee_double id(m_val);
    if (id.is_zero() || id.is_nan() || id.is_inf())
        return false;

    // convert to two's complement
    unsigned int m0 = id.mantissa0();
    unsigned int m1 = id.mantissa1();

    if (id.is_normal())
        m0 += 1U << 20;

    if (id.negative() != 0) {
        m0 = ~ m0;
        m1 = ~ m1;
        unsigned int tmp = m1;
        m1 += 1U;
        if (m1 <= tmp)
            m0 += 1U;
    }

    // get the right bit
    int j = i - id.exponent();
    if ((j += 20) >= 32)
        return ((m0 & 1U << 31) != 0);
    else if (j >= 0)
        return ((m0 & 1U << j) != 0);
    else if ((j += 32) >= 0)
        return ((m1 & 1U << j) != 0);
    else
        return false;
}


// protected methods and friend functions
sc_fxval_fast_observer *
sc_fxval_fast::lock_observer() const
{
    SC_ASSERT_(m_observer != 0, "lock observer failed");
    sc_fxval_fast_observer *tmp = m_observer;
    m_observer = 0;
    return tmp;
}

void
sc_fxval_fast::unlock_observer(sc_fxval_fast_observer *observer_) const
{
    SC_ASSERT_(observer_ != 0, "unlock observer failed");
    m_observer = observer_;
}

#define SCFX_FAIL_IF_(cnd)                                                    \
{                                                                             \
    if ((cnd))                                                             \
        return static_cast<double>(scfx_ieee_double::nan());                \
}

double
sc_fxval_fast::from_string(const char *s)
{
    SCFX_FAIL_IF_(s == 0 || *s == 0);

    scfx_string s2;
    s2 += s;
    s2 += '\0';

    bool sign_char;
    int sign = scfx_parse_sign(s, sign_char);

    sc_numrep numrep = scfx_parse_prefix(s);

    int base = 0;

    switch (numrep) {
      case SC_DEC:
        {
            base = 10;
            if (scfx_is_nan(s)) // special case: NaN
                return static_cast<double>(scfx_ieee_double::nan());
            if (scfx_is_inf(s)) // special case: Infinity
                return static_cast<double>(scfx_ieee_double::inf(sign));
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
            s = (const char*) s2 + 4;
            numrep = SC_BIN;
            break;
        }
      default:;// Martin, what is default???
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

        end++;
    }

    SCFX_FAIL_IF_(int_digits == 0 && frac_digits == 0);

    // [ exponent ]
    int exponent = 0;
    if (*end) {
        for (const char *e = end + 2; *e; e++)
            SCFX_FAIL_IF_(!scfx_is_digit(*e, SC_DEC));
        exponent = std::atoi(end + 1);
    }

    //
    // convert the mantissa
    //
    double integer = 0.0;
    if (int_digits != 0) {
        bool first_digit = true;

        for (; s < end; s++) {
            if (*s == '.')
                break;

            if (first_digit) {
                integer = scfx_to_digit(*s, numrep);
                switch (numrep) {
                  case SC_BIN:
                  case SC_OCT:
                  case SC_HEX:
                    {
                        if (integer >= (base >> 1))
                            integer -= base; // two's complement
                        break;
                    }
                  default:
                    ;
                }
                first_digit = false;
            } else {
                integer *= base;
                integer += scfx_to_digit(*s, numrep);
            }
        }
    }

    // [ . fraction ]
    double fraction = 0.0;
    if (frac_digits != 0) {
        s++;  // skip '.'

        bool first_digit = (int_digits == 0);
        double scale = 1.0;
        for (; s < end; s++) {
            scale /= base;

            if (first_digit) {
                fraction = scfx_to_digit(*s, numrep);
                switch (numrep) {
                  case SC_BIN:
                  case SC_OCT:
                  case SC_HEX:
                    {
                        if (fraction >= (base >> 1))
                            fraction -= base; // two's complement
                        break;
                    }
                  default:
                    ;
                }
                fraction *= scale;
                first_digit = false;
            } else {
                fraction += scfx_to_digit(*s, numrep) * scale;
            }
        }
    }

    double exp =
        (exponent != 0) ? std::pow((double) base, (double) exponent) : 1;

    return (sign * (integer + fraction) * exp);
}

#undef SCFX_FAIL_IF_

} // namespace sc_dt

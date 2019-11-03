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

  sc_bv_base.cpp -- Arbitrary size bit vector class.

  Original Author: Gene Bushuyev, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: sc_bv_base.cpp,v $
// Revision 1.2  2011/08/24 22:05:40  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.4  2006/04/11 23:12:26  acg
//   Andy Goodrich: Fixed bug in parsing of extended string constants like
//   0bus1110011.
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include <cstring>
#include <sstream>

#include "systemc/ext/dt/bit/messages.hh"
#include "systemc/ext/dt/bit/sc_bv_base.hh"
#include "systemc/ext/dt/fx/sc_fix.hh"
#include "systemc/ext/dt/fx/sc_ufix.hh"
#include "systemc/ext/utils/sc_report.hh"

namespace sc_dt
{

// ----------------------------------------------------------------------------
//  CLASS : sc_bv_base
//
//  Arbitrary size bit vector base class.
// ----------------------------------------------------------------------------

void
sc_bv_base::init(int length_, bool init_value)
{
    // check the length
    if (length_ <= 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_ZERO_LENGTH_, 0);
        sc_core::sc_abort(); // can't recover from here
    }
    // allocate memory for the data and control words
    m_len = length_;
    m_size = (m_len - 1) / SC_DIGIT_SIZE + 1;
    m_data = new sc_digit[m_size];
    // initialize the bits to 'init_value'
    sc_digit dw = init_value ? ~SC_DIGIT_ZERO : SC_DIGIT_ZERO;
    int sz = m_size;
    for (int i = 0; i < sz; ++i) {
        m_data[i] = dw;
    }
    clean_tail();
}

void
sc_bv_base::assign_from_string(const std::string &s)
{
    // s must have been converted to bin
    int len = m_len;
    int s_len = s.length() - 1;
    int min_len = sc_min(len, s_len);
    int i = 0;
    for (; i < min_len; ++i) {
        char c = s[s_len - i - 1];
        if (c != '0' && c != '1') {
            SC_REPORT_ERROR(sc_core::SC_ID_CANNOT_CONVERT_,
                "string can contain only '0' and '1' characters");
            // may continue, if suppressed
            c = '0';
        }
        set_bit(i, sc_logic_value_t(c - '0'));
    }
    // if formatted, fill the rest with sign(s), otherwise fill with zeros
    sc_logic_value_t fill = (s[s_len] == 'F' ? sc_logic_value_t(s[0] - '0')
                                             : sc_logic_value_t(0));
    for (; i < len; ++i) {
        set_bit(i, fill);
    }
}

// constructors
sc_bv_base::sc_bv_base(const char *a) : m_len(0), m_size(0), m_data(0)
{
    std::string s = convert_to_bin(a);
    init(s.length() - 1);
    assign_from_string(s);
}

sc_bv_base::sc_bv_base(const char *a, int length_) :
    m_len(0), m_size(0), m_data(0)
{
    init(length_);
    assign_from_string(convert_to_bin(a));
}

sc_bv_base::sc_bv_base(const sc_bv_base &a) :
    sc_proxy<sc_bv_base>(), m_len(a.m_len), m_size(a.m_size),
    m_data(new sc_digit[m_size])
{
    // copy the bits
    int sz = m_size;
    for (int i = 0; i < sz; ++i) {
        m_data[i] = a.m_data[i];
    }
}

// assignment operators
sc_bv_base &
sc_bv_base::operator = (const char *a)
{
    assign_from_string(convert_to_bin(a));
    return *this;
}


// ----------------------------------------------------------------------------
// convert formatted string to binary string

const std::string
convert_to_bin(const char *s)
{
    // Beware: logic character strings cannot start with '0x' or '0X',
    //         because this is seen as a hexadecimal encoding prefix!

    if (s == 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_CANNOT_CONVERT_,
                "character string is zero");
        return std::string();
    }
    if (*s == 0) {
        SC_REPORT_ERROR(sc_core::SC_ID_CANNOT_CONVERT_,
            "character string is empty");
        return std::string();
    }

    int n = strlen(s);
    int i = 0;
    if (s[0] == '-' || s[0] == '+') {
        ++i;
    }
    if (n > (i + 2) && s[i] == '0') {
        if (s[i + 1] == 'b' || s[i + 1] == 'B') {
            if (s[i + 2] == '0' || s[i + 2] == '1') {
                std::string str(&s[2]);
                str += "F";
                return str;
            }
        }
        if (s[i + 1] == 'b' || s[i + 1] == 'B' ||
            s[i + 1] == 'c' || s[i + 1] == 'C' ||
            s[i + 1] == 'd' || s[i + 1] == 'D' ||
            s[i + 1] == 'o' || s[i + 1] == 'O' ||
            s[i + 1] == 'x' || s[i + 1] == 'X') {
            try {
                // worst case length = n * 4
                sc_fix a(s, n * 4, n * 4, SC_TRN, SC_WRAP, 0, SC_ON);
                std::string str = a.to_bin();
                str += "F"; // mark the string as formatted
                // get rid of prefix (0b) and redundant leading bits
                const char *p = str.c_str() + 2;
                while (p[1] && p[0] == p[1]) {
                    ++p;
                }
                return std::string(p);
            } catch (const sc_core::sc_report &) {
                std::stringstream msg;
                msg << "character string '" << s << "' is not valid";
                SC_REPORT_ERROR(sc_core::SC_ID_CANNOT_CONVERT_,
                        msg.str().c_str());
                return std::string();
            }
        }
    }

    // bin by default
    std::string str(s);
    str += "U"; // mark the string as unformatted
    return str;
}

// convert binary string to formatted string
const std::string
convert_to_fmt(const std::string &s, sc_numrep numrep, bool w_prefix)
{
    int n = s.length();
    std::string str("0bus");
    // str += "0bus";
    str += s;
    sc_ufix a(str.c_str(), n, n, SC_TRN, SC_WRAP, 0, SC_ON);
    return a.to_string(numrep, w_prefix);
}

} // namespace sc_dt

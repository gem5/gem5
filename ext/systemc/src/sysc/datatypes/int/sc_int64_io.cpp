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

  sc_int64_io.cpp --

  Original Author: Amit Rao, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: sc_int64_io.cpp,v $
// Revision 1.2  2011/02/18 20:19:14  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:49:31  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#include "sysc/utils/sc_iostream.h"
#include "sysc/datatypes/int/sc_unsigned.h"
#include "sysc/datatypes/int/sc_signed.h"
#include "sysc/datatypes/int/sc_int_base.h"
#include "sysc/datatypes/int/sc_uint_base.h"


#if defined( _MSC_VER )

namespace sc_dt
{

static void
write_uint64(::std::ostream& os, uint64 val, int sign)
{
    const int WRITE_BUF_SIZE = 10 + sizeof(uint64)*3;
    char buf[WRITE_BUF_SIZE];
    char* buf_ptr = buf + WRITE_BUF_SIZE;
    const char* show_base = "";
    int show_base_len = 0;
    int show_pos = 0;
    fmtflags flags = os.flags();

    if ((flags & ::std::ios::basefield) == ::std::ios::oct) {
        do {
            *--buf_ptr = (char)((val & 7) + '0');
            val = val >> 3;
        } while (val != 0);
        if ((flags & ::std::ios::showbase) && (*buf_ptr != '0'))
            *--buf_ptr = '0';
    } else if ((flags & ::std::ios::basefield) == ::std::ios::hex) {
        const char* xdigs = (flags & ::std::ios::uppercase) ? 
            "0123456789ABCDEF0X" : 
            "0123456789abcdef0x";
        do {
            *--buf_ptr = xdigs[val & 15];
            val = val >> 4;
        } while (val != 0);
        if ((flags & ::std::ios::showbase)) {
            show_base = xdigs + 16;
            show_base_len = 2;
        }
    } else {
        while (val > UINT_MAX) {
            *--buf_ptr = (char)((val % 10) + '0');
            val /= 10;
        }
        unsigned ival = (unsigned) val;
        do {
            *--buf_ptr = (ival % 10) + '0';
            ival /= 10;
        } while (ival != 0);
        if (sign > 0 && (flags & ::std::ios::showpos))
            show_pos = 1;
    }

    int buf_len = buf + WRITE_BUF_SIZE - buf_ptr;
    int w = os.width(0);

    int len = buf_len + show_pos;
    if (sign < 0) len++;
    len += show_base_len;

    int padding = len > w ? 0 : w - len;
    fmtflags pad_kind = flags & ::std::ios::adjustfield;
    char fill_char = os.fill();

    if (padding > 0 &&
        ::std::ios::left != pad_kind &&
        ::std::ios::internal != pad_kind) {
        for (int i = padding - 1; i >= 0; --i) {
            if (! os.put(fill_char))
                goto fail;
        }
    }
    if (sign < 0 || show_pos) {
        if (! os.put(sign < 0 ? '-' : '+'))
            goto fail;
    }
    if (show_base_len) {
        if (! os.write(show_base, show_base_len))
            goto fail;
    }
    if ((fmtflags)::std::ios::internal == pad_kind && padding > 0) {
        for (int i = padding - 1; i >= 0; --i) {
            if (! os.put(fill_char))
                goto fail;
        }
    }
    if (! os.write(buf_ptr, buf_len))
        goto fail;
    if ((fmtflags)::std::ios::left == pad_kind && padding > 0) {
        for (int i = padding - 1; i >= 0; --i) {
            if (! os.put(fill_char))
                goto fail;
        }
    }
    os.osfx();
    return;
fail:
    //os.set(::std::ios::badbit);
    os.osfx();
}

::std::ostream&
operator << ( ::std::ostream& os, int64 n )
{
    if (os.opfx()) {
        int sign = 1;
        uint64 abs_n = (uint64) n;
        if (n < 0 && (os.flags() & (::std::ios::oct|::std::ios::hex)) == 0) {
            abs_n = -1*((uint64) n);
            sign = -1;
        }
        sc_dt::write_uint64(os, abs_n, sign);
    }
    return os;
}

::std::ostream&
operator << ( ::std::ostream& os, uint64 n )
{
    if (os.opfx()) {
        sc_dt::write_uint64(os, n, 0);
    }
    return os;
}

} // namespace sc_dt


#endif

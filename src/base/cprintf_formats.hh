/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 */

#ifndef __BASE_CPRINTF_FORMATS_HH__
#define __BASE_CPRINTF_FORMATS_HH__

#include <cstring>
#include <ostream>
#include <sstream>

namespace cp {

struct Format
{
    bool alternate_form;
    bool flush_left;
    bool print_sign;
    bool blank_space;
    bool fill_zero;
    bool uppercase;
    enum { dec, hex, oct } base;
    enum { none, string, integer, character, floating } format;
    enum { best, fixed, scientific } float_format;
    int precision;
    int width;
    bool get_precision;
    bool get_width;

    Format() { clear(); }

    void clear()
    {
        alternate_form = false;
        flush_left = false;
        print_sign = false;
        blank_space = false;
        fill_zero = false;
        uppercase = false;
        base = dec;
        format = none;
        float_format = best;
        precision = -1;
        width = 0;
        get_precision = false;
        get_width = false;
    }
};

template <typename T>
inline void
_format_char(std::ostream &out, const T &data, Format &fmt)
{
    using namespace std;

    out << data;
}

template <typename T>
inline void
_format_integer(std::ostream &out, const T &data, Format &fmt)
{
    using namespace std;

    ios::fmtflags flags(out.flags());

    switch (fmt.base) {
      case Format::hex:
        out.setf(std::ios::hex, std::ios::basefield);
        break;

      case Format::oct:
        out.setf(std::ios::oct, std::ios::basefield);
        break;

      case Format::dec:
        out.setf(std::ios::dec, std::ios::basefield);
        break;
    }

    if (fmt.alternate_form) {
        if (!fmt.fill_zero)
            out.setf(std::ios::showbase);
        else {
            switch (fmt.base) {
              case Format::hex:
                out << "0x";
                fmt.width -= 2;
                break;
              case Format::oct:
                out << "0";
                fmt.width -= 1;
                break;
              case Format::dec:
                break;
            }
        }
    }

    if (fmt.fill_zero)
        out.fill('0');

    if (fmt.width > 0)
        out.width(fmt.width);

    if (fmt.flush_left && !fmt.fill_zero)
        out.setf(std::ios::left);

    if (fmt.print_sign)
        out.setf(std::ios::showpos);

    if (fmt.uppercase)
        out.setf(std::ios::uppercase);

    out << data;

    out.flags(flags);
}

template <typename T>
inline void
_format_float(std::ostream &out, const T &data, Format &fmt)
{
    using namespace std;

    ios::fmtflags flags(out.flags());

    switch (fmt.float_format) {
      case Format::scientific:
        if (fmt.precision != -1) {
            if (fmt.width > 0)
                out.width(fmt.width);

            if (fmt.precision == 0)
                fmt.precision = 1;
            else
                out.setf(std::ios::scientific);

            out.precision(fmt.precision);
        } else
            if (fmt.width > 0)
                out.width(fmt.width);

        if (fmt.uppercase)
            out.setf(std::ios::uppercase);
        break;

      case Format::fixed:
        if (fmt.precision != -1) {
            if (fmt.width > 0)
                out.width(fmt.width);

            out.setf(std::ios::fixed);
            out.precision(fmt.precision);
        } else
            if (fmt.width > 0)
                out.width(fmt.width);

        break;

      default:
        if (fmt.precision != -1)
            out.precision(fmt.precision);

        if (fmt.width > 0)
            out.width(fmt.width);

        break;
    }

    out << data;

    out.flags(flags);
}

template <typename T>
inline void
_format_string(std::ostream &out, const T &data, Format &fmt)
{
    using namespace std;

#if defined(__GNUC__) && (__GNUC__ < 3) || 1
    if (fmt.width > 0) {
        std::stringstream foo;
        foo << data;
        int flen = foo.str().size();

        if (fmt.width > flen) {
            char *spaces = new char[fmt.width - flen + 1];
            memset(spaces, ' ', fmt.width - flen);
            spaces[fmt.width - flen] = 0;

            if (fmt.flush_left)
                out << foo.str() << spaces;
            else
                out << spaces << foo.str();

            delete [] spaces;
        } else
            out << data;
    } else
        out << data;
#else
    if (fmt.width > 0)
        out.width(fmt.width);
    if (fmt.flush_left)
        out.setf(std::ios::left);

    out << data;
#endif
}

/////////////////////////////////////////////////////////////////////////////
//
//  The code below controls the actual usage of formats for various types
//

//
// character formats
//
template <typename T>
inline void
format_char(std::ostream &out, const T &data, Format &fmt)
{ out << "<bad arg type for char format>"; }

inline void
format_char(std::ostream &out, char data, Format &fmt)
{ _format_char(out, data, fmt); }

inline void
format_char(std::ostream &out, unsigned char data, Format &fmt)
{ _format_char(out, data, fmt); }

inline void
format_char(std::ostream &out, signed char data, Format &fmt)
{ _format_char(out, data, fmt); }

inline void
format_char(std::ostream &out, short data, Format &fmt)
{ _format_char(out, (char)data, fmt); }

inline void
format_char(std::ostream &out, unsigned short data, Format &fmt)
{ _format_char(out, (char)data, fmt); }

inline void
format_char(std::ostream &out, int data, Format &fmt)
{ _format_char(out, (char)data, fmt); }

inline void
format_char(std::ostream &out, unsigned int data, Format &fmt)
{ _format_char(out, (char)data, fmt); }

inline void
format_char(std::ostream &out, long data, Format &fmt)
{ _format_char(out, (char)data, fmt); }

inline void
format_char(std::ostream &out, unsigned long data, Format &fmt)
{ _format_char(out, (char)data, fmt); }

inline void
format_char(std::ostream &out, long long data, Format &fmt)
{ _format_char(out, (char)data, fmt); }

inline void
format_char(std::ostream &out, unsigned long long data, Format &fmt)
{ _format_char(out, (char)data, fmt); }

//
// integer formats
//
template <typename T>
inline void
format_integer(std::ostream &out, const T &data, Format &fmt)
{ _format_integer(out, data, fmt); }
inline void
format_integer(std::ostream &out, char data, Format &fmt)
{ _format_integer(out, (int)data, fmt); }
inline void
format_integer(std::ostream &out, unsigned char data, Format &fmt)
{ _format_integer(out, (int)data, fmt); }
inline void
format_integer(std::ostream &out, signed char data, Format &fmt)
{ _format_integer(out, (int)data, fmt); }
#if 0
inline void
format_integer(std::ostream &out, short data, Format &fmt)
{ _format_integer(out, data, fmt); }
inline void
format_integer(std::ostream &out, unsigned short data, Format &fmt)
{ _format_integer(out, data, fmt); }
inline void
format_integer(std::ostream &out, int data, Format &fmt)
{ _format_integer(out, data, fmt); }
inline void
format_integer(std::ostream &out, unsigned int data, Format &fmt)
{ _format_integer(out, data, fmt); }
inline void
format_integer(std::ostream &out, long data, Format &fmt)
{ _format_integer(out, data, fmt); }
inline void
format_integer(std::ostream &out, unsigned long data, Format &fmt)
{ _format_integer(out, data, fmt); }
inline void
format_integer(std::ostream &out, long long data, Format &fmt)
{ _format_integer(out, data, fmt); }
inline void
format_integer(std::ostream &out, unsigned long long data, Format &fmt)
{ _format_integer(out, data, fmt); }
#endif

//
// floating point formats
//
template <typename T>
inline void
format_float(std::ostream &out, const T &data, Format &fmt)
{ out << "<bad arg type for float format>"; }

inline void
format_float(std::ostream &out, float data, Format &fmt)
{ _format_float(out, data, fmt); }

inline void
format_float(std::ostream &out, double data, Format &fmt)
{ _format_float(out, data, fmt); }

//
// string formats
//
template <typename T>
inline void
format_string(std::ostream &out, const T &data, Format &fmt)
{ _format_string(out, data, fmt); }

inline void
format_string(std::ostream &out, const std::stringstream &data, Format &fmt)
{ _format_string(out, data.str(), fmt); }

} // namespace cp

#endif // __CPRINTF_FORMATS_HH__

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
 */

#ifndef __BASE_CPRINTF_FORMATS_HH__
#define __BASE_CPRINTF_FORMATS_HH__

#include <cstdint>
#include <cstring>
#include <ostream>
#include <sstream>

#include "base/stl_helpers.hh"

namespace gem5
{

namespace cp
{

struct Format
{
    bool alternateForm;
    bool flushLeft;
    bool printSign;
    bool blankSpace;
    bool fillZero;
    bool uppercase;
    enum
    {
        Dec,
        Hex,
        Oct
    } base;
    enum
    {
        None,
        String,
        Integer,
        Character,
        Floating
    } format;
    enum
    {
        Best,
        Fixed,
        Scientific
    } floatFormat;
    int precision;
    int width;
    bool getPrecision;
    bool getWidth;

    Format() { clear(); }

    void
    clear()
    {
        alternateForm = false;
        flushLeft = false;
        printSign = false;
        blankSpace = false;
        fillZero = false;
        uppercase = false;
        base = Dec;
        format = None;
        floatFormat = Best;
        precision = -1;
        width = 0;
        getPrecision = false;
        getWidth = false;
    }
};

template <typename T>
static inline void
_formatChar(std::ostream &out, const T &data, Format &fmt)
{
    out << data;
}

template <typename T>
static inline void
_formatInteger(std::ostream &out, const T &data, Format &fmt)
{
    std::ios::fmtflags flags(out.flags());

    switch (fmt.base) {
      case Format::Hex:
        out.setf(std::ios::hex, std::ios::basefield);
        break;

      case Format::Oct:
        out.setf(std::ios::oct, std::ios::basefield);
        break;

      case Format::Dec:
        out.setf(std::ios::dec, std::ios::basefield);
        break;
    }

    if (fmt.alternateForm) {
        if (!fmt.fillZero) {
            out.setf(std::ios::showbase);
        } else {
            switch (fmt.base) {
              case Format::Hex:
                out << "0x";
                fmt.width -= 2;
                break;
              case Format::Oct:
                out << "0";
                fmt.width -= 1;
                break;
              case Format::Dec:
                break;
            }
        }
    }

    if (fmt.fillZero)
        out.fill('0');

    if (fmt.width > 0)
        out.width(fmt.width);

    if (fmt.flushLeft && !fmt.fillZero)
        out.setf(std::ios::left);

    if (fmt.printSign)
        out.setf(std::ios::showpos);

    if (fmt.uppercase)
        out.setf(std::ios::uppercase);

    out << data;

    out.flags(flags);
}

template <typename T>
static inline void
_formatFloat(std::ostream &out, const T &data, Format &fmt)
{
    std::ios::fmtflags flags(out.flags());

    if (fmt.fillZero)
        out.fill('0');

    switch (fmt.floatFormat) {
      case Format::Scientific:
        if (fmt.precision != -1) {
            if (fmt.width > 0)
                out.width(fmt.width);

            if (fmt.precision == 0)
                fmt.precision = 1;
            else
                out.setf(std::ios::scientific);

            out.precision(fmt.precision);
        } else if (fmt.width > 0) {
            out.width(fmt.width);
        }

        if (fmt.uppercase)
            out.setf(std::ios::uppercase);
        break;

      case Format::Fixed:
        if (fmt.precision != -1) {
            if (fmt.width > 0)
                out.width(fmt.width);

            out.setf(std::ios::fixed);
            out.precision(fmt.precision);
        } else if (fmt.width > 0) {
            out.width(fmt.width);
        }

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
static inline void
_formatString(std::ostream &out, const T &data, Format &fmt)
{
    using stl_helpers::operator<<;
    if (fmt.width > 0) {
        std::stringstream foo;
        foo << data;
        int flen = foo.str().size();

        if (fmt.width > flen) {
            std::string spaces(fmt.width - flen, ' ');
            if (fmt.flushLeft)
                out << foo.str() << spaces;
            else
                out << spaces << foo.str();
        } else {
            out << data;
        }
    } else {
        out << data;
    }
}

/////////////////////////////////////////////////////////////////////////////
//
//  The code below controls the actual usage of formats for various types
//

//
// character formats
//
template <typename T>
static inline void
formatChar(std::ostream &out, const T &data, Format &fmt)
{
    out << "<bad arg type for char format>";
}

static inline void
formatChar(std::ostream &out, char data, Format &fmt)
{
    _formatChar(out, data, fmt);
}

static inline void
formatChar(std::ostream &out, unsigned char data, Format &fmt)
{
    _formatChar(out, data, fmt);
}

static inline void
formatChar(std::ostream &out, signed char data, Format &fmt)
{
    _formatChar(out, data, fmt);
}

static inline void
formatChar(std::ostream &out, short data, Format &fmt)
{
    _formatChar(out, (char)data, fmt);
}

static inline void
formatChar(std::ostream &out, unsigned short data, Format &fmt)
{
    _formatChar(out, (char)data, fmt);
}

static inline void
formatChar(std::ostream &out, int data, Format &fmt)
{
    _formatChar(out, (char)data, fmt);
}

static inline void
formatChar(std::ostream &out, unsigned int data, Format &fmt)
{
    _formatChar(out, (char)data, fmt);
}

static inline void
formatChar(std::ostream &out, long data, Format &fmt)
{
    _formatChar(out, (char)data, fmt);
}

static inline void
formatChar(std::ostream &out, unsigned long data, Format &fmt)
{
    _formatChar(out, (char)data, fmt);
}

static inline void
formatChar(std::ostream &out, long long data, Format &fmt)
{
    _formatChar(out, (char)data, fmt);
}

static inline void
formatChar(std::ostream &out, unsigned long long data, Format &fmt)
{
    _formatChar(out, (char)data, fmt);
}

//
// integer formats
//
template <typename T>
static inline void
formatInteger(std::ostream &out, const T &data, Format &fmt)
{
    _formatInteger(out, data, fmt);
}
static inline void
formatInteger(std::ostream &out, char data, Format &fmt)
{
    _formatInteger(out, (int)data, fmt);
}
static inline void
formatInteger(std::ostream &out, unsigned char data, Format &fmt)
{
    _formatInteger(out, (int)data, fmt);
}
static inline void
formatInteger(std::ostream &out, signed char data, Format &fmt)
{
    _formatInteger(out, (int)data, fmt);
}
static inline void
formatInteger(std::ostream &out, const unsigned char *data, Format &fmt)
{
    _formatInteger(out, (uintptr_t)data, fmt);
}
static inline void
formatInteger(std::ostream &out, const signed char *data, Format &fmt)
{
    _formatInteger(out, (uintptr_t)data, fmt);
}

//
// floating point formats
//
template <typename T>
static inline void
formatFloat(std::ostream &out, const T &data, Format &fmt)
{
    out << "<bad arg type for float format>";
}

static inline void
formatFloat(std::ostream &out, float data, Format &fmt)
{
    _formatFloat(out, data, fmt);
}

static inline void
formatFloat(std::ostream &out, double data, Format &fmt)
{
    _formatFloat(out, data, fmt);
}

//
// string formats
//
template <typename T>
static inline void
formatString(std::ostream &out, const T &data, Format &fmt)
{
    _formatString(out, data, fmt);
}

} // namespace cp
} // namespace gem5

#endif // __CPRINTF_FORMATS_HH__

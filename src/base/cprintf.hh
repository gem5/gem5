/*
 * Copyright (c) 2014 ARM Limited
 * Copyright (c) 2002-2006 The Regents of The University of Michigan
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

#ifndef __BASE_CPRINTF_HH__
#define __BASE_CPRINTF_HH__

#include <ios>
#include <iostream>
#include <list>
#include <string>

#include "base/cprintf_formats.hh"

namespace gem5
{

namespace cp {

struct Print
{
  protected:
    std::ostream &stream;
    const char *format;
    const char *ptr;
    bool cont;

    std::ios::fmtflags savedFlags;
    char savedFill;
    int savedPrecision;
    int savedWidth;

    Format fmt;
    void process();
    void processFlag();

  public:
    Print(std::ostream &stream, const std::string &format);
    Print(std::ostream &stream, const char *format);
    ~Print();

    int
    getNumber(int data)
    {
        return data;
    }

    template <typename T>
    int
    getNumber(const T& data)
    {
        return 0;
    }

    template <typename T>
    void
    addArg(const T &data)
    {
        if (!cont)
            process();

        if (fmt.getWidth) {
            fmt.getWidth = false;
            cont = true;
            fmt.width = getNumber(data);
            return;
        }

        if (fmt.getPrecision) {
            fmt.getPrecision = false;
            cont = true;
            fmt.precision = getNumber(data);
            return;
        }

        switch (fmt.format) {
          case Format::Character:
            formatChar(stream, data, fmt);
            break;

          case Format::Integer:
            formatInteger(stream, data, fmt);
            break;

          case Format::Floating:
            formatFloat(stream, data, fmt);
            break;

          case Format::String:
            formatString(stream, data, fmt);
            break;

          default:
            stream << "<bad format>";
            break;
        }
    }

    void endArgs();
};

} // namespace cp

inline void
ccprintf(cp::Print &print)
{
    print.endArgs();
}


template<typename T, typename ...Args> void
ccprintf(cp::Print &print, const T &value, const Args &...args)
{
    print.addArg(value);

    ccprintf(print, args...);
}


template<typename ...Args> void
ccprintf(std::ostream &stream, const char *format, const Args &...args)
{
    cp::Print print(stream, format);

    ccprintf(print, args...);
}


template<typename ...Args> void
cprintf(const char *format, const Args &...args)
{
    ccprintf(std::cout, format, args...);
}

template<typename ...Args> std::string
csprintf(const char *format, const Args &...args)
{
    std::stringstream stream;
    ccprintf(stream, format, args...);
    return stream.str();
}

/*
 * functions again with std::string.  We have both so we don't waste
 * time converting const char * to std::string since we don't take
 * advantage of it.
 */
template<typename ...Args> void
ccprintf(std::ostream &stream, const std::string &format, const Args &...args)
{
    ccprintf(stream, format.c_str(), args...);
}

template<typename ...Args> void
cprintf(const std::string &format, const Args &...args)
{
    ccprintf(std::cout, format.c_str(), args...);
}

template<typename ...Args> std::string
csprintf(const std::string &format, const Args &...args)
{
    return csprintf(format.c_str(), args...);
}

} // namespace gem5

#endif // __CPRINTF_HH__

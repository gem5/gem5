/*
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

#include "base/cprintf.hh"

#include <iomanip>
#include <iostream>
#include <sstream>

#include "base/compiler.hh"

namespace gem5
{

namespace cp
{

Print::Print(std::ostream &stream, const std::string &format)
    : stream(stream), format(format.c_str()), ptr(format.c_str()), cont(false)
{
    savedFlags = stream.flags();
    savedFill = stream.fill();
    savedPrecision = stream.precision();
    savedWidth = stream.width();
}

Print::Print(std::ostream &stream, const char *format)
    : stream(stream), format(format), ptr(format), cont(false)
{
    savedFlags = stream.flags();
    savedFill = stream.fill();
    savedPrecision = stream.precision();
    savedWidth = stream.width();
}

Print::~Print()
{
}

void
Print::process()
{
    fmt.clear();

    size_t len;

    while (*ptr) {
        switch (*ptr) {
          case '%':
            if (ptr[1] != '%') {
                processFlag();
                return;
            }
            stream.put('%');
            ptr += 2;
            break;

          case '\n':
            stream << std::endl;
            ++ptr;
            break;
          case '\r':
            ++ptr;
            if (*ptr != '\n')
                stream << std::endl;
            break;

          default:
            len = strcspn(ptr, "%\n\r\0");
            stream.write(ptr, len);
            ptr += len;
            break;
        }
    }
}

void
Print::processFlag()
{
    bool done = false;
    bool end_number = false;
    bool have_precision = false;
    int number = 0;

    stream.fill(' ');
    stream.flags((std::ios::fmtflags)0);

    while (!done) {
        ++ptr;
        if (*ptr >= '0' && *ptr <= '9') {
            if (end_number)
                continue;
        } else if (number > 0) {
            end_number = true;
        }

        switch (*ptr) {
          case 's':
            fmt.format = Format::String;
            done = true;
            break;

          case 'c':
            fmt.format = Format::Character;
            done = true;
            break;

          case 'l':
            continue;

          case 'p':
            fmt.format = Format::Integer;
            fmt.base = Format::Hex;
            fmt.alternateForm = true;
            done = true;
            break;

          case 'X':
            fmt.uppercase = true;
            [[fallthrough]];
          case 'x':
            fmt.base = Format::Hex;
            fmt.format = Format::Integer;
            done = true;
            break;

          case 'o':
            fmt.base = Format::Oct;
            fmt.format = Format::Integer;
            done = true;
            break;

          case 'd':
          case 'i':
          case 'u':
            fmt.format = Format::Integer;
            done = true;
            break;

          case 'G':
            fmt.uppercase = true;
            [[fallthrough]];
          case 'g':
            fmt.format = Format::Floating;
            fmt.floatFormat = Format::Best;
            done = true;
            break;

          case 'E':
            fmt.uppercase = true;
            [[fallthrough]];
          case 'e':
            fmt.format = Format::Floating;
            fmt.floatFormat = Format::Scientific;
            done = true;
            break;

          case 'f':
            fmt.format = Format::Floating;
            fmt.floatFormat = Format::Fixed;
            done = true;
            break;

          case 'n':
            stream << "we don't do %n!!!\n";
            done = true;
            break;

          case '#':
            fmt.alternateForm = true;
            break;

          case '-':
            fmt.flushLeft = true;
            break;

          case '+':
            fmt.printSign = true;
            break;

          case ' ':
            fmt.blankSpace = true;
            break;

          case '.':
            fmt.width = number;
            fmt.precision = 0;
            have_precision = true;
            number = 0;
            end_number = false;
            break;

          case '0':
            if (number == 0) {
                fmt.fillZero = true;
                break;
            }
            [[fallthrough]];
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            number = number * 10 + (*ptr - '0');
            break;

          case '*':
            if (have_precision)
                fmt.getPrecision = true;
            else
                fmt.getWidth = true;
            break;

          case '%':
            GEM5_UNREACHABLE;
            break;

          default:
            done = true;
            break;
        }

        if (end_number) {
            if (have_precision)
                fmt.precision = number;
            else
                fmt.width = number;

            end_number = false;
            number = 0;
        }

        if (done) {
            if ((fmt.format == Format::Integer) && have_precision) {
                // specified a . but not a float, set width
                fmt.width = fmt.precision;
                // precision requries digits for width, must fill with 0
                fmt.fillZero = true;
            } else if ((fmt.format == Format::Floating) && !have_precision &&
                        fmt.fillZero) {
                // ambiguous case, matching printf
                fmt.precision = fmt.width;
            }
        }
    } // end while

    ++ptr;
}

void
Print::endArgs()
{
    size_t len;

    while (*ptr) {
        switch (*ptr) {
          case '%':
            if (ptr[1] != '%')
                stream << "<extra arg>";

            stream.put('%');
            ptr += 2;
            break;

          case '\n':
            stream << std::endl;
            ++ptr;
            break;
          case '\r':
            ++ptr;
            if (*ptr != '\n')
                stream << std::endl;
            break;

          default:
            len = strcspn(ptr, "%\n\r\0");
            stream.write(ptr, len);
            ptr += len;
            break;
        }
    }

    stream.flags(savedFlags);
    stream.fill(savedFill);
    stream.precision(savedPrecision);
    stream.width(savedWidth);
}

} // namespace cp
} // namespace gem5

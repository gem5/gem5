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

#include <cassert>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "base/compiler.hh"

using namespace std;

namespace cp {

Print::Print(std::ostream &stream, const std::string &format)
    : stream(stream), format(format.c_str()), ptr(format.c_str()), cont(false)
{
    saved_flags = stream.flags();
    saved_fill = stream.fill();
    saved_precision = stream.precision();
    saved_width = stream.width();
}

Print::Print(std::ostream &stream, const char *format)
    : stream(stream), format(format), ptr(format), cont(false)
{
    saved_flags = stream.flags();
    saved_fill = stream.fill();
    saved_precision = stream.precision();
    saved_width = stream.width();
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
                process_flag();
                return;
            }
            stream.put('%');
            ptr += 2;
            break;

          case '\n':
            stream << endl;
            ++ptr;
            break;
          case '\r':
            ++ptr;
            if (*ptr != '\n')
                stream << endl;
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
Print::process_flag()
{
    bool done = false;
    bool end_number = false;
    bool have_precision = false;
    int number = 0;

    stream.fill(' ');
    stream.flags((ios::fmtflags)0);

    while (!done) {
        ++ptr;
        if (*ptr >= '0' && *ptr <= '9') {
            if (end_number)
                continue;
        } else if (number > 0)
            end_number = true;

        switch (*ptr) {
          case 's':
            fmt.format = Format::string;
            done = true;
            break;

          case 'c':
            fmt.format = Format::character;
            done = true;
            break;

          case 'l':
            continue;

          case 'p':
            fmt.format = Format::integer;
            fmt.base = Format::hex;
            fmt.alternate_form = true;
            done = true;
            break;

          case 'X':
            fmt.uppercase = true;
            M5_FALLTHROUGH;
          case 'x':
            fmt.base = Format::hex;
            fmt.format = Format::integer;
            done = true;
            break;

          case 'o':
            fmt.base = Format::oct;
            fmt.format = Format::integer;
            done = true;
            break;

          case 'd':
          case 'i':
          case 'u':
            fmt.format = Format::integer;
            done = true;
            break;

          case 'G':
            fmt.uppercase = true;
            M5_FALLTHROUGH;
          case 'g':
            fmt.format = Format::floating;
            fmt.float_format = Format::best;
            done = true;
            break;

          case 'E':
            fmt.uppercase = true;
            M5_FALLTHROUGH;
          case 'e':
            fmt.format = Format::floating;
            fmt.float_format = Format::scientific;
            done = true;
            break;

          case 'f':
            fmt.format = Format::floating;
            fmt.float_format = Format::fixed;
            done = true;
            break;

          case 'n':
            stream << "we don't do %n!!!\n";
            done = true;
            break;

          case '#':
            fmt.alternate_form = true;
            break;

          case '-':
            fmt.flush_left = true;
            break;

          case '+':
            fmt.print_sign = true;
            break;

          case ' ':
            fmt.blank_space = true;
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
                fmt.fill_zero = true;
                break;
            }
            M5_FALLTHROUGH;
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
                fmt.get_precision = true;
            else
                fmt.get_width = true;
            break;

          case '%':
            assert(false && "we shouldn't get here");
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
            if ((fmt.format == Format::integer) && have_precision) {
                // specified a . but not a float, set width
                fmt.width = fmt.precision;
                // precision requries digits for width, must fill with 0
                fmt.fill_zero = true;
            } else if ((fmt.format == Format::floating) && !have_precision &&
                        fmt.fill_zero) {
                // ambiguous case, matching printf
                fmt.precision = fmt.width;
            }
        }
    } // end while

    ++ptr;
}

void
Print::end_args()
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
            stream << endl;
            ++ptr;
            break;
          case '\r':
            ++ptr;
            if (*ptr != '\n')
                stream << endl;
            break;

          default:
            len = strcspn(ptr, "%\n\r\0");
            stream.write(ptr, len);
            ptr += len;
            break;
        }
    }

    stream.flags(saved_flags);
    stream.fill(saved_fill);
    stream.precision(saved_precision);
    stream.width(saved_width);
}

} // namespace cp

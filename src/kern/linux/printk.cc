/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "kern/linux/printk.hh"

#include <sys/types.h>

#include <algorithm>
#include <iostream>
#include <sstream>

#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"

namespace Linux
{

int
printk(std::string &str, ThreadContext *tc, Addr format_ptr,
        PrintkVarArgs args)
{
    std::string format;
    std::ostringstream out;
    tc->getVirtProxy().readString(format, format_ptr);

    const char *p = format.c_str();

    while (*p) {
        switch (*p) {
          case '%': {
            bool more = true;
            bool islong = false;
            bool leftjustify = false;
            bool format = false;
            bool zero = false;
            int width = 0;
            while (more && *++p) {
                switch (*p) {
                  case 'l':
                  case 'L':
                    islong = true;
                    break;
                  case '-':
                    leftjustify = true;
                    break;
                  case '#':
                    format = true;
                    break;
                  case '0':
                    if (width)
                        width *= 10;
                    else
                        zero = true;
                    break;
                  default:
                    if (*p >= '1' && *p <= '9')
                        width = 10 * width + *p - '0';
                    else
                        more = false;
                    break;
                }
            }

            bool hexnum = false;
            bool octal = false;
            bool sign = false;
            switch (*p) {
              case 'X':
              case 'x':
                hexnum = true;
                break;
              case 'O':
              case 'o':
                octal = true;
                break;
              case 'D':
              case 'd':
                sign = true;
                break;
              case 'P':
                format = true;
                M5_FALLTHROUGH;
              case 'p':
                hexnum = true;
                break;
            }

            switch (*p) {
              case 'D':
              case 'd':
              case 'U':
              case 'u':
              case 'X':
              case 'x':
              case 'O':
              case 'o':
              case 'P':
              case 'p': {
                    if (hexnum)
                        out << std::hex;

                    if (octal)
                        out << std::oct;

                    if (format) {
                        if (!zero)
                            out.setf(std::ios::showbase);
                        else {
                            if (hexnum) {
                                out << "0x";
                                width -= 2;
                            } else if (octal) {
                                out << "0";
                                width -= 1;
                            }
                        }
                    }

                    if (zero)
                        out.fill('0');

                    if (width > 0)
                        out.width(width);

                    if (leftjustify && !zero)
                        out.setf(std::ios::left);

                    if (sign) {
                        if (islong)
                            out << args.get<int64_t>();
                        else
                            out << args.get<int32_t>();
                    } else {
                        if (islong)
                            out << args.get<uint64_t>();
                        else
                            out << args.get<uint32_t>();
                    }

                    if (zero)
                        out.fill(' ');

                    if (width > 0)
                        out.width(0);

                    out << std::dec;
                }
                break;

              case 's': {
                    Addr s_ptr = args.get<Addr>();
                    std::string s;
                    if (s_ptr)
                        tc->getVirtProxy().readString(s, s_ptr);
                    else
                        s = "<NULL>";

                    if (width > 0)
                        out.width(width);
                    if (leftjustify)
                        out.setf(std::ios::left);

                    out << s;
                }
                break;
              case 'C':
              case 'c': {
                    uint64_t mask = (*p == 'C') ? 0xffL : 0x7fL;
                    uint64_t num;
                    int cwidth;

                    if (islong) {
                        num = args.get<uint64_t>();
                        cwidth = sizeof(uint64_t);
                    } else {
                        num = args.get<uint32_t>();
                        cwidth = sizeof(uint32_t);
                    }

                    while (cwidth-- > 0) {
                        char c = (char)(num & mask);
                        if (c)
                            out << c;
                        num >>= 8;
                    }
                }
                break;
              case 'b': {
                    uint64_t n = args.get<uint64_t>();
                    Addr s_ptr = args.get<Addr>();
                    std::string s;
                    tc->getVirtProxy().readString(s, s_ptr);
                    out << s << ": " << n;
                }
                break;
              case '%':
                out << '%';
                break;
            }
            ++p;
          }
          break;
        case '\n':
          out << std::endl;
          ++p;
          break;
        case '\r':
          ++p;
          if (*p != '\n')
              out << std::endl;
          break;

        default: {
              size_t len = strcspn(p, "%\n\r\0");
              out.write(p, len);
              p += len;
            }
        }
    }

    str = out.str();
    return str.length();
}

} // namespace Linux

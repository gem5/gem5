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

#include <sys/types.h>

#include <algorithm>

#include "arch/vtophys.hh"
#include "base/cprintf.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "kern/tru64/printf.hh"
#include "sim/arguments.hh"

using namespace std;

namespace tru64 {

void
Printf(Arguments args)
{
    std::ostream &out = Trace::output();

    char *p = (char *)args++;

    ios::fmtflags saved_flags = out.flags();
    char old_fill = out.fill();
    int old_precision = out.precision();

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
                      out << hex;

                  if (octal)
                      out << oct;

                  if (format) {
                      if (!zero)
                          out.setf(ios::showbase);
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
                      out.setf(ios::left);

                  if (sign) {
                      if (islong)
                          out << (int64_t)args;
                      else
                          out << (int32_t)args;
                  } else {
                      if (islong)
                          out << (uint64_t)args;
                      else
                          out << (uint32_t)args;
                  }

                  if (zero)
                      out.fill(' ');

                  if (width > 0)
                      out.width(0);

                  out << dec;

                  ++args;
                }
                  break;

                case 's': {
                    const char *s = (char *)args;
                    if (!s)
                        s = "<NULL>";

                    if (width > 0)
                        out.width(width);
                    if (leftjustify)
                        out.setf(ios::left);

                    out << s;
                    ++args;
                }
                  break;
                case 'C':
                case 'c': {
                    uint64_t mask = (*p == 'C') ? 0xffL : 0x7fL;
                    uint64_t num;
                    int width;

                    if (islong) {
                        num = (uint64_t)args;
                        width = sizeof(uint64_t);
                    } else {
                        num = (uint32_t)args;
                        width = sizeof(uint32_t);
                    }

                    while (width-- > 0) {
                        char c = (char)(num & mask);
                        if (c)
                            out << c;
                        num >>= 8;
                    }

                    ++args;
                }
                  break;
                case 'b': {
                  uint64_t n = (uint64_t)args++;
                  char *s = (char *)args++;
                  out << s << ": " << n;
                }
                  break;
                case 'n':
                case 'N': {
                    args += 2;
#if 0
                    uint64_t n = (uint64_t)args++;
                    struct reg_values *rv = (struct reg_values *)args++;
#endif
                }
                  break;
                case 'r':
                case 'R': {
                    args += 2;
#if 0
                    uint64_t n = (uint64_t)args++;
                    struct reg_desc *rd = (struct reg_desc *)args++;
#endif
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
            out << endl;
            ++p;
            break;
          case '\r':
            ++p;
            if (*p != '\n')
                out << endl;
            break;

          default: {
              size_t len = strcspn(p, "%\n\r\0");
              out.write(p, len);
              p += len;
          }
        }
    }

    out.flags(saved_flags);
    out.fill(old_fill);
    out.precision(old_precision);
}

} // namespace Tru64

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

#include <sys/types.h>
#include <algorithm>

#include "base/cprintf.hh"
#include "base/trace.hh"
#include "sim/host.hh"
#include "targetarch/arguments.hh"
#include "targetarch/vtophys.hh"
#include "kern/linux/printk.hh"

using namespace std;


void
Printk(AlphaArguments args)
{
    char *p = (char *)args++;

    ios::fmtflags saved_flags = DebugOut().flags();
    char old_fill = DebugOut().fill();
    int old_precision = DebugOut().precision();

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
                      DebugOut() << hex;

                  if (octal)
                      DebugOut() << oct;

                  if (format) {
                      if (!zero)
                          DebugOut().setf(ios::showbase);
                      else {
                          if (hexnum) {
                              DebugOut() << "0x";
                              width -= 2;
                          } else if (octal) {
                              DebugOut() << "0";
                              width -= 1;
                          }
                      }
                  }

                  if (zero)
                      DebugOut().fill('0');

                  if (width > 0)
                      DebugOut().width(width);

                  if (leftjustify && !zero)
                      DebugOut().setf(ios::left);

                  if (sign) {
                      if (islong)
                          DebugOut() << (int64_t)args;
                      else
                          DebugOut() << (int32_t)args;
                  } else {
                      if (islong)
                          DebugOut() << (uint64_t)args;
                      else
                          DebugOut() << (uint32_t)args;
                  }

                  if (zero)
                      DebugOut().fill(' ');

                  if (width > 0)
                      DebugOut().width(0);

                  DebugOut() << dec;

                  ++args;
                }
                  break;

                case 's': {
                    char *s = (char *)args;
                    if (!s)
                        s = "<NULL>";

                    if (width > 0)
                        DebugOut().width(width);
                    if (leftjustify)
                        DebugOut().setf(ios::left);

                    DebugOut() << s;
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
                            DebugOut() << c;
                        num >>= 8;
                    }

                    ++args;
                }
                  break;
                case 'b': {
                  uint64_t n = (uint64_t)args++;
                  char *s = (char *)args++;
                  DebugOut() << s << ": " << n;
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
                  DebugOut() << '%';
                  break;
              }
              ++p;
          }
            break;
          case '\n':
            DebugOut() << endl;
            ++p;
            break;
          case '\r':
            ++p;
            if (*p != '\n')
                DebugOut() << endl;
            break;

          default: {
              size_t len = strcspn(p, "%\n\r\0");
              DebugOut().write(p, len);
              p += len;
          }
        }
    }

    DebugOut().flags(saved_flags);
    DebugOut().fill(old_fill);
    DebugOut().precision(old_precision);
}


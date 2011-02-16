/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
 * Copyright (c) 1993 The Hewlett-Packard Development Company
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
#include <stdarg.h>
#include <stdint.h>
#include "m5op.h"

/* The string s is terminated by a '\0' */
void
PutString(const char *s)
{
    while (*s)
        PutChar(*s++);
}

/* print c count times */
void
PutRepChar(char c, int count)
{
    while (count--)
        PutChar(c);
}

/* put string reverse */
void
PutStringReverse(const char *s, int index)
{
    while (index-- > 0)
        PutChar(s[index]);
}

/*
 * prints value in radix, in a field width width, with fill
 * character fill
 * if radix is negative, print as signed quantity
 * if width is negative, left justify
 * if width is 0, use whatever is needed
 * if fill is 0, use ' '
 */
void
PutNumber(long value, int radix, int width, char fill)
{
    char buffer[40];
    uint bufferindex = 0;
    ulong uvalue;
    ushort digit;
    ushort left = 0;
    ushort negative = 0;

    if (fill == 0)
        fill = ' ';

    if (width < 0) {
        width = -width;
        left = 1;
    }

    if (width < 0 || width > 80)
        width = 0;

    if (radix < 0) {
        radix = -radix;
        if (value < 0) {
            negative = 1;
            value = -value;
        }
    }

    switch (radix) {
      case 8:
      case 10:
      case 16:
        break;

      default:
        PutString("****");
        return;
    }

    uvalue = value;

    do {
        if (radix != 16) {
            digit = (ushort)(uvalue % radix);
            uvalue /= radix;
        } else {
            digit = (ushort)(uvalue & 0xf);
            uvalue = uvalue >> 4;
        }
        buffer[bufferindex] = digit + ((digit <= 9) ? '0' : ('A' - 10));
        bufferindex += 1;
    } while (uvalue != 0);

  /* fill # ' ' and negative cannot happen at once */
    if (negative) {
        buffer[bufferindex] = '-';
        bufferindex += 1;
    }

    if ((uint)width <= bufferindex) {
        PutStringReverse(buffer, bufferindex);
    } else {
        width -= bufferindex;
        if (!left)
            PutRepChar(fill, width);
        PutStringReverse(buffer, bufferindex);
        if (left)
            PutRepChar(fill, width);
    }
}

ulong
power(long base, long n)
{
    ulong p;

    for (p = 1; n > 0; --n)
        p = p * base;
    return p;
}

void
putFloat(double a, int fieldwidth, char fill)
{
    int i;
    ulong b;

    /*
     *  Put out everything before the decimal place.
     */
    PutNumber(((ulong) a), 10, fieldwidth, fill);

    /*
     *  Output the decimal place.
     */
    PutChar('.' & 0x7f);

    /*
     *  Output the n digits after the decimal place.
     */
    for (i = 1; i < 6; i++) {
        b = (ulong)(power(10, i) * (double)(a - (ulong) a));
        PutChar((char)(b % 10) + '0');
    }
}

const char *
FormatItem(const char *f, va_list *ap)
{
    char c;
    int fieldwidth = 0;
    int leftjust = 0;
    int radix = 0;
    char fill = ' ';

    if (*f == '0')
        fill = '0';

    while (c = *f++) {
        if (c >= '0' && c <= '9') {
            fieldwidth = (fieldwidth * 10) + (c - '0');
        } else {
            switch (c) {
              case '\000':
                return(--f);
              case '%':
                PutChar('%');
                return(f);
              case '-':
                leftjust = 1;
                break;
              case 'c': {
                  char a = (char)va_arg(*ap, int);

                  if (leftjust)
                      PutChar(a & 0x7f);
                  if (fieldwidth > 0)
                      PutRepChar(fill, fieldwidth - 1);
                  if (!leftjust)
                      PutChar(a & 0x7f);
                  return(f);
              }
              case 's': {
                  const char *a = va_arg(*ap, const char *);

                  if (leftjust)
                      PutString((const char *) a);
                  if (fieldwidth > strlen((const char *) a))
                      PutRepChar(fill, fieldwidth - strlen((const char *)a));
                  if (!leftjust)
                      PutString((const char *) a);
                  return(f);
              }
              case 'd':
                radix = -10;
                break;
              case 'u':
                radix = 10;
                break;
              case 'x':
                radix = 16;
                break;
              case 'X':
                radix = 16;
                break;
              case 'o':
                radix = 8;
                break;
              case 'f': {
                  double a = va_arg(*ap, double);

                  putFloat(a, fieldwidth, fill);
                  return(f);
              }
              default:   /* unknown switch! */
                radix = 3;
                break;
            }
        }

        if (radix)
            break;
    }

    if (leftjust)
        fieldwidth = -fieldwidth;

    long a = va_arg(*ap, long);
    PutNumber(a, radix, fieldwidth, fill);

    return(f);
}

int
printf(const char *f, ...)
{
    va_list ap;

    va_start(ap, f);

    while (*f) {
        if (*f == '%')
            f = FormatItem(f + 1, &ap);
        else
            PutChar(*f++);
    }

    if (*(f - 1) == '\n') {
        /* add a line-feed (SimOS console output goes to shell */
        PutChar('\r');
    }

    va_end(ap);         /* clean up */
    return 0;
}

void
panic(const char *f, ...)
{
    va_list ap;

    va_start(ap, f);

    printf("CONSOLE PANIC (looping): ");
    while (*f) {
        if (*f == '%')
            f = FormatItem(f + 1, &ap);
        else
            PutChar(*f++);
    }

    va_end(ap);         /* clean up */
    m5_panic();
}

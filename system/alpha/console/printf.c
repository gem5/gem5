/*
 * Copyright (c) 2003, 2004
 * The Regents of The University of Michigan
 * All Rights Reserved
 *
 * This code is part of the M5 simulator, developed by Nathan Binkert,
 * Erik Hallnor, Steve Raasch, and Steve Reinhardt, with contributions
 * from Ron Dreslinski, Dave Greene, Lisa Hsu, Ali Saidi, and Andrew
 * Schultz.
 *
 * Permission is granted to use, copy, create derivative works and
 * redistribute this software and such derivative works for any purpose,
 * so long as the copyright notice above, this grant of permission, and
 * the disclaimer below appear in all copies made; and so long as the
 * name of The University of Michigan is not used in any advertising or
 * publicity pertaining to the use or distribution of this software
 * without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED AS IS, WITHOUT REPRESENTATION FROM THE
 * UNIVERSITY OF MICHIGAN AS TO ITS FITNESS FOR ANY PURPOSE, AND WITHOUT
 * WARRANTY BY THE UNIVERSITY OF MICHIGAN OF ANY KIND, EITHER EXPRESS OR
 * IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE REGENTS OF
 * THE UNIVERSITY OF MICHIGAN SHALL NOT BE LIABLE FOR ANY DAMAGES,
 * INCLUDING DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, WITH RESPECT TO ANY CLAIM ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE SOFTWARE, EVEN IF IT HAS BEEN OR IS HEREAFTER
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 */

/*
 * Copyright 1993 Hewlett-Packard Development Company, L.P.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <sys/types.h>
#include <stdarg.h>

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
    while(1);
}

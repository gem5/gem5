/*****************************************************************************

       Copyright © 1993, 1994 Digital Equipment Corporation,
                       Maynard, Massachusetts.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its
documentation for any purpose and without fee is hereby granted, provided
that the copyright notice and this permission notice appear in all copies
of software and supporting documentation, and that the name of Digital not
be used in advertising or publicity pertaining to distribution of the software
without specific, written prior permission. Digital grants this permission
provided that you prominently mark, as not part of the original, any
modifications made to this software or documentation.

Digital Equipment Corporation disclaims all warranties and/or guarantees
with regard to this software, including all implied warranties of fitness for
a particular purpose and merchantability, and makes no representations
regarding the use of, or the results of the use of, the software and
documentation in terms of correctness, accuracy, reliability, currentness or
otherwise; and you rely on the software, documentation and results solely at
your own risk.

******************************************************************************/

#ifndef LINT
static char *rcsid = "$Id: printf.c,v 1.1.1.1 1997/10/30 23:27:12 verghese Exp $";
#endif

/*
 * $Log: printf.c,v $
 * Revision 1.1.1.1  1997/10/30 23:27:12  verghese
 * current 10/29/97
 *
 * Revision 1.1  1995/06/26  21:09:35  berc
 * Initial revision
 *
 * Revision 1.8  1994/10/06  20:29:08  fdh
 * Corrected unsigned long declaration.
 *
 * Revision 1.7  1994/08/05  20:16:23  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.6  1994/06/21  15:41:54  rusling
 * fixedup WNT compiler warnings
 *
 * Revision 1.5  1994/06/17  19:35:37  fdh
 * Clean-up...
 *
 * Revision 1.4  1994/01/19  10:40:08  rusling
 * Ported to Alpha Windows NT.
 *
 * Revision 1.3  1993/11/02  21:57:45  fdh
 * Fixed sign extension problem introduced in version 1.2
 *
 * Revision 1.2  1993/10/13  15:29:02  rusling
 * Added floating point support in printf.  This meant adding variable arguments to
 * it and FormatItem() and including stdarg.h.
 *
 * Revision 1.1  1993/06/08  19:56:24  fdh
 * Initial revision
 *
 */



/* printf.c
   L. S.
   Sun Feb 10 20:18:22 1985
 */

//#include "system.h"
#include "lib.h"
#include <stdarg.h>





/* The string s is terminated by a '\0' */
void
PutString(const char *s)
{
  while (*s) PutChar(*s++);
}

/* print c count times */
void
PutRepChar(char c, int count)
{
  while (count--) PutChar(c);
}

/* put string reverse */
void
PutStringReverse(const char *s, int index)
{
  while ((index--) > 0) PutChar(s[index]);
}

/* prints value in radix, in a field width width, with fill
   character fill
   if radix is negative, print as signed quantity
   if width is negative, left justify
   if width is 0, use whatever is needed
   if fill is 0, use ' '
 */
void
PutNumber(sl value, int radix, int width, char fill)
{
  char buffer[40];
  ui bufferindex = 0;
  ul uvalue;
  uw digit;
  uw left = FALSE;
  uw negative = FALSE;

  if (fill == 0) fill = ' ';

  if (width < 0) {
    width = -width;
    left = TRUE;
    }
  if (width < 0 || width > 80) width = 0;

  if (radix < 0) {
    radix = -radix;
    if (value < 0) {
      negative = TRUE;
      value = -value;
      }
    }
  switch (radix) {
    case 8:
    case 10:
    case 16: break;
    default: {
      PutString("****");
      return;
      }
    }
  uvalue = value;
  do {
    if (radix != 16)
    {
      digit = (uw)(uvalue % radix);
      uvalue /= radix;
    }
    else
    {
      digit = (uw)(uvalue & 0xf);
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
  if ((ui)width <= bufferindex) PutStringReverse(buffer, bufferindex);
  else {
    width -= bufferindex;
    if (!left) PutRepChar(fill, width);
    PutStringReverse(buffer, bufferindex);
    if (left) PutRepChar(fill, width);
    }
}

ul power(long base, long n)
{
  ul p;

  for (p = 1; n > 0; --n)
    p = p * base;
  return p;
}

void putFloat(double a, int fieldwidth, char fill)
{
  int i;
  ul b;

/*
 *  Put out everything before the decimal place.
 */
  PutNumber(((ul) a), 10, fieldwidth, fill);
/*
 *  Output the decimal place.
 */
  PutChar('.' & 0x7f);
/*
 *  Output the n digits after the decimal place.
 */
   for (i = 1; i < 6; i++) {
     b = (ul)(power(10, i) * (double)(a - (ul) a));
     PutChar((char)(b % 10) + '0');
   }
}
const char *
FormatItem(const char *f, va_list *ap)
{
  char c;
  int fieldwidth = 0;
  int leftjust = FALSE;
  int radix = 0;
  char fill = ' ';
  if (*f == '0') fill = '0';
  while (c = *f++) {
    if (c >= '0' && c <= '9') {
      fieldwidth = (fieldwidth * 10) + (c - '0');
      }
    else switch (c) {
      case '\000': return(--f);
      case '%': PutChar('%');
        return(f);
      case '-': leftjust = TRUE;
        break;
      case 'c': {
        char a = va_arg(*ap, char *);

        if (leftjust) PutChar(a & 0x7f);
        if (fieldwidth > 0) PutRepChar(fill, fieldwidth - 1);
        if (!leftjust) PutChar(a & 0x7f);
        return(f);
        }
      case 's': {
        const char *a = va_arg(*ap, const char *);

        if (leftjust) PutString((const char *) a);
        if (fieldwidth > strlen((const char *) a))
          PutRepChar(fill, fieldwidth - strlen((const char *)a));
        if (!leftjust) PutString((const char *) a);
        return(f);
        }
      case 'd': radix = -10;
        break;
      case 'u': radix = 10;
        break;
      case 'x': radix = 16;
        break;
      case 'X': radix = 16;
        break;
      case 'o': radix = 8;
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
    if (radix) break;
    }
  if (leftjust) fieldwidth = -fieldwidth;
  {
     sl a = va_arg(*ap, sl);
     PutNumber(a, radix, fieldwidth, fill);
  }
  return(f);
}

int
printf(const char *f, ...)
{
  va_list ap;

  va_start(ap, f);

  while (*f) {
    if (*f == '%') f = FormatItem(f + 1, &ap);
    else PutChar(*f++);
  }

  if (*(f-1)=='\n') {
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
    if (*f == '%') f = FormatItem(f + 1, &ap);
    else PutChar(*f++);
    }

  va_end(ap);         /* clean up */
  while(1);
}



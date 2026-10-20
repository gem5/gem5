/*
 * Copyright (c) 2026 The Board of Trustees of the Leland Stanford
 * Junior University
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

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "printf.h"

/* Width of the integer argument, selected by the length modifier. */
enum length
{
    LEN_INT,
    LEN_CHAR, /* hh */
    LEN_LONG, /* l  */
    LEN_SIZE, /* z  */
};

static void
emit(char c, int *count)
{
    _putchar(c);
    *count += 1;
}

/*
 * Render an integer in the given base, zero padded to `width` digits. Digits
 * are generated least significant first and replayed in reverse, which avoids
 * needing to find the leading digit up front.
 */
static void
emit_uint(uintmax_t value, unsigned base, bool upper, unsigned width,
          int *count)
{
    const char *const digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    char buf[20];
    unsigned len = 0;

    do {
        buf[len++] = digits[value % base];
        value /= base;
    } while (value != 0 && len < sizeof buf);

    while (width > len) {
        emit('0', count);
        --width;
    }
    while (len > 0) {
        emit(buf[--len], count);
    }
}

/*
 * Fetch the next integer argument. Returns its magnitude, setting *negative
 * for signed conversions of negative values.
 */
static uintmax_t
next_int(va_list *va, enum length len, bool is_signed, bool *negative)
{
    *negative = false;

    if (!is_signed) {
        switch (len) {
            case LEN_CHAR:
                return (unsigned char)va_arg(*va, unsigned int);
            case LEN_LONG:
                return va_arg(*va, unsigned long);
            case LEN_SIZE:
                return va_arg(*va, size_t);
            default:
                return va_arg(*va, unsigned int);
        }
    }

    intmax_t value;
    switch (len) {
        case LEN_CHAR:
            value = (signed char)va_arg(*va, int);
            break;
        case LEN_LONG:
            value = va_arg(*va, long);
            break;
        case LEN_SIZE:
            value = va_arg(*va, ptrdiff_t);
            break;
        default:
            value = va_arg(*va, int);
            break;
    }

    *negative = value < 0;
    /* Negate in the unsigned domain so INTMAX_MIN survives. */
    return *negative ? -(uintmax_t)value : (uintmax_t)value;
}

int
printf_(const char *format, ...)
{
    va_list va;
    va_start(va, format);

    int count = 0;

    for (const char *p = format; *p != '\0'; ++p) {
        if (*p != '%') {
            emit(*p, &count);
            continue;
        }
        if (*++p == '\0') {
            break;
        }

        /* Zero padded field width, as in %02hhx. */
        unsigned width = 0;
        if (*p == '0') {
            ++p;
        }
        while (*p >= '0' && *p <= '9') {
            width = width * 10 + (unsigned)(*p++ - '0');
        }

        enum length len = LEN_INT;
        if (p[0] == 'h' && p[1] == 'h') {
            p += 2;
            len = LEN_CHAR;
        } else if (*p == 'l') {
            ++p;
            len = LEN_LONG;
        } else if (*p == 'z') {
            ++p;
            len = LEN_SIZE;
        }

        bool negative;
        switch (*p) {
            case 'd': {
                const uintmax_t magnitude =
                    next_int(&va, len, true, &negative);
                if (negative) {
                    emit('-', &count);
                }
                emit_uint(magnitude, 10, false, width, &count);
                break;
            }

            case 'u':
                emit_uint(next_int(&va, len, false, &negative), 10, false,
                          width, &count);
                break;

            case 'x':
                emit_uint(next_int(&va, len, false, &negative), 16, false,
                          width, &count);
                break;

            case 'p':
                /* Pointers print as zero padded uppercase hex, with no 0x. */
                emit_uint((uintptr_t)va_arg(va, void *), 16, true,
                          sizeof(void *) * 2, &count);
                break;

            case 'c':
                emit((char)va_arg(va, int), &count);
                break;

            case 's': {
                const char *s = va_arg(va, const char *);
                if (s == NULL) {
                    s = "(null)";
                }
                while (*s != '\0') {
                    emit(*s++, &count);
                }
                break;
            }

            default:
                /* Unsupported conversion: echo it so the mistake is visible.
                 */
                emit('%', &count);
                emit(*p, &count);
                break;
        }
    }

    va_end(va);
    return count;
}

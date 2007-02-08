/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *          Dave Greene
 */

#ifndef __MISC_HH__
#define __MISC_HH__

#include <cassert>

#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "base/varargs.hh"

#if defined(__SUNPRO_CC)
#define __FUNCTION__ "how to fix me?"
#endif

//
// This implements a cprintf based panic() function.  panic() should
// be called when something happens that should never ever happen
// regardless of what the user does (i.e., an acutal m5 bug).  panic()
// calls abort which can dump core or enter the debugger.
//
//
void __panic(const char *func, const char *file, int line, const char *format,
             CPRINTF_DECLARATION) M5_ATTR_NORETURN;
void __panic(const char *func, const char *file, int line,
             const std::string &format, CPRINTF_DECLARATION)
M5_ATTR_NORETURN;

inline void
__panic(const char *func, const char *file, int line,
        const std::string &format, CPRINTF_DEFINITION)
{
    __panic(func, file, line, format.c_str(), VARARGS_ALLARGS);
}
M5_PRAGMA_NORETURN(__panic)
#define panic(...) __panic(__FUNCTION__, __FILE__, __LINE__, __VA_ARGS__)

//
// This implements a cprintf based fatal() function.  fatal() should
// be called when the simulation cannot continue due to some condition
// that is the user's fault (bad configuration, invalid arguments,
// etc.) and not a simulator bug.  fatal() calls exit(1), i.e., a
// "normal" exit with an error code, as opposed to abort() like
// panic() does.
//
void __fatal(const char *func, const char *file, int line, const char *format,
             CPRINTF_DECLARATION) M5_ATTR_NORETURN;
void __fatal(const char *func, const char *file, int line,
             const std::string &format, CPRINTF_DECLARATION)
    M5_ATTR_NORETURN;

inline void
__fatal(const char *func, const char *file, int line,
        const std::string &format, CPRINTF_DEFINITION)
{
    __fatal(func, file, line, format.c_str(), VARARGS_ALLARGS);
}
M5_PRAGMA_NORETURN(__fatal)
#define fatal(...) __fatal(__FUNCTION__, __FILE__, __LINE__, __VA_ARGS__)

//
// This implements a cprintf based warn
//
void __warn(const char *func, const char *file, int line, const char *format,
            CPRINTF_DECLARATION);
inline void
__warn(const char *func, const char *file, int line, const std::string &format,
       CPRINTF_DECLARATION)
{
    __warn(func, file, line, format, VARARGS_ALLARGS);
}
#define warn(...) __warn(__FUNCTION__, __FILE__, __LINE__, __VA_ARGS__)

// Only print the warning message the first time it is seen.  This
// doesn't check the warning string itself, it just only lets one
// warning come from the statement. So, even if the arguments change
// and that would have resulted in a different warning message,
// subsequent messages would still be supressed.
#define warn_once(...) do {                         \
        static bool once = false;                   \
        if (!once) {                                \
            warn(__VA_ARGS__);                       \
            once = true;                            \
        }                                           \
    } while (0)

//
// assert() that prints out the current cycle
//
#define m5_assert(TEST) do {                                            \
    if (!(TEST))                                                        \
        ccprintf(std::cerr, "Assertion failure, curTick = %d\n", curTick); \
    assert(TEST);                                                       \
} while (0)

#endif // __MISC_HH__

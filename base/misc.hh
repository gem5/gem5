/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __MISC_HH__
#define __MISC_HH__

#include <assert.h>
#include "base/cprintf.hh"

//
// This implements a cprintf based panic
//
void __panic(const std::string&, cp::ArgList &, const char*, const char*, int)
    __attribute__((noreturn));
#define __panic__(format, args...) \
    __panic(format, (*(new cp::ArgList), args), \
        __FUNCTION__, __FILE__, __LINE__)
#define panic(args...) \
    __panic__(args, cp::ArgListNull())

//
// This implements a cprintf based fatal
//
void __fatal(const std::string&, cp::ArgList &, const char*, const char*, int)
    __attribute__((noreturn));
#define __fatal__(format, args...) \
    __fatal(format, (*(new cp::ArgList), args), \
        __FUNCTION__, __FILE__, __LINE__)
#define fatal(args...) \
    __fatal__(args, cp::ArgListNull())

//
// This implements a cprintf based warn
//
void __warn(const std::string&, cp::ArgList &, const char*, const char*, int);
#define __warn__(format, args...) \
    __warn(format, (*(new cp::ArgList), args), \
           __FUNCTION__, __FILE__, __LINE__)
#define warn(args...) \
    __warn__(args, cp::ArgListNull())

//
// assert() that prints out the current cycle
//
#define m5_assert(TEST) \
   if (!(TEST)) { \
     std::cerr << "Assertion failure, curTick = " << curTick << std::endl; \
   } \
   assert(TEST);

#endif // __MISC_HH__

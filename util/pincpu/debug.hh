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

#ifndef __UTIL_PINCPU_DEBUG_HH__
#define __UTIL_PINCPU_DEBUG_HH__

#include <iostream>

#define ENABLE_DEBUGGING 0

#if ENABLE_DEBUGGING
#define DEBUG(...)                                                            \
    do {                                                                      \
        __VA_ARGS__;                                                          \
    } while (0)
#else
#define DEBUG(...)                                                            \
    do {                                                                      \
    } while (0)
#endif

#if ENABLE_DEBUGGING
static inline std::ostream &
dbgs()
{ return std::cerr; }
#else
struct DummyPrinter
{
};

static inline DummyPrinter
dbgs()
{ return DummyPrinter(); }

template <typename T>
DummyPrinter
operator<<(DummyPrinter, const T &)
{ return DummyPrinter(); }

static inline DummyPrinter
operator<<(DummyPrinter, std::ostream &(*)(std::ostream &))
{ return DummyPrinter(); }

static inline DummyPrinter
operator<<(DummyPrinter, std::ios_base &(*)(std::ios_base &))
{ return DummyPrinter(); }
#endif

#endif // __UTIL_PINCPU_DEBUG_HH__

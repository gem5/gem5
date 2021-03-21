/*
 * Copyright (c) 2015 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
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

#include "kern/freebsd/events.hh"

#include <sstream>

#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/DebugPrintf.hh"
#include "kern/system_events.hh"
#include "sim/core.hh"
#include "sim/system.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(FreeBSD, free_bsd);
namespace free_bsd
{

void
onUDelay(ThreadContext *tc, uint64_t div, uint64_t mul, uint64_t time)
{
    // convert parameter to ns
    if (div)
        time /= div;

    time *= mul;

    // Currently, only ARM full-system simulation uses UDelayEvents to skip
    // __delay and __loop_delay functions. One form involves setting quiesce
    // time to 0 with the assumption that quiesce will not happen. To avoid
    // the quiesce handling in this case, only execute the quiesce if time > 0.
    if (time > 0)
        tc->quiesceTick(curTick() + sim_clock::as_int::ns * time);
}

} // namespace free_bsd
} // namespace gem5

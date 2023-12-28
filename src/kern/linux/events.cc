/*
 * Copyright (c) 2011, 2016, 2023 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#include "kern/linux/events.hh"

#include <sstream>

#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/helpers.hh"
#include "kern/system_events.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

namespace linux
{

void
PanicOrOopsEvent::process(ThreadContext *tc)
{
    warn(descr());

    if (behaviour != KernelPanicOopsBehaviour::Continue) {
        inform("Dumping kernel dmesg buffer to %s...\n", fname);
        OutputStream *os = simout.create(fname);
        dumpDmesg(tc, *os->stream());
        simout.close(os);
    }

    if (behaviour == KernelPanicOopsBehaviour::DumpDmesgAndExit) {
        exitSimLoop(descr(), static_cast<int>(1), curTick(), false);
    } else if (behaviour == KernelPanicOopsBehaviour::DumpDmesgAndPanic) {
        panic(descr());
    }
}

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

} // namespace linux
} // namespace gem5

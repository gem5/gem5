/*
 * Copyright (c) 2011 ARM Limited
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
 *
 * Authors: Nathan Binkert
 *          Ali Saidi
 */

#include <sstream>

#include "arch/utility.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/DebugPrintf.hh"
#include "kern/linux/events.hh"
#include "kern/linux/printk.hh"
#include "kern/system_events.hh"
#include "sim/arguments.hh"
#include "sim/pseudo_inst.hh"
#include "sim/system.hh"

namespace Linux {

void
DebugPrintkEvent::process(ThreadContext *tc)
{
    if (DTRACE(DebugPrintf)) {
        std::stringstream ss;
        Arguments args(tc);
        Printk(ss, args);
        StringWrap name(tc->getSystemPtr()->name() + ".dprintk");
        DPRINTFN("%s", ss.str());
    }
    SkipFuncEvent::process(tc);
}

void
UDelayEvent::process(ThreadContext *tc)
{
    int arg_num  = 0;

    // Get the time in native size
    uint64_t time = TheISA::getArgument(tc, arg_num,  (uint16_t)-1, false);

    // convert parameter to ns
    if (argDivToNs)
        time /= argDivToNs;

    time *= argMultToNs;

    SkipFuncEvent::process(tc);

    // Currently, only ARM full-system simulation uses UDelayEvents to skip
    // __delay and __loop_delay functions. One form involves setting quiesce
    // time to 0 with the assumption that quiesce will not happen. To avoid
    // the quiesce handling in this case, only execute the quiesce if time > 0.
    if (time > 0) {
        PseudoInst::quiesceNs(tc, time);
    }
}


} // namespace linux

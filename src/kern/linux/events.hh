/*
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

#ifndef __KERN_LINUX_EVENTS_HH__
#define __KERN_LINUX_EVENTS_HH__

#include "kern/system_events.hh"

namespace Linux {

class DebugPrintkEvent : public SkipFuncEvent
{
  public:
    DebugPrintkEvent(PCEventQueue *q, const std::string &desc, Addr addr)
        : SkipFuncEvent(q, desc, addr) {}
    virtual void process(ThreadContext *xc);
};

/** A class to skip udelay() and related calls in the kernel.
 * This class has two additional parameters that take the argument to udelay and
 * manipulated it to come up with ns and eventually ticks to quiesce for.
 * See descriptions of argDivToNs and argMultToNs below.
 */
class UDelayEvent : public SkipFuncEvent
{
  private:
    /** value to divide arg by to create ns. This is present beacues the linux
     * kernel code sometime precomputes the first multiply that is done in
     * udelay() if the parameter is a constant. We need to undo it so here is
     * how. */
    uint64_t argDivToNs;

    /** value to multiple arg by to create ns. Nominally, this is 1000 to
     * convert us to ns, but since linux can do some preprocessing of constant
     * values something else might be required. */
    uint64_t argMultToNs;

  public:
    UDelayEvent(PCEventQueue *q, const std::string &desc, Addr addr,
            uint64_t mult, uint64_t div)
        : SkipFuncEvent(q, desc, addr), argDivToNs(div), argMultToNs(mult) {}
    virtual void process(ThreadContext *xc);
};


}

#endif

/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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

#ifndef __KERN_LINUX_LINUX_SYSTEM_HH__
#define __KERN_LINUX_LINUX_SYSTEM_HH__

#include "sim/host.hh"
#include "sim/system.hh"
#include "targetarch/isa_traits.hh"

/**
 * MAGIC address where the kernel arguments should go. Defined as
 * PARAM in linux kernel alpha-asm.
 */
const Addr PARAM_ADDR = ULL(0xfffffc000030a000);

class ExecContext;

class BreakPCEvent;
class DebugPrintkEvent;
class BreakPCEvent;
class LinuxSkipDelayLoopEvent;
class SkipFuncEvent;
class IdleStartEvent;
class PrintThreadInfo;

/**
 * This class contains linux specific system code (Loading, Events, Binning).
 * It points to objects that are the system binaries to load and patches them
 * appropriately to work in simulator.
 */
class LinuxSystem : public System
{
  private:
#ifndef NDEBUG
    /** Event to halt the simulator if the kernel calls panic()  */
    BreakPCEvent *kernelPanicEvent;

    /** Event to halt the simulator if the kernel calls die_if_kernel  */
    BreakPCEvent *kernelDieEvent;
#endif

    /**
     * Event to skip determine_cpu_caches() because we don't support
     * the IPRs that the code can access to figure out cache sizes
     */
    SkipFuncEvent *skipCacheProbeEvent;

    /** PC based event to skip the ide_delay_50ms() call */
    SkipFuncEvent *skipIdeDelay50msEvent;

    /**
     * PC based event to skip the dprink() call and emulate its
     * functionality
     */
    DebugPrintkEvent *debugPrintkEvent;

    /**
     * Skip calculate_delay_loop() rather than waiting for this to be
     * calculated
     */
    LinuxSkipDelayLoopEvent *skipDelayLoopEvent;

    /**
     * Event to print information about thread switches if the trace flag
     * Thread is set
     */
    PrintThreadInfo *printThreadEvent;

    /**
     * Event to bin Interrupts seperately from kernel code
     */
    InterruptStartEvent *intStartEvent;

    /**
     * Event to bin Interrupts seperately from kernel code
     */
    InterruptEndEvent *intEndEvent;
    InterruptEndEvent *intEndEvent2;
    InterruptEndEvent *intEndEvent3;

    /** Grab the PCBB of the idle process when it starts */
    IdleStartEvent *idleStartEvent;

  public:
    LinuxSystem(Params *p);
    ~LinuxSystem();

    void setDelayLoop(ExecContext *xc);
};

#endif // __KERN_LINUX_LINUX_SYSTEM_HH__

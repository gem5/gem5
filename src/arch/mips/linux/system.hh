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
 * Authors: Ali Saidi
 *          Lisa Hsu
 *          Nathan Binkert
 */

#ifndef __ARCH_MIPS_LINUX_SYSTEM_HH__
#define __ARCH_MIPS_LINUX_SYSTEM_HH__

class ThreadContext;

class BreakPCEvent;
class IdleStartEvent;

#include "arch/mips/idle_event.hh"
#include "arch/mips/system.hh"
#include "kern/linux/events.hh"
#include "params/LinuxMipsSystem.hh"

/**
 * This class contains linux specific system code (Loading, Events).
 * It points to objects that are the system binaries to load and patches them
 * appropriately to work in simulator.
 */
class LinuxMipsSystem : public MipsSystem
{
  private:
    class SkipDelayLoopEvent : public SkipFuncEvent
    {
      public:
        SkipDelayLoopEvent(PCEventQueue *q, const std::string &desc, Addr addr)
            : SkipFuncEvent(q, desc, addr) {}
        virtual void process(ThreadContext *tc);
    };

    class PrintThreadInfo : public PCEvent
    {
      public:
        PrintThreadInfo(PCEventQueue *q, const std::string &desc, Addr addr)
            : PCEvent(q, desc, addr) {}
        virtual void process(ThreadContext *tc);
    };


    /**
     * Addresses defining where the kernel bootloader places various
     * elements.  Details found in include/asm-mips/system.h
     */
    Addr KernelStart; // Lookup the symbol swapper_pg_dir

  public:
    Addr InitStack() const { return KernelStart + 0x02000; }
    Addr EmptyPGT() const  { return KernelStart + 0x04000; }
    Addr EmptyPGE() const  { return KernelStart + 0x08000; }
    Addr ZeroPGE() const   { return KernelStart + 0x0A000; }
    Addr StartAddr() const { return KernelStart + 0x10000; }

    Addr Param() const { return ZeroPGE() + 0x0; }
    Addr CommandLine() const { return Param() + 0x0; }
    Addr InitrdStart() const { return Param() + 0x100; }
    Addr InitrdSize() const { return Param() + 0x108; }
    static const int CommandLineSize = 256;

  public:
    typedef LinuxMipsSystemParams Params;
    LinuxMipsSystem(Params *p);
    ~LinuxMipsSystem();

    void setDelayLoop(ThreadContext *tc);
};

#endif // __ARCH_MIPS_LINUX_SYSTEM_HH__

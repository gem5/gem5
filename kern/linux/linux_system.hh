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

#ifndef __LINUX_SYSTEM_HH__
#define __LINUX_SYSTEM_HH__

#include <vector>

#include "sim/system.hh"
#include "sim/host.hh"
#include "targetarch/isa_traits.hh"

#include <map>

/**
 * MAGIC address where the kernel arguments should go. Defined as
 * PARAM in linux kernel alpha-asm.
 */
const Addr PARAM_ADDR =  ULL(0xfffffc000030a000);

class ExecContext;
class ElfObject;
class SymbolTable;
class DebugPrintkEvent;
class BreakPCEvent;
class LinuxSkipDelayLoopEvent;
class SkipFuncEvent;
class FnEvent;
class AlphaArguments;
class PrintThreadInfo;

/**
 * This class contains linux specific system code (Loading, Events, Binning).
 * It points to objects that are the system binaries to load and patches them
 * appropriately to work in simulator.
 */
class LinuxSystem : public System
{
  private:
    /** Object pointer for the kernel code */
    ElfObject *kernel;

    /** Object pointer for the console code */
    ElfObject *console;

    /** kernel Symbol table */
    SymbolTable *kernelSymtab;

    /** console symbol table */
    SymbolTable *consoleSymtab;

    /** Event to halt the simulator if the kernel calls panic()  */
    BreakPCEvent *kernelPanicEvent;

    /** Event to halt the simulator if the console calls panic() */
    BreakPCEvent *consolePanicEvent;

    /** Event to skip determine_cpu_caches() because we don't support the
     * IPRs that the code can access to figure out cache sizes
     */
    SkipFuncEvent *skipCacheProbeEvent;

    /** PC based event to skip the ide_delay_50ms() call */
    SkipFuncEvent *skipIdeDelay50msEvent;

    /** PC based event to skip the dprink() call and emulate its functionality */
    DebugPrintkEvent *debugPrintkEvent;

    /** Skip calculate_delay_loop() rather than waiting for this to be
     * calculated
     */
    LinuxSkipDelayLoopEvent *skipDelayLoopEvent;

    PrintThreadInfo *printThreadEvent;

    /** Begining of kernel code */
    Addr kernelStart;

    /** End of kernel code */
    Addr kernelEnd;

    /** Entry point in the kernel to start at */
    Addr kernelEntry;

    bool bin;
    std::vector<string> binned_fns;

  public:
    std::vector<RemoteGDB *>   remoteGDB;
    std::vector<GDBListener *> gdbListen;

    LinuxSystem(const std::string _name,
                const uint64_t _init_param,
                MemoryController *_memCtrl,
                PhysicalMemory *_physmem,
                const std::string &kernel_path,
                const std::string &console_path,
                const std::string &palcode,
                const std::string &boot_osflags,
                const bool _bin,
                const std::vector<std::string> &_binned_fns);

    ~LinuxSystem();

    void setDelayLoop(ExecContext *xc);

    int registerExecContext(ExecContext *xc);
    void replaceExecContext(ExecContext *xc, int xcIndex);

    /**
     * Returns the addess the kernel starts at.
     * @return address the kernel starts at
     */
    Addr getKernelStart() const { return kernelStart; }

    /**
     * Returns the addess the kernel ends at.
     * @return address the kernel ends at
     */
    Addr getKernelEnd() const { return kernelEnd; }

    /**
     * Returns the addess the entry point to the kernel code.
     * @return entry point of the kernel code
     */
    Addr getKernelEntry() const { return kernelEntry; }


    bool breakpoint();
};

#endif // __LINUX_SYSTEM_HH__

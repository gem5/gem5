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

#ifndef __LINUX_SYSTEM_HH__
#define __LINUX_SYSTEM_HH__

#include <vector>

#include "sim/system.hh"
#include "targetarch/isa_traits.hh"

#include <map>

class ExecContext;
class ElfObject;
class SymbolTable;

class BreakPCEvent;
class LinuxBadAddrEvent;
class LinuxSkipFuncEvent;
class LinuxSkipDelayLoopEvent;
class LinuxSkipIdeDelay50msEvent;
class LinuxPrintfEvent;
class LinuxDebugPrintfEvent;
class LinuxDumpMbufEvent;
class FnEvent;
class AlphaArguments;

class LinuxSystem : public System
{
  private:
    ElfObject *kernel;
    ElfObject *console;
    ElfObject *bootloader;

    SymbolTable *kernelSymtab;
    SymbolTable *bootloaderSymtab;
    SymbolTable *consoleSymtab;

    BreakPCEvent *kernelPanicEvent;
    BreakPCEvent *consolePanicEvent;
    LinuxSkipFuncEvent *skipCacheProbeEvent;
    LinuxSkipIdeDelay50msEvent *skipIdeDelay50msEvent;
    LinuxSkipDelayLoopEvent *skipDelayLoopEvent;

  private:

    Addr kernelStart;
    Addr kernelEnd;
    Addr kernelEntry;
    bool bin;
    std::vector<string> binned_fns;

  public:
    std::vector<RemoteGDB *>   remoteGDB;
    std::vector<GDBListener *> gdbListen;

  public:
    LinuxSystem(const std::string _name,
                const uint64_t _init_param,
                MemoryController *_memCtrl,
                PhysicalMemory *_physmem,
                const std::string &kernel_path,
                const std::string &console_path,
                const std::string &palcode,
                const std::string &boot_osflags,
                const std::string &bootloader_path,
                const bool _bin,
                const std::vector<std::string> &_binned_fns);

    ~LinuxSystem();

    void setDelayLoop(ExecContext *xc);

    int registerExecContext(ExecContext *xc);
    void replaceExecContext(ExecContext *xc, int xcIndex);

    Addr getKernelStart() const { return kernelStart; }
    Addr getKernelEnd() const { return kernelEnd; }
    Addr getKernelEntry() const { return kernelEntry; }
    bool breakpoint();
};

#endif // __LINUX_SYSTEM_HH__

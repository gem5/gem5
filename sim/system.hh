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

#ifndef __SYSTEM_HH__
#define __SYSTEM_HH__

#include <string>
#include <vector>

#include "sim/sim_object.hh"
#include "cpu/pc_event.hh"
#include "base/loader/symtab.hh"

#ifdef FS_MEASURE
#include "base/statistics.hh"
#include "sim/sw_context.hh"
#endif

class MemoryController;
class PhysicalMemory;
class RemoteGDB;
class GDBListener;

class ExecContext;

class System : public SimObject
{
#ifdef FS_MEASURE
  protected:
    std::map<const std::string, Statistics::MainBin *> fnBins;
    std::map<const Addr, SWContext *> swCtxMap;
#endif //FS_MEASURE

  public:
    const uint64_t init_param;
    MemoryController *memCtrl;
    PhysicalMemory *physmem;
    bool bin;

    PCEventQueue pcEventQueue;

    std::vector<ExecContext *> execContexts;

    virtual int registerExecContext(ExecContext *xc);
    virtual void replaceExecContext(int xcIndex, ExecContext *xc);

#ifdef FS_MEASURE
    Statistics::Scalar<Counter, Statistics::MainBin> fnCalls;
    Statistics::MainBin *nonPath;
#endif //FS_MEASURE

  public:
    System(const std::string _name, const uint64_t _init_param,
           MemoryController *, PhysicalMemory *, const bool);
    ~System();

    virtual Addr getKernelStart() const = 0;
    virtual Addr getKernelEnd() const = 0;
    virtual Addr getKernelEntry() const = 0;
    virtual bool breakpoint() = 0;

#ifdef FS_MEASURE
    Statistics::MainBin * getBin(const std::string &name);
    virtual bool findCaller(std::string, std::string) const = 0;

    SWContext *findContext(Addr pcb);
    bool addContext(Addr pcb, SWContext *ctx) {
        return (swCtxMap.insert(make_pair(pcb, ctx))).second;
    }
    void remContext(Addr pcb) {
        swCtxMap.erase(pcb);
        return;
    }

    virtual void dumpState(ExecContext *xc) const = 0;
#endif //FS_MEASURE

  public:
    ////////////////////////////////////////////
    //
    // STATIC GLOBAL SYSTEM LIST
    //
    ////////////////////////////////////////////

    static std::vector<System *> systemList;
    static int numSystemsRunning;

    static void printSystems();
};

#endif // __SYSTEM_HH__

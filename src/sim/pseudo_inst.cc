/*
 * Copyright (c) 2010-2011 ARM Limited
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
 * Copyright (c) 2011 Advanced Micro Devices, Inc.
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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
 */

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <fstream>
#include <string>

#include "arch/kernel_stats.hh"
#include "arch/vtophys.hh"
#include "base/debug.hh"
#include "base/output.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/quiesce_event.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "debug/Quiesce.hh"
#include "debug/WorkItems.hh"
#include "params/BaseCPU.hh"
#include "sim/full_system.hh"
#include "sim/pseudo_inst.hh"
#include "sim/serialize.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/stat_control.hh"
#include "sim/stats.hh"
#include "sim/system.hh"
#include "sim/vptr.hh"

using namespace std;

using namespace Stats;
using namespace TheISA;

namespace PseudoInst {

static inline void
panicFsOnlyPseudoInst(const char *name)
{
    panic("Pseudo inst \"%s\" is only available in Full System mode.");
}

void
arm(ThreadContext *tc)
{
    if (!FullSystem)
        panicFsOnlyPseudoInst("arm");

    if (tc->getKernelStats())
        tc->getKernelStats()->arm();
}

void
quiesce(ThreadContext *tc)
{
    if (!FullSystem)
        panicFsOnlyPseudoInst("quiesce");

    if (!tc->getCpuPtr()->params()->do_quiesce)
        return;

    DPRINTF(Quiesce, "%s: quiesce()\n", tc->getCpuPtr()->name());

    tc->suspend();
    if (tc->getKernelStats())
        tc->getKernelStats()->quiesce();
}

void
quiesceSkip(ThreadContext *tc)
{
    if (!FullSystem)
        panicFsOnlyPseudoInst("quiesceSkip");

    BaseCPU *cpu = tc->getCpuPtr();

    if (!cpu->params()->do_quiesce)
        return;

    EndQuiesceEvent *quiesceEvent = tc->getQuiesceEvent();

    Tick resume = curTick() + 1;

    cpu->reschedule(quiesceEvent, resume, true);

    DPRINTF(Quiesce, "%s: quiesceSkip() until %d\n",
            cpu->name(), resume);

    tc->suspend();
    if (tc->getKernelStats())
        tc->getKernelStats()->quiesce();
}

void
quiesceNs(ThreadContext *tc, uint64_t ns)
{
    if (!FullSystem)
        panicFsOnlyPseudoInst("quiesceNs");

    BaseCPU *cpu = tc->getCpuPtr();

    if (!cpu->params()->do_quiesce || ns == 0)
        return;

    EndQuiesceEvent *quiesceEvent = tc->getQuiesceEvent();

    Tick resume = curTick() + SimClock::Int::ns * ns;

    cpu->reschedule(quiesceEvent, resume, true);

    DPRINTF(Quiesce, "%s: quiesceNs(%d) until %d\n",
            cpu->name(), ns, resume);

    tc->suspend();
    if (tc->getKernelStats())
        tc->getKernelStats()->quiesce();
}

void
quiesceCycles(ThreadContext *tc, uint64_t cycles)
{
    if (!FullSystem)
        panicFsOnlyPseudoInst("quiesceCycles");

    BaseCPU *cpu = tc->getCpuPtr();

    if (!cpu->params()->do_quiesce || cycles == 0)
        return;

    EndQuiesceEvent *quiesceEvent = tc->getQuiesceEvent();

    Tick resume = curTick() + cpu->ticks(cycles);

    cpu->reschedule(quiesceEvent, resume, true);

    DPRINTF(Quiesce, "%s: quiesceCycles(%d) until %d\n",
            cpu->name(), cycles, resume);

    tc->suspend();
    if (tc->getKernelStats())
        tc->getKernelStats()->quiesce();
}

uint64_t
quiesceTime(ThreadContext *tc)
{
    if (!FullSystem) {
        panicFsOnlyPseudoInst("quiesceTime");
        return 0;
    }

    return (tc->readLastActivate() - tc->readLastSuspend()) /
        SimClock::Int::ns;
}

uint64_t
rpns(ThreadContext *tc)
{
    return curTick() / SimClock::Int::ns;
}

void
wakeCPU(ThreadContext *tc, uint64_t cpuid)
{
    System *sys = tc->getSystemPtr();
    ThreadContext *other_tc = sys->threadContexts[cpuid];
    if (other_tc->status() == ThreadContext::Suspended)
        other_tc->activate();
}

void
m5exit(ThreadContext *tc, Tick delay)
{
    Tick when = curTick() + delay * SimClock::Int::ns;
    exitSimLoop("m5_exit instruction encountered", 0, when);
}

void
loadsymbol(ThreadContext *tc)
{
    if (!FullSystem)
        panicFsOnlyPseudoInst("loadsymbol");

    const string &filename = tc->getCpuPtr()->system->params()->symbolfile;
    if (filename.empty()) {
        return;
    }

    std::string buffer;
    ifstream file(filename.c_str());

    if (!file)
        fatal("file error: Can't open symbol table file %s\n", filename);

    while (!file.eof()) {
        getline(file, buffer);

        if (buffer.empty())
            continue;

        string::size_type idx = buffer.find(' ');
        if (idx == string::npos)
            continue;

        string address = "0x" + buffer.substr(0, idx);
        eat_white(address);
        if (address.empty())
            continue;

        // Skip over letter and space
        string symbol = buffer.substr(idx + 3);
        eat_white(symbol);
        if (symbol.empty())
            continue;

        Addr addr;
        if (!to_number(address, addr))
            continue;

        if (!tc->getSystemPtr()->kernelSymtab->insert(addr, symbol))
            continue;


        DPRINTF(Loader, "Loaded symbol: %s @ %#llx\n", symbol, addr);
    }
    file.close();
}

void
addsymbol(ThreadContext *tc, Addr addr, Addr symbolAddr)
{
    if (!FullSystem)
        panicFsOnlyPseudoInst("addSymbol");

    char symb[100];
    CopyStringOut(tc, symb, symbolAddr, 100);
    std::string symbol(symb);

    DPRINTF(Loader, "Loaded symbol: %s @ %#llx\n", symbol, addr);

    tc->getSystemPtr()->kernelSymtab->insert(addr,symbol);
    debugSymbolTable->insert(addr,symbol);
}

uint64_t
initParam(ThreadContext *tc)
{
    if (!FullSystem) {
        panicFsOnlyPseudoInst("initParam");
        return 0;
    }

    return tc->getCpuPtr()->system->init_param;
}


void
resetstats(ThreadContext *tc, Tick delay, Tick period)
{
    if (!tc->getCpuPtr()->params()->do_statistics_insts)
        return;


    Tick when = curTick() + delay * SimClock::Int::ns;
    Tick repeat = period * SimClock::Int::ns;

    Stats::schedStatEvent(false, true, when, repeat);
}

void
dumpstats(ThreadContext *tc, Tick delay, Tick period)
{
    if (!tc->getCpuPtr()->params()->do_statistics_insts)
        return;


    Tick when = curTick() + delay * SimClock::Int::ns;
    Tick repeat = period * SimClock::Int::ns;

    Stats::schedStatEvent(true, false, when, repeat);
}

void
dumpresetstats(ThreadContext *tc, Tick delay, Tick period)
{
    if (!tc->getCpuPtr()->params()->do_statistics_insts)
        return;


    Tick when = curTick() + delay * SimClock::Int::ns;
    Tick repeat = period * SimClock::Int::ns;

    Stats::schedStatEvent(true, true, when, repeat);
}

void
m5checkpoint(ThreadContext *tc, Tick delay, Tick period)
{
    if (!tc->getCpuPtr()->params()->do_checkpoint_insts)
        return;

    Tick when = curTick() + delay * SimClock::Int::ns;
    Tick repeat = period * SimClock::Int::ns;

    exitSimLoop("checkpoint", 0, when, repeat);
}

uint64_t
readfile(ThreadContext *tc, Addr vaddr, uint64_t len, uint64_t offset)
{
    if (!FullSystem) {
        panicFsOnlyPseudoInst("readfile");
        return 0;
    }

    const string &file = tc->getSystemPtr()->params()->readfile;
    if (file.empty()) {
        return ULL(0);
    }

    uint64_t result = 0;

    int fd = ::open(file.c_str(), O_RDONLY, 0);
    if (fd < 0)
        panic("could not open file %s\n", file);

    if (::lseek(fd, offset, SEEK_SET) < 0)
        panic("could not seek: %s", strerror(errno));

    char *buf = new char[len];
    char *p = buf;
    while (len > 0) {
        int bytes = ::read(fd, p, len);
        if (bytes <= 0)
            break;

        p += bytes;
        result += bytes;
        len -= bytes;
    }

    close(fd);
    CopyIn(tc, vaddr, buf, result);
    delete [] buf;
    return result;
}

uint64_t
writefile(ThreadContext *tc, Addr vaddr, uint64_t len, uint64_t offset,
            Addr filename_addr)
{
    ostream *os;

    // copy out target filename
    char fn[100];
    std::string filename;
    CopyStringOut(tc, fn, filename_addr, 100);
    filename = std::string(fn);

    if (offset == 0) {
        // create a new file (truncate)
        os = simout.create(filename, true);
    } else {
        // do not truncate file if offset is non-zero
        // (ios::in flag is required as well to keep the existing data
        //  intact, otherwise existing data will be zeroed out.)
        os = simout.openFile(simout.directory() + filename,
                            ios::in | ios::out | ios::binary);
    }
    if (!os)
        panic("could not open file %s\n", filename);

    // seek to offset
    os->seekp(offset);

    // copy out data and write to file
    char *buf = new char[len];
    CopyOut(tc, buf, vaddr, len);
    os->write(buf, len);
    if (os->fail() || os->bad())
        panic("Error while doing writefile!\n");

    simout.close(os);

    delete [] buf;

    return len;
}

void
debugbreak(ThreadContext *tc)
{
    Debug::breakpoint();
}

void
switchcpu(ThreadContext *tc)
{
    exitSimLoop("switchcpu");
}

//
// This function is executed when annotated work items begin.  Depending on 
// what the user specified at the command line, the simulation may exit and/or
// take a checkpoint when a certain work item begins.
//
void
workbegin(ThreadContext *tc, uint64_t workid, uint64_t threadid)
{
    tc->getCpuPtr()->workItemBegin();
    System *sys = tc->getSystemPtr();
    const System::Params *params = sys->params();
    sys->workItemBegin(threadid, workid);

    DPRINTF(WorkItems, "Work Begin workid: %d, threadid %d\n", workid, 
            threadid);

    //
    // If specified, determine if this is the specific work item the user
    // identified
    //
    if (params->work_item_id == -1 || params->work_item_id == workid) {

        uint64_t systemWorkBeginCount = sys->incWorkItemsBegin();
        int cpuId = tc->getCpuPtr()->cpuId();

        if (params->work_cpus_ckpt_count != 0 &&
            sys->markWorkItem(cpuId) >= params->work_cpus_ckpt_count) {
            //
            // If active cpus equals checkpoint count, create checkpoint
            //
            exitSimLoop("checkpoint");
        }

        if (systemWorkBeginCount == params->work_begin_ckpt_count) {
            //
            // Note: the string specified as the cause of the exit event must
            // exactly equal "checkpoint" inorder to create a checkpoint
            //
            exitSimLoop("checkpoint");
        }

        if (systemWorkBeginCount == params->work_begin_exit_count) {
            //
            // If a certain number of work items started, exit simulation
            //
            exitSimLoop("work started count reach");
        }

        if (cpuId == params->work_begin_cpu_id_exit) {
            //
            // If work started on the cpu id specified, exit simulation
            //
            exitSimLoop("work started on specific cpu");
        }
    }
}

//
// This function is executed when annotated work items end.  Depending on 
// what the user specified at the command line, the simulation may exit and/or
// take a checkpoint when a certain work item ends.
//
void
workend(ThreadContext *tc, uint64_t workid, uint64_t threadid)
{
    tc->getCpuPtr()->workItemEnd();
    System *sys = tc->getSystemPtr();
    const System::Params *params = sys->params();
    sys->workItemEnd(threadid, workid);

    DPRINTF(WorkItems, "Work End workid: %d, threadid %d\n", workid, threadid);

    //
    // If specified, determine if this is the specific work item the user
    // identified
    //
    if (params->work_item_id == -1 || params->work_item_id == workid) {

        uint64_t systemWorkEndCount = sys->incWorkItemsEnd();
        int cpuId = tc->getCpuPtr()->cpuId();

        if (params->work_cpus_ckpt_count != 0 &&
            sys->markWorkItem(cpuId) >= params->work_cpus_ckpt_count) {
            //
            // If active cpus equals checkpoint count, create checkpoint
            //
            exitSimLoop("checkpoint");
        }

        if (params->work_end_ckpt_count != 0 &&
            systemWorkEndCount == params->work_end_ckpt_count) {
            //
            // If total work items completed equals checkpoint count, create
            // checkpoint
            //
            exitSimLoop("checkpoint");
        }

        if (params->work_end_exit_count != 0 &&
            systemWorkEndCount == params->work_end_exit_count) {
            //
            // If total work items completed equals exit count, exit simulation
            //
            exitSimLoop("work items exit count reached");
        }
    }
}

} // namespace PseudoInst

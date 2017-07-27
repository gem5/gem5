/*
 * Copyright (c) 2010-2012, 2015, 2017 ARM Limited
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

#include "sim/pseudo_inst.hh"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <fstream>
#include <string>
#include <vector>

#include <gem5/asm/generic/m5ops.h>

#include "arch/kernel_stats.hh"
#include "arch/pseudo_inst.hh"
#include "arch/utility.hh"
#include "arch/vtophys.hh"
#include "base/debug.hh"
#include "base/output.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/quiesce_event.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "debug/PseudoInst.hh"
#include "debug/Quiesce.hh"
#include "debug/WorkItems.hh"
#include "dev/net/dist_iface.hh"
#include "params/BaseCPU.hh"
#include "sim/full_system.hh"
#include "sim/initparam_keys.hh"
#include "sim/process.hh"
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

uint64_t
pseudoInst(ThreadContext *tc, uint8_t func, uint8_t subfunc)
{
    uint64_t args[4];

    DPRINTF(PseudoInst, "PseudoInst::pseudoInst(%i, %i)\n", func, subfunc);

    // We need to do this in a slightly convoluted way since
    // getArgument() might have side-effects on arg_num. We could have
    // used the Argument class, but due to the possible side effects
    // from getArgument, it'd most likely break.
    int arg_num(0);
    for (int i = 0; i < sizeof(args) / sizeof(*args); ++i) {
        args[arg_num] = getArgument(tc, arg_num, sizeof(uint64_t), false);
        ++arg_num;
    }

    switch (func) {
      case M5OP_ARM:
        arm(tc);
        break;

      case M5OP_QUIESCE:
        quiesce(tc);
        break;

      case M5OP_QUIESCE_NS:
        quiesceNs(tc, args[0]);
        break;

      case M5OP_QUIESCE_CYCLE:
        quiesceCycles(tc, args[0]);
        break;

      case M5OP_QUIESCE_TIME:
        return quiesceTime(tc);

      case M5OP_RPNS:
        return rpns(tc);

      case M5OP_WAKE_CPU:
        wakeCPU(tc, args[0]);
        break;

      case M5OP_EXIT:
        m5exit(tc, args[0]);
        break;

      case M5OP_FAIL:
        m5fail(tc, args[0], args[1]);
        break;

      case M5OP_INIT_PARAM:
        return initParam(tc, args[0], args[1]);

      case M5OP_LOAD_SYMBOL:
        loadsymbol(tc);
        break;

      case M5OP_RESET_STATS:
        resetstats(tc, args[0], args[1]);
        break;

      case M5OP_DUMP_STATS:
        dumpstats(tc, args[0], args[1]);
        break;

      case M5OP_DUMP_RESET_STATS:
        dumpresetstats(tc, args[0], args[1]);
        break;

      case M5OP_CHECKPOINT:
        m5checkpoint(tc, args[0], args[1]);
        break;

      case M5OP_WRITE_FILE:
        return writefile(tc, args[0], args[1], args[2], args[3]);

      case M5OP_READ_FILE:
        return readfile(tc, args[0], args[1], args[2]);

      case M5OP_DEBUG_BREAK:
        debugbreak(tc);
        break;

      case M5OP_SWITCH_CPU:
        switchcpu(tc);
        break;

      case M5OP_ADD_SYMBOL:
        addsymbol(tc, args[0], args[1]);
        break;

      case M5OP_PANIC:
        panic("M5 panic instruction called at %s\n", tc->pcState());

      case M5OP_WORK_BEGIN:
        workbegin(tc, args[0], args[1]);
        break;

      case M5OP_WORK_END:
        workend(tc, args[0], args[1]);
        break;

      case M5OP_ANNOTATE:
      case M5OP_RESERVED2:
      case M5OP_RESERVED3:
      case M5OP_RESERVED4:
      case M5OP_RESERVED5:
        warn("Unimplemented m5 op (0x%x)\n", func);
        break;

      /* SE mode functions */
      case M5OP_SE_SYSCALL:
        m5Syscall(tc);
        break;

      case M5OP_SE_PAGE_FAULT:
        m5PageFault(tc);
        break;

      /* dist-gem5 functions */
      case M5OP_DIST_TOGGLE_SYNC:
        togglesync(tc);
        break;

      default:
        warn("Unhandled m5 op: 0x%x\n", func);
        break;
    }

    return 0;
}

void
arm(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::arm()\n");
    if (!FullSystem)
        panicFsOnlyPseudoInst("arm");

    if (tc->getKernelStats())
        tc->getKernelStats()->arm();
}

void
quiesce(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::quiesce()\n");
    tc->quiesce();
}

void
quiesceSkip(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::quiesceSkip()\n");
    tc->quiesceTick(tc->getCpuPtr()->nextCycle() + 1);
}

void
quiesceNs(ThreadContext *tc, uint64_t ns)
{
    DPRINTF(PseudoInst, "PseudoInst::quiesceNs(%i)\n", ns);
    tc->quiesceTick(curTick() + SimClock::Int::ns * ns);
}

void
quiesceCycles(ThreadContext *tc, uint64_t cycles)
{
    DPRINTF(PseudoInst, "PseudoInst::quiesceCycles(%i)\n", cycles);
    tc->quiesceTick(tc->getCpuPtr()->clockEdge(Cycles(cycles)));
}

uint64_t
quiesceTime(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::quiesceTime()\n");

    return (tc->readLastActivate() - tc->readLastSuspend()) /
        SimClock::Int::ns;
}

uint64_t
rpns(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::rpns()\n");
    return curTick() / SimClock::Int::ns;
}

void
wakeCPU(ThreadContext *tc, uint64_t cpuid)
{
    DPRINTF(PseudoInst, "PseudoInst::wakeCPU(%i)\n", cpuid);
    System *sys = tc->getSystemPtr();

    if (sys->numContexts() <= cpuid) {
        warn("PseudoInst::wakeCPU(%i), cpuid greater than number of contexts"
             "(%i)\n",cpuid, sys->numContexts());
        return;
    }

    ThreadContext *other_tc = sys->threadContexts[cpuid];
    if (other_tc->status() == ThreadContext::Suspended)
        other_tc->activate();
}

void
m5exit(ThreadContext *tc, Tick delay)
{
    DPRINTF(PseudoInst, "PseudoInst::m5exit(%i)\n", delay);
    if (DistIface::readyToExit(delay)) {
        Tick when = curTick() + delay * SimClock::Int::ns;
        exitSimLoop("m5_exit instruction encountered", 0, when, 0, true);
    }
}

void
m5fail(ThreadContext *tc, Tick delay, uint64_t code)
{
    DPRINTF(PseudoInst, "PseudoInst::m5fail(%i, %i)\n", delay, code);
    Tick when = curTick() + delay * SimClock::Int::ns;
    exitSimLoop("m5_fail instruction encountered", code, when, 0, true);
}

void
loadsymbol(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::loadsymbol()\n");
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
    DPRINTF(PseudoInst, "PseudoInst::addsymbol(0x%x, 0x%x)\n",
            addr, symbolAddr);
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
initParam(ThreadContext *tc, uint64_t key_str1, uint64_t key_str2)
{
    DPRINTF(PseudoInst, "PseudoInst::initParam() key:%s%s\n", (char *)&key_str1,
            (char *)&key_str2);
    if (!FullSystem) {
        panicFsOnlyPseudoInst("initParam");
        return 0;
    }

    // The key parameter string is passed in via two 64-bit registers. We copy
    // out the characters from the 64-bit integer variables here and concatenate
    // them in the key_str character buffer
    const int len = 2 * sizeof(uint64_t) + 1;
    char key_str[len];
    memset(key_str, '\0', len);
    if (key_str1 == 0) {
        assert(key_str2 == 0);
    } else {
        strncpy(key_str, (char *)&key_str1, sizeof(uint64_t));
    }

    if (strlen(key_str) == sizeof(uint64_t)) {
        strncpy(key_str + sizeof(uint64_t), (char *)&key_str2,
                sizeof(uint64_t));
    } else {
        assert(key_str2 == 0);
    }

    // Compare the key parameter with the known values to select the return
    // value
    uint64_t val;
    if (strcmp(key_str, InitParamKey::DEFAULT) == 0) {
        val = tc->getCpuPtr()->system->init_param;
    } else if (strcmp(key_str, InitParamKey::DIST_RANK) == 0) {
        val = DistIface::rankParam();
    } else if (strcmp(key_str, InitParamKey::DIST_SIZE) == 0) {
        val = DistIface::sizeParam();
    } else {
        panic("Unknown key for initparam pseudo instruction:\"%s\"", key_str);
    }
    return val;
}


void
resetstats(ThreadContext *tc, Tick delay, Tick period)
{
    DPRINTF(PseudoInst, "PseudoInst::resetstats(%i, %i)\n", delay, period);
    if (!tc->getCpuPtr()->params()->do_statistics_insts)
        return;


    Tick when = curTick() + delay * SimClock::Int::ns;
    Tick repeat = period * SimClock::Int::ns;

    Stats::schedStatEvent(false, true, when, repeat);
}

void
dumpstats(ThreadContext *tc, Tick delay, Tick period)
{
    DPRINTF(PseudoInst, "PseudoInst::dumpstats(%i, %i)\n", delay, period);
    if (!tc->getCpuPtr()->params()->do_statistics_insts)
        return;


    Tick when = curTick() + delay * SimClock::Int::ns;
    Tick repeat = period * SimClock::Int::ns;

    Stats::schedStatEvent(true, false, when, repeat);
}

void
dumpresetstats(ThreadContext *tc, Tick delay, Tick period)
{
    DPRINTF(PseudoInst, "PseudoInst::dumpresetstats(%i, %i)\n", delay, period);
    if (!tc->getCpuPtr()->params()->do_statistics_insts)
        return;


    Tick when = curTick() + delay * SimClock::Int::ns;
    Tick repeat = period * SimClock::Int::ns;

    Stats::schedStatEvent(true, true, when, repeat);
}

void
m5checkpoint(ThreadContext *tc, Tick delay, Tick period)
{
    DPRINTF(PseudoInst, "PseudoInst::m5checkpoint(%i, %i)\n", delay, period);
    if (!tc->getCpuPtr()->params()->do_checkpoint_insts)
        return;

    if (DistIface::readyToCkpt(delay, period)) {
        Tick when = curTick() + delay * SimClock::Int::ns;
        Tick repeat = period * SimClock::Int::ns;
        exitSimLoop("checkpoint", 0, when, repeat);
    }
}

uint64_t
readfile(ThreadContext *tc, Addr vaddr, uint64_t len, uint64_t offset)
{
    DPRINTF(PseudoInst, "PseudoInst::readfile(0x%x, 0x%x, 0x%x)\n",
            vaddr, len, offset);
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
    DPRINTF(PseudoInst, "PseudoInst::writefile(0x%x, 0x%x, 0x%x, 0x%x)\n",
            vaddr, len, offset, filename_addr);

    // copy out target filename
    char fn[100];
    std::string filename;
    CopyStringOut(tc, fn, filename_addr, 100);
    filename = std::string(fn);

    OutputStream *out;
    if (offset == 0) {
        // create a new file (truncate)
        out = simout.create(filename, true, true);
    } else {
        // do not truncate file if offset is non-zero
        // (ios::in flag is required as well to keep the existing data
        //  intact, otherwise existing data will be zeroed out.)
        out = simout.open(filename, ios::in | ios::out | ios::binary, true);
    }

    ostream *os(out->stream());
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

    simout.close(out);

    delete [] buf;

    return len;
}

void
debugbreak(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::debugbreak()\n");
    Debug::breakpoint();
}

void
switchcpu(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::switchcpu()\n");
    exitSimLoop("switchcpu");
}

void
togglesync(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::togglesync()\n");
    DistIface::toggleSync(tc);
}

//
// This function is executed when annotated work items begin.  Depending on
// what the user specified at the command line, the simulation may exit and/or
// take a checkpoint when a certain work item begins.
//
void
workbegin(ThreadContext *tc, uint64_t workid, uint64_t threadid)
{
    DPRINTF(PseudoInst, "PseudoInst::workbegin(%i, %i)\n", workid, threadid);
    System *sys = tc->getSystemPtr();
    const System::Params *params = sys->params();

    if (params->exit_on_work_items) {
        exitSimLoop("workbegin", static_cast<int>(workid));
        return;
    }

    DPRINTF(WorkItems, "Work Begin workid: %d, threadid %d\n", workid,
            threadid);
    tc->getCpuPtr()->workItemBegin();
    sys->workItemBegin(threadid, workid);

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
    DPRINTF(PseudoInst, "PseudoInst::workend(%i, %i)\n", workid, threadid);
    System *sys = tc->getSystemPtr();
    const System::Params *params = sys->params();

    if (params->exit_on_work_items) {
        exitSimLoop("workend", static_cast<int>(workid));
        return;
    }

    DPRINTF(WorkItems, "Work End workid: %d, threadid %d\n", workid, threadid);
    tc->getCpuPtr()->workItemEnd();
    sys->workItemEnd(threadid, workid);

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

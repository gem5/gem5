/*
 * Copyright (c) 2010-2012, 2015, 2017 ARM Limited
 * Copyright (c) 2020 Barkhausen Institut
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
 */

#include "sim/pseudo_inst.hh"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <fstream>
#include <string>
#include <vector>

#include "base/debug.hh"
#include "base/output.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "debug/Quiesce.hh"
#include "debug/WorkItems.hh"
#include "dev/net/dist_iface.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "params/BaseCPU.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/serialize.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/stat_control.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace statistics;

GEM5_DEPRECATED_NAMESPACE(PseudoInst, pseudo_inst);
namespace pseudo_inst
{

/**
 * Unique keys to retrieve various params by the initParam pseudo inst.
 *
 * @note Each key may be at most 16 characters (because we use
 * two 64-bit registers to pass in the key to the initparam function).
 */
namespace
{

/**
 *  The default key (empty string)
 */
const std::string DEFAULT = "";
/**
 *  Unique key for "rank" param (distributed gem5 runs)
 */
const std::string DIST_RANK = "dist-rank";
/**
 *  Unique key for "size" param (distributed gem5 runs)
 */
const std::string DIST_SIZE = "dist-size";

} // anonymous namespace

void
arm(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::arm()\n");

    auto *workload = tc->getSystemPtr()->workload;
    if (workload)
        workload->recordArm();
}

void
quiesce(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::quiesce()\n");
    tc->quiesce();
}

void
quiesceSkip(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::quiesceSkip()\n");
    tc->quiesceTick(tc->getCpuPtr()->nextCycle() + 1);
}

void
quiesceNs(ThreadContext *tc, uint64_t ns)
{
    DPRINTF(PseudoInst, "pseudo_inst::quiesceNs(%i)\n", ns);
    tc->quiesceTick(curTick() + sim_clock::as_int::ns * ns);
}

void
quiesceCycles(ThreadContext *tc, uint64_t cycles)
{
    DPRINTF(PseudoInst, "pseudo_inst::quiesceCycles(%i)\n", cycles);
    tc->quiesceTick(tc->getCpuPtr()->clockEdge(Cycles(cycles)));
}

uint64_t
quiesceTime(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::quiesceTime()\n");

    return (tc->readLastActivate() - tc->readLastSuspend()) /
        sim_clock::as_int::ns;
}

uint64_t
rpns(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::rpns()\n");
    return curTick() / sim_clock::as_int::ns;
}

void
wakeCPU(ThreadContext *tc, uint64_t cpuid)
{
    DPRINTF(PseudoInst, "pseudo_inst::wakeCPU(%i)\n", cpuid);
    System *sys = tc->getSystemPtr();

    if (sys->threads.size() <= cpuid) {
        warn("pseudo_inst::wakeCPU(%i), cpuid greater than number of contexts"
             "(%i)\n", cpuid, sys->threads.size());
        return;
    }

    ThreadContext *other_tc = sys->threads[cpuid];
    if (other_tc->status() == ThreadContext::Suspended)
        other_tc->activate();
}

void
m5exit(ThreadContext *tc, Tick delay)
{
    DPRINTF(PseudoInst, "pseudo_inst::m5exit(%i)\n", delay);
    if (DistIface::readyToExit(delay)) {
        Tick when = curTick() + delay * sim_clock::as_int::ns;
        exitSimLoop("m5_exit instruction encountered", 0, when, 0, true);
    }
}

// m5sum is for sanity checking the gem5 op interface.
uint64_t
m5sum(ThreadContext *tc, uint64_t a, uint64_t b, uint64_t c,
                         uint64_t d, uint64_t e, uint64_t f)
{
    DPRINTF(PseudoInst, "pseudo_inst::m5sum(%#x, %#x, %#x, %#x, %#x, %#x)\n",
            a, b, c, d, e, f);
    return a + b + c + d + e + f;
}

void
m5fail(ThreadContext *tc, Tick delay, uint64_t code)
{
    DPRINTF(PseudoInst, "pseudo_inst::m5fail(%i, %i)\n", delay, code);
    Tick when = curTick() + delay * sim_clock::as_int::ns;
    exitSimLoop("m5_fail instruction encountered", code, when, 0, true);
}

void
loadsymbol(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::loadsymbol()\n");

    const std::string &filename = tc->getCpuPtr()->system->params().symbolfile;
    if (filename.empty()) {
        return;
    }

    std::string buffer;
    std::ifstream file(filename.c_str());

    if (!file)
        fatal("file error: Can't open symbol table file %s\n", filename);

    while (!file.eof()) {
        getline(file, buffer);

        if (buffer.empty())
            continue;

        std::string::size_type idx = buffer.find(' ');
        if (idx == std::string::npos)
            continue;

        std::string address = "0x" + buffer.substr(0, idx);
        eat_white(address);
        if (address.empty())
            continue;

        // Skip over letter and space
        std::string symbol = buffer.substr(idx + 3);
        eat_white(symbol);
        if (symbol.empty())
            continue;

        Addr addr;
        if (!to_number(address, addr))
            continue;

        if (!tc->getSystemPtr()->workload->insertSymbol(
                    { loader::Symbol::Binding::Global, symbol, addr })) {
            continue;
        }


        DPRINTF(Loader, "Loaded symbol: %s @ %#llx\n", symbol, addr);
    }
    file.close();
}

void
addsymbol(ThreadContext *tc, Addr addr, Addr symbolAddr)
{
    DPRINTF(PseudoInst, "pseudo_inst::addsymbol(0x%x, 0x%x)\n",
            addr, symbolAddr);

    std::string symbol;
    TranslatingPortProxy fs_proxy(tc);
    SETranslatingPortProxy se_proxy(tc);
    PortProxy &virt_proxy = FullSystem ? fs_proxy : se_proxy;

    virt_proxy.readString(symbol, symbolAddr);

    DPRINTF(Loader, "Loaded symbol: %s @ %#llx\n", symbol, addr);

    tc->getSystemPtr()->workload->insertSymbol(
            { loader::Symbol::Binding::Global, symbol, addr });
    loader::debugSymbolTable.insert(
            { loader::Symbol::Binding::Global, symbol, addr });
}

uint64_t
initParam(ThreadContext *tc, uint64_t key_str1, uint64_t key_str2)
{
    DPRINTF(PseudoInst, "pseudo_inst::initParam() key:%s%s\n",
        (char *)&key_str1, (char *)&key_str2);

    // The key parameter string is passed in via two 64-bit registers. We copy
    // out the characters from the 64-bit integer variables here, and
    // concatenate them in the key character buffer
    const int len = 2 * sizeof(uint64_t) + 1;
    char key[len];
    std::memset(key, '\0', len);

    std::array<uint64_t, 2> key_regs = {{ key_str1, key_str2 }};
    key_regs = letoh(key_regs);
    std::memcpy(key, key_regs.data(), sizeof(key_regs));

    // Check key parameter to figure out what to return.
    const std::string key_str(key);
    if (key == DEFAULT)
        return tc->getCpuPtr()->system->init_param;
    else if (key == DIST_RANK)
        return DistIface::rankParam();
    else if (key == DIST_SIZE)
        return DistIface::sizeParam();
    else
        panic("Unknown key for initparam pseudo instruction:\"%s\"", key_str);
}


void
resetstats(ThreadContext *tc, Tick delay, Tick period)
{
    DPRINTF(PseudoInst, "pseudo_inst::resetstats(%i, %i)\n", delay, period);
    if (!tc->getCpuPtr()->params().do_statistics_insts)
        return;


    Tick when = curTick() + delay * sim_clock::as_int::ns;
    Tick repeat = period * sim_clock::as_int::ns;

    statistics::schedStatEvent(false, true, when, repeat);
}

void
dumpstats(ThreadContext *tc, Tick delay, Tick period)
{
    DPRINTF(PseudoInst, "pseudo_inst::dumpstats(%i, %i)\n", delay, period);
    if (!tc->getCpuPtr()->params().do_statistics_insts)
        return;


    Tick when = curTick() + delay * sim_clock::as_int::ns;
    Tick repeat = period * sim_clock::as_int::ns;

    statistics::schedStatEvent(true, false, when, repeat);
}

void
dumpresetstats(ThreadContext *tc, Tick delay, Tick period)
{
    DPRINTF(PseudoInst, "pseudo_inst::dumpresetstats(%i, %i)\n", delay,
        period);
    if (!tc->getCpuPtr()->params().do_statistics_insts)
        return;


    Tick when = curTick() + delay * sim_clock::as_int::ns;
    Tick repeat = period * sim_clock::as_int::ns;

    statistics::schedStatEvent(true, true, when, repeat);
}

void
m5checkpoint(ThreadContext *tc, Tick delay, Tick period)
{
    DPRINTF(PseudoInst, "pseudo_inst::m5checkpoint(%i, %i)\n", delay, period);
    if (!tc->getCpuPtr()->params().do_checkpoint_insts)
        return;

    if (DistIface::readyToCkpt(delay, period)) {
        Tick when = curTick() + delay * sim_clock::as_int::ns;
        Tick repeat = period * sim_clock::as_int::ns;
        exitSimLoop("checkpoint", 0, when, repeat);
    }
}

uint64_t
readfile(ThreadContext *tc, Addr vaddr, uint64_t len, uint64_t offset)
{
    DPRINTF(PseudoInst, "pseudo_inst::readfile(0x%x, 0x%x, 0x%x)\n",
            vaddr, len, offset);

    const std::string &file = tc->getSystemPtr()->params().readfile;
    if (file.empty()) {
        return 0;
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
    TranslatingPortProxy fs_proxy(tc);
    SETranslatingPortProxy se_proxy(tc);
    PortProxy &virt_proxy = FullSystem ? fs_proxy : se_proxy;

    virt_proxy.writeBlob(vaddr, buf, result);
    delete [] buf;
    return result;
}

uint64_t
writefile(ThreadContext *tc, Addr vaddr, uint64_t len, uint64_t offset,
            Addr filename_addr)
{
    DPRINTF(PseudoInst, "pseudo_inst::writefile(0x%x, 0x%x, 0x%x, 0x%x)\n",
            vaddr, len, offset, filename_addr);

    // copy out target filename
    std::string filename;
    TranslatingPortProxy fs_proxy(tc);
    SETranslatingPortProxy se_proxy(tc);
    PortProxy &virt_proxy = FullSystem ? fs_proxy : se_proxy;

    virt_proxy.readString(filename, filename_addr);

    OutputStream *out;
    if (offset == 0) {
        // create a new file (truncate)
        out = simout.create(filename, true, true);
    } else {
        // do not truncate file if offset is non-zero
        // (ios::in flag is required as well to keep the existing data
        //  intact, otherwise existing data will be zeroed out.)
        out = simout.open(filename,
                std::ios::in | std::ios::out | std::ios::binary, true);
    }

    std::ostream *os(out->stream());
    if (!os)
        panic("could not open file %s\n", filename);

    if (offset != 0) {
        // seek to offset
        os->seekp(offset);
    }

    // copy out data and write to file
    char *buf = new char[len];

    virt_proxy.readBlob(vaddr, buf, len);
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
    DPRINTF(PseudoInst, "pseudo_inst::debugbreak()\n");
    debug::breakpoint();
}

void
switchcpu(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::switchcpu()\n");
    exitSimLoop("switchcpu");
}

void
togglesync(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::togglesync()\n");
    DistIface::toggleSync(tc);
}

void
triggerWorkloadEvent(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "pseudo_inst::triggerWorkloadEvent()\n");
    tc->getSystemPtr()->workload->event(tc);
}

//
// This function is executed when annotated work items begin.  Depending on
// what the user specified at the command line, the simulation may exit and/or
// take a checkpoint when a certain work item begins.
//
void
workbegin(ThreadContext *tc, uint64_t workid, uint64_t threadid)
{
    DPRINTF(PseudoInst, "pseudo_inst::workbegin(%i, %i)\n", workid, threadid);
    System *sys = tc->getSystemPtr();
    const System::Params &params = sys->params();

    if (params.exit_on_work_items) {
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
    if (params.work_item_id == -1 || params.work_item_id == workid) {

        uint64_t systemWorkBeginCount = sys->incWorkItemsBegin();
        int cpuId = tc->getCpuPtr()->cpuId();

        if (params.work_cpus_ckpt_count != 0 &&
            sys->markWorkItem(cpuId) >= params.work_cpus_ckpt_count) {
            //
            // If active cpus equals checkpoint count, create checkpoint
            //
            exitSimLoop("checkpoint");
        }

        if (systemWorkBeginCount == params.work_begin_ckpt_count) {
            //
            // Note: the string specified as the cause of the exit event must
            // exactly equal "checkpoint" inorder to create a checkpoint
            //
            exitSimLoop("checkpoint");
        }

        if (systemWorkBeginCount == params.work_begin_exit_count) {
            //
            // If a certain number of work items started, exit simulation
            //
            exitSimLoop("work started count reach");
        }

        if (cpuId == params.work_begin_cpu_id_exit) {
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
    DPRINTF(PseudoInst, "pseudo_inst::workend(%i, %i)\n", workid, threadid);
    System *sys = tc->getSystemPtr();
    const System::Params &params = sys->params();

    if (params.exit_on_work_items) {
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
    if (params.work_item_id == -1 || params.work_item_id == workid) {

        uint64_t systemWorkEndCount = sys->incWorkItemsEnd();
        int cpuId = tc->getCpuPtr()->cpuId();

        if (params.work_cpus_ckpt_count != 0 &&
            sys->markWorkItem(cpuId) >= params.work_cpus_ckpt_count) {
            //
            // If active cpus equals checkpoint count, create checkpoint
            //
            exitSimLoop("checkpoint");
        }

        if (params.work_end_ckpt_count != 0 &&
            systemWorkEndCount == params.work_end_ckpt_count) {
            //
            // If total work items completed equals checkpoint count, create
            // checkpoint
            //
            exitSimLoop("checkpoint");
        }

        if (params.work_end_exit_count != 0 &&
            systemWorkEndCount == params.work_end_exit_count) {
            //
            // If total work items completed equals exit count, exit simulation
            //
            exitSimLoop("work items exit count reached");
        }
    }
}

} // namespace pseudo_inst
} // namespace gem5

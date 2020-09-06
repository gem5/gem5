/*
 * Copyright (c) 2010-2013, 2016 ARM Limited
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
 * Copyright (c) 2002-2006 The Regents of The University of Michigan
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

#include "arch/arm/linux/fs_workload.hh"

#include "arch/arm/isa_traits.hh"
#include "arch/arm/linux/atag.hh"
#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/linux/threadinfo.hh"
#include "base/loader/dtb_file.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/pc_event.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "kern/linux/events.hh"
#include "kern/linux/helpers.hh"
#include "kern/system_events.hh"
#include "mem/physical.hh"
#include "sim/stat_control.hh"

using namespace Linux;

namespace ArmISA
{

FsLinux::FsLinux(Params *p) : ArmISA::FsWorkload(p),
    enableContextSwitchStatsDump(p->enable_context_switch_stats_dump)
{}

void
FsLinux::initState()
{
    ArmISA::FsWorkload::initState();

    // Load symbols at physical address, we might not want
    // to do this permanently, for but early bootup work
    // it is helpful.
    if (params()->early_kernel_symbols) {
        auto phys_globals = kernelObj->symtab().globals()->mask(_loadAddrMask);
        kernelSymtab.insert(*phys_globals);
        Loader::debugSymbolTable.insert(*phys_globals);
    }

    // Setup boot data structure
    // Check if the kernel image has a symbol that tells us it supports
    // device trees.
    bool kernel_has_fdt_support =
        kernelSymtab.find("unflatten_device_tree") != kernelSymtab.end();
    bool dtb_file_specified = params()->dtb_filename != "";

    if (kernel_has_fdt_support && dtb_file_specified) {
        // Kernel supports flattened device tree and dtb file specified.
        // Using Device Tree Blob to describe system configuration.
        inform("Loading DTB file: %s at address %#x\n", params()->dtb_filename,
                params()->atags_addr + _loadAddrOffset);

        auto *dtb_file = new ::Loader::DtbFile(params()->dtb_filename);

        if (!dtb_file->addBootCmdLine(
                    commandLine.c_str(), commandLine.size())) {
            warn("couldn't append bootargs to DTB file: %s\n",
                 params()->dtb_filename);
        }

        dtb_file->buildImage().
            offset(params()->atags_addr + _loadAddrOffset).
            write(system->physProxy);
        delete dtb_file;
    } else {
        // Using ATAGS
        // Warn if the kernel supports FDT and we haven't specified one
        if (kernel_has_fdt_support) {
            assert(!dtb_file_specified);
            warn("Kernel supports device tree, but no DTB file specified\n");
        }
        // Warn if the kernel doesn't support FDT and we have specified one
        if (dtb_file_specified) {
            assert(!kernel_has_fdt_support);
            warn("DTB file specified, but no device tree support in kernel\n");
        }

        AtagCore ac;
        ac.flags(1); // read-only
        ac.pagesize(8192);
        ac.rootdev(0);

        AddrRangeList atagRanges = system->getPhysMem().getConfAddrRanges();
        fatal_if(atagRanges.size() != 1,
                 "Expected a single ATAG memory entry but got %d",
                 atagRanges.size());
        AtagMem am;
        am.memSize(atagRanges.begin()->size());
        am.memStart(atagRanges.begin()->start());

        AtagCmdline ad;
        ad.cmdline(commandLine);

        DPRINTF(Loader, "boot command line %d bytes: %s\n",
                ad.size() << 2, commandLine);

        AtagNone an;

        uint32_t size = ac.size() + am.size() + ad.size() + an.size();
        uint32_t offset = 0;
        uint8_t *boot_data = new uint8_t[size << 2];

        offset += ac.copyOut(boot_data + offset);
        offset += am.copyOut(boot_data + offset);
        offset += ad.copyOut(boot_data + offset);
        offset += an.copyOut(boot_data + offset);

        DPRINTF(Loader, "Boot atags was %d bytes in total\n", size << 2);
        DDUMP(Loader, boot_data, size << 2);

        system->physProxy.writeBlob(params()->atags_addr + _loadAddrOffset,
                                    boot_data, size << 2);

        delete[] boot_data;
    }

    // Kernel boot requirements to set up r0, r1 and r2 in ARMv7
    for (auto *tc: system->threads) {
        tc->setIntReg(0, 0);
        tc->setIntReg(1, params()->machine_type);
        tc->setIntReg(2, params()->atags_addr + _loadAddrOffset);
    }
}

FsLinux::~FsLinux()
{
    delete debugPrintk;
    delete skipUDelay;
    delete skipConstUDelay;
    delete kernelOops;
    delete kernelPanic;

    delete dumpStats;
}

void
FsLinux::startup()
{
    FsWorkload::startup();

    if (enableContextSwitchStatsDump) {
        if (getArch() == Loader::Arm64)
            dumpStats = addKernelFuncEvent<DumpStats64>("__switch_to");
        else
            dumpStats = addKernelFuncEvent<DumpStats>("__switch_to");

        panic_if(!dumpStats, "dumpStats not created!");

        std::string task_filename = "tasks.txt";
        taskFile = simout.create(name() + "." + task_filename);

        for (auto *tc: system->threads) {
            uint32_t pid = tc->getCpuPtr()->getPid();
            if (pid != BaseCPU::invldPid) {
                mapPid(tc, pid);
                tc->getCpuPtr()->taskId(taskMap[pid]);
            }
        }
    }

    const std::string dmesg_output = name() + ".dmesg";
    if (params()->panic_on_panic) {
        kernelPanic = addKernelFuncEventOrPanic<Linux::KernelPanic>(
            "panic", "Kernel panic in simulated kernel", dmesg_output);
    } else {
        kernelPanic = addKernelFuncEventOrPanic<Linux::DmesgDump>(
            "panic", "Kernel panic in simulated kernel", dmesg_output);
    }

    if (params()->panic_on_oops) {
        kernelOops = addKernelFuncEventOrPanic<Linux::KernelPanic>(
            "oops_exit", "Kernel oops in guest", dmesg_output);
    } else {
        kernelOops = addKernelFuncEventOrPanic<Linux::DmesgDump>(
            "oops_exit", "Kernel oops in guest", dmesg_output);
    }

    // With ARM udelay() is #defined to __udelay
    // newer kernels use __loop_udelay and __loop_const_udelay symbols
    skipUDelay = addKernelFuncEvent<SkipUDelay<SkipFunc>>(
        "__loop_udelay", "__udelay", 1000, 0);
    if (!skipUDelay)
        skipUDelay = addKernelFuncEventOrPanic<SkipUDelay<SkipFunc>>(
         "__udelay", "__udelay", 1000, 0);

    // constant arguments to udelay() have some precomputation done ahead of
    // time. Constant comes from code.
    skipConstUDelay = addKernelFuncEvent<SkipUDelay<SkipFunc>>(
        "__loop_const_udelay", "__const_udelay", 1000, 107374);
    if (!skipConstUDelay) {
        skipConstUDelay = addKernelFuncEventOrPanic<SkipUDelay<SkipFunc>>(
            "__const_udelay", "__const_udelay", 1000, 107374);
    }

    if (getArch() == Loader::Arm64) {
        debugPrintk = addKernelFuncEvent<
            DebugPrintk<SkipFuncLinux64>>("dprintk");
    } else {
        debugPrintk = addKernelFuncEvent<
            DebugPrintk<SkipFuncLinux32>>("dprintk");
    }
}

void
FsLinux::mapPid(ThreadContext *tc, uint32_t pid)
{
    // Create a new unique identifier for this pid
    std::map<uint32_t, uint32_t>::iterator itr = taskMap.find(pid);
    if (itr == taskMap.end()) {
        uint32_t map_size = taskMap.size();
        if (map_size > ContextSwitchTaskId::MaxNormalTaskId + 1) {
            warn_once("Error out of identifiers for cache occupancy stats");
            taskMap[pid] = ContextSwitchTaskId::Unknown;
        } else {
            taskMap[pid] = map_size;
        }
    }
}

void
FsLinux::dumpDmesg()
{
    Linux::dumpDmesg(system->threads[0], std::cout);
}

/**
 * Extracts the information used by the DumpStatsPCEvent by reading the
 * thread_info pointer passed to __switch_to() in 32 bit ARM Linux
 *
 *  r0 = task_struct of the previously running process
 *  r1 = thread_info of the previously running process
 *  r2 = thread_info of the next process to run
 */
void
DumpStats::getTaskDetails(ThreadContext *tc, uint32_t &pid,
    uint32_t &tgid, std::string &next_task_str, int32_t &mm) {

    Linux::ThreadInfo ti(tc);
    Addr task_descriptor = tc->readIntReg(2);
    pid = ti.curTaskPID(task_descriptor);
    tgid = ti.curTaskTGID(task_descriptor);
    next_task_str = ti.curTaskName(task_descriptor);

    // Streamline treats pid == -1 as the kernel process.
    // Also pid == 0 implies idle process (except during Linux boot)
    mm = ti.curTaskMm(task_descriptor);
}

/**
 * Extracts the information used by the DumpStatsPCEvent64 by reading the
 * task_struct pointer passed to __switch_to() in 64 bit ARM Linux
 *
 *  r0 = task_struct of the previously running process
 *  r1 = task_struct of next process to run
 */
void
DumpStats64::getTaskDetails(ThreadContext *tc, uint32_t &pid,
    uint32_t &tgid, std::string &next_task_str, int32_t &mm) {

    Linux::ThreadInfo ti(tc);
    Addr task_struct = tc->readIntReg(1);
    pid = ti.curTaskPIDFromTaskStruct(task_struct);
    tgid = ti.curTaskTGIDFromTaskStruct(task_struct);
    next_task_str = ti.curTaskNameFromTaskStruct(task_struct);

    // Streamline treats pid == -1 as the kernel process.
    // Also pid == 0 implies idle process (except during Linux boot)
    mm = ti.curTaskMmFromTaskStruct(task_struct);
}

/** This function is called whenever the the kernel function
 *  "__switch_to" is called to change running tasks.
 */
void
DumpStats::process(ThreadContext *tc)
{
    uint32_t pid = 0;
    uint32_t tgid = 0;
    std::string next_task_str;
    int32_t mm = 0;

    getTaskDetails(tc, pid, tgid, next_task_str, mm);

    bool is_kernel = (mm == 0);
    if (is_kernel && (pid != 0)) {
        pid = -1;
        tgid = -1;
        next_task_str = "kernel";
    }

    FsLinux* wl = dynamic_cast<FsLinux *>(tc->getSystemPtr()->workload);
    panic_if(!wl, "System workload is not ARM Linux!");
    std::map<uint32_t, uint32_t>& taskMap = wl->taskMap;

    // Create a new unique identifier for this pid
    wl->mapPid(tc, pid);

    // Set cpu task id, output process info, and dump stats
    tc->getCpuPtr()->taskId(taskMap[pid]);
    tc->getCpuPtr()->setPid(pid);

    OutputStream* taskFile = wl->taskFile;

    // Task file is read by cache occupancy plotting script or
    // Streamline conversion script.
    ccprintf(*(taskFile->stream()),
             "tick=%lld %d cpu_id=%d next_pid=%d next_tgid=%d next_task=%s\n",
             curTick(), taskMap[pid], tc->cpuId(), (int)pid, (int)tgid,
             next_task_str);
    taskFile->stream()->flush();

    // Dump and reset statistics
    Stats::schedStatEvent(true, true, curTick(), 0);
}

} // namespace ArmISA

ArmISA::FsLinux *
ArmFsLinuxParams::create()
{
    return new ArmISA::FsLinux(this);
}

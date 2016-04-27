/*
 * Copyright (c) 2010-2013 ARM Limited
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
 *
 * Authors: Ali Saidi
 */

#include "arch/arm/linux/atag.hh"
#include "arch/arm/linux/system.hh"
#include "arch/arm/isa_traits.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/linux/threadinfo.hh"
#include "base/loader/dtb_object.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/pc_event.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "kern/linux/events.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/physical.hh"
#include "sim/stat_control.hh"

using namespace ArmISA;
using namespace Linux;

LinuxArmSystem::LinuxArmSystem(Params *p)
    : GenericArmSystem(p), dumpStatsPCEvent(nullptr),
      enableContextSwitchStatsDump(p->enable_context_switch_stats_dump),
      taskFile(nullptr), kernelPanicEvent(nullptr), kernelOopsEvent(nullptr)
{
    if (p->panic_on_panic) {
        kernelPanicEvent = addKernelFuncEventOrPanic<PanicPCEvent>(
            "panic", "Kernel panic in simulated kernel");
    }

    if (p->panic_on_oops) {
        kernelOopsEvent = addKernelFuncEventOrPanic<PanicPCEvent>(
            "oops_exit", "Kernel oops in guest");
    }

    // With ARM udelay() is #defined to __udelay
    // newer kernels use __loop_udelay and __loop_const_udelay symbols
    uDelaySkipEvent = addKernelFuncEvent<UDelayEvent>(
        "__loop_udelay", "__udelay", 1000, 0);
    if (!uDelaySkipEvent)
        uDelaySkipEvent = addKernelFuncEventOrPanic<UDelayEvent>(
         "__udelay", "__udelay", 1000, 0);

    // constant arguments to udelay() have some precomputation done ahead of
    // time. Constant comes from code.
    constUDelaySkipEvent = addKernelFuncEvent<UDelayEvent>(
        "__loop_const_udelay", "__const_udelay", 1000, 107374);
    if (!constUDelaySkipEvent)
        constUDelaySkipEvent = addKernelFuncEventOrPanic<UDelayEvent>(
         "__const_udelay", "__const_udelay", 1000, 107374);

}

void
LinuxArmSystem::initState()
{
    // Moved from the constructor to here since it relies on the
    // address map being resolved in the interconnect

    // Call the initialisation of the super class
    GenericArmSystem::initState();

    // Load symbols at physical address, we might not want
    // to do this permanently, for but early bootup work
    // it is helpful.
    if (params()->early_kernel_symbols) {
        kernel->loadGlobalSymbols(kernelSymtab, 0, 0, loadAddrMask);
        kernel->loadGlobalSymbols(debugSymbolTable, 0, 0, loadAddrMask);
    }

    // Setup boot data structure
    Addr addr = 0;
    // Check if the kernel image has a symbol that tells us it supports
    // device trees.
    bool kernel_has_fdt_support =
        kernelSymtab->findAddress("unflatten_device_tree", addr);
    bool dtb_file_specified = params()->dtb_filename != "";

    if (kernel_has_fdt_support && dtb_file_specified) {
        // Kernel supports flattened device tree and dtb file specified.
        // Using Device Tree Blob to describe system configuration.
        inform("Loading DTB file: %s at address %#x\n", params()->dtb_filename,
                params()->atags_addr + loadAddrOffset);

        ObjectFile *dtb_file = createObjectFile(params()->dtb_filename, true);
        if (!dtb_file) {
            fatal("couldn't load DTB file: %s\n", params()->dtb_filename);
        }

        DtbObject *_dtb_file = dynamic_cast<DtbObject*>(dtb_file);

        if (_dtb_file) {
            if (!_dtb_file->addBootCmdLine(params()->boot_osflags.c_str(),
                                           params()->boot_osflags.size())) {
                warn("couldn't append bootargs to DTB file: %s\n",
                     params()->dtb_filename);
            }
        } else {
            warn("dtb_file cast failed; couldn't append bootargs "
                 "to DTB file: %s\n", params()->dtb_filename);
        }

        dtb_file->setTextBase(params()->atags_addr + loadAddrOffset);
        dtb_file->loadSections(physProxy);
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

        AddrRangeList atagRanges = physmem.getConfAddrRanges();
        if (atagRanges.size() != 1) {
            fatal("Expected a single ATAG memory entry but got %d\n",
                  atagRanges.size());
        }
        AtagMem am;
        am.memSize(atagRanges.begin()->size());
        am.memStart(atagRanges.begin()->start());

        AtagCmdline ad;
        ad.cmdline(params()->boot_osflags);

        DPRINTF(Loader, "boot command line %d bytes: %s\n",
                ad.size() <<2, params()->boot_osflags.c_str());

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

        physProxy.writeBlob(params()->atags_addr + loadAddrOffset, boot_data,
                size << 2);

        delete[] boot_data;
    }

    // Kernel boot requirements to set up r0, r1 and r2 in ARMv7
    for (int i = 0; i < threadContexts.size(); i++) {
        threadContexts[i]->setIntReg(0, 0);
        threadContexts[i]->setIntReg(1, params()->machine_type);
        threadContexts[i]->setIntReg(2, params()->atags_addr + loadAddrOffset);
    }
}

LinuxArmSystem::~LinuxArmSystem()
{
    if (uDelaySkipEvent)
        delete uDelaySkipEvent;
    if (constUDelaySkipEvent)
        delete constUDelaySkipEvent;

    if (dumpStatsPCEvent)
        delete dumpStatsPCEvent;
}

LinuxArmSystem *
LinuxArmSystemParams::create()
{
    return new LinuxArmSystem(this);
}

void
LinuxArmSystem::startup()
{
    if (enableContextSwitchStatsDump) {
        dumpStatsPCEvent = addKernelFuncEvent<DumpStatsPCEvent>("__switch_to");
        if (!dumpStatsPCEvent)
           panic("dumpStatsPCEvent not created!");

        std::string task_filename = "tasks.txt";
        taskFile = simout.create(name() + "." + task_filename);

        for (int i = 0; i < _numContexts; i++) {
            ThreadContext *tc = threadContexts[i];
            uint32_t pid = tc->getCpuPtr()->getPid();
            if (pid != BaseCPU::invldPid) {
                mapPid(tc, pid);
                tc->getCpuPtr()->taskId(taskMap[pid]);
            }
        }
    }
}

void
LinuxArmSystem::mapPid(ThreadContext *tc, uint32_t pid)
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

/** This function is called whenever the the kernel function
 *  "__switch_to" is called to change running tasks.
 *
 *  r0 = task_struct of the previously running process
 *  r1 = task_info of the previously running process
 *  r2 = task_info of the next process to run
 */
void
DumpStatsPCEvent::process(ThreadContext *tc)
{
    Linux::ThreadInfo ti(tc);
    Addr task_descriptor = tc->readIntReg(2);
    uint32_t pid = ti.curTaskPID(task_descriptor);
    uint32_t tgid = ti.curTaskTGID(task_descriptor);
    std::string next_task_str = ti.curTaskName(task_descriptor);

    // Streamline treats pid == -1 as the kernel process.
    // Also pid == 0 implies idle process (except during Linux boot)
    int32_t mm = ti.curTaskMm(task_descriptor);
    bool is_kernel = (mm == 0);
    if (is_kernel && (pid != 0)) {
        pid = -1;
        tgid = -1;
        next_task_str = "kernel";
    }

    LinuxArmSystem* sys = dynamic_cast<LinuxArmSystem *>(tc->getSystemPtr());
    if (!sys) {
        panic("System is not LinuxArmSystem while getting Linux process info!");
    }
    std::map<uint32_t, uint32_t>& taskMap = sys->taskMap;

    // Create a new unique identifier for this pid
    sys->mapPid(tc, pid);

    // Set cpu task id, output process info, and dump stats
    tc->getCpuPtr()->taskId(taskMap[pid]);
    tc->getCpuPtr()->setPid(pid);

    OutputStream* taskFile = sys->taskFile;

    // Task file is read by cache occupancy plotting script or
    // Streamline conversion script.
    ccprintf(*(taskFile->stream()),
             "tick=%lld %d cpu_id=%d next_pid=%d next_tgid=%d next_task=%s\n",
             curTick(), taskMap[pid], tc->cpuId(), (int) pid, (int) tgid,
             next_task_str);
    taskFile->stream()->flush();

    // Dump and reset statistics
    Stats::schedStatEvent(true, true, curTick(), 0);
}


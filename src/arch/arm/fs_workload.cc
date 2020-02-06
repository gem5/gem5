/*
 * Copyright (c) 2010, 2012-2013, 2015,2017-2019 ARM Limited
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

#include "arch/arm/fs_workload.hh"

#include "arch/arm/faults.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"
#include "dev/arm/gic_v2.hh"
#include "kern/system_events.hh"
#include "params/ArmFsWorkload.hh"

namespace ArmISA
{

void
SkipFunc::returnFromFuncIn(ThreadContext *tc)
{
    PCState newPC = tc->pcState();
    if (inAArch64(tc)) {
        newPC.set(tc->readIntReg(INTREG_X30));
    } else {
        newPC.set(tc->readIntReg(ReturnAddressReg) & ~ULL(1));
    }

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        tc->pcStateNoRecord(newPC);
    } else {
        tc->pcState(newPC);
    }
}

FsWorkload::FsWorkload(Params *p) : KernelWorkload(*p)
{
    if (kernelObj) {
        kernelEntry = (kernelObj->entryPoint() & loadAddrMask()) +
            loadAddrOffset();
    }

    bootLoaders.reserve(p->boot_loader.size());
    for (const auto &bl : p->boot_loader) {
        std::unique_ptr<Loader::ObjectFile> bl_obj;
        bl_obj.reset(Loader::createObjectFile(bl));

        fatal_if(!bl_obj, "Could not read bootloader: %s", bl);
        bootLoaders.emplace_back(std::move(bl_obj));
    }

    bootldr = getBootLoader(kernelObj);

    fatal_if(!bootLoaders.empty() && !bootldr,
             "Can't find a matching boot loader / kernel combination!");

    if (bootldr)
        Loader::debugSymbolTable.insert(*bootldr->symtab().globals());
}

void
FsWorkload::initState()
{
    KernelWorkload::initState();

    // Reset CP15?? What does that mean -- ali

    // FPEXC.EN = 0

    for (auto *tc: system->threads) {
        Reset().invoke(tc);
        tc->activate();
    }

    auto *arm_sys = dynamic_cast<ArmSystem *>(system);

    if (bootldr) {
        bool is_gic_v2 =
            arm_sys->getGIC()->supportsVersion(BaseGic::GicVersion::GIC_V2);
        bootldr->buildImage().write(system->physProxy);

        inform("Using bootloader at address %#x", bootldr->entryPoint());

        // Put the address of the boot loader into r7 so we know
        // where to branch to after the reset fault
        // All other values needed by the boot loader to know what to do
        fatal_if(!arm_sys->params()->flags_addr,
                 "flags_addr must be set with bootloader");

        fatal_if(!arm_sys->params()->gic_cpu_addr && is_gic_v2,
                 "gic_cpu_addr must be set with bootloader");

        for (auto *tc: arm_sys->threads) {
            if (!arm_sys->highestELIs64())
                tc->setIntReg(3, kernelEntry);
            if (is_gic_v2)
                tc->setIntReg(4, arm_sys->params()->gic_cpu_addr);
            tc->setIntReg(5, arm_sys->params()->flags_addr);
        }
        inform("Using kernel entry physical address at %#x\n", kernelEntry);
    } else {
        // Set the initial PC to be at start of the kernel code
        if (!arm_sys->highestELIs64())
            arm_sys->threads[0]->pcState(kernelObj->entryPoint());
    }
}

    Loader::ObjectFile *
FsWorkload::getBootLoader(Loader::ObjectFile *const obj)
{
    if (obj) {
        for (auto &bl : bootLoaders) {
            if (bl->getArch() == obj->getArch())
                return bl.get();
        }
    } else if (!bootLoaders.empty()) {
        return bootLoaders[0].get();
    }

    return nullptr;
}

} // namespace ArmISA

ArmISA::FsWorkload *
ArmFsWorkloadParams::create()
{
    return new ArmISA::FsWorkload(this);
}

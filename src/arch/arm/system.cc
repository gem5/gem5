/*
 * Copyright (c) 2010, 2012-2013, 2015 ARM Limited
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

#include "arch/arm/system.hh"

#include <iostream>

#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/physical.hh"
#include "sim/full_system.hh"

using namespace std;
using namespace Linux;

ArmSystem::ArmSystem(Params *p)
    : System(p),
      bootLoaders(), bootldr(nullptr),
      _haveSecurity(p->have_security),
      _haveLPAE(p->have_lpae),
      _haveVirtualization(p->have_virtualization),
      _genericTimer(nullptr),
      _highestELIs64(p->highest_el_is_64),
      _resetAddr64(p->reset_addr_64),
      _physAddrRange64(p->phys_addr_range_64),
      _haveLargeAsid64(p->have_large_asid_64),
      _m5opRange(p->m5ops_base ?
                 RangeSize(p->m5ops_base, 0x10000) :
                 AddrRange(1, 0)), // Create an empty range if disabled
      multiProc(p->multi_proc)
{
    // Check if the physical address range is valid
    if (_highestELIs64 && (
            _physAddrRange64 < 32 ||
            _physAddrRange64 > 48 ||
            (_physAddrRange64 % 4 != 0 && _physAddrRange64 != 42))) {
        fatal("Invalid physical address range (%d)\n", _physAddrRange64);
    }

    bootLoaders.reserve(p->boot_loader.size());
    for (const auto &bl : p->boot_loader) {
        std::unique_ptr<ObjectFile> obj;
        obj.reset(createObjectFile(bl));

        fatal_if(!obj, "Could not read bootloader: %s\n", bl);
        bootLoaders.emplace_back(std::move(obj));
    }

    if (kernel) {
        bootldr = getBootLoader(kernel);
    } else if (!bootLoaders.empty()) {
        // No kernel specified, default to the first boot loader
        bootldr = bootLoaders[0].get();
    }

    if (!bootLoaders.empty() && !bootldr)
        fatal("Can't find a matching boot loader / kernel combination!");

    if (bootldr) {
        bootldr->loadGlobalSymbols(debugSymbolTable);
        if ((bootldr->getArch() == ObjectFile::Arm64) && !_highestELIs64) {
            warn("Highest ARM exception-level set to AArch32 but bootloader "
                  "is for AArch64. Assuming you wanted these to match.\n");
            _highestELIs64 = true;
        } else if ((bootldr->getArch() == ObjectFile::Arm) && _highestELIs64) {
            warn("Highest ARM exception-level set to AArch64 but bootloader "
                  "is for AArch32. Assuming you wanted these to match.\n");
            _highestELIs64 = false;
        }
    }

    debugPrintkEvent = addKernelFuncEvent<DebugPrintkEvent>("dprintk");
}

void
ArmSystem::initState()
{
    // Moved from the constructor to here since it relies on the
    // address map being resolved in the interconnect

    // Call the initialisation of the super class
    System::initState();

    const Params* p = params();

    if (bootldr) {
        bootldr->loadSections(physProxy);

        uint8_t jump_to_bl_32[] =
        {
            0x07, 0xf0, 0xa0, 0xe1  // branch to r7 in aarch32
        };

        uint8_t jump_to_bl_64[] =
        {
            0xe0, 0x00, 0x1f, 0xd6  // instruction "br x7" in aarch64
        };

        // write the jump to branch table into address 0
        if (!_highestELIs64)
            physProxy.writeBlob(0x0, jump_to_bl_32, sizeof(jump_to_bl_32));
        else
            physProxy.writeBlob(0x0, jump_to_bl_64, sizeof(jump_to_bl_64));

        inform("Using bootloader at address %#x\n", bootldr->entryPoint());

        // Put the address of the boot loader into r7 so we know
        // where to branch to after the reset fault
        // All other values needed by the boot loader to know what to do
        if (!p->gic_cpu_addr || !p->flags_addr)
            fatal("gic_cpu_addr && flags_addr must be set with bootloader\n");

        for (int i = 0; i < threadContexts.size(); i++) {
            if (!_highestELIs64)
                threadContexts[i]->setIntReg(3, (kernelEntry & loadAddrMask) +
                        loadAddrOffset);
            threadContexts[i]->setIntReg(4, params()->gic_cpu_addr);
            threadContexts[i]->setIntReg(5, params()->flags_addr);
            threadContexts[i]->setIntReg(7, bootldr->entryPoint());
        }
        inform("Using kernel entry physical address at %#x\n",
               (kernelEntry & loadAddrMask) + loadAddrOffset);
    } else {
        // Set the initial PC to be at start of the kernel code
        if (!_highestELIs64)
            threadContexts[0]->pcState((kernelEntry & loadAddrMask) +
                    loadAddrOffset);
    }
}

bool
ArmSystem::haveSecurity(ThreadContext *tc)
{
    if (!FullSystem)
        return false;

    ArmSystem *a_sys = dynamic_cast<ArmSystem *>(tc->getSystemPtr());
    assert(a_sys);
    return a_sys->haveSecurity();
}


ArmSystem::~ArmSystem()
{
    if (debugPrintkEvent)
        delete debugPrintkEvent;
}

ObjectFile *
ArmSystem::getBootLoader(ObjectFile *const obj)
{
    for (auto &bl : bootLoaders) {
        if (bl->getArch() == obj->getArch())
            return bl.get();
    }

    return nullptr;
}

bool
ArmSystem::haveLPAE(ThreadContext *tc)
{
    if (!FullSystem)
        return false;

    ArmSystem *a_sys = dynamic_cast<ArmSystem *>(tc->getSystemPtr());
    assert(a_sys);
    return a_sys->haveLPAE();
}

bool
ArmSystem::haveVirtualization(ThreadContext *tc)
{
    if (!FullSystem)
        return false;

    ArmSystem *a_sys = dynamic_cast<ArmSystem *>(tc->getSystemPtr());
    assert(a_sys);
    return a_sys->haveVirtualization();
}

bool
ArmSystem::highestELIs64(ThreadContext *tc)
{
    return FullSystem ?
        dynamic_cast<ArmSystem *>(tc->getSystemPtr())->highestELIs64() :
        true;
}

ExceptionLevel
ArmSystem::highestEL(ThreadContext *tc)
{
    return FullSystem ?
        dynamic_cast<ArmSystem *>(tc->getSystemPtr())->highestEL() :
        EL1;
}

Addr
ArmSystem::resetAddr64(ThreadContext *tc)
{
    return dynamic_cast<ArmSystem *>(tc->getSystemPtr())->resetAddr64();
}

uint8_t
ArmSystem::physAddrRange(ThreadContext *tc)
{
    return dynamic_cast<ArmSystem *>(tc->getSystemPtr())->physAddrRange();
}

Addr
ArmSystem::physAddrMask(ThreadContext *tc)
{
    return dynamic_cast<ArmSystem *>(tc->getSystemPtr())->physAddrMask();
}

bool
ArmSystem::haveLargeAsid64(ThreadContext *tc)
{
    return dynamic_cast<ArmSystem *>(tc->getSystemPtr())->haveLargeAsid64();
}

ArmSystem *
ArmSystemParams::create()
{
    return new ArmSystem(this);
}

void
GenericArmSystem::initState()
{
    // Moved from the constructor to here since it relies on the
    // address map being resolved in the interconnect

    // Call the initialisation of the super class
    ArmSystem::initState();
}

GenericArmSystem *
GenericArmSystemParams::create()
{

    return new GenericArmSystem(this);
}

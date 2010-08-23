/*
 * Copyright (c) 2010 ARM Limited
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

#include "arch/arm/isa_traits.hh"
#include "arch/arm/linux/atag.hh"
#include "arch/arm/linux/system.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "mem/physical.hh"

using namespace ArmISA;

LinuxArmSystem::LinuxArmSystem(Params *p)
    : ArmSystem(p)
{
    // Load symbols at physical address, we might not want
    // to do this perminately, for but early bootup work
    // it is helpfulp.
    kernel->loadGlobalSymbols(kernelSymtab, loadAddrMask);
    kernel->loadGlobalSymbols(debugSymbolTable, loadAddrMask);

    // Setup boot data structure
    AtagCore *ac = new AtagCore;
    ac->flags(1); // read-only
    ac->pagesize(8192);
    ac->rootdev(0);

    AtagMem *am = new AtagMem;
    am->memSize(params()->physmem->size());
    am->memStart(params()->physmem->start());

    AtagCmdline *ad = new AtagCmdline;
    ad->cmdline(params()->boot_osflags);

    DPRINTF(Loader, "boot command line %d bytes: %s\n", ad->size() <<2, params()->boot_osflags.c_str());

    AtagNone *an = new AtagNone;

    uint32_t size = ac->size() + am->size() + ad->size() + an->size();
    uint32_t offset = 0;
    uint8_t *boot_data = new uint8_t[size << 2];

    offset += ac->copyOut(boot_data + offset);
    offset += am->copyOut(boot_data + offset);
    offset += ad->copyOut(boot_data + offset);
    offset += an->copyOut(boot_data + offset);

    DPRINTF(Loader, "Boot atags was %d bytes in total\n", size << 2);
    DDUMP(Loader, boot_data, size << 2);

    functionalPort.writeBlob(ParamsList, boot_data, size << 2);

#ifndef NDEBUG
    kernelPanicEvent = addKernelFuncEvent<BreakPCEvent>("panic");
    if (!kernelPanicEvent)
        panic("could not find kernel symbol \'panic\'");
#endif
}

void
LinuxArmSystem::startup()
{
    ArmSystem::startup();
    ThreadContext *tc = threadContexts[0];

    // Set the initial PC to be at start of the kernel code
    tc->setPC(tc->getSystemPtr()->kernelEntry & loadAddrMask);
    tc->setNextPC(tc->readPC() + sizeof(MachInst));

    // Setup the machine type
    tc->setIntReg(0, 0);
    tc->setIntReg(1, params()->machine_type);
    tc->setIntReg(2, ParamsList);
}

LinuxArmSystem::~LinuxArmSystem()
{
}


LinuxArmSystem *
LinuxArmSystemParams::create()
{
    return new LinuxArmSystem(this);
}

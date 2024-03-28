/*
 * Copyright (c) 2018 TU Dresden
 * Copyright (c) 2020 Barkhausen Institut
 * All rights reserved
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

#include "arch/riscv/bare_metal/fs_workload.hh"

#include "arch/riscv/faults.hh"
#include "base/loader/object_file.hh"
#include "sim/system.hh"
#include "sim/workload.hh"

namespace gem5
{

namespace RiscvISA
{

BareMetal::BareMetal(const Params &p) : Workload(p),
    _isBareMetal(p.bare_metal),
    bootloader(loader::createObjectFile(p.bootloader))
{
    fatal_if(!bootloader, "Could not load bootloader file %s.", p.bootloader);
    bootloaderSymtab = bootloader->symtab();

    if (p.auto_reset_vect) {
        _resetVect = bootloader->entryPoint();
    } else {
        _resetVect = p.reset_vect;
    }

    loader::debugSymbolTable.insert(bootloaderSymtab);
}

BareMetal::~BareMetal()
{
    delete bootloader;
}

void
BareMetal::initState()
{
    Workload::initState();

    warn_if(!bootloader->buildImage().write(system->physProxy),
            "Could not load sections to memory.");

    for (auto *tc: system->threads) {
        RiscvISA::Reset().invoke(tc);
        tc->activate();
    }
}

} // namespace RiscvISA
} // namespace gem5

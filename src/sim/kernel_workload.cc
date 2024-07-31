/*
 * Copyright 2019 Google Inc.
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

#include "sim/kernel_workload.hh"

#include "debug/Loader.hh"
#include "params/KernelWorkload.hh"
#include "sim/system.hh"

namespace gem5
{

KernelWorkload::KernelWorkload(const Params &p) : Workload(p),
    _loadAddrMask(p.load_addr_mask), _loadAddrOffset(p.load_addr_offset),
    commandLine(p.command_line)
{
    if (params().object_file == "") {
        inform("No kernel set for full system simulation. "
               "Assuming you know what you're doing.");
    } else {
        kernelObj = loader::createObjectFile(params().object_file);
        inform("kernel located at: %s", params().object_file);

        fatal_if(!kernelObj,
                "Could not load kernel file %s", params().object_file);

        image = kernelObj->buildImage();

        _start = image.minAddr();
        _end = image.maxAddr();

        // If load_addr_mask is set to 0x0, then calculate the smallest mask to
        // cover all kernel addresses so gem5 can relocate the kernel to a new
        // offset.
        if (_loadAddrMask == 0)
            _loadAddrMask = mask(findMsbSet(_end - _start) + 1);

        image.move([this](Addr a) {
            return (a & _loadAddrMask) + _loadAddrOffset;
        });

        kernelSymtab = kernelObj->symtab();
        auto initKernelSymtab = kernelSymtab.mask(_loadAddrMask)
            ->offset(_loadAddrOffset)
            ->rename([](const std::string &name) {
                return "kernel_init." + name;
            });

        loader::debugSymbolTable.insert(*initKernelSymtab);
        loader::debugSymbolTable.insert(kernelSymtab);
    }

    // Loading only needs to happen once and after memory system is
    // connected so it will happen in initState()

    std::vector<Addr> extras_addrs = p.extras_addrs;
    if (extras_addrs.empty())
        extras_addrs.resize(p.extras.size(), MaxAddr);
    fatal_if(p.extras.size() != extras_addrs.size(),
        "Additional kernel objects, not all load addresses specified\n");
    for (int ker_idx = 0; ker_idx < p.extras.size(); ker_idx++) {
        const std::string &obj_name = p.extras[ker_idx];
        const bool raw = extras_addrs[ker_idx] != MaxAddr;
        auto *obj = loader::createObjectFile(obj_name, raw);
        fatal_if(!obj, "Failed to build additional kernel object '%s'.\n",
                 obj_name);
        extras.push_back(obj);
    }
}

void
KernelWorkload::initState()
{
    auto &phys_mem = system->physProxy;
    /**
     * Load the kernel code into memory.
     */
    auto mapper = [this](Addr a) {
        return (a & _loadAddrMask) + _loadAddrOffset;
    };
    if (params().object_file != "")  {
        if (params().addr_check) {
            auto memory_size = system->memSize();
            auto kernel_size = _end - _start;
            // Validate kernel mapping before loading binary
            fatal_if((kernel_size > memory_size),
                "Kernel is mapped to invalid location (not memory). start"
                " (%#lx) - end (%#lx) %#lx:%#lx\n"
                "The kernel size (%ld bytes, %ld MB) appears to be larger "
                "than total system memory (%ld bytes, %ld MB). Try increasing"
                " system memory.", _start, _end, mapper(_start), mapper(_end),
                kernel_size, kernel_size>>20, memory_size, memory_size>>20);
            fatal_if(!system->isMemAddr(mapper(_start)) ||
                !system->isMemAddr(mapper(_end)),
                "Kernel is mapped to invalid location (not memory). start"
                " (%#lx) - end (%#lx) %#lx:%#lx\n", _start, _end,
                mapper(_start), mapper(_end));

        }
        // Load program sections into memory
        image.write(phys_mem);

        DPRINTF(Loader, "Kernel start = %#x\n", _start);
        DPRINTF(Loader, "Kernel end   = %#x\n", _end);
        DPRINTF(Loader, "Kernel entry = %#x\n", kernelObj->entryPoint());
        DPRINTF(Loader, "Kernel loaded...\n");
    }

    std::vector<Addr> extras_addrs = params().extras_addrs;
    if (extras_addrs.empty())
        extras_addrs.resize(params().extras.size(), MaxAddr);
    for (int idx = 0; idx < extras.size(); idx++) {
        const Addr load_addr = extras_addrs[idx];
        auto image = extras[idx]->buildImage();
        if (load_addr != MaxAddr)
            image = image.offset(load_addr);
        else
            image = image.move(mapper);
        image.write(phys_mem);
    }
}

void
KernelWorkload::serialize(CheckpointOut &cp) const
{
    kernelSymtab.serialize("symtab", cp);
}

void
KernelWorkload::unserialize(CheckpointIn &cp)
{
    kernelSymtab.unserialize("symtab", cp);
}

} // namespace gem5

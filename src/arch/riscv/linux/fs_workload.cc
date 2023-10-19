/*
 * Copyright (c) 2021 Huawei International
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

#include "arch/riscv/linux/fs_workload.hh"

#include "arch/riscv/faults.hh"
#include "base/loader/dtb_file.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "sim/kernel_workload.hh"
#include "sim/system.hh"

namespace gem5
{

namespace RiscvISA
{

void
FsLinux::initState()
{
    KernelWorkload::initState();

    if (params().dtb_filename != "") {
        inform("Loading DTB file: %s at address %#x\n", params().dtb_filename,
                params().dtb_addr);

        auto *dtb_file = new loader::DtbFile(params().dtb_filename);

        if (!dtb_file->addBootCmdLine(
                    commandLine.c_str(), commandLine.size())) {
            warn("couldn't append bootargs to DTB file: %s\n",
                 params().dtb_filename);
        }

        dtb_file->buildImage().offset(params().dtb_addr)
            .write(system->physProxy);
        delete dtb_file;

        for (auto *tc: system->threads) {
            tc->setReg(int_reg::A1, params().dtb_addr);
        }
    } else {
        warn("No DTB file specified\n");
    }

    for (auto *tc: system->threads) {
        RiscvISA::Reset().invoke(tc);
        tc->activate();
    }
}

void
BootloaderKernelWorkload::loadBootloaderSymbolTable()
{
    if (params().bootloader_filename != "") {
        Addr bootloader_paddr_offset = params().bootloader_addr;
        bootloader = loader::createObjectFile(params().bootloader_filename);
        bootloaderSymbolTable = bootloader->symtab();
        auto renamedBootloaderSymbolTable = \
            bootloaderSymbolTable.offset(bootloader_paddr_offset)->rename(
                [](std::string &name) {
                    name = "bootloader." + name;
                }
            );
        loader::debugSymbolTable.insert(*renamedBootloaderSymbolTable);
    }
}

void
BootloaderKernelWorkload::loadKernelSymbolTable()
{
    if (params().kernel_filename != "") {
        Addr kernel_paddr_offset = params().kernel_addr;
        kernel = loader::createObjectFile(params().kernel_filename);
        kernelSymbolTable = kernel->symtab();
        auto renamedKernelSymbolTable = \
            kernelSymbolTable.offset(kernel_paddr_offset)->rename(
                [](std::string &name) {
                    name = "kernel." + name;
                }
            );
        loader::debugSymbolTable.insert(*renamedKernelSymbolTable);
    }
}

void
BootloaderKernelWorkload::loadBootloader()
{
    if (params().bootloader_filename != "") {
        Addr bootloader_addr_offset = params().bootloader_addr;
        bootloader->buildImage().offset(bootloader_addr_offset).write(
            system->physProxy
        );
        delete bootloader;

        inform("Loaded bootloader \'%s\' at 0x%llx\n",
               params().bootloader_filename,
               bootloader_addr_offset);
    } else {
        inform("Bootloader is not specified.\n");
    }
}

void
BootloaderKernelWorkload::loadKernel()
{
    if (params().kernel_filename != "") {
        Addr kernel_paddr_offset = params().kernel_addr;
        kernel->buildImage().offset(kernel_paddr_offset).write(
            system->physProxy
        );
        delete kernel;

        inform("Loaded kernel \'%s\' at 0x%llx\n",
                params().kernel_filename,
                kernel_paddr_offset);
    } else {
        inform("Kernel is not specified.\n");
    }
}


void
BootloaderKernelWorkload::loadDtb()
{
    if (params().dtb_filename != "") {
        auto *dtb_file = new loader::DtbFile(params().dtb_filename);

        dtb_file->buildImage().offset(params().dtb_addr)
            .write(system->physProxy);
        delete dtb_file;

        inform("Loaded DTB \'%s\' at 0x%llx\n",
                params().dtb_filename,
                params().dtb_addr);

        for (auto *tc: system->threads) {
            tc->setReg(int_reg::A1, params().dtb_addr);
        }
    } else {
        inform("DTB file is not specified.\n");
    }
}

void
BootloaderKernelWorkload::initState()
{
    loadBootloader();
    loadKernel();
    loadDtb();

    for (auto *tc: system->threads) {
        RiscvISA::Reset().invoke(tc);
        tc->activate();
    }
}

void
BootloaderKernelWorkload::serialize(CheckpointOut &checkpoint) const
{
    bootloaderSymbolTable.serialize("bootloader_symbol_table", checkpoint);
    kernelSymbolTable.serialize("kernel_symbol_table", checkpoint);
}

void
BootloaderKernelWorkload::unserialize(CheckpointIn &checkpoint)
{
    bootloaderSymbolTable.unserialize("bootloader_symbol_table", checkpoint);
    kernelSymbolTable.unserialize("kernel_symbol_table", checkpoint);
}

} // namespace RiscvISA
} // namespace gem5

/*
 * Copyright (c) 2010, 2012, 2017-2018 ARM Limited
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
 * Copyright (c) 2007-2008 The Florida State University
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

#include "arch/arm/process.hh"

#include "arch/arm/isa_traits.hh"
#include "arch/arm/types.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/page_table.hh"
#include "params/Process.hh"
#include "sim/aux_vector.hh"
#include "sim/byteswap.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_return.hh"
#include "sim/system.hh"

using namespace std;
using namespace ArmISA;

ArmProcess::ArmProcess(ProcessParams *params, ::Loader::ObjectFile *objFile,
                       ::Loader::Arch _arch)
    : Process(params,
              new EmulationPageTable(params->name, params->pid, PageBytes),
              objFile),
      arch(_arch)
{
    fatal_if(params->useArchPT, "Arch page tables not implemented.");
}

ArmProcess32::ArmProcess32(ProcessParams *params,
        ::Loader::ObjectFile *objFile, ::Loader::Arch _arch)
    : ArmProcess(params, objFile, _arch)
{
    Addr brk_point = roundUp(image.maxAddr(), PageBytes);
    Addr stack_base = 0xbf000000L;
    Addr max_stack_size = 8 * 1024 * 1024;
    Addr next_thread_stack_base = stack_base - max_stack_size;
    Addr mmap_end = 0x40000000L;

    memState = make_shared<MemState>(this, brk_point, stack_base,
                                     max_stack_size, next_thread_stack_base,
                                     mmap_end);
}

ArmProcess64::ArmProcess64(
        ProcessParams *params, ::Loader::ObjectFile *objFile,
        ::Loader::Arch _arch)
    : ArmProcess(params, objFile, _arch)
{
    Addr brk_point = roundUp(image.maxAddr(), PageBytes);
    Addr stack_base = 0x7fffff0000L;
    Addr max_stack_size = 8 * 1024 * 1024;
    Addr next_thread_stack_base = stack_base - max_stack_size;
    Addr mmap_end = 0x4000000000L;

    memState = make_shared<MemState>(this, brk_point, stack_base,
                                     max_stack_size, next_thread_stack_base,
                                     mmap_end);
}

void
ArmProcess32::initState()
{
    Process::initState();
    argsInit<uint32_t>(PageBytes, INTREG_SP);
    for (int i = 0; i < contextIds.size(); i++) {
        ThreadContext * tc = system->getThreadContext(contextIds[i]);
        CPACR cpacr = tc->readMiscReg(MISCREG_CPACR);
        // Enable the floating point coprocessors.
        cpacr.cp10 = 0x3;
        cpacr.cp11 = 0x3;
        tc->setMiscReg(MISCREG_CPACR, cpacr);
        // Generically enable floating point support.
        FPEXC fpexc = tc->readMiscReg(MISCREG_FPEXC);
        fpexc.en = 1;
        tc->setMiscReg(MISCREG_FPEXC, fpexc);
    }
}

void
ArmProcess64::initState()
{
    Process::initState();
    argsInit<uint64_t>(PageBytes, INTREG_SP0);
    for (int i = 0; i < contextIds.size(); i++) {
        ThreadContext * tc = system->getThreadContext(contextIds[i]);
        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
        cpsr.mode = MODE_EL0T;
        tc->setMiscReg(MISCREG_CPSR, cpsr);
        CPACR cpacr = tc->readMiscReg(MISCREG_CPACR_EL1);
        // Enable the floating point coprocessors.
        cpacr.cp10 = 0x3;
        cpacr.cp11 = 0x3;
        // Enable SVE.
        cpacr.zen = 0x3;
        tc->setMiscReg(MISCREG_CPACR_EL1, cpacr);
        // Generically enable floating point support.
        FPEXC fpexc = tc->readMiscReg(MISCREG_FPEXC);
        fpexc.en = 1;
        tc->setMiscReg(MISCREG_FPEXC, fpexc);
    }
}

uint32_t
ArmProcess32::armHwcapImpl() const
{
    enum ArmCpuFeature {
        Arm_Swp = 1 << 0,
        Arm_Half = 1 << 1,
        Arm_Thumb = 1 << 2,
        Arm_26Bit = 1 << 3,
        Arm_FastMult = 1 << 4,
        Arm_Fpa = 1 << 5,
        Arm_Vfp = 1 << 6,
        Arm_Edsp = 1 << 7,
        Arm_Java = 1 << 8,
        Arm_Iwmmxt = 1 << 9,
        Arm_Crunch = 1 << 10,
        Arm_ThumbEE = 1 << 11,
        Arm_Neon = 1 << 12,
        Arm_Vfpv3 = 1 << 13,
        Arm_Vfpv3d16 = 1 << 14
    };

    return Arm_Swp | Arm_Half | Arm_Thumb | Arm_FastMult |
           Arm_Vfp | Arm_Edsp | Arm_ThumbEE | Arm_Neon |
           Arm_Vfpv3 | Arm_Vfpv3d16;
}

uint32_t
ArmProcess64::armHwcapImpl() const
{
    // In order to know what these flags mean, please refer to Linux
    // /Documentation/arm64/elf_hwcaps.txt text file.
    enum ArmCpuFeature {
        Arm_Fp = 1 << 0,
        Arm_Asimd = 1 << 1,
        Arm_Evtstrm = 1 << 2,
        Arm_Aes = 1 << 3,
        Arm_Pmull = 1 << 4,
        Arm_Sha1 = 1 << 5,
        Arm_Sha2 = 1 << 6,
        Arm_Crc32 = 1 << 7,
        Arm_Atomics = 1 << 8,
        Arm_Fphp = 1 << 9,
        Arm_Asimdhp = 1 << 10,
        Arm_Cpuid = 1 << 11,
        Arm_Asimdrdm = 1 << 12,
        Arm_Jscvt = 1 << 13,
        Arm_Fcma = 1 << 14,
        Arm_Lrcpc = 1 << 15,
        Arm_Dcpop = 1 << 16,
        Arm_Sha3 = 1 << 17,
        Arm_Sm3 = 1 << 18,
        Arm_Sm4 = 1 << 19,
        Arm_Asimddp = 1 << 20,
        Arm_Sha512 = 1 << 21,
        Arm_Sve = 1 << 22,
        Arm_Asimdfhm = 1 << 23,
        Arm_Dit = 1 << 24,
        Arm_Uscat = 1 << 25,
        Arm_Ilrcpc = 1 << 26,
        Arm_Flagm = 1 << 27
    };

    uint32_t hwcap = 0;

    ThreadContext *tc = system->getThreadContext(contextIds[0]);

    const AA64PFR0 pf_r0 = tc->readMiscReg(MISCREG_ID_AA64PFR0_EL1);

    hwcap |= (pf_r0.fp == 0) ? Arm_Fp : 0;
    hwcap |= (pf_r0.fp == 1) ? Arm_Fphp | Arm_Fp : 0;
    hwcap |= (pf_r0.advsimd == 0) ? Arm_Asimd : 0;
    hwcap |= (pf_r0.advsimd == 1) ? Arm_Asimdhp | Arm_Asimd : 0;
    hwcap |= (pf_r0.sve >= 1) ? Arm_Sve : 0;
    hwcap |= (pf_r0.dit >= 1) ? Arm_Dit : 0;

    const AA64ISAR0 isa_r0 = tc->readMiscReg(MISCREG_ID_AA64ISAR0_EL1);

    hwcap |= (isa_r0.aes >= 1) ? Arm_Aes : 0;
    hwcap |= (isa_r0.aes >= 2) ? Arm_Pmull : 0;
    hwcap |= (isa_r0.sha1 >= 1) ? Arm_Sha1 : 0;
    hwcap |= (isa_r0.sha2 >= 1) ? Arm_Sha2 : 0;
    hwcap |= (isa_r0.sha2 >= 2) ? Arm_Sha512 : 0;
    hwcap |= (isa_r0.crc32 >= 1) ? Arm_Crc32 : 0;
    hwcap |= (isa_r0.atomic >= 1) ? Arm_Atomics : 0;
    hwcap |= (isa_r0.rdm >= 1) ? Arm_Asimdrdm : 0;
    hwcap |= (isa_r0.sha3 >= 1) ? Arm_Sha3 : 0;
    hwcap |= (isa_r0.sm3 >= 1) ? Arm_Sm3 : 0;
    hwcap |= (isa_r0.sm4 >= 1) ? Arm_Sm4 : 0;
    hwcap |= (isa_r0.dp >= 1) ? Arm_Asimddp : 0;
    hwcap |= (isa_r0.fhm >= 1) ? Arm_Asimdfhm : 0;
    hwcap |= (isa_r0.ts >= 1) ? Arm_Flagm : 0;

    const AA64ISAR1 isa_r1 = tc->readMiscReg(MISCREG_ID_AA64ISAR1_EL1);

    hwcap |= (isa_r1.dpb >= 1) ? Arm_Dcpop : 0;
    hwcap |= (isa_r1.jscvt >= 1) ? Arm_Jscvt : 0;
    hwcap |= (isa_r1.fcma >= 1) ? Arm_Fcma : 0;
    hwcap |= (isa_r1.lrcpc >= 1) ? Arm_Lrcpc : 0;
    hwcap |= (isa_r1.lrcpc >= 2) ? Arm_Ilrcpc : 0;

    const AA64MMFR2 mm_fr2 = tc->readMiscReg(MISCREG_ID_AA64MMFR2_EL1);

    hwcap |= (mm_fr2.at >= 1) ? Arm_Uscat : 0;

    return hwcap;
}

template <class IntType>
void
ArmProcess::argsInit(int pageSize, IntRegIndex spIndex)
{
    int intSize = sizeof(IntType);

    std::vector<AuxVector<IntType>> auxv;

    string filename;
    if (argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    //We want 16 byte alignment
    uint64_t align = 16;

    //Setup the auxilliary vectors. These will already have endian conversion.
    //Auxilliary vectors are loaded only for elf formatted executables.
    auto *elfObject = dynamic_cast<::Loader::ElfObject *>(objFile);
    if (elfObject) {

        if (objFile->getOpSys() == ::Loader::Linux) {
            IntType features = armHwcap<IntType>();

            //Bits which describe the system hardware capabilities
            //XXX Figure out what these should be
            auxv.emplace_back(M5_AT_HWCAP, features);
            //Frequency at which times() increments
            auxv.emplace_back(M5_AT_CLKTCK, 0x64);
            //Whether to enable "secure mode" in the executable
            auxv.emplace_back(M5_AT_SECURE, 0);
            // Pointer to 16 bytes of random data
            auxv.emplace_back(M5_AT_RANDOM, 0);
            //The filename of the program
            auxv.emplace_back(M5_AT_EXECFN, 0);
            //The string "v71" -- ARM v7 architecture
            auxv.emplace_back(M5_AT_PLATFORM, 0);
        }

        //The system page size
        auxv.emplace_back(M5_AT_PAGESZ, ArmISA::PageBytes);
        // For statically linked executables, this is the virtual address of
        // the program header tables if they appear in the executable image
        auxv.emplace_back(M5_AT_PHDR, elfObject->programHeaderTable());
        // This is the size of a program header entry from the elf file.
        auxv.emplace_back(M5_AT_PHENT, elfObject->programHeaderSize());
        // This is the number of program headers from the original elf file.
        auxv.emplace_back(M5_AT_PHNUM, elfObject->programHeaderCount());
        // This is the base address of the ELF interpreter; it should be
        // zero for static executables or contain the base address for
        // dynamic executables.
        auxv.emplace_back(M5_AT_BASE, getBias());
        //XXX Figure out what this should be.
        auxv.emplace_back(M5_AT_FLAGS, 0);
        //The entry point to the program
        auxv.emplace_back(M5_AT_ENTRY, objFile->entryPoint());
        //Different user and group IDs
        auxv.emplace_back(M5_AT_UID, uid());
        auxv.emplace_back(M5_AT_EUID, euid());
        auxv.emplace_back(M5_AT_GID, gid());
        auxv.emplace_back(M5_AT_EGID, egid());
    }

    //Figure out how big the initial stack nedes to be

    // A sentry NULL void pointer at the top of the stack.
    int sentry_size = intSize;

    string platform = "v71";
    int platform_size = platform.size() + 1;

    // Bytes for AT_RANDOM above, we'll just keep them 0
    int aux_random_size = 16; // as per the specification

    // The aux vectors are put on the stack in two groups. The first group are
    // the vectors that are generated as the elf is loaded. The second group
    // are the ones that were computed ahead of time and include the platform
    // string.
    int aux_data_size = filename.size() + 1;

    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }

    int info_block_size =
        sentry_size + env_data_size + arg_data_size +
        aux_data_size + platform_size + aux_random_size;

    //Each auxilliary vector is two 4 byte words
    int aux_array_size = intSize * 2 * (auxv.size() + 1);

    int envp_array_size = intSize * (envp.size() + 1);
    int argv_array_size = intSize * (argv.size() + 1);

    int argc_size = intSize;

    //Figure out the size of the contents of the actual initial frame
    int frame_size =
        info_block_size +
        aux_array_size +
        envp_array_size +
        argv_array_size +
        argc_size;

    //There needs to be padding after the auxiliary vector data so that the
    //very bottom of the stack is aligned properly.
    int partial_size = frame_size;
    int aligned_partial_size = roundUp(partial_size, align);
    int aux_padding = aligned_partial_size - partial_size;

    int space_needed = frame_size + aux_padding;

    memState->setStackMin(memState->getStackBase() - space_needed);
    memState->setStackMin(roundDown(memState->getStackMin(), align));
    memState->setStackSize(memState->getStackBase() - memState->getStackMin());

    // map memory
    memState->mapRegion(roundDown(memState->getStackMin(), pageSize),
                        roundUp(memState->getStackSize(), pageSize), "stack");

    // map out initial stack contents
    IntType sentry_base = memState->getStackBase() - sentry_size;
    IntType aux_data_base = sentry_base - aux_data_size;
    IntType env_data_base = aux_data_base - env_data_size;
    IntType arg_data_base = env_data_base - arg_data_size;
    IntType platform_base = arg_data_base - platform_size;
    IntType aux_random_base = platform_base - aux_random_size;
    IntType auxv_array_base = aux_random_base - aux_array_size - aux_padding;
    IntType envp_array_base = auxv_array_base - envp_array_size;
    IntType argv_array_base = envp_array_base - argv_array_size;
    IntType argc_base = argv_array_base - argc_size;

    DPRINTF(Stack, "The addresses of items on the initial stack:\n");
    DPRINTF(Stack, "0x%x - aux data\n", aux_data_base);
    DPRINTF(Stack, "0x%x - env data\n", env_data_base);
    DPRINTF(Stack, "0x%x - arg data\n", arg_data_base);
    DPRINTF(Stack, "0x%x - random data\n", aux_random_base);
    DPRINTF(Stack, "0x%x - platform base\n", platform_base);
    DPRINTF(Stack, "0x%x - auxv array\n", auxv_array_base);
    DPRINTF(Stack, "0x%x - envp array\n", envp_array_base);
    DPRINTF(Stack, "0x%x - argv array\n", argv_array_base);
    DPRINTF(Stack, "0x%x - argc \n", argc_base);
    DPRINTF(Stack, "0x%x - stack min\n", memState->getStackMin());

    // write contents to stack

    // figure out argc
    IntType argc = argv.size();
    IntType guestArgc = htole(argc);

    //Write out the sentry void *
    IntType sentry_NULL = 0;
    initVirtMem->writeBlob(sentry_base, &sentry_NULL, sentry_size);

    //Fix up the aux vectors which point to other data
    for (int i = auxv.size() - 1; i >= 0; i--) {
        if (auxv[i].type == M5_AT_PLATFORM) {
            auxv[i].val = platform_base;
            initVirtMem->writeString(platform_base, platform.c_str());
        } else if (auxv[i].type == M5_AT_EXECFN) {
            auxv[i].val = aux_data_base;
            initVirtMem->writeString(aux_data_base, filename.c_str());
        } else if (auxv[i].type == M5_AT_RANDOM) {
            auxv[i].val = aux_random_base;
            // Just leave the value 0, we don't want randomness
        }
    }

    //Copy the aux stuff
    Addr auxv_array_end = auxv_array_base;
    for (const auto &aux: auxv) {
        initVirtMem->write(auxv_array_end, aux, GuestByteOrder);
        auxv_array_end += sizeof(aux);
    }
    //Write out the terminating zeroed auxillary vector
    const AuxVector<IntType> zero(0, 0);
    initVirtMem->write(auxv_array_end, zero);
    auxv_array_end += sizeof(zero);

    copyStringArray(envp, envp_array_base, env_data_base,
                    LittleEndianByteOrder, *initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base,
                    LittleEndianByteOrder, *initVirtMem);

    initVirtMem->writeBlob(argc_base, &guestArgc, intSize);

    ThreadContext *tc = system->getThreadContext(contextIds[0]);
    //Set the stack pointer register
    tc->setIntReg(spIndex, memState->getStackMin());
    //A pointer to a function to run when the program exits. We'll set this
    //to zero explicitly to make sure this isn't used.
    tc->setIntReg(ArgumentReg0, 0);
    //Set argument regs 1 and 2 to argv[0] and envp[0] respectively
    if (argv.size() > 0) {
        tc->setIntReg(ArgumentReg1, arg_data_base + arg_data_size -
                                    argv[argv.size() - 1].size() - 1);
    } else {
        tc->setIntReg(ArgumentReg1, 0);
    }
    if (envp.size() > 0) {
        tc->setIntReg(ArgumentReg2, env_data_base + env_data_size -
                                    envp[envp.size() - 1].size() - 1);
    } else {
        tc->setIntReg(ArgumentReg2, 0);
    }

    PCState pc;
    pc.thumb(arch == ::Loader::Thumb);
    pc.nextThumb(pc.thumb());
    pc.aarch64(arch == ::Loader::Arm64);
    pc.nextAArch64(pc.aarch64());
    pc.set(getStartPC() & ~mask(1));
    tc->pcState(pc);

    //Align the "stackMin" to a page boundary.
    memState->setStackMin(roundDown(memState->getStackMin(), pageSize));
}

const std::vector<int> ArmProcess32::SyscallABI::ArgumentRegs = {
    0, 1, 2, 3, 4, 5, 6
};

const std::vector<int> ArmProcess64::SyscallABI::ArgumentRegs = {
    0, 1, 2, 3, 4, 5, 6
};

/*
 * Copyright (c) 2010, 2012 ARM Limited
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
 *
 * Authors: Stephen Hines
 *          Ali Saidi
 */

#include "arch/arm/process.hh"

#include "arch/arm/isa_traits.hh"
#include "arch/arm/types.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/page_table.hh"
#include "sim/aux_vector.hh"
#include "sim/byteswap.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_return.hh"
#include "sim/system.hh"

using namespace std;
using namespace ArmISA;

ArmProcess::ArmProcess(ProcessParams *params, ObjectFile *objFile,
                       ObjectFile::Arch _arch)
    : Process(params, objFile), arch(_arch)
{
}

ArmProcess32::ArmProcess32(ProcessParams *params, ObjectFile *objFile,
                           ObjectFile::Arch _arch)
    : ArmProcess(params, objFile, _arch)
{
    Addr brk_point = roundUp(objFile->dataBase() + objFile->dataSize() +
                             objFile->bssSize(), PageBytes);
    Addr stack_base = 0xbf000000L;
    Addr max_stack_size = 8 * 1024 * 1024;
    Addr next_thread_stack_base = stack_base - max_stack_size;
    Addr mmap_end = 0x40000000L;

    memState = make_shared<MemState>(brk_point, stack_base, max_stack_size,
                                     next_thread_stack_base, mmap_end);
}

ArmProcess64::ArmProcess64(ProcessParams *params, ObjectFile *objFile,
                           ObjectFile::Arch _arch)
    : ArmProcess(params, objFile, _arch)
{
    Addr brk_point = roundUp(objFile->dataBase() + objFile->dataSize() +
                             objFile->bssSize(), PageBytes);
    Addr stack_base = 0x7fffff0000L;
    Addr max_stack_size = 8 * 1024 * 1024;
    Addr next_thread_stack_base = stack_base - max_stack_size;
    Addr mmap_end = 0x4000000000L;

    memState = make_shared<MemState>(brk_point, stack_base, max_stack_size,
                                     next_thread_stack_base, mmap_end);
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
        tc->setMiscReg(MISCREG_CPACR_EL1, cpacr);
        // Generically enable floating point support.
        FPEXC fpexc = tc->readMiscReg(MISCREG_FPEXC);
        fpexc.en = 1;
        tc->setMiscReg(MISCREG_FPEXC, fpexc);
    }
}

template <class IntType>
void
ArmProcess::argsInit(int pageSize, IntRegIndex spIndex)
{
    int intSize = sizeof(IntType);

    typedef AuxVector<IntType> auxv_t;
    std::vector<auxv_t> auxv;

    string filename;
    if (argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    //We want 16 byte alignment
    uint64_t align = 16;

    // Patch the ld_bias for dynamic executables.
    updateBias();

    // load object file into target memory
    objFile->loadSections(initVirtMem);

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

    //Setup the auxilliary vectors. These will already have endian conversion.
    //Auxilliary vectors are loaded only for elf formatted executables.
    ElfObject * elfObject = dynamic_cast<ElfObject *>(objFile);
    if (elfObject) {

        if (objFile->getOpSys() == ObjectFile::Linux) {
            IntType features =
                Arm_Swp |
                Arm_Half |
                Arm_Thumb |
//                Arm_26Bit |
                Arm_FastMult |
//                Arm_Fpa |
                Arm_Vfp |
                Arm_Edsp |
//                Arm_Java |
//                Arm_Iwmmxt |
//                Arm_Crunch |
                Arm_ThumbEE |
                Arm_Neon |
                Arm_Vfpv3 |
                Arm_Vfpv3d16 |
                0;

            //Bits which describe the system hardware capabilities
            //XXX Figure out what these should be
            auxv.push_back(auxv_t(M5_AT_HWCAP, features));
            //Frequency at which times() increments
            auxv.push_back(auxv_t(M5_AT_CLKTCK, 0x64));
            //Whether to enable "secure mode" in the executable
            auxv.push_back(auxv_t(M5_AT_SECURE, 0));
            // Pointer to 16 bytes of random data
            auxv.push_back(auxv_t(M5_AT_RANDOM, 0));
            //The filename of the program
            auxv.push_back(auxv_t(M5_AT_EXECFN, 0));
            //The string "v71" -- ARM v7 architecture
            auxv.push_back(auxv_t(M5_AT_PLATFORM, 0));
        }

        //The system page size
        auxv.push_back(auxv_t(M5_AT_PAGESZ, ArmISA::PageBytes));
        // For statically linked executables, this is the virtual address of the
        // program header tables if they appear in the executable image
        auxv.push_back(auxv_t(M5_AT_PHDR, elfObject->programHeaderTable()));
        // This is the size of a program header entry from the elf file.
        auxv.push_back(auxv_t(M5_AT_PHENT, elfObject->programHeaderSize()));
        // This is the number of program headers from the original elf file.
        auxv.push_back(auxv_t(M5_AT_PHNUM, elfObject->programHeaderCount()));
        // This is the base address of the ELF interpreter; it should be
        // zero for static executables or contain the base address for
        // dynamic executables.
        auxv.push_back(auxv_t(M5_AT_BASE, getBias()));
        //XXX Figure out what this should be.
        auxv.push_back(auxv_t(M5_AT_FLAGS, 0));
        //The entry point to the program
        auxv.push_back(auxv_t(M5_AT_ENTRY, objFile->entryPoint()));
        //Different user and group IDs
        auxv.push_back(auxv_t(M5_AT_UID, uid()));
        auxv.push_back(auxv_t(M5_AT_EUID, euid()));
        auxv.push_back(auxv_t(M5_AT_GID, gid()));
        auxv.push_back(auxv_t(M5_AT_EGID, egid()));
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
    allocateMem(roundDown(memState->getStackMin(), pageSize),
                          roundUp(memState->getStackSize(), pageSize));

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
    IntType guestArgc = ArmISA::htog(argc);

    //Write out the sentry void *
    IntType sentry_NULL = 0;
    initVirtMem.writeBlob(sentry_base,
            (uint8_t*)&sentry_NULL, sentry_size);

    //Fix up the aux vectors which point to other data
    for (int i = auxv.size() - 1; i >= 0; i--) {
        if (auxv[i].a_type == M5_AT_PLATFORM) {
            auxv[i].a_val = platform_base;
            initVirtMem.writeString(platform_base, platform.c_str());
        } else if (auxv[i].a_type == M5_AT_EXECFN) {
            auxv[i].a_val = aux_data_base;
            initVirtMem.writeString(aux_data_base, filename.c_str());
        } else if (auxv[i].a_type == M5_AT_RANDOM) {
            auxv[i].a_val = aux_random_base;
            // Just leave the value 0, we don't want randomness
        }
    }

    //Copy the aux stuff
    for (int x = 0; x < auxv.size(); x++) {
        initVirtMem.writeBlob(auxv_array_base + x * 2 * intSize,
                (uint8_t*)&(auxv[x].a_type), intSize);
        initVirtMem.writeBlob(auxv_array_base + (x * 2 + 1) * intSize,
                (uint8_t*)&(auxv[x].a_val), intSize);
    }
    //Write out the terminating zeroed auxilliary vector
    const uint64_t zero = 0;
    initVirtMem.writeBlob(auxv_array_base + 2 * intSize * auxv.size(),
            (uint8_t*)&zero, 2 * intSize);

    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);

    initVirtMem.writeBlob(argc_base, (uint8_t*)&guestArgc, intSize);

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
    pc.thumb(arch == ObjectFile::Thumb);
    pc.nextThumb(pc.thumb());
    pc.aarch64(arch == ObjectFile::Arm64);
    pc.nextAArch64(pc.aarch64());
    pc.set(getStartPC() & ~mask(1));
    tc->pcState(pc);

    //Align the "stackMin" to a page boundary.
    memState->setStackMin(roundDown(memState->getStackMin(), pageSize));
}

ArmISA::IntReg
ArmProcess32::getSyscallArg(ThreadContext *tc, int &i)
{
    assert(i < 6);
    return tc->readIntReg(ArgumentReg0 + i++);
}

ArmISA::IntReg
ArmProcess64::getSyscallArg(ThreadContext *tc, int &i)
{
    assert(i < 8);
    return tc->readIntReg(ArgumentReg0 + i++);
}

ArmISA::IntReg
ArmProcess32::getSyscallArg(ThreadContext *tc, int &i, int width)
{
    assert(width == 32 || width == 64);
    if (width == 32)
        return getSyscallArg(tc, i);

    // 64 bit arguments are passed starting in an even register
    if (i % 2 != 0)
       i++;

    // Registers r0-r6 can be used
    assert(i < 5);
    uint64_t val;
    val = tc->readIntReg(ArgumentReg0 + i++);
    val |= ((uint64_t)tc->readIntReg(ArgumentReg0 + i++) << 32);
    return val;
}

ArmISA::IntReg
ArmProcess64::getSyscallArg(ThreadContext *tc, int &i, int width)
{
    return getSyscallArg(tc, i);
}


void
ArmProcess32::setSyscallArg(ThreadContext *tc, int i, ArmISA::IntReg val)
{
    assert(i < 6);
    tc->setIntReg(ArgumentReg0 + i, val);
}

void
ArmProcess64::setSyscallArg(ThreadContext *tc, int i, ArmISA::IntReg val)
{
    assert(i < 8);
    tc->setIntReg(ArgumentReg0 + i, val);
}

void
ArmProcess32::setSyscallReturn(ThreadContext *tc, SyscallReturn sysret)
{

    if (objFile->getOpSys() == ObjectFile::FreeBSD) {
        // Decode return value
        if (sysret.encodedValue() >= 0)
            // FreeBSD checks the carry bit to determine if syscall is succeeded
            tc->setCCReg(CCREG_C, 0);
        else {
            sysret = -sysret.encodedValue();
        }
    }

    tc->setIntReg(ReturnValueReg, sysret.encodedValue());
}

void
ArmProcess64::setSyscallReturn(ThreadContext *tc, SyscallReturn sysret)
{

    if (objFile->getOpSys() == ObjectFile::FreeBSD) {
        // Decode return value
        if (sysret.encodedValue() >= 0)
            // FreeBSD checks the carry bit to determine if syscall is succeeded
            tc->setCCReg(CCREG_C, 0);
        else {
            sysret = -sysret.encodedValue();
        }
    }

    tc->setIntReg(ReturnValueReg, sysret.encodedValue());
}

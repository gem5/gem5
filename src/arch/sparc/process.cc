/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/process.hh"
#include "arch/sparc/linux/process.hh"
#include "arch/sparc/solaris/process.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "cpu/exec_context.hh"
#include "mem/page_table.hh"
#include "mem/translating_port.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;
using namespace SparcISA;

SparcLiveProcess *
SparcLiveProcess::create(const std::string &nm, System *system, int stdin_fd,
        int stdout_fd, int stderr_fd, std::string executable,
        std::vector<std::string> &argv, std::vector<std::string> &envp)
{
    SparcLiveProcess *process = NULL;

    ObjectFile *objFile = createObjectFile(executable);
    if (objFile == NULL) {
        fatal("Can't load object file %s", executable);
    }


    if (objFile->getArch() != ObjectFile::SPARC)
        fatal("Object file with arch %x does not match architecture %x.",
                objFile->getArch(), ObjectFile::SPARC);
    switch (objFile->getOpSys()) {
      case ObjectFile::Linux:
        process = new SparcLinuxProcess(nm, objFile, system,
                                        stdin_fd, stdout_fd, stderr_fd,
                                        argv, envp);
        break;


      case ObjectFile::Solaris:
        process = new SparcSolarisProcess(nm, objFile, system,
                                        stdin_fd, stdout_fd, stderr_fd,
                                        argv, envp);
        break;
      default:
        fatal("Unknown/unsupported operating system.");
    }

    if (process == NULL)
        fatal("Unknown error creating process object.");
    return process;
}

SparcLiveProcess::SparcLiveProcess(const std::string &nm, ObjectFile *objFile,
        System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
        std::vector<std::string> &argv, std::vector<std::string> &envp)
    : LiveProcess(nm, objFile, _system, stdin_fd, stdout_fd, stderr_fd,
        argv, envp)
{

    // XXX all the below need to be updated for SPARC - Ali
    brk_point = objFile->dataBase() + objFile->dataSize() + objFile->bssSize();
    brk_point = roundUp(brk_point, VMPageSize);

    // Set up stack. On SPARC Linux, stack goes from the top of memory
    // downward, less the hole for the kernel address space.
    stack_base = ((Addr)0x80000000000ULL);

    // Set up region for mmaps.  Tru64 seems to start just above 0 and
    // grow up from there.
    mmap_start = mmap_end = 0x800000;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);
}

void
SparcLiveProcess::startup()
{
    argsInit(MachineBytes, VMPageSize);

    //From the SPARC ABI

    //The process runs in user mode
    execContexts[0]->setMiscRegWithEffect(MISCREG_PSTATE, 0x02);

    //Setup default FP state
    execContexts[0]->setMiscReg(MISCREG_FSR, 0);

    execContexts[0]->setMiscReg(MISCREG_TICK, 0);
    //
    /*
     * Register window management registers
     */

    //No windows contain info from other programs
    execContexts[0]->setMiscRegWithEffect(MISCREG_OTHERWIN, 0);
    //There are no windows to pop
    execContexts[0]->setMiscRegWithEffect(MISCREG_CANRESTORE, 0);
    //All windows are available to save into
    execContexts[0]->setMiscRegWithEffect(MISCREG_CANSAVE, NWindows - 2);
    //All windows are "clean"
    execContexts[0]->setMiscRegWithEffect(MISCREG_CLEANWIN, NWindows);
    //Start with register window 0
    execContexts[0]->setMiscRegWithEffect(MISCREG_CWP, 0);
}

m5_auxv_t buildAuxVect(int64_t type, int64_t val)
{
    m5_auxv_t result;
    result.a_type = TheISA::htog(type);
    result.a_val = TheISA::htog(val);
    return result;
}

void
SparcLiveProcess::argsInit(int intSize, int pageSize)
{
    Process::startup();

    Addr alignmentMask = ~(intSize - 1);

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    //These are the auxilliary vector types
    enum auxTypes
    {
        SPARC_AT_HWCAP = 16,
        SPARC_AT_PAGESZ = 6,
        SPARC_AT_CLKTCK = 17,
        SPARC_AT_PHDR = 3,
        SPARC_AT_PHENT = 4,
        SPARC_AT_PHNUM = 5,
        SPARC_AT_BASE = 7,
        SPARC_AT_FLAGS = 8,
        SPARC_AT_ENTRY = 9,
        SPARC_AT_UID = 11,
        SPARC_AT_EUID = 12,
        SPARC_AT_GID = 13,
        SPARC_AT_EGID = 14
    };

    enum hardwareCaps
    {
        M5_HWCAP_SPARC_FLUSH = 1,
        M5_HWCAP_SPARC_STBAR = 2,
        M5_HWCAP_SPARC_SWAP = 4,
        M5_HWCAP_SPARC_MULDIV = 8,
        M5_HWCAP_SPARC_V9 = 16,
        //This one should technically only be set
        //if there is a cheetah or cheetah_plus tlb,
        //but we'll use it all the time
        M5_HWCAP_SPARC_ULTRA3 = 32
    };

    const int64_t hwcap =
        M5_HWCAP_SPARC_FLUSH |
        M5_HWCAP_SPARC_STBAR |
        M5_HWCAP_SPARC_SWAP |
        M5_HWCAP_SPARC_MULDIV |
        M5_HWCAP_SPARC_V9 |
        M5_HWCAP_SPARC_ULTRA3;

    //Setup the auxilliary vectors. These will already have
    //endian conversion.
    auxv.push_back(buildAuxVect(SPARC_AT_EGID, 100));
    auxv.push_back(buildAuxVect(SPARC_AT_GID, 100));
    auxv.push_back(buildAuxVect(SPARC_AT_EUID, 100));
    auxv.push_back(buildAuxVect(SPARC_AT_UID, 100));
    //This would work, but the entry point is a protected member
    //auxv.push_back(buildAuxVect(SPARC_AT_ENTRY, objFile->entry));
    auxv.push_back(buildAuxVect(SPARC_AT_FLAGS, 0));
    //This is the address of the elf "interpreter", which I don't
    //think we currently set up. It should be set to 0 (I think)
    //auxv.push_back(buildAuxVect(SPARC_AT_BASE, 0));
    //This is the number of headers which were in the original elf
    //file. This information isn't avaibale by this point.
    //auxv.push_back(buildAuxVect(SPARC_AT_PHNUM, 3));
    //This is the size of a program header entry. This isn't easy
    //to compute here.
    //auxv.push_back(buildAuxVect(SPARC_AT_PHENT, blah));
    //This is should be set to load_addr (whatever that is) +
    //e_phoff. I think it's a pointer to the program headers.
    //auxv.push_back(buildAuxVect(SPARC_AT_PHDR, blah));
    //This should be easy to get right, but I won't set it for now
    //auxv.push_back(buildAuxVect(SPARC_AT_CLKTCK, blah));
    auxv.push_back(buildAuxVect(SPARC_AT_PAGESZ, SparcISA::VMPageSize));
    auxv.push_back(buildAuxVect(SPARC_AT_HWCAP, hwcap));

    //Figure out how big the initial stack needs to be

    //Each auxilliary vector is two 8 byte words
    int aux_data_size = 2 * intSize * auxv.size();
    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }

    int aux_array_size = intSize * 2 * (auxv.size() + 1);

    int argv_array_size = intSize * (argv.size() + 1);
    int envp_array_size = intSize * (envp.size() + 1);

    int argc_size = intSize;
    int window_save_size = intSize * 16;

    int info_block_size =
        (aux_data_size +
        env_data_size +
        arg_data_size +
        ~alignmentMask) & alignmentMask;

    int info_block_padding =
        info_block_size -
        aux_data_size -
        env_data_size -
        arg_data_size;

    int space_needed =
        info_block_size +
        aux_array_size +
        envp_array_size +
        argv_array_size +
        argc_size +
        window_save_size;

    stack_min = stack_base - space_needed;
    stack_min &= alignmentMask;
    stack_size = stack_base - stack_min;

    // map memory
    pTable->allocate(roundDown(stack_min, pageSize),
                     roundUp(stack_size, pageSize));

    // map out initial stack contents
    Addr aux_data_base = stack_base - aux_data_size - info_block_padding;
    Addr env_data_base = aux_data_base - env_data_size;
    Addr arg_data_base = env_data_base - arg_data_size;
    Addr auxv_array_base = arg_data_base - aux_array_size;
    Addr envp_array_base = auxv_array_base - envp_array_size;
    Addr argv_array_base = envp_array_base - argv_array_size;
    Addr argc_base = argv_array_base - argc_size;
    Addr window_save_base = argc_base - window_save_size;

    DPRINTF(Sparc, "The addresses of items on the initial stack:\n");
    DPRINTF(Sparc, "0x%x - aux data\n", aux_data_base);
    DPRINTF(Sparc, "0x%x - env data\n", env_data_base);
    DPRINTF(Sparc, "0x%x - arg data\n", arg_data_base);
    DPRINTF(Sparc, "0x%x - auxv array\n", auxv_array_base);
    DPRINTF(Sparc, "0x%x - envp array\n", envp_array_base);
    DPRINTF(Sparc, "0x%x - argv array\n", argv_array_base);
    DPRINTF(Sparc, "0x%x - argc \n", argc_base);
    DPRINTF(Sparc, "0x%x - window save\n", window_save_base);
    DPRINTF(Sparc, "0x%x - stack min\n", stack_min);

    // write contents to stack
    uint64_t argc = argv.size();
    uint64_t guestArgc = TheISA::htog(argc);

    //Copy the aux stuff
    for(int x = 0; x < auxv.size(); x++)
    {
        initVirtMem->writeBlob(auxv_array_base + x * 2 * intSize,
                (uint8_t*)&(auxv[x].a_type), intSize);
        initVirtMem->writeBlob(auxv_array_base + (x * 2 + 1) * intSize,
                (uint8_t*)&(auxv[x].a_val), intSize);
    }
    //Write out the terminating zeroed auxilliary vector
    const uint64_t zero = 0;
    initVirtMem->writeBlob(auxv_array_base + 2 * intSize * auxv.size(),
            (uint8_t*)&zero, 2 * intSize);

    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);

    initVirtMem->writeBlob(argc_base, (uint8_t*)&guestArgc, intSize);

    execContexts[0]->setIntReg(ArgumentReg0, argc);
    execContexts[0]->setIntReg(ArgumentReg1, argv_array_base);
    execContexts[0]->setIntReg(StackPointerReg, stack_min - StackBias);

    Addr prog_entry = objFile->entryPoint();
    execContexts[0]->setPC(prog_entry);
    execContexts[0]->setNextPC(prog_entry + sizeof(MachInst));
    execContexts[0]->setNextNPC(prog_entry + (2 * sizeof(MachInst)));

//    num_processes++;
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(SparcLiveProcess)

    VectorParam<string> cmd;
    Param<string> executable;
    Param<string> input;
    Param<string> output;
    VectorParam<string> env;
    SimObjectParam<System *> system;

END_DECLARE_SIM_OBJECT_PARAMS(SparcLiveProcess)


BEGIN_INIT_SIM_OBJECT_PARAMS(SparcLiveProcess)

    INIT_PARAM(cmd, "command line (executable plus arguments)"),
    INIT_PARAM(executable, "executable (overrides cmd[0] if set)"),
    INIT_PARAM(input, "filename for stdin (dflt: use sim stdin)"),
    INIT_PARAM(output, "filename for stdout/stderr (dflt: use sim stdout)"),
    INIT_PARAM(env, "environment settings"),
    INIT_PARAM(system, "system")

END_INIT_SIM_OBJECT_PARAMS(SparcLiveProcess)


CREATE_SIM_OBJECT(SparcLiveProcess)
{
    string in = input;
    string out = output;

    // initialize file descriptors to default: same as simulator
    int stdin_fd, stdout_fd, stderr_fd;

    if (in == "stdin" || in == "cin")
        stdin_fd = STDIN_FILENO;
    else
        stdin_fd = Process::openInputFile(input);

    if (out == "stdout" || out == "cout")
        stdout_fd = STDOUT_FILENO;
    else if (out == "stderr" || out == "cerr")
        stdout_fd = STDERR_FILENO;
    else
        stdout_fd = Process::openOutputFile(out);

    stderr_fd = (stdout_fd != STDOUT_FILENO) ? stdout_fd : STDERR_FILENO;

    return SparcLiveProcess::create(getInstanceName(), system,
                               stdin_fd, stdout_fd, stderr_fd,
                               (string)executable == "" ? cmd[0] : executable,
                               cmd, env);
}


REGISTER_SIM_OBJECT("SparcLiveProcess", SparcLiveProcess)



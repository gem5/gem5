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
 *
 * Authors: Gabe Black
 *          Ali Saidi
 */

#include "arch/mips/isa_traits.hh"
#include "arch/mips/process.hh"
#include "arch/mips/linux/process.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;
using namespace MipsISA;


MipsLiveProcess *
MipsLiveProcess::create(const std::string &nm, System *system, int stdin_fd,
        int stdout_fd, int stderr_fd, std::string executable,
        std::vector<std::string> &argv, std::vector<std::string> &envp)
{
    MipsLiveProcess *process = NULL;

    ObjectFile *objFile = createObjectFile(executable);
    if (objFile == NULL) {
        fatal("Can't load object file %s", executable);
    }


    if (objFile->getArch() != ObjectFile::Mips)
        fatal("Object file does not match architecture.");
    switch (objFile->getOpSys()) {
      case ObjectFile::Linux:
        process = new MipsLinuxProcess(nm, objFile, system,
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

MipsLiveProcess::MipsLiveProcess(const std::string &nm, ObjectFile *objFile,
        System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
        std::vector<std::string> &argv, std::vector<std::string> &envp)
    : LiveProcess(nm, objFile, _system, stdin_fd, stdout_fd, stderr_fd,
        argv, envp)
{

    // XXX all the below need to be updated for SPARC - Ali
    brk_point = objFile->dataBase() + objFile->dataSize() + objFile->bssSize();
    brk_point = roundUp(brk_point, VMPageSize);

    // Set up stack.  On Alpha, stack goes below text section.  This
    // code should get moved to some architecture-specific spot.
    stack_base = objFile->textBase() - (409600+4096);

    // Set up region for mmaps.  Tru64 seems to start just above 0 and
    // grow up from there.
    mmap_start = mmap_end = 0x10000;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);

}

void
MipsLiveProcess::startup()
{
    argsInit(MachineBytes, VMPageSize);
}




BEGIN_DECLARE_SIM_OBJECT_PARAMS(MipsLiveProcess)

    VectorParam<string> cmd;
    Param<string> executable;
    Param<string> input;
    Param<string> output;
    VectorParam<string> env;
    SimObjectParam<System *> system;

END_DECLARE_SIM_OBJECT_PARAMS(MipsLiveProcess)


BEGIN_INIT_SIM_OBJECT_PARAMS(MipsLiveProcess)

    INIT_PARAM(cmd, "command line (executable plus arguments)"),
    INIT_PARAM(executable, "executable (overrides cmd[0] if set)"),
    INIT_PARAM(input, "filename for stdin (dflt: use sim stdin)"),
    INIT_PARAM(output, "filename for stdout/stderr (dflt: use sim stdout)"),
    INIT_PARAM(env, "environment settings"),
    INIT_PARAM(system, "system")

END_INIT_SIM_OBJECT_PARAMS(MipsLiveProcess)


CREATE_SIM_OBJECT(MipsLiveProcess)
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

    return MipsLiveProcess::create(getInstanceName(), system,
                               stdin_fd, stdout_fd, stderr_fd,
                               (string)executable == "" ? cmd[0] : executable,
                               cmd, env);
}


REGISTER_SIM_OBJECT("MipsLiveProcess", MipsLiveProcess)



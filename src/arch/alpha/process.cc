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

#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/process.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"
#include "sim/system.hh"


using namespace AlphaISA;
using namespace std;

AlphaLiveProcess::AlphaLiveProcess(const std::string &nm, ObjectFile *objFile,
        System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
        std::vector<std::string> &argv, std::vector<std::string> &envp)
    : LiveProcess(nm, objFile, _system, stdin_fd, stdout_fd, stderr_fd,
        argv, envp)
{
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
AlphaLiveProcess::startup()
{
    argsInit(MachineBytes, VMPageSize);

    threadContexts[0]->setIntReg(GlobalPointerReg, objFile->globalPointer());
}



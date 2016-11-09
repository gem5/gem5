/*
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
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
 *          Timothy M. Jones
 */

#ifndef __POWER_PROCESS_HH__
#define __POWER_PROCESS_HH__

#include <string>
#include <vector>

#include "mem/page_table.hh"
#include "sim/process.hh"

class ObjectFile;

class PowerProcess : public Process
{
  protected:
    PowerProcess(ProcessParams * params, ObjectFile *objFile);

    void initState();

  public:
    void argsInit(int intSize, int pageSize);
    PowerISA::IntReg getSyscallArg(ThreadContext *tc, int &i);
    /// Explicitly import the otherwise hidden getSyscallArg
    using Process::getSyscallArg;
    void setSyscallArg(ThreadContext *tc, int i, PowerISA::IntReg val);
    void setSyscallReturn(ThreadContext *tc, SyscallReturn return_value);
};

/* No architectural page table defined for this ISA */
typedef NoArchPageTable ArchPageTable;

#endif // __POWER_PROCESS_HH__


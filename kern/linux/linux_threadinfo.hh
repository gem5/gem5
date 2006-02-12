/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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

#ifndef __KERN_LINUX_LINUX_TREADNIFO_HH__
#define __KERN_LINUX_LINUX_TREADNIFO_HH__

#include "kern/linux/thread_info.hh"
#include "kern/linux/sched.hh"
#include "sim/vptr.hh"

namespace Linux {

class ThreadInfo
{
  private:
    ExecContext *xc;

  public:
    ThreadInfo(ExecContext *exec) : xc(exec) {}
    ~ThreadInfo() {}

    inline VPtr<thread_info>
    curThreadInfo()
    {
        Addr current;

        /* Each kernel stack is only 2 pages, the start of which is the
         * thread_info struct. So we can get the address by masking off
         * the lower 14 bits.
         */
        current = xc->regs.intRegFile[StackPointerReg] & ~0x3fff;
        return VPtr<thread_info>(xc, current);
    }

    inline VPtr<task_struct>
    curTaskInfo()
    {
        Addr task = curThreadInfo()->task;
        return VPtr<task_struct>(xc, task);
    }

    std::string
    curTaskName()
    {
        return curTaskInfo()->name;
    }

    int32_t
    curTaskPID()
    {
        return curTaskInfo()->pid;
    }

    uint64_t
    curTaskStart()
    {
        return curTaskInfo()->start;
    }
};

/* namespace Linux */ }

#endif // __KERN_LINUX_LINUX_THREADINFO_HH__

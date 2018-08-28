/*
 * Copyright (c) 2012-2013, 2015 ARM Limited
 * Copyright (c) 2015-2016 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 *          Kevin Lim
 *          Brandon Potter
 */

#ifndef __SIM_SYSCALL_DESC_HH__
#define __SIM_SYSCALL_DESC_HH__

#include <string>

#include "base/types.hh"

class Process;
class SyscallDesc;
class SyscallReturn;
class ThreadContext;

SyscallReturn unimplementedFunc(SyscallDesc *desc, int num,
                                ThreadContext *tc);

/**
 * This class provides the wrapper interface for the system call
 * implementations which are defined in the sim/syscall_emul files and
 * bound to the ISAs in the architecture specific code
 * (i.e. arch/X86/linux/process.cc).
 */
class SyscallDesc {
  public:
    /** Typedef the function pointer here to clean up code below */
    typedef SyscallReturn (*SyscallExecutor)(SyscallDesc*, int num,
                                             ThreadContext*);

    SyscallDesc(const char *name,
                SyscallExecutor sys_exec=unimplementedFunc, int flags=0)
        : _name(name), executor(sys_exec), _flags(flags), _warned(false)
    {
    }

    /** Provide a mechanism to specify behavior for abnormal system calls */
    enum Flags {
        /**
         * Do not set return registers according to executor return value.
         * Used for system calls with non-standard return conventions that
         * explicitly set the thread context regs (e.g., sigreturn, clone)
         */
        SuppressReturnValue = 1,
        /** Warn only once for unimplemented system calls */
        WarnOnce = 2
        /* X2 = 4, // Remove these comments when the next field is added; */
        /* X3 = 8, // point is to make it obvious that this defines vector */
    };

    /**
     * Interface for invoking the system call funcion pointer. Note that
     * this acts as a gateway for all system calls and serves a good point
     * to add filters for behaviors or apply checks for all system calls.
     * @param callnum Number associated with call (by operating system)
     * @param proc Handle for the owning Process to pass information
     * @param tc Handle for owning ThreadContext to pass information
     */
    void doSyscall(int callnum, ThreadContext *tc, Fault *fault);

    /**
     * Return false if WarnOnce is set and a warning has already been issued.
     * Otherwise, return true. Updates state as a side effect to help
     * keep track of issued warnings.
     */
    bool needWarning();

    bool warnOnce() const { return (_flags & WarnOnce); }

    std::string name() { return _name; }

    int getFlags() const { return _flags; }

    void setFlags(int flags) { _flags = flags; }

  private:
    /** System call name (e.g., open, mmap, clone, socket, etc.) */
    std::string _name;

    /** Mechanism for ISAs to connect to the emul function definitions */
    SyscallExecutor executor;

    /**
     * Holds values set with the preceding enum; note that this has been
     * used primarily for features that are mutually exclusive, but there's
     * no reason that this needs to be true going forward.
     */
    int _flags;

    /** Set if WarnOnce is specified in flags AFTER first call */
    bool _warned;
};

#endif // __SIM_SYSCALL_DESC_HH__

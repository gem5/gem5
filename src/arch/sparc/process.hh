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

#ifndef __SPARC_PROCESS_HH__
#define __SPARC_PROCESS_HH__

#include <string>
#include <vector>
#include "sim/process.hh"

class ObjectFile;
class System;

class SparcLiveProcess : public LiveProcess
{
  protected:

    //The locations of the fill and spill handlers
    Addr fillStart, spillStart;

    SparcLiveProcess(const std::string &nm, ObjectFile *objFile,
                System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
                std::vector<std::string> &argv,
                std::vector<std::string> &envp,
                const std::string &cwd,
                uint64_t _uid, uint64_t _euid,
                uint64_t _gid, uint64_t _egid,
                uint64_t _pid, uint64_t _ppid);

  public:

    //Handles traps which request services from the operating system
    virtual void handleTrap(int trapNum, ThreadContext *tc);

    Addr readFillStart()
    { return fillStart; }

    Addr readSpillStart()
    { return spillStart; }

};

struct M5_32_auxv_t
{
    int32_t a_type;
    union {
        int32_t a_val;
        int32_t a_ptr;
        int32_t a_fcn;
    };

    M5_32_auxv_t()
    {}

    M5_32_auxv_t(int32_t type, int32_t val);
};

class Sparc32LiveProcess : public SparcLiveProcess
{
  protected:

    std::vector<M5_32_auxv_t> auxv;

    Sparc32LiveProcess(const std::string &nm, ObjectFile *objFile,
                System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
                std::vector<std::string> &argv,
                std::vector<std::string> &envp,
                const std::string &cwd,
                uint64_t _uid, uint64_t _euid,
                uint64_t _gid, uint64_t _egid,
                uint64_t _pid, uint64_t _ppid) :
            SparcLiveProcess(nm, objFile, _system,
                         stdin_fd, stdout_fd, stderr_fd,
                         argv, envp, cwd,
                         _uid, _euid, _gid, _egid, _pid, _ppid)
    {
        // Set up stack. On SPARC Linux, stack goes from the top of memory
        // downward, less the hole for the kernel address space.
        stack_base = (Addr)0xf0000000ULL;

        // Set up region for mmaps.
        mmap_start = mmap_end = 0x70000000;
    }

    void startup();

  public:

    void argsInit(int intSize, int pageSize);

};

struct M5_64_auxv_t
{
    int64_t a_type;
    union {
        int64_t a_val;
        int64_t a_ptr;
        int64_t a_fcn;
    };

    M5_64_auxv_t()
    {}

    M5_64_auxv_t(int64_t type, int64_t val);
};

class Sparc64LiveProcess : public SparcLiveProcess
{
  protected:

    static const Addr StackBias = 2047;

    std::vector<M5_64_auxv_t> auxv;

    Sparc64LiveProcess(const std::string &nm, ObjectFile *objFile,
                System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
                std::vector<std::string> &argv,
                std::vector<std::string> &envp,
                const std::string &cwd,
                uint64_t _uid, uint64_t _euid,
                uint64_t _gid, uint64_t _egid,
                uint64_t _pid, uint64_t _ppid) :
            SparcLiveProcess(nm, objFile, _system,
                         stdin_fd, stdout_fd, stderr_fd,
                         argv, envp, cwd,
                         _uid, _euid, _gid, _egid, _pid, _ppid)
    {
        // Set up stack. On SPARC Linux, stack goes from the top of memory
        // downward, less the hole for the kernel address space.
        stack_base = (Addr)0x80000000000ULL;

        // Set up region for mmaps.  Tru64 seems to start just above 0 and
        // grow up from there.
        mmap_start = mmap_end = 0xfffff80000000000ULL;
    }

    void startup();

  public:

    void argsInit(int intSize, int pageSize);

};

#endif // __SPARC_PROCESS_HH__

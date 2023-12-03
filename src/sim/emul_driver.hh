/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
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
 *
 * Author: Steve Reinhardt
 */

#ifndef __SIM_EMUL_DRIVER_HH
#define __SIM_EMUL_DRIVER_HH

#include <string>

#include "params/EmulatedDriver.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class Process;
class ThreadContext;

/**
 * EmulatedDriver is an abstract base class for fake SE-mode device drivers.
 *
 * Specific drivers that allow applications to communicate with simulated
 * hardware inside gem5 can be created by deriving from this class and
 * overriding the abstract virtual methods.
 *
 * Currently only open(), ioctl(), and mmap() calls are supported, but other
 * calls (e.g., read(), write()) could be added as needed.
 */
class EmulatedDriver : public SimObject
{
  protected:
    /**
     * filename for opening this driver (under /dev)
     */
    const std::string &filename;

  public:
    EmulatedDriver(const EmulatedDriverParams &p)
        : SimObject(p), filename(p.filename)
    {}

    /**
     * Check for a match with this driver's filename.
     */
    bool
    match(const std::string &s) const
    {
        return (s == filename);
    }

    /**
     * Abstract method, invoked when the user program calls open() on
     * the device driver.  The parameters are the same as those passed
     * to openFunc() (q.v.).
     * @return A newly allocated target fd, or -1 on error.
     */
    virtual int open(ThreadContext *tc, int mode, int flags) = 0;

    /**
     * Abstract method, invoked when the user program calls ioctl() on
     * the file descriptor returned by a previous open().  The parameters
     * are the same as those passed in to ioctlFunc() (q.v.).
     * @return The return code for the ioctl, or the negation of the errno
     * (see the SyscallReturn class).
     */
    virtual int ioctl(ThreadContext *tc, unsigned req, Addr buf) = 0;

    /**
     * Virtual method, invoked when the user program calls mmap() on
     * the file descriptor returned by a previous open().  The parameters
     * are the same as those passed in to mmapFunc() (q.v.).
     * @return The return ptr for the mmap, or the negation of the errno
     * (see the SyscallReturn class).
     */
    virtual Addr
    mmap(ThreadContext *tc, Addr start, uint64_t length, int prot,
         int tgtFlags, int tgtFd, off_t offset)
    {
        return -EBADF;
    }
};

} // namespace gem5

#endif // __SIM_EMUL_DRIVER_HH

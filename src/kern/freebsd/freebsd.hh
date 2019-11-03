/*
 * Copyright (c) 2015 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
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

#ifndef __FREEBSD_HH__
#define __FREEBSD_HH__

#include <string>

#include "base/types.hh"
#include "kern/operatingsystem.hh"

class ThreadContext;
class Process;

///
/// This class encapsulates the types, structures, constants,
/// functions, and syscall-number mappings specific to the Alpha FreeBSD
/// syscall interface.
///
class FreeBSD : public OperatingSystem
{

  public:

    //@{
    /// Basic FreeBSD types.
    typedef uint64_t size_t;
    typedef uint64_t off_t;
    typedef int64_t time_t;
    typedef int64_t clock_t;
    typedef uint32_t uid_t;
    typedef uint32_t gid_t;
    //@}

    /// Clock ticks per second, for times().
    static const int M5_SC_CLK_TCK = 100;

    //@{
    /// ioctl() command codes.
    static const unsigned TGT_TIOCGETA   = 0x402c7413;
    static const unsigned TGT_TIOCSETA   = 0x802c7414;
    static const unsigned TGT_TIOCSETAW  = 0x802c7415;
    static const unsigned TGT_FIONREAD   = 0x4004667f;
    //@}

    /// Return true for the ioctl codes for which we return ENOTTY
    /// *without* printing a warning, since we know that ENOTTY is the
    /// correct thing to return (and not just a sign that we don't
    /// recognize the ioctl code.
    static bool
    isTtyReq(unsigned req)
    {
        switch (req) {
          case TGT_TIOCGETA:
          case TGT_TIOCSETA:
          case TGT_TIOCSETAW:
          case TGT_FIONREAD:
            return true;
          default:
            return false;
        }
    }

    /// Resource constants for getrlimit().
    static const unsigned TGT_RLIMIT_CPU = 0;
    static const unsigned TGT_RLIMIT_FSIZE = 1;
    static const unsigned TGT_RLIMIT_DATA = 2;
    static const unsigned TGT_RLIMIT_STACK = 3;
    static const unsigned TGT_RLIMIT_CORE = 4;
    static const unsigned TGT_RLIMIT_RSS = 5;
    static const unsigned TGT_RLIMIT_MEMLOCK = 6;
    static const unsigned TGT_RLIMIT_NPROC = 7;
    static const unsigned TGT_RLIMIT_NOFILE = 8;
    static const unsigned TGT_RLIMIT_SBSIZE = 9;
    static const unsigned TGT_RLIMIT_VMEM = 10;
    static const unsigned TGT_RLIMIT_AS = TGT_RLIMIT_VMEM;
    static const unsigned TGT_RLIMIT_NPTS = 11;
    static const unsigned TGT_RLIMIT_SWAP = 12;
    static const unsigned TGT_RLIMIT_KQUEUES = 13;

    /// For getrusage().
    static const int TGT_RUSAGE_SELF     = 0;
    static const int TGT_RUSAGE_CHILDREN = -1;
    static const int TGT_RUSAGE_THREAD   = 1;

    // for *at syscalls
    static const int TGT_AT_FDCWD   = -100;

};  // class FreeBSD

#endif // __FREEBSD_HH__

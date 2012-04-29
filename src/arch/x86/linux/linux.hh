/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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
 */

#ifndef __ARCH_X86_LINUX_LINUX_HH__
#define __ARCH_X86_LINUX_LINUX_HH__

#include "kern/linux/linux.hh"

class X86Linux64 : public Linux
{
  public:

    typedef struct {
        uint64_t st_dev;
        uint64_t st_ino;
        uint64_t st_nlink;
        uint32_t st_mode;
        uint32_t st_uid;
        uint32_t st_gid;
        uint32_t __pad0;
        uint64_t st_rdev;
        int64_t st_size;
        int64_t st_blksize;
        int64_t st_blocks;
        uint64_t st_atimeX;
        uint64_t st_atime_nsec;
        uint64_t st_mtimeX;
        uint64_t st_mtime_nsec;
        uint64_t st_ctimeX;
        uint64_t st_ctime_nsec;
        int64_t unused0[3];
    } tgt_stat64;

    static OpenFlagTransTable openFlagTable[];

    static const int TGT_O_RDONLY       = 00000000;     //!< O_RDONLY
    static const int TGT_O_WRONLY       = 00000001;     //!< O_WRONLY
    static const int TGT_O_RDWR         = 00000002;     //!< O_RDWR
    static const int TGT_O_NONBLOCK     = 00004000;     //!< O_NONBLOCK
    static const int TGT_O_APPEND       = 00002000;     //!< O_APPEND
    static const int TGT_O_CREAT        = 00000100;     //!< O_CREAT
    static const int TGT_O_TRUNC        = 00001000;     //!< O_TRUNC
    static const int TGT_O_EXCL         = 00000200;     //!< O_EXCL
    static const int TGT_O_NOCTTY       = 00000400;     //!< O_NOCTTY
    static const int TGT_O_SYNC         = 00010000;     //!< O_SYNC
    static const int TGT_O_ASYNC        = 00020000;
    static const int TGT_O_DIRECT       = 00040000;     //!< O_DIRECTIO
    static const int TGT_O_LARGEFILE    = 00100000;
    static const int TGT_O_DIRECTORY    = 00200000;
    static const int TGT_O_NOFOLLOW     = 00400000;
    static const int TGT_O_NOATIME      = 01000000;
    static const int TGT_O_CLOEXEC      = 02000000;

    static const int NUM_OPEN_FLAGS;

    static const unsigned TGT_MAP_ANONYMOUS = 0x20;
    static const unsigned TGT_MAP_FIXED     = 0x10;

    typedef struct {
        uint64_t iov_base; // void *
        uint64_t iov_len;  // size_t
    } tgt_iovec;

    typedef struct {
        int64_t  uptime;    /* Seconds since boot */
        uint64_t loads[3];  /* 1, 5, and 15 minute load averages */
        uint64_t totalram;  /* Total usable main memory size */
        uint64_t freeram;   /* Available memory size */
        uint64_t sharedram; /* Amount of shared memory */
        uint64_t bufferram; /* Memory used by buffers */
        uint64_t totalswap; /* Total swap space size */
        uint64_t freeswap;  /* swap space still available */
        uint16_t procs;     /* Number of current processes */
        uint64_t totalhigh; /* Total high memory size */
        uint64_t freehigh;  /* Available high memory size */
        uint64_t mem_unit;  /* Memory unit size in bytes */
   } tgt_sysinfo;

};

class X86Linux32 : public Linux
{
  public:

    typedef struct {
        uint64_t st_dev;
        uint8_t __pad0[4];
        uint32_t __st_ino;
        uint32_t st_mode;
        uint32_t st_nlink;
        uint32_t st_uid;
        uint32_t st_gid;
        uint64_t st_rdev;
        uint8_t __pad3[4];
        int64_t st_size;
        uint32_t st_blksize;
        uint64_t st_blocks;
        uint32_t st_atimeX;
        uint32_t st_atime_nsec;
        uint32_t st_mtimeX;
        uint32_t st_mtime_nsec;
        uint32_t st_ctimeX;
        uint32_t st_ctime_nsec;
        uint64_t st_ino;
    } __attribute__((__packed__)) tgt_stat64;

    static OpenFlagTransTable openFlagTable[];

    static const int TGT_O_RDONLY       = 00000000;     //!< O_RDONLY
    static const int TGT_O_WRONLY       = 00000001;     //!< O_WRONLY
    static const int TGT_O_RDWR         = 00000002;     //!< O_RDWR
    static const int TGT_O_NONBLOCK     = 00004000;     //!< O_NONBLOCK
    static const int TGT_O_APPEND       = 00002000;     //!< O_APPEND
    static const int TGT_O_CREAT        = 00000100;     //!< O_CREAT
    static const int TGT_O_TRUNC        = 00001000;     //!< O_TRUNC
    static const int TGT_O_EXCL         = 00000200;     //!< O_EXCL
    static const int TGT_O_NOCTTY       = 00000400;     //!< O_NOCTTY
    static const int TGT_O_SYNC         = 00010000;     //!< O_SYNC
    static const int TGT_O_ASYNC        = 00020000;
    static const int TGT_O_DIRECT       = 00040000;     //!< O_DIRECTIO
    static const int TGT_O_LARGEFILE    = 00100000;
    static const int TGT_O_DIRECTORY    = 00200000;
    static const int TGT_O_NOFOLLOW     = 00400000;
    static const int TGT_O_NOATIME      = 01000000;
    static const int TGT_O_CLOEXEC      = 02000000;

    static const int NUM_OPEN_FLAGS;

    static const unsigned TGT_MAP_ANONYMOUS = 0x20;
    static const unsigned TGT_MAP_FIXED     = 0x10;

    typedef struct {
       int32_t  uptime;    /* Seconds since boot */
       uint32_t loads[3];  /* 1, 5, and 15 minute load averages */
       uint32_t totalram;  /* Total usable main memory size */
       uint32_t freeram;   /* Available memory size */
       uint32_t sharedram; /* Amount of shared memory */
       uint32_t bufferram; /* Memory used by buffers */
       uint32_t totalswap; /* Total swap space size */
       uint32_t freeswap;  /* swap space still available */
       uint16_t procs;     /* Number of current processes */
       uint32_t totalhigh; /* Total high memory size */
       uint32_t freehigh;  /* Available high memory size */
       uint32_t mem_unit;  /* Memory unit size in bytes */
    } tgt_sysinfo;

    static bool mmapGrowsDown() { return true; }
};

#endif

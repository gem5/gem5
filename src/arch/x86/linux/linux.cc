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

#include "arch/x86/linux/linux.hh"

#include <fcntl.h>
#include <sys/mman.h>

// open(2) flags translation table
SyscallFlagTransTable X86Linux64::openFlagTable[] = {
#ifdef _MSC_VER
  { X86Linux64::TGT_O_RDONLY,    _O_RDONLY },
  { X86Linux64::TGT_O_WRONLY,    _O_WRONLY },
  { X86Linux64::TGT_O_RDWR,      _O_RDWR },
  { X86Linux64::TGT_O_CREAT,     _O_CREAT },
  { X86Linux64::TGT_O_EXCL,      _O_EXCL },
#ifdef _O_NOCTTY
  { X86Linux64::TGT_O_NOCTTY,    _O_NOCTTY },
#endif
  { X86Linux64::TGT_O_TRUNC,     _O_TRUNC },
  { X86Linux64::TGT_O_APPEND,    _O_APPEND },
#ifdef _O_NONBLOCK
  { X86Linux64::TGT_O_NONBLOCK,  _O_NONBLOCK },
#endif
#ifdef _O_DSYNC
  { X86Linux64::TGT_O_DSYNC ,    _O_DSYNC },
#endif
  { X86Linux64::TGT_FASYNC,      _O_ASYNC },
  { X86Linux64::TGT_O_DIRECT,    _O_DIRECT },
  { X86Linux64::TGT_O_LARGEFILE, _O_LARGEFILE },
  { X86Linux64::TGT_O_DIRECTORY, _O_DIRECTORY },
  { X86Linux64::TGT_O_NOFOLLOW,  _O_NOFOLLOW },
  { X86Linux64::TGT_O_NOATIME,   _O_NOATIME },
#ifdef _O_CLOEXEC
  { X86Linux64::TGT_O_CLOEXEC,   _O_CLOEXEC },
#endif
#ifdef _O_SYNC
  { X86Linux64::TGT_O_SYNC,      _O_SYNC },
#endif
#ifdef _O_PATH
  { X86Linux64::TGT_O_PATH ,     _O_PATH },
#endif
#else /* !_MSC_VER */
  { X86Linux64::TGT_O_RDONLY,    O_RDONLY },
  { X86Linux64::TGT_O_WRONLY,    O_WRONLY },
  { X86Linux64::TGT_O_RDWR,      O_RDWR },
  { X86Linux64::TGT_O_CREAT,     O_CREAT },
  { X86Linux64::TGT_O_EXCL,      O_EXCL },
  { X86Linux64::TGT_O_NOCTTY,    O_NOCTTY },
  { X86Linux64::TGT_O_TRUNC,     O_TRUNC },
  { X86Linux64::TGT_O_APPEND,    O_APPEND },
  { X86Linux64::TGT_O_NONBLOCK,  O_NONBLOCK },
#ifdef O_DSYNC
  { X86Linux64::TGT_O_DSYNC,     O_DSYNC },
#endif
  { X86Linux64::TGT_FASYNC,      O_ASYNC },
  { X86Linux64::TGT_O_DIRECT,    O_DIRECT },
  { X86Linux64::TGT_O_LARGEFILE, O_LARGEFILE },
  { X86Linux64::TGT_O_DIRECTORY, O_DIRECTORY },
  { X86Linux64::TGT_O_NOFOLLOW,  O_NOFOLLOW },
  { X86Linux64::TGT_O_NOATIME,   O_NOATIME },
#ifdef O_CLOEXEC
  { X86Linux64::TGT_O_CLOEXEC,   O_CLOEXEC },
#endif
#ifdef O_SYNC
  { X86Linux64::TGT_O_SYNC,      O_SYNC },
#endif
#ifdef O_PATH
  { X86Linux64::TGT_O_PATH,      O_PATH },
#endif
#endif /* _MSC_VER */
};

const int X86Linux64::NUM_OPEN_FLAGS =
        sizeof(X86Linux64::openFlagTable) /
        sizeof(X86Linux64::openFlagTable[0]);

// mmap(2) flags translation table
SyscallFlagTransTable X86Linux64::mmapFlagTable[] = {
  { X86Linux64::TGT_MAP_SHARED,     MAP_SHARED },
  { X86Linux64::TGT_MAP_PRIVATE,    MAP_PRIVATE },
  { X86Linux64::TGT_MAP_32BIT,      MAP_32BIT},
  { X86Linux64::TGT_MAP_ANON,       MAP_ANON },
  { X86Linux64::TGT_MAP_DENYWRITE,  MAP_DENYWRITE },
  { X86Linux64::TGT_MAP_EXECUTABLE, MAP_EXECUTABLE },
  { X86Linux64::TGT_MAP_FILE,       MAP_FILE },
  { X86Linux64::TGT_MAP_GROWSDOWN,  MAP_GROWSDOWN },
#ifdef MAP_HUGETLB
  { X86Linux64::TGT_MAP_HUGETLB,    MAP_HUGETLB },
#endif
  { X86Linux64::TGT_MAP_LOCKED,     MAP_LOCKED },
  { X86Linux64::TGT_MAP_NONBLOCK,   MAP_NONBLOCK },
  { X86Linux64::TGT_MAP_NORESERVE,  MAP_NORESERVE },
  { X86Linux64::TGT_MAP_POPULATE,   MAP_POPULATE },
#ifdef MAP_STACK
  { X86Linux64::TGT_MAP_STACK,      MAP_STACK },
#endif
  { X86Linux64::TGT_MAP_ANONYMOUS,  MAP_ANONYMOUS },
  { X86Linux64::TGT_MAP_FIXED,      MAP_FIXED },
};

const unsigned X86Linux64::NUM_MMAP_FLAGS =
        sizeof(X86Linux64::mmapFlagTable) /
        sizeof(X86Linux64::mmapFlagTable[0]);

// open(2) flags translation table
SyscallFlagTransTable X86Linux32::openFlagTable[] = {
#ifdef _MSC_VER
  { X86Linux32::TGT_O_RDONLY,    _O_RDONLY },
  { X86Linux32::TGT_O_WRONLY,    _O_WRONLY },
  { X86Linux32::TGT_O_RDWR,      _O_RDWR },
  { X86Linux32::TGT_O_CREAT,     _O_CREAT },
  { X86Linux32::TGT_O_EXCL,      _O_EXCL },
#ifdef _O_NOCTTY
  { X86Linux32::TGT_O_NOCTTY,    _O_NOCTTY },
#endif
  { X86Linux32::TGT_O_TRUNC,     _O_TRUNC },
  { X86Linux32::TGT_O_APPEND,    _O_APPEND },
#ifdef _O_NONBLOCK
  { X86Linux32::TGT_O_NONBLOCK,  _O_NONBLOCK },
#endif
#ifdef O_DSYNC
  { X86Linux32::TGT_O_DSYNC,     _O_DSYNC },
#endif
  { X86Linux32::TGT_FASYNC,      _O_ASYNC },
  { X86Linux32::TGT_O_DIRECT,    _O_DIRECT },
  { X86Linux32::TGT_O_LARGEFILE, _O_LARGEFILE },
  { X86Linux32::TGT_O_DIRECTORY, _O_DIRECTORY },
  { X86Linux32::TGT_O_NOFOLLOW,  _O_NOFOLLOW },
  { X86Linux32::TGT_O_NOATIME,   _O_NOATIME },
#ifdef _O_CLOEXEC
  { X86Linux32::TGT_O_CLOEXEC,   _O_CLOEXEC },
#endif
#ifdef _O_SYNC
  { X86Linux32::TGT_O_SYNC,      _O_SYNC },
#endif
#ifdef _O_PATH
  { X86Linux32::TGT_O_PATH,      _O_PATH },
#endif
#else /* !_MSC_VER */
  { X86Linux32::TGT_O_RDONLY,    O_RDONLY },
  { X86Linux32::TGT_O_WRONLY,    O_WRONLY },
  { X86Linux32::TGT_O_RDWR,      O_RDWR },
  { X86Linux32::TGT_O_CREAT,     O_CREAT },
  { X86Linux32::TGT_O_EXCL,      O_EXCL },
  { X86Linux32::TGT_O_NOCTTY,    O_NOCTTY },
  { X86Linux32::TGT_O_TRUNC,     O_TRUNC },
  { X86Linux32::TGT_O_APPEND,    O_APPEND },
  { X86Linux32::TGT_O_NONBLOCK,  O_NONBLOCK },
#ifdef O_DSYNC
  { X86Linux32::TGT_O_DSYNC,     O_DSYNC },
#endif
  { X86Linux32::TGT_FASYNC,      O_ASYNC },
  { X86Linux32::TGT_O_DIRECT,    O_DIRECT },
  { X86Linux32::TGT_O_LARGEFILE, O_LARGEFILE },
  { X86Linux32::TGT_O_DIRECTORY, O_DIRECTORY },
  { X86Linux32::TGT_O_NOFOLLOW,  O_NOFOLLOW },
  { X86Linux32::TGT_O_NOATIME,   O_NOATIME },
#ifdef O_CLOEXEC
  { X86Linux32::TGT_O_CLOEXEC,   O_CLOEXEC },
#endif
#ifdef O_SYNC
  { X86Linux32::TGT_O_SYNC,      O_SYNC },
#endif
#ifdef O_PATH
  { X86Linux32::TGT_O_PATH,      O_PATH },
#endif
#endif /* _MSC_VER */
};

const int X86Linux32::NUM_OPEN_FLAGS =
        sizeof(X86Linux32::openFlagTable) /
        sizeof(X86Linux32::openFlagTable[0]);

// mmap(2) flags translation table
SyscallFlagTransTable X86Linux32::mmapFlagTable[] = {
  { X86Linux32::TGT_MAP_SHARED,     MAP_SHARED },
  { X86Linux32::TGT_MAP_PRIVATE,    MAP_PRIVATE },
  { X86Linux32::TGT_MAP_32BIT,      MAP_32BIT},
  { X86Linux32::TGT_MAP_ANON,       MAP_ANON },
  { X86Linux32::TGT_MAP_DENYWRITE,  MAP_DENYWRITE },
  { X86Linux32::TGT_MAP_EXECUTABLE, MAP_EXECUTABLE },
  { X86Linux32::TGT_MAP_FILE,       MAP_FILE },
  { X86Linux32::TGT_MAP_GROWSDOWN,  MAP_GROWSDOWN },
#ifdef MAP_HUGETLB
  { X86Linux32::TGT_MAP_HUGETLB,    MAP_HUGETLB },
#endif
  { X86Linux32::TGT_MAP_LOCKED,     MAP_LOCKED },
  { X86Linux32::TGT_MAP_NONBLOCK,   MAP_NONBLOCK },
  { X86Linux32::TGT_MAP_NORESERVE,  MAP_NORESERVE },
  { X86Linux32::TGT_MAP_POPULATE,   MAP_POPULATE },
#ifdef MAP_STACK
  { X86Linux32::TGT_MAP_STACK,      MAP_STACK },
#endif
  { X86Linux32::TGT_MAP_ANONYMOUS,  MAP_ANONYMOUS },
  { X86Linux32::TGT_MAP_FIXED,      MAP_FIXED },
};

const unsigned X86Linux32::NUM_MMAP_FLAGS =
        sizeof(X86Linux32::mmapFlagTable) /
        sizeof(X86Linux32::mmapFlagTable[0]);

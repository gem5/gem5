/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 */

#include "arch/mips/linux/linux.hh"

#include <fcntl.h>
#include <sys/mman.h>

// open(2) flags translation table
SyscallFlagTransTable MipsLinux::openFlagTable[] = {
#ifdef _MSC_VER
  { MipsLinux::TGT_O_RDONLY,    _O_RDONLY },
  { MipsLinux::TGT_O_WRONLY,    _O_WRONLY },
  { MipsLinux::TGT_O_RDWR,      _O_RDWR },
  { MipsLinux::TGT_O_CREAT,     _O_CREAT },
  { MipsLinux::TGT_O_EXCL,      _O_EXCL },
#ifdef _O_NOCTTY
  { MipsLinux::TGT_O_NOCTTY,    _O_NOCTTY },
#endif
  { MipsLinux::TGT_O_TRUNC,     _O_TRUNC },
  { MipsLinux::TGT_O_APPEND,    _O_APPEND },
#ifdef _O_NONBLOCK
  { MipsLinux::TGT_O_NONBLOCK,  _O_NONBLOCK },
#endif
#ifdef _O_DSYNC
  { MipsLinux::TGT_O_DSYNC,      _O_DSYNC },
#endif
  { MipsLinux::TGT_O_DIRECT,    _O_DIRECT },
  { MipsLinux::TGT_O_LARGEFILE, _O_LARGEFILE },
  { MipsLinux::TGT_O_DIRECTORY, _O_DIRECTORY },
  { MipsLinux::TGT_O_NOFOLLOW,  _O_NOFOLLOW },
  { MipsLinux::TGT_O_NOATIME,   _O_NOATIME },
#ifdef _O_CLOEXEC
  { MipsLinux::TGT_O_CLOEXEC,   _O_CLOEXEC },
#endif
#ifdef _O_SYNC
  { MipsLinux::TGT_O_SYNC,      _O_SYNC },
#endif
#ifdef _O_PATH
  { MipsLinux::TGT_O_PATH,      _O_PATH },
#endif
#else /* !_MSC_VER */
  { MipsLinux::TGT_O_RDONLY,    O_RDONLY },
  { MipsLinux::TGT_O_WRONLY,    O_WRONLY },
  { MipsLinux::TGT_O_RDWR,      O_RDWR },
  { MipsLinux::TGT_O_CREAT,     O_CREAT },
  { MipsLinux::TGT_O_EXCL,      O_EXCL },
  { MipsLinux::TGT_O_NOCTTY,    O_NOCTTY },
  { MipsLinux::TGT_O_TRUNC,     O_TRUNC },
  { MipsLinux::TGT_O_APPEND,    O_APPEND },
  { MipsLinux::TGT_O_NONBLOCK,  O_NONBLOCK },
#ifdef O_DSYNC
  { MipsLinux::TGT_O_DSYNC,     O_DSYNC },
#endif
  { MipsLinux::TGT_O_DIRECT,    O_DIRECT },
  { MipsLinux::TGT_O_LARGEFILE, O_LARGEFILE },
  { MipsLinux::TGT_O_DIRECTORY, O_DIRECTORY },
  { MipsLinux::TGT_O_NOFOLLOW,  O_NOFOLLOW },
  { MipsLinux::TGT_O_NOATIME,   O_NOATIME },
#ifdef O_CLOEXEC
  { MipsLinux::TGT_O_CLOEXEC,   O_CLOEXEC },
#endif
#ifdef O_SYNC
  { MipsLinux::TGT_O_SYNC,      O_SYNC },
#endif
#ifdef O_PATH
  { MipsLinux::TGT_O_PATH,      O_PATH },
#endif
#endif /* _MSC_VER */
};

const int MipsLinux::NUM_OPEN_FLAGS =
        (sizeof(MipsLinux::openFlagTable)/sizeof(MipsLinux::openFlagTable[0]));

// mmap(2) flags translation table
SyscallFlagTransTable MipsLinux::mmapFlagTable[] = {
  { MipsLinux::TGT_MAP_SHARED,     MAP_SHARED },
  { MipsLinux::TGT_MAP_PRIVATE,    MAP_PRIVATE },
  { MipsLinux::TGT_MAP_ANON,       MAP_ANON },
  { MipsLinux::TGT_MAP_DENYWRITE,  MAP_DENYWRITE },
  { MipsLinux::TGT_MAP_EXECUTABLE, MAP_EXECUTABLE },
  { MipsLinux::TGT_MAP_FILE,       MAP_FILE },
  { MipsLinux::TGT_MAP_GROWSDOWN,  MAP_GROWSDOWN },
  { MipsLinux::TGT_MAP_HUGETLB,    MAP_HUGETLB },
  { MipsLinux::TGT_MAP_LOCKED,     MAP_LOCKED },
  { MipsLinux::TGT_MAP_NONBLOCK,   MAP_NONBLOCK },
  { MipsLinux::TGT_MAP_NORESERVE,  MAP_NORESERVE },
  { MipsLinux::TGT_MAP_POPULATE,   MAP_POPULATE },
#ifdef MAP_STACK
  { MipsLinux::TGT_MAP_STACK,      MAP_STACK },
#endif
  { MipsLinux::TGT_MAP_ANONYMOUS,  MAP_ANONYMOUS },
  { MipsLinux::TGT_MAP_FIXED,      MAP_FIXED },
};

const unsigned MipsLinux::NUM_MMAP_FLAGS =
        sizeof(MipsLinux::mmapFlagTable) /
        sizeof(MipsLinux::mmapFlagTable[0]);


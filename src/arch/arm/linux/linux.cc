/*
 * Copyright (c) 2011 ARM Limited
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
 * Copyright (c) 2007-2008 The Florida State University
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
 */

#include <fcntl.h>

#include "arch/arm/linux/linux.hh"

// open(2) flags translation table
OpenFlagTransTable ArmLinux32::openFlagTable[] = {
#ifdef _MSC_VER
  { ArmLinux32::TGT_O_RDONLY,     _O_RDONLY },
  { ArmLinux32::TGT_O_WRONLY,     _O_WRONLY },
  { ArmLinux32::TGT_O_RDWR,       _O_RDWR },
  { ArmLinux32::TGT_O_APPEND,     _O_APPEND },
  { ArmLinux32::TGT_O_CREAT,      _O_CREAT },
  { ArmLinux32::TGT_O_TRUNC,      _O_TRUNC },
  { ArmLinux32::TGT_O_EXCL,       _O_EXCL },
#ifdef _O_NONBLOCK
  { ArmLinux32::TGT_O_NONBLOCK,   _O_NONBLOCK },
#endif
#ifdef _O_NOCTTY
  { ArmLinux32::TGT_O_NOCTTY,     _O_NOCTTY },
#endif
#ifdef _O_SYNC
  { ArmLinux32::TGT_O_SYNC,       _O_SYNC },
#endif
#else /* !_MSC_VER */
  { ArmLinux32::TGT_O_RDONLY,     O_RDONLY },
  { ArmLinux32::TGT_O_WRONLY,     O_WRONLY },
  { ArmLinux32::TGT_O_RDWR,       O_RDWR },
  { ArmLinux32::TGT_O_CREAT,      O_CREAT },
  { ArmLinux32::TGT_O_EXCL,       O_EXCL },
  { ArmLinux32::TGT_O_NOCTTY,     O_NOCTTY },
  { ArmLinux32::TGT_O_TRUNC,      O_TRUNC },
  { ArmLinux32::TGT_O_APPEND,     O_APPEND },
  { ArmLinux32::TGT_O_NONBLOCK,   O_NONBLOCK },
#ifdef O_SYNC
  { ArmLinux32::TGT_O_SYNC,       O_SYNC },
#endif
#ifdef FASYNC
  { ArmLinux32::TGT_FASYNC,       FASYNC },
#endif
#ifdef O_DIRECT
  { ArmLinux32::TGT_O_DIRECT,     O_DIRECT },
#endif
#ifdef O_LARGEFILE
  { ArmLinux32::TGT_O_LARGEFILE,  O_LARGEFILE },
#endif
#ifdef O_DIRECTORY
  { ArmLinux32::TGT_O_DIRECTORY,  O_DIRECTORY },
#endif
#ifdef O_NOFOLLOW
  { ArmLinux32::TGT_O_NOFOLLOW,   O_NOFOLLOW },
#endif
#endif /* _MSC_VER */
};

const int ArmLinux32::NUM_OPEN_FLAGS = sizeof(ArmLinux32::openFlagTable) /
                                       sizeof(ArmLinux32::openFlagTable[0]);

// open(2) flags translation table
OpenFlagTransTable ArmLinux64::openFlagTable[] = {
#ifdef _MSC_VER
  { ArmLinux64::TGT_O_RDONLY,     _O_RDONLY },
  { ArmLinux64::TGT_O_WRONLY,     _O_WRONLY },
  { ArmLinux64::TGT_O_RDWR,       _O_RDWR },
  { ArmLinux64::TGT_O_APPEND,     _O_APPEND },
  { ArmLinux64::TGT_O_CREAT,      _O_CREAT },
  { ArmLinux64::TGT_O_TRUNC,      _O_TRUNC },
  { ArmLinux64::TGT_O_EXCL,       _O_EXCL },
#ifdef _O_NONBLOCK
  { ArmLinux64::TGT_O_NONBLOCK,   _O_NONBLOCK },
#endif
#ifdef _O_NOCTTY
  { ArmLinux64::TGT_O_NOCTTY,     _O_NOCTTY },
#endif
#ifdef _O_SYNC
  { ArmLinux64::TGT_O_SYNC,       _O_SYNC },
#endif
#else /* !_MSC_VER */
  { ArmLinux64::TGT_O_RDONLY,     O_RDONLY },
  { ArmLinux64::TGT_O_WRONLY,     O_WRONLY },
  { ArmLinux64::TGT_O_RDWR,       O_RDWR },
  { ArmLinux64::TGT_O_CREAT,      O_CREAT },
  { ArmLinux64::TGT_O_EXCL,       O_EXCL },
  { ArmLinux64::TGT_O_NOCTTY,     O_NOCTTY },
  { ArmLinux64::TGT_O_TRUNC,      O_TRUNC },
  { ArmLinux64::TGT_O_APPEND,     O_APPEND },
  { ArmLinux64::TGT_O_NONBLOCK,   O_NONBLOCK },
#ifdef O_SYNC
  { ArmLinux64::TGT_O_SYNC,       O_SYNC },
#endif
#ifdef FASYNC
  { ArmLinux64::TGT_FASYNC,       FASYNC },
#endif
#ifdef O_DIRECT
  { ArmLinux64::TGT_O_DIRECT,     O_DIRECT },
#endif
#ifdef O_LARGEFILE
  { ArmLinux64::TGT_O_LARGEFILE,  O_LARGEFILE },
#endif
#ifdef O_DIRECTORY
  { ArmLinux64::TGT_O_DIRECTORY,  O_DIRECTORY },
#endif
#ifdef O_NOFOLLOW
  { ArmLinux64::TGT_O_NOFOLLOW,   O_NOFOLLOW },
#endif
#endif /* _MSC_VER */
};

const int ArmLinux64::NUM_OPEN_FLAGS = sizeof(ArmLinux64::openFlagTable) /
                                       sizeof(ArmLinux64::openFlagTable[0]);


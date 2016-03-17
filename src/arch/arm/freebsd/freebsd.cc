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

#include "arch/arm/freebsd/freebsd.hh"

#include <fcntl.h>

// open(2) flags translation table
SyscallFlagTransTable ArmFreebsd32::openFlagTable[] = {
  { ArmFreebsd32::TGT_O_RDONLY,     O_RDONLY },
  { ArmFreebsd32::TGT_O_WRONLY,     O_WRONLY },
  { ArmFreebsd32::TGT_O_RDWR,       O_RDWR },
  { ArmFreebsd32::TGT_O_CREAT,      O_CREAT },
  { ArmFreebsd32::TGT_O_EXCL,       O_EXCL },
  { ArmFreebsd32::TGT_O_NOCTTY,     O_NOCTTY },
  { ArmFreebsd32::TGT_O_TRUNC,      O_TRUNC },
  { ArmFreebsd32::TGT_O_APPEND,     O_APPEND },
  { ArmFreebsd32::TGT_O_NONBLOCK,   O_NONBLOCK },
  { ArmFreebsd32::TGT_O_SYNC,       O_SYNC },
  { ArmFreebsd32::TGT_FASYNC,       FASYNC },
#ifdef O_DIRECT
  { ArmFreebsd32::TGT_O_DIRECT,     O_DIRECT },
#endif
  { ArmFreebsd32::TGT_O_DIRECTORY,  O_DIRECTORY },
  { ArmFreebsd32::TGT_O_NOFOLLOW,   O_NOFOLLOW },
};

const int ArmFreebsd32::NUM_OPEN_FLAGS = sizeof(ArmFreebsd32::openFlagTable) /
                                       sizeof(ArmFreebsd32::openFlagTable[0]);

// open(2) flags translation table
SyscallFlagTransTable ArmFreebsd64::openFlagTable[] = {
  { ArmFreebsd64::TGT_O_RDONLY,     O_RDONLY },
  { ArmFreebsd64::TGT_O_WRONLY,     O_WRONLY },
  { ArmFreebsd64::TGT_O_RDWR,       O_RDWR },
  { ArmFreebsd64::TGT_O_CREAT,      O_CREAT },
  { ArmFreebsd64::TGT_O_EXCL,       O_EXCL },
  { ArmFreebsd64::TGT_O_NOCTTY,     O_NOCTTY },
  { ArmFreebsd64::TGT_O_TRUNC,      O_TRUNC },
  { ArmFreebsd64::TGT_O_APPEND,     O_APPEND },
  { ArmFreebsd64::TGT_O_NONBLOCK,   O_NONBLOCK },
  { ArmFreebsd64::TGT_O_SYNC,       O_SYNC },
  { ArmFreebsd64::TGT_FASYNC,       FASYNC },
#ifdef O_DIRECT
  { ArmFreebsd64::TGT_O_DIRECT,     O_DIRECT },
#endif
  { ArmFreebsd64::TGT_O_DIRECTORY,  O_DIRECTORY },
  { ArmFreebsd64::TGT_O_NOFOLLOW,   O_NOFOLLOW },
};

const int ArmFreebsd64::NUM_OPEN_FLAGS = sizeof(ArmFreebsd64::openFlagTable) /
                                       sizeof(ArmFreebsd64::openFlagTable[0]);


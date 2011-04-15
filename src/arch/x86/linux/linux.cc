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

#include <fcntl.h>

#include "arch/x86/linux/linux.hh"

// open(2) flags translation table
OpenFlagTransTable X86Linux64::openFlagTable[] = {
#ifdef _MSC_VER
  { TGT_O_RDONLY, _O_RDONLY },
  { TGT_O_WRONLY, _O_WRONLY },
  { TGT_O_RDWR, _O_RDWR },
  { TGT_O_APPEND, _O_APPEND },
  { TGT_O_CREAT, _O_CREAT },
  { TGT_O_TRUNC, _O_TRUNC },
  { TGT_O_EXCL, _O_EXCL },
#ifdef _O_NONBLOCK
  { TGT_O_NONBLOCK, _O_NONBLOCK },
#endif
#ifdef _O_NOCTTY
  { TGT_O_NOCTTY, _O_NOCTTY },
#endif
#ifdef _O_SYNC
  { TGT_O_SYNC, _O_SYNC },
#endif
#else /* !_MSC_VER */
  { TGT_O_RDONLY, O_RDONLY },
  { TGT_O_WRONLY, O_WRONLY },
  { TGT_O_RDWR, O_RDWR },
  { TGT_O_APPEND, O_APPEND },
  { TGT_O_CREAT, O_CREAT },
  { TGT_O_TRUNC, O_TRUNC },
  { TGT_O_EXCL, O_EXCL },
  { TGT_O_NONBLOCK, O_NONBLOCK },
  { TGT_O_NOCTTY, O_NOCTTY },
#ifdef O_SYNC
  { TGT_O_SYNC, O_SYNC },
#endif
#endif /* _MSC_VER */
};

const int X86Linux64::NUM_OPEN_FLAGS =
        sizeof(X86Linux64::openFlagTable) /
        sizeof(X86Linux64::openFlagTable[0]);

// open(2) flags translation table
OpenFlagTransTable X86Linux32::openFlagTable[] = {
#ifdef _MSC_VER
  { TGT_O_RDONLY, _O_RDONLY },
  { TGT_O_WRONLY, _O_WRONLY },
  { TGT_O_RDWR, _O_RDWR },
  { TGT_O_APPEND, _O_APPEND },
  { TGT_O_CREAT, _O_CREAT },
  { TGT_O_TRUNC, _O_TRUNC },
  { TGT_O_EXCL, _O_EXCL },
#ifdef _O_NONBLOCK
  { TGT_O_NONBLOCK, _O_NONBLOCK },
#endif
#ifdef _O_NOCTTY
  { TGT_O_NOCTTY, _O_NOCTTY },
#endif
#ifdef _O_SYNC
  { TGT_O_SYNC, _O_SYNC },
#endif
#else /* !_MSC_VER */
  { TGT_O_RDONLY, O_RDONLY },
  { TGT_O_WRONLY, O_WRONLY },
  { TGT_O_RDWR, O_RDWR },
  { TGT_O_APPEND, O_APPEND },
  { TGT_O_CREAT, O_CREAT },
  { TGT_O_TRUNC, O_TRUNC },
  { TGT_O_EXCL, O_EXCL },
  { TGT_O_NONBLOCK, O_NONBLOCK },
  { TGT_O_NOCTTY, O_NOCTTY },
#ifdef O_SYNC
  { TGT_O_SYNC, O_SYNC },
#endif
#endif /* _MSC_VER */
};

const int X86Linux32::NUM_OPEN_FLAGS =
        sizeof(X86Linux32::openFlagTable) /
        sizeof(X86Linux32::openFlagTable[0]);


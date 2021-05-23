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
 * Copyright (c) 2003-2005, 2006 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
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
 */

#include <fcntl.h>
#include <sys/mman.h>

/*
 * @file flag_tables.hh
 *
 * This file contains definitions of flag translation tables for the
 * open() system call.  Since the tables are nearly identical
 * across target platforms (other than the values of the target flags),
 * we use a template to build the tables which is inherited by each target
 * platform.
 */

namespace gem5
{

template <typename Target>
class OpenFlagTable
{
  public:
    // open(2) flags translation table
    static inline const std::map<int, int> openFlagTable = {
#ifdef _MSC_VER
      { Target::TGT_O_RDONLY,     _O_RDONLY },
      { Target::TGT_O_WRONLY,     _O_WRONLY },
      { Target::TGT_O_RDWR,       _O_RDWR },
      { Target::TGT_O_CREAT,      _O_CREAT },
      { Target::TGT_O_EXCL,       _O_EXCL },
      { Target::TGT_O_TRUNC,      _O_TRUNC },
      { Target::TGT_O_APPEND,     _O_APPEND },
#else /* !_MSC_VER */
      { Target::TGT_O_RDONLY,     O_RDONLY },
      { Target::TGT_O_WRONLY,     O_WRONLY },
      { Target::TGT_O_RDWR,       O_RDWR },
      { Target::TGT_O_CREAT,      O_CREAT },
      { Target::TGT_O_EXCL,       O_EXCL },
      { Target::TGT_O_TRUNC,      O_TRUNC },
      { Target::TGT_O_APPEND,     O_APPEND },
      { Target::TGT_O_NOCTTY,     O_NOCTTY },
      { Target::TGT_O_NONBLOCK,   O_NONBLOCK },
#ifdef O_DSYNC
      { Target::TGT_O_DSYNC,      O_DSYNC },
#endif
#ifdef O_ASYNC
      { Target::TGT_FASYNC,       O_ASYNC },
#endif
#ifdef O_DIRECT
      { Target::TGT_O_DIRECT,     O_DIRECT },
#endif
#ifdef O_LARGEFILE
      { Target::TGT_O_LARGEFILE,  O_LARGEFILE },
#endif
#ifdef O_DIRECTORY
      { Target::TGT_O_DIRECTORY,  O_DIRECTORY },
#endif
#ifdef O_NOFOLLOW
      { Target::TGT_O_NOFOLLOW,   O_NOFOLLOW },
#endif
#ifdef O_NOATIME
      { Target::TGT_O_NOATIME,    O_NOATIME },
#endif
#ifdef O_CLOEXEC
      { Target::TGT_O_CLOEXEC,    O_CLOEXEC },
#endif
#ifdef O_SYNC
      { Target::TGT_O_SYNC,       O_SYNC },
#endif
#ifdef O_PATH
      { Target::TGT_O_PATH,       O_PATH },
#endif
#endif /* _MSC_VER */
    };
};

} // namespace gem5

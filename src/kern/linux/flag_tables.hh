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
 *
 * Authors: Korey Sewell
 */

/*
 * @file flag_tables.hh
 *
 * This file contains definitions of flag translation tables for the
 * open() system call.  Since the tables are nearly identical
 * across target platforms (other than the values of the target flags),
 * we use a single copy of the tables but include it separately for
 * each target platform.  The TARGET macro must be #defined before including
 * to indicate the class for which the table is being defined.
 *
 * See src/arch/<*>/linux/linux.cc.
 */

// open(2) flags translation table
SyscallFlagTransTable TARGET::openFlagTable[] = {
#ifdef _MSC_VER
  { TARGET::TGT_O_RDONLY,     _O_RDONLY },
  { TARGET::TGT_O_WRONLY,     _O_WRONLY },
  { TARGET::TGT_O_RDWR,       _O_RDWR },
  { TARGET::TGT_O_CREAT,      _O_CREAT },
  { TARGET::TGT_O_EXCL,       _O_EXCL },
  { TARGET::TGT_O_TRUNC,      _O_TRUNC },
  { TARGET::TGT_O_APPEND,     _O_APPEND },
#else /* !_MSC_VER */
  { TARGET::TGT_O_RDONLY,     O_RDONLY },
  { TARGET::TGT_O_WRONLY,     O_WRONLY },
  { TARGET::TGT_O_RDWR,       O_RDWR },
  { TARGET::TGT_O_CREAT,      O_CREAT },
  { TARGET::TGT_O_EXCL,       O_EXCL },
  { TARGET::TGT_O_TRUNC,      O_TRUNC },
  { TARGET::TGT_O_APPEND,     O_APPEND },
  { TARGET::TGT_O_NOCTTY,     O_NOCTTY },
  { TARGET::TGT_O_NONBLOCK,   O_NONBLOCK },
#ifdef O_DSYNC
  { TARGET::TGT_O_DSYNC,      O_DSYNC },
#endif
#ifdef O_ASYNC
  { TARGET::TGT_FASYNC,       O_ASYNC },
#endif
#ifdef O_DIRECT
  { TARGET::TGT_O_DIRECT,     O_DIRECT },
#endif
#ifdef O_LARGEFILE
  { TARGET::TGT_O_LARGEFILE,  O_LARGEFILE },
#endif
#ifdef O_DIRECTORY
  { TARGET::TGT_O_DIRECTORY,  O_DIRECTORY },
#endif
#ifdef O_NOFOLLOW
  { TARGET::TGT_O_NOFOLLOW,   O_NOFOLLOW },
#endif
#ifdef O_NOATIME
  { TARGET::TGT_O_NOATIME,    O_NOATIME },
#endif
#ifdef O_CLOEXEC
  { TARGET::TGT_O_CLOEXEC,    O_CLOEXEC },
#endif
#ifdef O_SYNC
  { TARGET::TGT_O_SYNC,       O_SYNC },
#endif
#ifdef O_PATH
  { TARGET::TGT_O_PATH,       O_PATH },
#endif
#endif /* _MSC_VER */
};

const int TARGET::NUM_OPEN_FLAGS =
  sizeof(TARGET::openFlagTable) / sizeof(TARGET::openFlagTable[0]);

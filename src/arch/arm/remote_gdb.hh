/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Stephen Hines
 */

#ifndef __ARCH_ARM_REMOTE_GDB_HH__
#define __ARCH_ARM_REMOTE_GDB_HH__

#include "base/remote_gdb.hh"

class System;
class ThreadContext;

namespace ArmISA
{
// registers for arm with vfpv3/neon
const int NUMREGS   = 41;  /* r0-r15, cpsr, d0-d31, fpscr */
const int REG_R0 = 0;
const int REG_F0 = 8;
const int REG_CPSR  = 8;   /* bit 512 to bit 543  */
const int REG_FPSCR = 40;  /* bit 2592 to bit 2623 */

class RemoteGDB : public BaseRemoteGDB
{

protected:
  Addr notTakenBkpt;
  Addr takenBkpt;

protected:
  bool acc(Addr addr, size_t len);
  bool write(Addr addr, size_t size, const char *data);

  void getregs();
  void setregs();

  void clearSingleStep();
  void setSingleStep();

public:
  RemoteGDB(System *_system, ThreadContext *tc);
};
} // namespace ArmISA

#endif /* __ARCH_ARM_REMOTE_GDB_H__ */

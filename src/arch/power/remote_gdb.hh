/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Stephen Hines
 *          Timothy M. Jones
 */

#ifndef __ARCH_POWER_REMOTE_GDB_HH__
#define __ARCH_POWER_REMOTE_GDB_HH__

#include "arch/power/registers.hh"
#include "arch/power/remote_gdb.hh"
#include "base/remote_gdb.hh"

namespace PowerISA
{

const int GDB_REG_BYTES =
        NumIntArchRegs * 4 +
        NumFloatArchRegs * 8 +
        4 + /* PC  */
        4 + /* MSR */
        4 + /* CR  */
        4 + /* LR  */
        4 + /* CTR */
        4;  /* XER */
const int GdbFirstGPRIndex = 0;
const int GdbFirstFPRIndex = 16;
const int GdbPCIndex  = 96;
const int GdbMSRIndex = 97;
const int GdbCRIndex  = 98;
const int GdbLRIndex  = 99;
const int GdbCTRIndex = 100;
const int GdbXERIndex = 101;

class RemoteGDB : public BaseRemoteGDB
{
  public:
    RemoteGDB(System *_system, ThreadContext *tc);
  protected:
    bool acc(Addr addr, size_t len);
    void getregs();
    void setregs();
};

} // namespace PowerISA

#endif /* __ARCH_POWER_REMOTE_GDB_H__ */

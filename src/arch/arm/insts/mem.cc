/*
 * Copyright (c) 2010, 2012 ARM Limited
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

#include "arch/arm/insts/mem.hh"

#include "base/loader/symtab.hh"

using namespace std;

namespace ArmISA
{

void
MemoryReg::printOffset(std::ostream &os) const
{
    if (!add)
        os << "-";
    printIntReg(os, index);
    if (shiftType != LSL || shiftAmt != 0) {
        switch (shiftType) {
          case LSL:
            ccprintf(os, " LSL #%d", shiftAmt);
            break;
          case LSR:
            ccprintf(os, " LSR #%d", (shiftAmt == 0) ? 32 : shiftAmt);
            break;
          case ASR:
            ccprintf(os, " ASR #%d", (shiftAmt == 0) ? 32 : shiftAmt);
            break;
          case ROR:
            if (shiftAmt == 0) {
                ccprintf(os, " RRX");
            } else {
                ccprintf(os, " ROR #%d", shiftAmt);
            }
            break;
        }
    }
}

string
Swap::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ss << ", [";
    printIntReg(ss, base);
    ss << "]";
    return ss.str();
}

string
RfeOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    stringstream ss;
    switch (mode) {
      case DecrementAfter:
        printMnemonic(ss, "da");
        break;
      case DecrementBefore:
        printMnemonic(ss, "db");
        break;
      case IncrementAfter:
        printMnemonic(ss, "ia");
        break;
      case IncrementBefore:
        printMnemonic(ss, "ib");
        break;
    }
    printIntReg(ss, base);
    if (wb) {
        ss << "!";
    }
    return ss.str();
}

string
SrsOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    stringstream ss;
    switch (mode) {
      case DecrementAfter:
        printMnemonic(ss, "da");
        break;
      case DecrementBefore:
        printMnemonic(ss, "db");
        break;
      case IncrementAfter:
        printMnemonic(ss, "ia");
        break;
      case IncrementBefore:
        printMnemonic(ss, "ib");
        break;
    }
    printIntReg(ss, INTREG_SP);
    if (wb) {
        ss << "!";
    }
    ss << ", #";
    switch (regMode) {
      case MODE_USER:
        ss << "user";
        break;
      case MODE_FIQ:
        ss << "fiq";
        break;
      case MODE_IRQ:
        ss << "irq";
        break;
      case MODE_SVC:
        ss << "supervisor";
        break;
      case MODE_MON:
        ss << "monitor";
        break;
      case MODE_ABORT:
        ss << "abort";
        break;
      case MODE_HYP:
        ss << "hyp";
        break;
      case MODE_UNDEFINED:
        ss << "undefined";
        break;
      case MODE_SYSTEM:
        ss << "system";
        break;
      default:
        ss << "unrecognized";
        break;
    }
    return ss.str();
}

void
Memory::printInst(std::ostream &os, AddrMode addrMode) const
{
    printMnemonic(os);
    printDest(os);
    os << ", [";
    printIntReg(os, base);
    if (addrMode != AddrMd_PostIndex) {
        os << ", ";
        printOffset(os);
        os << "]";
        if (addrMode == AddrMd_PreIndex) {
            os << "!";
        }
    } else {
        os << "] ";
        printOffset(os);

    }
}

}

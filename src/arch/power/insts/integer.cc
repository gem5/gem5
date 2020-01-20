/*
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

#include "arch/power/insts/integer.hh"

using namespace std;
using namespace PowerISA;

string
IntOp::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    stringstream ss;
    bool printDest = true;
    bool printSrcs = true;
    bool printSecondSrc = true;

    // Generate the correct mnemonic
    string myMnemonic(mnemonic);

    // Special cases
    if (!myMnemonic.compare("or") && _srcRegIdx[0] == _srcRegIdx[1]) {
        myMnemonic = "mr";
        printSecondSrc = false;
    } else if (!myMnemonic.compare("mtlr") || !myMnemonic.compare("cmpi")) {
        printDest = false;
    } else if (!myMnemonic.compare("mflr")) {
        printSrcs = false;
    }

    // Additional characters depending on isa bits being set
    if (oeSet) myMnemonic = myMnemonic + "o";
    if (rcSet) myMnemonic = myMnemonic + ".";
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0 && printDest) {
        printReg(ss, _destRegIdx[0]);
    }

    // Print the (possibly) two source registers
    if (_numSrcRegs > 0 && printSrcs) {
        if (_numDestRegs > 0 && printDest) {
            ss << ", ";
        }
        printReg(ss, _srcRegIdx[0]);
        if (_numSrcRegs > 1 && printSecondSrc) {
          ss << ", ";
          printReg(ss, _srcRegIdx[1]);
        }
    }

    return ss.str();
}


string
IntImmOp::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    stringstream ss;

    // Generate the correct mnemonic
    string myMnemonic(mnemonic);

    // Special cases
    if (!myMnemonic.compare("addi") && _numSrcRegs == 0) {
        myMnemonic = "li";
    } else if (!myMnemonic.compare("addis") && _numSrcRegs == 0) {
        myMnemonic = "lis";
    }
    ccprintf(ss, "%-10s ", myMnemonic);

    // Print the first destination only
    if (_numDestRegs > 0) {
        printReg(ss, _destRegIdx[0]);
    }

    // Print the source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0) {
            ss << ", ";
        }
        printReg(ss, _srcRegIdx[0]);
    }

    // Print the immediate value last
    ss << ", " << (int32_t)imm;

    return ss.str();
}


string
IntShiftOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    // Print the first destination only
    if (_numDestRegs > 0) {
        printReg(ss, _destRegIdx[0]);
    }

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0) {
            ss << ", ";
        }
        printReg(ss, _srcRegIdx[0]);
    }

    // Print the shift
    ss << ", " << sh;

    return ss.str();
}


string
IntRotateOp::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    stringstream ss;

    ccprintf(ss, "%-10s ", mnemonic);

    // Print the first destination only
    if (_numDestRegs > 0) {
        printReg(ss, _destRegIdx[0]);
    }

    // Print the first source register
    if (_numSrcRegs > 0) {
        if (_numDestRegs > 0) {
            ss << ", ";
        }
        printReg(ss, _srcRegIdx[0]);
    }

    // Print the shift, mask begin and mask end
    ss << ", " << sh << ", " << mb << ", " << me;

    return ss.str();
}

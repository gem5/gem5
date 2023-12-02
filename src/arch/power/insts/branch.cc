/*
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2021 IBM Corporation
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

#include "arch/power/insts/branch.hh"
#include "arch/power/regs/int.hh"
#include "arch/power/regs/misc.hh"

#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"

namespace gem5
{

using namespace PowerISA;

const std::string &
PCDependentDisassembly::disassemble(
        Addr pc, const loader::SymbolTable *symtab) const
{
    if (!cachedDisassembly || pc != cachedPC || symtab != cachedSymtab) {
        if (!cachedDisassembly)
            cachedDisassembly.reset(new std::string);

        *cachedDisassembly = generateDisassembly(pc, symtab);
        cachedPC = pc;
        cachedSymtab = symtab;
    }

    return *cachedDisassembly;
}


std::unique_ptr<PCStateBase>
BranchOp::branchTarget(ThreadContext *tc) const
{
    Msr msr = tc->getReg(int_reg::Msr);
    Addr addr;

    if (aa)
        addr = li;
    else
        addr = tc->pcState().instAddr() + li;

    return std::make_unique<PowerISA::PCState>(
            msr.sf ? addr : addr & UINT32_MAX);
}


std::string
BranchOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    Addr target;

    // Generate correct mnemonic
    std::string myMnemonic(mnemonic);
    std::string suffix;

    // Additional characters depending on isa bits being set
    if (lk)
        suffix += "l";
    if (aa)
        suffix += "a";
    ccprintf(ss, "%-10s ", myMnemonic + suffix);

    if (aa)
        target = li;
    else
        target = pc + li;

    loader::SymbolTable::const_iterator it;
    if (symtab && (it = symtab->find(target)) != symtab->end())
        ss << it->name();
    else
        ccprintf(ss, "%#x", target);

    return ss.str();
}


std::unique_ptr<PCStateBase>
BranchDispCondOp::branchTarget(ThreadContext *tc) const
{
    Msr msr = tc->getReg(int_reg::Msr);
    Addr addr;

    if (aa)
        addr = bd;
    else
        addr = tc->pcState().instAddr() + bd;

    return std::make_unique<PowerISA::PCState>(
            msr.sf ? addr : addr & UINT32_MAX);
}


std::string
BranchDispCondOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    Addr target;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);
    std::string suffix;

    // Additional characters depending on isa bits being set
    if (lk)
        suffix += "l";
    if (aa)
        suffix += "a";
    ccprintf(ss, "%-10s ", myMnemonic + suffix);

    // Print BI and BO fields
    ss << (int) bi << ", " << (int) bo << ", ";

    if (aa)
        target = bd;
    else
        target = pc + bd;

    loader::SymbolTable::const_iterator it;
    if (symtab && (it = symtab->find(target)) != symtab->end())
        ss << it->name();
    else
        ccprintf(ss, "%#x", target);

    return ss.str();
}


std::unique_ptr<PCStateBase>
BranchRegCondOp::branchTarget(ThreadContext *tc) const
{
    Msr msr = tc->getReg(int_reg::Msr);
    Addr addr = tc->getReg(srcRegIdx(_numSrcRegs - 1)) & -4ULL;
    return std::make_unique<PowerISA::PCState>(
            msr.sf ? addr : addr & UINT32_MAX);
}


std::string
BranchRegCondOp::generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;

    // Generate the correct mnemonic
    std::string myMnemonic(mnemonic);
    std::string suffix;

    // Additional characters depending on isa bits being set
    if (lk)
        suffix += "l";
    ccprintf(ss, "%-10s ", myMnemonic + suffix);

    // Print the BI and BO fields
    ss << (int) bi << ", " << (int) bo;

    return ss.str();
}

} // namespace gem5

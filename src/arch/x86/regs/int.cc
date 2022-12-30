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
 */

#include "arch/x86/regs/int.hh"

#include <sstream>

namespace gem5
{

namespace X86ISA
{

std::string
FlatIntRegClassOps::regName(const RegId &id) const
{
    constexpr const char *abcdFormats[9] =
        {"", "%s",  "%sx",  "", "e%sx", "", "", "", "r%sx"};
    constexpr const char *piFormats[9] =
        {"", "%s",  "%s",   "", "e%s",  "", "", "", "r%s"};
    constexpr const char *longFormats[9] =
        {"", "r%sb", "r%sw", "", "r%sd", "", "", "", "r%s"};
    constexpr const char *microFormats[9] =
        {"", "t%db", "t%dw", "", "t%dd", "", "", "", "t%d"};

    // Fix size at 8 for now.
    constexpr unsigned size = 8;

    RegIndex reg_idx = id.index();

    std::ostringstream ss;

    const char * suffix = "";
    bool fold = reg_idx & IntFoldBit;
    reg_idx &= ~IntFoldBit;

    if (fold)
        suffix = "h";
    else if (reg_idx < 8 && size == 1)
        suffix = "l";

    switch (reg_idx) {
      case int_reg::Rax:
        ccprintf(ss, abcdFormats[size], "a");
        break;
      case int_reg::Rbx:
        ccprintf(ss, abcdFormats[size], "b");
        break;
      case int_reg::Rcx:
        ccprintf(ss, abcdFormats[size], "c");
        break;
      case int_reg::Rdx:
        ccprintf(ss, abcdFormats[size], "d");
        break;
      case int_reg::Rsp:
        ccprintf(ss, piFormats[size], "sp");
        break;
      case int_reg::Rbp:
        ccprintf(ss, piFormats[size], "bp");
        break;
      case int_reg::Rsi:
        ccprintf(ss, piFormats[size], "si");
        break;
      case int_reg::Rdi:
        ccprintf(ss, piFormats[size], "di");
        break;
      case int_reg::R8:
        ccprintf(ss, longFormats[size], "8");
        break;
      case int_reg::R9:
        ccprintf(ss, longFormats[size], "9");
        break;
      case int_reg::R10:
        ccprintf(ss, longFormats[size], "10");
        break;
      case int_reg::R11:
        ccprintf(ss, longFormats[size], "11");
        break;
      case int_reg::R12:
        ccprintf(ss, longFormats[size], "12");
        break;
      case int_reg::R13:
        ccprintf(ss, longFormats[size], "13");
        break;
      case int_reg::R14:
        ccprintf(ss, longFormats[size], "14");
        break;
      case int_reg::R15:
        ccprintf(ss, longFormats[size], "15");
        break;
      default:
        ccprintf(ss, microFormats[size],
                reg_idx - int_reg::MicroBegin);
    }
    ccprintf(ss, suffix);

    return ss.str();
}

RegId
IntRegClassOps::flatten(const BaseISA &isa, const RegId &id) const
{
    return {flatIntRegClass, (RegIndex)(id.index() & ~IntFoldBit)};
}

} // namespace X86ISA
} // namespace gem5

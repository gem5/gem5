/*
 * Copyright (c) 2010-2013, 2015-2024 Arm Limited
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

#include "arch/arm/regs/misc.hh"

#include <tuple>

#include "arch/arm/insts/misc64.hh"
#include "arch/arm/isa.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "dev/arm/gic_v3_cpu_interface.hh"
#include "params/ArmISA.hh"
#include "sim/full_system.hh"

namespace gem5
{

namespace ArmISA
{

namespace
{

std::unordered_map<MiscRegNum32, MiscRegIndex> miscRegNum32ToIdx{
    // MCR/MRC regs
    { MiscRegNum32(14, 0, 0, 0, 0), MISCREG_DBGDIDR },
    { MiscRegNum32(14, 0, 0, 0, 2), MISCREG_DBGDTRRXext },
    { MiscRegNum32(14, 0, 0, 0, 4), MISCREG_DBGBVR0 },
    { MiscRegNum32(14, 0, 0, 0, 5), MISCREG_DBGBCR0 },
    { MiscRegNum32(14, 0, 0, 0, 6), MISCREG_DBGWVR0 },
    { MiscRegNum32(14, 0, 0, 0, 7), MISCREG_DBGWCR0 },
    { MiscRegNum32(14, 0, 0, 1, 0), MISCREG_DBGDSCRint },
    { MiscRegNum32(14, 0, 0, 1, 4), MISCREG_DBGBVR1 },
    { MiscRegNum32(14, 0, 0, 1, 5), MISCREG_DBGBCR1 },
    { MiscRegNum32(14, 0, 0, 1, 6), MISCREG_DBGWVR1 },
    { MiscRegNum32(14, 0, 0, 1, 7), MISCREG_DBGWCR1 },
    { MiscRegNum32(14, 0, 0, 2, 2), MISCREG_DBGDSCRext },
    { MiscRegNum32(14, 0, 0, 2, 4), MISCREG_DBGBVR2 },
    { MiscRegNum32(14, 0, 0, 2, 5), MISCREG_DBGBCR2 },
    { MiscRegNum32(14, 0, 0, 2, 6), MISCREG_DBGWVR2 },
    { MiscRegNum32(14, 0, 0, 2, 7), MISCREG_DBGWCR2 },
    { MiscRegNum32(14, 0, 0, 3, 2), MISCREG_DBGDTRTXext },
    { MiscRegNum32(14, 0, 0, 3, 4), MISCREG_DBGBVR3 },
    { MiscRegNum32(14, 0, 0, 3, 5), MISCREG_DBGBCR3 },
    { MiscRegNum32(14, 0, 0, 3, 6), MISCREG_DBGWVR3 },
    { MiscRegNum32(14, 0, 0, 3, 7), MISCREG_DBGWCR3 },
    { MiscRegNum32(14, 0, 0, 4, 4), MISCREG_DBGBVR4 },
    { MiscRegNum32(14, 0, 0, 4, 5), MISCREG_DBGBCR4 },
    { MiscRegNum32(14, 0, 0, 4, 6), MISCREG_DBGWVR4 },
    { MiscRegNum32(14, 0, 0, 4, 7), MISCREG_DBGWCR4 },
    { MiscRegNum32(14, 0, 0, 5, 4), MISCREG_DBGBVR5 },
    { MiscRegNum32(14, 0, 0, 5, 5), MISCREG_DBGBCR5 },
    { MiscRegNum32(14, 0, 0, 5, 6), MISCREG_DBGWVR5 },
    { MiscRegNum32(14, 0, 0, 5, 7), MISCREG_DBGWCR5 },
    { MiscRegNum32(14, 0, 0, 6, 2), MISCREG_DBGOSECCR },
    { MiscRegNum32(14, 0, 0, 6, 4), MISCREG_DBGBVR6 },
    { MiscRegNum32(14, 0, 0, 6, 5), MISCREG_DBGBCR6 },
    { MiscRegNum32(14, 0, 0, 6, 6), MISCREG_DBGWVR6 },
    { MiscRegNum32(14, 0, 0, 6, 7), MISCREG_DBGWCR6 },
    { MiscRegNum32(14, 0, 0, 7, 0), MISCREG_DBGVCR },
    { MiscRegNum32(14, 0, 0, 7, 4), MISCREG_DBGBVR7 },
    { MiscRegNum32(14, 0, 0, 7, 5), MISCREG_DBGBCR7 },
    { MiscRegNum32(14, 0, 0, 7, 6), MISCREG_DBGWVR7 },
    { MiscRegNum32(14, 0, 0, 7, 7), MISCREG_DBGWCR7 },
    { MiscRegNum32(14, 0, 0, 8, 4), MISCREG_DBGBVR8 },
    { MiscRegNum32(14, 0, 0, 8, 5), MISCREG_DBGBCR8 },
    { MiscRegNum32(14, 0, 0, 8, 6), MISCREG_DBGWVR8 },
    { MiscRegNum32(14, 0, 0, 8, 7), MISCREG_DBGWCR8 },
    { MiscRegNum32(14, 0, 0, 9, 4), MISCREG_DBGBVR9 },
    { MiscRegNum32(14, 0, 0, 9, 5), MISCREG_DBGBCR9 },
    { MiscRegNum32(14, 0, 0, 9, 6), MISCREG_DBGWVR9 },
    { MiscRegNum32(14, 0, 0, 9, 7), MISCREG_DBGWCR9 },
    { MiscRegNum32(14, 0, 0, 10, 4), MISCREG_DBGBVR10 },
    { MiscRegNum32(14, 0, 0, 10, 5), MISCREG_DBGBCR10 },
    { MiscRegNum32(14, 0, 0, 10, 6), MISCREG_DBGWVR10 },
    { MiscRegNum32(14, 0, 0, 10, 7), MISCREG_DBGWCR10 },
    { MiscRegNum32(14, 0, 0, 11, 4), MISCREG_DBGBVR11 },
    { MiscRegNum32(14, 0, 0, 11, 5), MISCREG_DBGBCR11 },
    { MiscRegNum32(14, 0, 0, 11, 6), MISCREG_DBGWVR11 },
    { MiscRegNum32(14, 0, 0, 11, 7), MISCREG_DBGWCR11 },
    { MiscRegNum32(14, 0, 0, 12, 4), MISCREG_DBGBVR12 },
    { MiscRegNum32(14, 0, 0, 12, 5), MISCREG_DBGBCR12 },
    { MiscRegNum32(14, 0, 0, 12, 6), MISCREG_DBGWVR12 },
    { MiscRegNum32(14, 0, 0, 12, 7), MISCREG_DBGWCR12 },
    { MiscRegNum32(14, 0, 0, 13, 4), MISCREG_DBGBVR13 },
    { MiscRegNum32(14, 0, 0, 13, 5), MISCREG_DBGBCR13 },
    { MiscRegNum32(14, 0, 0, 13, 6), MISCREG_DBGWVR13 },
    { MiscRegNum32(14, 0, 0, 13, 7), MISCREG_DBGWCR13 },
    { MiscRegNum32(14, 0, 0, 14, 4), MISCREG_DBGBVR14 },
    { MiscRegNum32(14, 0, 0, 14, 5), MISCREG_DBGBCR14 },
    { MiscRegNum32(14, 0, 0, 14, 6), MISCREG_DBGWVR14 },
    { MiscRegNum32(14, 0, 0, 14, 7), MISCREG_DBGWCR14 },
    { MiscRegNum32(14, 0, 0, 15, 4), MISCREG_DBGBVR15 },
    { MiscRegNum32(14, 0, 0, 15, 5), MISCREG_DBGBCR15 },
    { MiscRegNum32(14, 0, 0, 15, 6), MISCREG_DBGWVR15 },
    { MiscRegNum32(14, 0, 0, 15, 7), MISCREG_DBGWCR15 },
    { MiscRegNum32(14, 0, 1, 0, 1), MISCREG_DBGBXVR0 },
    { MiscRegNum32(14, 0, 1, 0, 4), MISCREG_DBGOSLAR },
    { MiscRegNum32(14, 0, 1, 1, 1), MISCREG_DBGBXVR1 },
    { MiscRegNum32(14, 0, 1, 1, 4), MISCREG_DBGOSLSR },
    { MiscRegNum32(14, 0, 1, 2, 1), MISCREG_DBGBXVR2 },
    { MiscRegNum32(14, 0, 1, 3, 1), MISCREG_DBGBXVR3 },
    { MiscRegNum32(14, 0, 1, 3, 4), MISCREG_DBGOSDLR },
    { MiscRegNum32(14, 0, 1, 4, 1), MISCREG_DBGBXVR4 },
    { MiscRegNum32(14, 0, 1, 4, 4), MISCREG_DBGPRCR },
    { MiscRegNum32(14, 0, 1, 5, 1), MISCREG_DBGBXVR5 },
    { MiscRegNum32(14, 0, 1, 6, 1), MISCREG_DBGBXVR6 },
    { MiscRegNum32(14, 0, 1, 7, 1), MISCREG_DBGBXVR7 },
    { MiscRegNum32(14, 0, 1, 8, 1), MISCREG_DBGBXVR8 },
    { MiscRegNum32(14, 0, 1, 9, 1), MISCREG_DBGBXVR9 },
    { MiscRegNum32(14, 0, 1, 10, 1), MISCREG_DBGBXVR10 },
    { MiscRegNum32(14, 0, 1, 11, 1), MISCREG_DBGBXVR11 },
    { MiscRegNum32(14, 0, 1, 12, 1), MISCREG_DBGBXVR12 },
    { MiscRegNum32(14, 0, 1, 13, 1), MISCREG_DBGBXVR13 },
    { MiscRegNum32(14, 0, 1, 14, 1), MISCREG_DBGBXVR14 },
    { MiscRegNum32(14, 0, 1, 15, 1), MISCREG_DBGBXVR15 },
    { MiscRegNum32(14, 6, 1, 0, 0), MISCREG_TEEHBR },
    { MiscRegNum32(14, 7, 0, 0, 0), MISCREG_JIDR },
    { MiscRegNum32(14, 7, 1, 0, 0), MISCREG_JOSCR },
    { MiscRegNum32(14, 7, 2, 0, 0), MISCREG_JMCR },
    { MiscRegNum32(15, 0, 0, 0, 0), MISCREG_MIDR },
    { MiscRegNum32(15, 0, 0, 0, 1), MISCREG_CTR },
    { MiscRegNum32(15, 0, 0, 0, 2), MISCREG_TCMTR },
    { MiscRegNum32(15, 0, 0, 0, 3), MISCREG_TLBTR },
    { MiscRegNum32(15, 0, 0, 0, 4), MISCREG_MIDR },
    { MiscRegNum32(15, 0, 0, 0, 5), MISCREG_MPIDR },
    { MiscRegNum32(15, 0, 0, 0, 6), MISCREG_REVIDR },
    { MiscRegNum32(15, 0, 0, 0, 7), MISCREG_MIDR },
    { MiscRegNum32(15, 0, 0, 1, 0), MISCREG_ID_PFR0 },
    { MiscRegNum32(15, 0, 0, 1, 1), MISCREG_ID_PFR1 },
    { MiscRegNum32(15, 0, 0, 1, 2), MISCREG_ID_DFR0 },
    { MiscRegNum32(15, 0, 0, 1, 3), MISCREG_ID_AFR0 },
    { MiscRegNum32(15, 0, 0, 1, 4), MISCREG_ID_MMFR0 },
    { MiscRegNum32(15, 0, 0, 1, 5), MISCREG_ID_MMFR1 },
    { MiscRegNum32(15, 0, 0, 1, 6), MISCREG_ID_MMFR2 },
    { MiscRegNum32(15, 0, 0, 1, 7), MISCREG_ID_MMFR3 },
    { MiscRegNum32(15, 0, 0, 2, 0), MISCREG_ID_ISAR0 },
    { MiscRegNum32(15, 0, 0, 2, 1), MISCREG_ID_ISAR1 },
    { MiscRegNum32(15, 0, 0, 2, 2), MISCREG_ID_ISAR2 },
    { MiscRegNum32(15, 0, 0, 2, 3), MISCREG_ID_ISAR3 },
    { MiscRegNum32(15, 0, 0, 2, 4), MISCREG_ID_ISAR4 },
    { MiscRegNum32(15, 0, 0, 2, 5), MISCREG_ID_ISAR5 },
    { MiscRegNum32(15, 0, 0, 2, 6), MISCREG_ID_MMFR4 },
    { MiscRegNum32(15, 0, 0, 2, 7), MISCREG_ID_ISAR6 },
    { MiscRegNum32(15, 0, 0, 3, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 3, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 3, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 3, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 3, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 3, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 3, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 3, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 4, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 4, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 4, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 4, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 4, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 4, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 4, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 4, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 5, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 5, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 5, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 5, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 5, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 5, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 5, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 5, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 6, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 6, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 6, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 6, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 6, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 6, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 6, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 6, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 7, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 7, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 7, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 7, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 7, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 7, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 7, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 7, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 8, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 8, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 8, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 8, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 8, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 8, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 8, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 8, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 9, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 9, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 9, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 9, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 9, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 9, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 9, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 9, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 10, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 10, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 10, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 10, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 10, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 10, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 10, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 10, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 11, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 11, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 11, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 11, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 11, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 11, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 11, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 11, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 12, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 12, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 12, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 12, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 12, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 12, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 12, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 12, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 13, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 13, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 13, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 13, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 13, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 13, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 13, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 13, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 14, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 14, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 14, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 14, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 14, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 14, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 14, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 14, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 15, 0), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 15, 1), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 15, 2), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 15, 3), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 15, 4), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 15, 5), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 15, 6), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 0, 15, 7), MISCREG_RAZ },
    { MiscRegNum32(15, 0, 1, 0, 0), MISCREG_SCTLR },
    { MiscRegNum32(15, 0, 1, 0, 1), MISCREG_ACTLR },
    { MiscRegNum32(15, 0, 1, 0, 2), MISCREG_CPACR },
    { MiscRegNum32(15, 0, 1, 1, 0), MISCREG_SCR },
    { MiscRegNum32(15, 0, 1, 1, 1), MISCREG_SDER },
    { MiscRegNum32(15, 0, 1, 1, 2), MISCREG_NSACR },
    { MiscRegNum32(15, 0, 1, 3, 1), MISCREG_SDCR },
    { MiscRegNum32(15, 0, 2, 0, 0), MISCREG_TTBR0 },
    { MiscRegNum32(15, 0, 2, 0, 1), MISCREG_TTBR1 },
    { MiscRegNum32(15, 0, 2, 0, 2), MISCREG_TTBCR },
    { MiscRegNum32(15, 0, 3, 0, 0), MISCREG_DACR },
    { MiscRegNum32(15, 0, 4, 6, 0), MISCREG_ICC_PMR },
    { MiscRegNum32(15, 0, 5, 0, 0), MISCREG_DFSR },
    { MiscRegNum32(15, 0, 5, 0, 1), MISCREG_IFSR },
    { MiscRegNum32(15, 0, 5, 1, 0), MISCREG_ADFSR },
    { MiscRegNum32(15, 0, 5, 1, 1), MISCREG_AIFSR },
    { MiscRegNum32(15, 0, 6, 0, 0), MISCREG_DFAR },
    { MiscRegNum32(15, 0, 6, 0, 2), MISCREG_IFAR },
    { MiscRegNum32(15, 0, 7, 0, 4), MISCREG_NOP },
    { MiscRegNum32(15, 0, 7, 1, 0), MISCREG_ICIALLUIS },
    { MiscRegNum32(15, 0, 7, 1, 6), MISCREG_BPIALLIS },
    { MiscRegNum32(15, 0, 7, 2, 7), MISCREG_DBGDEVID0 },
    { MiscRegNum32(15, 0, 7, 4, 0), MISCREG_PAR },
    { MiscRegNum32(15, 0, 7, 5, 0), MISCREG_ICIALLU },
    { MiscRegNum32(15, 0, 7, 5, 1), MISCREG_ICIMVAU },
    { MiscRegNum32(15, 0, 7, 5, 4), MISCREG_CP15ISB },
    { MiscRegNum32(15, 0, 7, 5, 6), MISCREG_BPIALL },
    { MiscRegNum32(15, 0, 7, 5, 7), MISCREG_BPIMVA },
    { MiscRegNum32(15, 0, 7, 6, 1), MISCREG_DCIMVAC },
    { MiscRegNum32(15, 0, 7, 6, 2), MISCREG_DCISW },
    { MiscRegNum32(15, 0, 7, 8, 0), MISCREG_ATS1CPR },
    { MiscRegNum32(15, 0, 7, 8, 1), MISCREG_ATS1CPW },
    { MiscRegNum32(15, 0, 7, 8, 2), MISCREG_ATS1CUR },
    { MiscRegNum32(15, 0, 7, 8, 3), MISCREG_ATS1CUW },
    { MiscRegNum32(15, 0, 7, 8, 4), MISCREG_ATS12NSOPR },
    { MiscRegNum32(15, 0, 7, 8, 5), MISCREG_ATS12NSOPW },
    { MiscRegNum32(15, 0, 7, 8, 6), MISCREG_ATS12NSOUR },
    { MiscRegNum32(15, 0, 7, 8, 7), MISCREG_ATS12NSOUW },
    { MiscRegNum32(15, 0, 7, 10, 1), MISCREG_DCCMVAC },
    { MiscRegNum32(15, 0, 7, 10, 2), MISCREG_DCCSW },
    { MiscRegNum32(15, 0, 7, 10, 4), MISCREG_CP15DSB },
    { MiscRegNum32(15, 0, 7, 10, 5), MISCREG_CP15DMB },
    { MiscRegNum32(15, 0, 7, 11, 1), MISCREG_DCCMVAU },
    { MiscRegNum32(15, 0, 7, 13, 1), MISCREG_NOP },
    { MiscRegNum32(15, 0, 7, 14, 1), MISCREG_DCCIMVAC },
    { MiscRegNum32(15, 0, 7, 14, 2), MISCREG_DCCISW },
    { MiscRegNum32(15, 0, 8, 3, 0), MISCREG_TLBIALLIS },
    { MiscRegNum32(15, 0, 8, 3, 1), MISCREG_TLBIMVAIS },
    { MiscRegNum32(15, 0, 8, 3, 2), MISCREG_TLBIASIDIS },
    { MiscRegNum32(15, 0, 8, 3, 3), MISCREG_TLBIMVAAIS },
    { MiscRegNum32(15, 0, 8, 3, 5), MISCREG_TLBIMVALIS },
    { MiscRegNum32(15, 0, 8, 3, 7), MISCREG_TLBIMVAALIS },
    { MiscRegNum32(15, 0, 8, 5, 0), MISCREG_ITLBIALL },
    { MiscRegNum32(15, 0, 8, 5, 1), MISCREG_ITLBIMVA },
    { MiscRegNum32(15, 0, 8, 5, 2), MISCREG_ITLBIASID },
    { MiscRegNum32(15, 0, 8, 6, 0), MISCREG_DTLBIALL },
    { MiscRegNum32(15, 0, 8, 6, 1), MISCREG_DTLBIMVA },
    { MiscRegNum32(15, 0, 8, 6, 2), MISCREG_DTLBIASID },
    { MiscRegNum32(15, 0, 8, 7, 0), MISCREG_TLBIALL },
    { MiscRegNum32(15, 0, 8, 7, 1), MISCREG_TLBIMVA },
    { MiscRegNum32(15, 0, 8, 7, 2), MISCREG_TLBIASID },
    { MiscRegNum32(15, 0, 8, 7, 3), MISCREG_TLBIMVAA },
    { MiscRegNum32(15, 0, 8, 7, 5), MISCREG_TLBIMVAL },
    { MiscRegNum32(15, 0, 8, 7, 7), MISCREG_TLBIMVAAL },
    { MiscRegNum32(15, 0, 9, 12, 0), MISCREG_PMCR },
    { MiscRegNum32(15, 0, 9, 12, 1), MISCREG_PMCNTENSET },
    { MiscRegNum32(15, 0, 9, 12, 2), MISCREG_PMCNTENCLR },
    { MiscRegNum32(15, 0, 9, 12, 3), MISCREG_PMOVSR },
    { MiscRegNum32(15, 0, 9, 12, 4), MISCREG_PMSWINC },
    { MiscRegNum32(15, 0, 9, 12, 5), MISCREG_PMSELR },
    { MiscRegNum32(15, 0, 9, 12, 6), MISCREG_PMCEID0 },
    { MiscRegNum32(15, 0, 9, 12, 7), MISCREG_PMCEID1 },
    { MiscRegNum32(15, 0, 9, 13, 0), MISCREG_PMCCNTR },
    { MiscRegNum32(15, 0, 9, 13, 1), MISCREG_PMXEVTYPER_PMCCFILTR },
    { MiscRegNum32(15, 0, 9, 13, 2), MISCREG_PMXEVCNTR },
    { MiscRegNum32(15, 0, 9, 14, 0), MISCREG_PMUSERENR },
    { MiscRegNum32(15, 0, 9, 14, 1), MISCREG_PMINTENSET },
    { MiscRegNum32(15, 0, 9, 14, 2), MISCREG_PMINTENCLR },
    { MiscRegNum32(15, 0, 9, 14, 3), MISCREG_PMOVSSET },
    { MiscRegNum32(15, 0, 10, 2, 0), MISCREG_PRRR_MAIR0 },
    { MiscRegNum32(15, 0, 10, 2, 1), MISCREG_NMRR_MAIR1 },
    { MiscRegNum32(15, 0, 10, 3, 0), MISCREG_AMAIR0 },
    { MiscRegNum32(15, 0, 10, 3, 1), MISCREG_AMAIR1 },
    { MiscRegNum32(15, 0, 12, 0, 0), MISCREG_VBAR },
    { MiscRegNum32(15, 0, 12, 0, 1), MISCREG_MVBAR },
    { MiscRegNum32(15, 0, 12, 1, 0), MISCREG_ISR },
    { MiscRegNum32(15, 0, 12, 8, 0), MISCREG_ICC_IAR0 },
    { MiscRegNum32(15, 0, 12, 8, 1), MISCREG_ICC_EOIR0 },
    { MiscRegNum32(15, 0, 12, 8, 2), MISCREG_ICC_HPPIR0 },
    { MiscRegNum32(15, 0, 12, 8, 3), MISCREG_ICC_BPR0 },
    { MiscRegNum32(15, 0, 12, 8, 4), MISCREG_ICC_AP0R0 },
    { MiscRegNum32(15, 0, 12, 8, 5), MISCREG_ICC_AP0R1 },
    { MiscRegNum32(15, 0, 12, 8, 6), MISCREG_ICC_AP0R2 },
    { MiscRegNum32(15, 0, 12, 8, 7), MISCREG_ICC_AP0R3 },
    { MiscRegNum32(15, 0, 12, 9, 0), MISCREG_ICC_AP1R0 },
    { MiscRegNum32(15, 0, 12, 9, 1), MISCREG_ICC_AP1R1 },
    { MiscRegNum32(15, 0, 12, 9, 2), MISCREG_ICC_AP1R2 },
    { MiscRegNum32(15, 0, 12, 9, 3), MISCREG_ICC_AP1R3 },
    { MiscRegNum32(15, 0, 12, 11, 1), MISCREG_ICC_DIR },
    { MiscRegNum32(15, 0, 12, 11, 3), MISCREG_ICC_RPR },
    { MiscRegNum32(15, 0, 12, 12, 0), MISCREG_ICC_IAR1 },
    { MiscRegNum32(15, 0, 12, 12, 1), MISCREG_ICC_EOIR1 },
    { MiscRegNum32(15, 0, 12, 12, 2), MISCREG_ICC_HPPIR1 },
    { MiscRegNum32(15, 0, 12, 12, 3), MISCREG_ICC_BPR1 },
    { MiscRegNum32(15, 0, 12, 12, 4), MISCREG_ICC_CTLR },
    { MiscRegNum32(15, 0, 12, 12, 5), MISCREG_ICC_SRE },
    { MiscRegNum32(15, 0, 12, 12, 6), MISCREG_ICC_IGRPEN0 },
    { MiscRegNum32(15, 0, 12, 12, 7), MISCREG_ICC_IGRPEN1 },
    { MiscRegNum32(15, 0, 13, 0, 0), MISCREG_FCSEIDR },
    { MiscRegNum32(15, 0, 13, 0, 1), MISCREG_CONTEXTIDR },
    { MiscRegNum32(15, 0, 13, 0, 2), MISCREG_TPIDRURW },
    { MiscRegNum32(15, 0, 13, 0, 3), MISCREG_TPIDRURO },
    { MiscRegNum32(15, 0, 13, 0, 4), MISCREG_TPIDRPRW },
    { MiscRegNum32(15, 0, 14, 0, 0), MISCREG_CNTFRQ },
    { MiscRegNum32(15, 0, 14, 1, 0), MISCREG_CNTKCTL },
    { MiscRegNum32(15, 0, 14, 2, 0), MISCREG_CNTP_TVAL },
    { MiscRegNum32(15, 0, 14, 2, 1), MISCREG_CNTP_CTL },
    { MiscRegNum32(15, 0, 14, 3, 0), MISCREG_CNTV_TVAL },
    { MiscRegNum32(15, 0, 14, 3, 1), MISCREG_CNTV_CTL },
    { MiscRegNum32(15, 1, 0, 0, 0), MISCREG_CCSIDR },
    { MiscRegNum32(15, 1, 0, 0, 1), MISCREG_CLIDR },
    { MiscRegNum32(15, 1, 0, 0, 7), MISCREG_AIDR },
    { MiscRegNum32(15, 2, 0, 0, 0), MISCREG_CSSELR },
    { MiscRegNum32(15, 4, 0, 0, 0), MISCREG_VPIDR },
    { MiscRegNum32(15, 4, 0, 0, 5), MISCREG_VMPIDR },
    { MiscRegNum32(15, 4, 1, 0, 0), MISCREG_HSCTLR },
    { MiscRegNum32(15, 4, 1, 0, 1), MISCREG_HACTLR },
    { MiscRegNum32(15, 4, 1, 1, 0), MISCREG_HCR },
    { MiscRegNum32(15, 4, 1, 1, 1), MISCREG_HDCR },
    { MiscRegNum32(15, 4, 1, 1, 2), MISCREG_HCPTR },
    { MiscRegNum32(15, 4, 1, 1, 3), MISCREG_HSTR },
    { MiscRegNum32(15, 4, 1, 1, 4), MISCREG_HCR2 },
    { MiscRegNum32(15, 4, 1, 1, 7), MISCREG_HACR },
    { MiscRegNum32(15, 4, 2, 0, 2), MISCREG_HTCR },
    { MiscRegNum32(15, 4, 2, 1, 2), MISCREG_VTCR },
    { MiscRegNum32(15, 4, 5, 1, 0), MISCREG_HADFSR },
    { MiscRegNum32(15, 4, 5, 1, 1), MISCREG_HAIFSR },
    { MiscRegNum32(15, 4, 5, 2, 0), MISCREG_HSR },
    { MiscRegNum32(15, 4, 6, 0, 0), MISCREG_HDFAR },
    { MiscRegNum32(15, 4, 6, 0, 2), MISCREG_HIFAR },
    { MiscRegNum32(15, 4, 6, 0, 4), MISCREG_HPFAR },
    { MiscRegNum32(15, 4, 7, 8, 0), MISCREG_ATS1HR },
    { MiscRegNum32(15, 4, 7, 8, 1), MISCREG_ATS1HW },
    { MiscRegNum32(15, 4, 8, 0, 1), MISCREG_TLBIIPAS2IS },
    { MiscRegNum32(15, 4, 8, 0, 5), MISCREG_TLBIIPAS2LIS },
    { MiscRegNum32(15, 4, 8, 3, 0), MISCREG_TLBIALLHIS },
    { MiscRegNum32(15, 4, 8, 3, 1), MISCREG_TLBIMVAHIS },
    { MiscRegNum32(15, 4, 8, 3, 4), MISCREG_TLBIALLNSNHIS },
    { MiscRegNum32(15, 4, 8, 3, 5), MISCREG_TLBIMVALHIS },
    { MiscRegNum32(15, 4, 8, 4, 1), MISCREG_TLBIIPAS2 },
    { MiscRegNum32(15, 4, 8, 4, 5), MISCREG_TLBIIPAS2L },
    { MiscRegNum32(15, 4, 8, 7, 0), MISCREG_TLBIALLH },
    { MiscRegNum32(15, 4, 8, 7, 1), MISCREG_TLBIMVAH },
    { MiscRegNum32(15, 4, 8, 7, 4), MISCREG_TLBIALLNSNH },
    { MiscRegNum32(15, 4, 8, 7, 5), MISCREG_TLBIMVALH },
    { MiscRegNum32(15, 4, 10, 2, 0), MISCREG_HMAIR0 },
    { MiscRegNum32(15, 4, 10, 2, 1), MISCREG_HMAIR1 },
    { MiscRegNum32(15, 4, 10, 3, 0), MISCREG_HAMAIR0 },
    { MiscRegNum32(15, 4, 10, 3, 1), MISCREG_HAMAIR1 },
    { MiscRegNum32(15, 4, 12, 0, 0), MISCREG_HVBAR },
    { MiscRegNum32(15, 4, 12, 8, 0), MISCREG_ICH_AP0R0 },
    { MiscRegNum32(15, 4, 12, 8, 1), MISCREG_ICH_AP0R1 },
    { MiscRegNum32(15, 4, 12, 8, 2), MISCREG_ICH_AP0R2 },
    { MiscRegNum32(15, 4, 12, 8, 3), MISCREG_ICH_AP0R3 },
    { MiscRegNum32(15, 4, 12, 9, 0), MISCREG_ICH_AP1R0 },
    { MiscRegNum32(15, 4, 12, 9, 1), MISCREG_ICH_AP1R1 },
    { MiscRegNum32(15, 4, 12, 9, 2), MISCREG_ICH_AP1R2 },
    { MiscRegNum32(15, 4, 12, 9, 3), MISCREG_ICH_AP1R3 },
    { MiscRegNum32(15, 4, 12, 9, 5), MISCREG_ICC_HSRE },
    { MiscRegNum32(15, 4, 12, 11, 0), MISCREG_ICH_HCR },
    { MiscRegNum32(15, 4, 12, 11, 1), MISCREG_ICH_VTR },
    { MiscRegNum32(15, 4, 12, 11, 2), MISCREG_ICH_MISR },
    { MiscRegNum32(15, 4, 12, 11, 3), MISCREG_ICH_EISR },
    { MiscRegNum32(15, 4, 12, 11, 5), MISCREG_ICH_ELRSR },
    { MiscRegNum32(15, 4, 12, 11, 7), MISCREG_ICH_VMCR },
    { MiscRegNum32(15, 4, 12, 12, 0), MISCREG_ICH_LR0 },
    { MiscRegNum32(15, 4, 12, 12, 1), MISCREG_ICH_LR1 },
    { MiscRegNum32(15, 4, 12, 12, 2), MISCREG_ICH_LR2 },
    { MiscRegNum32(15, 4, 12, 12, 3), MISCREG_ICH_LR3 },
    { MiscRegNum32(15, 4, 12, 12, 4), MISCREG_ICH_LR4 },
    { MiscRegNum32(15, 4, 12, 12, 5), MISCREG_ICH_LR5 },
    { MiscRegNum32(15, 4, 12, 12, 6), MISCREG_ICH_LR6 },
    { MiscRegNum32(15, 4, 12, 12, 7), MISCREG_ICH_LR7 },
    { MiscRegNum32(15, 4, 12, 13, 0), MISCREG_ICH_LR8 },
    { MiscRegNum32(15, 4, 12, 13, 1), MISCREG_ICH_LR9 },
    { MiscRegNum32(15, 4, 12, 13, 2), MISCREG_ICH_LR10 },
    { MiscRegNum32(15, 4, 12, 13, 3), MISCREG_ICH_LR11 },
    { MiscRegNum32(15, 4, 12, 13, 4), MISCREG_ICH_LR12 },
    { MiscRegNum32(15, 4, 12, 13, 5), MISCREG_ICH_LR13 },
    { MiscRegNum32(15, 4, 12, 13, 6), MISCREG_ICH_LR14 },
    { MiscRegNum32(15, 4, 12, 13, 7), MISCREG_ICH_LR15 },
    { MiscRegNum32(15, 4, 12, 14, 0), MISCREG_ICH_LRC0 },
    { MiscRegNum32(15, 4, 12, 14, 1), MISCREG_ICH_LRC1 },
    { MiscRegNum32(15, 4, 12, 14, 2), MISCREG_ICH_LRC2 },
    { MiscRegNum32(15, 4, 12, 14, 3), MISCREG_ICH_LRC3 },
    { MiscRegNum32(15, 4, 12, 14, 4), MISCREG_ICH_LRC4 },
    { MiscRegNum32(15, 4, 12, 14, 5), MISCREG_ICH_LRC5 },
    { MiscRegNum32(15, 4, 12, 14, 6), MISCREG_ICH_LRC6 },
    { MiscRegNum32(15, 4, 12, 14, 7), MISCREG_ICH_LRC7 },
    { MiscRegNum32(15, 4, 12, 15, 0), MISCREG_ICH_LRC8 },
    { MiscRegNum32(15, 4, 12, 15, 1), MISCREG_ICH_LRC9 },
    { MiscRegNum32(15, 4, 12, 15, 2), MISCREG_ICH_LRC10 },
    { MiscRegNum32(15, 4, 12, 15, 3), MISCREG_ICH_LRC11 },
    { MiscRegNum32(15, 4, 12, 15, 4), MISCREG_ICH_LRC12 },
    { MiscRegNum32(15, 4, 12, 15, 5), MISCREG_ICH_LRC13 },
    { MiscRegNum32(15, 4, 12, 15, 6), MISCREG_ICH_LRC14 },
    { MiscRegNum32(15, 4, 12, 15, 7), MISCREG_ICH_LRC15 },
    { MiscRegNum32(15, 4, 13, 0, 2), MISCREG_HTPIDR },
    { MiscRegNum32(15, 4, 14, 1, 0), MISCREG_CNTHCTL },
    { MiscRegNum32(15, 4, 14, 2, 0), MISCREG_CNTHP_TVAL },
    { MiscRegNum32(15, 4, 14, 2, 1), MISCREG_CNTHP_CTL },
    { MiscRegNum32(15, 6, 12, 12, 4), MISCREG_ICC_MCTLR },
    { MiscRegNum32(15, 6, 12, 12, 5), MISCREG_ICC_MSRE },
    { MiscRegNum32(15, 6, 12, 12, 7), MISCREG_ICC_MGRPEN1 },
    // MCRR/MRRC regs
    { MiscRegNum32(15, 0, 2), MISCREG_TTBR0 },
    { MiscRegNum32(15, 0, 7), MISCREG_PAR },
    { MiscRegNum32(15, 0, 9), MISCREG_PMCCNTR }, // ARMv8 AArch32 register
    { MiscRegNum32(15, 0, 12), MISCREG_ICC_SGI1R },
    { MiscRegNum32(15, 0, 14), MISCREG_CNTPCT },
    { MiscRegNum32(15, 0, 15), MISCREG_CPUMERRSR },
    { MiscRegNum32(15, 1, 2), MISCREG_TTBR1 },
    { MiscRegNum32(15, 1, 12), MISCREG_ICC_ASGI1R },
    { MiscRegNum32(15, 1, 14), MISCREG_CNTVCT },
    { MiscRegNum32(15, 1, 15), MISCREG_L2MERRSR },
    { MiscRegNum32(15, 2, 12), MISCREG_ICC_SGI0R },
    { MiscRegNum32(15, 2, 14), MISCREG_CNTP_CVAL },
    { MiscRegNum32(15, 3, 14), MISCREG_CNTV_CVAL },
    { MiscRegNum32(15, 4, 2), MISCREG_HTTBR },
    { MiscRegNum32(15, 4, 14), MISCREG_CNTVOFF },
    { MiscRegNum32(15, 6, 2), MISCREG_VTTBR },
    { MiscRegNum32(15, 6, 14), MISCREG_CNTHP_CVAL },
};

}

MiscRegIndex
decodeCP14Reg(unsigned crn, unsigned opc1, unsigned crm, unsigned opc2)
{
    MiscRegNum32 cop_reg(14, opc1, crn, crm, opc2);
    auto it = miscRegNum32ToIdx.find(cop_reg);
    if (it != miscRegNum32ToIdx.end()) {
        return it->second;
    } else {
        warn("CP14 unimplemented crn[%d], opc1[%d], crm[%d], opc2[%d]",
             crn, opc1, crm, opc2);
        return MISCREG_UNKNOWN;
    }
}

MiscRegIndex
decodeCP15Reg(unsigned crn, unsigned opc1, unsigned crm, unsigned opc2)
{
    MiscRegNum32 cop_reg(15, opc1, crn, crm, opc2);
    auto it = miscRegNum32ToIdx.find(cop_reg);
    if (it != miscRegNum32ToIdx.end()) {
        return it->second;
    } else {
        if ((crn == 15) ||
            (crn == 9 && (crm <= 2 || crm >= 5)) ||
            (crn == 10 && opc1 == 0 && crm <= 1) ||
            (crn == 11 && opc1 <= 7 && (crm <= 8 || crm ==15))) {
            return MISCREG_IMPDEF_UNIMPL;
        } else {
            return MISCREG_UNKNOWN;
        }
    }
}

MiscRegIndex
decodeCP15Reg64(unsigned crm, unsigned opc1)
{
    MiscRegNum32 cop_reg(15, opc1, crm);
    auto it = miscRegNum32ToIdx.find(cop_reg);
    if (it != miscRegNum32ToIdx.end()) {
        return it->second;
    } else {
        return MISCREG_UNKNOWN;
    }
}

std::tuple<bool, bool>
canReadCoprocReg(MiscRegIndex reg, SCR scr, CPSR cpsr, ThreadContext *tc)
{
    bool secure = !scr.ns;
    bool can_read = false;
    bool undefined = false;
    auto& miscreg_info = lookUpMiscReg[reg].info;

    switch (cpsr.mode) {
      case MODE_USER:
        can_read = secure ? miscreg_info[MISCREG_USR_S_RD] :
                            miscreg_info[MISCREG_USR_NS_RD];
        break;
      case MODE_FIQ:
      case MODE_IRQ:
      case MODE_SVC:
      case MODE_ABORT:
      case MODE_UNDEFINED:
      case MODE_SYSTEM:
        can_read = secure ? miscreg_info[MISCREG_PRI_S_RD] :
                            miscreg_info[MISCREG_PRI_NS_RD];
        break;
      case MODE_MON:
        can_read = secure ? miscreg_info[MISCREG_MON_NS0_RD] :
                            miscreg_info[MISCREG_MON_NS1_RD];
        break;
      case MODE_HYP:
        can_read = miscreg_info[MISCREG_HYP_NS_RD];
        break;
      default:
        undefined = true;
    }

    switch (reg) {
      case MISCREG_CNTFRQ ... MISCREG_CNTVOFF:
        if (!undefined)
            undefined = AArch32isUndefinedGenericTimer(reg, tc);
        break;
      default:
        break;
    }

    // can't do permissions checkes on the root of a banked pair of regs
    assert(!miscreg_info[MISCREG_BANKED]);
    return std::make_tuple(can_read, undefined);
}

std::tuple<bool, bool>
canWriteCoprocReg(MiscRegIndex reg, SCR scr, CPSR cpsr, ThreadContext *tc)
{
    bool secure = !scr.ns;
    bool can_write = false;
    bool undefined = false;
    const auto& miscreg_info = lookUpMiscReg[reg].info;

    switch (cpsr.mode) {
      case MODE_USER:
        can_write = secure ? miscreg_info[MISCREG_USR_S_WR] :
                             miscreg_info[MISCREG_USR_NS_WR];
        break;
      case MODE_FIQ:
      case MODE_IRQ:
      case MODE_SVC:
      case MODE_ABORT:
      case MODE_UNDEFINED:
      case MODE_SYSTEM:
        can_write = secure ? miscreg_info[MISCREG_PRI_S_WR] :
                             miscreg_info[MISCREG_PRI_NS_WR];
        break;
      case MODE_MON:
        can_write = secure ? miscreg_info[MISCREG_MON_NS0_WR] :
                             miscreg_info[MISCREG_MON_NS1_WR];
        break;
      case MODE_HYP:
        can_write =  miscreg_info[MISCREG_HYP_NS_WR];
        break;
      default:
        undefined = true;
    }

    switch (reg) {
      case MISCREG_CNTFRQ ... MISCREG_CNTVOFF:
        if (!undefined)
            undefined = AArch32isUndefinedGenericTimer(reg, tc);
        break;
      default:
        break;
    }

    // can't do permissions checkes on the root of a banked pair of regs
    assert(!miscreg_info[MISCREG_BANKED]);
    return std::make_tuple(can_write, undefined);
}

bool
AArch32isUndefinedGenericTimer(MiscRegIndex reg, ThreadContext *tc)
{
    if (currEL(tc) == EL0 && ELIs32(tc, EL1)) {
        const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        bool trap_cond = condGenericTimerSystemAccessTrapEL1(reg, tc);
        if (trap_cond && (!EL2Enabled(tc) || !hcr.tge))
            return true;
    }
    return false;
}

int
snsBankedIndex(MiscRegIndex reg, ThreadContext *tc)
{
    SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    return snsBankedIndex(reg, tc, scr.ns);
}

int
snsBankedIndex(MiscRegIndex reg, ThreadContext *tc, bool ns)
{
    int reg_as_int = static_cast<int>(reg);
    if (lookUpMiscReg[reg].info[MISCREG_BANKED]) {
        reg_as_int += (ArmSystem::haveEL(tc, EL3) &&
                      !ArmSystem::highestELIs64(tc) && !ns) ? 2 : 1;
    }
    return reg_as_int;
}

int
snsBankedIndex64(MiscRegIndex reg, ThreadContext *tc)
{
    auto *isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
    SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    return isa->snsBankedIndex64(reg, scr.ns);
}

/**
 * If the reg is a child reg of a banked set, then the parent is the last
 * banked one in the list. This is messy, and the wish is to eventually have
 * the bitmap replaced with a better data structure. the preUnflatten function
 * initializes a lookup table to speed up the search for these banked
 * registers.
 */

int unflattenResultMiscReg[NUM_MISCREGS];

void
preUnflattenMiscReg()
{
    int reg = -1;
    for (int i = 0 ; i < NUM_MISCREGS; i++){
        if (lookUpMiscReg[i].info[MISCREG_BANKED])
            reg = i;
        if (lookUpMiscReg[i].info[MISCREG_BANKED_CHILD])
            unflattenResultMiscReg[i] = reg;
        else
            unflattenResultMiscReg[i] = i;
        // if this assert fails, no parent was found, and something is broken
        assert(unflattenResultMiscReg[i] > -1);
    }
}

int
unflattenMiscReg(int reg)
{
    return unflattenResultMiscReg[reg];
}

Fault
checkFaultAccessAArch64SysReg(MiscRegIndex reg, CPSR cpsr,
                              ThreadContext *tc, const MiscRegOp64 &inst)
{
    return lookUpMiscReg[reg].checkFault(tc, inst, currEL(cpsr));
}

std::vector<struct MiscRegLUTEntry> lookUpMiscReg(NUM_MISCREGS);

namespace {
// The map is translating a MiscRegIndex into AArch64 system register
// numbers (op0, op1, crn, crm, op2)
std::unordered_map<MiscRegIndex, MiscRegNum64> idxToMiscRegNum;

// The map is translating AArch64 system register numbers
// (op0, op1, crn, crm, op2) into a MiscRegIndex
std::unordered_map<MiscRegNum64, MiscRegIndex> miscRegNumToIdx{
    { MiscRegNum64(1, 0, 7, 1, 0), MISCREG_IC_IALLUIS },
    { MiscRegNum64(1, 0, 7, 5, 0), MISCREG_IC_IALLU },
    { MiscRegNum64(1, 0, 7, 6, 1), MISCREG_DC_IVAC_Xt },
    { MiscRegNum64(1, 0, 7, 6, 2), MISCREG_DC_ISW_Xt },
    { MiscRegNum64(1, 0, 7, 8, 0), MISCREG_AT_S1E1R_Xt },
    { MiscRegNum64(1, 0, 7, 8, 1), MISCREG_AT_S1E1W_Xt },
    { MiscRegNum64(1, 0, 7, 8, 2), MISCREG_AT_S1E0R_Xt },
    { MiscRegNum64(1, 0, 7, 8, 3), MISCREG_AT_S1E0W_Xt },
    { MiscRegNum64(1, 0, 7, 10, 2), MISCREG_DC_CSW_Xt },
    { MiscRegNum64(1, 0, 7, 14, 2), MISCREG_DC_CISW_Xt },
    { MiscRegNum64(1, 0, 8, 1, 0), MISCREG_TLBI_VMALLE1OS },
    { MiscRegNum64(1, 0, 8, 1, 1), MISCREG_TLBI_VAE1OS },
    { MiscRegNum64(1, 0, 8, 1, 2), MISCREG_TLBI_ASIDE1OS },
    { MiscRegNum64(1, 0, 8, 1, 3), MISCREG_TLBI_VAAE1OS },
    { MiscRegNum64(1, 0, 8, 1, 5), MISCREG_TLBI_VALE1OS },
    { MiscRegNum64(1, 0, 8, 1, 7), MISCREG_TLBI_VAALE1OS },
    { MiscRegNum64(1, 0, 8, 2, 1), MISCREG_TLBI_RVAE1IS },
    { MiscRegNum64(1, 0, 8, 2, 3), MISCREG_TLBI_RVAAE1IS },
    { MiscRegNum64(1, 0, 8, 2, 5), MISCREG_TLBI_RVALE1IS },
    { MiscRegNum64(1, 0, 8, 2, 7), MISCREG_TLBI_RVAALE1IS },
    { MiscRegNum64(1, 0, 8, 3, 0), MISCREG_TLBI_VMALLE1IS },
    { MiscRegNum64(1, 0, 8, 3, 1), MISCREG_TLBI_VAE1IS },
    { MiscRegNum64(1, 0, 8, 3, 2), MISCREG_TLBI_ASIDE1IS },
    { MiscRegNum64(1, 0, 8, 3, 3), MISCREG_TLBI_VAAE1IS },
    { MiscRegNum64(1, 0, 8, 3, 5), MISCREG_TLBI_VALE1IS },
    { MiscRegNum64(1, 0, 8, 3, 7), MISCREG_TLBI_VAALE1IS },
    { MiscRegNum64(1, 0, 8, 5, 1), MISCREG_TLBI_RVAE1OS },
    { MiscRegNum64(1, 0, 8, 5, 3), MISCREG_TLBI_RVAAE1OS },
    { MiscRegNum64(1, 0, 8, 5, 5), MISCREG_TLBI_RVALE1OS },
    { MiscRegNum64(1, 0, 8, 5, 7), MISCREG_TLBI_RVAALE1OS },
    { MiscRegNum64(1, 0, 8, 6, 1), MISCREG_TLBI_RVAE1 },
    { MiscRegNum64(1, 0, 8, 6, 3), MISCREG_TLBI_RVAAE1 },
    { MiscRegNum64(1, 0, 8, 6, 5), MISCREG_TLBI_RVALE1 },
    { MiscRegNum64(1, 0, 8, 6, 7), MISCREG_TLBI_RVAALE1 },
    { MiscRegNum64(1, 0, 8, 7, 0), MISCREG_TLBI_VMALLE1 },
    { MiscRegNum64(1, 0, 8, 7, 1), MISCREG_TLBI_VAE1 },
    { MiscRegNum64(1, 0, 8, 7, 2), MISCREG_TLBI_ASIDE1 },
    { MiscRegNum64(1, 0, 8, 7, 3), MISCREG_TLBI_VAAE1 },
    { MiscRegNum64(1, 0, 8, 7, 5), MISCREG_TLBI_VALE1 },
    { MiscRegNum64(1, 0, 8, 7, 7), MISCREG_TLBI_VAALE1 },
    { MiscRegNum64(1, 0, 9, 1, 0), MISCREG_TLBI_VMALLE1OSNXS },
    { MiscRegNum64(1, 0, 9, 1, 1), MISCREG_TLBI_VAE1OSNXS },
    { MiscRegNum64(1, 0, 9, 1, 2), MISCREG_TLBI_ASIDE1OSNXS },
    { MiscRegNum64(1, 0, 9, 1, 3), MISCREG_TLBI_VAAE1OSNXS },
    { MiscRegNum64(1, 0, 9, 1, 5), MISCREG_TLBI_VALE1OSNXS },
    { MiscRegNum64(1, 0, 9, 1, 7), MISCREG_TLBI_VAALE1OSNXS },
    { MiscRegNum64(1, 0, 9, 2, 1), MISCREG_TLBI_RVAE1ISNXS },
    { MiscRegNum64(1, 0, 9, 2, 3), MISCREG_TLBI_RVAAE1ISNXS },
    { MiscRegNum64(1, 0, 9, 2, 5), MISCREG_TLBI_RVALE1ISNXS },
    { MiscRegNum64(1, 0, 9, 2, 7), MISCREG_TLBI_RVAALE1ISNXS },
    { MiscRegNum64(1, 0, 9, 3, 0), MISCREG_TLBI_VMALLE1ISNXS },
    { MiscRegNum64(1, 0, 9, 3, 1), MISCREG_TLBI_VAE1ISNXS },
    { MiscRegNum64(1, 0, 9, 3, 2), MISCREG_TLBI_ASIDE1ISNXS },
    { MiscRegNum64(1, 0, 9, 3, 3), MISCREG_TLBI_VAAE1ISNXS },
    { MiscRegNum64(1, 0, 9, 3, 5), MISCREG_TLBI_VALE1ISNXS },
    { MiscRegNum64(1, 0, 9, 3, 7), MISCREG_TLBI_VAALE1ISNXS },
    { MiscRegNum64(1, 0, 9, 5, 1), MISCREG_TLBI_RVAE1OSNXS },
    { MiscRegNum64(1, 0, 9, 5, 3), MISCREG_TLBI_RVAAE1OSNXS },
    { MiscRegNum64(1, 0, 9, 5, 5), MISCREG_TLBI_RVALE1OSNXS },
    { MiscRegNum64(1, 0, 9, 5, 7), MISCREG_TLBI_RVAALE1OSNXS },
    { MiscRegNum64(1, 0, 9, 6, 1), MISCREG_TLBI_RVAE1NXS },
    { MiscRegNum64(1, 0, 9, 6, 3), MISCREG_TLBI_RVAAE1NXS },
    { MiscRegNum64(1, 0, 9, 6, 5), MISCREG_TLBI_RVALE1NXS },
    { MiscRegNum64(1, 0, 9, 6, 7), MISCREG_TLBI_RVAALE1NXS },
    { MiscRegNum64(1, 0, 9, 7, 0), MISCREG_TLBI_VMALLE1NXS },
    { MiscRegNum64(1, 0, 9, 7, 1), MISCREG_TLBI_VAE1NXS },
    { MiscRegNum64(1, 0, 9, 7, 2), MISCREG_TLBI_ASIDE1NXS },
    { MiscRegNum64(1, 0, 9, 7, 3), MISCREG_TLBI_VAAE1NXS },
    { MiscRegNum64(1, 0, 9, 7, 5), MISCREG_TLBI_VALE1NXS },
    { MiscRegNum64(1, 0, 9, 7, 7), MISCREG_TLBI_VAALE1NXS },
    { MiscRegNum64(1, 3, 7, 4, 1), MISCREG_DC_ZVA_Xt },
    { MiscRegNum64(1, 3, 7, 5, 1), MISCREG_IC_IVAU_Xt },
    { MiscRegNum64(1, 3, 7, 10, 1), MISCREG_DC_CVAC_Xt },
    { MiscRegNum64(1, 3, 7, 11, 1), MISCREG_DC_CVAU_Xt },
    { MiscRegNum64(1, 3, 7, 14, 1), MISCREG_DC_CIVAC_Xt },
    { MiscRegNum64(1, 4, 7, 8, 0), MISCREG_AT_S1E2R_Xt },
    { MiscRegNum64(1, 4, 7, 8, 1), MISCREG_AT_S1E2W_Xt },
    { MiscRegNum64(1, 4, 7, 8, 4), MISCREG_AT_S12E1R_Xt },
    { MiscRegNum64(1, 4, 7, 8, 5), MISCREG_AT_S12E1W_Xt },
    { MiscRegNum64(1, 4, 7, 8, 6), MISCREG_AT_S12E0R_Xt },
    { MiscRegNum64(1, 4, 7, 8, 7), MISCREG_AT_S12E0W_Xt },
    { MiscRegNum64(1, 4, 8, 0, 1), MISCREG_TLBI_IPAS2E1IS },
    { MiscRegNum64(1, 4, 8, 0, 2), MISCREG_TLBI_RIPAS2E1IS },
    { MiscRegNum64(1, 4, 8, 0, 5), MISCREG_TLBI_IPAS2LE1IS },
    { MiscRegNum64(1, 4, 8, 1, 0), MISCREG_TLBI_ALLE2OS },
    { MiscRegNum64(1, 4, 8, 1, 1), MISCREG_TLBI_VAE2OS },
    { MiscRegNum64(1, 4, 8, 1, 4), MISCREG_TLBI_ALLE1OS },
    { MiscRegNum64(1, 4, 8, 1, 5), MISCREG_TLBI_VALE2OS },
    { MiscRegNum64(1, 4, 8, 1, 6), MISCREG_TLBI_VMALLS12E1OS },
    { MiscRegNum64(1, 4, 8, 0, 6), MISCREG_TLBI_RIPAS2LE1IS },
    { MiscRegNum64(1, 4, 8, 2, 1), MISCREG_TLBI_RVAE2IS },
    { MiscRegNum64(1, 4, 8, 2, 5), MISCREG_TLBI_RVALE2IS },
    { MiscRegNum64(1, 4, 8, 3, 0), MISCREG_TLBI_ALLE2IS },
    { MiscRegNum64(1, 4, 8, 3, 1), MISCREG_TLBI_VAE2IS },
    { MiscRegNum64(1, 4, 8, 3, 4), MISCREG_TLBI_ALLE1IS },
    { MiscRegNum64(1, 4, 8, 3, 5), MISCREG_TLBI_VALE2IS },
    { MiscRegNum64(1, 4, 8, 3, 6), MISCREG_TLBI_VMALLS12E1IS },
    { MiscRegNum64(1, 4, 8, 4, 0), MISCREG_TLBI_IPAS2E1OS },
    { MiscRegNum64(1, 4, 8, 4, 1), MISCREG_TLBI_IPAS2E1 },
    { MiscRegNum64(1, 4, 8, 4, 2), MISCREG_TLBI_RIPAS2E1 },
    { MiscRegNum64(1, 4, 8, 4, 3), MISCREG_TLBI_RIPAS2E1OS },
    { MiscRegNum64(1, 4, 8, 4, 4), MISCREG_TLBI_IPAS2LE1OS },
    { MiscRegNum64(1, 4, 8, 4, 5), MISCREG_TLBI_IPAS2LE1 },
    { MiscRegNum64(1, 4, 8, 4, 6), MISCREG_TLBI_RIPAS2LE1 },
    { MiscRegNum64(1, 4, 8, 4, 7), MISCREG_TLBI_RIPAS2LE1OS },
    { MiscRegNum64(1, 4, 8, 5, 1), MISCREG_TLBI_RVAE2OS },
    { MiscRegNum64(1, 4, 8, 5, 5), MISCREG_TLBI_RVALE2OS },
    { MiscRegNum64(1, 4, 8, 6, 1), MISCREG_TLBI_RVAE2 },
    { MiscRegNum64(1, 4, 8, 6, 5), MISCREG_TLBI_RVALE2 },
    { MiscRegNum64(1, 4, 8, 7, 0), MISCREG_TLBI_ALLE2 },
    { MiscRegNum64(1, 4, 8, 7, 1), MISCREG_TLBI_VAE2 },
    { MiscRegNum64(1, 4, 8, 7, 4), MISCREG_TLBI_ALLE1 },
    { MiscRegNum64(1, 4, 8, 7, 5), MISCREG_TLBI_VALE2 },
    { MiscRegNum64(1, 4, 8, 7, 6), MISCREG_TLBI_VMALLS12E1 },
    { MiscRegNum64(1, 4, 9, 0, 1), MISCREG_TLBI_IPAS2E1ISNXS },
    { MiscRegNum64(1, 4, 9, 0, 2), MISCREG_TLBI_RIPAS2E1ISNXS },
    { MiscRegNum64(1, 4, 9, 0, 5), MISCREG_TLBI_IPAS2LE1ISNXS },
    { MiscRegNum64(1, 4, 9, 1, 0), MISCREG_TLBI_ALLE2OSNXS },
    { MiscRegNum64(1, 4, 9, 1, 1), MISCREG_TLBI_VAE2OSNXS },
    { MiscRegNum64(1, 4, 9, 1, 4), MISCREG_TLBI_ALLE1OSNXS },
    { MiscRegNum64(1, 4, 9, 1, 5), MISCREG_TLBI_VALE2OSNXS },
    { MiscRegNum64(1, 4, 9, 1, 6), MISCREG_TLBI_VMALLS12E1OSNXS },
    { MiscRegNum64(1, 4, 9, 0, 6), MISCREG_TLBI_RIPAS2LE1ISNXS },
    { MiscRegNum64(1, 4, 9, 2, 1), MISCREG_TLBI_RVAE2ISNXS },
    { MiscRegNum64(1, 4, 9, 2, 5), MISCREG_TLBI_RVALE2ISNXS },
    { MiscRegNum64(1, 4, 9, 3, 0), MISCREG_TLBI_ALLE2ISNXS },
    { MiscRegNum64(1, 4, 9, 3, 1), MISCREG_TLBI_VAE2ISNXS },
    { MiscRegNum64(1, 4, 9, 3, 4), MISCREG_TLBI_ALLE1ISNXS },
    { MiscRegNum64(1, 4, 9, 3, 5), MISCREG_TLBI_VALE2ISNXS },
    { MiscRegNum64(1, 4, 9, 3, 6), MISCREG_TLBI_VMALLS12E1ISNXS },
    { MiscRegNum64(1, 4, 9, 4, 0), MISCREG_TLBI_IPAS2E1OSNXS },
    { MiscRegNum64(1, 4, 9, 4, 1), MISCREG_TLBI_IPAS2E1NXS },
    { MiscRegNum64(1, 4, 9, 4, 2), MISCREG_TLBI_RIPAS2E1NXS },
    { MiscRegNum64(1, 4, 9, 4, 3), MISCREG_TLBI_RIPAS2E1OSNXS },
    { MiscRegNum64(1, 4, 9, 4, 4), MISCREG_TLBI_IPAS2LE1OSNXS },
    { MiscRegNum64(1, 4, 9, 4, 5), MISCREG_TLBI_IPAS2LE1NXS },
    { MiscRegNum64(1, 4, 9, 4, 6), MISCREG_TLBI_RIPAS2LE1NXS },
    { MiscRegNum64(1, 4, 9, 4, 7), MISCREG_TLBI_RIPAS2LE1OSNXS },
    { MiscRegNum64(1, 4, 9, 5, 1), MISCREG_TLBI_RVAE2OSNXS },
    { MiscRegNum64(1, 4, 9, 5, 5), MISCREG_TLBI_RVALE2OSNXS },
    { MiscRegNum64(1, 4, 9, 6, 1), MISCREG_TLBI_RVAE2NXS },
    { MiscRegNum64(1, 4, 9, 6, 5), MISCREG_TLBI_RVALE2NXS },
    { MiscRegNum64(1, 4, 9, 7, 0), MISCREG_TLBI_ALLE2NXS },
    { MiscRegNum64(1, 4, 9, 7, 1), MISCREG_TLBI_VAE2NXS },
    { MiscRegNum64(1, 4, 9, 7, 4), MISCREG_TLBI_ALLE1NXS },
    { MiscRegNum64(1, 4, 9, 7, 5), MISCREG_TLBI_VALE2NXS },
    { MiscRegNum64(1, 4, 9, 7, 6), MISCREG_TLBI_VMALLS12E1NXS },
    { MiscRegNum64(1, 6, 7, 8, 0), MISCREG_AT_S1E3R_Xt },
    { MiscRegNum64(1, 6, 7, 8, 1), MISCREG_AT_S1E3W_Xt },
    { MiscRegNum64(1, 6, 8, 1, 0), MISCREG_TLBI_ALLE3OS },
    { MiscRegNum64(1, 6, 8, 1, 1), MISCREG_TLBI_VAE3OS },
    { MiscRegNum64(1, 6, 8, 1, 5), MISCREG_TLBI_VALE3OS },
    { MiscRegNum64(1, 6, 8, 2, 1), MISCREG_TLBI_RVAE3IS },
    { MiscRegNum64(1, 6, 8, 2, 5), MISCREG_TLBI_RVALE3IS },
    { MiscRegNum64(1, 6, 8, 3, 0), MISCREG_TLBI_ALLE3IS },
    { MiscRegNum64(1, 6, 8, 3, 1), MISCREG_TLBI_VAE3IS },
    { MiscRegNum64(1, 6, 8, 3, 5), MISCREG_TLBI_VALE3IS },
    { MiscRegNum64(1, 6, 8, 5, 1), MISCREG_TLBI_RVAE3OS },
    { MiscRegNum64(1, 6, 8, 5, 5), MISCREG_TLBI_RVALE3OS },
    { MiscRegNum64(1, 6, 8, 6, 1), MISCREG_TLBI_RVAE3 },
    { MiscRegNum64(1, 6, 8, 6, 5), MISCREG_TLBI_RVALE3 },
    { MiscRegNum64(1, 6, 8, 7, 0), MISCREG_TLBI_ALLE3 },
    { MiscRegNum64(1, 6, 8, 7, 1), MISCREG_TLBI_VAE3 },
    { MiscRegNum64(1, 6, 8, 7, 5), MISCREG_TLBI_VALE3 },
    { MiscRegNum64(1, 6, 9, 1, 0), MISCREG_TLBI_ALLE3OSNXS },
    { MiscRegNum64(1, 6, 9, 1, 1), MISCREG_TLBI_VAE3OSNXS },
    { MiscRegNum64(1, 6, 9, 1, 5), MISCREG_TLBI_VALE3OSNXS },
    { MiscRegNum64(1, 6, 9, 2, 1), MISCREG_TLBI_RVAE3ISNXS },
    { MiscRegNum64(1, 6, 9, 2, 5), MISCREG_TLBI_RVALE3ISNXS },
    { MiscRegNum64(1, 6, 9, 3, 0), MISCREG_TLBI_ALLE3ISNXS },
    { MiscRegNum64(1, 6, 9, 3, 1), MISCREG_TLBI_VAE3ISNXS },
    { MiscRegNum64(1, 6, 9, 3, 5), MISCREG_TLBI_VALE3ISNXS },
    { MiscRegNum64(1, 6, 9, 5, 1), MISCREG_TLBI_RVAE3OSNXS },
    { MiscRegNum64(1, 6, 9, 5, 5), MISCREG_TLBI_RVALE3OSNXS },
    { MiscRegNum64(1, 6, 9, 6, 1), MISCREG_TLBI_RVAE3NXS },
    { MiscRegNum64(1, 6, 9, 6, 5), MISCREG_TLBI_RVALE3NXS },
    { MiscRegNum64(1, 6, 9, 7, 0), MISCREG_TLBI_ALLE3NXS },
    { MiscRegNum64(1, 6, 9, 7, 1), MISCREG_TLBI_VAE3NXS },
    { MiscRegNum64(1, 6, 9, 7, 5), MISCREG_TLBI_VALE3NXS },
    { MiscRegNum64(2, 0, 0, 0, 2), MISCREG_OSDTRRX_EL1 },
    { MiscRegNum64(2, 0, 0, 0, 4), MISCREG_DBGBVR0_EL1 },
    { MiscRegNum64(2, 0, 0, 0, 5), MISCREG_DBGBCR0_EL1 },
    { MiscRegNum64(2, 0, 0, 0, 6), MISCREG_DBGWVR0_EL1 },
    { MiscRegNum64(2, 0, 0, 0, 7), MISCREG_DBGWCR0_EL1 },
    { MiscRegNum64(2, 0, 0, 1, 4), MISCREG_DBGBVR1_EL1 },
    { MiscRegNum64(2, 0, 0, 1, 5), MISCREG_DBGBCR1_EL1 },
    { MiscRegNum64(2, 0, 0, 1, 6), MISCREG_DBGWVR1_EL1 },
    { MiscRegNum64(2, 0, 0, 1, 7), MISCREG_DBGWCR1_EL1 },
    { MiscRegNum64(2, 0, 0, 2, 0), MISCREG_MDCCINT_EL1 },
    { MiscRegNum64(2, 0, 0, 2, 2), MISCREG_MDSCR_EL1 },
    { MiscRegNum64(2, 0, 0, 2, 4), MISCREG_DBGBVR2_EL1 },
    { MiscRegNum64(2, 0, 0, 2, 5), MISCREG_DBGBCR2_EL1 },
    { MiscRegNum64(2, 0, 0, 2, 6), MISCREG_DBGWVR2_EL1 },
    { MiscRegNum64(2, 0, 0, 2, 7), MISCREG_DBGWCR2_EL1 },
    { MiscRegNum64(2, 0, 0, 3, 2), MISCREG_OSDTRTX_EL1 },
    { MiscRegNum64(2, 0, 0, 3, 4), MISCREG_DBGBVR3_EL1 },
    { MiscRegNum64(2, 0, 0, 3, 5), MISCREG_DBGBCR3_EL1 },
    { MiscRegNum64(2, 0, 0, 3, 6), MISCREG_DBGWVR3_EL1 },
    { MiscRegNum64(2, 0, 0, 3, 7), MISCREG_DBGWCR3_EL1 },
    { MiscRegNum64(2, 0, 0, 4, 4), MISCREG_DBGBVR4_EL1 },
    { MiscRegNum64(2, 0, 0, 4, 5), MISCREG_DBGBCR4_EL1 },
    { MiscRegNum64(2, 0, 0, 4, 6), MISCREG_DBGWVR4_EL1 },
    { MiscRegNum64(2, 0, 0, 4, 7), MISCREG_DBGWCR4_EL1 },
    { MiscRegNum64(2, 0, 0, 5, 4), MISCREG_DBGBVR5_EL1 },
    { MiscRegNum64(2, 0, 0, 5, 5), MISCREG_DBGBCR5_EL1 },
    { MiscRegNum64(2, 0, 0, 5, 6), MISCREG_DBGWVR5_EL1 },
    { MiscRegNum64(2, 0, 0, 5, 7), MISCREG_DBGWCR5_EL1 },
    { MiscRegNum64(2, 0, 0, 6, 2), MISCREG_OSECCR_EL1 },
    { MiscRegNum64(2, 0, 0, 6, 4), MISCREG_DBGBVR6_EL1 },
    { MiscRegNum64(2, 0, 0, 6, 5), MISCREG_DBGBCR6_EL1 },
    { MiscRegNum64(2, 0, 0, 6, 6), MISCREG_DBGWVR6_EL1 },
    { MiscRegNum64(2, 0, 0, 6, 7), MISCREG_DBGWCR6_EL1 },
    { MiscRegNum64(2, 0, 0, 7, 4), MISCREG_DBGBVR7_EL1 },
    { MiscRegNum64(2, 0, 0, 7, 5), MISCREG_DBGBCR7_EL1 },
    { MiscRegNum64(2, 0, 0, 7, 6), MISCREG_DBGWVR7_EL1 },
    { MiscRegNum64(2, 0, 0, 7, 7), MISCREG_DBGWCR7_EL1 },
    { MiscRegNum64(2, 0, 0, 8, 4), MISCREG_DBGBVR8_EL1 },
    { MiscRegNum64(2, 0, 0, 8, 5), MISCREG_DBGBCR8_EL1 },
    { MiscRegNum64(2, 0, 0, 8, 6), MISCREG_DBGWVR8_EL1 },
    { MiscRegNum64(2, 0, 0, 8, 7), MISCREG_DBGWCR8_EL1 },
    { MiscRegNum64(2, 0, 0, 9, 4), MISCREG_DBGBVR9_EL1 },
    { MiscRegNum64(2, 0, 0, 9, 5), MISCREG_DBGBCR9_EL1 },
    { MiscRegNum64(2, 0, 0, 9, 6), MISCREG_DBGWVR9_EL1 },
    { MiscRegNum64(2, 0, 0, 9, 7), MISCREG_DBGWCR9_EL1 },
    { MiscRegNum64(2, 0, 0, 10, 4), MISCREG_DBGBVR10_EL1 },
    { MiscRegNum64(2, 0, 0, 10, 5), MISCREG_DBGBCR10_EL1 },
    { MiscRegNum64(2, 0, 0, 10, 6), MISCREG_DBGWVR10_EL1 },
    { MiscRegNum64(2, 0, 0, 10, 7), MISCREG_DBGWCR10_EL1 },
    { MiscRegNum64(2, 0, 0, 11, 4), MISCREG_DBGBVR11_EL1 },
    { MiscRegNum64(2, 0, 0, 11, 5), MISCREG_DBGBCR11_EL1 },
    { MiscRegNum64(2, 0, 0, 11, 6), MISCREG_DBGWVR11_EL1 },
    { MiscRegNum64(2, 0, 0, 11, 7), MISCREG_DBGWCR11_EL1 },
    { MiscRegNum64(2, 0, 0, 12, 4), MISCREG_DBGBVR12_EL1 },
    { MiscRegNum64(2, 0, 0, 12, 5), MISCREG_DBGBCR12_EL1 },
    { MiscRegNum64(2, 0, 0, 12, 6), MISCREG_DBGWVR12_EL1 },
    { MiscRegNum64(2, 0, 0, 12, 7), MISCREG_DBGWCR12_EL1 },
    { MiscRegNum64(2, 0, 0, 13, 4), MISCREG_DBGBVR13_EL1 },
    { MiscRegNum64(2, 0, 0, 13, 5), MISCREG_DBGBCR13_EL1 },
    { MiscRegNum64(2, 0, 0, 13, 6), MISCREG_DBGWVR13_EL1 },
    { MiscRegNum64(2, 0, 0, 13, 7), MISCREG_DBGWCR13_EL1 },
    { MiscRegNum64(2, 0, 0, 14, 4), MISCREG_DBGBVR14_EL1 },
    { MiscRegNum64(2, 0, 0, 14, 5), MISCREG_DBGBCR14_EL1 },
    { MiscRegNum64(2, 0, 0, 14, 6), MISCREG_DBGWVR14_EL1 },
    { MiscRegNum64(2, 0, 0, 14, 7), MISCREG_DBGWCR14_EL1 },
    { MiscRegNum64(2, 0, 0, 15, 4), MISCREG_DBGBVR15_EL1 },
    { MiscRegNum64(2, 0, 0, 15, 5), MISCREG_DBGBCR15_EL1 },
    { MiscRegNum64(2, 0, 0, 15, 6), MISCREG_DBGWVR15_EL1 },
    { MiscRegNum64(2, 0, 0, 15, 7), MISCREG_DBGWCR15_EL1 },
    { MiscRegNum64(2, 0, 1, 0, 0), MISCREG_MDRAR_EL1 },
    { MiscRegNum64(2, 0, 1, 0, 4), MISCREG_OSLAR_EL1 },
    { MiscRegNum64(2, 0, 1, 1, 4), MISCREG_OSLSR_EL1 },
    { MiscRegNum64(2, 0, 1, 3, 4), MISCREG_OSDLR_EL1 },
    { MiscRegNum64(2, 0, 1, 4, 4), MISCREG_DBGPRCR_EL1 },
    { MiscRegNum64(2, 0, 7, 8, 6), MISCREG_DBGCLAIMSET_EL1 },
    { MiscRegNum64(2, 0, 7, 9, 6), MISCREG_DBGCLAIMCLR_EL1 },
    { MiscRegNum64(2, 0, 7, 14, 6), MISCREG_DBGAUTHSTATUS_EL1 },
    { MiscRegNum64(2, 2, 0, 0, 0), MISCREG_TEECR32_EL1 },
    { MiscRegNum64(2, 2, 1, 0, 0), MISCREG_TEEHBR32_EL1 },
    { MiscRegNum64(2, 3, 0, 1, 0), MISCREG_MDCCSR_EL0 },
    { MiscRegNum64(2, 3, 0, 4, 0), MISCREG_MDDTR_EL0 },
    { MiscRegNum64(2, 3, 0, 5, 0), MISCREG_MDDTRRX_EL0 },
    { MiscRegNum64(2, 4, 0, 7, 0), MISCREG_DBGVCR32_EL2 },
    { MiscRegNum64(3, 0, 0, 0, 0), MISCREG_MIDR_EL1 },
    { MiscRegNum64(3, 0, 0, 0, 5), MISCREG_MPIDR_EL1 },
    { MiscRegNum64(3, 0, 0, 0, 6), MISCREG_REVIDR_EL1 },
    { MiscRegNum64(3, 0, 0, 1, 0), MISCREG_ID_PFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 1, 1), MISCREG_ID_PFR1_EL1 },
    { MiscRegNum64(3, 0, 0, 1, 2), MISCREG_ID_DFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 1, 3), MISCREG_ID_AFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 1, 4), MISCREG_ID_MMFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 1, 5), MISCREG_ID_MMFR1_EL1 },
    { MiscRegNum64(3, 0, 0, 1, 6), MISCREG_ID_MMFR2_EL1 },
    { MiscRegNum64(3, 0, 0, 1, 7), MISCREG_ID_MMFR3_EL1 },
    { MiscRegNum64(3, 0, 0, 2, 0), MISCREG_ID_ISAR0_EL1 },
    { MiscRegNum64(3, 0, 0, 2, 1), MISCREG_ID_ISAR1_EL1 },
    { MiscRegNum64(3, 0, 0, 2, 2), MISCREG_ID_ISAR2_EL1 },
    { MiscRegNum64(3, 0, 0, 2, 3), MISCREG_ID_ISAR3_EL1 },
    { MiscRegNum64(3, 0, 0, 2, 4), MISCREG_ID_ISAR4_EL1 },
    { MiscRegNum64(3, 0, 0, 2, 5), MISCREG_ID_ISAR5_EL1 },
    { MiscRegNum64(3, 0, 0, 2, 6), MISCREG_ID_MMFR4_EL1 },
    { MiscRegNum64(3, 0, 0, 2, 7), MISCREG_ID_ISAR6_EL1 },
    { MiscRegNum64(3, 0, 0, 3, 0), MISCREG_MVFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 3, 1), MISCREG_MVFR1_EL1 },
    { MiscRegNum64(3, 0, 0, 3, 2), MISCREG_MVFR2_EL1 },
    { MiscRegNum64(3, 0, 0, 3, 3), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 3, 4), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 3, 5), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 3, 6), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 3, 7), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 4, 0), MISCREG_ID_AA64PFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 4, 1), MISCREG_ID_AA64PFR1_EL1 },
    { MiscRegNum64(3, 0, 0, 4, 2), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 4, 3), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 4, 4), MISCREG_ID_AA64ZFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 4, 5), MISCREG_ID_AA64SMFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 4, 6), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 4, 7), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 5, 0), MISCREG_ID_AA64DFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 5, 1), MISCREG_ID_AA64DFR1_EL1 },
    { MiscRegNum64(3, 0, 0, 5, 2), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 5, 3), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 5, 4), MISCREG_ID_AA64AFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 5, 5), MISCREG_ID_AA64AFR1_EL1 },
    { MiscRegNum64(3, 0, 0, 5, 6), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 5, 7), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 6, 0), MISCREG_ID_AA64ISAR0_EL1 },
    { MiscRegNum64(3, 0, 0, 6, 1), MISCREG_ID_AA64ISAR1_EL1 },
    { MiscRegNum64(3, 0, 0, 6, 2), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 6, 3), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 6, 4), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 6, 5), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 6, 6), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 6, 7), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 7, 0), MISCREG_ID_AA64MMFR0_EL1 },
    { MiscRegNum64(3, 0, 0, 7, 1), MISCREG_ID_AA64MMFR1_EL1 },
    { MiscRegNum64(3, 0, 0, 7, 2), MISCREG_ID_AA64MMFR2_EL1 },
    { MiscRegNum64(3, 0, 0, 7, 3), MISCREG_ID_AA64MMFR3_EL1 },
    { MiscRegNum64(3, 0, 0, 7, 4), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 7, 5), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 7, 6), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 0, 7, 7), MISCREG_RAZ },
    { MiscRegNum64(3, 0, 1, 0, 0), MISCREG_SCTLR_EL1 },
    { MiscRegNum64(3, 0, 1, 0, 1), MISCREG_ACTLR_EL1 },
    { MiscRegNum64(3, 0, 1, 0, 2), MISCREG_CPACR_EL1 },
    { MiscRegNum64(3, 0, 1, 0, 3), MISCREG_SCTLR2_EL1 },
    { MiscRegNum64(3, 0, 1, 2, 0), MISCREG_ZCR_EL1 },
    { MiscRegNum64(3, 0, 1, 2, 4), MISCREG_SMPRI_EL1 },
    { MiscRegNum64(3, 0, 1, 2, 6), MISCREG_SMCR_EL1 },
    { MiscRegNum64(3, 0, 2, 0, 0), MISCREG_TTBR0_EL1 },
    { MiscRegNum64(3, 0, 2, 0, 1), MISCREG_TTBR1_EL1 },
    { MiscRegNum64(3, 0, 2, 0, 2), MISCREG_TCR_EL1 },
    { MiscRegNum64(3, 0, 2, 0, 3), MISCREG_TCR2_EL1 },
    { MiscRegNum64(3, 0, 2, 1, 0), MISCREG_APIAKeyLo_EL1 },
    { MiscRegNum64(3, 0, 2, 1, 1), MISCREG_APIAKeyHi_EL1 },
    { MiscRegNum64(3, 0, 2, 1, 2), MISCREG_APIBKeyLo_EL1 },
    { MiscRegNum64(3, 0, 2, 1, 3), MISCREG_APIBKeyHi_EL1 },
    { MiscRegNum64(3, 0, 2, 2, 0), MISCREG_APDAKeyLo_EL1 },
    { MiscRegNum64(3, 0, 2, 2, 1), MISCREG_APDAKeyHi_EL1 },
    { MiscRegNum64(3, 0, 2, 2, 2), MISCREG_APDBKeyLo_EL1 },
    { MiscRegNum64(3, 0, 2, 2, 3), MISCREG_APDBKeyHi_EL1 },
    { MiscRegNum64(3, 0, 2, 3, 0), MISCREG_APGAKeyLo_EL1 },
    { MiscRegNum64(3, 0, 2, 3, 1), MISCREG_APGAKeyHi_EL1 },
    { MiscRegNum64(3, 0, 4, 0, 0), MISCREG_SPSR_EL1 },
    { MiscRegNum64(3, 0, 4, 0, 1), MISCREG_ELR_EL1 },
    { MiscRegNum64(3, 0, 4, 1, 0), MISCREG_SP_EL0 },
    { MiscRegNum64(3, 0, 4, 2, 0), MISCREG_SPSEL },
    { MiscRegNum64(3, 0, 4, 2, 2), MISCREG_CURRENTEL },
    { MiscRegNum64(3, 0, 4, 2, 3), MISCREG_PAN },
    { MiscRegNum64(3, 0, 4, 2, 4), MISCREG_UAO },
    { MiscRegNum64(3, 0, 4, 6, 0), MISCREG_ICC_PMR_EL1 },
    { MiscRegNum64(3, 0, 5, 1, 0), MISCREG_AFSR0_EL1 },
    { MiscRegNum64(3, 0, 5, 1, 1), MISCREG_AFSR1_EL1 },
    { MiscRegNum64(3, 0, 5, 2, 0), MISCREG_ESR_EL1 },
    { MiscRegNum64(3, 0, 5, 3, 0), MISCREG_ERRIDR_EL1 },
    { MiscRegNum64(3, 0, 5, 3, 1), MISCREG_ERRSELR_EL1 },
    { MiscRegNum64(3, 0, 5, 4, 0), MISCREG_ERXFR_EL1 },
    { MiscRegNum64(3, 0, 5, 4, 1), MISCREG_ERXCTLR_EL1 },
    { MiscRegNum64(3, 0, 5, 4, 2), MISCREG_ERXSTATUS_EL1 },
    { MiscRegNum64(3, 0, 5, 4, 3), MISCREG_ERXADDR_EL1 },
    { MiscRegNum64(3, 0, 5, 5, 0), MISCREG_ERXMISC0_EL1 },
    { MiscRegNum64(3, 0, 5, 5, 1), MISCREG_ERXMISC1_EL1 },
    { MiscRegNum64(3, 0, 6, 0, 0), MISCREG_FAR_EL1 },
    { MiscRegNum64(3, 0, 7, 4, 0), MISCREG_PAR_EL1 },
    { MiscRegNum64(3, 0, 9, 14, 1), MISCREG_PMINTENSET_EL1 },
    { MiscRegNum64(3, 0, 9, 14, 2), MISCREG_PMINTENCLR_EL1 },
    { MiscRegNum64(3, 0, 10, 2, 0), MISCREG_MAIR_EL1 },
    { MiscRegNum64(3, 0, 10, 3, 0), MISCREG_AMAIR_EL1 },
    { MiscRegNum64(3, 0, 10, 4, 4), MISCREG_MPAMIDR_EL1 },
    { MiscRegNum64(3, 0, 10, 5, 0), MISCREG_MPAM1_EL1 },
    { MiscRegNum64(3, 0, 10, 5, 1), MISCREG_MPAM0_EL1 },
    { MiscRegNum64(3, 0, 10, 5, 3), MISCREG_MPAMSM_EL1 },
    { MiscRegNum64(3, 0, 12, 0, 0), MISCREG_VBAR_EL1 },
    { MiscRegNum64(3, 0, 12, 0, 1), MISCREG_RVBAR_EL1 },
    { MiscRegNum64(3, 0, 12, 1, 0), MISCREG_ISR_EL1 },
    { MiscRegNum64(3, 0, 12, 1, 1), MISCREG_DISR_EL1 },
    { MiscRegNum64(3, 0, 12, 8, 0), MISCREG_ICC_IAR0_EL1 },
    { MiscRegNum64(3, 0, 12, 8, 1), MISCREG_ICC_EOIR0_EL1 },
    { MiscRegNum64(3, 0, 12, 8, 2), MISCREG_ICC_HPPIR0_EL1 },
    { MiscRegNum64(3, 0, 12, 8, 3), MISCREG_ICC_BPR0_EL1 },
    { MiscRegNum64(3, 0, 12, 8, 4), MISCREG_ICC_AP0R0_EL1 },
    { MiscRegNum64(3, 0, 12, 8, 5), MISCREG_ICC_AP0R1_EL1 },
    { MiscRegNum64(3, 0, 12, 8, 6), MISCREG_ICC_AP0R2_EL1 },
    { MiscRegNum64(3, 0, 12, 8, 7), MISCREG_ICC_AP0R3_EL1 },
    { MiscRegNum64(3, 0, 12, 9, 0), MISCREG_ICC_AP1R0_EL1 },
    { MiscRegNum64(3, 0, 12, 9, 1), MISCREG_ICC_AP1R1_EL1 },
    { MiscRegNum64(3, 0, 12, 9, 2), MISCREG_ICC_AP1R2_EL1 },
    { MiscRegNum64(3, 0, 12, 9, 3), MISCREG_ICC_AP1R3_EL1 },
    { MiscRegNum64(3, 0, 12, 11, 1), MISCREG_ICC_DIR_EL1 },
    { MiscRegNum64(3, 0, 12, 11, 3), MISCREG_ICC_RPR_EL1 },
    { MiscRegNum64(3, 0, 12, 11, 5), MISCREG_ICC_SGI1R_EL1 },
    { MiscRegNum64(3, 0, 12, 11, 6), MISCREG_ICC_ASGI1R_EL1 },
    { MiscRegNum64(3, 0, 12, 11, 7), MISCREG_ICC_SGI0R_EL1 },
    { MiscRegNum64(3, 0, 12, 12, 0), MISCREG_ICC_IAR1_EL1 },
    { MiscRegNum64(3, 0, 12, 12, 1), MISCREG_ICC_EOIR1_EL1 },
    { MiscRegNum64(3, 0, 12, 12, 2), MISCREG_ICC_HPPIR1_EL1 },
    { MiscRegNum64(3, 0, 12, 12, 3), MISCREG_ICC_BPR1_EL1 },
    { MiscRegNum64(3, 0, 12, 12, 4), MISCREG_ICC_CTLR_EL1 },
    { MiscRegNum64(3, 0, 12, 12, 5), MISCREG_ICC_SRE_EL1 },
    { MiscRegNum64(3, 0, 12, 12, 6), MISCREG_ICC_IGRPEN0_EL1 },
    { MiscRegNum64(3, 0, 12, 12, 7), MISCREG_ICC_IGRPEN1_EL1 },
    { MiscRegNum64(3, 0, 13, 0, 1), MISCREG_CONTEXTIDR_EL1 },
    { MiscRegNum64(3, 0, 13, 0, 4), MISCREG_TPIDR_EL1 },
    { MiscRegNum64(3, 0, 14, 1, 0), MISCREG_CNTKCTL_EL1 },
    { MiscRegNum64(3, 0, 15, 0, 0), MISCREG_IL1DATA0_EL1 },
    { MiscRegNum64(3, 0, 15, 0, 1), MISCREG_IL1DATA1_EL1 },
    { MiscRegNum64(3, 0, 15, 0, 2), MISCREG_IL1DATA2_EL1 },
    { MiscRegNum64(3, 0, 15, 0, 3), MISCREG_IL1DATA3_EL1 },
    { MiscRegNum64(3, 0, 15, 1, 0), MISCREG_DL1DATA0_EL1 },
    { MiscRegNum64(3, 0, 15, 1, 1), MISCREG_DL1DATA1_EL1 },
    { MiscRegNum64(3, 0, 15, 1, 2), MISCREG_DL1DATA2_EL1 },
    { MiscRegNum64(3, 0, 15, 1, 3), MISCREG_DL1DATA3_EL1 },
    { MiscRegNum64(3, 0, 15, 1, 4), MISCREG_DL1DATA4_EL1 },
    { MiscRegNum64(3, 1, 0, 0, 0), MISCREG_CCSIDR_EL1 },
    { MiscRegNum64(3, 1, 0, 0, 1), MISCREG_CLIDR_EL1 },
    { MiscRegNum64(3, 1, 0, 0, 6), MISCREG_SMIDR_EL1 },
    { MiscRegNum64(3, 1, 0, 0, 7), MISCREG_AIDR_EL1 },
    { MiscRegNum64(3, 1, 11, 0, 2), MISCREG_L2CTLR_EL1 },
    { MiscRegNum64(3, 1, 11, 0, 3), MISCREG_L2ECTLR_EL1 },
    { MiscRegNum64(3, 1, 15, 0, 0), MISCREG_L2ACTLR_EL1 },
    { MiscRegNum64(3, 1, 15, 2, 0), MISCREG_CPUACTLR_EL1 },
    { MiscRegNum64(3, 1, 15, 2, 1), MISCREG_CPUECTLR_EL1 },
    { MiscRegNum64(3, 1, 15, 2, 2), MISCREG_CPUMERRSR_EL1 },
    { MiscRegNum64(3, 1, 15, 2, 3), MISCREG_L2MERRSR_EL1 },
    { MiscRegNum64(3, 1, 15, 3, 0), MISCREG_CBAR_EL1 },
    { MiscRegNum64(3, 2, 0, 0, 0), MISCREG_CSSELR_EL1 },
    { MiscRegNum64(3, 3, 0, 0, 1), MISCREG_CTR_EL0 },
    { MiscRegNum64(3, 3, 0, 0, 7), MISCREG_DCZID_EL0 },
    { MiscRegNum64(3, 3, 2, 4, 0), MISCREG_RNDR },
    { MiscRegNum64(3, 3, 2, 4, 1), MISCREG_RNDRRS },
    { MiscRegNum64(3, 3, 4, 2, 0), MISCREG_NZCV },
    { MiscRegNum64(3, 3, 4, 2, 1), MISCREG_DAIF },
    { MiscRegNum64(3, 3, 4, 2, 2), MISCREG_SVCR },
    { MiscRegNum64(3, 3, 4, 4, 0), MISCREG_FPCR },
    { MiscRegNum64(3, 3, 4, 4, 1), MISCREG_FPSR },
    { MiscRegNum64(3, 3, 4, 5, 0), MISCREG_DSPSR_EL0 },
    { MiscRegNum64(3, 3, 4, 5, 1), MISCREG_DLR_EL0 },
    { MiscRegNum64(3, 3, 9, 12, 0), MISCREG_PMCR_EL0 },
    { MiscRegNum64(3, 3, 9, 12, 1), MISCREG_PMCNTENSET_EL0 },
    { MiscRegNum64(3, 3, 9, 12, 2), MISCREG_PMCNTENCLR_EL0 },
    { MiscRegNum64(3, 3, 9, 12, 3), MISCREG_PMOVSCLR_EL0 },
    { MiscRegNum64(3, 3, 9, 12, 4), MISCREG_PMSWINC_EL0 },
    { MiscRegNum64(3, 3, 9, 12, 5), MISCREG_PMSELR_EL0 },
    { MiscRegNum64(3, 3, 9, 12, 6), MISCREG_PMCEID0_EL0 },
    { MiscRegNum64(3, 3, 9, 12, 7), MISCREG_PMCEID1_EL0 },
    { MiscRegNum64(3, 3, 9, 13, 0), MISCREG_PMCCNTR_EL0 },
    { MiscRegNum64(3, 3, 9, 13, 1), MISCREG_PMXEVTYPER_EL0 },
    { MiscRegNum64(3, 3, 9, 13, 2), MISCREG_PMXEVCNTR_EL0 },
    { MiscRegNum64(3, 3, 9, 14, 0), MISCREG_PMUSERENR_EL0 },
    { MiscRegNum64(3, 3, 9, 14, 3), MISCREG_PMOVSSET_EL0 },
    { MiscRegNum64(3, 3, 13, 0, 2), MISCREG_TPIDR_EL0 },
    { MiscRegNum64(3, 3, 13, 0, 3), MISCREG_TPIDRRO_EL0 },
    { MiscRegNum64(3, 3, 13, 0, 5), MISCREG_TPIDR2_EL0 },
    { MiscRegNum64(3, 3, 14, 0, 0), MISCREG_CNTFRQ_EL0 },
    { MiscRegNum64(3, 3, 14, 0, 1), MISCREG_CNTPCT_EL0 },
    { MiscRegNum64(3, 3, 14, 0, 2), MISCREG_CNTVCT_EL0 },
    { MiscRegNum64(3, 3, 14, 2, 0), MISCREG_CNTP_TVAL_EL0 },
    { MiscRegNum64(3, 3, 14, 2, 1), MISCREG_CNTP_CTL_EL0 },
    { MiscRegNum64(3, 3, 14, 2, 2), MISCREG_CNTP_CVAL_EL0 },
    { MiscRegNum64(3, 3, 14, 3, 0), MISCREG_CNTV_TVAL_EL0 },
    { MiscRegNum64(3, 3, 14, 3, 1), MISCREG_CNTV_CTL_EL0 },
    { MiscRegNum64(3, 3, 14, 3, 2), MISCREG_CNTV_CVAL_EL0 },
    { MiscRegNum64(3, 3, 14, 8, 0), MISCREG_PMEVCNTR0_EL0 },
    { MiscRegNum64(3, 3, 14, 8, 1), MISCREG_PMEVCNTR1_EL0 },
    { MiscRegNum64(3, 3, 14, 8, 2), MISCREG_PMEVCNTR2_EL0 },
    { MiscRegNum64(3, 3, 14, 8, 3), MISCREG_PMEVCNTR3_EL0 },
    { MiscRegNum64(3, 3, 14, 8, 4), MISCREG_PMEVCNTR4_EL0 },
    { MiscRegNum64(3, 3, 14, 8, 5), MISCREG_PMEVCNTR5_EL0 },
    { MiscRegNum64(3, 3, 14, 12, 0), MISCREG_PMEVTYPER0_EL0 },
    { MiscRegNum64(3, 3, 14, 12, 1), MISCREG_PMEVTYPER1_EL0 },
    { MiscRegNum64(3, 3, 14, 12, 2), MISCREG_PMEVTYPER2_EL0 },
    { MiscRegNum64(3, 3, 14, 12, 3), MISCREG_PMEVTYPER3_EL0 },
    { MiscRegNum64(3, 3, 14, 12, 4), MISCREG_PMEVTYPER4_EL0 },
    { MiscRegNum64(3, 3, 14, 12, 5), MISCREG_PMEVTYPER5_EL0 },
    { MiscRegNum64(3, 3, 14, 15, 7), MISCREG_PMCCFILTR_EL0 },
    { MiscRegNum64(3, 4, 0, 0, 0), MISCREG_VPIDR_EL2 },
    { MiscRegNum64(3, 4, 0, 0, 5), MISCREG_VMPIDR_EL2 },
    { MiscRegNum64(3, 4, 1, 0, 0), MISCREG_SCTLR_EL2 },
    { MiscRegNum64(3, 4, 1, 0, 1), MISCREG_ACTLR_EL2 },
    { MiscRegNum64(3, 4, 1, 0, 3), MISCREG_SCTLR2_EL2 },
    { MiscRegNum64(3, 4, 1, 1, 0), MISCREG_HCR_EL2 },
    { MiscRegNum64(3, 4, 1, 1, 1), MISCREG_MDCR_EL2 },
    { MiscRegNum64(3, 4, 1, 1, 2), MISCREG_CPTR_EL2 },
    { MiscRegNum64(3, 4, 1, 1, 3), MISCREG_HSTR_EL2 },
    { MiscRegNum64(3, 4, 1, 1, 4), MISCREG_HFGRTR_EL2 },
    { MiscRegNum64(3, 4, 1, 1, 5), MISCREG_HFGWTR_EL2 },
    { MiscRegNum64(3, 4, 1, 1, 6), MISCREG_HFGITR_EL2 },
    { MiscRegNum64(3, 4, 1, 1, 7), MISCREG_HACR_EL2 },
    { MiscRegNum64(3, 4, 1, 2, 0), MISCREG_ZCR_EL2 },
    { MiscRegNum64(3, 4, 1, 2, 2), MISCREG_HCRX_EL2 },
    { MiscRegNum64(3, 4, 1, 2, 5), MISCREG_SMPRIMAP_EL2 },
    { MiscRegNum64(3, 4, 1, 2, 6), MISCREG_SMCR_EL2 },
    { MiscRegNum64(3, 4, 2, 0, 0), MISCREG_TTBR0_EL2 },
    { MiscRegNum64(3, 4, 2, 0, 1), MISCREG_TTBR1_EL2 },
    { MiscRegNum64(3, 4, 2, 0, 2), MISCREG_TCR_EL2 },
    { MiscRegNum64(3, 4, 2, 0, 3), MISCREG_TCR2_EL2 },
    { MiscRegNum64(3, 4, 2, 1, 0), MISCREG_VTTBR_EL2 },
    { MiscRegNum64(3, 4, 2, 1, 2), MISCREG_VTCR_EL2 },
    { MiscRegNum64(3, 4, 2, 6, 0), MISCREG_VSTTBR_EL2 },
    { MiscRegNum64(3, 4, 2, 6, 2), MISCREG_VSTCR_EL2 },
    { MiscRegNum64(3, 4, 3, 0, 0), MISCREG_DACR32_EL2 },
    { MiscRegNum64(3, 4, 3, 1, 4), MISCREG_HDFGRTR_EL2 },
    { MiscRegNum64(3, 4, 3, 1, 5), MISCREG_HDFGWTR_EL2 },
    { MiscRegNum64(3, 4, 4, 0, 0), MISCREG_SPSR_EL2 },
    { MiscRegNum64(3, 4, 4, 0, 1), MISCREG_ELR_EL2 },
    { MiscRegNum64(3, 4, 4, 1, 0), MISCREG_SP_EL1 },
    { MiscRegNum64(3, 4, 4, 3, 0), MISCREG_SPSR_IRQ_AA64 },
    { MiscRegNum64(3, 4, 4, 3, 1), MISCREG_SPSR_ABT_AA64 },
    { MiscRegNum64(3, 4, 4, 3, 2), MISCREG_SPSR_UND_AA64 },
    { MiscRegNum64(3, 4, 4, 3, 3), MISCREG_SPSR_FIQ_AA64 },
    { MiscRegNum64(3, 4, 5, 0, 1), MISCREG_IFSR32_EL2 },
    { MiscRegNum64(3, 4, 5, 1, 0), MISCREG_AFSR0_EL2 },
    { MiscRegNum64(3, 4, 5, 1, 1), MISCREG_AFSR1_EL2 },
    { MiscRegNum64(3, 4, 5, 2, 0), MISCREG_ESR_EL2 },
    { MiscRegNum64(3, 4, 5, 2, 3), MISCREG_VSESR_EL2 },
    { MiscRegNum64(3, 4, 5, 3, 0), MISCREG_FPEXC32_EL2 },
    { MiscRegNum64(3, 4, 6, 0, 0), MISCREG_FAR_EL2 },
    { MiscRegNum64(3, 4, 6, 0, 4), MISCREG_HPFAR_EL2 },
    { MiscRegNum64(3, 4, 10, 2, 0), MISCREG_MAIR_EL2 },
    { MiscRegNum64(3, 4, 10, 3, 0), MISCREG_AMAIR_EL2 },
    { MiscRegNum64(3, 4, 10, 4, 0), MISCREG_MPAMHCR_EL2 },
    { MiscRegNum64(3, 4, 10, 4, 1), MISCREG_MPAMVPMV_EL2 },
    { MiscRegNum64(3, 4, 10, 5, 0), MISCREG_MPAM2_EL2 },
    { MiscRegNum64(3, 4, 10, 6, 0), MISCREG_MPAMVPM0_EL2 },
    { MiscRegNum64(3, 4, 10, 6, 1), MISCREG_MPAMVPM1_EL2 },
    { MiscRegNum64(3, 4, 10, 6, 2), MISCREG_MPAMVPM2_EL2 },
    { MiscRegNum64(3, 4, 10, 6, 3), MISCREG_MPAMVPM3_EL2 },
    { MiscRegNum64(3, 4, 10, 6, 4), MISCREG_MPAMVPM4_EL2 },
    { MiscRegNum64(3, 4, 10, 6, 5), MISCREG_MPAMVPM5_EL2 },
    { MiscRegNum64(3, 4, 10, 6, 6), MISCREG_MPAMVPM6_EL2 },
    { MiscRegNum64(3, 4, 10, 6, 7), MISCREG_MPAMVPM7_EL2 },
    { MiscRegNum64(3, 4, 12, 0, 0), MISCREG_VBAR_EL2 },
    { MiscRegNum64(3, 4, 12, 0, 1), MISCREG_RVBAR_EL2 },
    { MiscRegNum64(3, 4, 12, 1, 1), MISCREG_VDISR_EL2 },
    { MiscRegNum64(3, 4, 12, 8, 0), MISCREG_ICH_AP0R0_EL2 },
    { MiscRegNum64(3, 4, 12, 8, 1), MISCREG_ICH_AP0R1_EL2 },
    { MiscRegNum64(3, 4, 12, 8, 2), MISCREG_ICH_AP0R2_EL2 },
    { MiscRegNum64(3, 4, 12, 8, 3), MISCREG_ICH_AP0R3_EL2 },
    { MiscRegNum64(3, 4, 12, 9, 0), MISCREG_ICH_AP1R0_EL2 },
    { MiscRegNum64(3, 4, 12, 9, 1), MISCREG_ICH_AP1R1_EL2 },
    { MiscRegNum64(3, 4, 12, 9, 2), MISCREG_ICH_AP1R2_EL2 },
    { MiscRegNum64(3, 4, 12, 9, 3), MISCREG_ICH_AP1R3_EL2 },
    { MiscRegNum64(3, 4, 12, 9, 5), MISCREG_ICC_SRE_EL2 },
    { MiscRegNum64(3, 4, 12, 11, 0), MISCREG_ICH_HCR_EL2 },
    { MiscRegNum64(3, 4, 12, 11, 1), MISCREG_ICH_VTR_EL2 },
    { MiscRegNum64(3, 4, 12, 11, 2), MISCREG_ICH_MISR_EL2 },
    { MiscRegNum64(3, 4, 12, 11, 3), MISCREG_ICH_EISR_EL2 },
    { MiscRegNum64(3, 4, 12, 11, 5), MISCREG_ICH_ELRSR_EL2 },
    { MiscRegNum64(3, 4, 12, 11, 7), MISCREG_ICH_VMCR_EL2 },
    { MiscRegNum64(3, 4, 12, 12, 0), MISCREG_ICH_LR0_EL2 },
    { MiscRegNum64(3, 4, 12, 12, 1), MISCREG_ICH_LR1_EL2 },
    { MiscRegNum64(3, 4, 12, 12, 2), MISCREG_ICH_LR2_EL2 },
    { MiscRegNum64(3, 4, 12, 12, 3), MISCREG_ICH_LR3_EL2 },
    { MiscRegNum64(3, 4, 12, 12, 4), MISCREG_ICH_LR4_EL2 },
    { MiscRegNum64(3, 4, 12, 12, 5), MISCREG_ICH_LR5_EL2 },
    { MiscRegNum64(3, 4, 12, 12, 6), MISCREG_ICH_LR6_EL2 },
    { MiscRegNum64(3, 4, 12, 12, 7), MISCREG_ICH_LR7_EL2 },
    { MiscRegNum64(3, 4, 12, 13, 0), MISCREG_ICH_LR8_EL2 },
    { MiscRegNum64(3, 4, 12, 13, 1), MISCREG_ICH_LR9_EL2 },
    { MiscRegNum64(3, 4, 12, 13, 2), MISCREG_ICH_LR10_EL2 },
    { MiscRegNum64(3, 4, 12, 13, 3), MISCREG_ICH_LR11_EL2 },
    { MiscRegNum64(3, 4, 12, 13, 4), MISCREG_ICH_LR12_EL2 },
    { MiscRegNum64(3, 4, 12, 13, 5), MISCREG_ICH_LR13_EL2 },
    { MiscRegNum64(3, 4, 12, 13, 6), MISCREG_ICH_LR14_EL2 },
    { MiscRegNum64(3, 4, 12, 13, 7), MISCREG_ICH_LR15_EL2 },
    { MiscRegNum64(3, 4, 13, 0, 1), MISCREG_CONTEXTIDR_EL2 },
    { MiscRegNum64(3, 4, 13, 0, 2), MISCREG_TPIDR_EL2 },
    { MiscRegNum64(3, 4, 14, 0, 3), MISCREG_CNTVOFF_EL2 },
    { MiscRegNum64(3, 4, 14, 1, 0), MISCREG_CNTHCTL_EL2 },
    { MiscRegNum64(3, 4, 14, 2, 0), MISCREG_CNTHP_TVAL_EL2 },
    { MiscRegNum64(3, 4, 14, 2, 1), MISCREG_CNTHP_CTL_EL2 },
    { MiscRegNum64(3, 4, 14, 2, 2), MISCREG_CNTHP_CVAL_EL2 },
    { MiscRegNum64(3, 4, 14, 3, 0), MISCREG_CNTHV_TVAL_EL2 },
    { MiscRegNum64(3, 4, 14, 3, 1), MISCREG_CNTHV_CTL_EL2 },
    { MiscRegNum64(3, 4, 14, 3, 2), MISCREG_CNTHV_CVAL_EL2 },
    { MiscRegNum64(3, 4, 14, 4, 0), MISCREG_CNTHVS_TVAL_EL2 },
    { MiscRegNum64(3, 4, 14, 4, 1), MISCREG_CNTHVS_CTL_EL2 },
    { MiscRegNum64(3, 4, 14, 4, 2), MISCREG_CNTHVS_CVAL_EL2 },
    { MiscRegNum64(3, 4, 14, 5, 0), MISCREG_CNTHPS_TVAL_EL2 },
    { MiscRegNum64(3, 4, 14, 5, 1), MISCREG_CNTHPS_CTL_EL2 },
    { MiscRegNum64(3, 4, 14, 5, 2), MISCREG_CNTHPS_CVAL_EL2 },
    { MiscRegNum64(3, 5, 1, 0, 0), MISCREG_SCTLR_EL12 },
    { MiscRegNum64(3, 5, 1, 0, 2), MISCREG_CPACR_EL12 },
    { MiscRegNum64(3, 5, 1, 0, 3), MISCREG_SCTLR2_EL12 },
    { MiscRegNum64(3, 5, 1, 2, 0), MISCREG_ZCR_EL12 },
    { MiscRegNum64(3, 5, 1, 2, 6), MISCREG_SMCR_EL12 },
    { MiscRegNum64(3, 5, 2, 0, 0), MISCREG_TTBR0_EL12 },
    { MiscRegNum64(3, 5, 2, 0, 1), MISCREG_TTBR1_EL12 },
    { MiscRegNum64(3, 5, 2, 0, 2), MISCREG_TCR_EL12 },
    { MiscRegNum64(3, 5, 2, 0, 3), MISCREG_TCR2_EL12 },
    { MiscRegNum64(3, 5, 4, 0, 0), MISCREG_SPSR_EL12 },
    { MiscRegNum64(3, 5, 4, 0, 1), MISCREG_ELR_EL12 },
    { MiscRegNum64(3, 5, 5, 1, 0), MISCREG_AFSR0_EL12 },
    { MiscRegNum64(3, 5, 5, 1, 1), MISCREG_AFSR1_EL12 },
    { MiscRegNum64(3, 5, 5, 2, 0), MISCREG_ESR_EL12 },
    { MiscRegNum64(3, 5, 6, 0, 0), MISCREG_FAR_EL12 },
    { MiscRegNum64(3, 5, 10, 2, 0), MISCREG_MAIR_EL12 },
    { MiscRegNum64(3, 5, 10, 3, 0), MISCREG_AMAIR_EL12 },
    { MiscRegNum64(3, 5, 10, 5, 0), MISCREG_MPAM1_EL12 },
    { MiscRegNum64(3, 5, 12, 0, 0), MISCREG_VBAR_EL12 },
    { MiscRegNum64(3, 5, 13, 0, 1), MISCREG_CONTEXTIDR_EL12 },
    { MiscRegNum64(3, 5, 14, 1, 0), MISCREG_CNTKCTL_EL12 },
    { MiscRegNum64(3, 5, 14, 2, 0), MISCREG_CNTP_TVAL_EL02 },
    { MiscRegNum64(3, 5, 14, 2, 1), MISCREG_CNTP_CTL_EL02 },
    { MiscRegNum64(3, 5, 14, 2, 2), MISCREG_CNTP_CVAL_EL02 },
    { MiscRegNum64(3, 5, 14, 3, 0), MISCREG_CNTV_TVAL_EL02 },
    { MiscRegNum64(3, 5, 14, 3, 1), MISCREG_CNTV_CTL_EL02 },
    { MiscRegNum64(3, 5, 14, 3, 2), MISCREG_CNTV_CVAL_EL02 },
    { MiscRegNum64(3, 6, 1, 0, 0), MISCREG_SCTLR_EL3 },
    { MiscRegNum64(3, 6, 1, 0, 1), MISCREG_ACTLR_EL3 },
    { MiscRegNum64(3, 6, 1, 0, 3), MISCREG_SCTLR2_EL3 },
    { MiscRegNum64(3, 6, 1, 1, 0), MISCREG_SCR_EL3 },
    { MiscRegNum64(3, 6, 1, 1, 1), MISCREG_SDER32_EL3 },
    { MiscRegNum64(3, 6, 1, 1, 2), MISCREG_CPTR_EL3 },
    { MiscRegNum64(3, 6, 1, 2, 0), MISCREG_ZCR_EL3 },
    { MiscRegNum64(3, 6, 1, 2, 6), MISCREG_SMCR_EL3 },
    { MiscRegNum64(3, 6, 1, 3, 1), MISCREG_MDCR_EL3 },
    { MiscRegNum64(3, 6, 2, 0, 0), MISCREG_TTBR0_EL3 },
    { MiscRegNum64(3, 6, 2, 0, 2), MISCREG_TCR_EL3 },
    { MiscRegNum64(3, 6, 4, 0, 0), MISCREG_SPSR_EL3 },
    { MiscRegNum64(3, 6, 4, 0, 1), MISCREG_ELR_EL3 },
    { MiscRegNum64(3, 6, 4, 1, 0), MISCREG_SP_EL2 },
    { MiscRegNum64(3, 6, 5, 1, 0), MISCREG_AFSR0_EL3 },
    { MiscRegNum64(3, 6, 5, 1, 1), MISCREG_AFSR1_EL3 },
    { MiscRegNum64(3, 6, 5, 2, 0), MISCREG_ESR_EL3 },
    { MiscRegNum64(3, 6, 6, 0, 0), MISCREG_FAR_EL3 },
    { MiscRegNum64(3, 6, 10, 2, 0), MISCREG_MAIR_EL3 },
    { MiscRegNum64(3, 6, 10, 3, 0), MISCREG_AMAIR_EL3 },
    { MiscRegNum64(3, 6, 10, 5, 0), MISCREG_MPAM3_EL3 },
    { MiscRegNum64(3, 6, 12, 0, 0), MISCREG_VBAR_EL3 },
    { MiscRegNum64(3, 6, 12, 0, 1), MISCREG_RVBAR_EL3 },
    { MiscRegNum64(3, 6, 12, 0, 2), MISCREG_RMR_EL3 },
    { MiscRegNum64(3, 6, 12, 12, 4), MISCREG_ICC_CTLR_EL3 },
    { MiscRegNum64(3, 6, 12, 12, 5), MISCREG_ICC_SRE_EL3 },
    { MiscRegNum64(3, 6, 12, 12, 7), MISCREG_ICC_IGRPEN1_EL3 },
    { MiscRegNum64(3, 6, 13, 0, 2), MISCREG_TPIDR_EL3 },
    { MiscRegNum64(3, 7, 14, 2, 0), MISCREG_CNTPS_TVAL_EL1 },
    { MiscRegNum64(3, 7, 14, 2, 1), MISCREG_CNTPS_CTL_EL1 },
    { MiscRegNum64(3, 7, 14, 2, 2), MISCREG_CNTPS_CVAL_EL1 }
};

template <bool read>
HFGTR
fgtRegister(ThreadContext *tc)
{
    if constexpr (read) {
        return tc->readMiscReg(MISCREG_HFGRTR_EL2);
    } else {
        return tc->readMiscReg(MISCREG_HFGWTR_EL2);
    }
}

template <bool read>
HDFGTR
fgtDebugRegister(ThreadContext *tc)
{
    if constexpr (read) {
        return tc->readMiscReg(MISCREG_HDFGRTR_EL2);
    } else {
        return tc->readMiscReg(MISCREG_HDFGWTR_EL2);
    }
}

/**
 * Template helper for fine grained traps at EL0
 *
 * @tparam read: is this a read access to the register?
 * @tparam r_bitfield: register (HFGTR) bitfield
 */
template<bool read, auto r_bitfield>
Fault
faultFgtEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool in_host = EL2Enabled(tc) && hcr.e2h && hcr.tge;
    if (fgtEnabled(tc) && !in_host &&
        fgtRegister<read>(tc).*r_bitfield) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL1
 *
 * @tparam read: is this a read access to the register?
 * @tparam r_bitfield: register (HFGTR) bitfield
 */
template<bool read, auto r_bitfield>
Fault
faultFgtEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (fgtEnabled(tc) && fgtRegister<read>(tc).*r_bitfield) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL1
 *
 * @tparam r_bitfield: register (HFGITR) bitfield
 */
template<auto r_bitfield>
Fault
faultFgtInstEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (fgtEnabled(tc) &&
        static_cast<HFGITR>(tc->readMiscReg(MISCREG_HFGITR_EL2)).*r_bitfield) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL1
 * for TLBI *NXS
 *
 * @tparam r_bitfield: register (HFGITR) bitfield
 */
template<auto r_bitfield>
Fault
faultFgtTlbiNxsEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_HCX)) {
        const HCRX hcrx = tc->readMiscReg(MISCREG_HCRX_EL2);
        if (auto fault = faultFgtInstEL1<r_bitfield>(entry, tc, inst);
            fault != NoFault && (!isHcrxEL2Enabled(tc) || !hcrx.fgtnxs)) {
            return fault;
        } else {
            return NoFault;
        }
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL1
 *
 * @tparam read: is this a read access to the register?
 * @tparam r_bitfield: register (HFGTR) bitfield
 */
template<bool read, auto r_bitfield>
Fault
faultFgtDebugEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (fgtEnabled(tc) && fgtDebugRegister<read>(tc).*r_bitfield) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL1
 *
 * @tparam g_bitfield: group (HCR) bitfield
 */
template <auto g_bitfield>
Fault
faultHcrEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    if (EL2Enabled(tc) && hcr.*g_bitfield) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL0
 *
 * @tparam read: is this a read access to the register?
 * @tparam g_bitfield: group (HCR) bitfield
 * @tparam r_bitfield: register (HFGTR) bitfield
 */
template<bool read, auto g_bitfield, auto r_bitfield>
Fault
faultHcrFgtEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool in_host = EL2Enabled(tc) && hcr.e2h && hcr.tge;

    if (EL2Enabled(tc) && !in_host && hcr.*g_bitfield) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtEL0<read, r_bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL1
 *
 * @tparam read: is this a read access to the register?
 * @tparam g_bitfield: group (HCR) bitfield
 * @tparam r_bitfield: register (HFGTR) bitfield
 */
template<bool read, auto g_bitfield, auto r_bitfield>
Fault
faultHcrFgtEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

    if (EL2Enabled(tc) && hcr.*g_bitfield) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtEL1<read, r_bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL1
 *
 * @tparam g_bitfield: group (HCR) bitfield
 * @tparam r_bitfield: register (HFGITR) bitfield
 */
template<auto g_bitfield, auto r_bitfield>
Fault
faultHcrFgtInstEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

    if (EL2Enabled(tc) && hcr.*g_bitfield) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtInstEL1<r_bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

/**
 * Template helper for fine grained traps at EL1
 * for TLBI *NXS instructions
 *
 * @tparam g_bitfield: group (HCR) bitfield
 * @tparam r_bitfield: register (HFGITR) bitfield
 */
template<auto g_bitfield, auto r_bitfield>
Fault
faultTlbiNxsEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

    if (EL2Enabled(tc) && hcr.*g_bitfield) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtTlbiNxsEL1<r_bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

Fault
faultSpEL0(const MiscRegLUTEntry &entry, ThreadContext *tc,
           const MiscRegOp64 &inst)
{
    if (tc->readMiscReg(MISCREG_SPSEL) == 0)
        return inst.undefined();
    else
        return NoFault;
}

Fault
faultDaif(const MiscRegLUTEntry &entry, ThreadContext *tc,
          const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const SCTLR sctlr = tc->readMiscRegNoEffect(MISCREG_SCTLR_EL1);
    if ((el2_enabled && hcr.e2h && hcr.tge) || sctlr.uma == 0) {
        if (el2_enabled && hcr.tge) {
            return inst.generateTrap(EL2);
        } else {
            return inst.generateTrap(EL1);
        }
    } else {
        return NoFault;
    }
}

Fault
faultDczvaEL0(const MiscRegLUTEntry &entry, ThreadContext *tc,
              const MiscRegOp64 &inst)
{
    if (!FullSystem)
        return NoFault;

    const SCTLR sctlr = tc->readMiscRegNoEffect(MISCREG_SCTLR_EL1);
    const SCTLR sctlr2 = tc->readMiscRegNoEffect(MISCREG_SCTLR_EL2);
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);

    const bool el2_enabled = EL2Enabled(tc);
    const bool in_host = hcr.e2h && hcr.tge;
    if (!(el2_enabled && in_host) && !sctlr.dze) {
        if (el2_enabled && hcr.tge) {
            return inst.generateTrap(EL2);
        } else {
            return inst.generateTrap(EL1);
        }
    } else if (el2_enabled && !in_host && hcr.tdz) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && in_host && !sctlr2.dze) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultCvacEL0(const MiscRegLUTEntry &entry, ThreadContext *tc,
             const MiscRegOp64 &inst)
{
    const SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
    const SCTLR sctlr2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

    const bool el2_enabled = EL2Enabled(tc);
    const bool in_host = hcr.e2h && hcr.tge;
    if (!(el2_enabled && in_host) && !sctlr.uci) {
        if (el2_enabled && hcr.tge) {
            return inst.generateTrap(EL2);
        } else {
            return inst.generateTrap(EL1);
        }
    } else if (el2_enabled && !in_host && hcr.tpc) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && in_host && !sctlr2.uci) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultFpcrEL0(const MiscRegLUTEntry &entry, ThreadContext *tc,
               const MiscRegOp64 &inst)
{
    const CPACR cpacr = tc->readMiscReg(MISCREG_CPACR_EL1);
    const CPTR cptr_el2 = tc->readMiscReg(MISCREG_CPTR_EL2);
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    const bool in_host = hcr.e2h && hcr.tge;
    if (!(el2_enabled && in_host) && cpacr.fpen != 0b11) {
        if (el2_enabled && hcr.tge) {
            return inst.generateTrap(EL2, ExceptionClass::UNKNOWN, inst.iss());
        } else {
            return inst.generateTrap(EL1,
                ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
        }
    } else if (el2_enabled && in_host && cptr_el2.fpen != 0b11) {
        return inst.generateTrap(EL2,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else if (el2_enabled && hcr.e2h && ((cptr_el2.fpen & 0b1) == 0b0)) {
        return inst.generateTrap(EL2,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else if (el2_enabled && !hcr.e2h && cptr_el2.tfp) {
        return inst.generateTrap(EL2,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else if (ArmSystem::haveEL(tc, EL3) && cptr_el3.tfp) {
        return inst.generateTrap(EL3,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else {
        return NoFault;
    }
}

Fault
faultFpcrEL1(const MiscRegLUTEntry &entry, ThreadContext *tc,
             const MiscRegOp64 &inst)
{
    const CPACR cpacr = tc->readMiscReg(MISCREG_CPACR_EL1);
    const CPTR cptr_el2 = tc->readMiscReg(MISCREG_CPTR_EL2);
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if ((cpacr.fpen & 0b1) == 0b0) {
        return inst.generateTrap(EL1,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else if (el2_enabled && !hcr.e2h && cptr_el2.tfp) {
        return inst.generateTrap(EL2,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else if (el2_enabled && hcr.e2h && ((cptr_el2.fpen & 0b1) == 0b0)) {
        return inst.generateTrap(EL2,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else if (ArmSystem::haveEL(tc, EL3) && cptr_el3.tfp) {
        return inst.generateTrap(EL3,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else {
        return NoFault;
    }
}

Fault
faultFpcrEL2(const MiscRegLUTEntry &entry, ThreadContext *tc,
             const MiscRegOp64 &inst)
{
    const CPTR cptr_el2 = tc->readMiscReg(MISCREG_CPTR_EL2);
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    if (!hcr.e2h && cptr_el2.tfp) {
        return inst.generateTrap(EL2,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else if (hcr.e2h && ((cptr_el2.fpen & 0b1) == 0b0)) {
        return inst.generateTrap(EL2,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else if (ArmSystem::haveEL(tc, EL3) && cptr_el3.tfp) {
        return inst.generateTrap(EL3,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else {
        return NoFault;
    }
}

Fault
faultFpcrEL3(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);
    if (cptr_el3.tfp) {
        return inst.generateTrap(EL3,
            ExceptionClass::TRAPPED_SIMD_FP, 0x1E00000);
    } else {
        return NoFault;
    }
}

Fault
faultPouEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
    const SCTLR sctlr2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

    const bool el2_enabled = EL2Enabled(tc);
    const bool in_host = hcr.e2h && hcr.tge;
    if (!(el2_enabled && in_host) && !sctlr.uci) {
        if (el2_enabled && hcr.tge) {
            return inst.generateTrap(EL2);
        } else {
            return inst.generateTrap(EL1);
        }
    } else if (el2_enabled && !in_host && hcr.tpu) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && !in_host &&
               HaveExt(tc, ArmExtension::FEAT_EVT) && hcr.tocu) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && in_host && !sctlr2.uci) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

template <auto bitfield>
Fault
faultPouEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && hcr.tpu) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && HaveExt(tc, ArmExtension::FEAT_EVT) &&
               hcr.tocu) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtInstEL1<bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

template <auto bitfield>
Fault
faultPouIsEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && hcr.tpu) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && HaveExt(tc, ArmExtension::FEAT_EVT) &&
               hcr.ticab) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtInstEL1<bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

Fault
faultCtrEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
     const SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
     const SCTLR sctlr2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
     const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

     const bool el2_enabled = EL2Enabled(tc);
     const bool in_host = hcr.e2h && hcr.tge;
     if (!(el2_enabled && in_host) && !sctlr.uct) {
         if (el2_enabled && hcr.tge) {
            return inst.generateTrap(EL2);
         } else {
            return inst.generateTrap(EL1);
         }
     } else if (auto fault = faultHcrFgtEL0<
                    true, &HCR::tid2, &HFGTR::ctrEL0>(entry, tc, inst);
                fault != NoFault) {
        return fault;
     } else if (el2_enabled && in_host && !sctlr2.uct) {
        return inst.generateTrap(EL2);
     } else {
         return NoFault;
     }
}

Fault
faultMdccsrEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const DBGDS32 mdscr = tc->readMiscReg(MISCREG_MDSCR_EL1);
    const HDCR mdcr_el2 = tc->readMiscReg(MISCREG_MDCR_EL2);
    const HDCR mdcr_el3 = tc->readMiscReg(MISCREG_MDCR_EL3);

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (mdscr.tdcc) {
        if (el2_enabled && hcr.tge) {
            return inst.generateTrap(EL2);
        } else {
            return inst.generateTrap(EL1);
        }
    } else if (el2_enabled && mdcr_el2.tdcc) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && (hcr.tge || (mdcr_el2.tde || mdcr_el2.tda))) {
        return inst.generateTrap(EL2);
    } else if (ArmSystem::haveEL(tc, EL3) && (mdcr_el3.tdcc || mdcr_el3.tda)) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultMdccsrEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HDCR mdcr_el2 = tc->readMiscReg(MISCREG_MDCR_EL2);
    const HDCR mdcr_el3 = tc->readMiscReg(MISCREG_MDCR_EL3);

    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && mdcr_el2.tdcc) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && (mdcr_el2.tde || mdcr_el2.tda)) {
        return inst.generateTrap(EL2);
    } else if (ArmSystem::haveEL(tc, EL3) && (mdcr_el3.tdcc || mdcr_el3.tda)) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultMdccsrEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HDCR mdcr_el3 = tc->readMiscReg(MISCREG_MDCR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && (mdcr_el3.tdcc || mdcr_el3.tda)) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultDebugEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HDCR mdcr_el2 = tc->readMiscReg(MISCREG_MDCR_EL2);
    const HDCR mdcr_el3 = tc->readMiscReg(MISCREG_MDCR_EL3);

    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && (mdcr_el2.tde || mdcr_el2.tda)) {
        return inst.generateTrap(EL2);
    } else if (ArmSystem::haveEL(tc, EL3) && mdcr_el3.tda) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultDebugEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HDCR mdcr_el3 = tc->readMiscReg(MISCREG_MDCR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && mdcr_el3.tda) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

template<bool read, auto r_bitfield>
Fault
faultDebugWithFgtEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (auto fault = faultFgtDebugEL1<read, r_bitfield>(entry, tc, inst);
        fault != NoFault) {
        return fault;
    } else {
        return faultDebugEL1(entry, tc, inst);
    }
}

template<bool read, auto r_bitfield>
Fault
faultDebugOsEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HDCR mdcr_el2 = tc->readMiscReg(MISCREG_MDCR_EL2);
    const HDCR mdcr_el3 = tc->readMiscReg(MISCREG_MDCR_EL3);

    if (auto fault = faultFgtDebugEL1<read, r_bitfield>(entry, tc, inst);
        fault != NoFault) {
        return fault;
    } else if (EL2Enabled(tc) && (mdcr_el2.tde || mdcr_el2.tdosa)) {
        return inst.generateTrap(EL2);
    } else if (ArmSystem::haveEL(tc, EL3) && mdcr_el3.tdosa) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultDebugOsEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HDCR mdcr_el3 = tc->readMiscReg(MISCREG_MDCR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && mdcr_el3.tdosa) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultHcrxEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && !scr.hxen) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultZcrEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CPACR cpacr_el1 = tc->readMiscReg(MISCREG_CPACR_EL1);
    const CPTR cptr_el2 = tc->readMiscReg(MISCREG_CPTR_EL2);
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (!(cpacr_el1.zen & 0x1)) {
        return inst.generateTrap(EL1, ExceptionClass::TRAPPED_SVE, 0);
    } else if (el2_enabled && !hcr.e2h && cptr_el2.tz) {
        return inst.generateTrap(EL2, ExceptionClass::TRAPPED_SVE, 0);
    } else if (el2_enabled && hcr.e2h && !(cptr_el2.zen & 0x1)) {
        return inst.generateTrap(EL2, ExceptionClass::TRAPPED_SVE, 0);
    } else if (ArmSystem::haveEL(tc, EL3) && !cptr_el3.ez) {
        return inst.generateTrap(EL3, ExceptionClass::TRAPPED_SVE, 0);
    } else {
        return NoFault;
    }
}

Fault
faultZcrEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CPTR cptr_el2 = tc->readMiscReg(MISCREG_CPTR_EL2);
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    if (!hcr.e2h && cptr_el2.tz) {
        return inst.generateTrap(EL2, ExceptionClass::TRAPPED_SVE, 0);
    } else if (hcr.e2h && !(cptr_el2.zen & 0x1)) {
        return inst.generateTrap(EL2, ExceptionClass::TRAPPED_SVE, 0);
    } else if (ArmSystem::haveEL(tc, EL3) && !cptr_el3.ez) {
        return inst.generateTrap(EL3, ExceptionClass::TRAPPED_SVE, 0);
    } else {
        return NoFault;
    }
}

Fault
faultZcrEL3(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);
    if (!cptr_el3.ez) {
        return inst.generateTrap(EL3, ExceptionClass::TRAPPED_SVE, 0);
    } else {
        return NoFault;
    }
}

Fault
faultGicv3(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    auto gic = static_cast<ArmSystem*>(tc->getSystemPtr())->getGIC();
    if (!gic->supportsVersion(BaseGic::GicVersion::GIC_V3)) {
        return inst.undefined();
    } else {
        return NoFault;
    }
}

Fault
faultIccSgiEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (auto fault = faultGicv3(entry, tc, inst); fault != NoFault) {
        return fault;
    }

    const Gicv3CPUInterface::ICH_HCR_EL2 ich_hcr =
        tc->readMiscReg(MISCREG_ICH_HCR_EL2);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    if (EL2Enabled(tc) && (hcr.fmo || hcr.imo || ich_hcr.TC)) {
        return inst.generateTrap(EL2);
    } else if (ArmSystem::haveEL(tc, EL3) && scr.irq && scr.fiq) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultIccSgiEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (auto fault = faultGicv3(entry, tc, inst); fault != NoFault) {
        return fault;
    }

    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && scr.irq && scr.fiq) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

template<bool read, auto g_bitfield>
Fault
faultSctlr2EL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_SCTLR2)) {
        const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        const HCRX hcrx = tc->readMiscReg(MISCREG_HCRX_EL2);
        if (
                auto fault = faultHcrFgtEL1<read, g_bitfield, &HFGTR::sctlrEL1>
                (
                    entry,
                    tc,
                    inst
                );
                fault != NoFault
        ) {
            return fault;
        } else if (
                    EL2Enabled(tc) && (!isHcrxEL2Enabled(tc) || !hcrx.sctlr2En)
                  ) {
            return inst.generateTrap(EL2);
        } else if (ArmSystem::haveEL(tc, EL3) && !scr.sctlr2En) {
            return inst.generateTrap(EL3);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultSctlr2EL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_SCTLR2)) {
        const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        if (ArmSystem::haveEL(tc, EL3) && !scr.sctlr2En) {
            return inst.generateTrap(EL3);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultSctlr2VheEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_SCTLR2)) {
        const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
        const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        if (hcr.e2h) {
            if (ArmSystem::haveEL(tc, EL3) && !scr.sctlr2En) {
                return inst.generateTrap(EL3);
            } else {
                return NoFault;
            }
        } else {
            return inst.undefined();
        }
    } else {
        return inst.undefined();
    }
}

template<bool read, auto g_bitfield>
Fault
faultTcr2EL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_TCR2)) {
        const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        const HCRX hcrx = tc->readMiscReg(MISCREG_HCRX_EL2);
        if (
                auto fault = faultHcrFgtEL1<read, g_bitfield, &HFGTR::sctlrEL1>
                (
                    entry,
                    tc,
                    inst
                );
                fault != NoFault
        ) {
            return fault;
        } else if (EL2Enabled(tc) && (!isHcrxEL2Enabled(tc) || !hcrx.tcr2En)) {
            return inst.generateTrap(EL2);
        } else if (ArmSystem::haveEL(tc, EL3) && !scr.tcr2En) {
            return inst.generateTrap(EL3);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultTcr2EL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_TCR2)) {
        const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        if (ArmSystem::haveEL(tc, EL3) && !scr.tcr2En) {
            return inst.generateTrap(EL3);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultTcr2VheEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_TCR2)) {
        const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
        const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        if (hcr.e2h) {
            if (ArmSystem::haveEL(tc, EL3) && !scr.tcr2En) {
                return inst.generateTrap(EL3);
            } else {
                return NoFault;
            }
        } else {
            return inst.undefined();
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultTcr2VheEL3(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_TCR2)) {
        const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
        const bool el2_host = EL2Enabled(tc) && hcr.e2h;
        if (el2_host) {
            return NoFault;
        } else {
            return inst.undefined();
        }
    } else {
        return inst.undefined();
    }
}

template<bool read, auto r_bitfield>
Fault
faultCpacrEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CPTR cptr_el2 = tc->readMiscReg(MISCREG_CPTR_EL2);
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);

    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && cptr_el2.tcpac) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtEL1<read, r_bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else if (ArmSystem::haveEL(tc, EL3) && cptr_el3.tcpac) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultCpacrEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && cptr_el3.tcpac) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultCpacrVheEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    if (hcr.e2h) {
        return faultCpacrEL2(entry, tc, inst);
    } else {
        return inst.undefined();
    }
}

template <auto bitfield>
Fault
faultTlbiOsEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && hcr.ttlb) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && HaveExt(tc, ArmExtension::FEAT_EVT) &&
               hcr.ttlbos) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtInstEL1<bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

template <auto bitfield>
Fault
faultTlbiOsNxsEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && hcr.ttlb) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && HaveExt(tc, ArmExtension::FEAT_EVT) &&
               hcr.ttlbos) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtTlbiNxsEL1<bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

template <auto bitfield>
Fault
faultTlbiIsEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && hcr.ttlb) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && HaveExt(tc, ArmExtension::FEAT_EVT) &&
               hcr.ttlbis) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtInstEL1<bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

template <auto bitfield>
Fault
faultTlbiIsNxsEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && hcr.ttlb) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && HaveExt(tc, ArmExtension::FEAT_EVT) &&
               hcr.ttlbis) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtTlbiNxsEL1<bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

template <bool read, auto r_bitfield>
Fault
faultCacheEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && hcr.tid2) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && HaveExt(tc, ArmExtension::FEAT_EVT) &&
               hcr.tid4) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtEL1<read, r_bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else {
        return NoFault;
    }
}

template <bool read, auto r_bitfield>
Fault
faultPauthEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    const bool el2_enabled = EL2Enabled(tc);

    if (el2_enabled && !hcr.apk) {
        return inst.generateTrap(EL2);
    } else if (auto fault = faultFgtEL1<read, r_bitfield>(entry, tc, inst);
               fault != NoFault) {
        return fault;
    } else if (ArmSystem::haveEL(tc, EL3) && !scr.apk) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultPauthEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && !scr.apk) {
        return inst.generateTrap(EL3);
    } else {
        return NoFault;
    }
}

Fault
faultGenericTimerEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool in_host = el2_enabled && hcr.e2h && hcr.tge;
    const CNTKCTL cntkctl_el1 = tc->readMiscReg(MISCREG_CNTKCTL_EL1);
    const CNTHCTL_E2H cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (!(in_host) && !cntkctl_el1.el0pcten && !cntkctl_el1.el0vcten) {
        if (el2_enabled && hcr.tge)
            return inst.generateTrap(EL2);
        else
            return inst.generateTrap(EL1);
    } else if (in_host && !cnthctl_el2.el0pcten && !cnthctl_el2.el0vcten) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultCntpctEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool in_host = el2_enabled && hcr.e2h && hcr.tge;
    const CNTKCTL cntkctl_el1 = tc->readMiscReg(MISCREG_CNTKCTL_EL1);
    const RegVal cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (!(in_host) && !cntkctl_el1.el0pcten) {
        if (el2_enabled && hcr.tge)
            return inst.generateTrap(EL2);
        else
            return inst.generateTrap(EL1);
    } else if (el2_enabled && !hcr.e2h &&
               !static_cast<CNTHCTL>(cnthctl_el2).el1pcten) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && hcr.e2h && !hcr.tge &&
               !static_cast<CNTHCTL_E2H>(cnthctl_el2).el1pcten) {
        return inst.generateTrap(EL2);
    } else if (in_host &&
               !static_cast<CNTHCTL_E2H>(cnthctl_el2).el0pcten) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultCntpctEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const RegVal cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (el2_enabled && hcr.e2h &&
        !static_cast<CNTHCTL_E2H>(cnthctl_el2).el1pcten) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && !hcr.e2h &&
               !static_cast<CNTHCTL>(cnthctl_el2).el1pcten) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultCntvctEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool in_host = el2_enabled && hcr.e2h && hcr.tge;
    const CNTKCTL cntkctl_el1 = tc->readMiscReg(MISCREG_CNTKCTL_EL1);
    const CNTHCTL_E2H cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (!(in_host) && !cntkctl_el1.el0vcten) {
        if (el2_enabled && hcr.tge)
            return inst.generateTrap(EL2);
        else
            return inst.generateTrap(EL1);
    } else if (in_host && !cnthctl_el2.el0vcten) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && !(hcr.e2h && hcr.tge) && cnthctl_el2.el1tvct) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultCntvctEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CNTHCTL cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (EL2Enabled(tc) && cnthctl_el2.el1tvct) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

//TODO: See faultCntpctEL0
Fault
faultCntpCtlEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool in_host = el2_enabled && hcr.e2h && hcr.tge;
    const CNTKCTL cntkctl_el1 = tc->readMiscReg(MISCREG_CNTKCTL_EL1);
    const RegVal cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (!(in_host) && !cntkctl_el1.el0pten) {
        if (el2_enabled && hcr.tge)
            return inst.generateTrap(EL2);
        else
            return inst.generateTrap(EL1);
    } else if (el2_enabled && !hcr.e2h &&
               !static_cast<CNTHCTL>(cnthctl_el2).el1pcen) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && hcr.e2h && !hcr.tge &&
               !static_cast<CNTHCTL_E2H>(cnthctl_el2).el1pten) {
        return inst.generateTrap(EL2);
    } else if (in_host &&
               !static_cast<CNTHCTL_E2H>(cnthctl_el2).el0pten) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultCntpCtlEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const RegVal cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (el2_enabled && !hcr.e2h &&
        !static_cast<CNTHCTL>(cnthctl_el2).el1pcen) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && hcr.e2h &&
               !static_cast<CNTHCTL_E2H>(cnthctl_el2).el1pten) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

//  TODO: see faultCntvctEL0
Fault
faultCntvCtlEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const bool in_host = el2_enabled && hcr.e2h && hcr.tge;
    const CNTKCTL cntkctl_el1 = tc->readMiscReg(MISCREG_CNTKCTL_EL1);
    const CNTHCTL_E2H cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (!(in_host) && !cntkctl_el1.el0vten) {
        if (el2_enabled && hcr.tge)
            return inst.generateTrap(EL2);
        else
            return inst.generateTrap(EL1);
    } else if (in_host && !cnthctl_el2.el0vten) {
        return inst.generateTrap(EL2);
    } else if (el2_enabled && !(hcr.e2h && hcr.tge) && cnthctl_el2.el1tvt) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultCntvCtlEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CNTHCTL cnthctl_el2 = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    if (EL2Enabled(tc) && cnthctl_el2.el1tvt) {
        return inst.generateTrap(EL2);
    } else {
        return NoFault;
    }
}

Fault
faultCntpsCtlEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && !scr.ns) {
        if (scr.eel2)
            return inst.undefined();
        else if (!scr.st)
            return inst.generateTrap(EL3);
        else
            return NoFault;
    } else {
        return inst.undefined();
    }
}

Fault
faultUnimplemented(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (entry.info[MISCREG_WARN_NOT_FAIL]) {
        return NoFault;
    } else {
        return inst.undefined();
    }
}

Fault
faultImpdefUnimplEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    if (EL2Enabled(tc) && hcr.tidcp) {
        return inst.generateTrap(EL2);
    } else {
        return faultUnimplemented(entry, tc, inst);
    }
}

Fault
faultEsm(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CPTR cptr_el3 = tc->readMiscReg(MISCREG_CPTR_EL3);
    if (ArmSystem::haveEL(tc, EL3) && !cptr_el3.esm) {
        return inst.generateTrap(EL3, ExceptionClass::TRAPPED_SME, 0);
    } else {
        return NoFault;
    }
}

Fault
faultTsmSmen(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr_el2 = tc->readMiscReg(MISCREG_HCR_EL2);
    const CPTR cptr_el2 = tc->readMiscReg(MISCREG_CPTR_EL2);
    const bool el2_enabled = EL2Enabled(tc);
    if (el2_enabled && !hcr_el2.e2h && cptr_el2.tsm) {
        return inst.generateTrap(EL2, ExceptionClass::TRAPPED_SME, 0);
    } else if (el2_enabled && hcr_el2.e2h && !(cptr_el2.smen & 0b1)) {
        return inst.generateTrap(EL2, ExceptionClass::TRAPPED_SME, 0);
    } else {
        return faultEsm(entry, tc, inst);
    }
}

Fault
faultSmenEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const CPACR cpacr = tc->readMiscReg(MISCREG_CPACR_EL1);
    if (!(cpacr.smen & 0b1)) {
        return inst.generateTrap(EL1, ExceptionClass::TRAPPED_SME, 0);
    } else {
        return faultTsmSmen(entry, tc, inst);
    }
}

Fault
faultSmenEL0(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const bool el2_enabled = EL2Enabled(tc);
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const bool in_host = hcr.e2h && hcr.tge;

    const CPACR cpacr = tc->readMiscReg(MISCREG_CPACR_EL1);
    const CPTR cptr_el2 = tc->readMiscReg(MISCREG_CPTR_EL2);
    if (!(el2_enabled && in_host) && cpacr.smen != 0b11) {
        if (el2_enabled && hcr.tge)
            return inst.generateTrap(EL2, ExceptionClass::TRAPPED_SME, 0);
        else
            return inst.generateTrap(EL1, ExceptionClass::TRAPPED_SME, 0);
    } else if (el2_enabled && in_host && cptr_el2.smen != 0b11) {
        return inst.generateTrap(EL2, ExceptionClass::TRAPPED_SME, 0);
    } else {
        return faultTsmSmen(entry, tc, inst);
    }
}

Fault
faultRng(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    if (HaveExt(tc, ArmExtension::FEAT_RNG_TRAP) && scr.trndr) {
        return inst.generateTrap(EL3);
    } else if (!HaveExt(tc, ArmExtension::FEAT_RNG)) {
        return inst.undefined();
    } else {
        return NoFault;
    }
}

Fault
faultFgtCtrlRegs(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_FGT)) {
        const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        if (ArmSystem::haveEL(tc, EL3) && !scr.fgten) {
            return inst.generateTrap(EL3);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultIdst(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_IDST)) {
        const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        if (EL2Enabled(tc) && hcr.tge) {
            return inst.generateTrap(EL2);
        } else {
            return inst.generateTrap(EL1);
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultMpamIdrEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_MPAM)) {
        MPAM mpam2 = tc->readMiscReg(MISCREG_MPAM2_EL2);
        MPAM mpam3 = tc->readMiscReg(MISCREG_MPAM3_EL3);
        MPAMIDR mpamidr = tc->readMiscReg(MISCREG_MPAMIDR_EL1);
        MPAMHCR mpamhcr = tc->readMiscReg(MISCREG_MPAMHCR_EL2);
        if (ArmSystem::haveEL(tc, EL3) && mpam3.el3.trapLower) {
            return inst.generateTrap(EL3);
        } else if (EL2Enabled(tc) && mpamidr.hasHcr && mpamhcr.trapMpamIdrEL1) {
            return inst.generateTrap(EL2);
        } else if (EL2Enabled(tc) && mpamidr.hasTidr && mpam2.el2.tidr) {
            return inst.generateTrap(EL2);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultMpam0EL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_MPAM)) {
        MPAM mpam2 = tc->readMiscReg(MISCREG_MPAM2_EL2);
        MPAM mpam3 = tc->readMiscReg(MISCREG_MPAM3_EL3);
        if (ArmSystem::haveEL(tc, EL3) && mpam3.el3.trapLower) {
            return inst.generateTrap(EL3);
        } else if (EL2Enabled(tc) && mpam2.el2.trapMpam0EL1) {
            return inst.generateTrap(EL2);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultMpam1EL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_MPAM)) {
        MPAM mpam2 = tc->readMiscReg(MISCREG_MPAM2_EL2);
        MPAM mpam3 = tc->readMiscReg(MISCREG_MPAM3_EL3);
        if (ArmSystem::haveEL(tc, EL3) && mpam3.el3.trapLower) {
            return inst.generateTrap(EL3);
        } else if (EL2Enabled(tc) && mpam2.el2.trapMpam1EL1) {
            return inst.generateTrap(EL2);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultMpamEL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_MPAM)) {
        MPAM mpam3 = tc->readMiscReg(MISCREG_MPAM3_EL3);
        if (ArmSystem::haveEL(tc, EL3) && mpam3.el3.trapLower) {
            return inst.generateTrap(EL3);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

Fault
faultMpam12EL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (ELIsInHost(tc, EL2)) {
        return faultMpamEL2(entry, tc, inst);
    } else {
        return inst.undefined();
    }
}

Fault
faultMpamsmEL1(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (HaveExt(tc, ArmExtension::FEAT_MPAM)) {
        MPAM mpam2 = tc->readMiscReg(MISCREG_MPAM2_EL2);
        MPAM mpam3 = tc->readMiscReg(MISCREG_MPAM3_EL3);
        if (ArmSystem::haveEL(tc, EL3) && mpam3.el3.trapLower) {
            return inst.generateTrap(EL3);
        } else if (EL2Enabled(tc) && mpam2.el2.enMpamSm) {
            return inst.generateTrap(EL2);
        } else {
            return NoFault;
        }
    } else {
        return inst.undefined();
    }
}

}

MiscRegIndex
decodeAArch64SysReg(unsigned op0, unsigned op1,
                    unsigned crn, unsigned crm,
                    unsigned op2)
{
    MiscRegNum64 sys_reg(op0, op1, crn, crm, op2);
    return decodeAArch64SysReg(sys_reg);
}

MiscRegIndex
decodeAArch64SysReg(const MiscRegNum64 &sys_reg)
{
    auto it = miscRegNumToIdx.find(sys_reg);
    if (it != miscRegNumToIdx.end()) {
        return it->second;
    } else {
        // Check for a pseudo register before returning MISCREG_UNKNOWN
        if ((sys_reg.op0 == 1 || sys_reg.op0 == 3) &&
            (sys_reg.crn == 11 || sys_reg.crn == 15)) {
            return MISCREG_IMPDEF_UNIMPL;
        } else {
            return MISCREG_UNKNOWN;
        }
    }
}

std::optional<MiscRegNum64>
encodeAArch64SysReg(MiscRegIndex misc_reg)
{
    if (auto it = idxToMiscRegNum.find(misc_reg);
        it != idxToMiscRegNum.end()) {
        return it->second;
    } else {
        return std::nullopt;
    }
}

Fault
MiscRegLUTEntry::checkFault(ThreadContext *tc,
                            const MiscRegOp64 &inst, ExceptionLevel el)
{
    return !inst.miscRead() ? faultWrite[el](*this, tc, inst) :
                              faultRead[el](*this, tc, inst);
}

template <MiscRegInfo Sec, MiscRegInfo NonSec>
Fault
MiscRegLUTEntry::defaultFault(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    if (isSecureBelowEL3(tc) ? entry.info[Sec] : entry.info[NonSec]) {
        return NoFault;
    } else {
        return inst.undefined();
    }
}

static Fault
defaultFaultE2H_EL2(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    if (hcr.e2h) {
        return NoFault;
    } else {
        return inst.undefined();
    }
}

static Fault
defaultFaultE2H_EL3(const MiscRegLUTEntry &entry,
    ThreadContext *tc, const MiscRegOp64 &inst)
{
    const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const bool el2_host = EL2Enabled(tc) && hcr.e2h;
    if (el2_host) {
        return NoFault;
    } else {
        return inst.undefined();
    }
}

MiscRegLUTEntryInitializer::chain
MiscRegLUTEntryInitializer::highest(ArmSystem *const sys) const
{
    switch (FullSystem ? sys->highestEL() : EL1) {
      case EL0:
      case EL1: priv(); break;
      case EL2: hyp(); break;
      case EL3: mon(); break;
    }
    return *this;
}

static CPSR
resetCPSR(ArmSystem *system)
{
    CPSR cpsr = 0;
    if (!FullSystem) {
        cpsr.mode = MODE_USER;
    } else {
        switch (system->highestEL()) {
            // Set initial EL to highest implemented EL using associated stack
            // pointer (SP_ELx); set RVBAR_ELx to implementation defined reset
            // value
          case EL3:
            cpsr.mode = MODE_EL3H;
            break;
          case EL2:
            cpsr.mode = MODE_EL2H;
            break;
          case EL1:
            cpsr.mode = MODE_EL1H;
            break;
          default:
            panic("Invalid highest implemented exception level");
            break;
        }

        // Initialize rest of CPSR
        cpsr.daif = 0xf;  // Mask all interrupts
        cpsr.ss = 0;
        cpsr.il = 0;
    }
    return cpsr;
}

void
ISA::initializeMiscRegMetadata()
{
    // the MiscReg metadata tables are shared across all instances of the
    // ISA object, so there's no need to initialize them multiple times.
    static bool completed = false;
    if (completed)
        return;

    // This boolean variable specifies if the system is running in aarch32 at
    // EL3 (aarch32EL3 = true). It is false if EL3 is not implemented, or it
    // is running in aarch64 (aarch32EL3 = false)
    bool aarch32EL3 = release->has(ArmExtension::SECURITY) && !highestELIs64;

    // Set Privileged Access Never on taking an exception to EL1 (Arm 8.1+),
    // unsupported
    bool SPAN = false;

    // Implicit error synchronization event enable (Arm 8.2+), unsupported
    bool IESB = false;

    // Load Multiple and Store Multiple Atomicity and Ordering (Arm 8.2+),
    // unsupported
    bool LSMAOE = false;

    // No Trap Load Multiple and Store Multiple (Arm 8.2+), unsupported
    bool nTLSMD = false;

    // Pointer authentication (Arm 8.3+), unsupported
    bool EnDA = true; // using APDAKey_EL1 key of instr addrs in ELs 0,1
    bool EnDB = true; // using APDBKey_EL1 key of instr addrs in ELs 0,1
    bool EnIA = true; // using APIAKey_EL1 key of instr addrs in ELs 0,1
    bool EnIB = true; // using APIBKey_EL1 key of instr addrs in ELs 0,1

    const bool vhe_implemented = release->has(ArmExtension::FEAT_VHE);
    const bool sel2_implemented = release->has(ArmExtension::FEAT_SEL2);

    const Params &p(params());

    uint32_t midr;
    if (p.midr != 0x0)
        midr = p.midr;
    else if (highestELIs64)
        // Cortex-A57 TRM r0p0 MIDR
        midr = 0x410fd070;
    else
        // Cortex-A15 TRM r0p0 MIDR
        midr = 0x410fc0f0;

    /**
     * Some registers alias with others, and therefore need to be translated.
     * When two mapping registers are given, they are the 32b lower and
     * upper halves, respectively, of the 64b register being mapped.
     * aligned with reference documentation ARM DDI 0487A.i pp 1540-1543
     *
     * NAM = "not architecturally mandated",
     * from ARM DDI 0487A.i, template text
     * "AArch64 System register ___ can be mapped to
     *  AArch32 System register ___, but this is not
     *  architecturally mandated."
     */

    InitReg(MISCREG_CPSR)
      .reset(resetCPSR(system))
      .allPrivileges();
    InitReg(MISCREG_SPSR)
      .allPrivileges();
    InitReg(MISCREG_SPSR_FIQ)
      .allPrivileges();
    InitReg(MISCREG_SPSR_IRQ)
      .allPrivileges();
    InitReg(MISCREG_SPSR_SVC)
      .allPrivileges();
    InitReg(MISCREG_SPSR_MON)
      .allPrivileges();
    InitReg(MISCREG_SPSR_ABT)
      .allPrivileges();
    InitReg(MISCREG_SPSR_HYP)
      .allPrivileges();
    InitReg(MISCREG_SPSR_UND)
      .allPrivileges();
    InitReg(MISCREG_ELR_HYP)
      .allPrivileges();
    InitReg(MISCREG_FPSID)
      .reset(p.fpsid)
      .allPrivileges();
    InitReg(MISCREG_FPSCR)
      .res0(mask(14, 13) | mask(6, 5))
      .allPrivileges();
    InitReg(MISCREG_MVFR1)
      .reset([] () {
        MVFR1 mvfr1 = 0;
        mvfr1.flushToZero = 1;
        mvfr1.defaultNaN = 1;
        mvfr1.advSimdLoadStore = 1;
        mvfr1.advSimdInteger = 1;
        mvfr1.advSimdSinglePrecision = 1;
        mvfr1.advSimdHalfPrecision = 1;
        mvfr1.vfpHalfPrecision = 1;
        return mvfr1;
      }())
      .allPrivileges();
    InitReg(MISCREG_MVFR0)
      .reset([] () {
        MVFR0 mvfr0 = 0;
        mvfr0.advSimdRegisters = 2;
        mvfr0.singlePrecision = 2;
        mvfr0.doublePrecision = 2;
        mvfr0.vfpExceptionTrapping = 0;
        mvfr0.divide = 1;
        mvfr0.squareRoot = 1;
        mvfr0.shortVectors = 1;
        mvfr0.roundingModes = 1;
        return mvfr0;
      }())
      .allPrivileges();
    InitReg(MISCREG_FPEXC)
      .allPrivileges();

    // Helper registers
    InitReg(MISCREG_CPSR_MODE)
      .allPrivileges();
    InitReg(MISCREG_CPSR_Q)
      .allPrivileges();
    InitReg(MISCREG_FPSCR_EXC)
      .allPrivileges();
    InitReg(MISCREG_FPSCR_QC)
      .allPrivileges();
    InitReg(MISCREG_LOCKADDR)
      .allPrivileges();
    InitReg(MISCREG_LOCKFLAG)
      .allPrivileges();
    InitReg(MISCREG_PRRR_MAIR0)
      .mutex()
      .banked();
    InitReg(MISCREG_PRRR_MAIR0_NS)
      .mutex()
      .privSecure(!aarch32EL3)
      .bankedChild();
    InitReg(MISCREG_PRRR_MAIR0_S)
      .mutex()
      .bankedChild();
    InitReg(MISCREG_NMRR_MAIR1)
      .mutex()
      .banked();
    InitReg(MISCREG_NMRR_MAIR1_NS)
      .mutex()
      .privSecure(!aarch32EL3)
      .bankedChild();
    InitReg(MISCREG_NMRR_MAIR1_S)
      .mutex()
      .bankedChild();
    InitReg(MISCREG_PMXEVTYPER_PMCCFILTR)
      .mutex();
    InitReg(MISCREG_SEV_MAILBOX)
      .reset(1) // Start with an event in the mailbox
      .allPrivileges();
    InitReg(MISCREG_TLBINEEDSYNC)
      .allPrivileges().exceptUserMode();

    // AArch32 CP14 registers
    InitReg(MISCREG_DBGDIDR)
      .reset(0x6 << 16) // Armv8 Debug architecture
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDSCRint)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDCCINT)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDTRTXint)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDTRRXint)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWFAR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGVCR)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGDTRRXext)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDSCRext)
      .allPrivileges();
    InitReg(MISCREG_DBGDTRTXext)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGOSECCR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBVR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGDRAR)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGBXVR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGOSLAR)
       .allPrivileges().monSecureRead(0).monNonSecureRead(0);
    InitReg(MISCREG_DBGOSLSR)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGOSDLR)
      .unimplemented()
      .warnNotFail()
      .allPrivileges();
    InitReg(MISCREG_DBGPRCR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDSAR)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGCLAIMSET)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGCLAIMCLR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGAUTHSTATUS)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDEVID2)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDEVID1)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDEVID0)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_TEECR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_JIDR)
      .raz() // Jazelle trivial implementation, RAZ/WI
      .allPrivileges();
    InitReg(MISCREG_TEEHBR)
      .allPrivileges();
    InitReg(MISCREG_JOSCR)
      .raz() // Jazelle trivial implementation, RAZ/WI
      .allPrivileges();
    InitReg(MISCREG_JMCR)
      .raz() // Jazelle trivial implementation, RAZ/WI
      .allPrivileges();

    // AArch32 CP15 registers
    InitReg(MISCREG_MIDR)
      .reset(midr)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CTR)
      .reset([system=p.system](){
          //all caches have the same line size in gem5
          //4 byte words in ARM
          unsigned line_size_words =
              system->cacheLineSize() / 4;
          unsigned log2_line_size_words = 0;

          while (line_size_words >>= 1) {
              ++log2_line_size_words;
          }

          CTR ctr = 0;
          //log2 of minimun i-cache line size (words)
          ctr.iCacheLineSize = log2_line_size_words;
          //b11 - gem5 uses pipt
          ctr.l1IndexPolicy = 0x3;
          //log2 of minimum d-cache line size (words)
          ctr.dCacheLineSize = log2_line_size_words;
          //log2 of max reservation size (words)
          ctr.erg = log2_line_size_words;
          //log2 of max writeback size (words)
          ctr.cwg = log2_line_size_words;
          //b100 - gem5 format is ARMv7
          ctr.format = 0x4;

          return ctr;
      }())
      .unserialize(0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_TCMTR)
      .raz() // No TCM's
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_TLBTR)
      .reset(1) // Separate Instruction and Data TLBs
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_MPIDR)
      .reset(0x80000000)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_REVIDR)
      .unimplemented()
      .warnNotFail()
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_PFR0)
      .reset(0x00000031) // !ThumbEE | !Jazelle | Thumb | ARM
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_PFR1)
      .reset([release=release,system=system](){
          // Timer | Virti | !M Profile | TrustZone | ARMv4
          bool have_timer = (system && system->getGenericTimer() != nullptr);
          return 0x00000001 |
              (release->has(ArmExtension::SECURITY) ?
                  0x00000010 : 0x0) |
              (release->has(ArmExtension::VIRTUALIZATION) ?
                  0x00001000 : 0x0) |
              (have_timer ? 0x00010000 : 0x0);
      }())
      .unserialize(0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_DFR0)
      .reset(p.pmu ? 0x03000000 : 0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AFR0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR0)
      .reset([p,release=release](){
          RegVal mmfr0 = p.id_mmfr0;
          if (release->has(ArmExtension::LPAE))
              mmfr0 = (mmfr0 & ~0xf) | 0x5;
          return mmfr0;
      }())
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR1)
      .reset(p.id_mmfr1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR2)
      .reset(p.id_mmfr2)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR3)
      .reset(p.id_mmfr3)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR4)
      .reset(p.id_mmfr4)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR0)
      .reset(p.id_isar0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR1)
      .reset(p.id_isar1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR2)
      .reset(p.id_isar2)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR3)
      .reset(p.id_isar3)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR4)
      .reset(p.id_isar4)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR5)
      .reset([p,release=release] () {
        ISAR5 isar5 = p.id_isar5;
        isar5.crc32 = release->has(ArmExtension::FEAT_CRC32) ? 0x1 : 0x0;
        isar5.sha2 = release->has(ArmExtension::FEAT_SHA256) ? 0x1 : 0x0;
        isar5.sha1 = release->has(ArmExtension::FEAT_SHA1) ? 0x1 : 0x0;
        isar5.aes = release->has(ArmExtension::FEAT_PMULL) ?
            0x2 : release->has(ArmExtension::FEAT_AES) ?
                0x1 : 0x0;
        isar5.rdm = release->has(ArmExtension::FEAT_RDM) ? 0x1 : 0x0;
        isar5.vcma = release->has(ArmExtension::FEAT_FCMA) ? 0x1 : 0x0;
        return isar5;
      }())
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR6)
      .reset([p,release=release] () {
        ISAR6 isar6 = p.id_isar6;
        isar6.jscvt = release->has(ArmExtension::FEAT_JSCVT) ? 0x1 : 0x0;
        return isar6;
      }())
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CCSIDR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CLIDR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_AIDR)
      .raz() // AUX ID set to 0
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CSSELR)
      .banked();
    InitReg(MISCREG_CSSELR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_CSSELR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_VPIDR)
      .reset(midr)
      .hyp().monNonSecure();
    InitReg(MISCREG_VMPIDR)
      .res1(mask(31, 31))
      .hyp().monNonSecure();
    InitReg(MISCREG_SCTLR)
      .banked()
      // readMiscRegNoEffect() uses this metadata
      // despite using children (below) as backing store
      .res0(0x8d22c600)
      .res1(0x00400800 | (SPAN   ? 0 : 0x800000)
                       | (LSMAOE ? 0 :     0x10)
                       | (nTLSMD ? 0 :      0x8));

    auto sctlr_reset = [aarch64=highestELIs64] ()
    {
        SCTLR sctlr = 0;
        if (aarch64) {
            sctlr.afe = 1;
            sctlr.tre = 1;
            sctlr.span = 1;
            sctlr.uwxn = 1;
            sctlr.ntwe = 1;
            sctlr.ntwi = 1;
            sctlr.cp15ben = 1;
            sctlr.sa0 = 1;
        } else {
            sctlr.u = 1;
            sctlr.xp = 1;
            sctlr.uci = 1;
            sctlr.dze = 1;
            sctlr.rao2 = 1;
            sctlr.rao3 = 1;
            sctlr.rao4 = 0xf;
        }
        return sctlr;
    }();
    InitReg(MISCREG_SCTLR_NS)
      .reset(sctlr_reset)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_SCTLR_S)
      .reset(sctlr_reset)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_ACTLR)
      .banked();
    InitReg(MISCREG_ACTLR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_ACTLR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_CPACR)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_SDCR)
      .mon();
    InitReg(MISCREG_SCR)
      .reset(release->has(ArmExtension::SECURITY) ? 0 : 1)
      .mon().secure().exceptUserMode()
      .res0(0xff40)  // [31:16], [6]
      .res1(0x0030); // [5:4]
    InitReg(MISCREG_SDER)
      .mon();
    InitReg(MISCREG_NSACR)
      .allPrivileges().hypWrite(0).privNonSecureWrite(0).exceptUserMode();
    InitReg(MISCREG_HSCTLR)
      .reset(0x30c50830)
      .hyp().monNonSecure()
      .res0(0x0512c7c0 | (EnDB   ? 0 :     0x2000)
                       | (IESB   ? 0 :   0x200000)
                       | (EnDA   ? 0 :  0x8000000)
                       | (EnIB   ? 0 : 0x40000000)
                       | (EnIA   ? 0 : 0x80000000))
      .res1(0x30c50830);
    InitReg(MISCREG_HACTLR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HCR)
      .hyp().monNonSecure()
      .res0(release->has(ArmExtension::VIRTUALIZATION) ?
          0x90000000 : mask(31, 0));
    InitReg(MISCREG_HCR2)
      .hyp().monNonSecure()
      .res0(release->has(ArmExtension::VIRTUALIZATION) ?
          0xffa9ff8c : mask(31, 0));
    InitReg(MISCREG_HDCR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HCPTR)
      .res0(mask(29, 21) | mask(19, 16) | mask(14, 14))
      .res1(mask(13, 12) | mask(9, 0))
      .hyp().monNonSecure();
    InitReg(MISCREG_HSTR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HACR)
      .unimplemented()
      .warnNotFail()
      .hyp().monNonSecure();
    InitReg(MISCREG_TTBR0)
      .banked();
    InitReg(MISCREG_TTBR0_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TTBR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TTBR1)
      .banked();
    InitReg(MISCREG_TTBR1_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TTBR1_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TTBCR)
      .banked();
    InitReg(MISCREG_TTBCR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TTBCR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HTCR)
      .hyp().monNonSecure();
    InitReg(MISCREG_VTCR)
      .hyp().monNonSecure();
    InitReg(MISCREG_DACR)
      .banked();
    InitReg(MISCREG_DACR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DACR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_DFSR)
      .banked()
      .res0(mask(31, 14) | mask(8, 8));
    InitReg(MISCREG_DFSR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DFSR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_IFSR)
      .banked()
      .res0(mask(31, 13) | mask(11, 11) | mask(8, 6));
    InitReg(MISCREG_IFSR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_IFSR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_ADFSR)
      .unimplemented()
      .warnNotFail()
      .banked();
    InitReg(MISCREG_ADFSR_NS)
      .unimplemented()
      .warnNotFail()
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_ADFSR_S)
      .unimplemented()
      .warnNotFail()
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_AIFSR)
      .unimplemented()
      .warnNotFail()
      .banked();
    InitReg(MISCREG_AIFSR_NS)
      .unimplemented()
      .warnNotFail()
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_AIFSR_S)
      .unimplemented()
      .warnNotFail()
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HADFSR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HAIFSR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HSR)
      .hyp().monNonSecure();
    InitReg(MISCREG_DFAR)
      .banked();
    InitReg(MISCREG_DFAR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DFAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_IFAR)
      .banked();
    InitReg(MISCREG_IFAR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_IFAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HDFAR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HIFAR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HPFAR)
      .hyp().monNonSecure();
    InitReg(MISCREG_ICIALLUIS)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_BPIALLIS)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_PAR)
      .banked();
    InitReg(MISCREG_PAR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_PAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_ICIALLU)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ICIMVAU)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_CP15ISB)
      .writes(1);
    InitReg(MISCREG_BPIALL)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_BPIMVA)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCIMVAC)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCISW)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1CPR)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1CPW)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1CUR)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1CUW)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS12NSOPR)
      .privSecureWrite().hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_ATS12NSOPW)
      .privSecureWrite().hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_ATS12NSOUR)
      .privSecureWrite().hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_ATS12NSOUW)
      .privSecureWrite().hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_DCCMVAC)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCCSW)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_CP15DSB)
      .writes(1);
    InitReg(MISCREG_CP15DMB)
      .writes(1);
    InitReg(MISCREG_DCCMVAU)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCCIMVAC)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCCISW)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1HR)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_ATS1HW)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIASIDIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAAIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVALIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAALIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ITLBIALL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ITLBIMVA)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ITLBIASID)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DTLBIALL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DTLBIMVA)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DTLBIASID)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIALL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVA)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIASID)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAA)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAAL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIIPAS2IS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIIPAS2LIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLHIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIMVAHIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLNSNHIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIMVALHIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIIPAS2)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIIPAS2L)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLH)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIMVAH)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLNSNH)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIMVALH)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_PMCR)
      .allPrivileges();
    InitReg(MISCREG_PMCNTENSET)
      .allPrivileges();
    InitReg(MISCREG_PMCNTENCLR)
      .allPrivileges();
    InitReg(MISCREG_PMOVSR)
      .allPrivileges();
    InitReg(MISCREG_PMSWINC)
      .allPrivileges();
    InitReg(MISCREG_PMSELR)
      .allPrivileges();
    InitReg(MISCREG_PMCEID0)
      .allPrivileges();
    InitReg(MISCREG_PMCEID1)
      .allPrivileges();
    InitReg(MISCREG_PMCCNTR)
      .allPrivileges();
    InitReg(MISCREG_PMXEVTYPER)
      .allPrivileges();
    InitReg(MISCREG_PMCCFILTR)
      .allPrivileges();
    InitReg(MISCREG_PMXEVCNTR)
      .allPrivileges();
    InitReg(MISCREG_PMUSERENR)
      .allPrivileges().userNonSecureWrite(0).userSecureWrite(0);
    InitReg(MISCREG_PMINTENSET)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_PMINTENCLR)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_PMOVSSET)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_L2CTLR)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2ECTLR)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_PRRR)
      .banked();
    InitReg(MISCREG_PRRR_NS)
      .bankedChild()
      .reset(
        (1 << 19) | // 19
        (0 << 18) | // 18
        (0 << 17) | // 17
        (1 << 16) | // 16
        (2 << 14) | // 15:14
        (0 << 12) | // 13:12
        (2 << 10) | // 11:10
        (2 << 8)  | // 9:8
        (2 << 6)  | // 7:6
        (2 << 4)  | // 5:4
        (1 << 2)  | // 3:2
        0)
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_PRRR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MAIR0)
      .banked();
    InitReg(MISCREG_MAIR0_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_MAIR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_NMRR)
      .banked();
    InitReg(MISCREG_NMRR_NS)
      .bankedChild()
      .reset(
        (1 << 30) | // 31:30
        (0 << 26) | // 27:26
        (0 << 24) | // 25:24
        (3 << 22) | // 23:22
        (2 << 20) | // 21:20
        (0 << 18) | // 19:18
        (0 << 16) | // 17:16
        (1 << 14) | // 15:14
        (0 << 12) | // 13:12
        (2 << 10) | // 11:10
        (0 << 8)  | // 9:8
        (3 << 6)  | // 7:6
        (2 << 4)  | // 5:4
        (0 << 2)  | // 3:2
        0)
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_NMRR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MAIR1)
      .banked();
    InitReg(MISCREG_MAIR1_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_MAIR1_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_AMAIR0)
      .res0(release->has(ArmExtension::LPAE) ? 0 : mask(31, 0))
      .banked();
    InitReg(MISCREG_AMAIR0_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_AMAIR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_AMAIR1)
      .res0(release->has(ArmExtension::LPAE) ? 0 : mask(31, 0))
      .banked();
    InitReg(MISCREG_AMAIR1_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_AMAIR1_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HMAIR0)
      .hyp().monNonSecure();
    InitReg(MISCREG_HMAIR1)
      .hyp().monNonSecure();
    InitReg(MISCREG_HAMAIR0)
      .unimplemented()
      .warnNotFail()
      .hyp().monNonSecure();
    InitReg(MISCREG_HAMAIR1)
      .unimplemented()
      .warnNotFail()
      .hyp().monNonSecure();
    InitReg(MISCREG_VBAR)
      .banked();
    InitReg(MISCREG_VBAR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_VBAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MVBAR)
      .reset(FullSystem ? system->resetAddr() : 0)
      .mon().secure()
      .hypRead(FullSystem && system->highestEL() == EL2)
      .privRead(FullSystem && system->highestEL() == EL1)
      .exceptUserMode();
    InitReg(MISCREG_RMR)
      .unimplemented()
      .mon().secure().exceptUserMode();
    InitReg(MISCREG_ISR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_HVBAR)
      .hyp().monNonSecure()
      .res0(0x1f);
    InitReg(MISCREG_FCSEIDR)
      .unimplemented()
      .warnNotFail()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CONTEXTIDR)
      .banked();
    InitReg(MISCREG_CONTEXTIDR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_CONTEXTIDR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TPIDRURW)
      .banked();
    InitReg(MISCREG_TPIDRURW_NS)
      .bankedChild()
      .allPrivileges()
      .privSecure(!aarch32EL3)
      .monSecure(0);
    InitReg(MISCREG_TPIDRURW_S)
      .bankedChild()
      .secure();
    InitReg(MISCREG_TPIDRURO)
      .banked();
    InitReg(MISCREG_TPIDRURO_NS)
      .bankedChild()
      .allPrivileges()
      .userNonSecureWrite(0).userSecureRead(1)
      .privSecure(!aarch32EL3)
      .monSecure(0);
    InitReg(MISCREG_TPIDRURO_S)
      .bankedChild()
      .secure().userSecureWrite(0);
    InitReg(MISCREG_TPIDRPRW)
      .banked();
    InitReg(MISCREG_TPIDRPRW_NS)
      .bankedChild()
      .nonSecure().exceptUserMode()
      .privSecure(!aarch32EL3);
    InitReg(MISCREG_TPIDRPRW_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HTPIDR)
      .hyp().monNonSecure();
    // BEGIN Generic Timer (AArch32)
    InitReg(MISCREG_CNTFRQ)
      .reads(1)
      .highest(system)
      .privSecureWrite(aarch32EL3);
    InitReg(MISCREG_CNTPCT)
      .unverifiable()
      .reads(1);
    InitReg(MISCREG_CNTVCT)
      .unverifiable()
      .reads(1);
    InitReg(MISCREG_CNTP_CTL)
      .banked();
    InitReg(MISCREG_CNTP_CTL_NS)
      .bankedChild()
      .nonSecure()
      .privSecure(!aarch32EL3)
      .userSecureRead(!aarch32EL3)
      .userSecureWrite(!aarch32EL3)
      .res0(0xfffffff8);
    InitReg(MISCREG_CNTP_CTL_S)
      .bankedChild()
      .secure()
      .privSecure(aarch32EL3)
      .res0(0xfffffff8);
    InitReg(MISCREG_CNTP_CVAL)
      .banked();
    InitReg(MISCREG_CNTP_CVAL_NS)
      .bankedChild()
      .nonSecure()
      .privSecure(!aarch32EL3)
      .userSecureRead(!aarch32EL3)
      .userSecureWrite(!aarch32EL3);
    InitReg(MISCREG_CNTP_CVAL_S)
      .bankedChild()
      .secure()
      .privSecure(aarch32EL3);
    InitReg(MISCREG_CNTP_TVAL)
      .banked();
    InitReg(MISCREG_CNTP_TVAL_NS)
      .bankedChild()
      .nonSecure()
      .privSecure(!aarch32EL3)
      .userSecureRead(!aarch32EL3)
      .userSecureWrite(!aarch32EL3);
    InitReg(MISCREG_CNTP_TVAL_S)
      .bankedChild()
      .secure()
      .privSecure(aarch32EL3);
    InitReg(MISCREG_CNTV_CTL)
      .allPrivileges()
      .res0(0xfffffff8);
    InitReg(MISCREG_CNTV_CVAL)
      .allPrivileges();
    InitReg(MISCREG_CNTV_TVAL)
      .allPrivileges();
    InitReg(MISCREG_CNTKCTL)
      .allPrivileges()
      .exceptUserMode()
      .res0(0xfffdfc00);
    InitReg(MISCREG_CNTHCTL)
      .monNonSecure()
      .hyp()
      .res0(0xfffdff00);
    InitReg(MISCREG_CNTHP_CTL)
      .monNonSecure()
      .hyp()
      .res0(0xfffffff8);
    InitReg(MISCREG_CNTHP_CVAL)
      .monNonSecure()
      .hyp();
    InitReg(MISCREG_CNTHP_TVAL)
      .monNonSecure()
      .hyp();
    InitReg(MISCREG_CNTVOFF)
      .monNonSecure()
      .hyp();
    // END Generic Timer (AArch32)
    InitReg(MISCREG_IL1DATA0)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA1)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA2)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA3)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA0)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA1)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA2)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA3)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA4)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_RAMINDEX)
      .unimplemented()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_L2ACTLR)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CBAR)
      .unimplemented()
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_HTTBR)
      .hyp().monNonSecure();
    InitReg(MISCREG_VTTBR)
      .hyp().monNonSecure();
    InitReg(MISCREG_CPUMERRSR)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2MERRSR)
      .unimplemented()
      .warnNotFail()
      .allPrivileges().exceptUserMode();

    // AArch64 registers (Op0=2);
    InitReg(MISCREG_MDCCINT_EL1)
      .fault(EL1, faultMdccsrEL1)
      .fault(EL2, faultMdccsrEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_OSDTRRX_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGDTRRXext);
    InitReg(MISCREG_MDSCR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::mdscrEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::mdscrEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGDSCRext);
    InitReg(MISCREG_OSDTRTX_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGDTRTXext);
    InitReg(MISCREG_OSECCR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::oseccrEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::oseccrEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGOSECCR);
    InitReg(MISCREG_DBGBVR0_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR0, MISCREG_DBGBXVR0);
    InitReg(MISCREG_DBGBVR1_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR1, MISCREG_DBGBXVR1);
    InitReg(MISCREG_DBGBVR2_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR2, MISCREG_DBGBXVR2);
    InitReg(MISCREG_DBGBVR3_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR3, MISCREG_DBGBXVR3);
    InitReg(MISCREG_DBGBVR4_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR4, MISCREG_DBGBXVR4);
    InitReg(MISCREG_DBGBVR5_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR5, MISCREG_DBGBXVR5);
    InitReg(MISCREG_DBGBVR6_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR6, MISCREG_DBGBXVR6);
    InitReg(MISCREG_DBGBVR7_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR7, MISCREG_DBGBXVR7);
    InitReg(MISCREG_DBGBVR8_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR8, MISCREG_DBGBXVR8);
    InitReg(MISCREG_DBGBVR9_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR9, MISCREG_DBGBXVR9);
    InitReg(MISCREG_DBGBVR10_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR10, MISCREG_DBGBXVR10);
    InitReg(MISCREG_DBGBVR11_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR11, MISCREG_DBGBXVR11);
    InitReg(MISCREG_DBGBVR12_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR12, MISCREG_DBGBXVR12);
    InitReg(MISCREG_DBGBVR13_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR13, MISCREG_DBGBXVR13);
    InitReg(MISCREG_DBGBVR14_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR14, MISCREG_DBGBXVR14);
    InitReg(MISCREG_DBGBVR15_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBVR15, MISCREG_DBGBXVR15);
    InitReg(MISCREG_DBGBCR0_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR0);
    InitReg(MISCREG_DBGBCR1_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR1);
    InitReg(MISCREG_DBGBCR2_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR2);
    InitReg(MISCREG_DBGBCR3_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR3);
    InitReg(MISCREG_DBGBCR4_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR4);
    InitReg(MISCREG_DBGBCR5_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR5);
    InitReg(MISCREG_DBGBCR6_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR6);
    InitReg(MISCREG_DBGBCR7_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR7);
    InitReg(MISCREG_DBGBCR8_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR8);
    InitReg(MISCREG_DBGBCR9_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR9);
    InitReg(MISCREG_DBGBCR10_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR10);
    InitReg(MISCREG_DBGBCR11_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR11);
    InitReg(MISCREG_DBGBCR12_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR12);
    InitReg(MISCREG_DBGBCR13_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR13);
    InitReg(MISCREG_DBGBCR14_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR14);
    InitReg(MISCREG_DBGBCR15_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgbcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgbcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGBCR15);
    InitReg(MISCREG_DBGWVR0_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR0);
    InitReg(MISCREG_DBGWVR1_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR1);
    InitReg(MISCREG_DBGWVR2_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR2);
    InitReg(MISCREG_DBGWVR3_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR3);
    InitReg(MISCREG_DBGWVR4_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR4);
    InitReg(MISCREG_DBGWVR5_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR5);
    InitReg(MISCREG_DBGWVR6_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR6);
    InitReg(MISCREG_DBGWVR7_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR7);
    InitReg(MISCREG_DBGWVR8_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR8);
    InitReg(MISCREG_DBGWVR9_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR9);
    InitReg(MISCREG_DBGWVR10_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR10);
    InitReg(MISCREG_DBGWVR11_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR11);
    InitReg(MISCREG_DBGWVR12_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR12);
    InitReg(MISCREG_DBGWVR13_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR13);
    InitReg(MISCREG_DBGWVR14_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR14);
    InitReg(MISCREG_DBGWVR15_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwvrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwvrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWVR15);
    InitReg(MISCREG_DBGWCR0_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR0);
    InitReg(MISCREG_DBGWCR1_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR1);
    InitReg(MISCREG_DBGWCR2_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR2);
    InitReg(MISCREG_DBGWCR3_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR3);
    InitReg(MISCREG_DBGWCR4_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR4);
    InitReg(MISCREG_DBGWCR5_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR5);
    InitReg(MISCREG_DBGWCR6_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR6);
    InitReg(MISCREG_DBGWCR7_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR7);
    InitReg(MISCREG_DBGWCR8_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR8);
    InitReg(MISCREG_DBGWCR9_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR9);
    InitReg(MISCREG_DBGWCR10_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR10);
    InitReg(MISCREG_DBGWCR11_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR11);
    InitReg(MISCREG_DBGWCR12_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR12);
    InitReg(MISCREG_DBGWCR13_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR13);
    InitReg(MISCREG_DBGWCR14_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR14);
    InitReg(MISCREG_DBGWCR15_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgwcrnEL1>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgwcrnEL1>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGWCR15);
    InitReg(MISCREG_MDCCSR_EL0)
      .allPrivileges().writes(0)
      .faultRead(EL0, faultMdccsrEL0)
      .faultRead(EL1, faultMdccsrEL1)
      .faultRead(EL2, faultMdccsrEL2)
      .mapsTo(MISCREG_DBGDSCRint);
    InitReg(MISCREG_MDDTR_EL0)
      .allPrivileges();
    InitReg(MISCREG_MDDTRTX_EL0)
      .allPrivileges();
    InitReg(MISCREG_MDDTRRX_EL0)
      .allPrivileges();
    InitReg(MISCREG_DBGVCR32_EL2)
      .hyp().mon()
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGVCR);
    InitReg(MISCREG_MDRAR_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL1, faultDebugEL1)
      .faultRead(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGDRAR);
    InitReg(MISCREG_OSLAR_EL1)
      .allPrivileges().exceptUserMode().reads(0)
      .faultWrite(EL1, faultDebugOsEL1<false, &HDFGTR::oslarEL1>)
      .faultWrite(EL2, faultDebugOsEL2)
      .mapsTo(MISCREG_DBGOSLAR);
    InitReg(MISCREG_OSLSR_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL1, faultDebugOsEL1<true, &HDFGTR::oslsrEL1>)
      .faultRead(EL2, faultDebugOsEL2)
      .mapsTo(MISCREG_DBGOSLSR);
    InitReg(MISCREG_OSDLR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugOsEL1<true, &HDFGTR::osdlrEL1>)
      .faultWrite(EL1, faultDebugOsEL1<false, &HDFGTR::osdlrEL1>)
      .fault(EL2, faultDebugOsEL2)
      .mapsTo(MISCREG_DBGOSDLR);
    InitReg(MISCREG_DBGPRCR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugOsEL1<true, &HDFGTR::dbgprcrEL1>)
      .faultWrite(EL1, faultDebugOsEL1<false, &HDFGTR::dbgprcrEL1>)
      .fault(EL2, faultDebugOsEL2)
      .mapsTo(MISCREG_DBGPRCR);
    InitReg(MISCREG_DBGCLAIMSET_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgclaim>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgclaim>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGCLAIMSET);
    InitReg(MISCREG_DBGCLAIMCLR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgclaim>)
      .faultWrite(EL1, faultDebugWithFgtEL1<false, &HDFGTR::dbgclaim>)
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGCLAIMCLR);
    InitReg(MISCREG_DBGAUTHSTATUS_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL1, faultDebugWithFgtEL1<true, &HDFGTR::dbgauthstatusEL1>)
      .faultRead(EL2, faultDebugEL2)
      .mapsTo(MISCREG_DBGAUTHSTATUS);
    InitReg(MISCREG_TEECR32_EL1);
    InitReg(MISCREG_TEEHBR32_EL1);

    // AArch64 registers (Op0=1,3);
    InitReg(MISCREG_MIDR_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultFgtEL1<true, &HFGTR::midrEL1>)
      .mapsTo(MISCREG_MIDR);
    InitReg(MISCREG_MPIDR_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultFgtEL1<true, &HFGTR::mpidrEL1>)
      .mapsTo(MISCREG_MPIDR);
    InitReg(MISCREG_REVIDR_EL1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::tid1, &HFGTR::revidrEL1>)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_PFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_PFR0);
    InitReg(MISCREG_ID_PFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_PFR1);
    InitReg(MISCREG_ID_DFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_DFR0);
    InitReg(MISCREG_ID_AFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_AFR0);
    InitReg(MISCREG_ID_MMFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_MMFR0);
    InitReg(MISCREG_ID_MMFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_MMFR1);
    InitReg(MISCREG_ID_MMFR2_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_MMFR2);
    InitReg(MISCREG_ID_MMFR3_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_MMFR3);
    InitReg(MISCREG_ID_MMFR4_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_MMFR4);
    InitReg(MISCREG_ID_ISAR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_ISAR0);
    InitReg(MISCREG_ID_ISAR1_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_ISAR1);
    InitReg(MISCREG_ID_ISAR2_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_ISAR2);
    InitReg(MISCREG_ID_ISAR3_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_ISAR3);
    InitReg(MISCREG_ID_ISAR4_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_ISAR4);
    InitReg(MISCREG_ID_ISAR5_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_ISAR5);
    InitReg(MISCREG_ID_ISAR6_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .mapsTo(MISCREG_ID_ISAR6);
    InitReg(MISCREG_MVFR0_EL1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_MVFR0);
    InitReg(MISCREG_MVFR1_EL1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_MVFR1);
    InitReg(MISCREG_MVFR2_EL1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64PFR0_EL1)
      .reset([this,release=release,tc=tc](){
          AA64PFR0 pfr0_el1 = 0;
          pfr0_el1.el0 = 0x2;
          pfr0_el1.el1 = 0x2;
          pfr0_el1.el2 = release->has(ArmExtension::VIRTUALIZATION)
                                  ? 0x2 : 0x0;
          pfr0_el1.el3 = release->has(ArmExtension::SECURITY) ? 0x2 : 0x0;
          pfr0_el1.sve = release->has(ArmExtension::FEAT_SVE) ? 0x1 : 0x0;
          pfr0_el1.sel2 = release->has(ArmExtension::FEAT_SEL2) ? 0x1 : 0x0;
          // See MPAM frac in MISCREG_ID_AA64PFR1_EL1. Currently supporting
          // MPAMv0p1
          pfr0_el1.mpam = 0x0;
          pfr0_el1.gic = FullSystem && getGICv3CPUInterface(tc) ? 0x1 : 0;
          return pfr0_el1;
      }())
      .unserialize(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64PFR1_EL1)
      .reset([release=release](){
          AA64PFR1 pfr1_el1 = 0;
          pfr1_el1.sme = release->has(ArmExtension::FEAT_SME) ? 0x1 : 0x0;
          pfr1_el1.mpamFrac = release->has(ArmExtension::FEAT_MPAM) ?
              0x1 : 0x0;
          return pfr1_el1;
      }())
      .unserialize(0)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64DFR0_EL1)
      .reset([p](){
          AA64DFR0 dfr0_el1 = p.id_aa64dfr0_el1;
          dfr0_el1.pmuver = p.pmu ? 1 : 0; // Enable PMUv3
          return dfr0_el1;
      }())
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64DFR1_EL1)
      .reset(p.id_aa64dfr1_el1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64AFR0_EL1)
      .reset(p.id_aa64afr0_el1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64AFR1_EL1)
      .reset(p.id_aa64afr1_el1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64ISAR0_EL1)
      .reset([p,release=release](){
          AA64ISAR0 isar0_el1 = p.id_aa64isar0_el1;
          isar0_el1.crc32 = release->has(ArmExtension::FEAT_CRC32) ? 0x1 : 0x0;
          isar0_el1.sha2 = release->has(ArmExtension::FEAT_SHA256) ? 0x1 : 0x0;
          isar0_el1.sha1 = release->has(ArmExtension::FEAT_SHA1) ? 0x1 : 0x0;
          isar0_el1.aes = release->has(ArmExtension::FEAT_PMULL) ?
              0x2 : release->has(ArmExtension::FEAT_AES) ?
                  0x1 : 0x0;
          isar0_el1.dp = release->has(ArmExtension::FEAT_DOTPROD) ? 0x1 : 0x0;
          isar0_el1.atomic = release->has(ArmExtension::FEAT_LSE) ? 0x2 : 0x0;
          isar0_el1.rdm = release->has(ArmExtension::FEAT_RDM) ? 0x1 : 0x0;
          isar0_el1.tme = release->has(ArmExtension::TME) ? 0x1 : 0x0;
          isar0_el1.tlb = release->has(ArmExtension::FEAT_TLBIRANGE) ?
              0x2 : release->has(ArmExtension::FEAT_TLBIOS) ?
                  0x1 : 0x0;
          isar0_el1.ts = release->has(ArmExtension::FEAT_FLAGM2) ?
              0x2 : release->has(ArmExtension::FEAT_FLAGM) ?
                  0x1 : 0x0;
          isar0_el1.rndr = release->has(ArmExtension::FEAT_RNG) ? 0x1 : 0x0;
          return isar0_el1;
      }())
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64ISAR1_EL1)
      .reset([p,release=release](){
          AA64ISAR1 isar1_el1 = p.id_aa64isar1_el1;
          isar1_el1.xs = release->has(ArmExtension::FEAT_XS) ? 0x1 : 0x0;
          isar1_el1.i8mm = release->has(ArmExtension::FEAT_I8MM) ? 0x1 : 0x0;
          isar1_el1.apa = release->has(ArmExtension::FEAT_PAuth) ? 0x1 : 0x0;
          isar1_el1.jscvt = release->has(ArmExtension::FEAT_JSCVT) ? 0x1 : 0x0;
          isar1_el1.fcma = release->has(ArmExtension::FEAT_FCMA) ? 0x1 : 0x0;
          isar1_el1.gpa = release->has(ArmExtension::FEAT_PAuth) ? 0x1 : 0x0;
          return isar1_el1;
      }())
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64MMFR0_EL1)
      .reset([p,asidbits=haveLargeAsid64,parange=physAddrRange](){
          AA64MMFR0 mmfr0_el1 = p.id_aa64mmfr0_el1;
          mmfr0_el1.asidbits = asidbits ? 0x2 : 0x0;
          mmfr0_el1.parange = encodePhysAddrRange64(parange);
          return mmfr0_el1;
      }())
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64MMFR1_EL1)
      .reset([p,release=release](){
          AA64MMFR1 mmfr1_el1 = p.id_aa64mmfr1_el1;
          mmfr1_el1.vmidbits =
            release->has(ArmExtension::FEAT_VMID16) ? 0x2 : 0x0;
          mmfr1_el1.vh = release->has(ArmExtension::FEAT_VHE) ? 0x1 : 0x0;
          mmfr1_el1.hpds = release->has(ArmExtension::FEAT_HPDS) ? 0x1 : 0x0;
          mmfr1_el1.pan = release->has(ArmExtension::FEAT_PAN) ? 0x1 : 0x0;
          mmfr1_el1.hcx = release->has(ArmExtension::FEAT_HCX) ? 0x1 : 0x0;
          return mmfr1_el1;
      }())
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64MMFR2_EL1)
      .reset([p,release=release](){
          AA64MMFR2 mmfr2_el1 = p.id_aa64mmfr2_el1;
          mmfr2_el1.uao = release->has(ArmExtension::FEAT_UAO) ? 0x1 : 0x0;
          mmfr2_el1.varange = release->has(ArmExtension::FEAT_LVA) ? 0x1 : 0x0;
          mmfr2_el1.st = release->has(ArmExtension::FEAT_TTST) ? 0x1 : 0x0;
          mmfr2_el1.ids = release->has(ArmExtension::FEAT_IDST) ? 0x1 : 0x0;
          mmfr2_el1.evt = release->has(ArmExtension::FEAT_EVT) ? 0x2 : 0x0;
          return mmfr2_el1;
      }())
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_ID_AA64MMFR3_EL1)
      .reset([p,release=release](){
          AA64MMFR3 mmfr3_el1 = 0;
          mmfr3_el1.sctlrx =
            release->has(ArmExtension::FEAT_SCTLR2) ? 0x1 : 0x0;
          mmfr3_el1.tcrx = release->has(ArmExtension::FEAT_TCR2) ? 0x1 : 0x0;
          return mmfr3_el1;
      }())
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
      .allPrivileges().writes(0);

    InitReg(MISCREG_APDAKeyHi_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apdaKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apdaKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APDAKeyLo_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apdaKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apdaKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APDBKeyHi_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apdbKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apdbKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APDBKeyLo_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apdbKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apdbKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APGAKeyHi_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apgaKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apgaKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APGAKeyLo_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apgaKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apgaKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APIAKeyHi_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apiaKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apiaKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APIAKeyLo_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apiaKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apiaKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APIBKeyHi_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apibKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apibKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APIBKeyLo_EL1)
      .faultRead(EL1, faultPauthEL1<true, &HFGTR::apibKey>)
      .faultWrite(EL1, faultPauthEL1<false, &HFGTR::apibKey>)
      .fault(EL2, faultPauthEL2)
      .allPrivileges().exceptUserMode();

    InitReg(MISCREG_CCSIDR_EL1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultCacheEL1<true, &HFGTR::ccsidrEL1>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_CLIDR_EL1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultCacheEL1<true, &HFGTR::clidrEL1>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_AIDR_EL1)
      .faultRead(EL0, faultIdst)
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::tid1, &HFGTR::aidrEL1>)
      .allPrivileges().writes(0);
    InitReg(MISCREG_CSSELR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultCacheEL1<true, &HFGTR::csselrEL1>)
      .faultWrite(EL1, faultCacheEL1<false, &HFGTR::csselrEL1>)
      .mapsTo(MISCREG_CSSELR_NS);
    InitReg(MISCREG_CTR_EL0)
      .faultRead(EL0, faultCtrEL0)
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::tid2, &HFGTR::ctrEL0>)
      .reads(1)
      .mapsTo(MISCREG_CTR);
    InitReg(MISCREG_DCZID_EL0)
      .reset(0x04) // DC ZVA clear 64-byte chunks
      .faultRead(EL0, faultFgtEL0<true, &HFGTR::dczidEL0>)
      .faultRead(EL1, faultFgtEL1<true, &HFGTR::dczidEL0>)
      .reads(1);
    InitReg(MISCREG_VPIDR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VPIDR);
    InitReg(MISCREG_VMPIDR_EL2)
      .hyp().mon()
      .res0(mask(63, 40) | mask(29, 25))
      .res1(mask(31, 31))
      .mapsTo(MISCREG_VMPIDR);
    InitReg(MISCREG_SCTLR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::sctlrEL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::sctlrEL1>)
      .res0( 0x20440 | (EnDB   ? 0 :     0x2000)
                     | (IESB   ? 0 :   0x200000)
                     | (EnDA   ? 0 :  0x8000000)
                     | (EnIB   ? 0 : 0x40000000)
                     | (EnIA   ? 0 : 0x80000000))
      .res1(0x500800 | (SPAN   ? 0 :   0x800000)
                     | (nTLSMD ? 0 :  0x8000000)
                     | (LSMAOE ? 0 : 0x10000000))
      .mapsTo(MISCREG_SCTLR_NS);
    InitReg(MISCREG_SCTLR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .res0( 0x20440 | (EnDB   ? 0 :     0x2000)
                     | (IESB   ? 0 :   0x200000)
                     | (EnDA   ? 0 :  0x8000000)
                     | (EnIB   ? 0 : 0x40000000)
                     | (EnIA   ? 0 : 0x80000000))
      .res1(0x500800 | (SPAN   ? 0 :   0x800000)
                     | (nTLSMD ? 0 :  0x8000000)
                     | (LSMAOE ? 0 : 0x10000000))
      .mapsTo(MISCREG_SCTLR_EL1);
    InitReg(MISCREG_SCTLR2_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultSctlr2EL1<true, &HCR::trvm>)
      .faultWrite(EL1, faultSctlr2EL1<false, &HCR::tvm>)
      .fault(EL2,faultSctlr2EL2);
    InitReg(MISCREG_SCTLR2_EL12)
      .fault(EL2, faultSctlr2VheEL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_SCTLR2_EL1);
    InitReg(MISCREG_ACTLR_EL1)
      .allPrivileges().exceptUserMode()
      .fault(EL1, faultHcrEL1<&HCR::tacr>)
      .mapsTo(MISCREG_ACTLR_NS);
    InitReg(MISCREG_CPACR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultCpacrEL1<true, &HFGTR::cpacrEL1>)
      .faultWrite(EL1, faultCpacrEL1<false, &HFGTR::cpacrEL1>)
      .fault(EL2, faultCpacrEL2)
      .mapsTo(MISCREG_CPACR);
    InitReg(MISCREG_CPACR_EL12)
      .fault(EL2, faultCpacrVheEL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_CPACR_EL1);
    InitReg(MISCREG_SCTLR_EL2)
      .hyp().mon()
      .res0(0x0512c7c0 | (EnDB   ? 0 :     0x2000)
                       | (IESB   ? 0 :   0x200000)
                       | (EnDA   ? 0 :  0x8000000)
                       | (EnIB   ? 0 : 0x40000000)
                       | (EnIA   ? 0 : 0x80000000))
      .res1(0x30c50830)
      .mapsTo(MISCREG_HSCTLR);
    InitReg(MISCREG_SCTLR2_EL2)
      .hyp().mon()
      .fault(EL2, faultSctlr2EL2);
    InitReg(MISCREG_ACTLR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HACTLR);
    InitReg(MISCREG_HCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HCR, MISCREG_HCR2);
    InitReg(MISCREG_HCRX_EL2)
      .hyp().mon()
      .fault(EL2, faultHcrxEL2);
    InitReg(MISCREG_MDCR_EL2)
      .hyp().mon()
      .fault(EL2, faultDebugEL2)
      .mapsTo(MISCREG_HDCR);
    InitReg(MISCREG_CPTR_EL2)
      .hyp().mon()
      .fault(EL2, faultCpacrEL2)
      .mapsTo(MISCREG_HCPTR);
    InitReg(MISCREG_HSTR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HSTR);
    InitReg(MISCREG_HACR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HACR);
    InitReg(MISCREG_SCTLR_EL3)
      .reset(0x30c50830)
      .mon()
      .res0(0x0512c7c0 | (EnDB   ? 0 :     0x2000)
                       | (IESB   ? 0 :   0x200000)
                       | (EnDA   ? 0 :  0x8000000)
                       | (EnIB   ? 0 : 0x40000000)
                       | (EnIA   ? 0 : 0x80000000))
      .res1(0x30c50830);
    InitReg(MISCREG_SCTLR2_EL3)
      .mon();
    InitReg(MISCREG_ACTLR_EL3)
      .mon();
    InitReg(MISCREG_SCR_EL3)
      .mon()
      .mapsTo(MISCREG_SCR); // NAM D7-2005
    InitReg(MISCREG_SDER32_EL3)
      .mon()
      .mapsTo(MISCREG_SDER);
    InitReg(MISCREG_CPTR_EL3)
      .mon();
    InitReg(MISCREG_MDCR_EL3)
      .mon()
      .mapsTo(MISCREG_SDCR);
    InitReg(MISCREG_TTBR0_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::ttbr0EL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::ttbr0EL1>)
      .mapsTo(MISCREG_TTBR0_NS);
    InitReg(MISCREG_TTBR0_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_TTBR0_EL1);
    InitReg(MISCREG_TTBR1_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::ttbr1EL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::ttbr1EL1>)
      .mapsTo(MISCREG_TTBR1_NS);
    InitReg(MISCREG_TTBR1_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_TTBR1_EL1);
    InitReg(MISCREG_TCR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::tcrEL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::tcrEL1>)
      .mapsTo(MISCREG_TTBCR_NS);
    InitReg(MISCREG_TCR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_TTBCR_NS);
    InitReg(MISCREG_TCR2_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultTcr2EL1<true, &HCR::trvm>)
      .faultWrite(EL1, faultTcr2EL1<false, &HCR::tvm>)
      .fault(EL2, faultTcr2EL2);
    InitReg(MISCREG_TCR2_EL12)
      .fault(EL2, faultTcr2VheEL2)
      .fault(EL3, faultTcr2VheEL3)
      .mapsTo(MISCREG_TCR2_EL1);
    InitReg(MISCREG_TTBR0_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HTTBR);
    InitReg(MISCREG_TTBR1_EL2)
      .hyp().mon();
    InitReg(MISCREG_TCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HTCR);
    InitReg(MISCREG_TCR2_EL2)
      .hyp().mon()
      .fault(EL2, faultTcr2EL2);
    InitReg(MISCREG_VTTBR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VTTBR);
    InitReg(MISCREG_VTCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VTCR);
    InitReg(MISCREG_VSTTBR_EL2)
      .hypSecure().mon();
    InitReg(MISCREG_VSTCR_EL2)
      .hypSecure().mon();
    InitReg(MISCREG_TTBR0_EL3)
      .mon();
    InitReg(MISCREG_TCR_EL3)
      .mon();
    InitReg(MISCREG_DACR32_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_DACR_NS);
    InitReg(MISCREG_SPSR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_SPSR_SVC); // NAM C5.2.17 SPSR_EL1
    InitReg(MISCREG_SPSR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_SPSR_SVC);
    InitReg(MISCREG_ELR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ELR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_ELR_EL1);
    InitReg(MISCREG_SP_EL0)
      .allPrivileges().exceptUserMode()
      .fault(EL1, faultSpEL0)
      .fault(EL2, faultSpEL0)
      .fault(EL3, faultSpEL0);
    InitReg(MISCREG_SPSEL)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CURRENTEL)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_PAN)
      .allPrivileges(release->has(ArmExtension::FEAT_PAN))
      .exceptUserMode();
    InitReg(MISCREG_UAO)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_NZCV)
      .allPrivileges();
    InitReg(MISCREG_DAIF)
      .allPrivileges()
      .fault(EL0, faultDaif);
    InitReg(MISCREG_FPCR)
      .allPrivileges()
      .fault(EL0, faultFpcrEL0)
      .fault(EL1, faultFpcrEL1)
      .fault(EL2, faultFpcrEL2)
      .fault(EL3, faultFpcrEL3);
    InitReg(MISCREG_FPSR)
      .allPrivileges()
      .fault(EL0, faultFpcrEL0)
      .fault(EL1, faultFpcrEL1)
      .fault(EL2, faultFpcrEL2)
      .fault(EL3, faultFpcrEL3);
    InitReg(MISCREG_DSPSR_EL0)
      .allPrivileges();
    InitReg(MISCREG_DLR_EL0)
      .allPrivileges();
    InitReg(MISCREG_SPSR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_SPSR_HYP); // NAM C5.2.18 SPSR_EL2
    InitReg(MISCREG_ELR_EL2)
      .hyp().mon();
    InitReg(MISCREG_SP_EL1)
      .hyp().mon();
    InitReg(MISCREG_SPSR_IRQ_AA64)
      .hyp().mon();
    InitReg(MISCREG_SPSR_ABT_AA64)
      .hyp().mon();
    InitReg(MISCREG_SPSR_UND_AA64)
      .hyp().mon();
    InitReg(MISCREG_SPSR_FIQ_AA64)
      .hyp().mon();
    InitReg(MISCREG_SPSR_EL3)
      .mon()
      .mapsTo(MISCREG_SPSR_MON); // NAM C5.2.19 SPSR_EL3
    InitReg(MISCREG_ELR_EL3)
      .mon();
    InitReg(MISCREG_SP_EL2)
      .mon();
    InitReg(MISCREG_AFSR0_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::afsr0EL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::afsr0EL1>)
      .mapsTo(MISCREG_ADFSR_NS);
    InitReg(MISCREG_AFSR0_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_ADFSR_NS);
    InitReg(MISCREG_AFSR1_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::afsr1EL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::afsr1EL1>)
      .mapsTo(MISCREG_AIFSR_NS);
    InitReg(MISCREG_AFSR1_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_AIFSR_NS);
    InitReg(MISCREG_ESR_EL1)
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::esrEL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::esrEL1>)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ESR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_ESR_EL1);
    InitReg(MISCREG_IFSR32_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_IFSR_NS);
    InitReg(MISCREG_AFSR0_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HADFSR);
    InitReg(MISCREG_AFSR1_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HAIFSR);
    InitReg(MISCREG_ESR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HSR);
    InitReg(MISCREG_FPEXC32_EL2)
      .fault(EL2, faultFpcrEL2)
      .fault(EL3, faultFpcrEL3)
      .mapsTo(MISCREG_FPEXC);
    InitReg(MISCREG_AFSR0_EL3)
      .mon();
    InitReg(MISCREG_AFSR1_EL3)
      .mon();
    InitReg(MISCREG_ESR_EL3)
      .mon();
    InitReg(MISCREG_FAR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::farEL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::farEL1>)
      .mapsTo(MISCREG_DFAR_NS, MISCREG_IFAR_NS);
    InitReg(MISCREG_FAR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_DFAR_NS, MISCREG_IFAR_NS);
    InitReg(MISCREG_FAR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HDFAR, MISCREG_HIFAR);
    InitReg(MISCREG_HPFAR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HPFAR);
    InitReg(MISCREG_FAR_EL3)
      .mon();
    InitReg(MISCREG_IC_IALLUIS)
      .warnNotFail()
      .faultWrite(EL1, faultPouIsEL1<&HFGITR::icialluis>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_PAR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_PAR_NS);
    InitReg(MISCREG_IC_IALLU)
      .warnNotFail()
      .faultWrite(EL1, faultPouEL1<&HFGITR::iciallu>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_IVAC_Xt)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::tpc, &HFGITR::dcivac>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_ISW_Xt)
      .warnNotFail()
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::tsw, &HFGITR::dcisw>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_AT_S1E1R_Xt)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::at, &HFGITR::ats1e1r>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_AT_S1E1W_Xt)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::at, &HFGITR::ats1e1w>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_AT_S1E0R_Xt)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::at, &HFGITR::ats1e0r>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_AT_S1E0W_Xt)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::at, &HFGITR::ats1e0w>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_CSW_Xt)
      .warnNotFail()
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::tsw, &HFGITR::dccsw>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_CISW_Xt)
      .warnNotFail()
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::tsw, &HFGITR::dccisw>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_ZVA_Xt)
      .writes(1)
      .faultWrite(EL0, faultDczvaEL0)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::tdz, &HFGITR::dczva>);
    InitReg(MISCREG_IC_IVAU_Xt)
      .faultWrite(EL0, faultPouEL0)
      .faultWrite(EL1, faultPouEL1<&HFGITR::icivau>)
      .writes(1);
    InitReg(MISCREG_DC_CVAC_Xt)
      .faultWrite(EL0, faultCvacEL0)
      .faultWrite(EL1, faultHcrEL1<&HCR::tpc>)
      .writes(1);
    InitReg(MISCREG_DC_CVAU_Xt)
      .faultWrite(EL0, faultPouEL0)
      .faultWrite(EL1, faultPouEL1<&HFGITR::dccvau>)
      .writes(1);
    InitReg(MISCREG_DC_CIVAC_Xt)
      .faultWrite(EL0, faultCvacEL0)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::tpc, &HFGITR::dccivac>)
      .writes(1);
    InitReg(MISCREG_AT_S1E2R_Xt)
      .monWrite().hypWrite();
    InitReg(MISCREG_AT_S1E2W_Xt)
      .monWrite().hypWrite();
    InitReg(MISCREG_AT_S12E1R_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S12E1W_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S12E0R_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S12E0W_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S1E3R_Xt)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S1E3W_Xt)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VMALLE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbivmalle1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbivae1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_ASIDE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbiaside1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAAE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbivaae1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VALE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbivale1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAALE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbivaale1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VMALLE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbivmalle1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbivae1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_ASIDE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbiaside1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAAE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbivaae1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VALE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbivale1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAALE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbivaale1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VMALLE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbivmalle1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbivae1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_ASIDE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbiaside1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAAE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbivaae1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VALE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbivale1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAALE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbivaale1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_IPAS2E1OS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_IPAS2LE1OS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE2OS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VAE2OS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE1OS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VALE2OS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VMALLS12E1OS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_IPAS2E1IS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_IPAS2LE1IS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE2IS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VAE2IS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE1IS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VALE2IS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VMALLS12E1IS)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_IPAS2E1)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_IPAS2LE1)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE2)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VAE2)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE1)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VALE2)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_VMALLS12E1)
      .monWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE3OS)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VAE3OS)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VALE3OS)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_ALLE3IS)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VAE3IS)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VALE3IS)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_ALLE3)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VAE3)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VALE3)
      .monSecureWrite().monNonSecureWrite();

    InitReg(MISCREG_TLBI_RVAE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbirvae1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAAE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbirvaae1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVALE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbirvale1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAALE1)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbirvaale1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RIPAS2E1)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RIPAS2LE1)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE2)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVALE2)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE3)
      .monWrite();
    InitReg(MISCREG_TLBI_RVALE3)
      .monWrite();
    InitReg(MISCREG_TLBI_RVAE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbirvae1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAAE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbirvaae1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVALE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbirvale1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAALE1IS)
      .faultWrite(EL1, faultTlbiIsEL1<&HFGITR::tlbirvaale1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RIPAS2E1IS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RIPAS2LE1IS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE2IS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVALE2IS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE3IS)
      .monWrite();
    InitReg(MISCREG_TLBI_RVALE3IS)
      .monWrite();
    InitReg(MISCREG_TLBI_RVAE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbirvae1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAAE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbirvaae1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVALE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbirvale1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAALE1OS)
      .faultWrite(EL1, faultTlbiOsEL1<&HFGITR::tlbirvaale1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RIPAS2E1OS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RIPAS2LE1OS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE2OS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVALE2OS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE3OS)
      .monWrite();
    InitReg(MISCREG_TLBI_RVALE3OS)
      .monWrite();
    InitReg(MISCREG_TLBI_VMALLE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbivmalle1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbivae1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_ASIDE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbiaside1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAAE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbivaae1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VALE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbivale1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAALE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbivaale1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VMALLE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbivmalle1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbivae1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_ASIDE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbiaside1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAAE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbivaae1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VALE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbivale1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAALE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbivaale1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VMALLE1NXS)
      .faultWrite(EL1, faultTlbiNxsEL1<&HCR::ttlb, &HFGITR::tlbivmalle1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAE1NXS)
      .faultWrite(EL1, faultTlbiNxsEL1<&HCR::ttlb, &HFGITR::tlbivae1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_ASIDE1NXS)
      .faultWrite(EL1, faultTlbiNxsEL1<&HCR::ttlb, &HFGITR::tlbiaside1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAAE1NXS)
      .faultWrite(EL1, faultTlbiNxsEL1<&HCR::ttlb, &HFGITR::tlbivaae1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VALE1NXS)
      .faultWrite(EL1, faultTlbiNxsEL1<&HCR::ttlb, &HFGITR::tlbivale1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAALE1NXS)
      .faultWrite(EL1, faultTlbiNxsEL1<&HCR::ttlb, &HFGITR::tlbivaale1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_IPAS2E1OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_IPAS2LE1OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_ALLE2OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VAE2OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_ALLE1OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VALE2OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VMALLS12E1OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_IPAS2E1ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_IPAS2LE1ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_ALLE2ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VAE2ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_ALLE1ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VALE2ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VMALLS12E1ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_IPAS2E1NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_IPAS2LE1NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_ALLE2NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VAE2NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_ALLE1NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VALE2NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_VMALLS12E1NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_ALLE3OSNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_VAE3OSNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_VALE3OSNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_ALLE3ISNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_VAE3ISNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_VALE3ISNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_ALLE3NXS)
      .monWrite();
    InitReg(MISCREG_TLBI_VAE3NXS)
      .monWrite();
    InitReg(MISCREG_TLBI_VALE3NXS)
      .monWrite();

    InitReg(MISCREG_TLBI_RVAE1NXS)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbirvae1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAAE1NXS)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbirvaae1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVALE1NXS)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbirvale1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAALE1NXS)
      .faultWrite(EL1, faultHcrFgtInstEL1<&HCR::ttlb, &HFGITR::tlbirvaale1>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RIPAS2E1NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RIPAS2LE1NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE2NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVALE2NXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE3NXS)
      .monWrite();
    InitReg(MISCREG_TLBI_RVALE3NXS)
      .monWrite();
    InitReg(MISCREG_TLBI_RVAE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbirvae1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAAE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbirvaae1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVALE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbirvale1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAALE1ISNXS)
      .faultWrite(EL1, faultTlbiIsNxsEL1<&HFGITR::tlbirvaale1is>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RIPAS2E1ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RIPAS2LE1ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE2ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVALE2ISNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE3ISNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_RVALE3ISNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_RVAE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbirvae1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAAE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbirvaae1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVALE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbirvale1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RVAALE1OSNXS)
      .faultWrite(EL1, faultTlbiOsNxsEL1<&HFGITR::tlbirvaale1os>)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_RIPAS2E1OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RIPAS2LE1OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE2OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVALE2OSNXS)
      .hypWrite().monWrite();
    InitReg(MISCREG_TLBI_RVAE3OSNXS)
      .monWrite();
    InitReg(MISCREG_TLBI_RVALE3OSNXS)
      .monWrite();
    InitReg(MISCREG_PMINTENSET_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_PMINTENSET);
    InitReg(MISCREG_PMINTENCLR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_PMINTENCLR);
    InitReg(MISCREG_PMCR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMCR);
    InitReg(MISCREG_PMCNTENSET_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMCNTENSET);
    InitReg(MISCREG_PMCNTENCLR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMCNTENCLR);
    InitReg(MISCREG_PMOVSCLR_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMOVSCLR);
    InitReg(MISCREG_PMSWINC_EL0)
      .writes(1).user()
      .mapsTo(MISCREG_PMSWINC);
    InitReg(MISCREG_PMSELR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMSELR);
    InitReg(MISCREG_PMCEID0_EL0)
      .reads(1).user()
      .mapsTo(MISCREG_PMCEID0);
    InitReg(MISCREG_PMCEID1_EL0)
      .reads(1).user()
      .mapsTo(MISCREG_PMCEID1);
    InitReg(MISCREG_PMCCNTR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMCCNTR);
    InitReg(MISCREG_PMXEVTYPER_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMXEVTYPER);
    InitReg(MISCREG_PMCCFILTR_EL0)
      .allPrivileges();
    InitReg(MISCREG_PMXEVCNTR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMXEVCNTR);
    InitReg(MISCREG_PMUSERENR_EL0)
      .allPrivileges().userNonSecureWrite(0).userSecureWrite(0)
      .mapsTo(MISCREG_PMUSERENR);
    InitReg(MISCREG_PMOVSSET_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMOVSSET);
    InitReg(MISCREG_MAIR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::mairEL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::mairEL1>)
      .mapsTo(MISCREG_PRRR_NS, MISCREG_NMRR_NS);
    InitReg(MISCREG_MAIR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_PRRR_NS, MISCREG_NMRR_NS);
    InitReg(MISCREG_AMAIR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::amairEL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::amairEL1>)
      .mapsTo(MISCREG_AMAIR0_NS, MISCREG_AMAIR1_NS);
    InitReg(MISCREG_AMAIR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_AMAIR0_NS, MISCREG_AMAIR1_NS);
    InitReg(MISCREG_MAIR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HMAIR0, MISCREG_HMAIR1);
    InitReg(MISCREG_AMAIR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HAMAIR0, MISCREG_HAMAIR1);
    InitReg(MISCREG_MAIR_EL3)
      .mon();
    InitReg(MISCREG_AMAIR_EL3)
      .mon();
    InitReg(MISCREG_L2CTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2ECTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_VBAR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultFgtEL1<true, &HFGTR::vbarEL1>)
      .faultWrite(EL1, faultFgtEL1<false, &HFGTR::vbarEL1>)
      .mapsTo(MISCREG_VBAR_NS);
    InitReg(MISCREG_VBAR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_VBAR_NS);
    InitReg(MISCREG_RVBAR_EL1)
      .reset(FullSystem && system->highestEL() == EL1 ?
          system->resetAddr() : 0)
      .privRead(FullSystem && system->highestEL() == EL1);
    InitReg(MISCREG_ISR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_VBAR_EL2)
      .hyp().mon()
      .res0(0x7ff)
      .mapsTo(MISCREG_HVBAR);
    InitReg(MISCREG_RVBAR_EL2)
      .reset(FullSystem && system->highestEL() == EL2 ?
          system->resetAddr() : 0)
      .hypRead(FullSystem && system->highestEL() == EL2);
    InitReg(MISCREG_VBAR_EL3)
      .mon();
    InitReg(MISCREG_RVBAR_EL3)
      .reset(FullSystem && system->highestEL() == EL3 ?
          system->resetAddr() : 0)
      .mon().writes(0);
    InitReg(MISCREG_RMR_EL3)
      .mon();
    InitReg(MISCREG_CONTEXTIDR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultHcrFgtEL1<true, &HCR::trvm, &HFGTR::contextidrEL1>)
      .faultWrite(EL1, faultHcrFgtEL1<false, &HCR::tvm, &HFGTR::contextidrEL1>)
      .mapsTo(MISCREG_CONTEXTIDR_NS);
    InitReg(MISCREG_CONTEXTIDR_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_CONTEXTIDR_NS);
    InitReg(MISCREG_TPIDR_EL1)
      .allPrivileges().exceptUserMode()
      .faultRead(EL1, faultFgtEL1<true, &HFGTR::tpidrEL1>)
      .faultWrite(EL1, faultFgtEL1<false, &HFGTR::tpidrEL1>)
      .mapsTo(MISCREG_TPIDRPRW_NS);
    InitReg(MISCREG_TPIDR_EL0)
      .allPrivileges()
      .faultRead(EL0, faultFgtEL0<true, &HFGTR::tpidrEL0>)
      .faultWrite(EL0, faultFgtEL0<false, &HFGTR::tpidrEL0>)
      .faultRead(EL1, faultFgtEL1<true, &HFGTR::tpidrEL0>)
      .faultWrite(EL1, faultFgtEL1<false, &HFGTR::tpidrEL0>)
      .mapsTo(MISCREG_TPIDRURW_NS);
    InitReg(MISCREG_TPIDRRO_EL0)
      .allPrivileges().userNonSecureWrite(0).userSecureWrite(0)
      .faultRead(EL0, faultFgtEL0<true, &HFGTR::tpidrroEL0>)
      .faultRead(EL1, faultFgtEL1<true, &HFGTR::tpidrroEL0>)
      .faultWrite(EL1, faultFgtEL1<false, &HFGTR::tpidrroEL0>)
      .mapsTo(MISCREG_TPIDRURO_NS);
    InitReg(MISCREG_TPIDR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HTPIDR);
    InitReg(MISCREG_TPIDR_EL3)
      .mon();
    // BEGIN Generic Timer (AArch64)
    InitReg(MISCREG_CNTFRQ_EL0)
      .reads(1)
      .faultRead(EL0, faultGenericTimerEL0)
      .highest(system)
      .privSecureWrite(aarch32EL3)
      .mapsTo(MISCREG_CNTFRQ);
    InitReg(MISCREG_CNTPCT_EL0)
      .unverifiable()
      .faultRead(EL0, faultCntpctEL0)
      .faultRead(EL1, faultCntpctEL1)
      .reads(1)
      .mapsTo(MISCREG_CNTPCT);
    InitReg(MISCREG_CNTVCT_EL0)
      .unverifiable()
      .faultRead(EL0, faultCntvctEL0)
      .faultRead(EL1, faultCntvctEL1)
      .reads(1)
      .mapsTo(MISCREG_CNTVCT);
    InitReg(MISCREG_CNTP_CTL_EL0)
      .allPrivileges()
      .fault(EL0, faultCntpCtlEL0)
      .fault(EL1, faultCntpCtlEL1)
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTP_CTL_NS);
    InitReg(MISCREG_CNTP_CVAL_EL0)
      .allPrivileges()
      .fault(EL0, faultCntpCtlEL0)
      .fault(EL1, faultCntpCtlEL1)
      .mapsTo(MISCREG_CNTP_CVAL_NS);
    InitReg(MISCREG_CNTP_TVAL_EL0)
      .allPrivileges()
      .fault(EL0, faultCntpCtlEL0)
      .fault(EL1, faultCntpCtlEL1)
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTP_TVAL_NS);
    InitReg(MISCREG_CNTV_CTL_EL0)
      .allPrivileges()
      .fault(EL0, faultCntvCtlEL0)
      .fault(EL1, faultCntvCtlEL1)
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTV_CTL);
    InitReg(MISCREG_CNTV_CVAL_EL0)
      .allPrivileges()
      .fault(EL0, faultCntvCtlEL0)
      .fault(EL1, faultCntvCtlEL1)
      .mapsTo(MISCREG_CNTV_CVAL);
    InitReg(MISCREG_CNTV_TVAL_EL0)
      .allPrivileges()
      .fault(EL0, faultCntvCtlEL0)
      .fault(EL1, faultCntvCtlEL1)
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTV_TVAL);
    InitReg(MISCREG_CNTP_CTL_EL02)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTP_CTL_NS);
    InitReg(MISCREG_CNTP_CVAL_EL02)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_CNTP_CVAL_NS);
    InitReg(MISCREG_CNTP_TVAL_EL02)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTP_TVAL_NS);
    InitReg(MISCREG_CNTV_CTL_EL02)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTV_CTL);
    InitReg(MISCREG_CNTV_CVAL_EL02)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .mapsTo(MISCREG_CNTV_CVAL);
    InitReg(MISCREG_CNTV_TVAL_EL02)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTV_TVAL);
    InitReg(MISCREG_CNTKCTL_EL1)
      .allPrivileges()
      .exceptUserMode()
      .res0(0xfffffffffffdfc00)
      .mapsTo(MISCREG_CNTKCTL);
    InitReg(MISCREG_CNTKCTL_EL12)
      .fault(EL2, defaultFaultE2H_EL2)
      .fault(EL3, defaultFaultE2H_EL3)
      .res0(0xfffffffffffdfc00)
      .mapsTo(MISCREG_CNTKCTL);
    InitReg(MISCREG_CNTPS_CTL_EL1)
      .mon()
      .privSecure()
      .fault(EL1, faultCntpsCtlEL1)
      .res0(0xfffffffffffffff8);
    InitReg(MISCREG_CNTPS_CVAL_EL1)
      .mon()
      .privSecure()
      .fault(EL1, faultCntpsCtlEL1);
    InitReg(MISCREG_CNTPS_TVAL_EL1)
      .mon()
      .privSecure()
      .fault(EL1, faultCntpsCtlEL1)
      .res0(0xffffffff00000000);
    InitReg(MISCREG_CNTHCTL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffc0000)
      .mapsTo(MISCREG_CNTHCTL);
    InitReg(MISCREG_CNTHP_CTL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTHP_CTL);
    InitReg(MISCREG_CNTHP_CVAL_EL2)
      .mon()
      .hyp()
      .mapsTo(MISCREG_CNTHP_CVAL);
    InitReg(MISCREG_CNTHP_TVAL_EL2)
      .mon()
      .hyp()
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTHP_TVAL);
    InitReg(MISCREG_CNTHPS_CTL_EL2)
      .mon(sel2_implemented)
      .hypSecure(sel2_implemented)
      .res0(0xfffffffffffffff8);
    InitReg(MISCREG_CNTHPS_CVAL_EL2)
      .mon(sel2_implemented)
      .hypSecure(sel2_implemented);
    InitReg(MISCREG_CNTHPS_TVAL_EL2)
      .mon(sel2_implemented)
      .hypSecure(sel2_implemented)
      .res0(0xffffffff00000000);
    InitReg(MISCREG_CNTHV_CTL_EL2)
      .mon(vhe_implemented)
      .hyp()
      .res0(0xfffffffffffffff8);
    InitReg(MISCREG_CNTHV_CVAL_EL2)
      .mon(vhe_implemented)
      .hyp(vhe_implemented);
    InitReg(MISCREG_CNTHV_TVAL_EL2)
      .mon(vhe_implemented)
      .hyp(vhe_implemented)
      .res0(0xffffffff00000000);
    InitReg(MISCREG_CNTHVS_CTL_EL2)
      .mon(vhe_implemented && sel2_implemented)
      .hypSecure(vhe_implemented && sel2_implemented)
      .res0(0xfffffffffffffff8);
    InitReg(MISCREG_CNTHVS_CVAL_EL2)
      .mon(vhe_implemented && sel2_implemented)
      .hypSecure(vhe_implemented && sel2_implemented);
    InitReg(MISCREG_CNTHVS_TVAL_EL2)
      .mon(vhe_implemented && sel2_implemented)
      .hypSecure(vhe_implemented && sel2_implemented)
      .res0(0xffffffff00000000);
    // ENDIF Armv8.1-VHE
    InitReg(MISCREG_CNTVOFF_EL2)
      .mon()
      .hyp()
      .mapsTo(MISCREG_CNTVOFF);
    // END Generic Timer (AArch64)
    InitReg(MISCREG_PMEVCNTR0_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR0);
    InitReg(MISCREG_PMEVCNTR1_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR1);
    InitReg(MISCREG_PMEVCNTR2_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR2);
    InitReg(MISCREG_PMEVCNTR3_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR3);
    InitReg(MISCREG_PMEVCNTR4_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR4);
    InitReg(MISCREG_PMEVCNTR5_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR5);
    InitReg(MISCREG_PMEVTYPER0_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER0);
    InitReg(MISCREG_PMEVTYPER1_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER1);
    InitReg(MISCREG_PMEVTYPER2_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER2);
    InitReg(MISCREG_PMEVTYPER3_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER3);
    InitReg(MISCREG_PMEVTYPER4_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER4);
    InitReg(MISCREG_PMEVTYPER5_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER5);
    InitReg(MISCREG_IL1DATA0_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA1_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA2_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA3_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA0_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA1_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA2_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA3_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA4_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2ACTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CPUACTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CPUECTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CPUMERRSR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2MERRSR_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_CBAR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CONTEXTIDR_EL2)
      .mon().hyp();

    // GICv3 AArch64
    InitReg(MISCREG_ICC_PMR_EL1)
        .res0(0xffffff00) // [31:8]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_PMR);
    InitReg(MISCREG_ICC_IAR0_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_IAR0);
    InitReg(MISCREG_ICC_EOIR0_EL1)
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_EOIR0);
    InitReg(MISCREG_ICC_HPPIR0_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_HPPIR0);
    InitReg(MISCREG_ICC_BPR0_EL1)
        .res0(0xfffffff8) // [31:3]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_BPR0);
    InitReg(MISCREG_ICC_AP0R0_EL1)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP0R0);
    InitReg(MISCREG_ICC_AP0R1_EL1)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP0R1);
    InitReg(MISCREG_ICC_AP0R2_EL1)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP0R2);
    InitReg(MISCREG_ICC_AP0R3_EL1)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP0R3);
    InitReg(MISCREG_ICC_AP1R0_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_AP1R0);
    InitReg(MISCREG_ICC_AP1R0_EL1_NS)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R0_NS);
    InitReg(MISCREG_ICC_AP1R0_EL1_S)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R0_S);
    InitReg(MISCREG_ICC_AP1R1_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_AP1R1);
    InitReg(MISCREG_ICC_AP1R1_EL1_NS)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R1_NS);
    InitReg(MISCREG_ICC_AP1R1_EL1_S)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R1_S);
    InitReg(MISCREG_ICC_AP1R2_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_AP1R2);
    InitReg(MISCREG_ICC_AP1R2_EL1_NS)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R2_NS);
    InitReg(MISCREG_ICC_AP1R2_EL1_S)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R2_S);
    InitReg(MISCREG_ICC_AP1R3_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_AP1R3);
    InitReg(MISCREG_ICC_AP1R3_EL1_NS)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R3_NS);
    InitReg(MISCREG_ICC_AP1R3_EL1_S)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R3_S);
    InitReg(MISCREG_ICC_DIR_EL1)
        .res0(0xFF000000) // [31:24]
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_DIR);
    InitReg(MISCREG_ICC_RPR_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_RPR);
    InitReg(MISCREG_ICC_SGI1R_EL1)
        .allPrivileges().exceptUserMode().reads(0)
        .faultWrite(EL1, faultIccSgiEL1)
        .faultWrite(EL2, faultIccSgiEL2)
        .mapsTo(MISCREG_ICC_SGI1R);
    InitReg(MISCREG_ICC_ASGI1R_EL1)
        .allPrivileges().exceptUserMode().reads(0)
        .faultWrite(EL1, faultIccSgiEL1)
        .faultWrite(EL2, faultIccSgiEL2)
        .mapsTo(MISCREG_ICC_ASGI1R);
    InitReg(MISCREG_ICC_SGI0R_EL1)
        .allPrivileges().exceptUserMode().reads(0)
        .faultWrite(EL1, faultIccSgiEL1)
        .faultWrite(EL2, faultIccSgiEL2)
        .mapsTo(MISCREG_ICC_SGI0R);
    InitReg(MISCREG_ICC_IAR1_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_IAR1);
    InitReg(MISCREG_ICC_EOIR1_EL1)
        .res0(0xFF000000) // [31:24]
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_EOIR1);
    InitReg(MISCREG_ICC_HPPIR1_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_HPPIR1);
    InitReg(MISCREG_ICC_BPR1_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_BPR1);
    InitReg(MISCREG_ICC_BPR1_EL1_NS)
        .bankedChild()
        .res0(0xfffffff8) // [31:3]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_BPR1_NS);
    InitReg(MISCREG_ICC_BPR1_EL1_S)
        .bankedChild()
        .res0(0xfffffff8) // [31:3]
        .secure().exceptUserMode()
        .mapsTo(MISCREG_ICC_BPR1_S);
    InitReg(MISCREG_ICC_CTLR_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_CTLR);
    InitReg(MISCREG_ICC_CTLR_EL1_NS)
        .bankedChild()
        .res0(0xFFFB00BC) // [31:19, 17:16, 7, 5:2]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_CTLR_NS);
    InitReg(MISCREG_ICC_CTLR_EL1_S)
        .bankedChild()
        .res0(0xFFFB00BC) // [31:19, 17:16, 7, 5:2]
        .secure().exceptUserMode()
        .mapsTo(MISCREG_ICC_CTLR_S);
    InitReg(MISCREG_ICC_SRE_EL1)
        .banked()
        .mapsTo(MISCREG_ICC_SRE);
    InitReg(MISCREG_ICC_SRE_EL1_NS)
        .bankedChild()
        .res0(0xFFFFFFF8) // [31:3]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_SRE_NS);
    InitReg(MISCREG_ICC_SRE_EL1_S)
        .bankedChild()
        .res0(0xFFFFFFF8) // [31:3]
        .secure().exceptUserMode()
        .mapsTo(MISCREG_ICC_SRE_S);
    InitReg(MISCREG_ICC_IGRPEN0_EL1)
        .res0(0xFFFFFFFE) // [31:1]
        .allPrivileges().exceptUserMode()
        .faultRead(EL1, faultFgtEL1<true, &HFGTR::iccIgrpEnEL1>)
        .faultWrite(EL1, faultFgtEL1<false, &HFGTR::iccIgrpEnEL1>)
        .mapsTo(MISCREG_ICC_IGRPEN0);
    InitReg(MISCREG_ICC_IGRPEN1_EL1)
        .banked64()
        .faultRead(EL1, faultFgtEL1<true, &HFGTR::iccIgrpEnEL1>)
        .faultWrite(EL1, faultFgtEL1<false, &HFGTR::iccIgrpEnEL1>)
        .mapsTo(MISCREG_ICC_IGRPEN1);
    InitReg(MISCREG_ICC_IGRPEN1_EL1_NS)
        .bankedChild()
        .res0(0xFFFFFFFE) // [31:1]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_IGRPEN1_NS);
    InitReg(MISCREG_ICC_IGRPEN1_EL1_S)
        .bankedChild()
        .res0(0xFFFFFFFE) // [31:1]
        .secure().exceptUserMode()
        .mapsTo(MISCREG_ICC_IGRPEN1_S);
    InitReg(MISCREG_ICC_SRE_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICC_HSRE);
    InitReg(MISCREG_ICC_CTLR_EL3)
        .mon()
        .mapsTo(MISCREG_ICC_MCTLR);
    InitReg(MISCREG_ICC_SRE_EL3)
        .mon()
        .mapsTo(MISCREG_ICC_MSRE);
    InitReg(MISCREG_ICC_IGRPEN1_EL3)
        .mon()
        .mapsTo(MISCREG_ICC_MGRPEN1);

    InitReg(MISCREG_ICH_AP0R0_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP0R0);
    InitReg(MISCREG_ICH_AP0R1_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP0R1);
    InitReg(MISCREG_ICH_AP0R2_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP0R2);
    InitReg(MISCREG_ICH_AP0R3_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP0R3);
    InitReg(MISCREG_ICH_AP1R0_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP1R0);
    InitReg(MISCREG_ICH_AP1R1_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP1R1);
    InitReg(MISCREG_ICH_AP1R2_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP1R2);
    InitReg(MISCREG_ICH_AP1R3_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP1R3);
    InitReg(MISCREG_ICH_HCR_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_HCR);
    InitReg(MISCREG_ICH_VTR_EL2)
        .hyp().mon().writes(0)
        .mapsTo(MISCREG_ICH_VTR);
    InitReg(MISCREG_ICH_MISR_EL2)
        .hyp().mon().writes(0)
        .mapsTo(MISCREG_ICH_MISR);
    InitReg(MISCREG_ICH_EISR_EL2)
        .hyp().mon().writes(0)
        .mapsTo(MISCREG_ICH_EISR);
    InitReg(MISCREG_ICH_ELRSR_EL2)
        .hyp().mon().writes(0)
        .mapsTo(MISCREG_ICH_ELRSR);
    InitReg(MISCREG_ICH_VMCR_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_VMCR);
    InitReg(MISCREG_ICH_LR0_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR0, MISCREG_ICH_LRC0);
    InitReg(MISCREG_ICH_LR1_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR1, MISCREG_ICH_LRC1);
    InitReg(MISCREG_ICH_LR2_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR2, MISCREG_ICH_LRC2);
    InitReg(MISCREG_ICH_LR3_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR3, MISCREG_ICH_LRC3);
    InitReg(MISCREG_ICH_LR4_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR4, MISCREG_ICH_LRC4);
    InitReg(MISCREG_ICH_LR5_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR5, MISCREG_ICH_LRC5);
    InitReg(MISCREG_ICH_LR6_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR6, MISCREG_ICH_LRC6);
    InitReg(MISCREG_ICH_LR7_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR7, MISCREG_ICH_LRC7);
    InitReg(MISCREG_ICH_LR8_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR8, MISCREG_ICH_LRC8);
    InitReg(MISCREG_ICH_LR9_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR9, MISCREG_ICH_LRC9);
    InitReg(MISCREG_ICH_LR10_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR10, MISCREG_ICH_LRC10);
    InitReg(MISCREG_ICH_LR11_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR11, MISCREG_ICH_LRC11);
    InitReg(MISCREG_ICH_LR12_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR12, MISCREG_ICH_LRC12);
    InitReg(MISCREG_ICH_LR13_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR13, MISCREG_ICH_LRC13);
    InitReg(MISCREG_ICH_LR14_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR14, MISCREG_ICH_LRC14);
    InitReg(MISCREG_ICH_LR15_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_LR15, MISCREG_ICH_LRC15);

    // GICv3 AArch32
    InitReg(MISCREG_ICC_AP0R0)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP0R1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP0R2)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP0R3)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R0)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R0_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R0_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R1_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R1_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R2)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R2_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R2_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R3)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R3_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R3_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_ASGI1R)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_BPR0)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_BPR1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_BPR1_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_BPR1_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_CTLR)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_CTLR_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_CTLR_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_DIR)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_EOIR0)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_EOIR1)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_HPPIR0)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_HPPIR1)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_HSRE)
        .hyp().mon();
    InitReg(MISCREG_ICC_IAR0)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_IAR1)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_IGRPEN0)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_IGRPEN1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_IGRPEN1_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_IGRPEN1_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_MCTLR)
        .mon();
    InitReg(MISCREG_ICC_MGRPEN1)
        .mon();
    InitReg(MISCREG_ICC_MSRE)
        .mon();
    InitReg(MISCREG_ICC_PMR)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_RPR)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_SGI0R)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_SGI1R)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_SRE)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_SRE_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_SRE_S)
        .allPrivileges().exceptUserMode();

    InitReg(MISCREG_ICH_AP0R0)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP0R1)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP0R2)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP0R3)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP1R0)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP1R1)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP1R2)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP1R3)
        .hyp().mon();
    InitReg(MISCREG_ICH_HCR)
        .hyp().mon();
    InitReg(MISCREG_ICH_VTR)
        .hyp().mon().writes(0);
    InitReg(MISCREG_ICH_MISR)
        .hyp().mon().writes(0);
    InitReg(MISCREG_ICH_EISR)
        .hyp().mon().writes(0);
    InitReg(MISCREG_ICH_ELRSR)
        .hyp().mon().writes(0);
    InitReg(MISCREG_ICH_VMCR)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR0)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR1)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR2)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR3)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR4)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR5)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR6)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR7)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR8)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR9)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR10)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR11)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR12)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR13)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR14)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR15)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC0)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC1)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC2)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC3)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC4)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC5)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC6)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC7)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC8)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC9)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC10)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC11)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC12)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC13)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC14)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC15)
        .hyp().mon();

    // SVE
    InitReg(MISCREG_ID_AA64ZFR0_EL1)
        .reset([this](){
            AA64ZFR0 zfr0_el1 = 0;
            zfr0_el1.f32mm = release->has(ArmExtension::FEAT_F32MM) ? 1 : 0;
            zfr0_el1.f64mm = release->has(ArmExtension::FEAT_F64MM) ? 1 : 0;
            zfr0_el1.i8mm = release->has(ArmExtension::FEAT_I8MM) ? 1 : 0;
            return zfr0_el1;
        }())
        .faultRead(EL0, faultIdst)
        .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ZCR_EL3)
        .reset(sveVL - 1)
        .fault(EL3, faultZcrEL3)
        .mon();
    InitReg(MISCREG_ZCR_EL2)
        .reset(sveVL - 1)
        .fault(EL2, faultZcrEL2)
        .fault(EL3, faultZcrEL3)
        .hyp().mon();
    InitReg(MISCREG_ZCR_EL12)
        .fault(EL2, defaultFaultE2H_EL2)
        .fault(EL3, defaultFaultE2H_EL3)
        .mapsTo(MISCREG_ZCR_EL1);
    InitReg(MISCREG_ZCR_EL1)
        .reset(sveVL - 1)
        .fault(EL1, faultZcrEL1)
        .fault(EL2, faultZcrEL2)
        .fault(EL3, faultZcrEL3)
        .allPrivileges().exceptUserMode();

    // SME
    InitReg(MISCREG_ID_AA64SMFR0_EL1)
        .reset([](){
            AA64SMFR0 smfr0_el1 = 0;
            smfr0_el1.f32f32 = 0x1;
            // The following BF16F32 is actually not implemented due to a
            // lack of BF16 support in gem5's fplib. However, as per the
            // SME spec the _only_ allowed value is 0x1.
            smfr0_el1.b16f32 = 0x1;
            smfr0_el1.f16f32 = 0x1;
            smfr0_el1.i8i32 = 0xF;
            smfr0_el1.f64f64 = 0x1;
            smfr0_el1.i16i64 = 0xF;
            smfr0_el1.smEver = 0;
            smfr0_el1.fa64 = 0x1;
            return smfr0_el1;
        }())
        .faultRead(EL0, faultIdst)
        .faultRead(EL1, faultHcrEL1<&HCR::tid3>)
        .allPrivileges().writes(0);
    InitReg(MISCREG_SVCR)
        .res0([](){
            SVCR svcr_mask = 0;
            svcr_mask.sm = 1;
            svcr_mask.za = 1;
            return ~svcr_mask;
        }())
        .fault(EL0, faultSmenEL0)
        .fault(EL1, faultSmenEL1)
        .fault(EL2, faultTsmSmen)
        .fault(EL3, faultEsm)
        .allPrivileges();
    InitReg(MISCREG_SMIDR_EL1)
        .reset([](){
            SMIDR smidr_el1 = 0;
            smidr_el1.affinity = 0;
            smidr_el1.smps = 0;
            smidr_el1.implementer = 0x41;
            return smidr_el1;
        }())
        .faultRead(EL0, faultIdst)
        .faultRead(EL1, faultHcrEL1<&HCR::tid1>)
        .allPrivileges().writes(0);
    InitReg(MISCREG_SMPRI_EL1)
        .res0(mask(63, 4))
        .fault(EL1, faultEsm)
        .fault(EL2, faultEsm)
        .fault(EL3, faultEsm)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_SMPRIMAP_EL2)
        .fault(EL2, faultEsm)
        .fault(EL3, faultEsm)
        .hyp().mon();
    InitReg(MISCREG_SMCR_EL3)
        .reset([this](){
            // We want to support FEAT_SME_FA64. Therefore, we enable it in
            // all SMCR_ELx registers by default. Runtime software might
            // change this later, but given that gem5 doesn't disable
            // instructions based on this flag we default to the most
            // representative value.
            SMCR smcr_el3 = 0;
            smcr_el3.fa64 = 1;
            smcr_el3.len = smeVL - 1;
            return smcr_el3;
        }())
        .fault(EL3, faultEsm)
        .mon();
    InitReg(MISCREG_SMCR_EL2)
        .reset([this](){
            // We want to support FEAT_SME_FA64. Therefore, we enable it in
            // all SMCR_ELx registers by default. Runtime software might
            // change this later, but given that gem5 doesn't disable
            // instructions based on this flag we default to the most
            // representative value.
            SMCR smcr_el2 = 0;
            smcr_el2.fa64 = 1;
            smcr_el2.len = smeVL - 1;
            return smcr_el2;
        }())
        .fault(EL2, faultTsmSmen)
        .fault(EL3, faultEsm)
        .hyp().mon();
    InitReg(MISCREG_SMCR_EL12)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_SMCR_EL1)
        .reset([this](){
            // We want to support FEAT_SME_FA64. Therefore, we enable it in
            // all SMCR_ELx registers by default. Runtime software might
            // change this later, but given that gem5 doesn't disable
            // instructions based on this flag we default to the most
            // representative value.
            SMCR smcr_el1 = 0;
            smcr_el1.fa64 = 1;
            smcr_el1.len = smeVL - 1;
            return smcr_el1;
        }())
        .fault(EL1, faultSmenEL1)
        .fault(EL2, faultTsmSmen)
        .fault(EL3, faultEsm)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_TPIDR2_EL0)
        .allPrivileges();

    InitReg(MISCREG_RNDR)
        .faultRead(EL0, faultRng)
        .faultRead(EL1, faultRng)
        .faultRead(EL2, faultRng)
        .faultRead(EL3, faultRng)
        .unverifiable()
        .allPrivileges().writes(0);
    InitReg(MISCREG_RNDRRS)
        .faultRead(EL0, faultRng)
        .faultRead(EL1, faultRng)
        .faultRead(EL2, faultRng)
        .faultRead(EL3, faultRng)
        .unverifiable()
        .allPrivileges().writes(0);

    // FEAT_FGT extension
    InitReg(MISCREG_HFGRTR_EL2)
      .fault(EL2, faultFgtCtrlRegs)
      .hyp().mon(release->has(ArmExtension::FEAT_FGT));
    InitReg(MISCREG_HFGWTR_EL2)
      .fault(EL2, faultFgtCtrlRegs)
      .hyp().mon(release->has(ArmExtension::FEAT_FGT));
    InitReg(MISCREG_HFGITR_EL2)
      .fault(EL2, faultFgtCtrlRegs)
      .hyp().mon(release->has(ArmExtension::FEAT_FGT));
    InitReg(MISCREG_HDFGRTR_EL2)
      .fault(EL2, faultFgtCtrlRegs)
      .hyp().mon(release->has(ArmExtension::FEAT_FGT));
    InitReg(MISCREG_HDFGWTR_EL2)
      .fault(EL2, faultFgtCtrlRegs)
      .hyp().mon(release->has(ArmExtension::FEAT_FGT));

    // Dummy registers
    InitReg(MISCREG_NOP)
      .allPrivileges();
    InitReg(MISCREG_RAZ)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_UNKNOWN);
    InitReg(MISCREG_IMPDEF_UNIMPL)
      .fault(EL1, faultImpdefUnimplEL1)
      .fault(EL2, faultUnimplemented)
      .fault(EL3, faultUnimplemented)
      .warnNotFail(impdefAsNop);

    // RAS extension (unimplemented)
    InitReg(MISCREG_ERRIDR_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_ERRSELR_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_ERXFR_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_ERXCTLR_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_ERXSTATUS_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_ERXADDR_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_ERXMISC0_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_ERXMISC1_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_DISR_EL1)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_VSESR_EL2)
      .warnNotFail()
      .fault(faultUnimplemented);
    InitReg(MISCREG_VDISR_EL2)
      .warnNotFail()
      .fault(faultUnimplemented);

    // MPAM extension
    InitReg(MISCREG_MPAMIDR_EL1)
        .reset(p.mpamidr_el1)
        .res0(mask(63, 62) | mask(56, 40) | mask(31, 21) | mask(16, 16))
        .faultRead(EL1, faultMpamIdrEL1)
        .faultRead(EL2, faultMpamEL2)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_MPAM0_EL1)
        .res0(mask(63, 48))
        .fault(EL1, faultMpam0EL1)
        .fault(EL2, faultMpamEL2)
        .priv().hyp().mon();
    InitReg(MISCREG_MPAM1_EL1)
        .res0(mask(62, 61) | mask(59, 48))
        .fault(EL1, faultMpam1EL1)
        .fault(EL2, faultMpamEL2)
        .priv().hyp().mon();
    InitReg(MISCREG_MPAM1_EL12)
        .res0(mask(59, 48))
        .fault(EL2, faultMpam12EL2)
        .fault(EL3, defaultFaultE2H_EL3)
        .hyp().mon();
    InitReg(MISCREG_MPAM2_EL2)
        .res0(mask(62, 59) | mask(57, 50))
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMHCR_EL2)
        .res0(mask(63, 32) | mask(30, 9) | mask(7, 2))
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPM0_EL2)
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPM1_EL2)
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPM2_EL2)
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPM3_EL2)
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPM4_EL2)
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPM5_EL2)
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPM6_EL2)
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPM7_EL2)
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAMVPMV_EL2)
        .res0(mask(63, 32))
        .fault(EL2, faultMpamEL2)
        .hyp().mon();
    InitReg(MISCREG_MPAM3_EL3)
        .res0(mask(59, 48))
        .mon();
    InitReg(MISCREG_MPAMSM_EL1)
        .res0(mask(63, 48) | mask(39, 32) | mask(15, 0))
        .fault(EL1, faultMpamsmEL1)
        .fault(EL2, faultMpamEL2)
        .allPrivileges().exceptUserMode();

    // Register mappings for some unimplemented registers:
    // ESR_EL1 -> DFSR
    // RMR_EL1 -> RMR
    // RMR_EL2 -> HRMR
    // DBGDTR_EL0 -> DBGDTR{R or T}Xint
    // DBGDTRRX_EL0 -> DBGDTRRXint
    // DBGDTRTX_EL0 -> DBGDTRRXint
    // MDCR_EL3 -> SDCR, NAM D7-2108 (the latter is unimpl. in gem5)

    // Populate the idxToMiscRegNum map
    assert(idxToMiscRegNum.empty());
    for (const auto& [key, val] : miscRegNumToIdx) {
        idxToMiscRegNum.insert({val, key});
    }

    completed = true;
}

} // namespace ArmISA
} // namespace gem5

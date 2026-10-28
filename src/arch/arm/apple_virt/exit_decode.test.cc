/*
 * Copyright (c) 2026 The Regents of The University of California
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

#include <gtest/gtest.h>

#include <cstdint>

#include "arch/arm/apple_virt/exit_decode.hh"

namespace gem5
{
namespace ArmISA
{
namespace AppleVirt
{
namespace
{

constexpr uint64_t
makeEsr(uint8_t ec, uint32_t iss)
{
    return (static_cast<uint64_t>(ec) << 26) | (iss & 0x1ffffff);
}

TEST(AppleVirtExitDecode, DecodesWfiAndWfe)
{
    EXPECT_EQ(decodeExit(makeEsr(0x01, 0)).kind, ExitKind::Wfi);
    EXPECT_EQ(decodeExit(makeEsr(0x01, 1)).kind, ExitKind::Wfe);
}

TEST(AppleVirtExitDecode, RejectsUnsupportedExceptionClassesAndWfxTypes)
{
    const auto unknown = decodeExit(makeEsr(0x00, 0x12345));
    EXPECT_EQ(unknown.kind, ExitKind::Unsupported);
    EXPECT_EQ(unknown.ec, 0x00);
    EXPECT_EQ(unknown.iss, 0x12345);

    EXPECT_EQ(decodeExit(makeEsr(0x01, 2)).kind, ExitKind::Unsupported);
    EXPECT_EQ(decodeExit(makeEsr(0x01, 3)).kind, ExitKind::Unsupported);
}

TEST(AppleVirtExitDecode, DecodesSystemRegisterAccess)
{
    constexpr uint32_t iss = (2U << 20) | (4U << 17) | (3U << 14) |
                             (9U << 10) | (17U << 5) | (12U << 1) | 1U;
    const auto exit = decodeExit(makeEsr(0x18, iss));

    EXPECT_EQ(exit.kind, ExitKind::SystemRegister);
    EXPECT_EQ(exit.systemRegister.op0, 2);
    EXPECT_EQ(exit.systemRegister.op1, 3);
    EXPECT_EQ(exit.systemRegister.crn, 9);
    EXPECT_EQ(exit.systemRegister.crm, 12);
    EXPECT_EQ(exit.systemRegister.op2, 4);
    EXPECT_EQ(exit.systemRegister.targetRegister, 17);
    EXPECT_TRUE(exit.systemRegister.read);
}

TEST(AppleVirtExitDecode, DecodesValidDataAbortRead)
{
    constexpr uint32_t iss = (1U << 24) | (2U << 22) | (1U << 21) |
                             (13U << 16) | (1U << 15) | (1U << 14);
    const auto exit = decodeExit(makeEsr(0x24, iss));

    EXPECT_EQ(exit.kind, ExitKind::DataAbort);
    EXPECT_EQ(exit.ec, 0x24);
    EXPECT_EQ(exit.iss, iss);
    EXPECT_TRUE(exit.dataAbort.valid);
    EXPECT_FALSE(exit.dataAbort.write);
    EXPECT_EQ(exit.dataAbort.size, 4);
    EXPECT_TRUE(exit.dataAbort.signExtend);
    EXPECT_EQ(exit.dataAbort.targetRegister, 13);
    EXPECT_TRUE(exit.dataAbort.sf);
    EXPECT_TRUE(exit.dataAbort.acquireRelease);
    EXPECT_FALSE(exit.dataAbort.cacheMaintenance);
    EXPECT_FALSE(exit.dataAbort.s1ptw);
}

TEST(AppleVirtExitDecode, DecodesValidDataAbortWrite)
{
    constexpr uint32_t iss =
        (1U << 24) | (3U << 22) | (31U << 16) | (1U << 8) | (1U << 6);
    const auto exit = decodeExit(makeEsr(0x25, iss));

    EXPECT_EQ(exit.kind, ExitKind::DataAbort);
    EXPECT_TRUE(exit.dataAbort.valid);
    EXPECT_TRUE(exit.dataAbort.write);
    EXPECT_EQ(exit.dataAbort.size, 8);
    EXPECT_FALSE(exit.dataAbort.signExtend);
    EXPECT_EQ(exit.dataAbort.targetRegister, 31);
    EXPECT_FALSE(exit.dataAbort.sf);
    EXPECT_FALSE(exit.dataAbort.acquireRelease);
    EXPECT_TRUE(exit.dataAbort.cacheMaintenance);
    EXPECT_FALSE(exit.dataAbort.s1ptw);
}

TEST(AppleVirtExitDecode, KeepsIndependentFieldsForInvalidSyndrome)
{
    constexpr uint32_t iss = (3U << 22) | (1U << 21) | (17U << 16) |
                             (1U << 15) | (1U << 14) | (1U << 8) | (1U << 7) |
                             (1U << 6);
    const auto exit = decodeExit(makeEsr(0x24, iss));

    EXPECT_EQ(exit.kind, ExitKind::DataAbort);
    EXPECT_FALSE(exit.dataAbort.valid);
    EXPECT_TRUE(exit.dataAbort.write);
    EXPECT_EQ(exit.dataAbort.size, 0);
    EXPECT_FALSE(exit.dataAbort.signExtend);
    EXPECT_EQ(exit.dataAbort.targetRegister, 0);
    EXPECT_FALSE(exit.dataAbort.sf);
    EXPECT_FALSE(exit.dataAbort.acquireRelease);
    EXPECT_TRUE(exit.dataAbort.cacheMaintenance);
    EXPECT_TRUE(exit.dataAbort.s1ptw);
}

} // anonymous namespace
} // namespace AppleVirt
} // namespace ArmISA
} // namespace gem5

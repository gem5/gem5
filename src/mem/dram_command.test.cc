/*
 * Copyright (c) 2026 CkingX256
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

#include "mem/dram_command.hh"

using gem5::memory::DramCommandInfo;
using Type = DramCommandInfo::Type;
using Scope = DramCommandInfo::Scope;

TEST(DramCommand, RankCommandsDoNotInventAddresses)
{
    for (auto type : {Type::REF, Type::PREA}) {
        DramCommandInfo event(type, 20, 30, 1);
        EXPECT_EQ(event.scope(), Scope::Rank);
        EXPECT_FALSE(event.bank.has_value());
        EXPECT_FALSE(event.row.has_value());
        EXPECT_THROW(DramCommandInfo(type, 20, 30, 1, 0),
                     std::invalid_argument);
    }
}

TEST(DramCommand, RowCommandsRequireActualRowIncludingZero)
{
    for (auto type : {Type::ACT, Type::RD, Type::WR}) {
        DramCommandInfo event(type, 20, 30, 1, 0, 0);
        EXPECT_EQ(event.scope(), Scope::Row);
        EXPECT_EQ(event.row.value(), 0);
        EXPECT_THROW(DramCommandInfo(type, 20, 30, 1, 0),
                     std::invalid_argument);
    }
}

TEST(DramCommand, AutoPrechargeIsAnImplicitBankOperation)
{
    DramCommandInfo event(Type::PRE, 20, 30, 1, 2, {}, true);
    EXPECT_EQ(event.scope(), Scope::Bank);
    EXPECT_TRUE(event.implicit);
    EXPECT_FALSE(event.row.has_value());
    EXPECT_THROW(DramCommandInfo(Type::PRE, 20, 30, 1, 2, 0),
                 std::invalid_argument);
    EXPECT_THROW(DramCommandInfo(Type::RD, 20, 30, 1, 2, 0, true),
                 std::invalid_argument);
}

TEST(DramCommand, FutureStartIsDistinctFromSchedulingTime)
{
    DramCommandInfo event(Type::ACT, 20, 30, 1, 2, 7);
    EXPECT_EQ(event.scheduledAt, 20);
    EXPECT_EQ(event.commandAt, 30);
    EXPECT_THROW(DramCommandInfo(Type::ACT, 30, 20, 1, 2, 7),
                 std::invalid_argument);
}

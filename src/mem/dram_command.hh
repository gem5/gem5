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

#ifndef __MEM_DRAM_COMMAND_HH__
#define __MEM_DRAM_COMMAND_HH__

#include <optional>
#include <stdexcept>

#include "base/types.hh"

namespace gem5
{

namespace memory
{

/** A scheduling observation, not a callback at command execution time.
 *
 * commandAt is the modeled command start, not its completion. Notifications
 * may describe future commands and need not arrive in commandAt order.
 * Addresses are local to the observed DRAMInterface. No subarray or column
 * is reported because DRAMInterface does not resolve those coordinates.
 * A row-scoped RD/WR identifies the addressed row, not a transfer of every
 * bit in that row. This interface does not describe data values or bit ranges.
 */
struct DramCommandInfo
{
    enum class Type
    {
        ACT,
        PRE,
        RD,
        WR,
        REF,
        PREA
    };
    enum class Scope
    {
        Row,
        Bank,
        Rank
    };

    const Type command;
    const Tick scheduledAt;
    const Tick commandAt;
    const uint8_t rank;
    const std::optional<uint8_t> bank;
    const std::optional<uint32_t> row;
    // A PRE used to model auto-precharge, rather than a bus command.
    const bool implicit;

    DramCommandInfo(Type command, Tick scheduled_at, Tick command_at,
                    uint8_t rank, std::optional<uint8_t> bank = {},
                    std::optional<uint32_t> row = {}, bool implicit = false)
        : command(command),
          scheduledAt(scheduled_at),
          commandAt(command_at),
          rank(rank),
          bank(bank),
          row(row),
          implicit(implicit)
    {
        const bool rank_wide = command == Type::REF || command == Type::PREA;
        const bool row_wide =
            command == Type::ACT || command == Type::RD || command == Type::WR;
        if (command_at < scheduled_at || (rank_wide && (bank || row)) ||
            (!rank_wide && !bank) || (row_wide != row.has_value()) ||
            (implicit && command != Type::PRE)) {
            throw std::invalid_argument("Invalid DRAM command observation");
        }
    }

    Scope
    scope() const
    {
        return row ? Scope::Row : (bank ? Scope::Bank : Scope::Rank);
    }

    const char *
    name() const
    {
        switch (command) {
            case Type::ACT:
                return "ACT";
            case Type::PRE:
                return "PRE";
            case Type::RD:
                return "RD";
            case Type::WR:
                return "WR";
            case Type::REF:
                return "REF";
            case Type::PREA:
                return "PREA";
        }
        return "unknown";
    }
};

} // namespace memory
} // namespace gem5

#endif // __MEM_DRAM_COMMAND_HH__

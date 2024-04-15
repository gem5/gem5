/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DEV_AMDGPU_MMIO_READER_HH__
#define __DEV_AMDGPU_MMIO_READER_HH__

#include <cstdint>
#include <list>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "base/logging.hh"
#include "mem/packet.hh"

namespace gem5
{

/**
 * Helper class to read Linux kernel MMIO trace from amdgpu modprobes. This
 * class is used rather than implementing MMIOs in code as it is easier to
 * update to never kernel versions this way. It also helps with setting values
 * for registers which are not documented.
 *
 * The class is designed to be able to read both raw MMIO input traces as well
 * as traces that have been filtered by the util script to reduce the file
 * size.
 *
 * An MMIO trace is provided with the gem5 release. To see instructions on how
 * to generate the file yourself, see the documentation on the MMIO trace
 * generation script provided in util.
 */
class AMDMMIOReader
{
  private:
    /**
     * These are the BAR values from the system where the trace is collected.
     * If you have collected your own trace, you may need to change these BARs
     * for the MMIO trace to be read correctly!
     */
    const uint64_t BAR0 = 0x2400000000;
    const uint64_t BAR2 = 0x2200000000;
    const uint32_t BAR5 = 0xecf00000;
    const uint32_t ROM = 0xc0000;

    /* Sizes based on Vega Frontier Edition */
    const uint64_t BAR0_SIZE = 0x400000000; // 16GB
    const uint64_t BAR2_SIZE = 0x200000;    // 2MB
    const uint32_t BAR5_SIZE = 0x80000;     // 512kB
    const uint32_t ROM_SIZE = 0x20000;      // 128kB

    /**
     * The information we want from each relevant line of trace are:
     *  (1) The BAR number where the accessed address is mapped to.
     *  (2) The offset from the BAR.
     *  (3) Type of the access, if it's Read/Write/Unknown
     *  (4) Data from the access if available (not for Unknown).
     *  (5) An index representing the order of trace.
     */

    /* trace_entry containing (3), (4), and (5). */
    typedef std::tuple<uint64_t, std::tuple<char, uint64_t>> trace_entry_t;

    /* Trace entries are recorded as a list for each offset in a given BAR. */
    typedef std::unordered_map<uint32_t, std::list<trace_entry_t>> trace_BAR_t;

    /* There are 7 BARs (BAR0-BAR5 + expansion ROM) */
    trace_BAR_t trace_BARs[6];

    /* Indexes used to print driver loading progress. */
    uint64_t trace_index = 0;
    uint64_t trace_final_index = 0;
    uint64_t trace_cur_index = 0;

    /* An entry in the MMIO trace. */
    struct MmioTrace
    {
        char event;
        uint16_t size;
        uint16_t bar;
        uint64_t addr;
        uint64_t data;
        uint64_t index;
    } mtrace;

    /* Lines in the MMIO trace we care about begin with R, W, or UNKNOWN. */
    bool
    traceIsRead(std::vector<std::string> tokens) const
    {
        return tokens[0] == "R";
    }

    bool
    traceIsWrite(std::vector<std::string> tokens) const
    {
        return tokens[0] == "W";
    }

    bool
    traceIsUnknown(std::vector<std::string> tokens) const
    {
        return tokens[0] == "UNKNOWN";
    }

    /* Checks if this line of trace is W/R/UNKNOWN */
    bool
    isIO(std::vector<std::string> tokens) const
    {
        return tokens[0] == "R" || tokens[0] == "W" || tokens[0] == "UNKNOWN";
    }

    /* Checks if this line of trace is in a BAR we care about (0, 2, 5) */
    bool
    isRelevant(std::vector<std::string> tokens)
    {
        uint64_t addr = strtoull(tokens[4].c_str(), nullptr, 16);
        uint16_t bar = traceGetBAR(addr);
        return (bar == 0 || bar == 2 || bar == 5);
    }

    uint8_t
    traceGetBAR(uint64_t addr)
    {
        if (BAR0 <= addr && addr < (BAR0 + BAR0_SIZE)) {
            return 0;
        } else if (BAR2 <= addr && addr < (BAR2 + BAR2_SIZE)) {
            return 2;
        } else if (BAR5 <= addr && addr < (BAR5 + BAR5_SIZE)) {
            return 5;
        } else if (ROM <= addr && addr < (ROM + ROM_SIZE)) {
            return 6;
        } else {
            return -1;
        }
    }

    uint64_t
    traceGetOffset(uint64_t addr)
    {
        if (addr >= BAR0 && addr < (BAR0 + BAR0_SIZE)) {
            return addr - BAR0;
        } else if (addr >= BAR2 && addr < (BAR2 + BAR2_SIZE)) {
            return addr - BAR2;
        } else if (addr >= BAR5 && addr < (BAR5 + BAR5_SIZE)) {
            return addr - BAR5;
        } else if (addr >= ROM && addr < (ROM + ROM_SIZE)) {
            return addr - ROM;
        } else {
            panic("Can't find offset for the address in MMIO trace!");
        }
    }

    void
    traceParseTokens(std::vector<std::string> tokens)
    {
        if (traceIsRead(tokens) || traceIsWrite(tokens)) {
            mtrace.event = traceIsRead(tokens) ? 'R' : 'W';
            mtrace.size = strtoul(tokens[1].c_str(), nullptr, 10);
            mtrace.addr = strtoull(tokens[4].c_str(), nullptr, 16);
            mtrace.data = strtoull(tokens[5].c_str(), nullptr, 16);
        } else if (traceIsUnknown(tokens)) {
            mtrace.event = 'U';
            mtrace.size = 0;
            mtrace.addr = strtoull(tokens[3].c_str(), nullptr, 16);
            mtrace.data = 0;
        }
        mtrace.bar = traceGetBAR(mtrace.addr);

        mtrace.index = trace_index;
        trace_index++;
    }

    void
    recordMtrace()
    {
        trace_entry_t trace_entry;
        trace_entry = std::make_tuple(
            mtrace.index, std::make_tuple(mtrace.event, mtrace.data));

        uint16_t barnum = mtrace.bar;
        uint64_t offset = traceGetOffset(mtrace.addr);

        trace_BARs[barnum][offset].push_back(trace_entry);
    }

  public:
    AMDMMIOReader() {}

    ~AMDMMIOReader() {}

    /**
     * Read an MMIO trace gathered from a real system and place the MMIO
     * values read and written into the MMIO trace entry map.
     *
     * @param trace_file Absolute path of MMIO trace file to read.
     */
    void readMMIOTrace(std::string trace_file);

    /**
     * Get the next MMIO read from the trace file to an offset in a BAR and
     * write the value to the packet provided.
     *
     * @param pkt Packet to write the MMIO trace value
     * @param barnum The BAR of the MMIO read
     * @param offset The offset in the BAR of the MMIO read.
     */
    void readFromTrace(PacketPtr pkt, int barnum, Addr offset);

    /**
     * Get the next MMIO write from the trace file to an offset in a BAR and
     * compare the value with the data in the packet provided. This is only
     * used for debugging and is otherwise intercepted by MMIO interface.
     *
     * @param pkt Packet to write the MMIO trace value
     * @param barnum The BAR of the MMIO read
     * @param offset The offset in the BAR of the MMIO read.
     */
    void writeFromTrace(PacketPtr pkt, int barnum, Addr offset);
};

} // namespace gem5

#endif // __DEV_AMDGPU_MMIO_READER_HH__

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

#include "dev/amdgpu/mmio_reader.hh"

#include <fstream>

#include "base/trace.hh"
#include "debug/AMDGPUDevice.hh"
#include "mem/packet_access.hh"

namespace gem5
{

void
AMDMMIOReader::readMMIOTrace(std::string trace_file)
{
    std::ifstream tracefile(trace_file);
    std::string line, token;

    mtrace.event = 0;
    mtrace.size = 0;
    mtrace.bar = 0;
    mtrace.addr = 0;
    mtrace.data = 0;

    trace_index = 0;

    while (std::getline(tracefile, line)) {
        std::stringstream l(line);
        std::vector <std::string> tokens;

        while (std::getline(l, token, ' '))
            tokens.push_back(token);

       if (traceIsRead(tokens) && isRelevant(tokens)) {
           traceParseTokens(tokens);
           if (trace_index > trace_cur_index) {
               recordMtrace();
           }
        }
    }

    trace_final_index = trace_index;
}

void
AMDMMIOReader::readFromTrace(PacketPtr pkt, int barnum, Addr offset)
{
    uint64_t value = 0;

    /* If the offset exists for this BAR, return the value, otherwise 0. */
    if (trace_BARs[barnum].count(offset) > 0 &&
       trace_BARs[barnum][offset].size() > 0) {

        value =  std::get<1>(std::get<1>(trace_BARs[barnum][offset].front()));
        DPRINTF(AMDGPUDevice, "Read MMIO %d\n", trace_cur_index);
        DPRINTF(AMDGPUDevice, "Reading from trace with offset: %#x on BAR %#x"
                             ". Progress is: %#f\n", offset, barnum,
                             float(trace_cur_index)/float(trace_final_index));

        /* Leave at least one value for this offset. */
        if (trace_BARs[barnum][offset].size() > 1) {
            trace_BARs[barnum][offset].pop_front();
        }
        trace_cur_index++;
    }

    // Write the read value to the packet
    pkt->setUintX(value, ByteOrder::little);
}

void
AMDMMIOReader::writeFromTrace(PacketPtr pkt, int barnum, Addr offset)
{
    /* If the offset exists for this BAR, verify the value, otherwise 0. */
    if (trace_BARs[barnum].count(offset) > 0 &&
       trace_BARs[barnum][offset].size() > 0 &&
       trace_cur_index == std::get<0>(trace_BARs[barnum][offset].front())) {

        DPRINTF(AMDGPUDevice, "Write matches trace with offset: %#x on "
                             "BAR %#x. Progress is: %#f\n", offset, barnum,
                             float(trace_cur_index)/float(trace_final_index));
        trace_BARs[barnum][offset].pop_front();
        trace_cur_index++;
    }
}

} // namespace gem5

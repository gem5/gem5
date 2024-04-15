/*
 * Copyright (c) 2013 ARM Limited
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

#include <cassert>

#include "mem/dramsim2_wrapper.hh"

#include <fstream>

#include "DRAMSim2/MultiChannelMemorySystem.h"
#include "base/compiler.hh"
#include "base/logging.hh"

/**
 * DRAMSim2 requires SHOW_SIM_OUTPUT to be defined (declared extern in
 * the DRAMSim2 print macros), otherwise we get linking errors due to
 * undefined references
 */
int SHOW_SIM_OUTPUT = 0;

namespace gem5
{

namespace memory
{

DRAMSim2Wrapper::DRAMSim2Wrapper(const std::string &config_file,
                                 const std::string &system_file,
                                 const std::string &working_dir,
                                 const std::string &trace_file,
                                 unsigned int memory_size_mb,
                                 bool enable_debug)
    : dramsim(new DRAMSim::MultiChannelMemorySystem(
          config_file, system_file, working_dir, trace_file, memory_size_mb,
          NULL, NULL)),
      _clockPeriod(0.0),
      _queueSize(0),
      _burstSize(0)
{
    // tell DRAMSim2 to ignore its internal notion of a CPU frequency
    dramsim->setCPUClockSpeed(0);

    // switch on debug output if requested
    if (enable_debug)
        SHOW_SIM_OUTPUT = 1;

    // there is no way of getting DRAMSim2 to tell us what frequency
    // it is assuming, so we have to extract it ourselves
    _clockPeriod =
        extractConfig<double>("tCK=", working_dir + '/' + config_file);

    if (!_clockPeriod)
        fatal("DRAMSim2 wrapper failed to get clock\n");

    // we also need to know what transaction queue size DRAMSim2 is
    // using so we can stall when responses are blocked
    _queueSize = extractConfig<unsigned int>("TRANS_QUEUE_DEPTH=",
                                             working_dir + '/' + system_file);

    if (!_queueSize)
        fatal("DRAMSim2 wrapper failed to get queue size\n");

    // finally, get the data bus bits and burst length so we can add a
    // sanity check for the burst size
    unsigned int dataBusBits = extractConfig<unsigned int>(
        "JEDEC_DATA_BUS_BITS=", working_dir + '/' + system_file);
    unsigned int burstLength =
        extractConfig<unsigned int>("BL=", working_dir + '/' + config_file);

    if (!dataBusBits || !burstLength)
        fatal("DRAMSim22 wrapper failed to get burst size\n");

    _burstSize = dataBusBits * burstLength / 8;
}

DRAMSim2Wrapper::~DRAMSim2Wrapper() { delete dramsim; }

template <typename T>
T
DRAMSim2Wrapper::extractConfig(const std::string &field_name,
                               const std::string &file_name) const
{
    std::ifstream file_stream(file_name.c_str(), ios::in);

    if (!file_stream.good())
        fatal("DRAMSim2 wrapper could not open %s for reading\n", file_name);

    bool found = false;
    T res;
    std::string line;
    while (!found && file_stream) {
        getline(file_stream, line);
        if (line.substr(0, field_name.size()) == field_name) {
            found = true;
            istringstream iss(line.substr(field_name.size()));
            iss >> res;
        }
    }

    file_stream.close();

    if (!found)
        fatal("DRAMSim2 wrapper could not find %s in %s\n", field_name,
              file_name);

    return res;
}

void
DRAMSim2Wrapper::printStats()
{
    dramsim->printStats(true);
}

void
DRAMSim2Wrapper::setCallbacks(DRAMSim::TransactionCompleteCB *read_callback,
                              DRAMSim::TransactionCompleteCB *write_callback)
{
    // simply pass it on, for now we ignore the power callback
    dramsim->RegisterCallbacks(read_callback, write_callback, NULL);
}

bool
DRAMSim2Wrapper::canAccept() const
{
    return dramsim->willAcceptTransaction();
}

void
DRAMSim2Wrapper::enqueue(bool is_write, uint64_t addr)
{
    [[maybe_unused]] bool success = dramsim->addTransaction(is_write, addr);
    assert(success);
}

double
DRAMSim2Wrapper::clockPeriod() const
{
    return _clockPeriod;
}

unsigned int
DRAMSim2Wrapper::queueSize() const
{
    return _queueSize;
}

unsigned int
DRAMSim2Wrapper::burstSize() const
{
    return _burstSize;
}

void
DRAMSim2Wrapper::tick()
{
    dramsim->update();
}

} // namespace memory
} // namespace gem5

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
 *
 */

/**
 * @file
 * DRAMsim3Wrapper declaration
 */

#ifndef __MEM_DRAMSIM3_WRAPPER_HH__
#define __MEM_DRAMSIM3_WRAPPER_HH__

#include <functional>
#include <string>

/**
 * Forward declaration to avoid includes
 */
namespace dramsim3 {

class MemorySystem;

}

/**
 * Wrapper class to avoid having DRAMsim3 names like ClockDomain etc
 * clashing with the normal gem5 world. Many of the DRAMsim3 headers
 * do not make use of namespaces, and quite a few also open up
 * std. The only thing that needs to be exposed externally are the
 * callbacks. This wrapper effectively avoids clashes by not including
 * any of the conventional gem5 headers (e.g. Packet or SimObject).
 */
class DRAMsim3Wrapper
{

  private:

    dramsim3::MemorySystem* dramsim;

    double _clockPeriod;

    unsigned int _queueSize;

    unsigned int _burstSize;

    template <typename T>
    T extractConfig(const std::string& field_name,
                    const std::string& file_name) const;

  public:

    /**
     * Create an instance of the DRAMsim3 multi-channel memory
     * controller using a specific config and system description.
     *
     * @param config_file Memory config file
     * @param working_dir Path pre-pended to config files
     */
    DRAMsim3Wrapper(const std::string& config_file,
                    const std::string& working_dir,
                    std::function<void(uint64_t)> read_cb,
                    std::function<void(uint64_t)> write_cb);
    ~DRAMsim3Wrapper();

    /**
     * Print the stats gathered in DRAMsim3.
     */
    void printStats();

    /**
     * Reset stats (useful for fastforwarding switch)
     */
    void resetStats();

    /**
     * Set the callbacks to use for read and write completion.
     *
     * @param read_callback Callback used for read completions
     * @param write_callback Callback used for write completions
     */
    void setCallbacks(std::function<void(uint64_t)> read_complete,
                      std::function<void(uint64_t)> write_complete);

    /**
     * Determine if the controller can accept a new packet or not.
     *
     * @return true if the controller can accept transactions
     */
    bool canAccept(uint64_t addr, bool is_write) const;

    /**
     * Enqueue a packet. This assumes that canAccept has returned true.
     *
     * @param pkt Packet to turn into a DRAMsim3 transaction
     */
    void enqueue(uint64_t addr, bool is_write);

    /**
     * Get the internal clock period used by DRAMsim3, specified in
     * ns.
     *
     * @return The clock period of the DRAM interface in ns
     */
    double clockPeriod() const;

    /**
     * Get the transaction queue size used by DRAMsim3.
     *
     * @return The queue size counted in number of transactions
     */
    unsigned int queueSize() const;

    /**
     * Get the burst size in bytes used by DRAMsim3.
     *
     * @return The burst size in bytes (data width * burst length)
     */
    unsigned int burstSize() const;

    /**
     * Progress the memory controller one cycle
     */
    void tick();
};

#endif //__MEM_DRAMSIM3_WRAPPER_HH__

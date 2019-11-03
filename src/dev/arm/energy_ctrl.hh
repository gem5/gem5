/*
 * Copyright (c) 2012-2014 ARM Limited
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
 * Authors: Vasileios Spiliopoulos
 *          Akash Bagdia
 *          Stephan Diestelhorst
 */

/**
 * @file
 * The energy controller is a device being used to manage power and energy
 * related control operations within the system. It provides the necessary
 * software interface to the kernel. The kernel will require gem5 specific
 * drivers to access this device.
 *
 * Tasks handled by the controller are:
 * a) Dynamic voltage and frequency scaling control operations
 *
 * Note that the registers defined do not resemble any specific controller
 * device in real hardware. They are currently design to accomodate the gem5
 * system requirements.
 */

#ifndef __DEV_ARM_ENERGY_CTRL_HH__
#define __DEV_ARM_ENERGY_CTRL_HH__

#include "dev/io_device.hh"
#include "params/EnergyCtrl.hh"

class DVFSHandler;

class EnergyCtrl : public BasicPioDevice
{
  public:
    /**
     * Discovery flows:
     * ----------------
     *   * get basic DVFS handler information
     *     read(DVFS_HANDLER_STATUS)
     *     read(DVFS_HANDLER_TRANS_LATENCY)
     *
     *   * get the number of domain IDs
     *     read(DVFS_NUM_DOMAINS) -> domains
     *
     *   * query the driver to get the IDs for all i in domains
     *     write(DVFS_DOMAINID_AT_INDEX <- i)
     *     read(DOMAIN_ID) -> domainID_i
     *
     *   * for each domainID i get voltage / frequency pairs
     *     write(DOMAIN_ID <- domainID_i)
     *     read(NUM_OF_PERF_LEVELS) -> levels_i
     *     * for each l in levels_i
     *       write(PERF_LEVEL_TO_READ <- l)
     *       read(FREQ_AT_PERF_LEVEL) -> freq_l_i
     *       read(VOLT_AT_PERF_LEVEL) -> volt_l_i
     *
     *
     * Setting a specific performance level (V/F combination)
     * ------------------------------------------------------
     *   * get performance for domain_ID i
     *     write(DOMAIN_ID <- i)
     *     read(PERF_LEVEL) -> perf_level_i
     *
     *   * set performance for domain_ID i
     *     write(DOMAIN_ID <- i)
     *     write(PERF_LEVEL <- perf_level_i)
     *     * wait for DVFS transition completion
     *       while (!read(PERF_LEVEL_ACK));
     */

    enum Registers {
        DVFS_HANDLER_STATUS = 0,
        DVFS_NUM_DOMAINS,
        DVFS_DOMAINID_AT_INDEX,
        DVFS_HANDLER_TRANS_LATENCY,
        DOMAIN_ID,
        PERF_LEVEL,
        PERF_LEVEL_ACK,
        NUM_OF_PERF_LEVELS,
        PERF_LEVEL_TO_READ,
        FREQ_AT_PERF_LEVEL,
        VOLT_AT_PERF_LEVEL,
        PIO_NUM_FIELDS
    };

    typedef EnergyCtrlParams Params;
    EnergyCtrl(const Params *p);

    /**
     * Read command sent to the device
     * @param pkt Packet describing this request
     * @return number of ticks it took to complete
     */
    Tick read(PacketPtr pkt) override;
    /**
     * Write command sent to the device
     * @param pkt Packet describing this request
     * @return number of ticks it took to complete
     */
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void startup() override;
    void init() override;

  private:
    DVFSHandler *dvfsHandler;

    /**
     * Cluster ID (DOMAIN_ID) R/W register, programmed to ID of the domain for
     * which the set/get performance level command can be issued
     */
    uint32_t domainID;

    /**
     * Index for getting the domain ID from the domain ID list available with
     * the DVFS handler
     */
    uint32_t domainIDIndexToRead;

    /**
     * Acknowledgment (PERF_LEVEL_ACK) RO register, software polls this
     * register to read back the status of the last programmed change in the
     * domain ID and/or the performance level. Valid values are:
     * '0' - Ack is not OK yet
     * '1' - Ack is OK
     * It is a read destructive register with a read of '1' resets the ack to
     * '0'.
     */
    uint32_t perfLevelAck;

    uint32_t perfLevelToRead;

    static uint32_t ticksTokHz(Tick period) {
        return (uint32_t)(SimClock::Int::ms / period);
    }

    static uint32_t toMicroVolt(double voltage) {
        return (uint32_t)(voltage * 1000000);
    }

    /**
      * Update the acknowledgment that is read back by the software to confirm
      * newly requested performance level has been set.
     */
    void updatePLAck() {
        perfLevelAck = 1;
    }

    EventFunctionWrapper updateAckEvent;
};
#endif //__DEV_ARM_ENERGY_CTRL_HH__

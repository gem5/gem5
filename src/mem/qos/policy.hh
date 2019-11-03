/*
 * Copyright (c) 2018 ARM Limited
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
 * Author: Matteo Andreozzi
 */

#ifndef __MEM_QOS_POLICY_HH__
#define __MEM_QOS_POLICY_HH__

#include "base/trace.hh"
#include "debug/QOS.hh"
#include "mem/qos/mem_ctrl.hh"
#include "mem/packet.hh"
#include "sim/system.hh"

namespace QoS {

/**
 * QoS Policy base class
 *
 * QoS Policy base class: all QoS policies derive from this class to
 * implement specific QoS scheduling algorithms
 *
 */
class Policy : public SimObject
{
  public:
    using Params = QoSPolicyParams;
    Policy(const Params* p);

    virtual ~Policy();

    virtual void regStats() override {};

    virtual void init() override {};

    /**
     * Setting a pointer to the Memory Controller implementing
     * the policy.
     */
    void setMemCtrl(MemCtrl* mem) { memCtrl = mem; };

    /**
     * Builds a MasterID/value pair given a master input.
     * This will be lookuped in the system list of masters in order
     * to retrieve the associated MasterID.
     * In case the master name/object cannot be resolved, the pairing
     * method will panic.
     *
     * @param master Master to lookup in the system
     * @param value Value to be associated with the MasterID
     * @return A MasterID/Value pair.
     */
    template <typename M, typename T>
    std::pair<MasterID, T> pair(M master, T value);

    /**
     * Schedules data - must be defined by derived class
     *
     * @param mId master id to schedule
     * @param data data to schedule
     * @return QoS priority value
     */
    virtual uint8_t schedule(const MasterID mId, const uint64_t data) = 0;

    /**
     * Schedules a packet. Non virtual interface for the scheduling
     * method requiring a master ID.
     *
     * @param pkt pointer to packet to schedule
     * @return QoS priority value
     */
    uint8_t schedule(const PacketPtr pkt);

  protected:
    /** Pointer to parent memory controller implementing the policy */
    MemCtrl* memCtrl;
};

template <typename M, typename T>
std::pair<MasterID, T>
Policy::pair(M master, T value)
{
    auto id = memCtrl->system()->lookupMasterId(master);

    panic_if(id == Request::invldMasterId,
             "Unable to find master %s\n", master);

    DPRINTF(QOS,
            "Master %s [id %d] associated with QoS data %d\n",
            master, id, value);

    return std::pair<MasterID, T>(id, value);
}

} // namespace QoS

#endif /* __MEM_QOS_POLICY_HH__ */

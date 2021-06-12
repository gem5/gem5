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
 */

#ifndef __MEM_QOS_POLICY_HH__
#define __MEM_QOS_POLICY_HH__

#include <cstdint>
#include <utility>

#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/QOS.hh"
#include "mem/qos/mem_ctrl.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace gem5
{

struct QoSPolicyParams;

namespace memory
{

GEM5_DEPRECATED_NAMESPACE(QoS, qos);
namespace qos
{

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
    Policy(const Params &p);

    virtual ~Policy();

    virtual void regStats() override {};

    virtual void init() override {};

    /**
     * Setting a pointer to the Memory Controller implementing
     * the policy.
     */
    void setMemCtrl(MemCtrl* mem) { memCtrl = mem; };

    /**
     * Builds a RequestorID/value pair given a requestor input.
     * This will be looked up in the system list of requestors in order
     * to retrieve the associated RequestorID.
     * In case the requestor name/object cannot be resolved, the pairing
     * method will panic.
     *
     * @param requestor Requestor to lookup in the system
     * @param value Value to be associated with the RequestorID
     * @return A RequestorID/Value pair.
     */
    template <typename Requestor, typename T>
    std::pair<RequestorID, T> pair(Requestor requestor, T value);

    /**
     * Schedules data - must be defined by derived class
     *
     * @param requestor_id requestor id to schedule
     * @param data data to schedule
     * @return QoS priority value
     */
    virtual uint8_t schedule(const RequestorID requestor_id,
                              const uint64_t data) = 0;

    /**
     * Schedules a packet. Non virtual interface for the scheduling
     * method requiring a requestor id.
     *
     * @param pkt pointer to packet to schedule
     * @return QoS priority value
     */
    uint8_t schedule(const PacketPtr pkt);

  protected:
    /** Pointer to parent memory controller implementing the policy */
    MemCtrl* memCtrl;
};

template <typename Requestor, typename T>
std::pair<RequestorID, T>
Policy::pair(Requestor requestor, T value)
{
    auto id = memCtrl->system()->lookupRequestorId(requestor);

    panic_if(id == Request::invldRequestorId,
             "Unable to find requestor %s\n", requestor);

    DPRINTF(QOS,
            "Requestor %s [id %d] associated with QoS data %d\n",
            requestor, id, value);

    return std::pair<RequestorID, T>(id, value);
}

} // namespace qos
} // namespace memory
} // namespace gem5

#endif /* __MEM_QOS_POLICY_HH__ */

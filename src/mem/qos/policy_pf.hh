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

#ifndef __MEM_QOS_POLICY_PF_HH__
#define __MEM_QOS_POLICY_PF_HH__

#include <vector>

#include "base/compiler.hh"
#include "mem/qos/policy.hh"
#include "mem/request.hh"

namespace gem5
{

struct QoSPropFairPolicyParams;

namespace memory
{

namespace qos
{

/**
 * Proportional Fair QoS Policy
 * Providing a configurable fair scheduling policy based on
 * utilization; utilization is directly proportional to a
 * score which is inversely proportional to the QoS priority
 * Users can tune the policy by adjusting the weight parameter
 * (weight of the formula)
 *
 * This is the formula used by the policy
 * ((1.0 - weight) * old_score) + (weight * served_bytes);
 */
class PropFairPolicy : public Policy
{
    using Params = QoSPropFairPolicyParams;

  public:
    PropFairPolicy(const Params &);
    virtual ~PropFairPolicy();

    /**
     * Initialize the requestor's score by providing
     * the requestor's name and initial score value.
     * The requestor's name has to match a name in the system.
     *
     * @param requestor requestor's name to lookup.
     * @param score initial score value for the requestor
     */
    void initRequestorName(const std::string requestor, const double score);

    /**
     * Initialize the requestor's score by providing
     * the requestor's SimObject pointer and initial score value.
     * The requestor's pointer has to match a requestor in the system.
     *
     * @param requestor requestor's SimObject pointer to lookup.
     * @param score initial score value for the requestor
     */
    void initRequestorObj(const SimObject *requestor, const double score);

    /**
     * Schedules a packet based on proportional fair configuration
     *
     * @param id requestor id to schedule
     * @param pkt_size size of the packet
     * @return QoS priority value
     */
    virtual uint8_t schedule(const RequestorID id,
                             const uint64_t pkt_size) override;

  protected:
    template <typename Requestor>
    void initRequestor(const Requestor requestor, const double score);

    inline double updateScore(const double old_score,
                              const uint64_t served_bytes) const;

  protected:
    /** PF Policy weight */
    const double weight;

    /** history is keeping track of every requestor's score */
    using RequestorHistory = std::pair<RequestorID, double>;
    std::vector<RequestorHistory> history;
};

} // namespace qos
} // namespace memory
} // namespace gem5

#endif // __MEM_QOS_POLICY_PF_HH__

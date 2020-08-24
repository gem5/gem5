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

#ifndef __MEM_QOS_POLICY_FIXED_PRIO_HH__
#define __MEM_QOS_POLICY_FIXED_PRIO_HH__

#include "mem/qos/policy.hh"
#include "params/QoSFixedPriorityPolicy.hh"

namespace QoS {

/**
 * Fixed Priority QoS Policy
 *
 * Fixed Priority Policy: based on a configured RequestorID to priority map,
 * it returns a fixed QoS priority value: every requestor has a fixed priority.
 */
class FixedPriorityPolicy : public Policy
{
    using Params = QoSFixedPriorityPolicyParams;

  public:
    FixedPriorityPolicy(const Params*);
    virtual ~FixedPriorityPolicy();

    void init() override;

    /**
     * Initialize the fixed requestor's priority by providing
     * the requestor's name and priority value.
     * The requestor's name has to match a name in the system.
     *
     * @param requestor requestor's name to lookup.
     * @param priority priority value for the requestor
     */
    void initRequestorName(std::string requestor, uint8_t priority);

    /**
     * Initialize the fixed requestor's priority by providing
     * the requestor's SimObject pointer and priority value.
     *
     * @param requestor requestor's SimObject pointer to lookup.
     * @param priority priority value for the requestor
     */
    void initRequestorObj(const SimObject* requestor, uint8_t priority);

    /**
     * Schedules a packet based on fixed priority configuration
     *
     * @param id requestor id to schedule
     * @param data data to schedule
     * @return QoS priority value
     */
    virtual uint8_t schedule(const RequestorID, const uint64_t) override;

  protected:
    /** Default fixed priority value for non-listed requestors */
    const uint8_t defaultPriority;

    /**
     * Priority map, associates configured requestors with
     * a fixed QoS priority value
     */
    std::map<RequestorID, uint8_t> priorityMap;
};

} // namespace QoS

#endif // __MEM_QOS_POLICY_FIXED_PRIO_HH__

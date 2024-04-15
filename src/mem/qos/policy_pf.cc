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

#include "mem/qos/policy_pf.hh"

#include <algorithm>

#include "base/logging.hh"
#include "params/QoSPropFairPolicy.hh"

namespace gem5
{

namespace memory
{

namespace qos
{

PropFairPolicy::PropFairPolicy(const Params &p) : Policy(p), weight(p.weight)
{
    fatal_if(weight < 0 || weight > 1,
             "weight must be a value between 0 and 1");
}

PropFairPolicy::~PropFairPolicy() {}

template <typename Requestor>
void
PropFairPolicy::initRequestor(const Requestor requestor, const double score)
{
    RequestorID id = memCtrl->system()->lookupRequestorId(requestor);

    assert(id != Request::invldRequestorId);

    // Setting the Initial score for the selected requestor.
    history.push_back(std::make_pair(id, score));

    fatal_if(history.size() > memCtrl->numPriorities(),
             "Policy's maximum number of requestors is currently dictated "
             "by the maximum number of priorities\n");
}

void
PropFairPolicy::initRequestorName(const std::string requestor,
                                  const double score)
{
    initRequestor(requestor, score);
}

void
PropFairPolicy::initRequestorObj(const SimObject *requestor,
                                 const double score)
{
    initRequestor(requestor, score);
}

double
PropFairPolicy::updateScore(const double old_score,
                            const uint64_t served_bytes) const
{
    return ((1.0 - weight) * old_score) + (weight * served_bytes);
}

uint8_t
PropFairPolicy::schedule(const RequestorID pkt_id, const uint64_t pkt_size)
{
    auto sort_pred = [](const RequestorHistory &lhs,
                        const RequestorHistory &rhs) {
        return lhs.second > rhs.second;
    };

    // Sorting in reverse in base of personal history:
    // First elements have higher history/score -> lower priority.
    // The qos priority is the position in the sorted vector.
    std::sort(history.begin(), history.end(), sort_pred);

    const double served_bytes = static_cast<double>(pkt_size);

    uint8_t pkt_priority = 0;
    for (auto m_hist = history.begin(); m_hist != history.end(); m_hist++) {
        RequestorID curr_id = m_hist->first;
        double &curr_score = m_hist->second;

        if (curr_id == pkt_id) {
            // The qos priority is the position in the sorted vector.
            pkt_priority = std::distance(history.begin(), m_hist);

            curr_score = updateScore(curr_score, served_bytes);
        } else {
            curr_score = updateScore(curr_score, 0);
        }
    }

    return pkt_priority;
}

} // namespace qos
} // namespace memory
} // namespace gem5

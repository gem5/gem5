/*
 * Copyright 2023 Google, Inc
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

#include <utility>

#include "base/logging.hh"
#include "mem/backdoor_manager.hh"

namespace gem5
{

BackdoorManager::BackdoorManager(const std::vector<AddrRange> &original_ranges,
                                 const std::vector<AddrRange> &remapped_ranges)
    : originalRanges(original_ranges),
      remappedRanges(remapped_ranges),
      backdoorLists(original_ranges.size())
{}

MemBackdoorPtr
BackdoorManager::getRevertedBackdoor(MemBackdoorPtr backdoor,
                                     const AddrRange &pkt_range)
{
    MemBackdoorPtr reverted_backdoor = findBackdoor(pkt_range);
    if (reverted_backdoor == nullptr) {
        reverted_backdoor = createRevertedBackdoor(backdoor, pkt_range);
    }
    return reverted_backdoor;
}

MemBackdoorPtr
BackdoorManager::createRevertedBackdoor(MemBackdoorPtr backdoor,
                                        const AddrRange &pkt_range)
{
    std::unique_ptr<MemBackdoor> reverted_backdoor =
        std::make_unique<MemBackdoor>();
    reverted_backdoor->flags(backdoor->flags());
    reverted_backdoor->ptr(backdoor->ptr());

    Addr addr = pkt_range.start();
    for (int i = 0; i < originalRanges.size(); ++i) {
        if (originalRanges[i].contains(addr)) {
            /** Does not support interleaved range backdoors. */
            if (originalRanges[i].interleaved() ||
                remappedRanges[i].interleaved()) {
                return nullptr;
            }

            /** Shrink the backdoor to fit inside address range. */
            AddrRange shrinked_backdoor_range =
                backdoor->range() & remappedRanges[i];

            Addr backdoor_offset =
                shrinked_backdoor_range.start() - remappedRanges[i].start();
            Addr backdoor_size = shrinked_backdoor_range.size();

            /** Create the backdoor in original address view. */
            reverted_backdoor->range(AddrRange(
                originalRanges[i].start() + backdoor_offset,
                originalRanges[i].start() + backdoor_offset + backdoor_size));

            /**
             * The backdoor pointer also needs to be shrinked to point to the
             * beginning of the range.
             */
            Addr shrinked_offset =
                shrinked_backdoor_range.start() - backdoor->range().start();
            reverted_backdoor->ptr(backdoor->ptr() + shrinked_offset);

            /**
             * Bind the life cycle of the created backdoor with the target
             * backdoor. Invalid and delete the created backdoor when the
             * target backdoor is invalidated.
             */
            MemBackdoorPtr reverted_backdoor_raw_ptr = reverted_backdoor.get();
            auto it = backdoorLists[i].insert(backdoorLists[i].end(),
                                              std::move(reverted_backdoor));
            backdoor->addInvalidationCallback(
                [this, i, it](const MemBackdoor &backdoor) {
                    (*it)->invalidate(); // *it is unique_ptr reverted_backdoor
                    this->backdoorLists[i].erase(it);
                });
            return reverted_backdoor_raw_ptr;
        }
    }
    // Backdoor is not valid. Return an empty one.
    panic("Target does not provide valid backdoor.");
}

MemBackdoorPtr
BackdoorManager::findBackdoor(const AddrRange &pkt_range) const
{
    Addr addr = pkt_range.start();
    Addr size = pkt_range.size();
    for (int i = 0; i < originalRanges.size(); ++i) {
        /** The original ranges should be disjoint, so at most one range
         * contains the begin address.
         */
        if (originalRanges[i].contains(addr)) {
            if (!originalRanges[i].contains(addr + size - 1)) {
                /** The request range doesn't fit in any address range. */
                return nullptr;
            }
            for (const auto &backdoor : backdoorLists[i]) {
                if (backdoor->range().contains(addr) &&
                    backdoor->range().contains(addr + size - 1)) {
                    return backdoor.get();
                }
            }
        }
    }
    return nullptr;
}

} // namespace gem5

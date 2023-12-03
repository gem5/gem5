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

#ifndef __MEM_BACKDOOR_MANAGER_HH__
#define __MEM_BACKDOOR_MANAGER_HH__

#include <list>
#include <memory>
#include <vector>

#include "mem/backdoor.hh"
#include "mem/packet.hh"

namespace gem5
{

/**
 * This class manages the backdoors for RangeAddrMapper. It provides
 * functionalities such as backdoor remapping, resource managing.
 */
class BackdoorManager
{
  public:
    explicit BackdoorManager(const std::vector<AddrRange> &original_ranges,
                             const std::vector<AddrRange> &remapped_ranges);

    MemBackdoorPtr getRevertedBackdoor(MemBackdoorPtr backdoor,
                                       const AddrRange &pkt_range);

  protected:
    /**
     * This function creates a new backdoor, whose address range contains the
     * original request address. The address range is in initiator address
     * view, and shouldn't exceed the original address range.
     */
    MemBackdoorPtr createRevertedBackdoor(MemBackdoorPtr backdoor,
                                          const AddrRange &pkt_range);
    /**
     * This function returns a created backdoor that fulfills the request, or
     * returns nullptr if there's no.
     */
    MemBackdoorPtr findBackdoor(const AddrRange &pkt_range) const;

    const std::vector<AddrRange> &originalRanges;
    const std::vector<AddrRange> &remappedRanges;

    /**
     * In this vector, each entry contains a list of backdoors that in the
     * range in original address view.
     */
    std::vector<std::list<std::unique_ptr<MemBackdoor>>> backdoorLists;
};
} // namespace gem5

#endif //__MEM_BACKDOOR_MANAGER_HH__

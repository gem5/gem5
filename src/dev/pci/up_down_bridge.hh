/*
 * Copyright (c) 2025 REDS institute of the HEIG-VD
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

#ifndef __DEV_PCI_ONE_WAY_BRIDGE_HH__
#define __DEV_PCI_ONE_WAY_BRIDGE_HH__

#include "mem/bridge.hh"
#include "params/PciUpDownBridge.hh"

namespace gem5
{

/**
 * This bridge lets packets pass from upstream to downstream of a PCI bridge.
 *
 * All the address ranges are dynamically determined based on the connected
 * bus. A PCI configuration range can be set, the bridge will be able to
 * respond to any of the address in that range.
 */
class PciUpDownBridge : public BridgeBase
{
  private:
    /** Addresses ranges that the memory side buses can respond to. */
    AddrRangeList memSideRanges;

    /** PCI configuration range that is behind the bridge. */
    AddrRange configRange;

  protected:
    /**
     * Get a list of the non-overlapping address ranges the bridge is
     * responsible for.
     *
     * @return a list of ranges responded to
     */
    AddrRangeList getAddrRanges() const override;

    /**
     * Called when the memory side port receives an address range change from
     * the peer response port. This allows the bridge to dynamically update
     * address ranges that can pass through with the new ones.
     */
    void recvRangeChange() override;

  public:
    /**
     * Set the PCI configuration range that is behind the bridge.
     */
    void setConfigRange(AddrRange config_range);

    PARAMS(PciUpDownBridge);

    PciUpDownBridge(const Params &p);
};

} // namespace gem5

#endif //__DEV_PCI_ONE_WAY_BRIDGE_HH__

/*
 * Copyright (c) 2025 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DEV_AMDGPU_XGMI_HIVE_HH__
#define __DEV_AMDGPU_XGMI_HIVE_HH__

#include <map>
#include <unordered_map>

#include "params/XGMIHive.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class AddrRange;
class AMDGPUDevice;

/*
 * This class represents an xGMI hive, which is a collection of AMDGPU devices.
 * A hive sits at the system level and manages global framebuffer (i.e., shared
 * memory) address ranges.
 */
class XGMIHive : public SimObject
{
  public:
    XGMIHive(const XGMIHiveParams &p);

    // Called after all SimObjects are constructed. Here we can setup initial
    // framebuffer offsets based on the number of nodes in the hive and their
    // memory sizes.
    void init() override;

    // Called when an AMDGPU device is constructed to add information about
    // its GPU ID and framebuffer size.
    void addNode(AMDGPUDevice *device);

    // Called when an AMDGPU device receives an update from the OS about its
    // PCIe BAR0 address range. Communication of xGMI is seen as a system
    // address in the PCIe BAR0 range.
    void updateAddressRange(int gpuId, const AddrRange &bar0Range);

    // Used to check if an address with the system bit set should be routed to
    // another GPU in this hive or to the host.
    bool isXgmiAddress(Addr addr) const;

    // When xGMI is enabled a GPU's framebuffer addresses are offset by
    // a base address determined by the hive. This function returns that base
    // address for a given GPU ID.
    Addr getXgmiBaseAddr(int gpuId) const;

    int
    getNodeCount() const
    {
        return gpuFrameSize.size();
    }
    Addr getFrameSize(int gpuId) const;

    uint64_t getHiveId() const { return hiveId; }

    int getDeviceId(Addr addr) const;
    AMDGPUDevice *getDevice(int gpuId) const;
    const std::unordered_map<int, AMDGPUDevice *>& getDevices() const { return gpuDevices; }
    Addr getDeviceBase(int gpuId) const;

  private:
    const int hiveId = 0;

    // For these ranges we use gem5's gpuId rather than xGMI node ID as gem5
    // sets up the local frame buffer address ranges based on gpuId. This
    // removes a lot of complexity which would otherwise needed to sort ranges
    // based on xGMI node ID. The node ID itself is only used to pass to the
    // driver via an MMIO. The frame sizes we want ordered for us.
    std::unordered_map<int, AddrRange> gpuAddressRange;
    std::unordered_map<int, AddrRange> gpuFrameOffset;
    std::map<int, AddrRange> gpuFrameSize;
    std::unordered_map<int, AMDGPUDevice *> gpuDevices;
};

} // namespace gem5

#endif // __DEV_AMDGPU_XGMI_HIVE_HH__

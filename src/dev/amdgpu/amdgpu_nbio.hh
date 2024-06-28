/*
 * Copyright (c) 2023 Advanced Micro Devices, Inc.
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
 *
 */

#ifndef __DEV_AMDGPU_AMDGPU_NBIO__
#define __DEV_AMDGPU_AMDGPU_NBIO__

#include <unordered_map>

#include "base/types.hh"
#include "mem/packet.hh"

namespace gem5
{

class AMDGPUDevice;

/**
 * MMIO offsets for NBIO. NBIO handles initialization such as device
 * discovery and psp functions. Values taken from:
 *
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *      drivers/gpu/drm/amd/amdgpu/amdgpu_discovery.c
 *
 * The addresses in the file are dword addresses. Here they are converted
 * to byte addresses so gem5 does not need to do any shifting.
 */
#define AMDGPU_MM_INDEX                                   0x00000
#define AMDGPU_MM_INDEX_HI                                0x00018
#define AMDGPU_MM_DATA                                    0x00004

#define AMDGPU_PCIE_INDEX                                 0x00030
#define AMDGPU_PCIE_INDEX2                                0x00038
#define AMDGPU_PCIE_DATA                                  0x00034
#define AMDGPU_PCIE_DATA2                                 0x0003c

// Message bus related to psp
#define AMDGPU_MP0_SMN_C2PMSG_33                          0x58184
#define AMDGPU_MP0_SMN_C2PMSG_35                          0x5818c
#define AMDGPU_MP0_SMN_C2PMSG_64                          0x58200
#define AMDGPU_MP0_SMN_C2PMSG_69                          0x58214
#define AMDGPU_MP0_SMN_C2PMSG_70                          0x58218
#define AMDGPU_MP0_SMN_C2PMSG_71                          0x5821c
#define AMDGPU_MP0_SMN_C2PMSG_81                          0x58244
#define AMDGPU_MP1_SMN_C2PMSG_90                          0x58a68

// Device specific invalidation engines used during initialization
#define VEGA10_INV_ENG17_ACK1                             0x0a318
#define VEGA10_INV_ENG17_ACK2                             0x69c18
#define VEGA10_INV_ENG17_SEM1                             0x0a288
#define VEGA10_INV_ENG17_SEM2                             0x69b88

#define MI100_INV_ENG17_ACK1                              0x0a318
#define MI100_INV_ENG17_ACK2                              0x6a918
#define MI100_INV_ENG17_ACK3                              0x76918
#define MI100_INV_ENG17_SEM1                              0x0a288
#define MI100_INV_ENG17_SEM2                              0x6a888
#define MI100_INV_ENG17_SEM3                              0x76888

#define MI200_INV_ENG17_ACK1                              0x0a318
#define MI200_INV_ENG17_ACK2                              0x6b018
#define MI200_INV_ENG17_SEM1                              0x0a288
#define MI200_INV_ENG17_SEM2                              0x6af88

class AMDGPUNbio
{
  public:
    AMDGPUNbio();

    void setGPUDevice(AMDGPUDevice *gpu_device);

    void readMMIO(PacketPtr pkt, Addr offset);
    void writeMMIO(PacketPtr pkt, Addr offset);

    bool readFrame(PacketPtr pkt, Addr offset);
    void writeFrame(PacketPtr pkt, Addr offset);

  private:
    AMDGPUDevice *gpuDevice;

    /*
     * Driver initialization sequence helper variables.
     */
    uint64_t mm_index_reg = 0;
    uint32_t pcie_index_reg = 0;
    uint32_t pcie_index2_reg = 0;
    std::unordered_map<uint32_t, uint32_t> triggered_reads;

    /*
     * PSP variables used in initialization.
     */
    Addr psp_ring = 0;
    Addr psp_ring_dev_addr = 0;
    Addr psp_ring_listen_addr = 0;
    int psp_ring_size = 0;
    int psp_ring_value = 0;

    /*
     * Hold values of other registers not explicitly modelled by other blocks.
     */
    using GPURegMap = std::unordered_map<uint64_t, uint32_t>;
    GPURegMap regs;
};

} // namespace gem5

#endif // __DEV_AMDGPU_AMDGPU_NBIO__

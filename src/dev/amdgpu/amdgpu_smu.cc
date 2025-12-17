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

#include "dev/amdgpu/amdgpu_smu.hh"

#include "debug/AMDGPUDevice.hh"
#include "dev/amdgpu/amdgpu_device.hh"
#include "mem/packet_access.hh"

namespace gem5
{

void
AMDGPUSmu::readMMIO(PacketPtr pkt, Addr offset)
{
    uint32_t regval = 0;

    switch (offset) {
        case MI200_SMUIO_MCM_CONFIG:
            regval = (gpuDevice->getGpuId() << 4);
            break;
        default:
            DPRINTF(AMDGPUDevice, "SMU read of unknown MMIO offset %x (%x)\n",
                    offset, pkt->getAddr());
            break;
    }

    pkt->setLE<uint32_t>(regval);

    DPRINTF(AMDGPUDevice, "SMU read MMIO offset %x (%x): %x\n", offset,
            pkt->getAddr(), pkt->getLE<uint32_t>());
}

void
AMDGPUSmu::writeMMIO(PacketPtr pkt, Addr offset)
{
    switch (offset) {
        default:
            DPRINTF(AMDGPUDevice, "SMU write of unknown MMIO offset %x (%x)\n",
                    offset, pkt->getAddr());
    }
}

void
AMDGPUSmu::setGPUDevice(AMDGPUDevice *gpu_device)
{
    gpuDevice = gpu_device;
}

} // namespace gem5

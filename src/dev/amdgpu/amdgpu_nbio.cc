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
 */

#include "dev/amdgpu/amdgpu_nbio.hh"

#include "debug/AMDGPUDevice.hh"
#include "dev/amdgpu/amdgpu_device.hh"
#include "mem/packet_access.hh"

namespace gem5
{

AMDGPUNbio::AMDGPUNbio()
{
    // All read-before-write MMIOs go here
    triggered_reads[AMDGPU_MP0_SMN_C2PMSG_64] = 0x80000000;
}

void
AMDGPUNbio::setGPUDevice(AMDGPUDevice *gpu_device)
{
    gpuDevice = gpu_device;
}

void
AMDGPUNbio::readMMIO(PacketPtr pkt, Addr offset)
{
    switch (offset) {
      // This is a PCIe status register. At some point during driver init
      // the driver checks that interrupts are enabled. This is only
      // checked once, so if the MMIO trace does not exactly line up with
      // what the driver is doing in gem5, this may still have the first
      // bit zero causing driver to fail. Therefore, we always set this
      // bit to one as there is no harm to do so.
      case AMDGPU_PCIE_DATA_REG:
        {
          uint32_t value = pkt->getLE<uint32_t>() | 0x1;
          DPRINTF(AMDGPUDevice, "Marking interrupts enabled: %#lx\n", value);
          pkt->setLE<uint32_t>(value);
        }
        break;
      case AMDGPU_MM_DATA:
        //pkt->setLE<uint32_t>(regs[mm_index_reg]);
        pkt->setLE<uint32_t>(gpuDevice->getRegVal(mm_index_reg));
        break;
      case VEGA10_INV_ENG17_ACK1:
      case VEGA10_INV_ENG17_ACK2:
      case MI100_INV_ENG17_ACK2:
      case MI100_INV_ENG17_ACK3:
        pkt->setLE<uint32_t>(0x10001);
        break;
      case VEGA10_INV_ENG17_SEM1:
      case VEGA10_INV_ENG17_SEM2:
      case MI100_INV_ENG17_SEM2:
      case MI100_INV_ENG17_SEM3:
        pkt->setLE<uint32_t>(0x1);
        break;
      // PSP responds with bit 31 set when ready
      case AMDGPU_MP0_SMN_C2PMSG_35:
        pkt->setLE<uint32_t>(0x80000000);
        break;
      default:
        if (triggered_reads.count(offset)) {
            DPRINTF(AMDGPUDevice, "Found triggered read for %#x\n", offset);
            pkt->setLE<uint32_t>(triggered_reads[offset]);
        } else if (gpuDevice->haveRegVal(offset)) {
            uint32_t reg_val = gpuDevice->getRegVal(offset);

            DPRINTF(AMDGPUDevice, "Reading value of %#lx from regs: %#lx\n",
                    offset, reg_val);

            pkt->setLE<uint32_t>(reg_val);
        } else {
            DPRINTF(AMDGPUDevice, "NBIO Unknown MMIO %#x (%#x)\n", offset,
                    pkt->getAddr());
        }
        break;
    }
}

void
AMDGPUNbio::writeMMIO(PacketPtr pkt, Addr offset)
{
    if (offset == AMDGPU_MM_INDEX) {
        assert(pkt->getSize() == 4);
        mm_index_reg = insertBits(mm_index_reg, 31, 0,
                                  pkt->getLE<uint32_t>());
    } else if (offset == AMDGPU_MM_INDEX_HI) {
        assert(pkt->getSize() == 4);
        mm_index_reg = insertBits(mm_index_reg, 63, 32,
                                  pkt->getLE<uint32_t>());
    } else if (offset == AMDGPU_MM_DATA) {
        DPRINTF(AMDGPUDevice, "MM write to reg %#lx data %#lx\n",
                mm_index_reg, pkt->getLE<uint32_t>());
        gpuDevice->setRegVal(AMDGPU_MM_DATA, pkt->getLE<uint32_t>());
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_35) {
        // See psp_v3_1_bootloader_load_sos in amdgpu driver code.
        if (pkt->getLE<uint32_t>() == 0x10000) {
            triggered_reads[AMDGPU_MP0_SMN_C2PMSG_81] = 0xdf40b31;
        }
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_64) {
        triggered_reads[AMDGPU_MP0_SMN_C2PMSG_64] =
            0x80000000 + pkt->getLE<uint32_t>();
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_69) {
        // PSP ring low addr
        psp_ring = insertBits(psp_ring, 31, 0, pkt->getLE<uint32_t>());
        psp_ring_listen_addr = psp_ring
                             - gpuDevice->getVM().getSysAddrRangeLow() + 0xc;
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_70) {
        // PSP ring high addr
        psp_ring = insertBits(psp_ring, 63, 32, pkt->getLE<uint32_t>());
        psp_ring_listen_addr = psp_ring
                             - gpuDevice->getVM().getSysAddrRangeLow() + 0xc;
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_71) {
        // PSP ring size
        psp_ring_size = pkt->getLE<uint32_t>();
    }
}

bool
AMDGPUNbio::readFrame(PacketPtr pkt, Addr offset)
{
    if (offset == psp_ring_dev_addr) {
        psp_ring_value++;
        pkt->setUintX(psp_ring_value, ByteOrder::little);

        return true;
    }

    return false;
}

void
AMDGPUNbio::writeFrame(PacketPtr pkt, Addr offset)
{
    if (offset == psp_ring_listen_addr) {
        DPRINTF(AMDGPUDevice, "Saw psp_ring_listen_addr with size %ld value "
                "%ld\n", pkt->getSize(), pkt->getUintX(ByteOrder::little));

        /*
         * In ROCm versions 4.x this packet is a 4 byte value. In ROCm 5.x
         * the packet is 8 bytes and mapped as a system address which needs
         * to be subtracted out to get the framebuffer address.
         */
        if (pkt->getSize() == 4) {
            psp_ring_dev_addr = pkt->getLE<uint32_t>();
        } else if (pkt->getSize() == 8) {
            psp_ring_dev_addr = pkt->getUintX(ByteOrder::little)
                              - gpuDevice->getVM().getSysAddrRangeLow();
        } else {
            panic("Invalid write size to psp_ring_listen_addr\n");
        }

        DPRINTF(AMDGPUDevice, "Setting PSP ring device address to %#lx\n",
                psp_ring_dev_addr);
    }
}

} // namespace gem5

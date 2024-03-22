/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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

#ifndef __DEV_AMDGPU_INTERRUPT_HANDLER__
#define __DEV_AMDGPU_INTERRUPT_HANDLER__

#include <bitset>
#include <iostream>
#include <queue>
#include <vector>

#include "base/addr_range.hh"
#include "base/flags.hh"
#include "base/types.hh"
#include "dev/amdgpu/amdgpu_device.hh"
#include "dev/dma_device.hh"
#include "params/AMDGPUInterruptHandler.hh"

namespace gem5
{

/**
 * Defines from driver code. Taken from
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *     drivers/gpu/drm/amd/include/soc15_ih_clientid.h
 */
enum soc15_ih_clientid
{
    SOC15_IH_CLIENTID_RLC       = 0x07,
    SOC15_IH_CLIENTID_SDMA0     = 0x08,
    SOC15_IH_CLIENTID_SDMA1     = 0x09,
    SOC15_IH_CLIENTID_SDMA2     = 0x01,
    SOC15_IH_CLIENTID_SDMA3     = 0x04,
    SOC15_IH_CLIENTID_SDMA4     = 0x05,
    SOC15_IH_CLIENTID_SDMA5     = 0x11,
    SOC15_IH_CLIENTID_SDMA6     = 0x13,
    SOC15_IH_CLIENTID_SDMA7     = 0x18,
    SOC15_IH_CLIENTID_GRBM_CP   = 0x14
};

enum ihSourceId
{
    CP_EOP                      = 181,
    TRAP_ID                     = 224
};

/**
 * MSI-style interrupts. Send a "cookie" response to clear interrupts.
 * From [1] we know the size of the struct is 8 dwords. Then we can look at the register shift offsets in [2] to guess the rest.
 * Or we can also look at [3].
 *
 * [1] https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *         drivers/gpu/drm/amd/amdkfd/kfd_device.c#L316
 * [2] https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *         drivers/gpu/drm/amd/include/asic_reg/oss/osssys_4_0_sh_mask.h#L122
 * [3] https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
           drivers/gpu/drm/amd/amdgpu/amdgpu_irq.h#L46
 *
 */
constexpr uint32_t INTR_COOKIE_SIZE = 32; // in bytes

typedef struct
{
    uint32_t clientId : 8;
    uint32_t sourceId : 8;
    uint32_t ringId : 8;
    uint32_t vmId : 4;
    uint32_t reserved1 : 3;
    uint32_t vmid_type : 1;
    uint32_t timestamp_Lo;
    uint32_t timestamp_Hi : 16;
    uint32_t reserved2 : 15;
    uint32_t timestamp_src : 1;
    uint32_t pasid : 16;
    uint32_t nodeId : 8;
    uint32_t reserved3 : 7;
    uint32_t pasid_src : 1;
    uint32_t source_data_dw1;
    uint32_t source_data_dw2;
    uint32_t source_data_dw3;
    uint32_t source_data_dw4;
} AMDGPUInterruptCookie;
static_assert(sizeof(AMDGPUInterruptCookie) == INTR_COOKIE_SIZE);

/**
 * Struct to contain all interrupt handler related registers.
 */
typedef struct
{
    uint32_t IH_Cntl;
    uint32_t IH_Base;
    uint32_t IH_Base_Hi;
    Addr baseAddr;
    uint32_t IH_Rptr;
    uint32_t IH_Wptr;
    uint32_t IH_Wptr_Addr_Lo;
    uint32_t IH_Wptr_Addr_Hi;
    Addr WptrAddr;
    uint32_t IH_Doorbell;
} AMDGPUIHRegs;

class AMDGPUInterruptHandler : public DmaDevice
{
  public:
    class DmaEvent : public Event
    {
      private:
        AMDGPUInterruptHandler *deviceIh;
        uint32_t data;

      public:
        DmaEvent(AMDGPUInterruptHandler *deviceIh, uint32_t data)
            : Event(), deviceIh(deviceIh), data(data)
        {
            setFlags(Event::AutoDelete);
        }
        void process();
        const char *description() const {
            return "AMDGPUInterruptHandler Dma";
        }

        void setData(uint32_t _data) { data = _data; }
        uint32_t getData() { return data; }
    };

    struct SenderState : public Packet::SenderState
    {
        SenderState(Packet::SenderState *sender_state, Addr addr)
            : saved(sender_state), _addr(addr)
        {
        }
        Packet::SenderState *saved;
        Addr _addr;
    };

    AMDGPUInterruptHandler(const AMDGPUInterruptHandlerParams &p);

    Tick write(PacketPtr pkt) override { return 0; }
    Tick read(PacketPtr pkt) override { return 0; }
    AddrRangeList getAddrRanges() const override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void setGPUDevice(AMDGPUDevice *gpu_device) { gpuDevice = gpu_device; }
    void prepareInterruptCookie(ContextID cntxtId, uint32_t ring_id,
        uint32_t client_id, uint32_t source_id, unsigned node_id);
    void submitInterruptCookie();
    void submitWritePointer();
    void intrPost();

    /**
     * Methods for setting the values of interrupt handler MMIO registers.
     */
    void writeMMIO(PacketPtr pkt, Addr mmio_offset);

    uint32_t getDoorbellOffset() const { return regs.IH_Doorbell; }
    void setCntl(const uint32_t &data);
    void setBase(const uint32_t &data);
    void setBaseHi(const uint32_t &data);
    void setRptr(const uint32_t &data);
    void setWptr(const uint32_t &data);
    void setWptrAddrLo(const uint32_t &data);
    void setWptrAddrHi(const uint32_t &data);
    void setDoorbellOffset(const uint32_t &data);
    void updateRptr(const uint32_t &data);

  private:
    AMDGPUDevice *gpuDevice;
    AMDGPUIHRegs regs;
    std::queue<AMDGPUInterruptCookie*> interruptQueue;
    AMDGPUInterruptHandler::DmaEvent *dmaEvent;
};

} // namespace gem5

#endif // __DEV_AMDGPU_INTERRUPT_HANDLER__

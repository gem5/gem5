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

#ifndef __DEV_AMDGPU_PM4_PACKET_PROCESSOR__
#define __DEV_AMDGPU_PM4_PACKET_PROCESSOR__

#include <unordered_map>

#include "dev/amdgpu/amdgpu_device.hh"
#include "dev/amdgpu/pm4_defines.hh"
#include "dev/amdgpu/pm4_queues.hh"
#include "dev/dma_virt_device.hh"
#include "params/PM4PacketProcessor.hh"

namespace gem5
{

class AMDGPUDevice;




class PM4PacketProcessor : public DmaVirtDevice
{
    AMDGPUDevice *gpuDevice;
    /* First graphics queue */
    PrimaryQueue pq;
    PM4MapQueues pq_pkt;
    /* First compute queue */
    QueueDesc kiq;
    PM4MapQueues kiq_pkt;

    /* All PM4 queues, indexed by VMID */
    std::unordered_map<uint16_t, PM4Queue *> queues;
    /* A map of PM4 queues based on doorbell offset */
    std::unordered_map<uint32_t, PM4Queue *> queuesMap;

    int _ipId;
    AddrRange _mmioRange;

  public:
    PM4PacketProcessor(const PM4PacketProcessorParams &p);

    void setGPUDevice(AMDGPUDevice *gpu_device);

    /**
     * Inherited methods.
     */
    Tick write(PacketPtr pkt) override { return 0; }
    Tick read(PacketPtr pkt) override { return 0; }
    AddrRangeList getAddrRanges() const override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Method for functional translation.
     */
    TranslationGenPtr translate(Addr vaddr, Addr size) override;

    uint32_t getKiqDoorbellOffset() { return kiq.doorbell & 0x1ffffffc; }
    uint32_t getPqDoorbellOffset() { return pq.doorbellOffset; }

    Addr getGARTAddr(Addr addr) const;

    /**
     * Based on an offset communicated through doorbell write, the
     * PM4PacketProcessor identifies which queue needs processing.
     */
    PM4Queue* getQueue(Addr offset, bool gfx = false);
    /**
     * The first graphics queue, the Primary Queueu a.k.a. RB0, needs to be
     * mapped since all queue details are communicated through MMIOs to
     * special registers.
     */
    void mapPq(Addr offset);
    /**
     * The first compute queue, the Kernel Interface Queueu a.k.a. KIQ, needs
     * to be mapped since all queue details are communicated through MMIOs to
     * special registers.
     */
    void mapKiq(Addr offset);
    /**
     * This method creates a new PM4Queue based on a queue descriptor and an
     * offset.
     */
    void newQueue(QueueDesc *q, Addr offset, PM4MapQueues *pkt = nullptr,
                  int id = -1);

    /**
     * This method start processing a PM4Queue from the current read pointer
     * to the newly communicated write pointer (i.e., wptrOffset).
     */
    void process(PM4Queue *q, Addr wptrOffset);

    /**
     * Update read index on doorbell rings. We use write index, however read
     * index == write index when the queue is empty. This allows us to save
     * previous read index when a queue is remapped. The remapped queue will
     * read from the previous read index rather than reset to zero.
     */
    void updateReadIndex(Addr offset, uint64_t rd_idx);

    /**
     * This method decodes the next packet in a PM4Queue.
     */
    void decodeNext(PM4Queue *q);
    /**
     * This method calls other PM4 packet processing methods based on the
     * header of a PM4 packet.
     */
    void decodeHeader(PM4Queue *q, PM4Header header);

    /* Methods that implement PM4 packets */
    void writeData(PM4Queue *q, PM4WriteData *pkt, PM4Header header);
    void writeDataDone(PM4Queue *q, PM4WriteData *pkt, Addr addr);
    void mapQueues(PM4Queue *q, PM4MapQueues *pkt);
    void unmapQueues(PM4Queue *q, PM4UnmapQueues *pkt);
    void doneMQDWrite(Addr mqdAddr, Addr addr);
    void mapProcess(uint32_t pasid, uint64_t ptBase, uint32_t shMemBases);
    void mapProcessGfx9(PM4Queue *q, PM4MapProcess *pkt);
    void mapProcessGfx90a(PM4Queue *q, PM4MapProcessMI200 *pkt);
    void processMQD(PM4MapQueues *pkt, PM4Queue *q, Addr addr, QueueDesc *mqd,
                    uint16_t vmid);
    void processSDMAMQD(PM4MapQueues *pkt, PM4Queue *q, Addr addr,
                        SDMAQueueDesc *mqd, uint16_t vmid);
    void releaseMem(PM4Queue *q, PM4ReleaseMem *pkt);
    void releaseMemDone(PM4Queue *q, PM4ReleaseMem *pkt, Addr addr);
    void runList(PM4Queue *q, PM4RunList *pkt);
    void indirectBuffer(PM4Queue *q, PM4IndirectBuf *pkt);
    void switchBuffer(PM4Queue *q, PM4SwitchBuf *pkt);
    void setUconfigReg(PM4Queue *q, PM4SetUconfigReg *pkt);
    void waitRegMem(PM4Queue *q, PM4WaitRegMem *pkt);
    void queryStatus(PM4Queue *q, PM4QueryStatus *pkt);
    void queryStatusDone(PM4Queue *q, PM4QueryStatus *pkt);

    /* Methods that implement MMIO regs */
    void writeMMIO(PacketPtr pkt, Addr mmio_offset);

    void setHqdVmid(uint32_t data);
    void setHqdActive(uint32_t data);
    void setHqdPqBase(uint32_t data);
    void setHqdPqBaseHi(uint32_t data);
    void setHqdPqDoorbellCtrl(uint32_t data);
    void setHqdPqPtr(uint32_t data);
    void setHqdPqWptrLo(uint32_t data);
    void setHqdPqWptrHi(uint32_t data);
    void setHqdPqRptrReportAddr(uint32_t data);
    void setHqdPqRptrReportAddrHi(uint32_t data);
    void setHqdPqWptrPollAddr(uint32_t data);
    void setHqdPqWptrPollAddrHi(uint32_t data);
    void setHqdPqControl(uint32_t data);
    void setHqdIbCtrl(uint32_t data);
    void setRbVmid(uint32_t data);
    void setRbCntl(uint32_t data);
    void setRbWptrLo(uint32_t data);
    void setRbWptrHi(uint32_t data);
    void setRbRptrAddrLo(uint32_t data);
    void setRbRptrAddrHi(uint32_t data);
    void setRbWptrPollAddrLo(uint32_t data);
    void setRbWptrPollAddrHi(uint32_t data);
    void setRbBaseLo(uint32_t data);
    void setRbBaseHi(uint32_t data);
    void setRbDoorbellCntrl(uint32_t data);
    void setRbDoorbellRangeLo(uint32_t data);
    void setRbDoorbellRangeHi(uint32_t data);

    int getIpId() const { return _ipId; }
    AddrRange getMMIORange() const { return _mmioRange; }
};

} // namespace gem5

#endif //__DEV_AMDGPU_PM4_PACKET_PROCESSOR__

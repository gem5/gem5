/*
 * Copyright (c) 2012-2020 ARM Limited
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
 * Copyright (c) 2013 Amin Farmahini-Farahani
 * All rights reserved.
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

/**
 * @file
 * HeteroMemCtrl declaration
 */

#ifndef __HETERO_MEM_CTRL_HH__
#define __HETERO_MEM_CTRL_HH__

#include "mem/mem_ctrl.hh"
#include "params/HeteroMemCtrl.hh"

namespace gem5
{

namespace memory
{
class HeteroMemCtrl : public MemCtrl
{
  private:
    /**
     * Create pointer to interface of the actual nvm media when connected.
     */
    NVMInterface *nvm;
    MemPacketQueue::iterator chooseNext(MemPacketQueue &queue,
                                        Tick extra_col_delay,
                                        MemInterface *mem_int) override;
    virtual std::pair<MemPacketQueue::iterator, Tick>
    chooseNextFRFCFS(MemPacketQueue &queue, Tick extra_col_delay,
                     MemInterface *mem_intr) override;
    Tick doBurstAccess(MemPacket *mem_pkt, MemInterface *mem_int) override;
    Tick minReadToWriteDataGap() override;
    Tick minWriteToReadDataGap() override;
    AddrRangeList getAddrRanges() override;

    /**
     * Burst-align an address.
     *
     * @param addr The potentially unaligned address
     * @param mem_intr The DRAM memory interface this packet belongs to
     *
     * @return An address aligned to a memory burst
     */
    virtual Addr burstAlign(Addr addr, MemInterface *mem_intr) const override;

    /**
     * Check if mem pkt's size is sane
     *
     * @param mem_pkt memory packet
     * @param mem_intr memory interface
     * @return a boolean indicating if the mem pkt size is less than
     * the burst size of the related mem interface
     */
    virtual bool pktSizeCheck(MemPacket *mem_pkt,
                              MemInterface *mem_intr) const override;

    virtual void processRespondEvent(MemInterface *mem_intr,
                                     MemPacketQueue &queue,
                                     EventFunctionWrapper &resp_event,
                                     bool &retry_rd_req) override;

    /**
     * Checks if the memory interface is already busy
     *
     * @param mem_intr memory interface to check
     * @return a boolean indicating if memory is busy
     */
    virtual bool memBusy(MemInterface *mem_intr) override;

    /**
     * Will access nvm memory interface and select non-deterministic
     * reads to issue
     */
    virtual void nonDetermReads(MemInterface *mem_intr) override;

    /**
     * Will check if all writes are for nvm interface
     * and nvm's write resp queue is full.
     *
     * @param mem_intr memory interface to use
     * @return a boolean showing if nvm is blocked with writes
     */
    virtual bool nvmWriteBlock(MemInterface *mem_intr) override;

  public:
    HeteroMemCtrl(const HeteroMemCtrlParams &p);

    bool allIntfDrained() const override;
    DrainState drain() override;
    void drainResume() override;

  protected:
    Tick recvAtomic(PacketPtr pkt) override;
    void recvFunctional(PacketPtr pkt) override;
    bool recvTimingReq(PacketPtr pkt) override;
};

} // namespace memory
} // namespace gem5

#endif //__HETERO_MEM_CTRL_HH__

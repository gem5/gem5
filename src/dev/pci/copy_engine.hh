/*
 * Copyright (c) 2012 ARM Limited
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
 * Copyright (c) 2008 The Regents of The University of Michigan
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

/* @file
 * Device model for Intel's I/O Acceleration Technology (I/OAT).
 * A DMA asyncronous copy engine
 */

#ifndef __DEV_PCI_COPY_ENGINE_HH__
#define __DEV_PCI_COPY_ENGINE_HH__

#include <vector>

#include "base/statistics.hh"
#include "dev/pci/copy_engine_defs.hh"
#include "dev/pci/device.hh"
#include "params/CopyEngine.hh"
#include "sim/drain.hh"
#include "sim/eventq.hh"

namespace gem5
{

class CopyEngine : public PciDevice
{
    class CopyEngineChannel : public Drainable, public Serializable
    {
      private:
        DmaPort cePort;
        CopyEngine *ce;
        copy_engine_reg::ChanRegs  cr;
        int channelId;
        copy_engine_reg::DmaDesc *curDmaDesc;
        uint8_t *copyBuffer;

        bool busy;
        bool underReset;
        bool refreshNext;
        Addr lastDescriptorAddr;
        Addr fetchAddress;

        Tick latBeforeBegin;
        Tick latAfterCompletion;

        uint64_t completionDataReg;

        enum ChannelState
        {
            Idle,
            AddressFetch,
            DescriptorFetch,
            DMARead,
            DMAWrite,
            CompletionWrite
        };

        ChannelState nextState;

      public:
        CopyEngineChannel(CopyEngine *_ce, int cid);
        virtual ~CopyEngineChannel();
        Port &getPort();

        std::string
        name()
        {
            assert(ce);
            return ce->name() + csprintf("-chan%d", channelId);
        }

        virtual Tick read(PacketPtr pkt)
                        { panic("CopyEngineChannel has no I/O access\n");}
        virtual Tick write(PacketPtr pkt)
                        { panic("CopyEngineChannel has no I/O access\n"); }

        void channelRead(PacketPtr pkt, Addr daddr, int size);
        void channelWrite(PacketPtr pkt, Addr daddr, int size);

        DrainState drain() override;
        void drainResume() override;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

      private:
        void fetchDescriptor(Addr address);
        void fetchDescComplete();
        EventFunctionWrapper fetchCompleteEvent;

        void fetchNextAddr(Addr address);
        void fetchAddrComplete();
        EventFunctionWrapper addrCompleteEvent;

        void readCopyBytes();
        void readCopyBytesComplete();
        EventFunctionWrapper readCompleteEvent;

        void writeCopyBytes();
        void writeCopyBytesComplete();
        EventFunctionWrapper writeCompleteEvent;

        void writeCompletionStatus();
        void writeStatusComplete();
        EventFunctionWrapper statusCompleteEvent;


        void continueProcessing();
        void recvCommand();
        bool inDrain();
        void restartStateMachine();
    };

  private:

    struct CopyEngineStats : public statistics::Group
    {
        CopyEngineStats(statistics::Group *parent,
            const uint8_t& channel_count);

        statistics::Vector bytesCopied;
        statistics::Vector copiesProcessed;
    } copyEngineStats;

    // device registers
    copy_engine_reg::Regs regs;

    // Array of channels each one with regs/dma port/etc
    std::vector<CopyEngineChannel*> chan;

  public:
    PARAMS(CopyEngine);
    CopyEngine(const Params &params);
    ~CopyEngine();

    Port &getPort(const std::string &if_name,
            PortID idx = InvalidPortID) override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif //__DEV_PCI_COPY_ENGINE_HH__

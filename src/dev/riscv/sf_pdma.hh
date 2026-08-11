/*
 * Copyright (c) 2025 Nikita Proshkin
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

#ifndef _SF_PDMA_HH_
#define _SF_PDMA_HH_

#include <queue>

#include "base/chunk_generator.hh"
#include "base/circlebuf.hh"
#include "dev/io_device.hh"
#include "dev/platform.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/SfPDMA.hh"
#include "sf_pdma_defs.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

/**
 * @file
 * Model of the SiFive FU540 Platform DMA engine.
 * Memcpy DMA engine with variable number of channels.
 *
 * Interface corresponds with the one described in Chapter 12 of
 * https://static.dev.sifive.com/FU540-C000-v1.0.pdf
 *
 * - Compatible with sf-pdma Linux driver;
 * - Supports out-of-order transactions mode.
 */

namespace gem5
{

class SfPDMA;

struct PDMATiming
{
    Cycles chanRegRdWrDelay;
    Cycles latBeforeBegin;
    Cycles latBeforeCompletion;
};

class PDMAChannel : public Drainable, public Serializable
{
  private:
    class PDMAPkt : public Packet::SenderState
    {
      public:
        std::unique_ptr<uint8_t[]> buf;
        const size_t bufSize;

        MemCmd::Command cmd;
        size_t pktSize;
        Tick pktSendTime;
        uint64_t id;

        PDMAPkt(size_t bufSize) : buf(new uint8_t[bufSize]), bufSize(bufSize)
        {}

        struct Comparator
        {
            bool
            operator()(PDMAPkt *a, PDMAPkt *b) const
            {
                return a->id > b->id;
            }
        };
    };

    class PDMAReqPort : public RequestPort
    {
      protected:
        PDMAChannel &chan;

        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;

      public:
        PDMAReqPort(const std::string &name, PDMAChannel &chan);
        virtual ~PDMAReqPort() {}
    };

    const size_t FIFO_SIZE = 256;

    // State
    PDMACtrl ctrl = 0;
    PDMARegs next = {0};
    PDMARegs exec = {0};

    /**
     * Driver requires pdma to stop transfer or dma request resulted with an
     * error (was stoped by iommu, for example)
     */
    bool shutdownDescr = false;

    bool busErr = false;
    //

    SfPDMA *pdma;
    const System *system;
    int chanId;
    PDMAReqPort dma;
    const RequestorID requestorId;
    const uint32_t streamId;

    int doneIrq;
    int errIrq;
    Platform *platform;

    size_t pktsN;

    PDMATiming delays;
    const uint32_t dmaWidth;

    // Descriptor processing

    /**
     * Pool (of size pktsN) of buffers used to generate pkts
     */
    std::queue<std::unique_ptr<PDMAPkt>> idlePkts;

    /**
     * Need to sort possible out-of-order Read responses when issuing several
     * parallel in-flight requests
     */
    std::priority_queue<PDMAPkt *, std::vector<PDMAPkt *>, PDMAPkt::Comparator>
            pendingPkts;

    size_t inFlightW = 0;
    size_t inFlightR = 0;
    size_t inFlightMax;

    /**
     * Use ChunkGenerators to handle unaligned src/dest addresses and to avoid
     * triggering cache system asserts about requests not fitting into cache
     * block
     */
    std::unique_ptr<ChunkGenerator> rChunks;
    std::unique_ptr<ChunkGenerator> wrChunks;

    Fifo<uint8_t> dataFifo;

    uint64_t idsForReads = 0;
    uint64_t pendingId = 0;

    PacketPtr nextTimingPkt = nullptr;
    bool waitingRetry = false;

    Tick startTime;

    /**
     * This function should be called when driver sets the 'run' bit.
     * Reset channel state according to new transfer params and start
     * descriptor processing.
     *
     * @param repeated Transfer was repeated because 'repeat' bit was set in
     *                 the config
     */
    void run(bool repeated);

    /**
     * Generate ReadReq/WriteReq pkts to read/write data to dataFifo.
     * Check that wrChunks or rChunks are not done and that there is enough
     * ready data in dataFifo for write request packet.
     * Return nulltpr when cannot assemble the packet.
     * For read packets don't check if there is any free space in fifo.
     * Pkt must be consumed by port or we will lose data.
     */
    PacketPtr buildPkt(PDMAPkt *pkt, MemCmd::Command cmd);

    /**
     * Entry point to start descriptor processing.
     */
    void sendPkts();

    bool readyForDrain();

    /**
     * Update Control register and trigger irqs when descriptor is processed.
     */
    void processCompletion();

    void sendAtomicPkts();

    // Timing mode
    void processTiming();
    void sendTimingPkts();

    /**
     * Pick up pkts from the priority_queue in the right order based on ids and
     * feed their data to fifo. Later write requests will consume it.
     */
    void fillFifo();

    /**
     * Functions to call from the recvTimingResp of the dma port.
     */
    void processRespSuccess(PDMAPkt *pkt);
    void processRespError(PDMAPkt *pkt);

    MemberEventWrapper<&PDMAChannel::sendPkts> sendPktsEvent;
    MemberEventWrapper<&PDMAChannel::processCompletion> sendComplEvent;

  protected:
    struct PDMAStats : public statistics::Group
    {
        PDMAStats(PDMAChannel &chan);

        statistics::Histogram turnaroundRdPkts;
        statistics::Histogram turnaroundWrPkts;
        statistics::Scalar bytesTransmitted;
        statistics::Scalar workTime;
        statistics::Formula averageSpeed;
    } stats;

  public:
    PDMAChannel(SfPDMA *pdma, const SfPDMAParams &p, int cid);
    virtual ~PDMAChannel() {}

    Port &getPort();
    std::string name() const;

    // Offset inside channel mem map
    Tick read(PacketPtr pkt, Addr addr, int size);
    Tick write(PacketPtr pkt, Addr addr, int size);

    DrainState drain() override;
    void drainResume() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

class SfPDMA : public PioDevice
{
  private:
    const Addr pioAddr;
    const uint8_t chanCnt;
    std::vector<std::unique_ptr<PDMAChannel>> chans;

    PDMAChannel *chanByAddr(Addr addr);

  public:
    SfPDMA(const SfPDMAParams &p);
    virtual ~SfPDMA() {}

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    AddrRangeList getAddrRanges() const override;

    Port &getPort(
            const std::string &if_name, PortID idx = InvalidPortID) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif

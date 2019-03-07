/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 *
 * Authors: Ali Saidi
 */

/* @file
 * Device model for Intel's 8254x line of gigabit ethernet controllers.
 */

#ifndef __DEV_NET_I8254XGBE_HH__
#define __DEV_NET_I8254XGBE_HH__

#include <deque>
#include <string>

#include "base/cp_annotate.hh"
#include "base/inet.hh"
#include "debug/EthernetDesc.hh"
#include "debug/EthernetIntr.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/i8254xGBe_defs.hh"
#include "dev/net/pktfifo.hh"
#include "dev/pci/device.hh"
#include "params/IGbE.hh"
#include "sim/eventq.hh"

class IGbEInt;

class IGbE : public EtherDevice
{
  private:
    IGbEInt *etherInt;
    CPA *cpa;

    // device registers
    iGbReg::Regs regs;

    // eeprom data, status and control bits
    int eeOpBits, eeAddrBits, eeDataBits;
    uint8_t eeOpcode, eeAddr;
    uint16_t flash[iGbReg::EEPROM_SIZE];

    // packet fifos
    PacketFifo rxFifo;
    PacketFifo txFifo;

    // Packet that we are currently putting into the txFifo
    EthPacketPtr txPacket;

    // Should to Rx/Tx State machine tick?
    bool inTick;
    bool rxTick;
    bool txTick;
    bool txFifoTick;

    bool rxDmaPacket;

    // Number of bytes copied from current RX packet
    unsigned pktOffset;

    // Delays in managaging descriptors
    Tick fetchDelay, wbDelay;
    Tick fetchCompDelay, wbCompDelay;
    Tick rxWriteDelay, txReadDelay;

    // Event and function to deal with RDTR timer expiring
    void rdtrProcess() {
        rxDescCache.writeback(0);
        DPRINTF(EthernetIntr,
                "Posting RXT interrupt because RDTR timer expired\n");
        postInterrupt(iGbReg::IT_RXT);
    }

    EventFunctionWrapper rdtrEvent;

    // Event and function to deal with RADV timer expiring
    void radvProcess() {
        rxDescCache.writeback(0);
        DPRINTF(EthernetIntr,
                "Posting RXT interrupt because RADV timer expired\n");
        postInterrupt(iGbReg::IT_RXT);
    }

    EventFunctionWrapper radvEvent;

    // Event and function to deal with TADV timer expiring
    void tadvProcess() {
        txDescCache.writeback(0);
        DPRINTF(EthernetIntr,
                "Posting TXDW interrupt because TADV timer expired\n");
        postInterrupt(iGbReg::IT_TXDW);
    }

    EventFunctionWrapper tadvEvent;

    // Event and function to deal with TIDV timer expiring
    void tidvProcess() {
        txDescCache.writeback(0);
        DPRINTF(EthernetIntr,
                "Posting TXDW interrupt because TIDV timer expired\n");
        postInterrupt(iGbReg::IT_TXDW);
    }
    EventFunctionWrapper tidvEvent;

    // Main event to tick the device
    void tick();
    EventFunctionWrapper tickEvent;


    uint64_t macAddr;

    void rxStateMachine();
    void txStateMachine();
    void txWire();

    /** Write an interrupt into the interrupt pending register and check mask
     * and interrupt limit timer before sending interrupt to CPU
     * @param t the type of interrupt we are posting
     * @param now should we ignore the interrupt limiting timer
     */
    void postInterrupt(iGbReg::IntTypes t, bool now = false);

    /** Check and see if changes to the mask register have caused an interrupt
     * to need to be sent or perhaps removed an interrupt cause.
     */
    void chkInterrupt();

    /** Send an interrupt to the cpu
     */
    void delayIntEvent();
    void cpuPostInt();
    // Event to moderate interrupts
    EventFunctionWrapper interEvent;

    /** Clear the interupt line to the cpu
     */
    void cpuClearInt();

    Tick intClock() { return SimClock::Int::ns * 1024; }

    /** This function is used to restart the clock so it can handle things like
     * draining and resume in one place. */
    void restartClock();

    /** Check if all the draining things that need to occur have occured and
     * handle the drain event if so.
     */
    void checkDrain();

    void anBegin(std::string sm, std::string st, int flags = CPA::FL_NONE) {
        if (cpa)
            cpa->hwBegin((CPA::flags)flags, sys, macAddr, sm, st);
    }

    void anQ(std::string sm, std::string q) {
        if (cpa)
            cpa->hwQ(CPA::FL_NONE, sys, macAddr, sm, q, macAddr);
    }

    void anDq(std::string sm, std::string q) {
        if (cpa)
            cpa->hwDq(CPA::FL_NONE, sys, macAddr, sm, q, macAddr);
    }

    void anPq(std::string sm, std::string q, int num = 1) {
        if (cpa)
            cpa->hwPq(CPA::FL_NONE, sys, macAddr, sm, q, macAddr, NULL, num);
    }

    void anRq(std::string sm, std::string q, int num = 1) {
        if (cpa)
            cpa->hwRq(CPA::FL_NONE, sys, macAddr, sm, q, macAddr, NULL, num);
    }

    void anWe(std::string sm, std::string q) {
        if (cpa)
            cpa->hwWe(CPA::FL_NONE, sys, macAddr, sm, q, macAddr);
    }

    void anWf(std::string sm, std::string q) {
        if (cpa)
            cpa->hwWf(CPA::FL_NONE, sys, macAddr, sm, q, macAddr);
    }


    template<class T>
    class DescCache : public Serializable
    {
      protected:
        virtual Addr descBase() const = 0;
        virtual long descHead() const = 0;
        virtual long descTail() const = 0;
        virtual long descLen() const = 0;
        virtual void updateHead(long h) = 0;
        virtual void enableSm() = 0;
        virtual void actionAfterWb() {}
        virtual void fetchAfterWb() = 0;

        typedef std::deque<T *> CacheType;
        CacheType usedCache;
        CacheType unusedCache;

        T *fetchBuf;
        T *wbBuf;

        // Pointer to the device we cache for
        IGbE *igbe;

        // Name of this  descriptor cache
        std::string _name;

        // How far we've cached
        int cachePnt;

        // The size of the descriptor cache
        int size;

        // How many descriptors we are currently fetching
        int curFetching;

        // How many descriptors we are currently writing back
        int wbOut;

        // if the we wrote back to the end of the descriptor ring and are going
        // to have to wrap and write more
        bool moreToWb;

        // What the alignment is of the next descriptor writeback
        Addr wbAlignment;

        /** The packet that is currently being dmad to memory if any */
        EthPacketPtr pktPtr;

        /** Shortcut for DMA address translation */
        Addr pciToDma(Addr a) { return igbe->pciToDma(a); }

      public:
        /** Annotate sm*/
        std::string annSmFetch, annSmWb, annUnusedDescQ, annUsedCacheQ,
            annUsedDescQ, annUnusedCacheQ, annDescQ;

        DescCache(IGbE *i, const std::string n, int s);
        virtual ~DescCache();

        std::string name() { return _name; }

        /** If the address/len/head change when we've got descriptors that are
         * dirty that is very bad. This function checks that we don't and if we
         * do panics.
         */
        void areaChanged();

        void writeback(Addr aMask);
        void writeback1();
        EventFunctionWrapper wbDelayEvent;

        /** Fetch a chunk of descriptors into the descriptor cache.
         * Calls fetchComplete when the memory system returns the data
         */
        void fetchDescriptors();
        void fetchDescriptors1();
        EventFunctionWrapper fetchDelayEvent;

        /** Called by event when dma to read descriptors is completed
         */
        void fetchComplete();
        EventFunctionWrapper fetchEvent;

        /** Called by event when dma to writeback descriptors is completed
         */
        void wbComplete();
        EventFunctionWrapper wbEvent;

        /* Return the number of descriptors left in the ring, so the device has
         * a way to figure out if it needs to interrupt.
         */
        unsigned
        descLeft() const
        {
            unsigned left = unusedCache.size();
            if (cachePnt > descTail())
                left += (descLen() - cachePnt + descTail());
            else
                left += (descTail() - cachePnt);

            return left;
        }

        /* Return the number of descriptors used and not written back.
         */
        unsigned descUsed() const { return usedCache.size(); }

        /* Return the number of cache unused descriptors we have. */
        unsigned descUnused() const { return unusedCache.size(); }

        /* Get into a state where the descriptor address/head/etc colud be
         * changed */
        void reset();


        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        virtual bool hasOutstandingEvents() {
            return wbEvent.scheduled() || fetchEvent.scheduled();
        }

    };


    class RxDescCache : public DescCache<iGbReg::RxDesc>
    {
      protected:
        Addr descBase() const override { return igbe->regs.rdba(); }
        long descHead() const override { return igbe->regs.rdh(); }
        long descLen() const override { return igbe->regs.rdlen() >> 4; }
        long descTail() const override { return igbe->regs.rdt(); }
        void updateHead(long h) override { igbe->regs.rdh(h); }
        void enableSm() override;
        void fetchAfterWb() override {
            if (!igbe->rxTick && igbe->drainState() == DrainState::Running)
                fetchDescriptors();
        }

        bool pktDone;

        /** Variable to head with header/data completion events */
        int splitCount;

        /** Bytes of packet that have been copied, so we know when to
            set EOP */
        unsigned bytesCopied;

      public:
        RxDescCache(IGbE *i, std::string n, int s);

        /** Write the given packet into the buffer(s) pointed to by the
         * descriptor and update the book keeping. Should only be called when
         * there are no dma's pending.
         * @param packet ethernet packet to write
         * @param pkt_offset bytes already copied from the packet to memory
         * @return pkt_offset + number of bytes copied during this call
         */
        int writePacket(EthPacketPtr packet, int pkt_offset);

        /** Called by event when dma to write packet is completed
         */
        void pktComplete();

        /** Check if the dma on the packet has completed and RX state machine
         * can continue
         */
        bool packetDone();

        EventFunctionWrapper pktEvent;

        // Event to handle issuing header and data write at the same time
        // and only callking pktComplete() when both are completed
        void pktSplitDone();
        EventFunctionWrapper pktHdrEvent;
        EventFunctionWrapper pktDataEvent;

        bool hasOutstandingEvents() override;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };
    friend class RxDescCache;

    RxDescCache rxDescCache;

    class TxDescCache  : public DescCache<iGbReg::TxDesc>
    {
      protected:
        Addr descBase() const override { return igbe->regs.tdba(); }
        long descHead() const override { return igbe->regs.tdh(); }
        long descTail() const override { return igbe->regs.tdt(); }
        long descLen() const override { return igbe->regs.tdlen() >> 4; }
        void updateHead(long h) override { igbe->regs.tdh(h); }
        void enableSm() override;
        void actionAfterWb() override;
        void fetchAfterWb() override {
            if (!igbe->txTick && igbe->drainState() == DrainState::Running)
                fetchDescriptors();
        }



        bool pktDone;
        bool isTcp;
        bool pktWaiting;
        bool pktMultiDesc;
        Addr completionAddress;
        bool completionEnabled;
        uint32_t descEnd;


        // tso variables
        bool useTso;
        Addr tsoHeaderLen;
        Addr tsoMss;
        Addr tsoTotalLen;
        Addr tsoUsedLen;
        Addr tsoPrevSeq;
        Addr tsoPktPayloadBytes;
        bool tsoLoadedHeader;
        bool tsoPktHasHeader;
        uint8_t tsoHeader[256];
        Addr tsoDescBytesUsed;
        Addr tsoCopyBytes;
        int tsoPkts;

      public:
        TxDescCache(IGbE *i, std::string n, int s);

        /** Tell the cache to DMA a packet from main memory into its buffer and
         * return the size the of the packet to reserve space in tx fifo.
         * @return size of the packet
         */
        unsigned getPacketSize(EthPacketPtr p);
        void getPacketData(EthPacketPtr p);
        void processContextDesc();

        /** Return the number of dsecriptors in a cache block for threshold
         * operations.
         */
        unsigned
        descInBlock(unsigned num_desc)
        {
            return num_desc / igbe->cacheBlockSize() / sizeof(iGbReg::TxDesc);
        }

        /** Ask if the packet has been transfered so the state machine can give
         * it to the fifo.
         * @return packet available in descriptor cache
         */
        bool packetAvailable();

        /** Ask if we are still waiting for the packet to be transfered.
         * @return packet still in transit.
         */
        bool packetWaiting() { return pktWaiting; }

        /** Ask if this packet is composed of multiple descriptors
         * so even if we've got data, we need to wait for more before
         * we can send it out.
         * @return packet can't be sent out because it's a multi-descriptor
         * packet
         */
        bool packetMultiDesc() { return pktMultiDesc;}

        /** Called by event when dma to write packet is completed
         */
        void pktComplete();
        EventFunctionWrapper pktEvent;

        void headerComplete();
        EventFunctionWrapper headerEvent;


        void completionWriteback(Addr a, bool enabled) {
            DPRINTF(EthernetDesc,
                    "Completion writeback Addr: %#x enabled: %d\n",
                    a, enabled);
            completionAddress = a;
            completionEnabled = enabled;
        }

        bool hasOutstandingEvents() override;

        void nullCallback() {
            DPRINTF(EthernetDesc, "Completion writeback complete\n");
        }
        EventFunctionWrapper nullEvent;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    friend class TxDescCache;

    TxDescCache txDescCache;

  public:
    typedef IGbEParams Params;
    const Params *
    params() const {
        return dynamic_cast<const Params *>(_params);
    }

    IGbE(const Params *params);
    ~IGbE();
    void init() override;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    Tick lastInterrupt;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    Tick writeConfig(PacketPtr pkt) override;

    bool ethRxPkt(EthPacketPtr packet);
    void ethTxDone();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    DrainState drain() override;
    void drainResume() override;

};

class IGbEInt : public EtherInt
{
  private:
    IGbE *dev;

  public:
    IGbEInt(const std::string &name, IGbE *d)
        : EtherInt(name), dev(d)
    { }

    virtual bool recvPacket(EthPacketPtr pkt) { return dev->ethRxPkt(pkt); }
    virtual void sendDone() { dev->ethTxDone(); }
};

#endif //__DEV_NET_I8254XGBE_HH__

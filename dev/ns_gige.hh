/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
 * Device module for modelling the National Semiconductor
 * DP83820 ethernet controller
 */

#ifndef __NS_GIGE_HH__
#define __NS_GIGE_HH__

#include "dev/dma.hh"
#include "dev/etherint.hh"
#include "dev/etherpkt.hh"
#include "sim/eventq.hh"
#include "dev/ns_gige_reg.h"
#include "base/statistics.hh"
#include "dev/pcidev.hh"
#include "dev/tsunami.hh"
#include "dev/pciconfigall.hh"

/** defined by the NS83820 data sheet */
#define MAX_TX_FIFO_SIZE 8192
#define MAX_RX_FIFO_SIZE 32768

/** length of ethernet address in bytes */
#define EADDR_LEN 6

/** Transmit State Machine states */
enum tx_state { txIdle, txDescRefr, txDescRead, txFifoBlock, txFragRead,
               txDescWrite };

/** Receive State Machine States */
enum rx_state { rxIdle, rxDescRefr, rxDescRead, rxFifoBlock, rxFragWrite,
                rxDescWrite, rxAdvance };

/**
 * Ethernet device registers
 */
struct dp_regs {
    uint32_t	command;
    uint32_t	config;
    uint32_t	mear;
    uint32_t	ptscr;
    uint32_t    isr;
    uint32_t    imr;
    uint32_t    ier;
    uint32_t    ihr;
    uint32_t    txdp;
    uint32_t    txdp_hi;
    uint32_t    txcfg;
    uint32_t    gpior;
    uint32_t    rxdp;
    uint32_t    rxdp_hi;
    uint32_t    rxcfg;
    uint32_t    pqcr;
    uint32_t    wcsr;
    uint32_t    pcr;
    uint32_t    rfcr;
    uint32_t    rfdr;
    uint32_t    srr;
    uint32_t    mibc;
    uint32_t    vrcr;
    uint32_t    vtcr;
    uint32_t    vdr;
    uint32_t    ccsr;
    uint32_t    tbicr;
    uint32_t    tbisr;
    uint32_t    tanar;
    uint32_t    tanlpar;
    uint32_t    taner;
    uint32_t    tesr;

    /** for perfect match memory.  the linux driver doesn't use any other ROM */
    uint8_t perfectMatch[EADDR_LEN];

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

/** an enum indicating direction, transmit or receive, used as a param for
    some fns */
enum dir_t { tx, rx };

class DmaEngine;
class IntrControl;
class EtherDevInt;
class PhysicalMemory;

/**
 * NS DP82830 Ethernet device model
 */
class EtherDev : public PciDev, public DmaHolder
{
  private:
    /** pointer to the chipset */
    Tsunami *tsunami;

  protected:
    Addr addr;
    Addr mask;

    /** device register file */
    dp_regs regs;

     /*** BASIC STRUCTURES FOR TX/RX ***/
    /* Data FIFOs */
    typedef std::deque<PacketPtr> pktbuf_t;
    typedef pktbuf_t::iterator pktiter_t;
    pktbuf_t txFifo;
    pktbuf_t rxFifo;

    /** for the tx side, to track addrs to write updated cmdsts to */
    typedef std::deque<uint32_t> txdpbuf_t; /* ASSUME32 */
    txdpbuf_t descAddrFifo;

    /** various helper vars */
    uint32_t txPacketLen;
    uint8_t *txPacketBufPtr;
    uint8_t *rxPacketBufPtr;
    uint8_t *rxDescBufPtr;
    uint32_t fragLen;
    uint32_t rxCopied;

    /** DescCaches */
    ns_desc txDescCache;
    ns_desc rxDescCache;

    /* tx State Machine */
    tx_state txState;
    /** Current Transmit Descriptor Done */
    bool CTDD;
    uint32_t txFifoCnt; /* amt of data in the txDataFifo in bytes (logical) */
    uint32_t txFifoAvail; /* current amt of free space in txDataFifo in byes */
    bool txHalt;
    bool txPacketFlag;  /* when set, indicates not working on a new packet */
    Addr txFragPtr; /* ptr to the next byte in the current fragment */
    uint32_t txDescCnt; /* count of bytes remaining in the current descriptor */

    /** rx State Machine */
    rx_state rxState;
    bool CRDD; /* Current Receive Descriptor Done */
    uint32_t rxPktBytes; /* num of bytes in the current packet being drained
                            from rxDataFifo */
    uint32_t rxFifoCnt; /* number of bytes in the rxFifo */
    bool rxHalt;
    bool rxPacketFlag;  /* when set, indicates not working on a new packet */
    Addr rxFragPtr; /* ptr to the next byte in current fragment */
    uint32_t rxDescCnt; /* count of bytes remaining in the current descriptor */

    bool extstsEnable;
    uint32_t maxTxBurst;
    uint32_t maxRxBurst;

    PhysicalMemory *physmem;

  protected:
    /**
     * Receive dma for descriptors done callback
     */
    class RxDescDone : public DmaCallback
    {
      public:
        EtherDev *ethernet;

      public:
        RxDescDone(EtherDev *e);
        std::string name() const;
        virtual void process();
    };

    /**
     * Receive dma done callback
     */
    class RxDone : public DmaCallback
    {
      public:
        EtherDev *ethernet;

      public:
        RxDone(EtherDev *e);
        std::string name() const;
        virtual void process();
    };

    /**
     * Transmit dma for descriptors done callback
     */
    class TxDescDone : public DmaCallback
    {
      public:
        EtherDev *ethernet;

      public:
        TxDescDone(EtherDev *e);
        std::string name() const;
        virtual void process();
    };

    /*
     * Transmit dma done callback
     */
    class TxDone : public DmaCallback
    {
      public:
        EtherDev *ethernet;
        PacketPtr packet;

      public:
        TxDone(EtherDev *e);
        std::string name() const;
        virtual void process();
    };

    friend class TxDescDone;
    friend class TxDone;
    friend class RxDescDone;
    friend class RxDone;

    RxDescDone rxDescDoneCB;
    RxDone rxDoneCB;
    TxDescDone txDescDoneCB;
    TxDone txDoneCB;

    DmaEngine *dma;
    DmaRequest readRequest;
    DmaRequest writeRequest;
    DmaRequest readDescRequest;
    DmaRequest writeDescRequest;
    PacketPtr rxPacket;
    DmaPhys readPhys;
    DmaPhys writePhys;
    DmaPhys readDescPhys;
    DmaPhys writeDescPhys;

    EtherDevInt *interface;

  protected:
    IntrControl *intctrl;
    Tick txDelay;
    Tick rxDelay;

    void txReset();
    void rxReset();
    void regsReset() {
        memset(&regs, 0, sizeof(regs));
        regs.mear = 0x12;
        regs.isr = 0x00608000;
        regs.txcfg = 0x120;
        regs.rxcfg = 0x4;
        regs.srr = 0x0103;
        regs.mibc = 0x2;
        regs.vdr = 0x81;
        regs.tesr = 0xc000;
    }

    void txKick();
    void rxKick();

    /*
     * Retransmit event
     */
    class TxEvent : public Event
    {
      protected:
        EtherDev *dev;

      public:
        TxEvent(EtherDev *_dev)
            : Event(&mainEventQueue), dev(_dev) {}
        void process() { dev->transmit(); }
        virtual const char *description() { return "retransmit"; }
    };
    friend class TxEvent;
    TxEvent txEvent;
    void transmit();


    void txDescDone();
    void rxDescDone();
    void txDone(PacketPtr packet);
    void rxDone();

    void txDump() const;
    void rxDump() const;

    void devIntrPost(uint32_t interrupts);
    void devIntrClear(uint32_t interrupts);
    void devIntrChangeMask();

    bool cpuPendingIntr;
    void cpuIntrPost();
    void cpuIntrClear();

    bool rxFilterEnable;
    bool rxFilter(PacketPtr packet);
    bool acceptBroadcast;
    bool acceptMulticast;
    bool acceptUnicast;
    bool acceptPerfect;
    bool acceptArp;

    bool udpChecksum(PacketPtr packet, bool gen);
    bool tcpChecksum(PacketPtr packet, bool gen);
    bool ipChecksum(PacketPtr packet, bool gen);
    uint16_t checksumCalc(uint16_t *pseudo, uint16_t *buf, uint32_t len);

  public:
    EtherDev(const std::string &name, DmaEngine *de, bool use_interface,
             IntrControl *i, MemoryController *mmu, PhysicalMemory *pmem,
             PCIConfigAll *cf, PciConfigData *cd, Tsunami *t, uint32_t bus,
             uint32_t dev, uint32_t func, bool rx_filter, const int eaddr[6],
             Tick tx_delay, Tick rx_delay, Addr addr, Addr mask);
    ~EtherDev();

    virtual void WriteConfig(int offset, int size, uint32_t data);
    virtual void ReadConfig(int offset, int size, uint8_t *data);



    Fault read(MemReqPtr req, uint8_t *data);
    Fault write(MemReqPtr req, const uint8_t *data);

    bool cpuIntrPending() const;
    void cpuIntrAck() { cpuIntrClear(); }

    bool recvPacket(PacketPtr packet);
    void transferDone();

    void setInterface(EtherDevInt *i) { assert(!interface); interface = i; }

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    virtual DmaRequest *find_dmareq(uint32_t &id) {
        if (id == 0)
            return(&readRequest);
        else if (id == 1)
            return(&writeRequest);
        else
            return(NULL);
    }

  public:
    void regStats();

  private:
    Statistics::Scalar<> txBytes;
    Statistics::Scalar<> rxBytes;
    Statistics::Scalar<> txPackets;
    Statistics::Scalar<> rxPackets;
    Statistics::Formula txBandwidth;
    Statistics::Formula rxBandwidth;
    Statistics::Formula txPacketRate;
    Statistics::Formula rxPacketRate;

    void readOneDesc(dir_t dir, uint32_t len = sizeof(ns_desc));
    void readOneFrag();
    void writeOneFrag();
};

/*
 * Ethernet Interface for an Ethernet Device
 */
class EtherDevInt : public EtherInt
{
  private:
    EtherDev *dev;

  public:
    EtherDevInt(const std::string &name, EtherDev *d)
        : EtherInt(name), dev(d) { dev->setInterface(this); }

    virtual bool recvPacket(PacketPtr &pkt) { return dev->recvPacket(pkt); }
    virtual void sendDone() { dev->transferDone(); }
};

#endif // __NS_GIGE_HH__

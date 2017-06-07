/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Lisa Hsu
 */

/** @file
 * Device module for modelling the National Semiconductor
 * DP83820 ethernet controller
 */

#ifndef __DEV_NET_NS_GIGE_HH__
#define __DEV_NET_NS_GIGE_HH__

#include "base/inet.hh"
#include "dev/io_device.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/ns_gige_reg.h"
#include "dev/net/pktfifo.hh"
#include "params/NSGigE.hh"
#include "sim/eventq.hh"

// Hash filtering constants
const uint16_t FHASH_ADDR  = 0x100;
const uint16_t FHASH_SIZE  = 0x100;

// EEPROM constants
const uint8_t  EEPROM_READ = 0x2;
const uint8_t  EEPROM_SIZE = 64; // Size in words of NSC93C46 EEPROM
const uint8_t  EEPROM_PMATCH2_ADDR = 0xA; // EEPROM Address of PMATCH word 2
const uint8_t  EEPROM_PMATCH1_ADDR = 0xB; // EEPROM Address of PMATCH word 1
const uint8_t  EEPROM_PMATCH0_ADDR = 0xC; // EEPROM Address of PMATCH word 0

/**
 * Ethernet device registers
 */
struct dp_regs {
    uint32_t    command;
    uint32_t    config;
    uint32_t    mear;
    uint32_t    ptscr;
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
    uint32_t    brar;
    uint32_t    brdr;
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
};

struct dp_rom {
    /**
     * for perfect match memory.
     * the linux driver doesn't use any other ROM
     */
    uint8_t perfectMatch[ETH_ADDR_LEN];

    /**
     * for hash table memory.
     * used by the freebsd driver
     */
    uint8_t filterHash[FHASH_SIZE];
};

class NSGigEInt;
class Packet;

/**
 * NS DP83820 Ethernet device model
 */
class NSGigE : public EtherDevBase
{
  public:
    /** Transmit State Machine states */
    enum TxState
    {
        txIdle,
        txDescRefr,
        txDescRead,
        txFifoBlock,
        txFragRead,
        txDescWrite,
        txAdvance
    };

    /** Receive State Machine States */
    enum RxState
    {
        rxIdle,
        rxDescRefr,
        rxDescRead,
        rxFifoBlock,
        rxFragWrite,
        rxDescWrite,
        rxAdvance
    };

    enum DmaState
    {
        dmaIdle,
        dmaReading,
        dmaWriting,
        dmaReadWaiting,
        dmaWriteWaiting
    };

    /** EEPROM State Machine States */
    enum EEPROMState
    {
        eepromStart,
        eepromGetOpcode,
        eepromGetAddress,
        eepromRead
    };

  protected:
    /** device register file */
    dp_regs regs;
    dp_rom rom;

    /** pci settings */
    bool ioEnable;
#if 0
    bool memEnable;
    bool bmEnable;
#endif

    /*** BASIC STRUCTURES FOR TX/RX ***/
    /* Data FIFOs */
    PacketFifo txFifo;
    PacketFifo rxFifo;

    /** various helper vars */
    EthPacketPtr txPacket;
    EthPacketPtr rxPacket;
    uint8_t *txPacketBufPtr;
    uint8_t *rxPacketBufPtr;
    uint32_t txXferLen;
    uint32_t rxXferLen;
    bool rxDmaFree;
    bool txDmaFree;

    /** DescCaches */
    ns_desc32 txDesc32;
    ns_desc32 rxDesc32;
    ns_desc64 txDesc64;
    ns_desc64 rxDesc64;

    /* tx State Machine */
    TxState txState;
    bool txEnable;

    /** Current Transmit Descriptor Done */
    bool CTDD;
    /** halt the tx state machine after next packet */
    bool txHalt;
    /** ptr to the next byte in the current fragment */
    Addr txFragPtr;
    /** count of bytes remaining in the current descriptor */
    uint32_t txDescCnt;
    DmaState txDmaState;

    /** rx State Machine */
    RxState rxState;
    bool rxEnable;

    /** Current Receive Descriptor Done */
    bool CRDD;
    /** num of bytes in the current packet being drained from rxDataFifo */
    uint32_t rxPktBytes;
    /** halt the rx state machine after current packet */
    bool rxHalt;
    /** ptr to the next byte in current fragment */
    Addr rxFragPtr;
    /** count of bytes remaining in the current descriptor */
    uint32_t rxDescCnt;
    DmaState rxDmaState;

    bool extstsEnable;

    /** EEPROM State Machine */
    EEPROMState eepromState;
    bool eepromClk;
    uint8_t eepromBitsToRx;
    uint8_t eepromOpcode;
    uint8_t eepromAddress;
    uint16_t eepromData;

  protected:
    Tick dmaReadDelay;
    Tick dmaWriteDelay;

    Tick dmaReadFactor;
    Tick dmaWriteFactor;

    void *rxDmaData;
    Addr  rxDmaAddr;
    int   rxDmaLen;
    bool  doRxDmaRead();
    bool  doRxDmaWrite();

    void *txDmaData;
    Addr  txDmaAddr;
    int   txDmaLen;
    bool  doTxDmaRead();
    bool  doTxDmaWrite();

    void rxDmaReadDone();
    EventFunctionWrapper rxDmaReadEvent;

    void rxDmaWriteDone();
    EventFunctionWrapper rxDmaWriteEvent;

    void txDmaReadDone();
    EventFunctionWrapper txDmaReadEvent;

    void txDmaWriteDone();
    EventFunctionWrapper txDmaWriteEvent;

    bool dmaDescFree;
    bool dmaDataFree;

  protected:
    Tick txDelay;
    Tick rxDelay;

    void txReset();
    void rxReset();
    void regsReset();

    void rxKick();
    Tick rxKickTick;
    EventFunctionWrapper rxKickEvent;

    void txKick();
    Tick txKickTick;
    EventFunctionWrapper txKickEvent;

    void eepromKick();

    /**
     * Retransmit event
     */
    void transmit();
    void txEventTransmit()
    {
        transmit();
        if (txState == txFifoBlock)
            txKick();
    }
    EventFunctionWrapper txEvent;

    void txDump() const;
    void rxDump() const;

    /**
     * receive address filter
     */
    bool rxFilterEnable;
    bool rxFilter(const EthPacketPtr &packet);
    bool acceptBroadcast;
    bool acceptMulticast;
    bool acceptUnicast;
    bool acceptPerfect;
    bool acceptArp;
    bool multicastHashEnable;

    /**
     * Interrupt management
     */
    void devIntrPost(uint32_t interrupts);
    void devIntrClear(uint32_t interrupts);
    void devIntrChangeMask();

    Tick intrDelay;
    Tick intrTick;
    bool cpuPendingIntr;
    void cpuIntrPost(Tick when);
    void cpuInterrupt();
    void cpuIntrClear();

    EventFunctionWrapper *intrEvent;
    NSGigEInt *interface;

  public:
    typedef NSGigEParams Params;
    const Params *params() const {
        return dynamic_cast<const Params *>(_params);
    }

    NSGigE(Params *params);
    ~NSGigE();

    EtherInt *getEthPort(const std::string &if_name, int idx) override;

    Tick writeConfig(PacketPtr pkt) override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    bool cpuIntrPending() const;
    void cpuIntrAck() { cpuIntrClear(); }

    bool recvPacket(EthPacketPtr packet);
    void transferDone();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void drainResume() override;
};

/*
 * Ethernet Interface for an Ethernet Device
 */
class NSGigEInt : public EtherInt
{
  private:
    NSGigE *dev;

  public:
    NSGigEInt(const std::string &name, NSGigE *d)
        : EtherInt(name), dev(d)
    { }

    virtual bool recvPacket(EthPacketPtr pkt) { return dev->recvPacket(pkt); }
    virtual void sendDone() { dev->transferDone(); }
};

#endif // __DEV_NET_NS_GIGE_HH__

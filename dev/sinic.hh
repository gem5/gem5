/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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

#ifndef __DEV_SINIC_HH__
#define __DEV_SINIC_HH__

#include "base/inet.hh"
#include "base/statistics.hh"
#include "dev/etherint.hh"
#include "dev/etherpkt.hh"
#include "dev/io_device.hh"
#include "dev/pcidev.hh"
#include "dev/pktfifo.hh"
#include "dev/sinicreg.hh"
#include "mem/bus/bus.hh"
#include "sim/eventq.hh"

namespace Sinic {

class Interface;
class Base : public PciDev
{
  protected:
    bool rxEnable;
    bool txEnable;

  protected:
    Tick intrDelay;
    Tick intrTick;
    bool cpuIntrEnable;
    bool cpuPendingIntr;
    void cpuIntrPost(Tick when);
    void cpuInterrupt();
    void cpuIntrClear();

    typedef EventWrapper<Base, &Base::cpuInterrupt> IntrEvent;
    friend class IntrEvent;
    IntrEvent *intrEvent;
    Interface *interface;

    bool cpuIntrPending() const;
    void cpuIntrAck() { cpuIntrClear(); }

/**
 * Serialization stuff
 */
  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

/**
 * Construction/Destruction/Parameters
 */
  public:
    struct Params : public PciDev::Params
    {
        Tick intr_delay;
    };

    Base(Params *p);
};

class Device : public Base
{
  protected:
    Platform *plat;
    PhysicalMemory *physmem;

  protected:
    /** Receive State Machine States */
    enum RxState {
        rxIdle,
        rxFifoBlock,
        rxBeginCopy,
        rxCopy,
        rxCopyDone
    };

    /** Transmit State Machine states */
    enum TxState {
        txIdle,
        txFifoBlock,
        txBeginCopy,
        txCopy,
        txCopyDone
    };

    /** device register file */
    struct {
        uint32_t Config;
        uint32_t RxMaxCopy;
        uint32_t TxMaxCopy;
        uint32_t RxThreshold;
        uint32_t TxThreshold;
        uint32_t IntrStatus;
        uint32_t IntrMask;
        uint64_t RxData;
        uint64_t RxDone;
        uint64_t TxData;
        uint64_t TxDone;
    } regs;

  private:
    Addr addr;
    static const Addr size = Regs::Size;

  protected:
    RxState rxState;
    PacketFifo rxFifo;
    PacketPtr rxPacket;
    uint8_t *rxPacketBufPtr;
    int rxPktBytes;
    uint64_t rxDoneData;
    Addr rxDmaAddr;
    uint8_t *rxDmaData;
    int rxDmaLen;

    TxState txState;
    PacketFifo txFifo;
    PacketPtr txPacket;
    uint8_t *txPacketBufPtr;
    int txPktBytes;
    Addr txDmaAddr;
    uint8_t *txDmaData;
    int txDmaLen;

  protected:
    void reset();

    void rxKick();
    Tick rxKickTick;
    typedef EventWrapper<Device, &Device::rxKick> RxKickEvent;
    friend class RxKickEvent;

    void txKick();
    Tick txKickTick;
    typedef EventWrapper<Device, &Device::txKick> TxKickEvent;
    friend class TxKickEvent;

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
    typedef EventWrapper<Device, &Device::txEventTransmit> TxEvent;
    friend class TxEvent;
    TxEvent txEvent;

    void txDump() const;
    void rxDump() const;

    /**
     * receive address filter
     */
    bool rxFilter(const PacketPtr &packet);

/**
 * device configuration
 */
    void changeConfig(uint32_t newconfig);

/**
 * device ethernet interface
 */
  public:
    bool recvPacket(PacketPtr packet);
    void transferDone();
    void setInterface(Interface *i) { assert(!interface); interface = i; }

/**
 * DMA parameters
 */
  protected:
    void rxDmaCopy();
    void rxDmaDone();
    friend class EventWrapper<Device, &Device::rxDmaDone>;
    EventWrapper<Device, &Device::rxDmaDone> rxDmaEvent;

    void txDmaCopy();
    void txDmaDone();
    friend class EventWrapper<Device, &Device::txDmaDone>;
    EventWrapper<Device, &Device::rxDmaDone> txDmaEvent;

    Tick dmaReadDelay;
    Tick dmaReadFactor;
    Tick dmaWriteDelay;
    Tick dmaWriteFactor;

/**
 * PIO parameters
 */
  protected:
    MemReqPtr rxPioRequest;
    MemReqPtr txPioRequest;

/**
 * Interrupt management
 */
  protected:
    void devIntrPost(uint32_t interrupts);
    void devIntrClear(uint32_t interrupts = Regs::Intr_All);
    void devIntrChangeMask(uint32_t newmask);

/**
 * PCI Configuration interface
 */
  public:
    virtual void WriteConfig(int offset, int size, uint32_t data);

/**
 * Memory Interface
 */
  public:
    virtual Fault read(MemReqPtr &req, uint8_t *data);
    virtual Fault write(MemReqPtr &req, const uint8_t *data);
    Tick cacheAccess(MemReqPtr &req);

/**
 * Statistics
 */
  private:
    Stats::Scalar<> rxBytes;
    Stats::Formula  rxBandwidth;
    Stats::Scalar<> rxPackets;
    Stats::Formula  rxPacketRate;
    Stats::Scalar<> rxIpPackets;
    Stats::Scalar<> rxTcpPackets;
    Stats::Scalar<> rxUdpPackets;
    Stats::Scalar<> rxIpChecksums;
    Stats::Scalar<> rxTcpChecksums;
    Stats::Scalar<> rxUdpChecksums;

    Stats::Scalar<> txBytes;
    Stats::Formula  txBandwidth;
    Stats::Formula totBandwidth;
    Stats::Formula totPackets;
    Stats::Formula totBytes;
    Stats::Formula totPacketRate;
    Stats::Scalar<> txPackets;
    Stats::Formula  txPacketRate;
    Stats::Scalar<> txIpPackets;
    Stats::Scalar<> txTcpPackets;
    Stats::Scalar<> txUdpPackets;
    Stats::Scalar<> txIpChecksums;
    Stats::Scalar<> txTcpChecksums;
    Stats::Scalar<> txUdpChecksums;

  public:
    virtual void regStats();

/**
 * Serialization stuff
 */
  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

/**
 * Construction/Destruction/Parameters
 */
  public:
    struct Params : public Base::Params
    {
        IntrControl *i;
        PhysicalMemory *pmem;
        Tick tx_delay;
        Tick rx_delay;
        HierParams *hier;
        Bus *header_bus;
        Bus *payload_bus;
        Tick pio_latency;
        PhysicalMemory *physmem;
        IntrControl *intctrl;
        bool rx_filter;
        Net::EthAddr eaddr;
        uint32_t rx_max_copy;
        uint32_t tx_max_copy;
        uint32_t rx_fifo_size;
        uint32_t tx_fifo_size;
        uint32_t rx_fifo_threshold;
        uint32_t tx_fifo_threshold;
        Tick dma_read_delay;
        Tick dma_read_factor;
        Tick dma_write_delay;
        Tick dma_write_factor;
    };

  protected:
    const Params *params() const { return (const Params *)_params; }

  public:
    Device(Params *params);
    ~Device();
};

/*
 * Ethernet Interface for an Ethernet Device
 */
class Interface : public EtherInt
{
  private:
    Device *dev;

  public:
    Interface(const std::string &name, Device *d)
        : EtherInt(name), dev(d) { dev->setInterface(this); }

    virtual bool recvPacket(PacketPtr pkt) { return dev->recvPacket(pkt); }
    virtual void sendDone() { dev->transferDone(); }
};

/* namespace Sinic */ }

#endif // __DEV_SINIC_HH__

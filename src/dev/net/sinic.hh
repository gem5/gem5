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
 */

#ifndef __DEV_NET_SINIC_HH__
#define __DEV_NET_SINIC_HH__

#include "base/inet.hh"
#include "base/statistics.hh"
#include "dev/io_device.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/pktfifo.hh"
#include "dev/net/sinicreg.hh"
#include "dev/pci/device.hh"
#include "params/Sinic.hh"
#include "sim/eventq.hh"

namespace Sinic {

class Interface;
class Base : public EtherDevBase
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

    EventFunctionWrapper *intrEvent;
    Interface *interface;

    bool cpuIntrPending() const;
    void cpuIntrAck() { cpuIntrClear(); }

/**
 * Serialization stuff
 */
  public:
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

/**
 * Construction/Destruction/Parameters
 */
  public:
    typedef SinicParams Params;
    const Params *params() const { return (const Params *)_params; }
    Base(const Params *p);
};

class Device : public Base
{
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
        uint32_t Config;       // 0x00
        uint32_t Command;      // 0x04
        uint32_t IntrStatus;   // 0x08
        uint32_t IntrMask;     // 0x0c
        uint32_t RxMaxCopy;    // 0x10
        uint32_t TxMaxCopy;    // 0x14
        uint32_t ZeroCopySize; // 0x18
        uint32_t ZeroCopyMark; // 0x1c
        uint32_t VirtualCount; // 0x20
        uint32_t RxMaxIntr;    // 0x24
        uint32_t RxFifoSize;   // 0x28
        uint32_t TxFifoSize;   // 0x2c
        uint32_t RxFifoLow;    // 0x30
        uint32_t TxFifoLow;    // 0x34
        uint32_t RxFifoHigh;   // 0x38
        uint32_t TxFifoHigh;   // 0x3c
        uint64_t RxData;       // 0x40
        uint64_t RxDone;       // 0x48
        uint64_t RxWait;       // 0x50
        uint64_t TxData;       // 0x58
        uint64_t TxDone;       // 0x60
        uint64_t TxWait;       // 0x68
        uint64_t HwAddr;       // 0x70
        uint64_t RxStatus;     // 0x78
    } regs;

    struct VirtualReg {
        uint64_t RxData;
        uint64_t RxDone;
        uint64_t TxData;
        uint64_t TxDone;

        PacketFifo::iterator rxIndex;
        unsigned rxPacketOffset;
        unsigned rxPacketBytes;
        uint64_t rxDoneData;

        Counter rxUnique;
        Counter txUnique;

        VirtualReg()
            : RxData(0), RxDone(0), TxData(0), TxDone(0),
              rxPacketOffset(0), rxPacketBytes(0), rxDoneData(0)
        { }
    };
    typedef std::vector<VirtualReg> VirtualRegs;
    typedef std::list<unsigned> VirtualList;
    Counter rxUnique;
    Counter txUnique;
    VirtualRegs virtualRegs;
    VirtualList rxList;
    VirtualList rxBusy;
    int rxActive;
    VirtualList txList;

    int rxBusyCount;
    int rxMappedCount;
    int rxDirtyCount;

    uint8_t  &regData8(Addr daddr) { return *((uint8_t *)&regs + daddr); }
    uint32_t &regData32(Addr daddr) { return *(uint32_t *)&regData8(daddr); }
    uint64_t &regData64(Addr daddr) { return *(uint64_t *)&regData8(daddr); }

  protected:
    RxState rxState;
    PacketFifo rxFifo;
    PacketFifo::iterator rxFifoPtr;
    bool rxEmpty;
    bool rxLow;
    Addr rxDmaAddr;
    uint8_t *rxDmaData;
    unsigned rxDmaLen;

    TxState txState;
    PacketFifo txFifo;
    bool txFull;
    EthPacketPtr txPacket;
    int txPacketOffset;
    int txPacketBytes;
    Addr txDmaAddr;
    uint8_t *txDmaData;
    int txDmaLen;

  protected:
    void reset();

    void rxKick();
    Tick rxKickTick;

    void txKick();
    Tick txKickTick;

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
    bool rxFilter(const EthPacketPtr &packet);

/**
 * device configuration
 */
    void changeConfig(uint32_t newconfig);
    void command(uint32_t command);

/**
 * device ethernet interface
 */
  public:
    bool recvPacket(EthPacketPtr packet);
    void transferDone();
    EtherInt *getEthPort(const std::string &if_name, int idx) override;

/**
 * DMA parameters
 */
  protected:
    void rxDmaDone();
    EventFunctionWrapper rxDmaEvent;

    void txDmaDone();
    EventFunctionWrapper txDmaEvent;

    Tick dmaReadDelay;
    Tick dmaReadFactor;
    Tick dmaWriteDelay;
    Tick dmaWriteFactor;

/**
 * Interrupt management
 */
  protected:
    void devIntrPost(uint32_t interrupts);
    void devIntrClear(uint32_t interrupts = Regs::Intr_All);
    void devIntrChangeMask(uint32_t newmask);

/**
 * Memory Interface
 */
  public:
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    virtual void drainResume() override;

    void prepareIO(ContextID cpu, int index);
    void prepareRead(ContextID cpu, int index);
    void prepareWrite(ContextID cpu, int index);
 //   Fault iprRead(Addr daddr, ContextID cpu, uint64_t &result);

/**
 * Statistics
 */
  private:
    Stats::Scalar totalVnicDistance;
    Stats::Scalar numVnicDistance;
    Stats::Scalar maxVnicDistance;
    Stats::Formula avgVnicDistance;

    int _maxVnicDistance;

  public:
    void regStats() override;
    void resetStats() override;

/**
 * Serialization stuff
 */
  public:
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public:
    Device(const Params *p);
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
        : EtherInt(name), dev(d)
    { }

    virtual bool recvPacket(EthPacketPtr pkt) { return dev->recvPacket(pkt); }
    virtual void sendDone() { dev->transferDone(); }
};

} // namespace Sinic

#endif // __DEV_NET_SINIC_HH__

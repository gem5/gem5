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
 */

#include <cstdio>
#include <deque>
#include <string>

#include "base/inet.hh"
#include "cpu/exec_context.hh"
#include "cpu/intr_control.hh"
#include "dev/etherlink.hh"
#include "dev/sinic.hh"
#include "dev/pciconfigall.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/dma_interface.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "sim/builder.hh"
#include "sim/debug.hh"
#include "sim/eventq.hh"
#include "sim/host.hh"
#include "sim/stats.hh"
#include "targetarch/vtophys.hh"

using namespace Net;

namespace Sinic {

const char *RxStateStrings[] =
{
    "rxIdle",
    "rxFifoBlock",
    "rxBeginCopy",
    "rxCopy",
    "rxCopyDone"
};

const char *TxStateStrings[] =
{
    "txIdle",
    "txFifoBlock",
    "txBeginCopy",
    "txCopy",
    "txCopyDone"
};


///////////////////////////////////////////////////////////////////////
//
// Sinic PCI Device
//
Base::Base(Params *p)
    : PciDev(p), rxEnable(false), txEnable(false), cycleTime(p->cycle_time),
      intrDelay(p->intr_delay), intrTick(0), cpuIntrEnable(false),
      cpuPendingIntr(false), intrEvent(0), interface(NULL)
{
}

Device::Device(Params *p)
    : Base(p), plat(p->plat), physmem(p->physmem),
      rxFifo(p->rx_fifo_size), txFifo(p->tx_fifo_size),
      rxKickTick(0), txKickTick(0),
      txEvent(this), rxDmaEvent(this), txDmaEvent(this),
      dmaReadDelay(p->dma_read_delay), dmaReadFactor(p->dma_read_factor),
      dmaWriteDelay(p->dma_write_delay), dmaWriteFactor(p->dma_write_factor)
{
    reset();

    if (p->io_bus) {
        pioInterface = newPioInterface(p->name, p->hier, p->io_bus, this,
                                       &Device::cacheAccess);

        pioLatency = p->pio_latency * p->io_bus->clockRate;

        if (p->payload_bus)
            dmaInterface = new DMAInterface<Bus>(p->name + ".dma", p->io_bus,
                                                 p->payload_bus, 1,
                                                 p->dma_no_allocate);
        else
            dmaInterface = new DMAInterface<Bus>(p->name + ".dma", p->io_bus,
                                                 p->io_bus, 1,
                                                 p->dma_no_allocate);
    } else if (p->payload_bus) {
        pioInterface = newPioInterface(p->name, p->hier, p->payload_bus, this,
                                       &Device::cacheAccess);

        pioLatency = p->pio_latency * p->payload_bus->clockRate;

        dmaInterface = new DMAInterface<Bus>(p->name + ".dma", p->payload_bus,
                                             p->payload_bus, 1,
                                             p->dma_no_allocate);
    }
}

Device::~Device()
{}

void
Device::regStats()
{
    rxBytes
        .name(name() + ".rxBytes")
        .desc("Bytes Received")
        .prereq(rxBytes)
        ;

    rxBandwidth
        .name(name() + ".rxBandwidth")
        .desc("Receive Bandwidth (bits/s)")
        .precision(0)
        .prereq(rxBytes)
        ;

    rxPackets
        .name(name() + ".rxPackets")
        .desc("Number of Packets Received")
        .prereq(rxBytes)
        ;

    rxPacketRate
        .name(name() + ".rxPPS")
        .desc("Packet Reception Rate (packets/s)")
        .precision(0)
        .prereq(rxBytes)
        ;

    rxIpPackets
        .name(name() + ".rxIpPackets")
        .desc("Number of IP Packets Received")
        .prereq(rxBytes)
        ;

    rxTcpPackets
        .name(name() + ".rxTcpPackets")
        .desc("Number of Packets Received")
        .prereq(rxBytes)
        ;

    rxUdpPackets
        .name(name() + ".rxUdpPackets")
        .desc("Number of UDP Packets Received")
        .prereq(rxBytes)
        ;

    rxIpChecksums
        .name(name() + ".rxIpChecksums")
        .desc("Number of rx IP Checksums done by device")
        .precision(0)
        .prereq(rxBytes)
        ;

    rxTcpChecksums
        .name(name() + ".rxTcpChecksums")
        .desc("Number of rx TCP Checksums done by device")
        .precision(0)
        .prereq(rxBytes)
        ;

    rxUdpChecksums
        .name(name() + ".rxUdpChecksums")
        .desc("Number of rx UDP Checksums done by device")
        .precision(0)
        .prereq(rxBytes)
        ;

    totBandwidth
        .name(name() + ".totBandwidth")
        .desc("Total Bandwidth (bits/s)")
        .precision(0)
        .prereq(totBytes)
        ;

    totPackets
        .name(name() + ".totPackets")
        .desc("Total Packets")
        .precision(0)
        .prereq(totBytes)
        ;

    totBytes
        .name(name() + ".totBytes")
        .desc("Total Bytes")
        .precision(0)
        .prereq(totBytes)
        ;

    totPacketRate
        .name(name() + ".totPPS")
        .desc("Total Tranmission Rate (packets/s)")
        .precision(0)
        .prereq(totBytes)
        ;

    txBytes
        .name(name() + ".txBytes")
        .desc("Bytes Transmitted")
        .prereq(txBytes)
        ;

    txBandwidth
        .name(name() + ".txBandwidth")
        .desc("Transmit Bandwidth (bits/s)")
        .precision(0)
        .prereq(txBytes)
        ;

    txPackets
        .name(name() + ".txPackets")
        .desc("Number of Packets Transmitted")
        .prereq(txBytes)
        ;

    txPacketRate
        .name(name() + ".txPPS")
        .desc("Packet Tranmission Rate (packets/s)")
        .precision(0)
        .prereq(txBytes)
        ;

    txIpPackets
        .name(name() + ".txIpPackets")
        .desc("Number of IP Packets Transmitted")
        .prereq(txBytes)
        ;

    txTcpPackets
        .name(name() + ".txTcpPackets")
        .desc("Number of TCP Packets Transmitted")
        .prereq(txBytes)
        ;

    txUdpPackets
        .name(name() + ".txUdpPackets")
        .desc("Number of Packets Transmitted")
        .prereq(txBytes)
        ;

    txIpChecksums
        .name(name() + ".txIpChecksums")
        .desc("Number of tx IP Checksums done by device")
        .precision(0)
        .prereq(txBytes)
        ;

    txTcpChecksums
        .name(name() + ".txTcpChecksums")
        .desc("Number of tx TCP Checksums done by device")
        .precision(0)
        .prereq(txBytes)
        ;

    txUdpChecksums
        .name(name() + ".txUdpChecksums")
        .desc("Number of tx UDP Checksums done by device")
        .precision(0)
        .prereq(txBytes)
        ;

    txBandwidth = txBytes * Stats::constant(8) / simSeconds;
    rxBandwidth = rxBytes * Stats::constant(8) / simSeconds;
    totBandwidth = txBandwidth + rxBandwidth;
    totBytes = txBytes + rxBytes;
    totPackets = txPackets + rxPackets;
    txPacketRate = txPackets / simSeconds;
    rxPacketRate = rxPackets / simSeconds;
}

/**
 * This is to write to the PCI general configuration registers
 */
void
Device::WriteConfig(int offset, int size, uint32_t data)
{
    switch (offset) {
      case PCI0_BASE_ADDR0:
        // Need to catch writes to BARs to update the PIO interface
        PciDev::WriteConfig(offset, size, data);
        if (BARAddrs[0] != 0) {
            if (pioInterface)
                pioInterface->addAddrRange(RangeSize(BARAddrs[0], BARSize[0]));

            BARAddrs[0] &= EV5::PAddrUncachedMask;
        }
        break;

      default:
        PciDev::WriteConfig(offset, size, data);
    }
}

/**
 * This reads the device registers, which are detailed in the NS83820
 * spec sheet
 */
Fault
Device::read(MemReqPtr &req, uint8_t *data)
{
    assert(config.hdr.command & PCI_CMD_MSE);

    //The mask is to give you only the offset into the device register file
    Addr daddr = req->paddr & 0xfff;

    if (Regs::regSize(daddr) == 0)
        panic("invalid address: da=%#x pa=%#x va=%#x size=%d",
              daddr, req->paddr, req->vaddr, req->size);

    if (req->size != Regs::regSize(daddr))
        panic("invalid size for reg %s: da=%#x pa=%#x va=%#x size=%d",
              Regs::regName(daddr), daddr, req->paddr, req->vaddr, req->size);

    DPRINTF(EthernetPIO, "read reg=%s da=%#x pa=%#x va=%#x size=%d\n",
            Regs::regName(daddr), daddr, req->paddr, req->vaddr, req->size);

    uint32_t &reg32 = *(uint32_t *)data;
    uint64_t &reg64 = *(uint64_t *)data;

    switch (daddr) {
      case Regs::Config:
        reg32 = regs.Config;
        break;

      case Regs::RxMaxCopy:
        reg32 = regs.RxMaxCopy;
        break;

      case Regs::TxMaxCopy:
        reg32 = regs.TxMaxCopy;
        break;

      case Regs::RxThreshold:
        reg32 = regs.RxThreshold;
        break;

      case Regs::TxThreshold:
        reg32 = regs.TxThreshold;
        break;

      case Regs::IntrStatus:
        reg32 = regs.IntrStatus;
        devIntrClear();
        break;

      case Regs::IntrMask:
        reg32 = regs.IntrMask;
        break;

      case Regs::RxData:
        reg64 = regs.RxData;
        break;

      case Regs::RxDone:
      case Regs::RxWait:
        reg64 = Regs::set_RxDone_FifoLen(regs.RxDone,
                                         min(rxFifo.packets(), 255));
        break;

      case Regs::TxData:
        reg64 = regs.TxData;
        break;

      case Regs::TxDone:
      case Regs::TxWait:
        reg64 = Regs::set_TxDone_FifoLen(regs.TxDone,
                                         min(txFifo.packets(), 255));
        break;

      case Regs::HwAddr:
        reg64 = params()->eaddr;
        break;

      default:
        panic("reading write only register %s: da=%#x pa=%#x va=%#x size=%d",
              Regs::regName(daddr), daddr, req->paddr, req->vaddr, req->size);
    }

    DPRINTF(EthernetPIO, "read reg=%s done val=%#x\n", Regs::regName(daddr),
            Regs::regSize(daddr) == 4 ? reg32 : reg64);

    return No_Fault;
}

Fault
Device::write(MemReqPtr &req, const uint8_t *data)
{
    assert(config.hdr.command & PCI_CMD_MSE);
    Addr daddr = req->paddr & 0xfff;

    if (Regs::regSize(daddr) == 0)
        panic("invalid address: da=%#x pa=%#x va=%#x size=%d",
              daddr, req->paddr, req->vaddr, req->size);

    if (req->size != Regs::regSize(daddr))
        panic("invalid size: reg=%s da=%#x pa=%#x va=%#x size=%d",
              Regs::regName(daddr), daddr, req->paddr, req->vaddr, req->size);

    uint32_t reg32 = *(uint32_t *)data;
    uint64_t reg64 = *(uint64_t *)data;

    DPRINTF(EthernetPIO, "write reg=%s val=%#x da=%#x pa=%#x va=%#x size=%d\n",
            Regs::regName(daddr), Regs::regSize(daddr) == 4 ? reg32 : reg64,
            daddr, req->paddr, req->vaddr, req->size);


    switch (daddr) {
      case Regs::Config:
        changeConfig(reg32);
        break;

      case Regs::RxThreshold:
        regs.RxThreshold = reg32;
        break;

      case Regs::TxThreshold:
        regs.TxThreshold = reg32;
        break;

      case Regs::IntrMask:
        devIntrChangeMask(reg32);
        break;

      case Regs::RxData:
        if (rxState != rxIdle)
            panic("receive machine busy with another request!");

        regs.RxDone = 0;
        regs.RxData = reg64;
        if (rxEnable) {
            rxState = rxFifoBlock;
            rxKick();
        }
        break;

      case Regs::TxData:
        if (txState != txIdle)
            panic("transmit machine busy with another request!");

        regs.TxDone = 0;
        regs.TxData = reg64;
        if (txEnable) {
            txState = txFifoBlock;
            txKick();
        }
        break;

      default:
        panic("writing read only register %s: da=%#x pa=%#x va=%#x size=%d",
              Regs::regName(daddr), daddr, req->paddr, req->vaddr, req->size);
    }

    return No_Fault;
}

void
Device::devIntrPost(uint32_t interrupts)
{
    if ((interrupts & Regs::Intr_Res))
        panic("Cannot set a reserved interrupt");

    regs.IntrStatus |= interrupts;

    DPRINTF(EthernetIntr,
            "interrupt written to intStatus: intr=%#x status=%#x mask=%#x\n",
            interrupts, regs.IntrStatus, regs.IntrMask);

    if ((regs.IntrStatus & regs.IntrMask)) {
        Tick when = curTick;
        if ((regs.IntrStatus & regs.IntrMask & Regs::Intr_NoDelay) == 0)
            when += intrDelay;
        cpuIntrPost(when);
    }
}

void
Device::devIntrClear(uint32_t interrupts)
{
    if ((interrupts & Regs::Intr_Res))
        panic("Cannot clear a reserved interrupt");

    regs.IntrStatus &= ~interrupts;

    DPRINTF(EthernetIntr,
            "interrupt cleared from intStatus: intr=%x status=%x mask=%x\n",
            interrupts, regs.IntrStatus, regs.IntrMask);

    if (!(regs.IntrStatus & regs.IntrMask))
        cpuIntrClear();
}

void
Device::devIntrChangeMask(uint32_t newmask)
{
    if (regs.IntrMask == newmask)
        return;

    regs.IntrMask = newmask;

    DPRINTF(EthernetIntr,
            "interrupt mask changed: intStatus=%x intMask=%x masked=%x\n",
            regs.IntrStatus, regs.IntrMask, regs.IntrStatus & regs.IntrMask);

    if (regs.IntrStatus & regs.IntrMask)
        cpuIntrPost(curTick);
    else
        cpuIntrClear();
}

void
Base::cpuIntrPost(Tick when)
{
    // If the interrupt you want to post is later than an interrupt
    // already scheduled, just let it post in the coming one and don't
    // schedule another.
    // HOWEVER, must be sure that the scheduled intrTick is in the
    // future (this was formerly the source of a bug)
    /**
     * @todo this warning should be removed and the intrTick code should
     * be fixed.
     */
    assert(when >= curTick);
    assert(intrTick >= curTick || intrTick == 0);
    if (!cpuIntrEnable) {
        DPRINTF(EthernetIntr, "interrupts not enabled.\n",
                intrTick);
        return;
    }

    if (when > intrTick && intrTick != 0) {
        DPRINTF(EthernetIntr, "don't need to schedule event...intrTick=%d\n",
                intrTick);
        return;
    }

    intrTick = when;
    if (intrTick < curTick) {
        debug_break();
        intrTick = curTick;
    }

    DPRINTF(EthernetIntr, "going to schedule an interrupt for intrTick=%d\n",
            intrTick);

    if (intrEvent)
        intrEvent->squash();
    intrEvent = new IntrEvent(this, true);
    intrEvent->schedule(intrTick);
}

void
Base::cpuInterrupt()
{
    assert(intrTick == curTick);

    // Whether or not there's a pending interrupt, we don't care about
    // it anymore
    intrEvent = 0;
    intrTick = 0;

    // Don't send an interrupt if there's already one
    if (cpuPendingIntr) {
        DPRINTF(EthernetIntr,
                "would send an interrupt now, but there's already pending\n");
    } else {
        // Send interrupt
        cpuPendingIntr = true;

        DPRINTF(EthernetIntr, "posting interrupt\n");
        intrPost();
    }
}

void
Base::cpuIntrClear()
{
    if (!cpuPendingIntr)
        return;

    if (intrEvent) {
        intrEvent->squash();
        intrEvent = 0;
    }

    intrTick = 0;

    cpuPendingIntr = false;

    DPRINTF(EthernetIntr, "clearing cchip interrupt\n");
    intrClear();
}

bool
Base::cpuIntrPending() const
{ return cpuPendingIntr; }

void
Device::changeConfig(uint32_t newconf)
{
    uint32_t changed = regs.Config ^ newconf;
    if (!changed)
        return;

    regs.Config = newconf;

    if ((changed & Regs::Config_Reset)) {
        assert(regs.Config & Regs::Config_Reset);
        reset();
        regs.Config &= ~Regs::Config_Reset;
    }

    if ((changed & Regs::Config_IntEn)) {
        cpuIntrEnable = regs.Config & Regs::Config_IntEn;
        if (cpuIntrEnable) {
            if (regs.IntrStatus & regs.IntrMask)
                cpuIntrPost(curTick);
        } else {
            cpuIntrClear();
        }
    }

    if ((changed & Regs::Config_TxEn)) {
        txEnable = regs.Config & Regs::Config_TxEn;
        if (txEnable)
            txKick();
    }

    if ((changed & Regs::Config_RxEn)) {
        rxEnable = regs.Config & Regs::Config_RxEn;
        if (rxEnable)
            rxKick();
    }
}

void
Device::reset()
{
    using namespace Regs;
    memset(&regs, 0, sizeof(regs));
    regs.RxMaxCopy = params()->rx_max_copy;
    regs.TxMaxCopy = params()->tx_max_copy;
    regs.IntrMask = Intr_TxFifo | Intr_RxFifo | Intr_RxData;

    rxState = rxIdle;
    txState = txIdle;

    rxFifo.clear();
    txFifo.clear();
}

void
Device::rxDmaCopy()
{
    assert(rxState == rxCopy);
    rxState = rxCopyDone;
    physmem->dma_write(rxDmaAddr, (uint8_t *)rxDmaData, rxDmaLen);
    DPRINTF(EthernetDMA, "rx dma write paddr=%#x len=%d\n",
            rxDmaAddr, rxDmaLen);
    DDUMP(EthernetDMA, rxDmaData, rxDmaLen);
}

void
Device::rxDmaDone()
{
    rxDmaCopy();
    rxKick();
}

void
Device::rxKick()
{
    DPRINTF(EthernetSM, "receive kick rxState=%s (rxFifo.size=%d)\n",
            RxStateStrings[rxState], rxFifo.size());

    if (rxKickTick > curTick) {
        DPRINTF(EthernetSM, "receive kick exiting, can't run till %d\n",
                rxKickTick);
        return;
    }

  next:
    switch (rxState) {
      case rxIdle:
        if (rxPioRequest) {
            pioInterface->respond(rxPioRequest, curTick);
            rxPioRequest = 0;
        }
        goto exit;

      case rxFifoBlock:
        if (rxPacket) {
            rxState = rxBeginCopy;
            break;
        }

        if (rxFifo.empty()) {
            DPRINTF(EthernetSM, "receive waiting for data.  Nothing to do.\n");
            goto exit;
        }

        // Grab a new packet from the fifo.
        rxPacket = rxFifo.front();
        rxPacketBufPtr = rxPacket->data;
        rxPktBytes = rxPacket->length;
        assert(rxPktBytes);

        rxDoneData = 0;
        /* scope for variables */ {
            IpPtr ip(rxPacket);
            if (ip) {
                rxDoneData |= Regs::RxDone_IpPacket;
                rxIpChecksums++;
                if (cksum(ip) != 0) {
                    DPRINTF(EthernetCksum, "Rx IP Checksum Error\n");
                    rxDoneData |= Regs::RxDone_IpError;
                }
                TcpPtr tcp(ip);
                UdpPtr udp(ip);
                if (tcp) {
                    rxDoneData |= Regs::RxDone_TcpPacket;
                    rxTcpChecksums++;
                    if (cksum(tcp) != 0) {
                        DPRINTF(EthernetCksum, "Rx TCP Checksum Error\n");
                        rxDoneData |= Regs::RxDone_TcpError;
                    }
                } else if (udp) {
                    rxDoneData |= Regs::RxDone_UdpPacket;
                    rxUdpChecksums++;
                    if (cksum(udp) != 0) {
                        DPRINTF(EthernetCksum, "Rx UDP Checksum Error\n");
                        rxDoneData |= Regs::RxDone_UdpError;
                    }
                }
            }
        }
        rxState = rxBeginCopy;
        break;

      case rxBeginCopy:
        rxDmaAddr = plat->pciToDma(Regs::get_RxData_Addr(regs.RxData));
        rxDmaLen = min<int>(Regs::get_RxData_Len(regs.RxData), rxPktBytes);
        rxDmaData = rxPacketBufPtr;

        if (dmaInterface) {
            if (!dmaInterface->busy()) {
                dmaInterface->doDMA(WriteInvalidate, rxDmaAddr, rxDmaLen,
                                    curTick, &rxDmaEvent, true);
                rxState = rxCopy;
            }
            goto exit;
        }

        rxState = rxCopy;
        if (dmaWriteDelay != 0 || dmaWriteFactor != 0) {
            Tick factor = ((rxDmaLen + ULL(63)) >> ULL(6)) * dmaWriteFactor;
            Tick start = curTick + dmaWriteDelay + factor;
            rxDmaEvent.schedule(start);
            goto exit;
        }

        rxDmaCopy();
        break;

      case rxCopy:
        DPRINTF(EthernetSM, "receive machine still copying\n");
        goto exit;

      case rxCopyDone:
        regs.RxDone = rxDoneData | rxDmaLen;

        if (rxPktBytes == rxDmaLen) {
            rxPacket = NULL;
            rxFifo.pop();
        } else {
            regs.RxDone |= Regs::RxDone_More;
            rxPktBytes -= rxDmaLen;
            rxPacketBufPtr += rxDmaLen;
        }

        regs.RxDone |= Regs::RxDone_Complete;
        devIntrPost(Regs::Intr_RxData);
        rxState = rxIdle;
        break;

      default:
        panic("Invalid rxState!");
    }

    DPRINTF(EthernetSM, "entering next rxState=%s\n",
            RxStateStrings[rxState]);

    goto next;

  exit:
    /**
     * @todo do we want to schedule a future kick?
     */
    DPRINTF(EthernetSM, "rx state machine exited rxState=%s\n",
            RxStateStrings[rxState]);
}

void
Device::txDmaCopy()
{
    assert(txState == txCopy);
    txState = txCopyDone;
    physmem->dma_read((uint8_t *)txDmaData, txDmaAddr, txDmaLen);
    DPRINTF(EthernetDMA, "tx dma read paddr=%#x len=%d\n",
            txDmaAddr, txDmaLen);
    DDUMP(EthernetDMA, txDmaData, txDmaLen);
}

void
Device::txDmaDone()
{
    txDmaCopy();
    txKick();
}

void
Device::transmit()
{
    if (txFifo.empty()) {
        DPRINTF(Ethernet, "nothing to transmit\n");
        return;
    }

    PacketPtr packet = txFifo.front();
    if (!interface->sendPacket(packet)) {
        DPRINTF(Ethernet, "Packet Transmit: failed txFifo available %d\n",
                txFifo.avail());
        goto reschedule;
    }

    txFifo.pop();

#if TRACING_ON
    if (DTRACE(Ethernet)) {
        IpPtr ip(packet);
        if (ip) {
            DPRINTF(Ethernet, "ID is %d\n", ip->id());
            TcpPtr tcp(ip);
            if (tcp) {
                DPRINTF(Ethernet, "Src Port=%d, Dest Port=%d\n",
                        tcp->sport(), tcp->dport());
            }
        }
    }
#endif

    DDUMP(Ethernet, packet->data, packet->length);
    txBytes += packet->length;
    txPackets++;

    DPRINTF(Ethernet, "Packet Transmit: successful txFifo Available %d\n",
            txFifo.avail());

    if (txFifo.size() <= params()->tx_fifo_threshold)
        devIntrPost(Regs::Intr_TxFifo);

    devIntrPost(Regs::Intr_TxDone);

  reschedule:
   if (!txFifo.empty() && !txEvent.scheduled()) {
       DPRINTF(Ethernet, "reschedule transmit\n");
       txEvent.schedule(curTick + retryTime);
   }
}

void
Device::txKick()
{
    DPRINTF(EthernetSM, "transmit kick txState=%s (txFifo.size=%d)\n",
            TxStateStrings[txState], txFifo.size());

    if (txKickTick > curTick) {
        DPRINTF(EthernetSM, "transmit kick exiting, can't run till %d\n",
                txKickTick);
        return;
    }

  next:
    switch (txState) {
      case txIdle:
        if (txPioRequest) {
            pioInterface->respond(txPioRequest, curTick + pioLatency);
            txPioRequest = 0;
        }
        goto exit;

      case txFifoBlock:
        if (!txPacket) {
            // Grab a new packet from the fifo.
            txPacket = new PacketData(16384);
            txPacketBufPtr = txPacket->data;
        }

        if (txFifo.avail() - txPacket->length <
            Regs::get_TxData_Len(regs.TxData)) {
            DPRINTF(EthernetSM, "transmit fifo full.  Nothing to do.\n");
            goto exit;
        }

        txState = txBeginCopy;
        break;

      case txBeginCopy:
        txDmaAddr = plat->pciToDma(Regs::get_TxData_Addr(regs.TxData));
        txDmaLen = Regs::get_TxData_Len(regs.TxData);
        txDmaData = txPacketBufPtr;

        if (dmaInterface) {
            if (!dmaInterface->busy()) {
                dmaInterface->doDMA(Read, txDmaAddr, txDmaLen,
                                    curTick, &txDmaEvent, true);
                txState = txCopy;
            }

            goto exit;
        }

        txState = txCopy;
        if (dmaReadDelay != 0 || dmaReadFactor != 0) {
            Tick factor = ((txDmaLen + ULL(63)) >> ULL(6)) * dmaReadFactor;
            Tick start = curTick + dmaReadDelay + factor;
            txDmaEvent.schedule(start);
            goto exit;
        }

        txDmaCopy();
        break;

      case txCopy:
        DPRINTF(EthernetSM, "transmit machine still copying\n");
        goto exit;

      case txCopyDone:
        txPacket->length += txDmaLen;
        if ((regs.TxData & Regs::TxData_More)) {
            txPacketBufPtr += txDmaLen;
        } else {
            assert(txPacket->length <= txFifo.avail());
            if ((regs.TxData & Regs::TxData_Checksum)) {
                IpPtr ip(txPacket);
                if (ip) {
                    TcpPtr tcp(ip);
                    if (tcp) {
                        tcp->sum(0);
                        tcp->sum(cksum(tcp));
                        txTcpChecksums++;
                    }

                    UdpPtr udp(ip);
                    if (udp) {
                        udp->sum(0);
                        udp->sum(cksum(udp));
                        txUdpChecksums++;
                    }

                    ip->sum(0);
                    ip->sum(cksum(ip));
                    txIpChecksums++;
                }
            }
            txFifo.push(txPacket);
            txPacket = 0;
            transmit();
        }

        regs.TxDone = txDmaLen | Regs::TxDone_Complete;
        devIntrPost(Regs::Intr_TxData);
        txState = txIdle;
        break;

      default:
        panic("Invalid txState!");
    }

    DPRINTF(EthernetSM, "entering next txState=%s\n",
            TxStateStrings[txState]);

    goto next;

  exit:
    /**
     * @todo do we want to schedule a future kick?
     */
    DPRINTF(EthernetSM, "tx state machine exited txState=%s\n",
            TxStateStrings[txState]);
}

void
Device::transferDone()
{
    if (txFifo.empty()) {
        DPRINTF(Ethernet, "transfer complete: txFifo empty...nothing to do\n");
        return;
    }

    DPRINTF(Ethernet, "transfer complete: data in txFifo...schedule xmit\n");

    if (txEvent.scheduled())
        txEvent.reschedule(curTick + cycles(1));
    else
        txEvent.schedule(curTick + cycles(1));
}

bool
Device::rxFilter(const PacketPtr &packet)
{
    if (!Regs::get_Config_Filter(regs.Config))
        return false;

    panic("receive filter not implemented\n");
    bool drop = true;

#if 0
    string type;

    EthHdr *eth = packet->eth();
    if (eth->unicast()) {
        // If we're accepting all unicast addresses
        if (acceptUnicast)
            drop = false;

        // If we make a perfect match
        if (acceptPerfect && params->eaddr == eth.dst())
            drop = false;

        if (acceptArp && eth->type() == ETH_TYPE_ARP)
            drop = false;

    } else if (eth->broadcast()) {
        // if we're accepting broadcasts
        if (acceptBroadcast)
            drop = false;

    } else if (eth->multicast()) {
        // if we're accepting all multicasts
        if (acceptMulticast)
            drop = false;

    }

    if (drop) {
        DPRINTF(Ethernet, "rxFilter drop\n");
        DDUMP(EthernetData, packet->data, packet->length);
    }
#endif
    return drop;
}

bool
Device::recvPacket(PacketPtr packet)
{
    rxBytes += packet->length;
    rxPackets++;

    DPRINTF(Ethernet, "Receiving packet from wire, rxFifo Available is %d\n",
            rxFifo.avail());

    if (!rxEnable) {
        DPRINTF(Ethernet, "receive disabled...packet dropped\n");
        interface->recvDone();
        return true;
    }

    if (rxFilter(packet)) {
        DPRINTF(Ethernet, "packet filtered...dropped\n");
        interface->recvDone();
        return true;
    }

    if (rxFifo.size() >= params()->rx_fifo_threshold)
        devIntrPost(Regs::Intr_RxFifo);

    if (!rxFifo.push(packet)) {
        DPRINTF(Ethernet,
                "packet will not fit in receive buffer...packet dropped\n");
        return false;
    }

    interface->recvDone();
    devIntrPost(Regs::Intr_RxDone);
    rxKick();
    return true;
}

//=====================================================================
//
//
void
Base::serialize(ostream &os)
{
    // Serialize the PciDev base class
    PciDev::serialize(os);

    SERIALIZE_SCALAR(rxEnable);
    SERIALIZE_SCALAR(txEnable);
    SERIALIZE_SCALAR(cpuIntrEnable);

    /*
     * Keep track of pending interrupt status.
     */
    SERIALIZE_SCALAR(intrTick);
    SERIALIZE_SCALAR(cpuPendingIntr);
    Tick intrEventTick = 0;
    if (intrEvent)
        intrEventTick = intrEvent->when();
    SERIALIZE_SCALAR(intrEventTick);
}

void
Base::unserialize(Checkpoint *cp, const std::string &section)
{
    // Unserialize the PciDev base class
    PciDev::unserialize(cp, section);

    UNSERIALIZE_SCALAR(rxEnable);
    UNSERIALIZE_SCALAR(txEnable);
    UNSERIALIZE_SCALAR(cpuIntrEnable);

    /*
     * Keep track of pending interrupt status.
     */
    UNSERIALIZE_SCALAR(intrTick);
    UNSERIALIZE_SCALAR(cpuPendingIntr);
    Tick intrEventTick;
    UNSERIALIZE_SCALAR(intrEventTick);
    if (intrEventTick) {
        intrEvent = new IntrEvent(this, true);
        intrEvent->schedule(intrEventTick);
    }
}

void
Device::serialize(ostream &os)
{
    // Serialize the PciDev base class
    Base::serialize(os);

    if (rxDmaEvent.scheduled())
        rxDmaCopy();

    if (txDmaEvent.scheduled())
        txDmaCopy();

    /*
     * Serialize the device registers
     */
    SERIALIZE_SCALAR(regs.Config);
    SERIALIZE_SCALAR(regs.RxMaxCopy);
    SERIALIZE_SCALAR(regs.TxMaxCopy);
    SERIALIZE_SCALAR(regs.RxThreshold);
    SERIALIZE_SCALAR(regs.TxThreshold);
    SERIALIZE_SCALAR(regs.IntrStatus);
    SERIALIZE_SCALAR(regs.IntrMask);
    SERIALIZE_SCALAR(regs.RxData);
    SERIALIZE_SCALAR(regs.RxDone);
    SERIALIZE_SCALAR(regs.TxData);
    SERIALIZE_SCALAR(regs.TxDone);

    /*
     * Serialize rx state machine
     */
    int rxState = this->rxState;
    SERIALIZE_SCALAR(rxState);
    rxFifo.serialize("rxFifo", os);
    bool rxPacketExists = rxPacket;
    SERIALIZE_SCALAR(rxPacketExists);
    if (rxPacketExists) {
        rxPacket->serialize("rxPacket", os);
        uint32_t rxPktBufPtr = (uint32_t) (rxPacketBufPtr - rxPacket->data);
        SERIALIZE_SCALAR(rxPktBufPtr);
        SERIALIZE_SCALAR(rxPktBytes);
    }
    SERIALIZE_SCALAR(rxDoneData);

    /*
     * Serialize tx state machine
     */
    int txState = this->txState;
    SERIALIZE_SCALAR(txState);
    txFifo.serialize("txFifo", os);
    bool txPacketExists = txPacket;
    SERIALIZE_SCALAR(txPacketExists);
    if (txPacketExists) {
        txPacket->serialize("txPacket", os);
        uint32_t txPktBufPtr = (uint32_t) (txPacketBufPtr - txPacket->data);
        SERIALIZE_SCALAR(txPktBufPtr);
        SERIALIZE_SCALAR(txPktBytes);
    }

    /*
     * If there's a pending transmit, store the time so we can
     * reschedule it later
     */
    Tick transmitTick = txEvent.scheduled() ? txEvent.when() - curTick : 0;
    SERIALIZE_SCALAR(transmitTick);
}

void
Device::unserialize(Checkpoint *cp, const std::string &section)
{
    // Unserialize the PciDev base class
    Base::unserialize(cp, section);

    /*
     * Unserialize the device registers
     */
    UNSERIALIZE_SCALAR(regs.Config);
    UNSERIALIZE_SCALAR(regs.RxMaxCopy);
    UNSERIALIZE_SCALAR(regs.TxMaxCopy);
    UNSERIALIZE_SCALAR(regs.RxThreshold);
    UNSERIALIZE_SCALAR(regs.TxThreshold);
    UNSERIALIZE_SCALAR(regs.IntrStatus);
    UNSERIALIZE_SCALAR(regs.IntrMask);
    UNSERIALIZE_SCALAR(regs.RxData);
    UNSERIALIZE_SCALAR(regs.RxDone);
    UNSERIALIZE_SCALAR(regs.TxData);
    UNSERIALIZE_SCALAR(regs.TxDone);

    /*
     * Unserialize rx state machine
     */
    int rxState;
    UNSERIALIZE_SCALAR(rxState);
    this->rxState = (RxState) rxState;
    rxFifo.unserialize("rxFifo", cp, section);
    bool rxPacketExists;
    UNSERIALIZE_SCALAR(rxPacketExists);
    rxPacket = 0;
    if (rxPacketExists) {
        rxPacket = new PacketData(16384);
        rxPacket->unserialize("rxPacket", cp, section);
        uint32_t rxPktBufPtr;
        UNSERIALIZE_SCALAR(rxPktBufPtr);
        this->rxPacketBufPtr = (uint8_t *) rxPacket->data + rxPktBufPtr;
        UNSERIALIZE_SCALAR(rxPktBytes);
    }
    UNSERIALIZE_SCALAR(rxDoneData);

    /*
     * Unserialize tx state machine
     */
    int txState;
    UNSERIALIZE_SCALAR(txState);
    this->txState = (TxState) txState;
    txFifo.unserialize("txFifo", cp, section);
    bool txPacketExists;
    UNSERIALIZE_SCALAR(txPacketExists);
    txPacket = 0;
    if (txPacketExists) {
        txPacket = new PacketData(16384);
        txPacket->unserialize("txPacket", cp, section);
        uint32_t txPktBufPtr;
        UNSERIALIZE_SCALAR(txPktBufPtr);
        this->txPacketBufPtr = (uint8_t *) txPacket->data + txPktBufPtr;
        UNSERIALIZE_SCALAR(txPktBytes);
    }

    /*
     * If there's a pending transmit, reschedule it now
     */
    Tick transmitTick;
    UNSERIALIZE_SCALAR(transmitTick);
    if (transmitTick)
        txEvent.schedule(curTick + transmitTick);

    /*
     * re-add addrRanges to bus bridges
     */
    if (pioInterface)
        pioInterface->addAddrRange(RangeSize(BARAddrs[0], BARSize[0]));
}

Tick
Device::cacheAccess(MemReqPtr &req)
{
    //The mask is to give you only the offset into the device register file
    Addr daddr = req->paddr - addr;

    DPRINTF(EthernetPIO, "timing access to paddr=%#x (daddr=%#x)\n",
            req->paddr, daddr);

    Tick when = curTick + pioLatency;

    switch (daddr) {
      case Regs::RxDone:
        if (rxState != rxIdle) {
            rxPioRequest = req;
            when = 0;
        }
        break;

      case Regs::TxDone:
        if (txState != txIdle) {
            txPioRequest = req;
            when = 0;
        }
        break;
    }

    return when;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Interface)

    SimObjectParam<EtherInt *> peer;
    SimObjectParam<Device *> device;

END_DECLARE_SIM_OBJECT_PARAMS(Interface)

BEGIN_INIT_SIM_OBJECT_PARAMS(Interface)

    INIT_PARAM_DFLT(peer, "peer interface", NULL),
    INIT_PARAM(device, "Ethernet device of this interface")

END_INIT_SIM_OBJECT_PARAMS(Interface)

CREATE_SIM_OBJECT(Interface)
{
    Interface *dev_int = new Interface(getInstanceName(), device);

    EtherInt *p = (EtherInt *)peer;
    if (p) {
        dev_int->setPeer(p);
        p->setPeer(dev_int);
    }

    return dev_int;
}

REGISTER_SIM_OBJECT("SinicInt", Interface)


BEGIN_DECLARE_SIM_OBJECT_PARAMS(Device)

    Param<Addr> addr;
    Param<Tick> cycle_time;
    Param<Tick> tx_delay;
    Param<Tick> rx_delay;
    Param<Tick> intr_delay;
    SimObjectParam<MemoryController *> mmu;
    SimObjectParam<PhysicalMemory *> physmem;
    Param<bool> rx_filter;
    Param<string> hardware_address;
    SimObjectParam<Bus*> io_bus;
    SimObjectParam<Bus*> payload_bus;
    SimObjectParam<HierParams *> hier;
    Param<Tick> pio_latency;
    SimObjectParam<PciConfigAll *> configspace;
    SimObjectParam<PciConfigData *> configdata;
    SimObjectParam<Platform *> platform;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;
    Param<uint32_t> rx_max_copy;
    Param<uint32_t> tx_max_copy;
    Param<uint32_t> rx_fifo_size;
    Param<uint32_t> tx_fifo_size;
    Param<uint32_t> rx_fifo_threshold;
    Param<uint32_t> tx_fifo_threshold;
    Param<Tick> dma_read_delay;
    Param<Tick> dma_read_factor;
    Param<Tick> dma_write_delay;
    Param<Tick> dma_write_factor;
    Param<bool> dma_no_allocate;

END_DECLARE_SIM_OBJECT_PARAMS(Device)

BEGIN_INIT_SIM_OBJECT_PARAMS(Device)

    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(cycle_time, "State machine cycle time"),
    INIT_PARAM_DFLT(tx_delay, "Transmit Delay", 1000),
    INIT_PARAM_DFLT(rx_delay, "Receive Delay", 1000),
    INIT_PARAM_DFLT(intr_delay, "Interrupt Delay in microseconds", 0),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(physmem, "Physical Memory"),
    INIT_PARAM_DFLT(rx_filter, "Enable Receive Filter", true),
    INIT_PARAM_DFLT(hardware_address, "Ethernet Hardware Address",
                    "00:99:00:00:00:01"),
    INIT_PARAM_DFLT(io_bus, "The IO Bus to attach to for headers", NULL),
    INIT_PARAM_DFLT(payload_bus, "The IO Bus to attach to for payload", NULL),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM(configspace, "PCI Configspace"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(platform, "Platform"),
    INIT_PARAM(pci_bus, "PCI bus"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code"),
    INIT_PARAM_DFLT(rx_max_copy, "rx max copy", 16*1024),
    INIT_PARAM_DFLT(tx_max_copy, "rx max copy", 16*1024),
    INIT_PARAM_DFLT(rx_fifo_size, "max size in bytes of rxFifo", 64*1024),
    INIT_PARAM_DFLT(tx_fifo_size, "max size in bytes of txFifo", 64*1024),
    INIT_PARAM_DFLT(rx_fifo_threshold, "max size in bytes of rxFifo", 48*1024),
    INIT_PARAM_DFLT(tx_fifo_threshold, "max size in bytes of txFifo", 16*1024),
    INIT_PARAM_DFLT(dma_read_delay, "fixed delay for dma reads", 0),
    INIT_PARAM_DFLT(dma_read_factor, "multiplier for dma reads", 0),
    INIT_PARAM_DFLT(dma_write_delay, "fixed delay for dma writes", 0),
    INIT_PARAM_DFLT(dma_write_factor, "multiplier for dma writes", 0),
    INIT_PARAM_DFLT(dma_no_allocate, "Should we allocat on read in cache", true)

END_INIT_SIM_OBJECT_PARAMS(Device)


CREATE_SIM_OBJECT(Device)
{
    Device::Params *params = new Device::Params;
    params->name = getInstanceName();
    params->intr_delay = intr_delay;
    params->physmem = physmem;
    params->cycle_time = cycle_time;
    params->tx_delay = tx_delay;
    params->rx_delay = rx_delay;
    params->mmu = mmu;
    params->hier = hier;
    params->io_bus = io_bus;
    params->payload_bus = payload_bus;
    params->pio_latency = pio_latency;
    params->configSpace = configspace;
    params->configData = configdata;
    params->plat = platform;
    params->busNum = pci_bus;
    params->deviceNum = pci_dev;
    params->functionNum = pci_func;
    params->rx_filter = rx_filter;
    params->eaddr = hardware_address;
    params->rx_max_copy = rx_max_copy;
    params->tx_max_copy = tx_max_copy;
    params->rx_fifo_size = rx_fifo_size;
    params->tx_fifo_size = tx_fifo_size;
    params->rx_fifo_threshold = rx_fifo_threshold;
    params->tx_fifo_threshold = tx_fifo_threshold;
    params->dma_read_delay = dma_read_delay;
    params->dma_read_factor = dma_read_factor;
    params->dma_write_delay = dma_write_delay;
    params->dma_write_factor = dma_write_factor;
    params->dma_no_allocate = dma_no_allocate;
    return new Device(params);
}

REGISTER_SIM_OBJECT("Sinic", Device)

/* namespace Sinic */ }

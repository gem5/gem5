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

#include <deque>
#include <limits>
#include <string>

#include "base/inet.hh"
#include "cpu/thread_context.hh"
#include "cpu/intr_control.hh"
#include "dev/etherlink.hh"
#include "dev/sinic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/builder.hh"
#include "sim/debug.hh"
#include "sim/eventq.hh"
#include "sim/host.hh"
#include "sim/stats.hh"
#include "arch/vtophys.hh"

using namespace Net;
using namespace TheISA;

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
    : PciDev(p), rxEnable(false), txEnable(false), clock(p->clock),
      intrDelay(p->intr_delay), intrTick(0), cpuIntrEnable(false),
      cpuPendingIntr(false), intrEvent(0), interface(NULL)
{
}

Device::Device(Params *p)
    : Base(p), rxUnique(0), txUnique(0),
      virtualRegs(p->virtual_count < 1 ? 1 : p->virtual_count),
      rxFifo(p->rx_fifo_size), txFifo(p->tx_fifo_size),
      rxKickTick(0), txKickTick(0),
      txEvent(this), rxDmaEvent(this), txDmaEvent(this),
      dmaReadDelay(p->dma_read_delay), dmaReadFactor(p->dma_read_factor),
      dmaWriteDelay(p->dma_write_delay), dmaWriteFactor(p->dma_write_factor)
{
    reset();

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

void
Device::prepareIO(int cpu, int index)
{
    int size = virtualRegs.size();
    if (index > size)
        panic("Trying to access a vnic that doesn't exist %d > %d\n",
              index, size);
}

void
Device::prepareRead(int cpu, int index)
{
    using namespace Regs;
    prepareIO(cpu, index);

    VirtualReg &vnic = virtualRegs[index];

    // update rx registers
    uint64_t rxdone = vnic.RxDone;
    rxdone = set_RxDone_Packets(rxdone, rxFifo.countPacketsAfter(rxFifoPtr));
    rxdone = set_RxDone_Empty(rxdone, rxFifo.empty());
    rxdone = set_RxDone_High(rxdone, rxFifo.size() > regs.RxFifoMark);
    rxdone = set_RxDone_NotHigh(rxdone, rxLow);
    regs.RxData = vnic.RxData;
    regs.RxDone = rxdone;
    regs.RxWait = rxdone;

    // update tx regsiters
    uint64_t txdone = vnic.TxDone;
    txdone = set_TxDone_Packets(txdone, txFifo.packets());
    txdone = set_TxDone_Full(txdone, txFifo.avail() < regs.TxMaxCopy);
    txdone = set_TxDone_Low(txdone, txFifo.size() < regs.TxFifoMark);
    regs.TxData = vnic.TxData;
    regs.TxDone = txdone;
    regs.TxWait = txdone;
}

void
Device::prepareWrite(int cpu, int index)
{
    prepareIO(cpu, index);
}

/**
 * I/O read of device register
 */
Tick
Device::read(PacketPtr pkt)
{
    assert(config.command & PCI_CMD_MSE);
    assert(pkt->getAddr() >= BARAddrs[0] && pkt->getSize() < BARSize[0]);

    int cpu = pkt->req->getCpuNum();
    Addr daddr = pkt->getAddr() - BARAddrs[0];
    Addr index = daddr >> Regs::VirtualShift;
    Addr raddr = daddr & Regs::VirtualMask;

    pkt->allocate();

    if (!regValid(raddr))
        panic("invalid register: cpu=%d vnic=%d da=%#x pa=%#x size=%d",
              cpu, index, daddr, pkt->getAddr(), pkt->getSize());

    const Regs::Info &info = regInfo(raddr);
    if (!info.read)
        panic("read %s (write only): "
              "cpu=%d vnic=%d da=%#x pa=%#x size=%d",
              info.name, cpu, index, daddr, pkt->getAddr(), pkt->getSize());

        panic("read %s (invalid size): "
              "cpu=%d vnic=%d da=%#x pa=%#x size=%d",
              info.name, cpu, index, daddr, pkt->getAddr(), pkt->getSize());

    prepareRead(cpu, index);

    uint64_t value = 0;
    if (pkt->getSize() == 4) {
        uint32_t reg = regData32(raddr);
        pkt->set(reg);
        value = reg;
    }

    if (pkt->getSize() == 8) {
        uint64_t reg = regData64(raddr);
        pkt->set(reg);
        value = reg;
    }

    DPRINTF(EthernetPIO,
            "read %s: cpu=%d vnic=%d da=%#x pa=%#x size=%d val=%#x\n",
            info.name, cpu, index, daddr, pkt->getAddr(), pkt->getSize(), value);

    // reading the interrupt status register has the side effect of
    // clearing it
    if (raddr == Regs::IntrStatus)
        devIntrClear();

    return pioDelay;
}

/**
 * IPR read of device register

    Fault
Device::iprRead(Addr daddr, int cpu, uint64_t &result)
{
    if (!regValid(daddr))
        panic("invalid address: da=%#x", daddr);

    const Regs::Info &info = regInfo(daddr);
    if (!info.read)
        panic("reading %s (write only): cpu=%d da=%#x", info.name, cpu, daddr);

    DPRINTF(EthernetPIO, "IPR read %s: cpu=%d da=%#x\n",
            info.name, cpu, daddr);

    prepareRead(cpu, 0);

    if (info.size == 4)
        result = regData32(daddr);

    if (info.size == 8)
        result = regData64(daddr);

    DPRINTF(EthernetPIO, "IPR read %s: cpu=%s da=%#x val=%#x\n",
            info.name, cpu, result);

    return NoFault;
}
*/
/**
 * I/O write of device register
 */
Tick
Device::write(PacketPtr pkt)
{
    assert(config.command & PCI_CMD_MSE);
    assert(pkt->getAddr() >= BARAddrs[0] && pkt->getSize() < BARSize[0]);

    int cpu = pkt->req->getCpuNum();
    Addr daddr = pkt->getAddr() - BARAddrs[0];
    Addr index = daddr >> Regs::VirtualShift;
    Addr raddr = daddr & Regs::VirtualMask;

    if (!regValid(raddr))
        panic("invalid register: cpu=%d, da=%#x pa=%#x size=%d",
                cpu, daddr, pkt->getAddr(), pkt->getSize());

    const Regs::Info &info = regInfo(raddr);
    if (!info.write)
        panic("write %s (read only): "
              "cpu=%d vnic=%d da=%#x pa=%#x size=%d",
              info.name, cpu, index, daddr, pkt->getAddr(), pkt->getSize());

    if (pkt->getSize() != info.size)
        panic("write %s (invalid size): "
              "cpu=%d vnic=%d da=%#x pa=%#x size=%d",
              info.name, cpu, index, daddr, pkt->getAddr(), pkt->getSize());

    VirtualReg &vnic = virtualRegs[index];

    DPRINTF(EthernetPIO,
            "write %s vnic %d: cpu=%d val=%#x da=%#x pa=%#x size=%d\n",
            info.name, index, cpu, info.size == 4 ? pkt->get<uint32_t>() :
            pkt->get<uint64_t>(), daddr, pkt->getAddr(), pkt->getSize());

    prepareWrite(cpu, index);

    switch (raddr) {
      case Regs::Config:
        changeConfig(pkt->get<uint32_t>());
        break;

      case Regs::Command:
        command(pkt->get<uint32_t>());
        break;

      case Regs::IntrStatus:
        devIntrClear(regs.IntrStatus & pkt->get<uint32_t>());
        break;

      case Regs::IntrMask:
        devIntrChangeMask(pkt->get<uint32_t>());
        break;

      case Regs::RxData:
        if (Regs::get_RxDone_Busy(vnic.RxDone))
            panic("receive machine busy with another request! rxState=%s",
                  RxStateStrings[rxState]);

        vnic.rxUnique = rxUnique++;
        vnic.RxDone = Regs::RxDone_Busy;
        vnic.RxData = pkt->get<uint64_t>();

        if (Regs::get_RxData_Vaddr(pkt->get<uint64_t>())) {
            panic("vtophys not implemented in newmem");
/*            Addr vaddr = Regs::get_RxData_Addr(reg64);
            Addr paddr = vtophys(req->xc, vaddr);
            DPRINTF(EthernetPIO, "write RxData vnic %d (rxunique %d): "
                    "vaddr=%#x, paddr=%#x\n",
                    index, vnic.rxUnique, vaddr, paddr);

            vnic.RxData = Regs::set_RxData_Addr(vnic.RxData, paddr);*/
        } else {
            DPRINTF(EthernetPIO, "write RxData vnic %d (rxunique %d)\n",
                    index, vnic.rxUnique);
        }

        if (vnic.rxPacket == rxFifo.end()) {
            DPRINTF(EthernetPIO, "request new packet...appending to rxList\n");
            rxList.push_back(index);
        } else {
            DPRINTF(EthernetPIO, "packet exists...appending to rxBusy\n");
            rxBusy.push_back(index);
        }

        if (rxEnable && (rxState == rxIdle || rxState == rxFifoBlock)) {
            rxState = rxFifoBlock;
            rxKick();
        }
        break;

      case Regs::TxData:
        if (Regs::get_TxDone_Busy(vnic.TxDone))
            panic("transmit machine busy with another request! txState=%s",
                  TxStateStrings[txState]);

        vnic.txUnique = txUnique++;
        vnic.TxDone = Regs::TxDone_Busy;

        if (Regs::get_TxData_Vaddr(pkt->get<uint64_t>())) {
            panic("vtophys won't work here in newmem.\n");
            /*Addr vaddr = Regs::get_TxData_Addr(reg64);
            Addr paddr = vtophys(req->xc, vaddr);
            DPRINTF(EthernetPIO, "write TxData vnic %d (rxunique %d): "
                    "vaddr=%#x, paddr=%#x\n",
                    index, vnic.txUnique, vaddr, paddr);

            vnic.TxData = Regs::set_TxData_Addr(vnic.TxData, paddr);*/
        } else {
            DPRINTF(EthernetPIO, "write TxData vnic %d (rxunique %d)\n",
                    index, vnic.txUnique);
        }

        if (txList.empty() || txList.front() != index)
            txList.push_back(index);
        if (txEnable && txState == txIdle && txList.front() == index) {
            txState = txFifoBlock;
            txKick();
        }
        break;
    }

    return pioDelay;
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

    interrupts = regs.IntrStatus & regs.IntrMask;

    // Intr_RxHigh is special, we only signal it if we've emptied the fifo
    // and then filled it above the high watermark
    if (rxEmpty)
        rxEmpty = false;
    else
        interrupts &= ~Regs::Intr_RxHigh;

    // Intr_TxLow is special, we only signal it if we've filled up the fifo
    // and then dropped below the low watermark
    if (txFull)
        txFull = false;
    else
        interrupts &= ~Regs::Intr_TxLow;

    if (interrupts) {
        Tick when = curTick;
        if ((interrupts & Regs::Intr_NoDelay) == 0)
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
Device::command(uint32_t command)
{
    if (command & Regs::Command_Intr)
        devIntrPost(Regs::Intr_Soft);

    if (command & Regs::Command_Reset)
        reset();
}

void
Device::reset()
{
    using namespace Regs;

    memset(&regs, 0, sizeof(regs));

    regs.Config = 0;
    if (params()->rx_thread)
        regs.Config |= Config_RxThread;
    if (params()->tx_thread)
        regs.Config |= Config_TxThread;
    if (params()->rss)
        regs.Config |= Config_RSS;
    if (params()->zero_copy)
        regs.Config |= Config_ZeroCopy;
    if (params()->delay_copy)
        regs.Config |= Config_DelayCopy;
    if (params()->virtual_addr)
        regs.Config |= Config_Vaddr;

    if (params()->delay_copy && params()->zero_copy)
        panic("Can't delay copy and zero copy");

    regs.IntrMask = Intr_Soft | Intr_RxHigh | Intr_RxPacket | Intr_TxLow;
    regs.RxMaxCopy = params()->rx_max_copy;
    regs.TxMaxCopy = params()->tx_max_copy;
    regs.RxMaxIntr = params()->rx_max_intr;
    regs.VirtualCount = params()->virtual_count;
    regs.RxFifoSize = params()->rx_fifo_size;
    regs.TxFifoSize = params()->tx_fifo_size;
    regs.RxFifoMark = params()->rx_fifo_threshold;
    regs.TxFifoMark = params()->tx_fifo_threshold;
    regs.HwAddr = params()->eaddr;

    rxList.clear();
    rxBusy.clear();
    rxActive = -1;
    txList.clear();

    rxState = rxIdle;
    txState = txIdle;

    rxFifo.clear();
    rxFifoPtr = rxFifo.end();
    txFifo.clear();
    rxEmpty = false;
    rxLow = true;
    txFull = false;

    int size = virtualRegs.size();
    virtualRegs.clear();
    virtualRegs.resize(size);
    for (int i = 0; i < size; ++i)
        virtualRegs[i].rxPacket = rxFifo.end();
}

void
Device::rxDmaDone()
{
    assert(rxState == rxCopy);
    rxState = rxCopyDone;
    DPRINTF(EthernetDMA, "end rx dma write paddr=%#x len=%d\n",
            rxDmaAddr, rxDmaLen);
    DDUMP(EthernetData, rxDmaData, rxDmaLen);

    // If the transmit state machine  has a pending DMA, let it go first
    if (txState == txBeginCopy)
        txKick();

    rxKick();
}

void
Device::rxKick()
{
    VirtualReg *vnic = NULL;

    DPRINTF(EthernetSM, "rxKick: rxState=%s (rxFifo.size=%d)\n",
            RxStateStrings[rxState], rxFifo.size());

    if (rxKickTick > curTick) {
        DPRINTF(EthernetSM, "rxKick: exiting, can't run till %d\n",
                rxKickTick);
        return;
    }

  next:
    if (rxState == rxIdle)
        goto exit;

    if (rxActive == -1) {
        if (rxState != rxFifoBlock)
            panic("no active vnic while in state %s", RxStateStrings[rxState]);

        DPRINTF(EthernetSM, "processing rxState=%s\n",
                RxStateStrings[rxState]);
    } else {
        vnic = &virtualRegs[rxActive];
        DPRINTF(EthernetSM,
                "processing rxState=%s for vnic %d (rxunique %d)\n",
                RxStateStrings[rxState], rxActive, vnic->rxUnique);
    }

    switch (rxState) {
      case rxFifoBlock:
        if (DTRACE(EthernetSM)) {
            PacketFifo::iterator end = rxFifo.end();
            int size = virtualRegs.size();
            for (int i = 0; i < size; ++i) {
                VirtualReg *vn = &virtualRegs[i];
                if (vn->rxPacket != end &&
                    !Regs::get_RxDone_Busy(vn->RxDone)) {
                    DPRINTF(EthernetSM,
                            "vnic %d (rxunique %d), has outstanding packet %d\n",
                            i, vn->rxUnique,
                            rxFifo.countPacketsBefore(vn->rxPacket));
                }
            }
        }

        if (!rxBusy.empty()) {
            rxActive = rxBusy.front();
            rxBusy.pop_front();
            vnic = &virtualRegs[rxActive];

            if (vnic->rxPacket == rxFifo.end())
                panic("continuing vnic without packet\n");

            DPRINTF(EthernetSM,
                    "continue processing for vnic %d (rxunique %d)\n",
                    rxActive, vnic->rxUnique);

            rxState = rxBeginCopy;

            break;
        }

        if (rxFifoPtr == rxFifo.end()) {
            DPRINTF(EthernetSM, "receive waiting for data.  Nothing to do.\n");
            goto exit;
        }

        if (rxList.empty())
            panic("Not idle, but nothing to do!");

        assert(!rxFifo.empty());

        rxActive = rxList.front();
        rxList.pop_front();
        vnic = &virtualRegs[rxActive];

        DPRINTF(EthernetSM,
                "processing new packet for vnic %d (rxunique %d)\n",
                rxActive, vnic->rxUnique);

        // Grab a new packet from the fifo.
        vnic->rxPacket = rxFifoPtr++;
        vnic->rxPacketOffset = 0;
        vnic->rxPacketBytes = (*vnic->rxPacket)->length;
        assert(vnic->rxPacketBytes);

        vnic->rxDoneData = 0;
        /* scope for variables */ {
            IpPtr ip(*vnic->rxPacket);
            if (ip) {
                DPRINTF(Ethernet, "ID is %d\n", ip->id());
                vnic->rxDoneData |= Regs::RxDone_IpPacket;
                rxIpChecksums++;
                if (cksum(ip) != 0) {
                    DPRINTF(EthernetCksum, "Rx IP Checksum Error\n");
                    vnic->rxDoneData |= Regs::RxDone_IpError;
                }
                TcpPtr tcp(ip);
                UdpPtr udp(ip);
                if (tcp) {
                    DPRINTF(Ethernet,
                            "Src Port=%d, Dest Port=%d, Seq=%d, Ack=%d\n",
                            tcp->sport(), tcp->dport(), tcp->seq(),
                            tcp->ack());
                    vnic->rxDoneData |= Regs::RxDone_TcpPacket;
                    rxTcpChecksums++;
                    if (cksum(tcp) != 0) {
                        DPRINTF(EthernetCksum, "Rx TCP Checksum Error\n");
                        vnic->rxDoneData |= Regs::RxDone_TcpError;
                    }
                } else if (udp) {
                    vnic->rxDoneData |= Regs::RxDone_UdpPacket;
                    rxUdpChecksums++;
                    if (cksum(udp) != 0) {
                        DPRINTF(EthernetCksum, "Rx UDP Checksum Error\n");
                        vnic->rxDoneData |= Regs::RxDone_UdpError;
                    }
                }
            }
        }
        rxState = rxBeginCopy;
        break;

      case rxBeginCopy:
        if (dmaPending() || getState() != Running)
            goto exit;

        rxDmaAddr = params()->platform->pciToDma(
                Regs::get_RxData_Addr(vnic->RxData));
        rxDmaLen = std::min<int>(Regs::get_RxData_Len(vnic->RxData),
                            vnic->rxPacketBytes);
        rxDmaData = (*vnic->rxPacket)->data + vnic->rxPacketOffset;
        rxState = rxCopy;
        if (rxDmaAddr == 1LL) {
            rxState = rxCopyDone;
            break;
        }


        dmaWrite(rxDmaAddr, rxDmaLen, &rxDmaEvent, rxDmaData);
        break;

      case rxCopy:
        DPRINTF(EthernetSM, "receive machine still copying\n");
        goto exit;

      case rxCopyDone:
        vnic->RxDone = vnic->rxDoneData;
        vnic->RxDone |= Regs::RxDone_Complete;

        if (vnic->rxPacketBytes == rxDmaLen) {
            // Packet is complete.  Indicate how many bytes were copied
            vnic->RxDone = Regs::set_RxDone_CopyLen(vnic->RxDone, rxDmaLen);

            DPRINTF(EthernetSM,
                    "rxKick: packet complete on vnic %d (rxunique %d)\n",
                    rxActive, vnic->rxUnique);
            rxFifo.remove(vnic->rxPacket);
            vnic->rxPacket = rxFifo.end();
        } else {
            vnic->rxPacketBytes -= rxDmaLen;
            vnic->rxPacketOffset += rxDmaLen;
            vnic->RxDone |= Regs::RxDone_More;
            vnic->RxDone = Regs::set_RxDone_CopyLen(vnic->RxDone,
                                                    vnic->rxPacketBytes);
            DPRINTF(EthernetSM,
                    "rxKick: packet not complete on vnic %d (rxunique %d): "
                    "%d bytes left\n",
                    rxActive, vnic->rxUnique, vnic->rxPacketBytes);
        }

        rxActive = -1;
        rxState = rxBusy.empty() && rxList.empty() ? rxIdle : rxFifoBlock;

        if (rxFifo.empty()) {
            devIntrPost(Regs::Intr_RxEmpty);
            rxEmpty = true;
        }

        if (rxFifo.size() < params()->rx_fifo_low_mark)
            rxLow = true;

        if (rxFifo.size() > params()->rx_fifo_threshold)
            rxLow = false;

        devIntrPost(Regs::Intr_RxDMA);
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
Device::txDmaDone()
{
    assert(txState == txCopy);
    txState = txCopyDone;
    DPRINTF(EthernetDMA, "tx dma read paddr=%#x len=%d\n",
            txDmaAddr, txDmaLen);
    DDUMP(EthernetData, txDmaData, txDmaLen);

    // If the receive state machine  has a pending DMA, let it go first
    if (rxState == rxBeginCopy)
        rxKick();

    txKick();
}

void
Device::transmit()
{
    if (txFifo.empty()) {
        DPRINTF(Ethernet, "nothing to transmit\n");
        return;
    }

    uint32_t interrupts;
    EthPacketPtr packet = txFifo.front();
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
                DPRINTF(Ethernet,
                        "Src Port=%d, Dest Port=%d, Seq=%d, Ack=%d\n",
                        tcp->sport(), tcp->dport(), tcp->seq(),
                        tcp->ack());
            }
        }
    }
#endif

    DDUMP(EthernetData, packet->data, packet->length);
    txBytes += packet->length;
    txPackets++;

    DPRINTF(Ethernet, "Packet Transmit: successful txFifo Available %d\n",
            txFifo.avail());

    interrupts = Regs::Intr_TxPacket;
    if (txFifo.size() < regs.TxFifoMark)
        interrupts |= Regs::Intr_TxLow;
    devIntrPost(interrupts);

  reschedule:
   if (!txFifo.empty() && !txEvent.scheduled()) {
       DPRINTF(Ethernet, "reschedule transmit\n");
       txEvent.schedule(curTick + retryTime);
   }
}

void
Device::txKick()
{
    VirtualReg *vnic;
    DPRINTF(EthernetSM, "txKick: txState=%s (txFifo.size=%d)\n",
            TxStateStrings[txState], txFifo.size());

    if (txKickTick > curTick) {
        DPRINTF(EthernetSM, "txKick: exiting, can't run till %d\n",
                txKickTick);
        return;
    }

  next:
    if (txState == txIdle)
        goto exit;

    assert(!txList.empty());
    vnic = &virtualRegs[txList.front()];

    switch (txState) {
      case txFifoBlock:
        assert(Regs::get_TxDone_Busy(vnic->TxDone));
        if (!txPacket) {
            // Grab a new packet from the fifo.
            txPacket = new EthPacketData(16384);
            txPacketOffset = 0;
        }

        if (txFifo.avail() - txPacket->length <
            Regs::get_TxData_Len(vnic->TxData)) {
            DPRINTF(EthernetSM, "transmit fifo full.  Nothing to do.\n");
            goto exit;
        }

        txState = txBeginCopy;
        break;

      case txBeginCopy:
        if (dmaPending() || getState() != Running)
            goto exit;

        txDmaAddr = params()->platform->pciToDma(
                Regs::get_TxData_Addr(vnic->TxData));
        txDmaLen = Regs::get_TxData_Len(vnic->TxData);
        txDmaData = txPacket->data + txPacketOffset;
        txState = txCopy;

        dmaRead(txDmaAddr, txDmaLen, &txDmaEvent, txDmaData);
        break;

      case txCopy:
        DPRINTF(EthernetSM, "transmit machine still copying\n");
        goto exit;

      case txCopyDone:
        vnic->TxDone = txDmaLen | Regs::TxDone_Complete;
        txPacket->length += txDmaLen;
        if ((vnic->TxData & Regs::TxData_More)) {
            txPacketOffset += txDmaLen;
            txState = txIdle;
            devIntrPost(Regs::Intr_TxDMA);
            break;
        }

        assert(txPacket->length <= txFifo.avail());
        if ((vnic->TxData & Regs::TxData_Checksum)) {
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
        if (txFifo.avail() < regs.TxMaxCopy) {
            devIntrPost(Regs::Intr_TxFull);
            txFull = true;
        }
        txPacket = 0;
        transmit();
        txList.pop_front();
        txState = txList.empty() ? txIdle : txFifoBlock;
        devIntrPost(Regs::Intr_TxDMA);
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
Device::rxFilter(const EthPacketPtr &packet)
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
Device::recvPacket(EthPacketPtr packet)
{
    rxBytes += packet->length;
    rxPackets++;

    DPRINTF(Ethernet, "Receiving packet from wire, rxFifo Available is %d\n",
            rxFifo.avail());

    if (!rxEnable) {
        DPRINTF(Ethernet, "receive disabled...packet dropped\n");
        return true;
    }

    if (rxFilter(packet)) {
        DPRINTF(Ethernet, "packet filtered...dropped\n");
        return true;
    }

    if (rxFifo.size() >= regs.RxFifoMark)
        devIntrPost(Regs::Intr_RxHigh);

    if (!rxFifo.push(packet)) {
        DPRINTF(Ethernet,
                "packet will not fit in receive buffer...packet dropped\n");
        return false;
    }

    // If we were at the last element, back up one ot go to the new
    // last element of the list.
    if (rxFifoPtr == rxFifo.end())
        --rxFifoPtr;

    devIntrPost(Regs::Intr_RxPacket);
    rxKick();
    return true;
}

void
Device::resume()
{
    SimObject::resume();

    // During drain we could have left the state machines in a waiting state and
    // they wouldn't get out until some other event occured to kick them.
    // This way they'll get out immediately
    txKick();
    rxKick();
}

//=====================================================================
//
//
void
Base::serialize(std::ostream &os)
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
Device::serialize(std::ostream &os)
{
    int count;

    // Serialize the PciDev base class
    Base::serialize(os);

    if (rxState == rxCopy)
        panic("can't serialize with an in flight dma request rxState=%s",
              RxStateStrings[rxState]);

    if (txState == txCopy)
        panic("can't serialize with an in flight dma request txState=%s",
              TxStateStrings[txState]);

    /*
     * Serialize the device registers
     */
    SERIALIZE_SCALAR(regs.Config);
    SERIALIZE_SCALAR(regs.IntrStatus);
    SERIALIZE_SCALAR(regs.IntrMask);
    SERIALIZE_SCALAR(regs.RxMaxCopy);
    SERIALIZE_SCALAR(regs.TxMaxCopy);
    SERIALIZE_SCALAR(regs.RxMaxIntr);
    SERIALIZE_SCALAR(regs.VirtualCount);
    SERIALIZE_SCALAR(regs.RxData);
    SERIALIZE_SCALAR(regs.RxDone);
    SERIALIZE_SCALAR(regs.TxData);
    SERIALIZE_SCALAR(regs.TxDone);

    /*
     * Serialize the virtual nic state
     */
    int virtualRegsSize = virtualRegs.size();
    SERIALIZE_SCALAR(virtualRegsSize);
    for (int i = 0; i < virtualRegsSize; ++i) {
        VirtualReg *vnic = &virtualRegs[i];

        std::string reg = csprintf("vnic%d", i);
        paramOut(os, reg + ".RxData", vnic->RxData);
        paramOut(os, reg + ".RxDone", vnic->RxDone);
        paramOut(os, reg + ".TxData", vnic->TxData);
        paramOut(os, reg + ".TxDone", vnic->TxDone);

        bool rxPacketExists = vnic->rxPacket != rxFifo.end();
        paramOut(os, reg + ".rxPacketExists", rxPacketExists);
        if (rxPacketExists) {
            int rxPacket = 0;
            PacketFifo::iterator i = rxFifo.begin();
            while (i != vnic->rxPacket) {
                assert(i != rxFifo.end());
                ++i;
                ++rxPacket;
            }

            paramOut(os, reg + ".rxPacket", rxPacket);
            paramOut(os, reg + ".rxPacketOffset", vnic->rxPacketOffset);
            paramOut(os, reg + ".rxPacketBytes", vnic->rxPacketBytes);
        }
        paramOut(os, reg + ".rxDoneData", vnic->rxDoneData);
    }

    int rxFifoPtr = rxFifo.countPacketsBefore(this->rxFifoPtr);
    SERIALIZE_SCALAR(rxFifoPtr);

    SERIALIZE_SCALAR(rxActive);

    VirtualList::iterator i, end;
    for (count = 0, i = rxList.begin(), end = rxList.end(); i != end; ++i)
        paramOut(os, csprintf("rxList%d", count++), *i);
    int rxListSize = count;
    SERIALIZE_SCALAR(rxListSize);

    for (count = 0, i = rxBusy.begin(), end = rxBusy.end(); i != end; ++i)
        paramOut(os, csprintf("rxBusy%d", count++), *i);
    int rxBusySize = count;
    SERIALIZE_SCALAR(rxBusySize);

    for (count = 0, i = txList.begin(), end = txList.end(); i != end; ++i)
        paramOut(os, csprintf("txList%d", count++), *i);
    int txListSize = count;
    SERIALIZE_SCALAR(txListSize);

    /*
     * Serialize rx state machine
     */
    int rxState = this->rxState;
    SERIALIZE_SCALAR(rxState);
    SERIALIZE_SCALAR(rxEmpty);
    SERIALIZE_SCALAR(rxLow);
    rxFifo.serialize("rxFifo", os);

    /*
     * Serialize tx state machine
     */
    int txState = this->txState;
    SERIALIZE_SCALAR(txState);
    SERIALIZE_SCALAR(txFull);
    txFifo.serialize("txFifo", os);
    bool txPacketExists = txPacket;
    SERIALIZE_SCALAR(txPacketExists);
    if (txPacketExists) {
        txPacket->serialize("txPacket", os);
        SERIALIZE_SCALAR(txPacketOffset);
        SERIALIZE_SCALAR(txPacketBytes);
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
    UNSERIALIZE_SCALAR(regs.IntrStatus);
    UNSERIALIZE_SCALAR(regs.IntrMask);
    UNSERIALIZE_SCALAR(regs.RxMaxCopy);
    UNSERIALIZE_SCALAR(regs.TxMaxCopy);
    UNSERIALIZE_SCALAR(regs.RxMaxIntr);
    UNSERIALIZE_SCALAR(regs.VirtualCount);
    UNSERIALIZE_SCALAR(regs.RxData);
    UNSERIALIZE_SCALAR(regs.RxDone);
    UNSERIALIZE_SCALAR(regs.TxData);
    UNSERIALIZE_SCALAR(regs.TxDone);

    UNSERIALIZE_SCALAR(rxActive);

    int rxListSize;
    UNSERIALIZE_SCALAR(rxListSize);
    rxList.clear();
    for (int i = 0; i < rxListSize; ++i) {
        int value;
        paramIn(cp, section, csprintf("rxList%d", i), value);
        rxList.push_back(value);
    }

    int rxBusySize;
    UNSERIALIZE_SCALAR(rxBusySize);
    rxBusy.clear();
    for (int i = 0; i < rxBusySize; ++i) {
        int value;
        paramIn(cp, section, csprintf("rxBusy%d", i), value);
        rxBusy.push_back(value);
    }

    int txListSize;
    UNSERIALIZE_SCALAR(txListSize);
    txList.clear();
    for (int i = 0; i < txListSize; ++i) {
        int value;
        paramIn(cp, section, csprintf("txList%d", i), value);
        txList.push_back(value);
    }

    /*
     * Unserialize rx state machine
     */
    int rxState;
    UNSERIALIZE_SCALAR(rxState);
    UNSERIALIZE_SCALAR(rxEmpty);
    UNSERIALIZE_SCALAR(rxLow);
    this->rxState = (RxState) rxState;
    rxFifo.unserialize("rxFifo", cp, section);

    int rxFifoPtr;
    UNSERIALIZE_SCALAR(rxFifoPtr);
    this->rxFifoPtr = rxFifo.begin();
    for (int i = 0; i < rxFifoPtr; ++i)
        ++this->rxFifoPtr;

    /*
     * Unserialize tx state machine
     */
    int txState;
    UNSERIALIZE_SCALAR(txState);
    UNSERIALIZE_SCALAR(txFull);
    this->txState = (TxState) txState;
    txFifo.unserialize("txFifo", cp, section);
    bool txPacketExists;
    UNSERIALIZE_SCALAR(txPacketExists);
    txPacket = 0;
    if (txPacketExists) {
        txPacket = new EthPacketData(16384);
        txPacket->unserialize("txPacket", cp, section);
        UNSERIALIZE_SCALAR(txPacketOffset);
        UNSERIALIZE_SCALAR(txPacketBytes);
    }

    /*
     * unserialize the virtual nic registers/state
     *
     * this must be done after the unserialization of the rxFifo
     * because the packet iterators depend on the fifo being populated
     */
    int virtualRegsSize;
    UNSERIALIZE_SCALAR(virtualRegsSize);
    virtualRegs.clear();
    virtualRegs.resize(virtualRegsSize);
    for (int i = 0; i < virtualRegsSize; ++i) {
        VirtualReg *vnic = &virtualRegs[i];
        std::string reg = csprintf("vnic%d", i);

        paramIn(cp, section, reg + ".RxData", vnic->RxData);
        paramIn(cp, section, reg + ".RxDone", vnic->RxDone);
        paramIn(cp, section, reg + ".TxData", vnic->TxData);
        paramIn(cp, section, reg + ".TxDone", vnic->TxDone);

        vnic->rxUnique = rxUnique++;
        vnic->txUnique = txUnique++;

        bool rxPacketExists;
        paramIn(cp, section, reg + ".rxPacketExists", rxPacketExists);
        if (rxPacketExists) {
            int rxPacket;
            paramIn(cp, section, reg + ".rxPacket", rxPacket);
            vnic->rxPacket = rxFifo.begin();
            while (rxPacket--)
                ++vnic->rxPacket;

            paramIn(cp, section, reg + ".rxPacketOffset",
                    vnic->rxPacketOffset);
            paramIn(cp, section, reg + ".rxPacketBytes", vnic->rxPacketBytes);
        } else {
            vnic->rxPacket = rxFifo.end();
        }
        paramIn(cp, section, reg + ".rxDoneData", vnic->rxDoneData);
    }

    /*
     * If there's a pending transmit, reschedule it now
     */
    Tick transmitTick;
    UNSERIALIZE_SCALAR(transmitTick);
    if (transmitTick)
        txEvent.schedule(curTick + transmitTick);

    pioPort->sendStatusChange(Port::RangeChange);

}

/* namespace Sinic */ }

BEGIN_DECLARE_SIM_OBJECT_PARAMS_WNS(Sinic, SinicInterface)

    SimObjectParam<EtherInt *> peer;
    SimObjectParam<Sinic::Device *> device;
END_DECLARE_SIM_OBJECT_PARAMS_WNS(Sinic, SinicInterface)

BEGIN_INIT_SIM_OBJECT_PARAMS_WNS(Sinic, SinicInterface)

    INIT_PARAM_DFLT(peer, "peer interface", NULL),
    INIT_PARAM(device, "Ethernet device of this interface")

END_INIT_SIM_OBJECT_PARAMS_WNS(Sinic, SinicInterface)

CREATE_SIM_OBJECT_WNS(Sinic, SinicInterface)
{
    Sinic::Interface *dev_int = new Sinic::Interface(getInstanceName(), device);

    EtherInt *p = (EtherInt *)peer;
    if (p) {
        dev_int->setPeer(p);
        p->setPeer(dev_int);
    }

    return dev_int;
}

REGISTER_SIM_OBJECT_WNS(Sinic, "SinicInt", SinicInterface)


BEGIN_DECLARE_SIM_OBJECT_PARAMS_WNS(Sinic, SinicDevice)


    SimObjectParam<System *> system;
    SimObjectParam<Platform *> platform;
    Param<Tick> min_backoff_delay;
    Param<Tick> max_backoff_delay;
    SimObjectParam<PciConfigData *> configdata;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;
    Param<Tick> pio_latency;
    Param<Tick> config_latency;
    Param<Tick> intr_delay;

    Param<Tick> clock;
    Param<Tick> dma_read_delay;
    Param<Tick> dma_read_factor;
    Param<Tick> dma_write_delay;
    Param<Tick> dma_write_factor;

    Param<Tick> rx_delay;
    Param<Tick> tx_delay;
    Param<uint32_t> rx_max_copy;
    Param<uint32_t> tx_max_copy;
    Param<uint32_t> rx_max_intr;
    Param<uint32_t> rx_fifo_size;
    Param<uint32_t> tx_fifo_size;
    Param<uint32_t> rx_fifo_threshold;
    Param<uint32_t> rx_fifo_low_mark;
    Param<uint32_t> tx_fifo_high_mark;
    Param<uint32_t> tx_fifo_threshold;

    Param<bool> rx_filter;
    Param<std::string> hardware_address;
    Param<bool> rx_thread;
    Param<bool> tx_thread;
    Param<bool> rss;
    Param<uint32_t> virtual_count;
    Param<bool> zero_copy;
    Param<bool> delay_copy;
    Param<bool> virtual_addr;

END_DECLARE_SIM_OBJECT_PARAMS_WNS(Sinic, SinicDevice)

BEGIN_INIT_SIM_OBJECT_PARAMS_WNS(Sinic, SinicDevice)


    INIT_PARAM(system, "System pointer"),
    INIT_PARAM(platform, "Platform pointer"),
    INIT_PARAM(min_backoff_delay, "Minimum delay after receving a nack packed"),
    INIT_PARAM(max_backoff_delay, "Maximum delay after receving a nack packed"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(pci_bus, "PCI bus ID"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code"),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM(config_latency, "Number of cycles for a config read or write"),
    INIT_PARAM(intr_delay, "Interrupt Delay"),
    INIT_PARAM(clock, "State machine cycle time"),

    INIT_PARAM(dma_read_delay, "fixed delay for dma reads"),
    INIT_PARAM(dma_read_factor, "multiplier for dma reads"),
    INIT_PARAM(dma_write_delay, "fixed delay for dma writes"),
    INIT_PARAM(dma_write_factor, "multiplier for dma writes"),

    INIT_PARAM(rx_delay, "Receive Delay"),
    INIT_PARAM(tx_delay, "Transmit Delay"),
    INIT_PARAM(rx_max_copy, "rx max copy"),
    INIT_PARAM(tx_max_copy, "rx max copy"),
    INIT_PARAM(rx_max_intr, "rx max intr"),
    INIT_PARAM(rx_fifo_size, "max size in bytes of rxFifo"),
    INIT_PARAM(tx_fifo_size, "max size in bytes of txFifo"),
    INIT_PARAM(rx_fifo_threshold, "max size in bytes of rxFifo"),
    INIT_PARAM(rx_fifo_low_mark, "max size in bytes of rxFifo"),
    INIT_PARAM(tx_fifo_high_mark, "max size in bytes of txFifo"),
    INIT_PARAM(tx_fifo_threshold, "max size in bytes of txFifo"),

    INIT_PARAM(rx_filter, "Enable Receive Filter"),
    INIT_PARAM(hardware_address, "Ethernet Hardware Address"),
    INIT_PARAM(rx_thread, ""),
    INIT_PARAM(tx_thread, ""),
    INIT_PARAM(rss, ""),
    INIT_PARAM(virtual_count, ""),
    INIT_PARAM(zero_copy, ""),
    INIT_PARAM(delay_copy, ""),
    INIT_PARAM(virtual_addr, "")

END_INIT_SIM_OBJECT_PARAMS_WNS(Sinic, SinicDevice)


CREATE_SIM_OBJECT_WNS(Sinic, SinicDevice)
{
    Sinic::Sinic::Device::Params *params = new Device::Params;
    params->name = getInstanceName();
    params->platform = platform;
    params->system = system;
    params->min_backoff_delay = min_backoff_delay;
    params->max_backoff_delay = max_backoff_delay;
    params->configData = configdata;
    params->busNum = pci_bus;
    params->deviceNum = pci_dev;
    params->functionNum = pci_func;
    params->pio_delay = pio_latency;
    params->config_delay = config_latency;
    params->intr_delay = intr_delay;
    params->clock = clock;

    params->dma_read_delay = dma_read_delay;
    params->dma_read_factor = dma_read_factor;
    params->dma_write_delay = dma_write_delay;
    params->dma_write_factor = dma_write_factor;

    params->tx_delay = tx_delay;
    params->rx_delay = rx_delay;
    params->rx_max_copy = rx_max_copy;
    params->tx_max_copy = tx_max_copy;
    params->rx_max_intr = rx_max_intr;
    params->rx_fifo_size = rx_fifo_size;
    params->tx_fifo_size = tx_fifo_size;
    params->rx_fifo_threshold = rx_fifo_threshold;
    params->rx_fifo_low_mark = rx_fifo_low_mark;
    params->tx_fifo_high_mark = tx_fifo_high_mark;
    params->tx_fifo_threshold = tx_fifo_threshold;

    params->rx_filter = rx_filter;
    params->eaddr = hardware_address;
    params->rx_thread = rx_thread;
    params->tx_thread = tx_thread;
    params->rss = rss;
    params->virtual_count = virtual_count;
    params->zero_copy = zero_copy;
    params->delay_copy = delay_copy;
    params->virtual_addr = virtual_addr;

    return new Sinic::Device(params);
}

REGISTER_SIM_OBJECT_WNS(Sinic, "Sinic", SinicDevice)


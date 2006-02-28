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
    : Base(p), plat(p->plat), physmem(p->physmem),
      rxFifo(p->rx_fifo_size), txFifo(p->tx_fifo_size),
      rxKickTick(0), txKickTick(0),
      txEvent(this), rxDmaEvent(this), txDmaEvent(this),
      dmaReadDelay(p->dma_read_delay), dmaReadFactor(p->dma_read_factor),
      dmaWriteDelay(p->dma_write_delay), dmaWriteFactor(p->dma_write_factor)
{
    reset();

    if (p->pio_bus) {
        pioInterface = newPioInterface(p->name + ".pio", p->hier, p->pio_bus,
                                       this, &Device::cacheAccess);
        pioLatency = p->pio_latency * p->pio_bus->clockRate;
    }

    if (p->header_bus) {
        if (p->payload_bus)
            dmaInterface = new DMAInterface<Bus>(p->name + ".dma",
                                                 p->header_bus,
                                                 p->payload_bus, 1,
                                                 p->dma_no_allocate);
        else
            dmaInterface = new DMAInterface<Bus>(p->name + ".dma",
                                                 p->header_bus,
                                                 p->header_bus, 1,
                                                 p->dma_no_allocate);
    } else if (p->payload_bus)
        panic("must define a header bus if defining a payload bus");
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
Device::writeConfig(int offset, int size, const uint8_t *data)
{
    switch (offset) {
      case PCI0_BASE_ADDR0:
        // Need to catch writes to BARs to update the PIO interface
        PciDev::writeConfig(offset, size, data);
        if (BARAddrs[0] != 0) {
            if (pioInterface)
                pioInterface->addAddrRange(RangeSize(BARAddrs[0], BARSize[0]));

            BARAddrs[0] &= EV5::PAddrUncachedMask;
        }
        break;

      default:
        PciDev::writeConfig(offset, size, data);
    }
}

void
Device::prepareIO(int cpu, int index)
{
    int size = virtualRegs.size();
    if (index < size)
        return;

    virtualRegs.resize(index + 1);
    for (int i = size; i <= index; ++i)
        virtualRegs[i].rxPacket = rxFifo.end();
}

void
Device::prepareRead(int cpu, int index)
{
    using namespace Regs;
    prepareIO(cpu, index);

    VirtualReg &vnic = virtualRegs[index];

    // update rx registers
    uint64_t rxdone = vnic.RxDone;
    rxdone = set_RxDone_Packets(rxdone, rxFifo.packets());
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
Fault
Device::read(MemReqPtr &req, uint8_t *data)
{
    assert(config.command & PCI_CMD_MSE);
    Fault fault = readBar(req, data);

    if (fault->isMachineCheckFault()) {
        panic("address does not map to a BAR pa=%#x va=%#x size=%d",
              req->paddr, req->vaddr, req->size);

        return genMachineCheckFault();
    }

    return fault;
}

Fault
Device::readBar0(MemReqPtr &req, Addr daddr, uint8_t *data)
{
    int cpu = (req->xc->regs.ipr[TheISA::IPR_PALtemp16] >> 8) & 0xff;
    Addr index = daddr >> Regs::VirtualShift;
    Addr raddr = daddr & Regs::VirtualMask;

    if (!regValid(raddr))
        panic("invalid register: cpu=%d, da=%#x pa=%#x va=%#x size=%d",
              cpu, daddr, req->paddr, req->vaddr, req->size);

    const Regs::Info &info = regInfo(raddr);
    if (!info.read)
        panic("reading %s (write only): cpu=%d da=%#x pa=%#x va=%#x size=%d",
              info.name, cpu, daddr, req->paddr, req->vaddr, req->size);

    if (req->size != info.size)
        panic("invalid size for reg %s: cpu=%d da=%#x pa=%#x va=%#x size=%d",
              info.name, cpu, daddr, req->paddr, req->vaddr, req->size);

    prepareRead(cpu, index);

    uint64_t value = 0;
    if (req->size == 4) {
        uint32_t &reg = *(uint32_t *)data;
        reg = regData32(raddr);
        value = reg;
    }

    if (req->size == 8) {
        uint64_t &reg = *(uint64_t *)data;
        reg = regData64(raddr);
        value = reg;
    }

    DPRINTF(EthernetPIO,
            "read %s cpu=%d da=%#x pa=%#x va=%#x size=%d val=%#x\n",
            info.name, cpu, daddr, req->paddr, req->vaddr, req->size, value);

    // reading the interrupt status register has the side effect of
    // clearing it
    if (raddr == Regs::IntrStatus)
        devIntrClear();

    return NoFault;
}

/**
 * IPR read of device register
 */
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

/**
 * I/O write of device register
 */
Fault
Device::write(MemReqPtr &req, const uint8_t *data)
{
    assert(config.command & PCI_CMD_MSE);
    Fault fault = writeBar(req, data);

    if (fault->isMachineCheckFault()) {
        panic("address does not map to a BAR pa=%#x va=%#x size=%d",
              req->paddr, req->vaddr, req->size);

        return genMachineCheckFault();
    }

    return fault;
}

Fault
Device::writeBar0(MemReqPtr &req, Addr daddr, const uint8_t *data)
{
    int cpu = (req->xc->regs.ipr[TheISA::IPR_PALtemp16] >> 8) & 0xff;
    Addr index = daddr >> Regs::VirtualShift;
    Addr raddr = daddr & Regs::VirtualMask;

    if (!regValid(raddr))
        panic("invalid address: cpu=%d da=%#x pa=%#x va=%#x size=%d",
              cpu, daddr, req->paddr, req->vaddr, req->size);

    const Regs::Info &info = regInfo(raddr);
    if (!info.write)
        panic("writing %s (read only): cpu=%d da=%#x",
              info.name, cpu, daddr);

    if (req->size != info.size)
        panic("invalid size for %s: cpu=%d da=%#x pa=%#x va=%#x size=%d",
              info.name, cpu, daddr, req->paddr, req->vaddr, req->size);

    uint32_t reg32 = *(uint32_t *)data;
    uint64_t reg64 = *(uint64_t *)data;
    VirtualReg &vnic = virtualRegs[index];

    DPRINTF(EthernetPIO,
            "write %s: cpu=%d val=%#x da=%#x pa=%#x va=%#x size=%d\n",
            info.name, cpu, info.size == 4 ? reg32 : reg64,
            daddr, req->paddr, req->vaddr, req->size);

    prepareWrite(cpu, index);

    switch (raddr) {
      case Regs::Config:
        changeConfig(reg32);
        break;

      case Regs::Command:
        command(reg32);
        break;

      case Regs::IntrStatus:
        devIntrClear(regs.IntrStatus & reg32);
        break;

      case Regs::IntrMask:
        devIntrChangeMask(reg32);
        break;

      case Regs::RxData:
        if (Regs::get_RxDone_Busy(vnic.RxDone))
            panic("receive machine busy with another request! rxState=%s",
                  RxStateStrings[rxState]);

        vnic.RxDone = Regs::RxDone_Busy;
        vnic.RxData = reg64;
        rxList.push_back(index);
        if (rxEnable && rxState == rxIdle) {
            rxState = rxFifoBlock;
            rxKick();
        }
        break;

      case Regs::TxData:
        if (Regs::get_TxDone_Busy(vnic.TxDone))
            panic("transmit machine busy with another request! txState=%s",
                  TxStateStrings[txState]);

        vnic.TxDone = Regs::TxDone_Busy;
        vnic.TxData = reg64;
        if (txList.empty() || txList.front() != index)
            txList.push_back(index);
        if (txEnable && txState == txIdle && txList.front() == index) {
            txState = txFifoBlock;
            txKick();
        }
        break;
    }

    return NoFault;
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
    regs.IntrMask = Intr_Soft | Intr_RxHigh | Intr_RxPacket | Intr_TxLow;
    regs.RxMaxCopy = params()->rx_max_copy;
    regs.TxMaxCopy = params()->tx_max_copy;
    regs.RxMaxIntr = params()->rx_max_intr;
    regs.RxFifoSize = params()->rx_fifo_size;
    regs.TxFifoSize = params()->tx_fifo_size;
    regs.RxFifoMark = params()->rx_fifo_threshold;
    regs.TxFifoMark = params()->tx_fifo_threshold;
    regs.HwAddr = params()->eaddr;

    rxList.clear();
    txList.clear();

    rxState = rxIdle;
    txState = txIdle;

    rxFifo.clear();
    rxFifoPtr = rxFifo.end();
    txFifo.clear();
    rxEmpty = false;
    txFull = false;

    int size = virtualRegs.size();
    virtualRegs.clear();
    virtualRegs.resize(size);
    for (int i = 0; i < size; ++i)
        virtualRegs[i].rxPacket = rxFifo.end();
}

void
Device::rxDmaCopy()
{
    assert(rxState == rxCopy);
    rxState = rxCopyDone;
    physmem->dma_write(rxDmaAddr, (uint8_t *)rxDmaData, rxDmaLen);
    DPRINTF(EthernetDMA, "rx dma write paddr=%#x len=%d\n",
            rxDmaAddr, rxDmaLen);
    DDUMP(EthernetData, rxDmaData, rxDmaLen);
}

void
Device::rxDmaDone()
{
    rxDmaCopy();

    // If the transmit state machine  has a pending DMA, let it go first
    if (txState == txBeginCopy)
        txKick();

    rxKick();
}

void
Device::rxKick()
{
    VirtualReg *vnic;

    DPRINTF(EthernetSM, "receive kick rxState=%s (rxFifo.size=%d)\n",
            RxStateStrings[rxState], rxFifo.size());

    if (rxKickTick > curTick) {
        DPRINTF(EthernetSM, "receive kick exiting, can't run till %d\n",
                rxKickTick);
        return;
    }

  next:
    if (rxState == rxIdle)
        goto exit;

    assert(!rxList.empty());
    vnic = &virtualRegs[rxList.front()];

    DPRINTF(EthernetSM, "processing rxState=%s for virtual nic %d\n",
            RxStateStrings[rxState], rxList.front());

    switch (rxState) {
      case rxFifoBlock:
        if (vnic->rxPacket != rxFifo.end()) {
            rxState = rxBeginCopy;
            break;
        }

        if (rxFifoPtr == rxFifo.end()) {
            DPRINTF(EthernetSM, "receive waiting for data.  Nothing to do.\n");
            goto exit;
        }

        assert(!rxFifo.empty());

        // Grab a new packet from the fifo.
        vnic->rxPacket = rxFifoPtr++;
        vnic->rxPacketOffset = 0;
        vnic->rxPacketBytes = (*vnic->rxPacket)->length;
        assert(vnic->rxPacketBytes);

        vnic->rxDoneData = 0;
        /* scope for variables */ {
            IpPtr ip(*vnic->rxPacket);
            if (ip) {
                vnic->rxDoneData |= Regs::RxDone_IpPacket;
                rxIpChecksums++;
                if (cksum(ip) != 0) {
                    DPRINTF(EthernetCksum, "Rx IP Checksum Error\n");
                    vnic->rxDoneData |= Regs::RxDone_IpError;
                }
                TcpPtr tcp(ip);
                UdpPtr udp(ip);
                if (tcp) {
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
        if (dmaInterface && dmaInterface->busy())
            goto exit;

        rxDmaAddr = plat->pciToDma(Regs::get_RxData_Addr(vnic->RxData));
        rxDmaLen = min<int>(Regs::get_RxData_Len(vnic->RxData),
                            vnic->rxPacketBytes);
        rxDmaData = (*vnic->rxPacket)->data + vnic->rxPacketOffset;
        rxState = rxCopy;

        if (dmaInterface) {
            dmaInterface->doDMA(WriteInvalidate, rxDmaAddr, rxDmaLen,
                                curTick, &rxDmaEvent, true);
            goto exit;
        }

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
        vnic->RxDone = vnic->rxDoneData | rxDmaLen;
        vnic->RxDone |= Regs::RxDone_Complete;

        if (vnic->rxPacketBytes == rxDmaLen) {
            DPRINTF(EthernetSM, "rxKick: packet complete on vnic %d\n",
                    rxList.front());
            rxFifo.remove(vnic->rxPacket);
            vnic->rxPacket = rxFifo.end();
        } else {
            vnic->RxDone |= Regs::RxDone_More;
            vnic->rxPacketBytes -= rxDmaLen;
            vnic->rxPacketOffset += rxDmaLen;
            DPRINTF(EthernetSM,
                    "rxKick: packet not complete on vnic %d: %d bytes left\n",
                    rxList.front(), vnic->rxPacketBytes);
        }

        rxList.pop_front();
        rxState = rxList.empty() ? rxIdle : rxFifoBlock;

        if (rxFifo.empty()) {
            devIntrPost(Regs::Intr_RxEmpty);
            rxEmpty = true;
        }

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
Device::txDmaCopy()
{
    assert(txState == txCopy);
    txState = txCopyDone;
    physmem->dma_read((uint8_t *)txDmaData, txDmaAddr, txDmaLen);
    DPRINTF(EthernetDMA, "tx dma read paddr=%#x len=%d\n",
            txDmaAddr, txDmaLen);
    DDUMP(EthernetData, txDmaData, txDmaLen);
}

void
Device::txDmaDone()
{
    txDmaCopy();

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
    DPRINTF(EthernetSM, "transmit kick txState=%s (txFifo.size=%d)\n",
            TxStateStrings[txState], txFifo.size());

    if (txKickTick > curTick) {
        DPRINTF(EthernetSM, "transmit kick exiting, can't run till %d\n",
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
        assert(Regs::get_TxDone_Busy(vnic->TxData));
        if (!txPacket) {
            // Grab a new packet from the fifo.
            txPacket = new PacketData(16384);
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
        if (dmaInterface && dmaInterface->busy())
            goto exit;

        txDmaAddr = plat->pciToDma(Regs::get_TxData_Addr(vnic->TxData));
        txDmaLen = Regs::get_TxData_Len(vnic->TxData);
        txDmaData = txPacket->data + txPacketOffset;
        txState = txCopy;

        if (dmaInterface) {
            dmaInterface->doDMA(Read, txDmaAddr, txDmaLen,
                                curTick, &txDmaEvent, true);
            goto exit;
        }

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

        string reg = csprintf("vnic%d", i);
        paramOut(os, reg + ".RxData", vnic->RxData);
        paramOut(os, reg + ".RxDone", vnic->RxDone);
        paramOut(os, reg + ".TxData", vnic->TxData);
        paramOut(os, reg + ".TxDone", vnic->TxDone);

        PacketFifo::iterator rxFifoPtr;

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

    VirtualList::iterator i, end;
    int count;

    int rxListSize = rxList.size();
    SERIALIZE_SCALAR(rxListSize);
    for (count = 0, i = rxList.begin(), end = rxList.end(); i != end; ++i)
        paramOut(os, csprintf("rxList%d", count++), *i);

    int txListSize = txList.size();
    SERIALIZE_SCALAR(txListSize);
    for (count = 0, i = txList.begin(), end = txList.end(); i != end; ++i)
        paramOut(os, csprintf("txList%d", count++), *i);

    /*
     * Serialize rx state machine
     */
    int rxState = this->rxState;
    SERIALIZE_SCALAR(rxState);
    SERIALIZE_SCALAR(rxEmpty);
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
    UNSERIALIZE_SCALAR(regs.RxData);
    UNSERIALIZE_SCALAR(regs.RxDone);
    UNSERIALIZE_SCALAR(regs.TxData);
    UNSERIALIZE_SCALAR(regs.TxDone);

    int rxListSize;
    UNSERIALIZE_SCALAR(rxListSize);
    rxList.clear();
    for (int i = 0; i < rxListSize; ++i) {
        int value;
        paramIn(cp, section, csprintf("rxList%d", i), value);
        rxList.push_back(value);
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
    this->rxState = (RxState) rxState;
    rxFifo.unserialize("rxFifo", cp, section);

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
        txPacket = new PacketData(16384);
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
        string reg = csprintf("vnic%d", i);

        paramIn(cp, section, reg + ".RxData", vnic->RxData);
        paramIn(cp, section, reg + ".RxDone", vnic->RxDone);
        paramIn(cp, section, reg + ".TxData", vnic->TxData);
        paramIn(cp, section, reg + ".TxDone", vnic->TxDone);

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

    /*
     * re-add addrRanges to bus bridges
     */
    if (pioInterface) {
        pioInterface->addAddrRange(RangeSize(BARAddrs[0], BARSize[0]));
        pioInterface->addAddrRange(RangeSize(BARAddrs[1], BARSize[1]));
    }
}

Tick
Device::cacheAccess(MemReqPtr &req)
{
    Addr daddr;
    int bar;
    if (!getBAR(req->paddr, daddr, bar))
        panic("address does not map to a BAR pa=%#x va=%#x size=%d",
              req->paddr, req->vaddr, req->size);

    DPRINTF(EthernetPIO, "timing %s to paddr=%#x bar=%d daddr=%#x\n",
            req->cmd.toString(), req->paddr, bar, daddr);

    return curTick + pioLatency;
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

    Param<Tick> clock;

    Param<Addr> addr;
    SimObjectParam<MemoryController *> mmu;
    SimObjectParam<PhysicalMemory *> physmem;
    SimObjectParam<PciConfigAll *> configspace;
    SimObjectParam<PciConfigData *> configdata;
    SimObjectParam<Platform *> platform;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;

    SimObjectParam<HierParams *> hier;
    SimObjectParam<Bus*> pio_bus;
    SimObjectParam<Bus*> dma_bus;
    SimObjectParam<Bus*> payload_bus;
    Param<Tick> dma_read_delay;
    Param<Tick> dma_read_factor;
    Param<Tick> dma_write_delay;
    Param<Tick> dma_write_factor;
    Param<bool> dma_no_allocate;
    Param<Tick> pio_latency;
    Param<Tick> intr_delay;

    Param<Tick> rx_delay;
    Param<Tick> tx_delay;
    Param<uint32_t> rx_max_copy;
    Param<uint32_t> tx_max_copy;
    Param<uint32_t> rx_max_intr;
    Param<uint32_t> rx_fifo_size;
    Param<uint32_t> tx_fifo_size;
    Param<uint32_t> rx_fifo_threshold;
    Param<uint32_t> tx_fifo_threshold;

    Param<bool> rx_filter;
    Param<string> hardware_address;
    Param<bool> rx_thread;
    Param<bool> tx_thread;

END_DECLARE_SIM_OBJECT_PARAMS(Device)

BEGIN_INIT_SIM_OBJECT_PARAMS(Device)

    INIT_PARAM(clock, "State machine cycle time"),

    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(physmem, "Physical Memory"),
    INIT_PARAM(configspace, "PCI Configspace"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(platform, "Platform"),
    INIT_PARAM(pci_bus, "PCI bus"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code"),

    INIT_PARAM(hier, "Hierarchy global variables"),
    INIT_PARAM(pio_bus, ""),
    INIT_PARAM(dma_bus, ""),
    INIT_PARAM(payload_bus, "The IO Bus to attach to for payload"),
    INIT_PARAM(dma_read_delay, "fixed delay for dma reads"),
    INIT_PARAM(dma_read_factor, "multiplier for dma reads"),
    INIT_PARAM(dma_write_delay, "fixed delay for dma writes"),
    INIT_PARAM(dma_write_factor, "multiplier for dma writes"),
    INIT_PARAM(dma_no_allocate, "Should we allocat on read in cache"),
    INIT_PARAM(pio_latency, "Programmed IO latency in bus cycles"),
    INIT_PARAM(intr_delay, "Interrupt Delay"),

    INIT_PARAM(rx_delay, "Receive Delay"),
    INIT_PARAM(tx_delay, "Transmit Delay"),
    INIT_PARAM(rx_max_copy, "rx max copy"),
    INIT_PARAM(tx_max_copy, "rx max copy"),
    INIT_PARAM(rx_max_intr, "rx max intr"),
    INIT_PARAM(rx_fifo_size, "max size in bytes of rxFifo"),
    INIT_PARAM(tx_fifo_size, "max size in bytes of txFifo"),
    INIT_PARAM(rx_fifo_threshold, "max size in bytes of rxFifo"),
    INIT_PARAM(tx_fifo_threshold, "max size in bytes of txFifo"),

    INIT_PARAM(rx_filter, "Enable Receive Filter"),
    INIT_PARAM(hardware_address, "Ethernet Hardware Address"),
    INIT_PARAM(rx_thread, ""),
    INIT_PARAM(tx_thread, "")

END_INIT_SIM_OBJECT_PARAMS(Device)


CREATE_SIM_OBJECT(Device)
{
    Device::Params *params = new Device::Params;

    params->name = getInstanceName();

    params->clock = clock;

    params->mmu = mmu;
    params->physmem = physmem;
    params->configSpace = configspace;
    params->configData = configdata;
    params->plat = platform;
    params->busNum = pci_bus;
    params->deviceNum = pci_dev;
    params->functionNum = pci_func;

    params->hier = hier;
    params->pio_bus = pio_bus;
    params->header_bus = dma_bus;
    params->payload_bus = payload_bus;
    params->dma_read_delay = dma_read_delay;
    params->dma_read_factor = dma_read_factor;
    params->dma_write_delay = dma_write_delay;
    params->dma_write_factor = dma_write_factor;
    params->dma_no_allocate = dma_no_allocate;
    params->pio_latency = pio_latency;
    params->intr_delay = intr_delay;

    params->tx_delay = tx_delay;
    params->rx_delay = rx_delay;
    params->rx_max_copy = rx_max_copy;
    params->tx_max_copy = tx_max_copy;
    params->rx_max_intr = rx_max_intr;
    params->rx_fifo_size = rx_fifo_size;
    params->tx_fifo_size = tx_fifo_size;
    params->rx_fifo_threshold = rx_fifo_threshold;
    params->tx_fifo_threshold = tx_fifo_threshold;

    params->rx_filter = rx_filter;
    params->eaddr = hardware_address;
    params->rx_thread = rx_thread;
    params->tx_thread = tx_thread;

    return new Device(params);
}

REGISTER_SIM_OBJECT("Sinic", Device)

/* namespace Sinic */ }

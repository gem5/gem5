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

#include "dev/net/sinic.hh"

#include <deque>
#include <limits>
#include <string>

#ifdef SINIC_VTOPHYS
#include "arch/vtophys.hh"

#endif
#include "base/compiler.hh"
#include "base/debug.hh"
#include "base/inet.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "debug/EthernetAll.hh"
#include "dev/net/etherlink.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/eventq.hh"
#include "sim/stats.hh"

using namespace std;
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
Base::Base(const Params *p)
    : EtherDevBase(p), rxEnable(false), txEnable(false),
      intrDelay(p->intr_delay), intrTick(0), cpuIntrEnable(false),
      cpuPendingIntr(false), intrEvent(0), interface(NULL)
{
}

Device::Device(const Params *p)
    : Base(p), rxUnique(0), txUnique(0),
      virtualRegs(p->virtual_count < 1 ? 1 : p->virtual_count),
      rxFifo(p->rx_fifo_size), txFifo(p->tx_fifo_size),
      rxKickTick(0), txKickTick(0),
      txEvent([this]{ txEventTransmit(); }, name()),
      rxDmaEvent([this]{ rxDmaDone(); }, name()),
      txDmaEvent([this]{ txDmaDone(); }, name()),
      dmaReadDelay(p->dma_read_delay), dmaReadFactor(p->dma_read_factor),
      dmaWriteDelay(p->dma_write_delay), dmaWriteFactor(p->dma_write_factor)
{
    interface = new Interface(name() + ".int0", this);
    reset();

}

Device::~Device()
{}

void
Device::regStats()
{
    Base::regStats();

    _maxVnicDistance = 0;

    maxVnicDistance
        .name(name() + ".maxVnicDistance")
        .desc("maximum vnic distance")
        ;

    totalVnicDistance
        .name(name() + ".totalVnicDistance")
        .desc("total vnic distance")
        ;
    numVnicDistance
        .name(name() + ".numVnicDistance")
        .desc("number of vnic distance measurements")
        ;

    avgVnicDistance
        .name(name() + ".avgVnicDistance")
        .desc("average vnic distance")
        ;

    avgVnicDistance = totalVnicDistance / numVnicDistance;
}

void
Device::resetStats()
{
    Base::resetStats();

    _maxVnicDistance = 0;
}

Port &
Device::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "interface")
        return *interface;
    return EtherDevBase::getPort(if_name, idx);
}


void
Device::prepareIO(ContextID cpu, int index)
{
    int size = virtualRegs.size();
    if (index > size)
        panic("Trying to access a vnic that doesn't exist %d > %d\n",
              index, size);
}

//add stats for head of line blocking
//add stats for average fifo length
//add stats for average number of vnics busy

void
Device::prepareRead(ContextID cpu, int index)
{
    using namespace Regs;
    prepareIO(cpu, index);

    VirtualReg &vnic = virtualRegs[index];

    // update rx registers
    uint64_t rxdone = vnic.RxDone;
    rxdone = set_RxDone_Packets(rxdone, rxFifo.countPacketsAfter(rxFifoPtr));
    rxdone = set_RxDone_Empty(rxdone, rxFifo.empty());
    rxdone = set_RxDone_High(rxdone, rxFifo.size() > regs.RxFifoHigh);
    rxdone = set_RxDone_NotHigh(rxdone, rxLow);
    regs.RxData = vnic.RxData;
    regs.RxDone = rxdone;
    regs.RxWait = rxdone;

    // update tx regsiters
    uint64_t txdone = vnic.TxDone;
    txdone = set_TxDone_Packets(txdone, txFifo.packets());
    txdone = set_TxDone_Full(txdone, txFifo.avail() < regs.TxMaxCopy);
    txdone = set_TxDone_Low(txdone, txFifo.size() < regs.TxFifoLow);
    regs.TxData = vnic.TxData;
    regs.TxDone = txdone;
    regs.TxWait = txdone;

    int head = 0xffff;

    if (!rxFifo.empty()) {
        int vnic = rxFifo.begin()->priv;
        if (vnic != -1 && virtualRegs[vnic].rxPacketOffset > 0)
            head = vnic;
    }

    regs.RxStatus = set_RxStatus_Head(regs.RxStatus, head);
    regs.RxStatus = set_RxStatus_Busy(regs.RxStatus, rxBusyCount);
    regs.RxStatus = set_RxStatus_Mapped(regs.RxStatus, rxMappedCount);
    regs.RxStatus = set_RxStatus_Dirty(regs.RxStatus, rxDirtyCount);
}

void
Device::prepareWrite(ContextID cpu, int index)
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

    ContextID cpu = pkt->req->contextId();
    Addr daddr = pkt->getAddr() - BARAddrs[0];
    Addr index = daddr >> Regs::VirtualShift;
    Addr raddr = daddr & Regs::VirtualMask;

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

    uint64_t value M5_VAR_USED = 0;
    if (pkt->getSize() == 4) {
        uint32_t reg = regData32(raddr);
        pkt->setLE(reg);
        value = reg;
    }

    if (pkt->getSize() == 8) {
        uint64_t reg = regData64(raddr);
        pkt->setLE(reg);
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
Device::iprRead(Addr daddr, ContextID cpu, uint64_t &result)
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

    ContextID cpu = pkt->req->contextId();
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
            info.name, index, cpu, info.size == 4 ?
            pkt->getLE<uint32_t>() : pkt->getLE<uint64_t>(),
            daddr, pkt->getAddr(), pkt->getSize());

    prepareWrite(cpu, index);

    switch (raddr) {
      case Regs::Config:
        changeConfig(pkt->getLE<uint32_t>());
        break;

      case Regs::Command:
        command(pkt->getLE<uint32_t>());
        break;

      case Regs::IntrStatus:
        devIntrClear(regs.IntrStatus &
                pkt->getLE<uint32_t>());
        break;

      case Regs::IntrMask:
        devIntrChangeMask(pkt->getLE<uint32_t>());
        break;

      case Regs::RxData:
        if (Regs::get_RxDone_Busy(vnic.RxDone))
            panic("receive machine busy with another request! rxState=%s",
                  RxStateStrings[rxState]);

        vnic.rxUnique = rxUnique++;
        vnic.RxDone = Regs::RxDone_Busy;
        vnic.RxData = pkt->getLE<uint64_t>();
        rxBusyCount++;

        if (Regs::get_RxData_Vaddr(pkt->getLE<uint64_t>())) {
            panic("vtophys not implemented in newmem");
#ifdef SINIC_VTOPHYS
            Addr vaddr = Regs::get_RxData_Addr(reg64);
            Addr paddr = vtophys(req->xc, vaddr);
            DPRINTF(EthernetPIO, "write RxData vnic %d (rxunique %d): "
                    "vaddr=%#x, paddr=%#x\n",
                    index, vnic.rxUnique, vaddr, paddr);

            vnic.RxData = Regs::set_RxData_Addr(vnic.RxData, paddr);
#endif
        } else {
            DPRINTF(EthernetPIO, "write RxData vnic %d (rxunique %d)\n",
                    index, vnic.rxUnique);
        }

        if (vnic.rxIndex == rxFifo.end()) {
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

        if (Regs::get_TxData_Vaddr(pkt->getLE<uint64_t>())) {
            panic("vtophys won't work here in newmem.\n");
#ifdef SINIC_VTOPHYS
            Addr vaddr = Regs::get_TxData_Addr(reg64);
            Addr paddr = vtophys(req->xc, vaddr);
            DPRINTF(EthernetPIO, "write TxData vnic %d (txunique %d): "
                    "vaddr=%#x, paddr=%#x\n",
                    index, vnic.txUnique, vaddr, paddr);

            vnic.TxData = Regs::set_TxData_Addr(vnic.TxData, paddr);
#endif
        } else {
            DPRINTF(EthernetPIO, "write TxData vnic %d (txunique %d)\n",
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
        Tick when = curTick();
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
        cpuIntrPost(curTick());
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
    assert(when >= curTick());
    assert(intrTick >= curTick() || intrTick == 0);
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
    if (intrTick < curTick()) {
        intrTick = curTick();
    }

    DPRINTF(EthernetIntr, "going to schedule an interrupt for intrTick=%d\n",
            intrTick);

    if (intrEvent)
        intrEvent->squash();

    intrEvent = new EventFunctionWrapper([this]{ cpuInterrupt(); },
                                         name(), true);
    schedule(intrEvent, intrTick);
}

void
Base::cpuInterrupt()
{
    assert(intrTick == curTick());

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
                cpuIntrPost(curTick());
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
    regs.ZeroCopySize = params()->zero_copy_size;
    regs.ZeroCopyMark = params()->zero_copy_threshold;
    regs.VirtualCount = params()->virtual_count;
    regs.RxMaxIntr = params()->rx_max_intr;
    regs.RxFifoSize = params()->rx_fifo_size;
    regs.TxFifoSize = params()->tx_fifo_size;
    regs.RxFifoLow = params()->rx_fifo_low_mark;
    regs.TxFifoLow = params()->tx_fifo_threshold;
    regs.RxFifoHigh = params()->rx_fifo_threshold;
    regs.TxFifoHigh = params()->tx_fifo_high_mark;
    regs.HwAddr = params()->hardware_address;

    if (regs.RxMaxCopy < regs.ZeroCopyMark)
        panic("Must be able to copy at least as many bytes as the threshold");

    if (regs.ZeroCopySize >= regs.ZeroCopyMark)
        panic("The number of bytes to copy must be less than the threshold");

    rxList.clear();
    rxBusy.clear();
    rxActive = -1;
    txList.clear();
    rxBusyCount = 0;
    rxDirtyCount = 0;
    rxMappedCount = 0;

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
        virtualRegs[i].rxIndex = rxFifo.end();
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

    if (rxKickTick > curTick()) {
        DPRINTF(EthernetSM, "rxKick: exiting, can't run till %d\n",
                rxKickTick);
        return;
    }

  next:
    rxFifo.check();
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
                bool busy = Regs::get_RxDone_Busy(vn->RxDone);
                if (vn->rxIndex != end) {
#ifndef NDEBUG
                    bool dirty = vn->rxPacketOffset > 0;
                    const char *status;

                    if (busy && dirty)
                        status = "busy,dirty";
                    else if (busy)
                        status = "busy";
                    else if (dirty)
                        status = "dirty";
                    else
                        status = "mapped";

                    DPRINTF(EthernetSM,
                            "vnic %d %s (rxunique %d), packet %d, slack %d\n",
                            i, status, vn->rxUnique,
                            rxFifo.countPacketsBefore(vn->rxIndex),
                            vn->rxIndex->slack);
#endif
                } else if (busy) {
                    DPRINTF(EthernetSM, "vnic %d unmapped (rxunique %d)\n",
                            i, vn->rxUnique);
                }
            }
        }

        if (!rxBusy.empty()) {
            rxActive = rxBusy.front();
            rxBusy.pop_front();
            vnic = &virtualRegs[rxActive];

            if (vnic->rxIndex == rxFifo.end())
                panic("continuing vnic without packet\n");

            DPRINTF(EthernetSM,
                    "continue processing for vnic %d (rxunique %d)\n",
                    rxActive, vnic->rxUnique);

            rxState = rxBeginCopy;

            int vnic_distance = rxFifo.countPacketsBefore(vnic->rxIndex);
            totalVnicDistance += vnic_distance;
            numVnicDistance += 1;
            if (vnic_distance > _maxVnicDistance) {
                maxVnicDistance = vnic_distance;
                _maxVnicDistance = vnic_distance;
            }

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
        vnic->rxIndex = rxFifoPtr++;
        vnic->rxIndex->priv = rxActive;
        vnic->rxPacketOffset = 0;
        vnic->rxPacketBytes = vnic->rxIndex->packet->length;
        assert(vnic->rxPacketBytes);
        rxMappedCount++;

        vnic->rxDoneData = 0;
        /* scope for variables */ {
            IpPtr ip(vnic->rxIndex->packet);
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
        if (dmaPending() || drainState() != DrainState::Running)
            goto exit;

        rxDmaAddr = pciToDma(Regs::get_RxData_Addr(vnic->RxData));
        rxDmaLen = min<unsigned>(Regs::get_RxData_Len(vnic->RxData),
                                 vnic->rxPacketBytes);

        /*
         * if we're doing zero/delay copy and we're below the fifo
         * threshold, see if we should try to do the zero/defer copy
         */
        if ((Regs::get_Config_ZeroCopy(regs.Config) ||
             Regs::get_Config_DelayCopy(regs.Config)) &&
            !Regs::get_RxData_NoDelay(vnic->RxData) && rxLow) {
            if (rxDmaLen > regs.ZeroCopyMark)
                rxDmaLen = regs.ZeroCopySize;
        }
        rxDmaData = vnic->rxIndex->packet->data + vnic->rxPacketOffset;
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
        rxBusyCount--;

        if (vnic->rxPacketBytes == rxDmaLen) {
            if (vnic->rxPacketOffset)
                rxDirtyCount--;

            // Packet is complete.  Indicate how many bytes were copied
            vnic->RxDone = Regs::set_RxDone_CopyLen(vnic->RxDone, rxDmaLen);

            DPRINTF(EthernetSM,
                    "rxKick: packet complete on vnic %d (rxunique %d)\n",
                    rxActive, vnic->rxUnique);
            rxFifo.remove(vnic->rxIndex);
            vnic->rxIndex = rxFifo.end();
            rxMappedCount--;
        } else {
            if (!vnic->rxPacketOffset)
                rxDirtyCount++;

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

        if (rxFifo.size() < regs.RxFifoLow)
            rxLow = true;

        if (rxFifo.size() > regs.RxFifoHigh)
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
        return;
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
    if (txFifo.size() < regs.TxFifoLow)
        interrupts |= Regs::Intr_TxLow;
    devIntrPost(interrupts);
}

void
Device::txKick()
{
    VirtualReg *vnic;
    DPRINTF(EthernetSM, "txKick: txState=%s (txFifo.size=%d)\n",
            TxStateStrings[txState], txFifo.size());

    if (txKickTick > curTick()) {
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
            txPacket = make_shared<EthPacketData>(16384);
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
        if (dmaPending() || drainState() != DrainState::Running)
            goto exit;

        txDmaAddr = pciToDma(Regs::get_TxData_Addr(vnic->TxData));
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
        txPacket->simLength += txDmaLen;
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

    reschedule(txEvent, clockEdge(Cycles(1)), true);
}

bool
Device::rxFilter(const EthPacketPtr &packet)
{
    if (!Regs::get_Config_Filter(regs.Config))
        return false;

    panic("receive filter not implemented\n");
    bool drop = true;
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

    if (rxFifo.size() >= regs.RxFifoHigh)
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
Device::drainResume()
{
    Drainable::drainResume();

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
Base::serialize(CheckpointOut &cp) const
{
    // Serialize the PciDevice base class
    PciDevice::serialize(cp);

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
Base::unserialize(CheckpointIn &cp)
{
    // Unserialize the PciDevice base class
    PciDevice::unserialize(cp);

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
        intrEvent = new EventFunctionWrapper([this]{ cpuInterrupt(); },
                                             name(), true);
        schedule(intrEvent, intrEventTick);
    }
}

void
Device::serialize(CheckpointOut &cp) const
{
    int count;

    // Serialize the PciDevice base class
    Base::serialize(cp);

    if (rxState == rxCopy)
        panic("can't serialize with an in flight dma request rxState=%s",
              RxStateStrings[rxState]);

    if (txState == txCopy)
        panic("can't serialize with an in flight dma request txState=%s",
              TxStateStrings[txState]);

    /*
     * Serialize the device registers that could be modified by the OS.
     */
    SERIALIZE_SCALAR(regs.Config);
    SERIALIZE_SCALAR(regs.IntrStatus);
    SERIALIZE_SCALAR(regs.IntrMask);
    SERIALIZE_SCALAR(regs.RxData);
    SERIALIZE_SCALAR(regs.TxData);

    /*
     * Serialize the virtual nic state
     */
    int virtualRegsSize = virtualRegs.size();
    SERIALIZE_SCALAR(virtualRegsSize);
    for (int i = 0; i < virtualRegsSize; ++i) {
        const VirtualReg *vnic = &virtualRegs[i];

        std::string reg = csprintf("vnic%d", i);
        paramOut(cp, reg + ".RxData", vnic->RxData);
        paramOut(cp, reg + ".RxDone", vnic->RxDone);
        paramOut(cp, reg + ".TxData", vnic->TxData);
        paramOut(cp, reg + ".TxDone", vnic->TxDone);

        bool rxPacketExists = vnic->rxIndex != rxFifo.end();
        paramOut(cp, reg + ".rxPacketExists", rxPacketExists);
        if (rxPacketExists) {
            int rxPacket = 0;
            auto i = rxFifo.begin();
            while (i != vnic->rxIndex) {
                assert(i != rxFifo.end());
                ++i;
                ++rxPacket;
            }

            paramOut(cp, reg + ".rxPacket", rxPacket);
            paramOut(cp, reg + ".rxPacketOffset", vnic->rxPacketOffset);
            paramOut(cp, reg + ".rxPacketBytes", vnic->rxPacketBytes);
        }
        paramOut(cp, reg + ".rxDoneData", vnic->rxDoneData);
    }

    int rxFifoPtr = -1;
    if (this->rxFifoPtr != rxFifo.end())
        rxFifoPtr = rxFifo.countPacketsBefore(this->rxFifoPtr);
    SERIALIZE_SCALAR(rxFifoPtr);

    SERIALIZE_SCALAR(rxActive);
    SERIALIZE_SCALAR(rxBusyCount);
    SERIALIZE_SCALAR(rxDirtyCount);
    SERIALIZE_SCALAR(rxMappedCount);

    VirtualList::const_iterator i, end;
    for (count = 0, i = rxList.begin(), end = rxList.end(); i != end; ++i)
        paramOut(cp, csprintf("rxList%d", count++), *i);
    int rxListSize = count;
    SERIALIZE_SCALAR(rxListSize);

    for (count = 0, i = rxBusy.begin(), end = rxBusy.end(); i != end; ++i)
        paramOut(cp, csprintf("rxBusy%d", count++), *i);
    int rxBusySize = count;
    SERIALIZE_SCALAR(rxBusySize);

    for (count = 0, i = txList.begin(), end = txList.end(); i != end; ++i)
        paramOut(cp, csprintf("txList%d", count++), *i);
    int txListSize = count;
    SERIALIZE_SCALAR(txListSize);

    /*
     * Serialize rx state machine
     */
    int rxState = this->rxState;
    SERIALIZE_SCALAR(rxState);
    SERIALIZE_SCALAR(rxEmpty);
    SERIALIZE_SCALAR(rxLow);
    rxFifo.serialize("rxFifo", cp);

    /*
     * Serialize tx state machine
     */
    int txState = this->txState;
    SERIALIZE_SCALAR(txState);
    SERIALIZE_SCALAR(txFull);
    txFifo.serialize("txFifo", cp);
    bool txPacketExists = txPacket != nullptr;
    SERIALIZE_SCALAR(txPacketExists);
    if (txPacketExists) {
        txPacket->serialize("txPacket", cp);
        SERIALIZE_SCALAR(txPacketOffset);
        SERIALIZE_SCALAR(txPacketBytes);
    }

    /*
     * If there's a pending transmit, store the time so we can
     * reschedule it later
     */
    Tick transmitTick = txEvent.scheduled() ? txEvent.when() - curTick() : 0;
    SERIALIZE_SCALAR(transmitTick);
}

void
Device::unserialize(CheckpointIn &cp)
{
    // Unserialize the PciDevice base class
    Base::unserialize(cp);

    /*
     * Unserialize the device registers that may have been written by the OS.
     */
    UNSERIALIZE_SCALAR(regs.Config);
    UNSERIALIZE_SCALAR(regs.IntrStatus);
    UNSERIALIZE_SCALAR(regs.IntrMask);
    UNSERIALIZE_SCALAR(regs.RxData);
    UNSERIALIZE_SCALAR(regs.TxData);

    UNSERIALIZE_SCALAR(rxActive);
    UNSERIALIZE_SCALAR(rxBusyCount);
    UNSERIALIZE_SCALAR(rxDirtyCount);
    UNSERIALIZE_SCALAR(rxMappedCount);

    int rxListSize;
    UNSERIALIZE_SCALAR(rxListSize);
    rxList.clear();
    for (int i = 0; i < rxListSize; ++i) {
        int value;
        paramIn(cp, csprintf("rxList%d", i), value);
        rxList.push_back(value);
    }

    int rxBusySize;
    UNSERIALIZE_SCALAR(rxBusySize);
    rxBusy.clear();
    for (int i = 0; i < rxBusySize; ++i) {
        int value;
        paramIn(cp, csprintf("rxBusy%d", i), value);
        rxBusy.push_back(value);
    }

    int txListSize;
    UNSERIALIZE_SCALAR(txListSize);
    txList.clear();
    for (int i = 0; i < txListSize; ++i) {
        int value;
        paramIn(cp, csprintf("txList%d", i), value);
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
    rxFifo.unserialize("rxFifo", cp);

    int rxFifoPtr;
    UNSERIALIZE_SCALAR(rxFifoPtr);
    if (rxFifoPtr >= 0) {
        this->rxFifoPtr = rxFifo.begin();
        for (int i = 0; i < rxFifoPtr; ++i)
            ++this->rxFifoPtr;
    } else {
        this->rxFifoPtr = rxFifo.end();
    }

    /*
     * Unserialize tx state machine
     */
    int txState;
    UNSERIALIZE_SCALAR(txState);
    UNSERIALIZE_SCALAR(txFull);
    this->txState = (TxState) txState;
    txFifo.unserialize("txFifo", cp);
    bool txPacketExists;
    UNSERIALIZE_SCALAR(txPacketExists);
    txPacket = 0;
    if (txPacketExists) {
        txPacket = make_shared<EthPacketData>(16384);
        txPacket->unserialize("txPacket", cp);
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

        paramIn(cp, reg + ".RxData", vnic->RxData);
        paramIn(cp, reg + ".RxDone", vnic->RxDone);
        paramIn(cp, reg + ".TxData", vnic->TxData);
        paramIn(cp, reg + ".TxDone", vnic->TxDone);

        vnic->rxUnique = rxUnique++;
        vnic->txUnique = txUnique++;

        bool rxPacketExists;
        paramIn(cp, reg + ".rxPacketExists", rxPacketExists);
        if (rxPacketExists) {
            int rxPacket;
            paramIn(cp, reg + ".rxPacket", rxPacket);
            vnic->rxIndex = rxFifo.begin();
            while (rxPacket--)
                ++vnic->rxIndex;

            paramIn(cp, reg + ".rxPacketOffset",
                    vnic->rxPacketOffset);
            paramIn(cp, reg + ".rxPacketBytes", vnic->rxPacketBytes);
        } else {
            vnic->rxIndex = rxFifo.end();
        }
        paramIn(cp, reg + ".rxDoneData", vnic->rxDoneData);
    }

    /*
     * If there's a pending transmit, reschedule it now
     */
    Tick transmitTick;
    UNSERIALIZE_SCALAR(transmitTick);
    if (transmitTick)
        schedule(txEvent, curTick() + transmitTick);

    pioPort.sendRangeChange();

}

} // namespace Sinic

Sinic::Device *
SinicParams::create()
{
    return new Sinic::Device(this);
}

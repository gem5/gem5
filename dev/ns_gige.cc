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
 * DP83820 ethernet controller.  Does not support priority queueing
 */
#include <cstdio>
#include <deque>
#include <string>

#include "base/inet.hh"
#include "cpu/exec_context.hh"
#include "cpu/intr_control.hh"
#include "dev/dma.hh"
#include "dev/ns_gige.hh"
#include "dev/etherlink.hh"
#include "mem/functional_mem/memory_control.hh"
#include "mem/functional_mem/physical_memory.hh"
#include "sim/builder.hh"
#include "sim/host.hh"
#include "sim/sim_stats.hh"
#include "targetarch/vtophys.hh"

using namespace std;

///////////////////////////////////////////////////////////////////////
//
// EtherDev PCI Device
//
EtherDev::EtherDev(const string &_name, DmaEngine *de, bool use_interface,
                   IntrControl *i, MemoryController *mmu, PhysicalMemory *pmem,
                   PCIConfigAll *cf, PciConfigData *cd, Tsunami *t, uint32_t bus,
                   uint32_t dev, uint32_t func, bool rx_filter,
                   const int eaddr[6], Tick tx_delay, Tick rx_delay, Addr addr,
                   Addr mask)
    : PciDev(_name, mmu, cf, cd, bus, dev, func), tsunami(t),
      addr(addr), mask(mask), txPacketLen(0),
      txPacketBufPtr(NULL), rxPacketBufPtr(NULL), rxDescBufPtr(NULL),
      fragLen(0), rxCopied(0), txState(txIdle), CTDD(false), txFifoCnt(0),
      txFifoAvail(MAX_TX_FIFO_SIZE), txHalt(false), txPacketFlag(false),
      txFragPtr(0), txDescCnt(0), rxState(rxIdle), CRDD(false),
      rxPktBytes(0), rxFifoCnt(0), rxHalt(false), rxPacketFlag(false),
      rxFragPtr(0), rxDescCnt(0), extstsEnable(false), maxTxBurst(0),
      maxRxBurst(0), physmem(pmem),
      rxDescDoneCB(this), rxDoneCB(this), txDescDoneCB(this), txDoneCB(this),
      dma(de), readRequest(use_interface), writeRequest(use_interface),
      readDescRequest(use_interface), writeDescRequest(use_interface),
      interface(NULL), intctrl(i), txDelay(tx_delay), rxDelay(rx_delay),
      txEvent(this), cpuPendingIntr(false), rxFilterEnable(rx_filter),
      acceptBroadcast(false), acceptMulticast(false), acceptUnicast(false),
      acceptPerfect(false), acceptArp(false)
{
    tsunami->ethernet = this;

    memset(&regs, 0, sizeof(regs));
    regsReset();
    regs.perfectMatch[0] = eaddr[0];
    regs.perfectMatch[1] = eaddr[1];
    regs.perfectMatch[2] = eaddr[2];
    regs.perfectMatch[3] = eaddr[3];
    regs.perfectMatch[4] = eaddr[4];
    regs.perfectMatch[5] = eaddr[5];

}

EtherDev::~EtherDev()
{}

void
EtherDev::regStats()
{
    txBytes
        .name(name() + ".txBytes")
        .desc("Bytes Transmitted")
        .prereq(txBytes)
        ;

    rxBytes
        .name(name() + ".rxBytes")
        .desc("Bytes Received")
        .prereq(rxBytes)
        ;

    txPackets
        .name(name() + ".txPackets")
        .desc("Number of Packets Transmitted")
        .prereq(txBytes)
        ;

    rxPackets
        .name(name() + ".rxPackets")
        .desc("Number of Packets Received")
        .prereq(rxBytes)
        ;

    txBandwidth
        .name(name() + ".txBandwidth")
        .desc("Transmit Bandwidth (bits/s)")
        .precision(0)
        .prereq(txBytes)
        ;

    rxBandwidth
        .name(name() + ".rxBandwidth")
        .desc("Receive Bandwidth (bits/s)")
        .precision(0)
        .prereq(rxBytes)
        ;

    txPacketRate
        .name(name() + ".txPPS")
        .desc("Packet Tranmission Rate (packets/s)")
        .precision(0)
        .prereq(txBytes)
        ;

    rxPacketRate
        .name(name() + ".rxPPS")
        .desc("Packet Reception Rate (packets/s)")
        .precision(0)
        .prereq(rxBytes)
        ;

    txBandwidth = txBytes * Statistics::constant(8) / simSeconds;
    rxBandwidth = rxBytes * Statistics::constant(8) / simSeconds;
    txPacketRate = txPackets / simSeconds;
    rxPacketRate = rxPackets / simSeconds;
}

void
EtherDev::ReadConfig(int offset, int size, uint8_t *data)
{
    if (offset < PCI_DEVICE_SPECIFIC)
        PciDev::ReadConfig(offset, size, data);
    else {
        panic("need to do this\n");
    }
}

void
EtherDev::WriteConfig(int offset, int size, uint32_t data)
{
    if (offset < PCI_DEVICE_SPECIFIC)
        PciDev::WriteConfig(offset, size, data);
    else
        panic("Need to do that\n");
}

Fault
EtherDev::read(MemReqPtr req, uint8_t *data)
{
    DPRINTF(Ethernet, "read  va=%#x size=%d\n", req->vaddr, req->size);

    Addr daddr = req->paddr - addr;

    if (daddr > LAST)
        panic("Accessing reserved register");

    switch (req->size) {
      case sizeof(uint32_t):
        {
            uint32_t &reg = *(uint32_t *)data;

            switch (daddr) {
              case CR:
                reg = regs.command;
                reg &= ~(CR_RXD | CR_TXD | CR_TXR | CR_RXR);
                break;

              case CFG:
                reg = regs.config;
                break;

              case MEAR:
                reg = regs.mear;
                break;

              case PTSCR:
                reg = regs.ptscr;
                break;

              case ISR:
                reg = regs.isr;
                regs.isr = 0;
                break;

              case IMR:
                reg = regs.imr;
                break;

              case IER:
                reg = regs.ier;
                break;

              case IHR:
                reg = regs.ihr;
                break;

              case TXDP:
                reg = regs.txdp;
                break;

              case TXDP_HI:
                reg = regs.txdp_hi;
                break;

              case TXCFG:
                reg = regs.txcfg;
                break;

              case GPIOR:
                reg = regs.gpior;
                break;

              case RXDP:
                reg = regs.rxdp;
                break;

              case RXDP_HI:
                reg = regs.rxdp_hi;
                break;

              case RXCFG:
                reg = regs.rxcfg;
                break;

              case PQCR:
                reg = regs.pqcr;
                break;

              case WCSR:
                reg = regs.wcsr;
                break;

              case PCR:
                reg = regs.pcr;
                break;

              case RFCR:
                reg = regs.rfcr;
                break;

              case RFDR:

                switch (regs.rfcr & RFCR_RFADDR) {
                  case 0x000:
                    reg = regs.perfectMatch[1] << 8;
                    reg += regs.perfectMatch[0];
                    break;
                  case 0x002:
                    reg = regs.perfectMatch[3] << 8;
                    reg += regs.perfectMatch[2];
                    break;
                  case 0x004:
                    reg = regs.perfectMatch[5] << 8;
                    reg += regs.perfectMatch[4];
                    break;
                  default:
                    panic("reading from RFDR for something for other than PMATCH!\n");
                    //didn't implement other RFDR functionality b/c driver didn't use
                }
                break;

              case SRR:
                reg = regs.srr;
                break;

              case MIBC:
                reg = regs.mibc;
                reg &= ~(MIBC_MIBS | MIBC_ACLR);
                break;

              case VRCR:
                reg = regs.vrcr;
                break;

              case VTCR:
                reg = regs.vtcr;
                break;

              case VDR:
                reg = regs.vdr;
                break;

              case CCSR:
                reg = regs.ccsr;
                break;

              case TBICR:
                reg = regs.tbicr;
                break;

              case TBISR:
                reg = regs.tbisr;
                break;

              case TANAR:
                reg = regs.tanar;
                break;

              case TANLPAR:
                reg = regs.tanlpar;
                break;

              case TANER:
                reg = regs.taner;
                break;

              case TESR:
                reg = regs.tesr;
                break;

              default:
                panic("reading unimplemented register: addr = %#x", daddr);
            }

            DPRINTF(Ethernet, "read from %#x: data=%d data=%#x\n", daddr, reg, reg);
        }
        break;

      default:
        panic("accessing register with invalid size: addr=%#x, size=%d",
              daddr, req->size);
    }

    return No_Fault;
}

Fault
EtherDev::write(MemReqPtr req, const uint8_t *data)
{
    DPRINTF(Ethernet, "write va=%#x size=%d\n", req->vaddr, req->size);

    Addr daddr = req->paddr - addr;

    if (daddr > LAST && daddr <= RESERVED)
        panic("Accessing reserved register");

    if (daddr > RESERVED)
        panic("higher memory accesses not implemented!\n");

    if (req->size == sizeof(uint32_t)) {
        uint32_t reg = *(uint32_t *)data;
        DPRINTF(Ethernet, "write data=%d data=%#x\n", reg, reg);

        switch (daddr) {
          case CR:
            regs.command = reg;
            if ((reg & (CR_TXE | CR_TXD)) == (CR_TXE | CR_TXD)) {
                txHalt = true;
            } else if (reg & CR_TXE) {
                if (txState == txIdle)
                    txKick();
            } else if (reg & CR_TXD) {
                txHalt = true;
            }

            if ((reg & (CR_RXE | CR_RXD)) == (CR_RXE | CR_RXD)) {
                rxHalt = true;
            } else if (reg & CR_RXE) {
                if (rxState == rxIdle) {
                    rxKick();
                }
            } else if (reg & CR_RXD) {
                rxHalt = true;
            }

            if (reg & CR_TXR)
                txReset();

            if (reg & CR_RXR)
                rxReset();

            if (reg & CR_SWI)
                devIntrPost(ISR_SWI);

            if (reg & CR_RST) {
                txReset();
                rxReset();
                regsReset();
            }
            break;

          case CFG:
            regs.config = reg;
            if (reg & CFG_LNKSTS || reg & CFG_SPDSTS || reg & CFG_DUPSTS
                || reg & CFG_RESERVED || reg & CFG_T64ADDR
                || reg & CFG_PCI64_DET)
                panic("writing to read-only or reserved CFG bits!\n");

#if 0
              if (reg & CFG_TBI_EN) ;
              if (reg & CFG_MODE_1000) ;
#endif

            if (reg & CFG_AUTO_1000)
                panic("CFG_AUTO_1000 not implemented!\n");

#if 0
            if (reg & CFG_PINT_DUPSTS || reg & CFG_PINT_LNKSTS || reg & CFG_PINT_SPDSTS) ;
            if (reg & CFG_TMRTEST) ;
            if (reg & CFG_MRM_DIS) ;
            if (reg & CFG_MWI_DIS) ;
#endif

            if (reg & CFG_T64ADDR)
                panic("CFG_T64ADDR is read only register!\n");

            if (reg & CFG_PCI64_DET)
                panic("CFG_PCI64_DET is read only register!\n");

#if 0
              if (reg & CFG_DATA64_EN) ;
              if (reg & CFG_M64ADDR) ;
              if (reg & CFG_PHY_RST) ;
              if (reg & CFG_PHY_DIS) ;
#endif

            if (reg & CFG_EXTSTS_EN)
                extstsEnable = true;
            else
                extstsEnable = false;

#if 0
              if (reg & CFG_REQALG) ;
              if (reg & CFG_SB) ;
              if (reg & CFG_POW) ;
              if (reg & CFG_EXD) ;
              if (reg & CFG_PESEL) ;
              if (reg & CFG_BROM_DIS) ;
              if (reg & CFG_EXT_125) ;
              if (reg & CFG_BEM) ;
#endif
            break;

          case MEAR:
            regs.mear = reg;
            /* since phy is completely faked, MEAR_MD* don't matter
               and since the driver never uses MEAR_EE*, they don't matter */
#if 0
            if (reg & MEAR_EEDI) ;
            if (reg & MEAR_EEDO) ; //this one is read only
            if (reg & MEAR_EECLK) ;
            if (reg & MEAR_EESEL) ;
            if (reg & MEAR_MDIO) ;
            if (reg & MEAR_MDDIR) ;
            if (reg & MEAR_MDC) ;
#endif
            break;

          case PTSCR:
            regs.ptscr = reg;
            /* these control BISTs for various parts of chip - we don't care or do */
            break;

          case ISR: /* writing to the ISR has no effect */
            panic("ISR is a read only register!\n");

          case IMR:
            regs.imr = reg;
            devIntrChangeMask();
            break;

          case IER:
            regs.ier = reg;
            break;

          case IHR:
            regs.ihr = reg;
            /* not going to implement real interrupt holdoff */
            break;

          case TXDP:
            regs.txdp = (reg & 0xFFFFFFFC);
            assert(txState == txIdle);
            CTDD = false;
            break;

          case TXDP_HI:
            regs.txdp_hi = reg;
            break;

          case TXCFG:
            regs.txcfg = reg;
#if 0
            if (reg & TXCFG_CSI) ;
            if (reg & TXCFG_HBI) ;
            if (reg & TXCFG_MLB) ;
            if (reg & TXCFG_ATP) ;
            if (reg & TXCFG_ECRETRY) ;  /* this could easily be implemented, but
                                           considering the network is just a fake
                                           pipe, wouldn't make sense to do this */

            if (reg & TXCFG_BRST_DIS) ;
#endif

#if 0 /* current 2.6 driver doesn't use these.  if we upgrade, may need these */
            if (reg & TXCFG_MXDMA1024)
                maxTxBurst = 1024;

            if (reg & TXCFG_MXDMA8)
                maxTxBurst = 8;

            if (reg & TXCFG_MXDMA16)
                maxTxBurst = 16;

            if (reg & TXCFG_MXDMA32)
                maxTxBurst = 32;

            if (reg & TXCFG_MXDMA64)
                maxTxBurst = 64;

            if (reg & TXCFG_MXDMA128)
                maxTxBurst = 128;

            if (reg & TXCFG_MXDMA256)
                maxTxBurst = 256;
#endif

            if (reg & TXCFG_MXDMA512)
                maxTxBurst = 512;

            break;

          case GPIOR:
            regs.gpior = reg;
            /* these just control general purpose i/o pins, don't matter */
            break;

          case RXCFG:
            regs.rxcfg = reg;
#if 0
            if (reg & RXCFG_AEP) ;
            if (reg & RXCFG_ARP) ;
            if (reg & RXCFG_STRIPCRC) ;
            if (reg & RXCFG_RX_RD) ;
            if (reg & RXCFG_ALP) ;
            if (reg & RXCFG_AIRL) ;
#endif

            if (reg & RXCFG_MXDMA512)
                maxRxBurst = 512;

#if 0
            if (reg & (RXCFG_DRTH | RXCFG_DRTH0)) ;
#endif
            break;

          case PQCR:
            /* there is no priority queueing used in the linux 2.6 driver */
            regs.pqcr = reg;
            break;

          case WCSR:
            /* not going to implement wake on LAN */
            regs.wcsr = reg;
            break;

          case PCR:
            /* not going to implement pause control */
            regs.pcr = reg;
            break;

          case RFCR:
            regs.rfcr = reg;
            rxFilterEnable = (reg & RFCR_RFEN) ? true : false;

            acceptBroadcast = (reg & RFCR_AAB) ? true : false;

            acceptMulticast = (reg & RFCR_AAM) ? true : false;

            acceptUnicast = (reg & RFCR_AAU) ? true : false;

            acceptPerfect = (reg & RFCR_APM) ? true : false;

            acceptArp = (reg & RFCR_AARP) ? true : false;

            if (reg & RFCR_APAT)
                panic("RFCR_APAT not implemented!\n");

            if (reg & RFCR_MHEN || reg & RFCR_UHEN)
                panic("hash filtering not implemented!\n");

            if (reg & RFCR_ULM)
                panic("RFCR_ULM not implemented!\n");

            break;

          case RFDR:
            panic("the driver never writes to RFDR, something is wrong!\n");

          case BRAR:
            panic("the driver never uses BRAR, something is wrong!\n");

          case BRDR:
            panic("the driver never uses BRDR, something is wrong!\n");

          case SRR:
            panic("SRR is read only register!\n");

          case MIBC:
            panic("the driver never uses MIBC, something is wrong!\n");

          case VRCR:
            regs.vrcr = reg;
            break;

          case VTCR:
            regs.vtcr = reg;
            break;

          case VDR:
            panic("the driver never uses VDR, something is wrong!\n");
            break;

          case CCSR:
            /* not going to implement clockrun stuff */
            regs.ccsr = reg;
            break;

          case TBICR:
            regs.tbicr = reg;
            if (reg & TBICR_MR_LOOPBACK)
                panic("TBICR_MR_LOOPBACK never used, something wrong!\n");

            if (reg & TBICR_MR_AN_ENABLE) {
                regs.tanlpar = regs.tanar;
                regs.tbisr |= (TBISR_MR_AN_COMPLETE | TBISR_MR_LINK_STATUS);
            }

#if 0
            if (reg & TBICR_MR_RESTART_AN) ;
#endif

            break;

          case TBISR:
            panic("TBISR is read only register!\n");

          case TANAR:
            regs.tanar = reg;
            if (reg & TANAR_PS2)
                panic("this isn't used in driver, something wrong!\n");

            if (reg & TANAR_PS1)
                panic("this isn't used in driver, something wrong!\n");
            break;

          case TANLPAR:
            panic("this should only be written to by the fake phy!\n");

          case TANER:
            panic("TANER is read only register!\n");

          case TESR:
            regs.tesr = reg;
            break;

          default:
            panic("thought i covered all the register, what is this? addr=%#x",
                  daddr);
        }
    } else
        panic("Invalid Request Size");

    return No_Fault;
}

void
EtherDev::devIntrPost(uint32_t interrupts)
{
DPRINTF(Ethernet, "interrupt posted intr=%x isr=%x imr=%x\n",
    interrupts, regs.isr, regs.imr);

if (interrupts & ISR_RESERVE)
    panic("Cannot set a reserved interrupt");

if (interrupts & ISR_TXRCMP)
    regs.isr |= ISR_TXRCMP;

if (interrupts & ISR_RXRCMP)
    regs.isr |= ISR_RXRCMP;

//ISR_DPERR  not implemented
//ISR_SSERR not implemented
//ISR_RMABT not implemented
//ISR_RXSOVR not implemented
//ISR_HIBINT not implemented
//ISR_PHY not implemented
//ISR_PME not implemented

if (interrupts & ISR_SWI)
    regs.isr |= ISR_SWI;

//ISR_MIB not implemented
//ISR_TXURN not implemented

 if (interrupts & ISR_TXIDLE)
     regs.isr |= ISR_TXIDLE;

 if (interrupts & ISR_TXERR)
     regs.isr |= ISR_TXERR;

 if (interrupts & ISR_TXDESC)
     regs.isr |= ISR_TXDESC;

 if (interrupts & ISR_TXOK)
     regs.isr |= ISR_TXOK;

 if (interrupts & ISR_RXORN)
     regs.isr |= ISR_RXORN;

 if (interrupts & ISR_RXIDLE)
     regs.isr |= ISR_RXIDLE;

//ISR_RXEARLY not implemented

 if (interrupts & ISR_RXERR)
     regs.isr |= ISR_RXERR;

 if (interrupts & ISR_RXOK)
     regs.isr |= ISR_RXOK;

 if ((regs.isr & regs.imr))
        cpuIntrPost();
}

void
EtherDev::devIntrClear(uint32_t interrupts)
{
    DPRINTF(Ethernet, "interrupt cleared intr=%x isr=%x imr=%x\n",
            interrupts, regs.isr, regs.imr);

    if (interrupts & ISR_RESERVE)
        panic("Cannot clear a reserved interrupt");

    if (interrupts & ISR_TXRCMP)
        regs.isr &= ~ISR_TXRCMP;

    if (interrupts & ISR_RXRCMP)
        regs.isr &= ~ISR_RXRCMP;

//ISR_DPERR  not implemented
//ISR_SSERR not implemented
//ISR_RMABT not implemented
//ISR_RXSOVR not implemented
//ISR_HIBINT not implemented
//ISR_PHY not implemented
//ISR_PME not implemented

    if (interrupts & ISR_SWI)
        regs.isr &= ~ISR_SWI;

//ISR_MIB not implemented
//ISR_TXURN not implemented

    if (interrupts & ISR_TXIDLE)
        regs.isr &= ~ISR_TXIDLE;

    if (interrupts & ISR_TXERR)
        regs.isr &= ~ISR_TXERR;

    if (interrupts & ISR_TXDESC)
        regs.isr &= ~ISR_TXDESC;

    if (interrupts & ISR_TXOK)
        regs.isr &= ~ISR_TXOK;

    if (interrupts & ISR_RXORN)
        regs.isr &= ~ISR_RXORN;

    if (interrupts & ISR_RXIDLE)
        regs.isr &= ~ISR_RXIDLE;

//ISR_RXEARLY not implemented

    if (interrupts & ISR_RXERR)
        regs.isr &= ~ISR_RXERR;

    if (interrupts & ISR_RXOK)
        regs.isr &= ~ISR_RXOK;

    if ((regs.isr & regs.imr))
        cpuIntrPost();

    if (!(regs.isr & regs.imr))
        cpuIntrClear();
}

void
EtherDev::devIntrChangeMask()
{
    DPRINTF(Ethernet, "iterrupt mask changed\n");

    if (regs.isr & regs.imr)
        cpuIntrPost();
    else
        cpuIntrClear();
}

void
EtherDev::cpuIntrPost()
{
    if (!cpuPendingIntr) {
        if (regs.ier) {
            cpuPendingIntr = true;
            intctrl->post(TheISA::INTLEVEL_IRQ1, TheISA::INTINDEX_ETHERNET);
        }
    }
}

void
EtherDev::cpuIntrClear()
{
    if (cpuPendingIntr) {
        cpuPendingIntr = false;
        intctrl->clear(TheISA::INTLEVEL_IRQ1, TheISA::INTINDEX_ETHERNET);
    }
}

bool
EtherDev::cpuIntrPending() const
{ return cpuPendingIntr; }

void
EtherDev::txReset()
{

    DPRINTF(Ethernet, "transmit reset\n");

    txPacketFlag = false;
    CTDD = false;
    txFifoCnt = 0;
    txFifoAvail = 0;
    txHalt = false;
    txFifo.clear();
    descAddrFifo.clear();
    regs.command &= ~CR_TXE;
    txState = txIdle;
}

void
EtherDev::rxReset()
{
    DPRINTF(Ethernet, "receive reset\n");

    rxPacketFlag = false;
    CRDD = false;
    fragLen = 0;
    rxFifoCnt = 0;
    rxHalt = false;
    rxFifo.clear();
    regs.command &= ~CR_RXE;
    rxState = rxIdle;
}

/**
 * This sets up a DMA transfer to read one data segment from the rxFifo into
 * the buffer indicated by rxDescCache.bufptr. Assumes the value of rxFragPtr
 * is already correctly set.
 */
void
EtherDev::writeOneFrag()
{
    /* i think there is no need for an "in use" warning here like in old */
    fragLen = rxFifo.front()->length; //length of whole packet
    fragLen = (fragLen < rxDescCnt) ? fragLen : rxDescCnt;

    writePhys.addr = rxFragPtr;
    writePhys.length = fragLen;

    // Set up DMA request area
    writeRequest.init(&rxDoneCB, 0, false, &writePhys, 1, fragLen,
                     rxDescBufPtr, fragLen, curTick);

    dma->doTransfer(&readRequest);
}

void
EtherDev::rxKick()
{
    DPRINTF(Ethernet, "receive state machine activated!\n");

    if (CRDD) {
        rxState = rxDescRefr;
        readOneDesc(rx, LINK_LEN);
    } else {
        rxState = rxDescRead;
        readOneDesc(rx);
    }
}

EtherDev::RxDescDone::RxDescDone(EtherDev *e)
    : ethernet(e)
{
}

std::string
EtherDev::RxDescDone::name() const
{
    return ethernet->name() + ".rxDescDoneCB";
}

void
EtherDev::RxDescDone::process()
{
    DPRINTF(Ethernet, "receive descriptor done callback\n");
    ethernet->rxDescDone();
}

void
EtherDev::rxDescDone()
{
    if (rxState == rxDescRefr) {
        if (rxDescCache.link == 0) {
            rxState = rxIdle;
            regs.command &= ~CR_RXE;
            devIntrPost(ISR_RXIDLE);
            return;
        } else {
            rxState = rxDescRead;
            regs.rxdp = rxDescCache.link;
            CRDD = false;
            readOneDesc(rx);
        }
    } else if (rxState == rxDescRead) {
        if (rxDescCache.cmdsts & CMDSTS_OWN) {
            rxState = rxIdle;
            regs.command &= ~CR_RXE;
            devIntrPost(ISR_RXIDLE);
        } else {
            rxState = rxFifoBlock;
            rxFragPtr = rxDescCache.bufptr;
            rxDescCnt = rxDescCache.cmdsts & CMDSTS_LEN_MASK;

            if (!rxFifo.empty()) {
                rxState = rxFragWrite;
                if (!rxPacketFlag) { // reading a new packet
                    rxPacketBufPtr = rxFifo.front()->data;
                    rxPacketBufPtr -= rxDescCnt;
                    rxDescBufPtr = rxPacketBufPtr;
                    rxCopied = 0;
                } else {
                    rxDescBufPtr = rxPacketBufPtr - rxDescCnt;
                }
                writeOneFrag();
            }
        }
    } else if (rxState == rxDescWrite) {
        devIntrPost(ISR_RXOK);

        if (rxDescCache.cmdsts & CMDSTS_INTR)
            devIntrPost(ISR_RXDESC);

        if (rxDescCache.link == 0 || ((rxPktBytes != 0) && rxHalt)) {
            rxState = rxIdle;
            regs.command &= ~CR_RXE;
            devIntrPost(ISR_RXIDLE);
            rxHalt = false;
        } else {
            rxState = rxDescRead;
            regs.rxdp = rxDescCache.link;
            CRDD = false;
            readOneDesc(rx);
        }
    }
}

EtherDev::RxDone::RxDone(EtherDev *e)
    : ethernet(e)
{
}

std::string
EtherDev::RxDone::name() const
{
    return ethernet->name() + ".rxDoneCB";
}

void
EtherDev::RxDone::process()
{
    DPRINTF(Ethernet, "receive done callback\n");
    ethernet->rxDone();
}

void
EtherDev::rxDone()
{
    DPRINTF(Ethernet, "packet received to host memory\n");

    if (!rxDescCache.cmdsts & CMDSTS_OWN)
        panic("This descriptor is already owned by the driver!\n");

    rxState = rxFifoBlock;
    rxCopied += fragLen;
    rxFifoCnt -= fragLen;

    if (rxDescCnt) { /* there is still data left in the descriptor */
        rxState = rxFragWrite;
        rxDescBufPtr += fragLen;
        writeOneFrag();
    } else {
        rxState = rxDescWrite;
        if (rxPktBytes == 0) {  /* packet is done */
            rxDescCache.cmdsts |= CMDSTS_OWN;
            rxDescCache.cmdsts &= ~CMDSTS_MORE;
            rxDescCache.cmdsts |= CMDSTS_OK;
            rxDescCache.cmdsts += rxCopied;   //i.e. set CMDSTS_SIZE

            rxPacketFlag = false;
            if (rxFilterEnable) {
                rxDescCache.cmdsts &= ~CMDSTS_DEST_MASK;
                if (rxFifo.front()->IsUnicast())
                    rxDescCache.cmdsts |= CMDSTS_DEST_SELF;
                if (rxFifo.front()->IsMulticast())
                    rxDescCache.cmdsts |= CMDSTS_DEST_MULTI;
                if (rxFifo.front()->IsBroadcast())
                    rxDescCache.cmdsts |= CMDSTS_DEST_MASK;
            }

            PacketPtr &pkt = rxFifo.front();
            eth_header *eth = (eth_header *) pkt->data;
            if (eth->type == 0x800 && extstsEnable) {
                rxDescCache.extsts |= EXTSTS_IPPKT;
                if (!ipChecksum(pkt, false))
                    rxDescCache.extsts |= EXTSTS_IPERR;
                ip_header *ip = rxFifo.front()->getIpHdr();

                if (ip->protocol == 6) {
                    rxDescCache.extsts |= EXTSTS_TCPPKT;
                    if (!tcpChecksum(pkt, false))
                        rxDescCache.extsts |= EXTSTS_TCPERR;
                } else if (ip->protocol == 17) {
                    rxDescCache.extsts |= EXTSTS_UDPPKT;
                    if (!udpChecksum(pkt, false))
                        rxDescCache.extsts |= EXTSTS_UDPERR;
                }
            }

            rxFifo.front() = NULL;
            rxFifo.pop_front();
        } else { /* just the descriptor is done */
            rxDescCache.cmdsts |= CMDSTS_OWN;
            rxDescCache.cmdsts |= CMDSTS_MORE;
        }
        writeDescPhys.addr = regs.rxdp + LINK_LEN + BUFPTR_LEN;
        writeDescPhys.length = CMDSTS_LEN;

        writeDescRequest.init(&rxDescDoneCB, 0, true, &writeDescPhys, 1,
                              CMDSTS_LEN, (uint8_t *) &rxDescCache.cmdsts,
                              CMDSTS_LEN, curTick);
    }
}

/**
 * This sets up a DMA transfer to read one descriptor into the network device.
 */
void
EtherDev::readOneDesc(dir_t dir, uint32_t len) {
    readDescPhys.addr = (dir == tx) ? regs.txdp : regs.rxdp;
    readDescPhys.length = len;

    ns_desc *cache = (dir == tx) ? &txDescCache : &rxDescCache;

    /* THIS ASSUMES THAT DESC_LEN < regs.txcfg's maxdma value,
       which is 512 bytes in the driver, so i'll just hard code it here */
    readDescRequest.init(&txDescDoneCB, 0, false, &readDescPhys, 1,
                         len, (uint8_t *) cache , len, curTick);

    dma->doTransfer(&readDescRequest);
}

/**
 * This sets up a DMA transfer to read one data segment of the descriptor in
 * txDescCache.  Assumes the value of txFragPtr is already correctly set
 */
void
EtherDev::readOneFrag()
{
    /* i think there is no need for an "in use" warning here like in old */
    fragLen = (txDescCnt < txFifoAvail) ? txDescCnt : txFifoAvail;
    readPhys.addr = txFragPtr;
    readPhys.length = fragLen;

    // Set up DMA request area
    readRequest.init(&txDoneCB, 0, false, &readPhys, 1, fragLen,
                     txPacketBufPtr, fragLen, curTick);

    dma->doTransfer(&readRequest);
}

void
EtherDev::transmit()
{
    if (txFifo.empty()) {
        DPRINTF(Ethernet, "nothing to transmit\n");
        return;
    }

   if (interface->sendPacket(txFifo.front())) {
        DPRINTF(Ethernet, "transmit packet\n");
        txBytes += txFifo.front()->length;
        txPackets++;

        txFifoCnt -= txFifo.front()->length;

        txFifo.front() = NULL;
        txFifo.pop_front();

        txDescCache.cmdsts &= ~CMDSTS_OK;
    } else {
        txDescCache.cmdsts &= ~CMDSTS_ERR;
    }

    txDescCache.cmdsts &= ~CMDSTS_OWN;

    writeDescPhys.addr = descAddrFifo.front() + LINK_LEN + BUFPTR_LEN;
    writeDescPhys.length = CMDSTS_LEN;

    descAddrFifo.front() = 0;
    descAddrFifo.pop_front();

    writeDescRequest.init(&txDescDoneCB, 0, true, &writeDescPhys, 1,
                          writeDescPhys.length,
                          (uint8_t *) &(txDescCache.cmdsts),
                          writeDescPhys.length, curTick);

    dma->doTransfer(&writeDescRequest);

    transmit();
}

void
EtherDev::txKick()
{
    DPRINTF(Ethernet, "transmit state machine activated\n");
#if 0
    if (DTRACE(Ethernet))
        txDump();
#endif

    if (CTDD) {
        txState = txDescRefr;
        readOneDesc(tx, LINK_LEN);
    } else {
        txState = txDescRead;
        readOneDesc(tx);
    }
}

EtherDev::TxDescDone::TxDescDone(EtherDev *e)
    : ethernet(e)
{
}

std::string
EtherDev::TxDescDone::name() const
{
    return ethernet->name() + ".txDescDoneCB";
}

void
EtherDev::TxDescDone::process()
{
    DPRINTF(Ethernet, "transmit descriptor done callback\n");
    ethernet->txDescDone();

}

void
EtherDev::txDescDone()
{
    if (txState == txFifoBlock) {
        if (txDescCache.cmdsts & CMDSTS_OK) {
            devIntrPost(ISR_TXOK);
        } else if (txDescCache.cmdsts & CMDSTS_ERR) {
            devIntrPost(ISR_TXERR);
        }
    } else if (txState == txDescRefr || txState == txDescWrite) {

        if (txState == txDescWrite) {
            if (txDescCache.cmdsts & CMDSTS_INTR) {
                devIntrPost(ISR_TXDESC);
            }
        }

        if (txDescCache.link == 0) {
            txState = txIdle;
            regs.command &= ~CR_TXE;
            devIntrPost(ISR_TXIDLE);
            return;
        } else {
            txState = txDescRead;
            regs.txdp = txDescCache.link;
            CTDD = false;
            readOneDesc(tx);
        }
    } else if (txState == txDescRead) {
        if (txDescCache.cmdsts & CMDSTS_OWN) {
            txState = txFifoBlock;
            txFragPtr = txDescCache.bufptr;
            txDescCnt = txDescCache.cmdsts & CMDSTS_LEN_MASK;

            if (txFifoAvail >= ((regs.txcfg & TXCFG_FLTH_MASK) >> 8)) {
                txState = txFragRead;
                if (!txPacketFlag) {
                    txPacketFlag = true;
                    /* find the total length of this packet */
                    txPacketLen = txDescCnt;
                    bool more = txDescCache.cmdsts & CMDSTS_MORE;
                    uint8_t *addr = (uint8_t *) regs.txdp;
                    while (more) {
                        addr = physmem->dma_addr(((ns_desc *) addr)->link, sizeof(ns_desc));
                        /* !!!!!!mask needed? */
                        txPacketLen += ((ns_desc *)addr)->cmdsts & CMDSTS_LEN_MASK;
                        more = ((ns_desc *) addr)->cmdsts & CMDSTS_MORE;
                    }
                    PacketPtr &packet = txDoneCB.packet;
                    packet = new EtherPacket;
                    packet->length = txPacketLen;
                    packet->data = new uint8_t[txPacketLen];
                    txPacketBufPtr = packet->data;
                }
                readOneFrag();
            }
        } else {
            txState = txIdle;
            regs.command &= ~CR_TXE;
            devIntrPost(ISR_TXIDLE);
        }
    }
}

EtherDev::TxDone::TxDone(EtherDev *e)
    : ethernet(e)
{
}

std::string
EtherDev::TxDone::name() const
{
    return ethernet->name() + ".txDoneCB";
}


void
EtherDev::TxDone::process()
{
    DPRINTF(Ethernet, "transmit done callback\n");
    ethernet->txDone(packet);
}

void
EtherDev::txDone(PacketPtr packet)
{
    DPRINTF(Ethernet, "transmit done\n");

    if (!txDescCache.cmdsts & CMDSTS_OWN)
        panic("This descriptor is already owned by the driver!\n");

    txState = txFifoBlock;

    txPacketBufPtr += fragLen;  /* hope this ptr manipulation is right! */
    txDescCnt -= fragLen;
    txFifoCnt += fragLen;

    if (txFifoCnt >= (regs.txcfg & TXCFG_DRTH_MASK)) {
        if (txFifo.empty()) {
            txFifoCnt -= (uint32_t) (txPacketBufPtr - packet->data);
        } else {
            transmit();
        }
    }

    if (txDescCnt) { /* if there is still more data to go in this desc */
        if (txFifoAvail >= regs.txcfg & TXCFG_FLTH_MASK) {
            txState = txFragRead;
            readOneFrag();
        }
    } else { /* this descriptor is done */
        /* but there is more descriptors for this packet */
        if (txDescCache.cmdsts & CMDSTS_MORE) {
            txState = txDescWrite;
            txDescCache.cmdsts &= ~CMDSTS_OWN;
            writeDescPhys.addr = regs.txdp + LINK_LEN + BUFPTR_LEN;
            writeDescPhys.length = CMDSTS_LEN;

            writeDescRequest.init(&txDescDoneCB, 0, true, &writeDescPhys, 1,
                                  writeDescPhys.length,
                                  (uint8_t*) &txDescCache.cmdsts,
                                  writeDescPhys.length, curTick);
        } else { /* this packet is totally done */
            /* deal with the the packet that just finished */
            if (regs.vtcr & VTCR_PPCHK && extstsEnable) {
                if (txDescCache.extsts & EXTSTS_UDPPKT) {
                    udpChecksum(packet, true);
                } else if (txDescCache.extsts & EXTSTS_TCPPKT) {
                    tcpChecksum(packet, true);
                } else if (txDescCache.extsts & EXTSTS_IPPKT) {
                    ipChecksum(packet, true);
                }
            }

            txFifo.push_back(packet);
            transmit();
            txPacketFlag = false;
            descAddrFifo.push_back(regs.txdp);

            /* if there is not another descriptor ready for reading, go idle */
            if (txDescCache.link == 0 || txHalt) {
                txState = txIdle;
                devIntrPost(ISR_TXIDLE);
                txHalt = false;
            } else { /* else go read next descriptor */
                txState = txDescRead;
                regs.txdp = txDescCache.link;
                CTDD = false;
                readOneDesc(tx);
            }
        }
    }
}

void
EtherDev::transferDone()
{
    if (txFifo.empty())
        return;

    DPRINTF(Ethernet, "schedule transmit\n");

    if (txEvent.scheduled())
        txEvent.reschedule(curTick + 1);
    else
        txEvent.schedule(curTick + 1);
}

void
EtherDev::txDump() const
{
#if 0
    int i = tx_ptr;
    for (int loop = 0; loop < tx_ring_len; loop++) {
        es_desc *desc = &tx_ring[i];

        if (desc->addr)
            cprintf("desc[%d]: addr=%#x, len=%d, flags=%#x\n",
                    i, desc->addr, desc->length, desc->flags);

        if (++i >= tx_ring_len)
            i = 0;
    }
#endif
}

void
EtherDev::rxDump() const
{
#if 0
    int i = rx_ptr;
    for (int loop = 0; loop < rx_ring_len; loop++) {
        es_desc *desc = &rx_ring[i];

        if (desc->addr)
            cprintf("desc[%d]: addr=%#x, len=%d, flags=%#x\n",
                    i, desc->addr, desc->length, desc->flags);

        if (++i >= rx_ring_len)
            i = 0;
    }
#endif
}

bool
EtherDev::rxFilter(PacketPtr packet)
{
    bool drop = true;
    string type;

    if (packet->IsUnicast()) {
        type = "unicast";

        // If we're accepting all unicast addresses
        if (acceptUnicast)
            drop = false;

        // If we make a perfect match
        if ((acceptPerfect)
            && (memcmp(regs.perfectMatch, packet->data, sizeof(regs.perfectMatch)) == 0))
            drop = false;

        eth_header *eth = (eth_header *) packet->data;
        if ((acceptArp) && (eth->type == 0x806))
            drop = false;

    } else if (packet->IsBroadcast()) {
        type = "broadcast";

        // if we're accepting broadcasts
        if (acceptBroadcast)
            drop = false;

    } else if (packet->IsMulticast()) {
        type = "multicast";

        // if we're accepting all multicasts
        if (acceptMulticast)
            drop = false;

    } else {
        type = "unknown";

        // oh well, punt on this one
    }

    if (drop) {
        DPRINTF(Ethernet, "rxFilter drop\n");
        DDUMP(EthernetData, packet->data, packet->length);
    }

    return drop;
}

bool
EtherDev::recvPacket(PacketPtr packet)
{
    rxBytes += packet->length;
    rxPackets++;

    if (rxState == rxIdle) {
        DPRINTF(Ethernet, "receive disabled...packet dropped\n");
        interface->recvDone();
        return true;
    }

    if (rxFilterEnable && rxFilter(packet)) {
        DPRINTF(Ethernet, "packet filtered...dropped\n");
        interface->recvDone();
        return true;
    }

    if (rxFifoCnt + packet->length >= MAX_RX_FIFO_SIZE) {
        DPRINTF(Ethernet,
                "packet will not fit in receive buffer...packet dropped\n");
        devIntrPost(ISR_RXORN);
        return false;
    }

    rxFifo.push_back(packet);
    rxPktBytes = packet->length;
    rxFifoCnt += packet->length;
    interface->recvDone();

    return true;
}

bool
EtherDev::udpChecksum(PacketPtr packet, bool gen)
{
    udp_header *hdr = (udp_header *)  packet->getTransportHdr();

    ip_header *ip = packet->getIpHdr();

    pseudo_header *pseudo = new pseudo_header;

    pseudo->src_ip_addr = ip->src_ip_addr;
    pseudo->dest_ip_addr = ip->dest_ip_addr;
    pseudo->protocol = ip->protocol;
    pseudo->len = hdr->len;

    uint16_t cksum = checksumCalc((uint16_t *) pseudo, (uint16_t *) hdr,
                                  (uint32_t) hdr->len);

    delete pseudo;
    if (gen)
        hdr->chksum = cksum;
    else
        if (cksum != 0)
            return false;

    return true;
}

bool
EtherDev::tcpChecksum(PacketPtr packet, bool gen)
{
    tcp_header *hdr = (tcp_header *) packet->getTransportHdr();

    ip_header *ip = packet->getIpHdr();

    pseudo_header *pseudo = new pseudo_header;

    pseudo->src_ip_addr = ip->src_ip_addr;
    pseudo->dest_ip_addr = ip->dest_ip_addr;
    pseudo->protocol = ip->protocol;
    pseudo->len = ip->dgram_len - (ip->vers_len & 0xf);

    uint16_t cksum = checksumCalc((uint16_t *) pseudo, (uint16_t *) hdr,
                                  (uint32_t) pseudo->len);

    delete pseudo;
    if (gen)
        hdr->chksum = cksum;
    else
        if (cksum != 0)
            return false;

    return true;
}

bool
EtherDev::ipChecksum(PacketPtr packet, bool gen)
{
    ip_header *hdr = packet->getIpHdr();

    uint16_t cksum = checksumCalc(NULL, (uint16_t *) hdr, (hdr->vers_len & 0xf));

    if (gen)
        hdr->hdr_chksum = cksum;
    else
        if (cksum != 0)
            return false;

    return true;
}

uint16_t
EtherDev::checksumCalc(uint16_t *pseudo, uint16_t *buf, uint32_t len)
{
    uint32_t sum = 0;

    uint16_t last_pad = 0;
    if (len & 1) {
        last_pad = buf[len/2] & 0xff;
        len--;
        sum += last_pad;
    }

    if (pseudo) {
        sum = pseudo[0] + pseudo[1] + pseudo[2] +
            pseudo[3] + pseudo[4] + pseudo[5];
    }

    for (int i=0; i < (len/2); ++i) {
        sum += buf[i];
    }

    while (sum >> 16)
        sum = (sum >> 16) + (sum & 0xffff);

    return ~sum;
}

//=====================================================================
//
//
void
dp_regs::serialize(ostream &os)
{
    SERIALIZE_SCALAR(command);
    SERIALIZE_SCALAR(config);
    SERIALIZE_SCALAR(isr);
    SERIALIZE_SCALAR(imr);
}

void
dp_regs::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(command);
    UNSERIALIZE_SCALAR(config);
    UNSERIALIZE_SCALAR(isr);
    UNSERIALIZE_SCALAR(imr);
#if 0
    UNSERIALIZE_SCALAR(tx_ring);
    UNSERIALIZE_SCALAR(rx_ring);
    UNSERIALIZE_SCALAR(tx_ring_len);
    UNSERIALIZE_SCALAR(rx_ring_len);
    UNSERIALIZE_SCALAR(rom_addr);
    UNSERIALIZE_SCALAR(rom_data);
    UNSERIALIZE_SCALAR(rxfilt_ctl);
    UNSERIALIZE_SCALAR(rxfilt_data);

    UNSERIALIZE_ARRAY(perfect,EADDR_LEN);
    UNSERIALIZE_ARRAY(hash_table,ES_HASH_SIZE);

    UNSERIALIZE_SCALAR(tx_ring_ptr);
    UNSERIALIZE_SCALAR(rx_ring_ptr);
#endif
}

//---------------------------------------

void
EtherPacket::serialize(ostream &os)
{
    SERIALIZE_SCALAR(length);
    SERIALIZE_ARRAY(data, length);
}

void
EtherPacket::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(length);
    data = new uint8_t[length];
    UNSERIALIZE_ARRAY(data, length);
}

//---------------------------------------

void
EtherDev::serialize(ostream &os)
{

#if 0
    regs.serialize(os);

    // tx_ring & rx_ring are contained in the physmem...
    SERIALIZE_SCALAR(cpuPendingIntr);
    SERIALIZE_SCALAR(tx_ptr);
    SERIALIZE_SCALAR(rx_ptr);

    SERIALIZE_SCALAR(rxDoneCB.ptr);
    SERIALIZE_SCALAR(rxDoneCB.ignore);

    SERIALIZE_SCALAR(txDoneCB.ptr);
    SERIALIZE_SCALAR(txDoneCB.ignore);

    for (int i=0; i<ES_MAX_DMA_SEGS; ++i) {
        paramOut(os, csprintf("readPhys%d.addr",i),   readPhys[i].addr);
        paramOut(os, csprintf("readPhys%d.length",i), readPhys[i].length);
        paramOut(os, csprintf("writePhys%d.addr",i),   writePhys[i].addr);
        paramOut(os, csprintf("writePhys%d.length",i), writePhys[i].length);
    }

    SERIALIZE_SCALAR(txEnable);
    SERIALIZE_SCALAR(rxEnable);
    SERIALIZE_SCALAR(txDelay);
    SERIALIZE_SCALAR(rxDelay);

    SERIALIZE_SCALAR(txbuf_len);

    //Calculate the number here, actually dump them at end
    int numTxPkts=0;
    for (pktiter_t p=txbuf.begin(); p!=txbuf.end(); ++p) {
        numTxPkts++;
    }
    SERIALIZE_SCALAR(numTxPkts);

    SERIALIZE_SCALAR(rxbuf_len);
    int numRxPkts=0;
    for (pktiter_t p=rxbuf.begin(); p!=rxbuf.end(); ++p) {
        numRxPkts++;
    }
    SERIALIZE_SCALAR(numRxPkts);

    // output whether the tx and rx packets exist
    bool txPacketExists = false;
    if (txDoneCB.packet)
        txPacketExists = true;
    SERIALIZE_SCALAR(txPacketExists);

    bool rxPacketExists = false;
    if (rxPacket)
        rxPacketExists = true;
    SERIALIZE_SCALAR(rxPacketExists);

    // output the names (unique by pointer) of the read and write requests
    paramOut(os, csprintf("readReqName"), readRequest.name());
    paramOut(os, csprintf("writeReqName"), writeRequest.name());

    // Serialize txPacket, because its data is needed for readRequest
    if (txPacketExists) {
        nameOut(os, csprintf("%s.txPacket", name()));
        txDoneCB.packet->serialize(os);
    }

    // Serialize rxPacket, because its data is needed for writeRequest
    if (rxPacketExists) {
        nameOut(os, csprintf("%s.rxPacket", name()));
        rxPacket->serialize(os);
    }

    // create a section for the readRequest
    nameOut(os, readRequest.name());
    paramOut(os, csprintf("parent"), name());
    paramOut(os, csprintf("id"), 0);
    readRequest.serialize(os);

    // create a section for the writeRequest
    nameOut(os, writeRequest.name());
    paramOut(os, csprintf("parent"), name());
    paramOut(os, csprintf("id"), 1);
    writeRequest.serialize(os);

    //Redo the buffers, this time outputing them to the file
    numTxPkts = 0;
    for (pktiter_t p=txbuf.begin(); p!=txbuf.end(); ++p) {
        nameOut(os, csprintf("%s.txbuf%d", name(),numTxPkts++));
        (*p)->serialize(os);
    }

    numRxPkts = 0;
    for (pktiter_t p=rxbuf.begin(); p!=rxbuf.end(); ++p) {
        nameOut(os, csprintf("%s.rxbuf%d", name(),numRxPkts++));
        (*p)->serialize(os);
    }
#endif
}

void
EtherDev::unserialize(Checkpoint *cp, const std::string &section)
{
#if 0
    regs.unserialize(cp, section);

    UNSERIALIZE_SCALAR(cpuPendingIntr);

    // initialize the tx_ring
    txReset();

    // initialize the rx_ring
    rxReset();

    UNSERIALIZE_SCALAR(tx_ptr);
    UNSERIALIZE_SCALAR(rx_ptr);

    PacketPtr p;
    UNSERIALIZE_SCALAR(txbuf_len);
    int numTxPkts;
    UNSERIALIZE_SCALAR(numTxPkts);
    for (int i=0; i<numTxPkts; ++i) {
        p = new EtherPacket;
        p->unserialize(cp, csprintf("%s.txbuf%d", section, i));
        txbuf.push_back(p);
    }

    UNSERIALIZE_SCALAR(rxbuf_len);
    int numRxPkts;
    UNSERIALIZE_SCALAR(numRxPkts);
    for (int i=0; i<numRxPkts; ++i) {
        p = new EtherPacket;
        p->unserialize(cp, csprintf("%s.rxbuf%d", section, i));
        rxbuf.push_back(p);
    }

    UNSERIALIZE_SCALAR(rxDoneCB.ptr);
    UNSERIALIZE_SCALAR(rxDoneCB.ignore);

    UNSERIALIZE_SCALAR(txDoneCB.ptr);
    UNSERIALIZE_SCALAR(txDoneCB.ignore);

    for (int i=0; i<ES_MAX_DMA_SEGS; ++i) {
        paramIn(cp, section, csprintf("readPhys%d.addr",i),
                readPhys[i].addr);
        paramIn(cp, section, csprintf("readPhys%d.length",i),
                readPhys[i].length);
        paramIn(cp, section, csprintf("writePhys%d.addr",i),
                writePhys[i].addr);
        paramIn(cp, section, csprintf("writePhys%d.length",i),
                writePhys[i].length);
    }

    UNSERIALIZE_SCALAR(txEnable);
    UNSERIALIZE_SCALAR(rxEnable);
    UNSERIALIZE_SCALAR(txDelay);
    UNSERIALIZE_SCALAR(rxDelay);

    // Unserialize the current txPacket
    bool txPacketExists;
    UNSERIALIZE_SCALAR(txPacketExists);

    txDoneCB.packet = NULL;
    if (txPacketExists) {
        txDoneCB.packet = new EtherPacket;
        txDoneCB.packet->unserialize(cp, csprintf("%s.txPacket", section));
    }

    // Unserialize the current rxPacket
    bool rxPacketExists;
    UNSERIALIZE_SCALAR(rxPacketExists);

    rxPacket = NULL;
    if (rxPacketExists) {
        rxPacket = new EtherPacket;
        rxPacket->unserialize(cp, csprintf("%s.rxPacket", section));
    }

    std::string readReqName, writeReqName;
    UNSERIALIZE_SCALAR(readReqName);
    UNSERIALIZE_SCALAR(writeReqName);

    // Unserialize and fixup the readRequest
    readRequest.unserialize(cp, readReqName);
    readRequest.phys = readPhys;
    readRequest.bufferCB = 0;
    readRequest.dmaDoneCB = &txDoneCB;
    readRequest.data = NULL;

    if (txDoneCB.packet)
        readRequest.data = txDoneCB.packet->data;

    // Unserialize and fixup the writeRequest
    writeRequest.unserialize(cp, writeReqName);
    writeRequest.phys = writePhys;
    writeRequest.bufferCB = 0;
    writeRequest.dmaDoneCB = &rxDoneCB;
    writeRequest.data = NULL;

    if (rxPacket)
        writeRequest.data = rxPacket->data;
#endif
}


//=====================================================================


BEGIN_DECLARE_SIM_OBJECT_PARAMS(EtherDevInt)

    SimObjectParam<EtherInt *> peer;
    SimObjectParam<EtherDev *> device;

END_DECLARE_SIM_OBJECT_PARAMS(EtherDevInt)

BEGIN_INIT_SIM_OBJECT_PARAMS(EtherDevInt)

    INIT_PARAM_DFLT(peer, "peer interface", NULL),
    INIT_PARAM(device, "Ethernet device of this interface")

END_INIT_SIM_OBJECT_PARAMS(EtherDevInt)

CREATE_SIM_OBJECT(EtherDevInt)
{
    EtherDevInt *dev_int = new EtherDevInt(getInstanceName(), device);

    EtherInt *p = (EtherInt *)peer;
    if (p) {
        dev_int->setPeer(p);
        p->setPeer(dev_int);
    }

    return dev_int;
}

REGISTER_SIM_OBJECT("EtherDevInt", EtherDevInt)


BEGIN_DECLARE_SIM_OBJECT_PARAMS(EtherDev)

    Param<Tick> tx_delay;
    Param<Tick> rx_delay;
    SimObjectParam<DmaEngine *> engine;
    Param<bool> use_interface;
    SimObjectParam<IntrControl *> intr_ctrl;
    SimObjectParam<MemoryController *> mmu;
    SimObjectParam<PhysicalMemory *> physmem;
    Param<Addr> addr;
    Param<Addr> mask;
    Param<bool> rx_filter;
    Param<string> hardware_address;
    SimObjectParam<PCIConfigAll *> configspace;
    SimObjectParam<PciConfigData *> configdata;
    SimObjectParam<Tsunami *> tsunami;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;

END_DECLARE_SIM_OBJECT_PARAMS(EtherDev)

BEGIN_INIT_SIM_OBJECT_PARAMS(EtherDev)

    INIT_PARAM_DFLT(tx_delay, "Transmit Delay", 1000),
    INIT_PARAM_DFLT(rx_delay, "Receive Delay", 1000),
    INIT_PARAM(engine, "DMA Engine"),
    INIT_PARAM_DFLT(use_interface, "Use DMA Interface", true),
    INIT_PARAM(intr_ctrl, "Interrupt Controller"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(physmem, "Physical Memory"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask"),
    INIT_PARAM_DFLT(rx_filter, "Enable Receive Filter", true),
    INIT_PARAM_DFLT(hardware_address, "Ethernet Hardware Address",
                    "00:99:00:00:00:01"),
    INIT_PARAM(configspace, "PCI Configspace"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(pci_bus, "PCI bus"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code")

END_INIT_SIM_OBJECT_PARAMS(EtherDev)


CREATE_SIM_OBJECT(EtherDev)
{
    int eaddr[6];
    sscanf(((string)hardware_address).c_str(), "%x:%x:%x:%x:%x:%x",
           &eaddr[0], &eaddr[1], &eaddr[2], &eaddr[3], &eaddr[4], &eaddr[5]);

    return new EtherDev(getInstanceName(), engine, use_interface,
                        intr_ctrl, mmu, physmem, configspace, configdata,
                        tsunami, pci_bus, pci_dev, pci_func, rx_filter, eaddr,
                        tx_delay, rx_delay, addr, mask);
}

REGISTER_SIM_OBJECT("EtherDev", EtherDev)

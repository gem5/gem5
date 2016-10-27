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
 * DP83820 ethernet controller.  Does not support priority queueing
 */

#include "dev/net/ns_gige.hh"

#include <deque>
#include <memory>
#include <string>

#include "base/debug.hh"
#include "base/inet.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "debug/EthernetAll.hh"
#include "dev/net/etherlink.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/NSGigE.hh"
#include "sim/system.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
using std::make_shared;
using std::min;
using std::ostream;
using std::string;

const char *NsRxStateStrings[] =
{
    "rxIdle",
    "rxDescRefr",
    "rxDescRead",
    "rxFifoBlock",
    "rxFragWrite",
    "rxDescWrite",
    "rxAdvance"
};

const char *NsTxStateStrings[] =
{
    "txIdle",
    "txDescRefr",
    "txDescRead",
    "txFifoBlock",
    "txFragRead",
    "txDescWrite",
    "txAdvance"
};

const char *NsDmaState[] =
{
    "dmaIdle",
    "dmaReading",
    "dmaWriting",
    "dmaReadWaiting",
    "dmaWriteWaiting"
};

using namespace Net;
using namespace TheISA;

///////////////////////////////////////////////////////////////////////
//
// NSGigE PCI Device
//
NSGigE::NSGigE(Params *p)
    : EtherDevBase(p), ioEnable(false),
      txFifo(p->tx_fifo_size), rxFifo(p->rx_fifo_size),
      txPacket(0), rxPacket(0), txPacketBufPtr(NULL), rxPacketBufPtr(NULL),
      txXferLen(0), rxXferLen(0), rxDmaFree(false), txDmaFree(false),
      txState(txIdle), txEnable(false), CTDD(false), txHalt(false),
      txFragPtr(0), txDescCnt(0), txDmaState(dmaIdle), rxState(rxIdle),
      rxEnable(false), CRDD(false), rxPktBytes(0), rxHalt(false),
      rxFragPtr(0), rxDescCnt(0), rxDmaState(dmaIdle), extstsEnable(false),
      eepromState(eepromStart), eepromClk(false), eepromBitsToRx(0),
      eepromOpcode(0), eepromAddress(0), eepromData(0),
      dmaReadDelay(p->dma_read_delay), dmaWriteDelay(p->dma_write_delay),
      dmaReadFactor(p->dma_read_factor), dmaWriteFactor(p->dma_write_factor),
      rxDmaData(NULL), rxDmaAddr(0), rxDmaLen(0),
      txDmaData(NULL), txDmaAddr(0), txDmaLen(0),
      rxDmaReadEvent(this), rxDmaWriteEvent(this),
      txDmaReadEvent(this), txDmaWriteEvent(this),
      dmaDescFree(p->dma_desc_free), dmaDataFree(p->dma_data_free),
      txDelay(p->tx_delay), rxDelay(p->rx_delay),
      rxKickTick(0), rxKickEvent(this), txKickTick(0), txKickEvent(this),
      txEvent(this), rxFilterEnable(p->rx_filter),
      acceptBroadcast(false), acceptMulticast(false), acceptUnicast(false),
      acceptPerfect(false), acceptArp(false), multicastHashEnable(false),
      intrDelay(p->intr_delay), intrTick(0), cpuPendingIntr(false),
      intrEvent(0), interface(0)
{


    interface = new NSGigEInt(name() + ".int0", this);

    regsReset();
    memcpy(&rom.perfectMatch, p->hardware_address.bytes(), ETH_ADDR_LEN);

    memset(&rxDesc32, 0, sizeof(rxDesc32));
    memset(&txDesc32, 0, sizeof(txDesc32));
    memset(&rxDesc64, 0, sizeof(rxDesc64));
    memset(&txDesc64, 0, sizeof(txDesc64));
}

NSGigE::~NSGigE()
{
    delete interface;
}

/**
 * This is to write to the PCI general configuration registers
 */
Tick
NSGigE::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC)
        PciDevice::writeConfig(pkt);
    else
        panic("Device specific PCI config space not implemented!\n");

    switch (offset) {
        // seems to work fine without all these PCI settings, but i
        // put in the IO to double check, an assertion will fail if we
        // need to properly implement it
      case PCI_COMMAND:
        if (config.data[offset] & PCI_CMD_IOSE)
            ioEnable = true;
        else
            ioEnable = false;
        break;
    }

    return configDelay;
}

EtherInt*
NSGigE::getEthPort(const std::string &if_name, int idx)
{
    if (if_name == "interface") {
       if (interface->getPeer())
           panic("interface already connected to\n");
       return interface;
    }
    return NULL;
}

/**
 * This reads the device registers, which are detailed in the NS83820
 * spec sheet
 */
Tick
NSGigE::read(PacketPtr pkt)
{
    assert(ioEnable);

    //The mask is to give you only the offset into the device register file
    Addr daddr = pkt->getAddr() & 0xfff;
    DPRINTF(EthernetPIO, "read  da=%#x pa=%#x size=%d\n",
            daddr, pkt->getAddr(), pkt->getSize());


    // there are some reserved registers, you can see ns_gige_reg.h and
    // the spec sheet for details
    if (daddr > LAST && daddr <=  RESERVED) {
        panic("Accessing reserved register");
    } else if (daddr > RESERVED && daddr <= 0x3FC) {
        return readConfig(pkt);
    } else if (daddr >= MIB_START && daddr <= MIB_END) {
        // don't implement all the MIB's.  hopefully the kernel
        // doesn't actually DEPEND upon their values
        // MIB are just hardware stats keepers
        pkt->set<uint32_t>(0);
        pkt->makeAtomicResponse();
        return pioDelay;
    } else if (daddr > 0x3FC)
        panic("Something is messed up!\n");

    assert(pkt->getSize() == sizeof(uint32_t));
        uint32_t &reg = *pkt->getPtr<uint32_t>();
        uint16_t rfaddr;

        switch (daddr) {
          case CR:
            reg = regs.command;
            //these are supposed to be cleared on a read
            reg &= ~(CR_RXD | CR_TXD | CR_TXR | CR_RXR);
            break;

          case CFGR:
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
            devIntrClear(ISR_ALL);
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

          case TX_CFG:
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

          case RX_CFG:
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

            // see the spec sheet for how RFCR and RFDR work
            // basically, you write to RFCR to tell the machine
            // what you want to do next, then you act upon RFDR,
            // and the device will be prepared b/c of what you
            // wrote to RFCR
          case RFCR:
            reg = regs.rfcr;
            break;

          case RFDR:
            rfaddr = (uint16_t)(regs.rfcr & RFCR_RFADDR);
            switch (rfaddr) {
              // Read from perfect match ROM octets
              case 0x000:
                reg = rom.perfectMatch[1];
                reg = reg << 8;
                reg += rom.perfectMatch[0];
                break;
              case 0x002:
                reg = rom.perfectMatch[3] << 8;
                reg += rom.perfectMatch[2];
                break;
              case 0x004:
                reg = rom.perfectMatch[5] << 8;
                reg += rom.perfectMatch[4];
                break;
              default:
                // Read filter hash table
                if (rfaddr >= FHASH_ADDR &&
                    rfaddr < FHASH_ADDR + FHASH_SIZE) {

                    // Only word-aligned reads supported
                    if (rfaddr % 2)
                        panic("unaligned read from filter hash table!");

                    reg = rom.filterHash[rfaddr - FHASH_ADDR + 1] << 8;
                    reg += rom.filterHash[rfaddr - FHASH_ADDR];
                    break;
                }

                panic("reading RFDR for something other than pattern"
                      " matching or hashing! %#x\n", rfaddr);
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

          case M5REG:
            reg = 0;
            if (params()->rx_thread)
                reg |= M5REG_RX_THREAD;
            if (params()->tx_thread)
                reg |= M5REG_TX_THREAD;
            if (params()->rss)
                reg |= M5REG_RSS;
            break;

          default:
            panic("reading unimplemented register: addr=%#x", daddr);
        }

        DPRINTF(EthernetPIO, "read from %#x: data=%d data=%#x\n",
                daddr, reg, reg);

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
NSGigE::write(PacketPtr pkt)
{
    assert(ioEnable);

    Addr daddr = pkt->getAddr() & 0xfff;
    DPRINTF(EthernetPIO, "write da=%#x pa=%#x size=%d\n",
            daddr, pkt->getAddr(), pkt->getSize());

    if (daddr > LAST && daddr <=  RESERVED) {
        panic("Accessing reserved register");
    } else if (daddr > RESERVED && daddr <= 0x3FC) {
        return writeConfig(pkt);
    } else if (daddr > 0x3FC)
        panic("Something is messed up!\n");

    if (pkt->getSize() == sizeof(uint32_t)) {
        uint32_t reg = pkt->get<uint32_t>();
        uint16_t rfaddr;

        DPRINTF(EthernetPIO, "write data=%d data=%#x\n", reg, reg);

        switch (daddr) {
          case CR:
            regs.command = reg;
            if (reg & CR_TXD) {
                txEnable = false;
            } else if (reg & CR_TXE) {
                txEnable = true;

                // the kernel is enabling the transmit machine
                if (txState == txIdle)
                    txKick();
            }

            if (reg & CR_RXD) {
                rxEnable = false;
            } else if (reg & CR_RXE) {
                rxEnable = true;

                if (rxState == rxIdle)
                    rxKick();
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

          case CFGR:
            if (reg & CFGR_LNKSTS ||
                reg & CFGR_SPDSTS ||
                reg & CFGR_DUPSTS ||
                reg & CFGR_RESERVED ||
                reg & CFGR_T64ADDR ||
                reg & CFGR_PCI64_DET) {
                // First clear all writable bits
                regs.config &= CFGR_LNKSTS | CFGR_SPDSTS | CFGR_DUPSTS |
                    CFGR_RESERVED | CFGR_T64ADDR |
                    CFGR_PCI64_DET;
                // Now set the appropriate writable bits
                regs.config |= reg & ~(CFGR_LNKSTS | CFGR_SPDSTS | CFGR_DUPSTS |
                                       CFGR_RESERVED | CFGR_T64ADDR |
                                       CFGR_PCI64_DET);
            }

// all these #if 0's are because i don't THINK the kernel needs to
// have these implemented. if there is a problem relating to one of
// these, you may need to add functionality in.

// grouped together and #if 0'ed to avoid empty if body and make clang happy
#if 0
            if (reg & CFGR_TBI_EN) ;
            if (reg & CFGR_MODE_1000) ;

            if (reg & CFGR_PINT_DUPSTS ||
                reg & CFGR_PINT_LNKSTS ||
                reg & CFGR_PINT_SPDSTS)
                ;

            if (reg & CFGR_TMRTEST) ;
            if (reg & CFGR_MRM_DIS) ;
            if (reg & CFGR_MWI_DIS) ;

            if (reg & CFGR_DATA64_EN) ;
            if (reg & CFGR_M64ADDR) ;
            if (reg & CFGR_PHY_RST) ;
            if (reg & CFGR_PHY_DIS) ;

            if (reg & CFGR_REQALG) ;
            if (reg & CFGR_SB) ;
            if (reg & CFGR_POW) ;
            if (reg & CFGR_EXD) ;
            if (reg & CFGR_PESEL) ;
            if (reg & CFGR_BROM_DIS) ;
            if (reg & CFGR_EXT_125) ;
            if (reg & CFGR_BEM) ;

            if (reg & CFGR_T64ADDR) ;
            // panic("CFGR_T64ADDR is read only register!\n");
#endif
            if (reg & CFGR_AUTO_1000)
                panic("CFGR_AUTO_1000 not implemented!\n");

            if (reg & CFGR_PCI64_DET)
                panic("CFGR_PCI64_DET is read only register!\n");

            if (reg & CFGR_EXTSTS_EN)
                extstsEnable = true;
            else
                extstsEnable = false;
            break;

          case MEAR:
            // Clear writable bits
            regs.mear &= MEAR_EEDO;
            // Set appropriate writable bits
            regs.mear |= reg & ~MEAR_EEDO;

            // FreeBSD uses the EEPROM to read PMATCH (for the MAC address)
            // even though it could get it through RFDR
            if (reg & MEAR_EESEL) {
                // Rising edge of clock
                if (reg & MEAR_EECLK && !eepromClk)
                    eepromKick();
            }
            else {
                eepromState = eepromStart;
                regs.mear &= ~MEAR_EEDI;
            }

            eepromClk = reg & MEAR_EECLK;

            // since phy is completely faked, MEAR_MD* don't matter

// grouped together and #if 0'ed to avoid empty if body and make clang happy
#if 0
            if (reg & MEAR_MDIO) ;
            if (reg & MEAR_MDDIR) ;
            if (reg & MEAR_MDC) ;
#endif
            break;

          case PTSCR:
            regs.ptscr = reg & ~(PTSCR_RBIST_RDONLY);
            // these control BISTs for various parts of chip - we
            // don't care or do just fake that the BIST is done
            if (reg & PTSCR_RBIST_EN)
                regs.ptscr |= PTSCR_RBIST_DONE;
            if (reg & PTSCR_EEBIST_EN)
                regs.ptscr &= ~PTSCR_EEBIST_EN;
            if (reg & PTSCR_EELOAD_EN)
                regs.ptscr &= ~PTSCR_EELOAD_EN;
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

          case TX_CFG:
            regs.txcfg = reg;
#if 0
            if (reg & TX_CFG_CSI) ;
            if (reg & TX_CFG_HBI) ;
            if (reg & TX_CFG_MLB) ;
            if (reg & TX_CFG_ATP) ;
            if (reg & TX_CFG_ECRETRY) {
                /*
                 * this could easily be implemented, but considering
                 * the network is just a fake pipe, wouldn't make
                 * sense to do this
                 */
            }

            if (reg & TX_CFG_BRST_DIS) ;
#endif

#if 0
            /* we handle our own DMA, ignore the kernel's exhortations */
            if (reg & TX_CFG_MXDMA) ;
#endif

            // also, we currently don't care about fill/drain
            // thresholds though this may change in the future with
            // more realistic networks or a driver which changes it
            // according to feedback

            break;

          case GPIOR:
            // Only write writable bits
            regs.gpior &= GPIOR_UNUSED | GPIOR_GP5_IN | GPIOR_GP4_IN
                        | GPIOR_GP3_IN | GPIOR_GP2_IN | GPIOR_GP1_IN;
            regs.gpior |= reg & ~(GPIOR_UNUSED | GPIOR_GP5_IN | GPIOR_GP4_IN
                                | GPIOR_GP3_IN | GPIOR_GP2_IN | GPIOR_GP1_IN);
            /* these just control general purpose i/o pins, don't matter */
            break;

          case RXDP:
            regs.rxdp = reg;
            CRDD = false;
            break;

          case RXDP_HI:
            regs.rxdp_hi = reg;
            break;

          case RX_CFG:
            regs.rxcfg = reg;
#if 0
            if (reg & RX_CFG_AEP) ;
            if (reg & RX_CFG_ARP) ;
            if (reg & RX_CFG_STRIPCRC) ;
            if (reg & RX_CFG_RX_RD) ;
            if (reg & RX_CFG_ALP) ;
            if (reg & RX_CFG_AIRL) ;

            /* we handle our own DMA, ignore what kernel says about it */
            if (reg & RX_CFG_MXDMA) ;

            //also, we currently don't care about fill/drain thresholds
            //though this may change in the future with more realistic
            //networks or a driver which changes it according to feedback
            if (reg & (RX_CFG_DRTH | RX_CFG_DRTH0)) ;
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
            multicastHashEnable = (reg & RFCR_MHEN) ? true : false;

#if 0
            if (reg & RFCR_APAT)
                panic("RFCR_APAT not implemented!\n");
#endif
            if (reg & RFCR_UHEN)
                panic("Unicast hash filtering not used by drivers!\n");

            if (reg & RFCR_ULM)
                panic("RFCR_ULM not implemented!\n");

            break;

          case RFDR:
            rfaddr = (uint16_t)(regs.rfcr & RFCR_RFADDR);
            switch (rfaddr) {
              case 0x000:
                rom.perfectMatch[0] = (uint8_t)reg;
                rom.perfectMatch[1] = (uint8_t)(reg >> 8);
                break;
              case 0x002:
                rom.perfectMatch[2] = (uint8_t)reg;
                rom.perfectMatch[3] = (uint8_t)(reg >> 8);
                break;
              case 0x004:
                rom.perfectMatch[4] = (uint8_t)reg;
                rom.perfectMatch[5] = (uint8_t)(reg >> 8);
                break;
              default:

                if (rfaddr >= FHASH_ADDR &&
                    rfaddr < FHASH_ADDR + FHASH_SIZE) {

                    // Only word-aligned writes supported
                    if (rfaddr % 2)
                        panic("unaligned write to filter hash table!");

                    rom.filterHash[rfaddr - FHASH_ADDR] = (uint8_t)reg;
                    rom.filterHash[rfaddr - FHASH_ADDR + 1]
                        = (uint8_t)(reg >> 8);
                    break;
                }
                panic("writing RFDR for something other than pattern matching "
                    "or hashing! %#x\n", rfaddr);
            }

          case BRAR:
            regs.brar = reg;
            break;

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
            // Only write the writable bits
            regs.tanar &= TANAR_RF1 | TANAR_RF2 | TANAR_UNUSED;
            regs.tanar |= reg & ~(TANAR_RF1 | TANAR_RF2 | TANAR_UNUSED);

            // Pause capability unimplemented
#if 0
            if (reg & TANAR_PS2) ;
            if (reg & TANAR_PS1) ;
#endif

            break;

          case TANLPAR:
            panic("this should only be written to by the fake phy!\n");

          case TANER:
            panic("TANER is read only register!\n");

          case TESR:
            regs.tesr = reg;
            break;

          default:
            panic("invalid register access daddr=%#x", daddr);
        }
    } else {
        panic("Invalid Request Size");
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
NSGigE::devIntrPost(uint32_t interrupts)
{
    if (interrupts & ISR_RESERVE)
        panic("Cannot set a reserved interrupt");

    if (interrupts & ISR_NOIMPL)
        warn("interrupt not implemented %#x\n", interrupts);

    interrupts &= ISR_IMPL;
    regs.isr |= interrupts;

    if (interrupts & regs.imr) {
        if (interrupts & ISR_SWI) {
            totalSwi++;
        }
        if (interrupts & ISR_RXIDLE) {
            totalRxIdle++;
        }
        if (interrupts & ISR_RXOK) {
            totalRxOk++;
        }
        if (interrupts & ISR_RXDESC) {
            totalRxDesc++;
        }
        if (interrupts & ISR_TXOK) {
            totalTxOk++;
        }
        if (interrupts & ISR_TXIDLE) {
            totalTxIdle++;
        }
        if (interrupts & ISR_TXDESC) {
            totalTxDesc++;
        }
        if (interrupts & ISR_RXORN) {
            totalRxOrn++;
        }
    }

    DPRINTF(EthernetIntr,
            "interrupt written to ISR: intr=%#x isr=%#x imr=%#x\n",
            interrupts, regs.isr, regs.imr);

    if ((regs.isr & regs.imr)) {
        Tick when = curTick();
        if ((regs.isr & regs.imr & ISR_NODELAY) == 0)
            when += intrDelay;
        postedInterrupts++;
        cpuIntrPost(when);
    }
}

/* writing this interrupt counting stats inside this means that this function
   is now limited to being used to clear all interrupts upon the kernel
   reading isr and servicing.  just telling you in case you were thinking
   of expanding use.
*/
void
NSGigE::devIntrClear(uint32_t interrupts)
{
    if (interrupts & ISR_RESERVE)
        panic("Cannot clear a reserved interrupt");

    if (regs.isr & regs.imr & ISR_SWI) {
        postedSwi++;
    }
    if (regs.isr & regs.imr & ISR_RXIDLE) {
        postedRxIdle++;
    }
    if (regs.isr & regs.imr & ISR_RXOK) {
        postedRxOk++;
    }
    if (regs.isr & regs.imr & ISR_RXDESC) {
            postedRxDesc++;
    }
    if (regs.isr & regs.imr & ISR_TXOK) {
        postedTxOk++;
    }
    if (regs.isr & regs.imr & ISR_TXIDLE) {
        postedTxIdle++;
    }
    if (regs.isr & regs.imr & ISR_TXDESC) {
        postedTxDesc++;
    }
    if (regs.isr & regs.imr & ISR_RXORN) {
        postedRxOrn++;
    }

    interrupts &= ~ISR_NOIMPL;
    regs.isr &= ~interrupts;

    DPRINTF(EthernetIntr,
            "interrupt cleared from ISR: intr=%x isr=%x imr=%x\n",
            interrupts, regs.isr, regs.imr);

    if (!(regs.isr & regs.imr))
        cpuIntrClear();
}

void
NSGigE::devIntrChangeMask()
{
    DPRINTF(EthernetIntr, "interrupt mask changed: isr=%x imr=%x masked=%x\n",
            regs.isr, regs.imr, regs.isr & regs.imr);

    if (regs.isr & regs.imr)
        cpuIntrPost(curTick());
    else
        cpuIntrClear();
}

void
NSGigE::cpuIntrPost(Tick when)
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
    intrEvent = new IntrEvent(this, true);
    schedule(intrEvent, intrTick);
}

void
NSGigE::cpuInterrupt()
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
NSGigE::cpuIntrClear()
{
    if (!cpuPendingIntr)
        return;

    if (intrEvent) {
        intrEvent->squash();
        intrEvent = 0;
    }

    intrTick = 0;

    cpuPendingIntr = false;

    DPRINTF(EthernetIntr, "clearing interrupt\n");
    intrClear();
}

bool
NSGigE::cpuIntrPending() const
{ return cpuPendingIntr; }

void
NSGigE::txReset()
{

    DPRINTF(Ethernet, "transmit reset\n");

    CTDD = false;
    txEnable = false;;
    txFragPtr = 0;
    assert(txDescCnt == 0);
    txFifo.clear();
    txState = txIdle;
    assert(txDmaState == dmaIdle);
}

void
NSGigE::rxReset()
{
    DPRINTF(Ethernet, "receive reset\n");

    CRDD = false;
    assert(rxPktBytes == 0);
    rxEnable = false;
    rxFragPtr = 0;
    assert(rxDescCnt == 0);
    assert(rxDmaState == dmaIdle);
    rxFifo.clear();
    rxState = rxIdle;
}

void
NSGigE::regsReset()
{
    memset(&regs, 0, sizeof(regs));
    regs.config = (CFGR_LNKSTS | CFGR_TBI_EN | CFGR_MODE_1000);
    regs.mear = 0x12;
    regs.txcfg = 0x120; // set drain threshold to 1024 bytes and
                        // fill threshold to 32 bytes
    regs.rxcfg = 0x4;   // set drain threshold to 16 bytes
    regs.srr = 0x0103;  // set the silicon revision to rev B or 0x103
    regs.mibc = MIBC_FRZ;
    regs.vdr = 0x81;    // set the vlan tag type to 802.1q
    regs.tesr = 0xc000; // TBI capable of both full and half duplex
    regs.brar = 0xffffffff;

    extstsEnable = false;
    acceptBroadcast = false;
    acceptMulticast = false;
    acceptUnicast = false;
    acceptPerfect = false;
    acceptArp = false;
}

bool
NSGigE::doRxDmaRead()
{
    assert(rxDmaState == dmaIdle || rxDmaState == dmaReadWaiting);
    rxDmaState = dmaReading;

    if (dmaPending() || drainState() != DrainState::Running)
        rxDmaState = dmaReadWaiting;
    else
        dmaRead(rxDmaAddr, rxDmaLen, &rxDmaReadEvent, (uint8_t*)rxDmaData);

    return true;
}

void
NSGigE::rxDmaReadDone()
{
    assert(rxDmaState == dmaReading);
    rxDmaState = dmaIdle;

    DPRINTF(EthernetDMA, "rx dma read  paddr=%#x len=%d\n",
            rxDmaAddr, rxDmaLen);
    DDUMP(EthernetDMA, rxDmaData, rxDmaLen);

    // If the transmit state machine has a pending DMA, let it go first
    if (txDmaState == dmaReadWaiting || txDmaState == dmaWriteWaiting)
        txKick();

    rxKick();
}

bool
NSGigE::doRxDmaWrite()
{
    assert(rxDmaState == dmaIdle || rxDmaState == dmaWriteWaiting);
    rxDmaState = dmaWriting;

    if (dmaPending() || drainState() != DrainState::Running)
        rxDmaState = dmaWriteWaiting;
    else
        dmaWrite(rxDmaAddr, rxDmaLen, &rxDmaWriteEvent, (uint8_t*)rxDmaData);
    return true;
}

void
NSGigE::rxDmaWriteDone()
{
    assert(rxDmaState == dmaWriting);
    rxDmaState = dmaIdle;

    DPRINTF(EthernetDMA, "rx dma write paddr=%#x len=%d\n",
            rxDmaAddr, rxDmaLen);
    DDUMP(EthernetDMA, rxDmaData, rxDmaLen);

    // If the transmit state machine has a pending DMA, let it go first
    if (txDmaState == dmaReadWaiting || txDmaState == dmaWriteWaiting)
        txKick();

    rxKick();
}

void
NSGigE::rxKick()
{
    bool is64bit = (bool)(regs.config & CFGR_M64ADDR);

    DPRINTF(EthernetSM,
            "receive kick rxState=%s (rxBuf.size=%d) %d-bit\n",
            NsRxStateStrings[rxState], rxFifo.size(), is64bit ? 64 : 32);

    Addr link, bufptr;
    uint32_t &cmdsts = is64bit ? rxDesc64.cmdsts : rxDesc32.cmdsts;
    uint32_t &extsts = is64bit ? rxDesc64.extsts : rxDesc32.extsts;

  next:
    if (rxKickTick > curTick()) {
        DPRINTF(EthernetSM, "receive kick exiting, can't run till %d\n",
                rxKickTick);

        goto exit;
    }

    // Go to the next state machine clock tick.
    rxKickTick = clockEdge(Cycles(1));

    switch(rxDmaState) {
      case dmaReadWaiting:
        if (doRxDmaRead())
            goto exit;
        break;
      case dmaWriteWaiting:
        if (doRxDmaWrite())
            goto exit;
        break;
      default:
        break;
    }

    link = is64bit ? (Addr)rxDesc64.link : (Addr)rxDesc32.link;
    bufptr = is64bit ? (Addr)rxDesc64.bufptr : (Addr)rxDesc32.bufptr;

    // see state machine from spec for details
    // the way this works is, if you finish work on one state and can
    // go directly to another, you do that through jumping to the
    // label "next".  however, if you have intermediate work, like DMA
    // so that you can't go to the next state yet, you go to exit and
    // exit the loop.  however, when the DMA is done it will trigger
    // an event and come back to this loop.
    switch (rxState) {
      case rxIdle:
        if (!rxEnable) {
            DPRINTF(EthernetSM, "Receive Disabled! Nothing to do.\n");
            goto exit;
        }

        if (CRDD) {
            rxState = rxDescRefr;

            rxDmaAddr = regs.rxdp & 0x3fffffff;
            rxDmaData =
                is64bit ? (void *)&rxDesc64.link : (void *)&rxDesc32.link;
            rxDmaLen = is64bit ? sizeof(rxDesc64.link) : sizeof(rxDesc32.link);
            rxDmaFree = dmaDescFree;

            descDmaReads++;
            descDmaRdBytes += rxDmaLen;

            if (doRxDmaRead())
                goto exit;
        } else {
            rxState = rxDescRead;

            rxDmaAddr = regs.rxdp & 0x3fffffff;
            rxDmaData = is64bit ? (void *)&rxDesc64 : (void *)&rxDesc32;
            rxDmaLen = is64bit ? sizeof(rxDesc64) : sizeof(rxDesc32);
            rxDmaFree = dmaDescFree;

            descDmaReads++;
            descDmaRdBytes += rxDmaLen;

            if (doRxDmaRead())
                goto exit;
        }
        break;

      case rxDescRefr:
        if (rxDmaState != dmaIdle)
            goto exit;

        rxState = rxAdvance;
        break;

     case rxDescRead:
        if (rxDmaState != dmaIdle)
            goto exit;

        DPRINTF(EthernetDesc, "rxDesc: addr=%08x read descriptor\n",
                regs.rxdp & 0x3fffffff);
        DPRINTF(EthernetDesc,
                "rxDesc: link=%#x bufptr=%#x cmdsts=%08x extsts=%08x\n",
                link, bufptr, cmdsts, extsts);

        if (cmdsts & CMDSTS_OWN) {
            devIntrPost(ISR_RXIDLE);
            rxState = rxIdle;
            goto exit;
        } else {
            rxState = rxFifoBlock;
            rxFragPtr = bufptr;
            rxDescCnt = cmdsts & CMDSTS_LEN_MASK;
        }
        break;

      case rxFifoBlock:
        if (!rxPacket) {
            /**
             * @todo in reality, we should be able to start processing
             * the packet as it arrives, and not have to wait for the
             * full packet ot be in the receive fifo.
             */
            if (rxFifo.empty())
                goto exit;

            DPRINTF(EthernetSM, "****processing receive of new packet****\n");

            // If we don't have a packet, grab a new one from the fifo.
            rxPacket = rxFifo.front();
            rxPktBytes = rxPacket->length;
            rxPacketBufPtr = rxPacket->data;

#if TRACING_ON
            if (DTRACE(Ethernet)) {
                IpPtr ip(rxPacket);
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

            // sanity check - i think the driver behaves like this
            assert(rxDescCnt >= rxPktBytes);
            rxFifo.pop();
        }


        // dont' need the && rxDescCnt > 0 if driver sanity check
        // above holds
        if (rxPktBytes > 0) {
            rxState = rxFragWrite;
            // don't need min<>(rxPktBytes,rxDescCnt) if above sanity
            // check holds
            rxXferLen = rxPktBytes;

            rxDmaAddr = rxFragPtr & 0x3fffffff;
            rxDmaData = rxPacketBufPtr;
            rxDmaLen = rxXferLen;
            rxDmaFree = dmaDataFree;

            if (doRxDmaWrite())
                goto exit;

        } else {
            rxState = rxDescWrite;

            //if (rxPktBytes == 0) {  /* packet is done */
            assert(rxPktBytes == 0);
            DPRINTF(EthernetSM, "done with receiving packet\n");

            cmdsts |= CMDSTS_OWN;
            cmdsts &= ~CMDSTS_MORE;
            cmdsts |= CMDSTS_OK;
            cmdsts &= 0xffff0000;
            cmdsts += rxPacket->length;   //i.e. set CMDSTS_SIZE

#if 0
            /*
             * all the driver uses these are for its own stats keeping
             * which we don't care about, aren't necessary for
             * functionality and doing this would just slow us down.
             * if they end up using this in a later version for
             * functional purposes, just undef
             */
            if (rxFilterEnable) {
                cmdsts &= ~CMDSTS_DEST_MASK;
                const EthAddr &dst = rxFifoFront()->dst();
                if (dst->unicast())
                    cmdsts |= CMDSTS_DEST_SELF;
                if (dst->multicast())
                    cmdsts |= CMDSTS_DEST_MULTI;
                if (dst->broadcast())
                    cmdsts |= CMDSTS_DEST_MASK;
            }
#endif

            IpPtr ip(rxPacket);
            if (extstsEnable && ip) {
                extsts |= EXTSTS_IPPKT;
                rxIpChecksums++;
                if (cksum(ip) != 0) {
                    DPRINTF(EthernetCksum, "Rx IP Checksum Error\n");
                    extsts |= EXTSTS_IPERR;
                }
                TcpPtr tcp(ip);
                UdpPtr udp(ip);
                if (tcp) {
                    extsts |= EXTSTS_TCPPKT;
                    rxTcpChecksums++;
                    if (cksum(tcp) != 0) {
                        DPRINTF(EthernetCksum, "Rx TCP Checksum Error\n");
                        extsts |= EXTSTS_TCPERR;

                    }
                } else if (udp) {
                    extsts |= EXTSTS_UDPPKT;
                    rxUdpChecksums++;
                    if (cksum(udp) != 0) {
                        DPRINTF(EthernetCksum, "Rx UDP Checksum Error\n");
                        extsts |= EXTSTS_UDPERR;
                    }
                }
            }
            rxPacket = 0;

            /*
             * the driver seems to always receive into desc buffers
             * of size 1514, so you never have a pkt that is split
             * into multiple descriptors on the receive side, so
             * i don't implement that case, hence the assert above.
             */

            DPRINTF(EthernetDesc,
                    "rxDesc: addr=%08x writeback cmdsts extsts\n",
                    regs.rxdp & 0x3fffffff);
            DPRINTF(EthernetDesc,
                    "rxDesc: link=%#x bufptr=%#x cmdsts=%08x extsts=%08x\n",
                    link, bufptr, cmdsts, extsts);

            rxDmaAddr = regs.rxdp & 0x3fffffff;
            rxDmaData = &cmdsts;
            if (is64bit) {
                rxDmaAddr += offsetof(ns_desc64, cmdsts);
                rxDmaLen = sizeof(rxDesc64.cmdsts) + sizeof(rxDesc64.extsts);
            } else {
                rxDmaAddr += offsetof(ns_desc32, cmdsts);
                rxDmaLen = sizeof(rxDesc32.cmdsts) + sizeof(rxDesc32.extsts);
            }
            rxDmaFree = dmaDescFree;

            descDmaWrites++;
            descDmaWrBytes += rxDmaLen;

            if (doRxDmaWrite())
                goto exit;
        }
        break;

      case rxFragWrite:
        if (rxDmaState != dmaIdle)
            goto exit;

        rxPacketBufPtr += rxXferLen;
        rxFragPtr += rxXferLen;
        rxPktBytes -= rxXferLen;

        rxState = rxFifoBlock;
        break;

      case rxDescWrite:
        if (rxDmaState != dmaIdle)
            goto exit;

        assert(cmdsts & CMDSTS_OWN);

        assert(rxPacket == 0);
        devIntrPost(ISR_RXOK);

        if (cmdsts & CMDSTS_INTR)
            devIntrPost(ISR_RXDESC);

        if (!rxEnable) {
            DPRINTF(EthernetSM, "Halting the RX state machine\n");
            rxState = rxIdle;
            goto exit;
        } else
            rxState = rxAdvance;
        break;

      case rxAdvance:
        if (link == 0) {
            devIntrPost(ISR_RXIDLE);
            rxState = rxIdle;
            CRDD = true;
            goto exit;
        } else {
            if (rxDmaState != dmaIdle)
                goto exit;
            rxState = rxDescRead;
            regs.rxdp = link;
            CRDD = false;

            rxDmaAddr = regs.rxdp & 0x3fffffff;
            rxDmaData = is64bit ? (void *)&rxDesc64 : (void *)&rxDesc32;
            rxDmaLen = is64bit ? sizeof(rxDesc64) : sizeof(rxDesc32);
            rxDmaFree = dmaDescFree;

            if (doRxDmaRead())
                goto exit;
        }
        break;

      default:
        panic("Invalid rxState!");
    }

    DPRINTF(EthernetSM, "entering next rxState=%s\n",
            NsRxStateStrings[rxState]);
    goto next;

  exit:
    /**
     * @todo do we want to schedule a future kick?
     */
    DPRINTF(EthernetSM, "rx state machine exited rxState=%s\n",
            NsRxStateStrings[rxState]);

    if (!rxKickEvent.scheduled())
        schedule(rxKickEvent, rxKickTick);
}

void
NSGigE::transmit()
{
    if (txFifo.empty()) {
        DPRINTF(Ethernet, "nothing to transmit\n");
        return;
    }

    DPRINTF(Ethernet, "Attempt Pkt Transmit: txFifo length=%d\n",
            txFifo.size());
    if (interface->sendPacket(txFifo.front())) {
#if TRACING_ON
        if (DTRACE(Ethernet)) {
            IpPtr ip(txFifo.front());
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

        DDUMP(EthernetData, txFifo.front()->data, txFifo.front()->length);
        txBytes += txFifo.front()->length;
        txPackets++;

        DPRINTF(Ethernet, "Successful Xmit! now txFifoAvail is %d\n",
                txFifo.avail());
        txFifo.pop();

        /*
         * normally do a writeback of the descriptor here, and ONLY
         * after that is done, send this interrupt.  but since our
         * stuff never actually fails, just do this interrupt here,
         * otherwise the code has to stray from this nice format.
         * besides, it's functionally the same.
         */
        devIntrPost(ISR_TXOK);
    }

   if (!txFifo.empty() && !txEvent.scheduled()) {
       DPRINTF(Ethernet, "reschedule transmit\n");
       schedule(txEvent, curTick() + retryTime);
   }
}

bool
NSGigE::doTxDmaRead()
{
    assert(txDmaState == dmaIdle || txDmaState == dmaReadWaiting);
    txDmaState = dmaReading;

    if (dmaPending() || drainState() != DrainState::Running)
        txDmaState = dmaReadWaiting;
    else
        dmaRead(txDmaAddr, txDmaLen, &txDmaReadEvent, (uint8_t*)txDmaData);

    return true;
}

void
NSGigE::txDmaReadDone()
{
    assert(txDmaState == dmaReading);
    txDmaState = dmaIdle;

    DPRINTF(EthernetDMA, "tx dma read  paddr=%#x len=%d\n",
            txDmaAddr, txDmaLen);
    DDUMP(EthernetDMA, txDmaData, txDmaLen);

    // If the receive state machine  has a pending DMA, let it go first
    if (rxDmaState == dmaReadWaiting || rxDmaState == dmaWriteWaiting)
        rxKick();

    txKick();
}

bool
NSGigE::doTxDmaWrite()
{
    assert(txDmaState == dmaIdle || txDmaState == dmaWriteWaiting);
    txDmaState = dmaWriting;

    if (dmaPending() || drainState() != DrainState::Running)
        txDmaState = dmaWriteWaiting;
    else
        dmaWrite(txDmaAddr, txDmaLen, &txDmaWriteEvent, (uint8_t*)txDmaData);
    return true;
}

void
NSGigE::txDmaWriteDone()
{
    assert(txDmaState == dmaWriting);
    txDmaState = dmaIdle;

    DPRINTF(EthernetDMA, "tx dma write paddr=%#x len=%d\n",
            txDmaAddr, txDmaLen);
    DDUMP(EthernetDMA, txDmaData, txDmaLen);

    // If the receive state machine  has a pending DMA, let it go first
    if (rxDmaState == dmaReadWaiting || rxDmaState == dmaWriteWaiting)
        rxKick();

    txKick();
}

void
NSGigE::txKick()
{
    bool is64bit = (bool)(regs.config & CFGR_M64ADDR);

    DPRINTF(EthernetSM, "transmit kick txState=%s %d-bit\n",
            NsTxStateStrings[txState], is64bit ? 64 : 32);

    Addr link, bufptr;
    uint32_t &cmdsts = is64bit ? txDesc64.cmdsts : txDesc32.cmdsts;
    uint32_t &extsts = is64bit ? txDesc64.extsts : txDesc32.extsts;

  next:
    if (txKickTick > curTick()) {
        DPRINTF(EthernetSM, "transmit kick exiting, can't run till %d\n",
                txKickTick);
        goto exit;
    }

    // Go to the next state machine clock tick.
    txKickTick = clockEdge(Cycles(1));

    switch(txDmaState) {
      case dmaReadWaiting:
        if (doTxDmaRead())
            goto exit;
        break;
      case dmaWriteWaiting:
        if (doTxDmaWrite())
            goto exit;
        break;
      default:
        break;
    }

    link = is64bit ? (Addr)txDesc64.link : (Addr)txDesc32.link;
    bufptr = is64bit ? (Addr)txDesc64.bufptr : (Addr)txDesc32.bufptr;
    switch (txState) {
      case txIdle:
        if (!txEnable) {
            DPRINTF(EthernetSM, "Transmit disabled.  Nothing to do.\n");
            goto exit;
        }

        if (CTDD) {
            txState = txDescRefr;

            txDmaAddr = regs.txdp & 0x3fffffff;
            txDmaData =
                is64bit ? (void *)&txDesc64.link : (void *)&txDesc32.link;
            txDmaLen = is64bit ? sizeof(txDesc64.link) : sizeof(txDesc32.link);
            txDmaFree = dmaDescFree;

            descDmaReads++;
            descDmaRdBytes += txDmaLen;

            if (doTxDmaRead())
                goto exit;

        } else {
            txState = txDescRead;

            txDmaAddr = regs.txdp & 0x3fffffff;
            txDmaData = is64bit ? (void *)&txDesc64 : (void *)&txDesc32;
            txDmaLen = is64bit ? sizeof(txDesc64) : sizeof(txDesc32);
            txDmaFree = dmaDescFree;

            descDmaReads++;
            descDmaRdBytes += txDmaLen;

            if (doTxDmaRead())
                goto exit;
        }
        break;

      case txDescRefr:
        if (txDmaState != dmaIdle)
            goto exit;

        txState = txAdvance;
        break;

      case txDescRead:
        if (txDmaState != dmaIdle)
            goto exit;

        DPRINTF(EthernetDesc, "txDesc: addr=%08x read descriptor\n",
                regs.txdp & 0x3fffffff);
        DPRINTF(EthernetDesc,
                "txDesc: link=%#x bufptr=%#x cmdsts=%#08x extsts=%#08x\n",
                link, bufptr, cmdsts, extsts);

        if (cmdsts & CMDSTS_OWN) {
            txState = txFifoBlock;
            txFragPtr = bufptr;
            txDescCnt = cmdsts & CMDSTS_LEN_MASK;
        } else {
            devIntrPost(ISR_TXIDLE);
            txState = txIdle;
            goto exit;
        }
        break;

      case txFifoBlock:
        if (!txPacket) {
            DPRINTF(EthernetSM, "****starting the tx of a new packet****\n");
            txPacket = make_shared<EthPacketData>(16384);
            txPacketBufPtr = txPacket->data;
        }

        if (txDescCnt == 0) {
            DPRINTF(EthernetSM, "the txDescCnt == 0, done with descriptor\n");
            if (cmdsts & CMDSTS_MORE) {
                DPRINTF(EthernetSM, "there are more descriptors to come\n");
                txState = txDescWrite;

                cmdsts &= ~CMDSTS_OWN;

                txDmaAddr = regs.txdp & 0x3fffffff;
                txDmaData = &cmdsts;
                if (is64bit) {
                    txDmaAddr += offsetof(ns_desc64, cmdsts);
                    txDmaLen = sizeof(txDesc64.cmdsts);
                } else {
                    txDmaAddr += offsetof(ns_desc32, cmdsts);
                    txDmaLen = sizeof(txDesc32.cmdsts);
                }
                txDmaFree = dmaDescFree;

                if (doTxDmaWrite())
                    goto exit;

            } else { /* this packet is totally done */
                DPRINTF(EthernetSM, "This packet is done, let's wrap it up\n");
                /* deal with the the packet that just finished */
                if ((regs.vtcr & VTCR_PPCHK) && extstsEnable) {
                    IpPtr ip(txPacket);
                    if (extsts & EXTSTS_UDPPKT) {
                        UdpPtr udp(ip);
                        if (udp) {
                            udp->sum(0);
                            udp->sum(cksum(udp));
                            txUdpChecksums++;
                        } else {
                            Debug::breakpoint();
                            warn_once("UDPPKT set, but not UDP!\n");
                        }
                    } else if (extsts & EXTSTS_TCPPKT) {
                        TcpPtr tcp(ip);
                        if (tcp) {
                            tcp->sum(0);
                            tcp->sum(cksum(tcp));
                            txTcpChecksums++;
                        } else {
                            warn_once("TCPPKT set, but not UDP!\n");
                        }
                    }
                    if (extsts & EXTSTS_IPPKT) {
                        if (ip) {
                            ip->sum(0);
                            ip->sum(cksum(ip));
                            txIpChecksums++;
                        } else {
                            warn_once("IPPKT set, but not UDP!\n");
                        }
                    }
                }

                txPacket->simLength = txPacketBufPtr - txPacket->data;
                txPacket->length = txPacketBufPtr - txPacket->data;
                // this is just because the receive can't handle a
                // packet bigger want to make sure
                if (txPacket->length > 1514)
                    panic("transmit packet too large, %s > 1514\n",
                          txPacket->length);

#ifndef NDEBUG
                bool success =
#endif
                    txFifo.push(txPacket);
                assert(success);

                /*
                 * this following section is not tqo spec, but
                 * functionally shouldn't be any different.  normally,
                 * the chip will wait til the transmit has occurred
                 * before writing back the descriptor because it has
                 * to wait to see that it was successfully transmitted
                 * to decide whether to set CMDSTS_OK or not.
                 * however, in the simulator since it is always
                 * successfully transmitted, and writing it exactly to
                 * spec would complicate the code, we just do it here
                 */

                cmdsts &= ~CMDSTS_OWN;
                cmdsts |= CMDSTS_OK;

                DPRINTF(EthernetDesc,
                        "txDesc writeback: cmdsts=%08x extsts=%08x\n",
                        cmdsts, extsts);

                txDmaFree = dmaDescFree;
                txDmaAddr = regs.txdp & 0x3fffffff;
                txDmaData = &cmdsts;
                if (is64bit) {
                    txDmaAddr += offsetof(ns_desc64, cmdsts);
                    txDmaLen =
                        sizeof(txDesc64.cmdsts) + sizeof(txDesc64.extsts);
                } else {
                    txDmaAddr += offsetof(ns_desc32, cmdsts);
                    txDmaLen =
                        sizeof(txDesc32.cmdsts) + sizeof(txDesc32.extsts);
                }

                descDmaWrites++;
                descDmaWrBytes += txDmaLen;

                transmit();
                txPacket = 0;

                if (!txEnable) {
                    DPRINTF(EthernetSM, "halting TX state machine\n");
                    txState = txIdle;
                    goto exit;
                } else
                    txState = txAdvance;

                if (doTxDmaWrite())
                    goto exit;
            }
        } else {
            DPRINTF(EthernetSM, "this descriptor isn't done yet\n");
            if (!txFifo.full()) {
                txState = txFragRead;

                /*
                 * The number of bytes transferred is either whatever
                 * is left in the descriptor (txDescCnt), or if there
                 * is not enough room in the fifo, just whatever room
                 * is left in the fifo
                 */
                txXferLen = min<uint32_t>(txDescCnt, txFifo.avail());

                txDmaAddr = txFragPtr & 0x3fffffff;
                txDmaData = txPacketBufPtr;
                txDmaLen = txXferLen;
                txDmaFree = dmaDataFree;

                if (doTxDmaRead())
                    goto exit;
            } else {
                txState = txFifoBlock;
                transmit();

                goto exit;
            }

        }
        break;

      case txFragRead:
        if (txDmaState != dmaIdle)
            goto exit;

        txPacketBufPtr += txXferLen;
        txFragPtr += txXferLen;
        txDescCnt -= txXferLen;
        txFifo.reserve(txXferLen);

        txState = txFifoBlock;
        break;

      case txDescWrite:
        if (txDmaState != dmaIdle)
            goto exit;

        if (cmdsts & CMDSTS_INTR)
            devIntrPost(ISR_TXDESC);

        if (!txEnable) {
            DPRINTF(EthernetSM, "halting TX state machine\n");
            txState = txIdle;
            goto exit;
        } else
            txState = txAdvance;
        break;

      case txAdvance:
        if (link == 0) {
            devIntrPost(ISR_TXIDLE);
            txState = txIdle;
            goto exit;
        } else {
            if (txDmaState != dmaIdle)
                goto exit;
            txState = txDescRead;
            regs.txdp = link;
            CTDD = false;

            txDmaAddr = link & 0x3fffffff;
            txDmaData = is64bit ? (void *)&txDesc64 : (void *)&txDesc32;
            txDmaLen = is64bit ? sizeof(txDesc64) : sizeof(txDesc32);
            txDmaFree = dmaDescFree;

            if (doTxDmaRead())
                goto exit;
        }
        break;

      default:
        panic("invalid state");
    }

    DPRINTF(EthernetSM, "entering next txState=%s\n",
            NsTxStateStrings[txState]);
    goto next;

  exit:
    /**
     * @todo do we want to schedule a future kick?
     */
    DPRINTF(EthernetSM, "tx state machine exited txState=%s\n",
            NsTxStateStrings[txState]);

    if (!txKickEvent.scheduled())
        schedule(txKickEvent, txKickTick);
}

/**
 * Advance the EEPROM state machine
 * Called on rising edge of EEPROM clock bit in MEAR
 */
void
NSGigE::eepromKick()
{
    switch (eepromState) {

      case eepromStart:

        // Wait for start bit
        if (regs.mear & MEAR_EEDI) {
            // Set up to get 2 opcode bits
            eepromState = eepromGetOpcode;
            eepromBitsToRx = 2;
            eepromOpcode = 0;
        }
        break;

      case eepromGetOpcode:
        eepromOpcode <<= 1;
        eepromOpcode += (regs.mear & MEAR_EEDI) ? 1 : 0;
        --eepromBitsToRx;

        // Done getting opcode
        if (eepromBitsToRx == 0) {
            if (eepromOpcode != EEPROM_READ)
                panic("only EEPROM reads are implemented!");

            // Set up to get address
            eepromState = eepromGetAddress;
            eepromBitsToRx = 6;
            eepromAddress = 0;
        }
        break;

      case eepromGetAddress:
        eepromAddress <<= 1;
        eepromAddress += (regs.mear & MEAR_EEDI) ? 1 : 0;
        --eepromBitsToRx;

        // Done getting address
        if (eepromBitsToRx == 0) {

            if (eepromAddress >= EEPROM_SIZE)
                panic("EEPROM read access out of range!");

            switch (eepromAddress) {

              case EEPROM_PMATCH2_ADDR:
                eepromData = rom.perfectMatch[5];
                eepromData <<= 8;
                eepromData += rom.perfectMatch[4];
                break;

              case EEPROM_PMATCH1_ADDR:
                eepromData = rom.perfectMatch[3];
                eepromData <<= 8;
                eepromData += rom.perfectMatch[2];
                break;

              case EEPROM_PMATCH0_ADDR:
                eepromData = rom.perfectMatch[1];
                eepromData <<= 8;
                eepromData += rom.perfectMatch[0];
                break;

              default:
                panic("FreeBSD driver only uses EEPROM to read PMATCH!");
            }
            // Set up to read data
            eepromState = eepromRead;
            eepromBitsToRx = 16;

            // Clear data in bit
            regs.mear &= ~MEAR_EEDI;
        }
        break;

      case eepromRead:
        // Clear Data Out bit
        regs.mear &= ~MEAR_EEDO;
        // Set bit to value of current EEPROM bit
        regs.mear |= (eepromData & 0x8000) ? MEAR_EEDO : 0x0;

        eepromData <<= 1;
        --eepromBitsToRx;

        // All done
        if (eepromBitsToRx == 0) {
            eepromState = eepromStart;
        }
        break;

      default:
        panic("invalid EEPROM state");
    }

}

void
NSGigE::transferDone()
{
    if (txFifo.empty()) {
        DPRINTF(Ethernet, "transfer complete: txFifo empty...nothing to do\n");
        return;
    }

    DPRINTF(Ethernet, "transfer complete: data in txFifo...schedule xmit\n");

    reschedule(txEvent, clockEdge(Cycles(1)), true);
}

bool
NSGigE::rxFilter(const EthPacketPtr &packet)
{
    EthPtr eth = packet;
    bool drop = true;
    string type;

    const EthAddr &dst = eth->dst();
    if (dst.unicast()) {
        // If we're accepting all unicast addresses
        if (acceptUnicast)
            drop = false;

        // If we make a perfect match
        if (acceptPerfect && dst == rom.perfectMatch)
            drop = false;

        if (acceptArp && eth->type() == ETH_TYPE_ARP)
            drop = false;

    } else if (dst.broadcast()) {
        // if we're accepting broadcasts
        if (acceptBroadcast)
            drop = false;

    } else if (dst.multicast()) {
        // if we're accepting all multicasts
        if (acceptMulticast)
            drop = false;

        // Multicast hashing faked - all packets accepted
        if (multicastHashEnable)
            drop = false;
    }

    if (drop) {
        DPRINTF(Ethernet, "rxFilter drop\n");
        DDUMP(EthernetData, packet->data, packet->length);
    }

    return drop;
}

bool
NSGigE::recvPacket(EthPacketPtr packet)
{
    rxBytes += packet->length;
    rxPackets++;

    DPRINTF(Ethernet, "Receiving packet from wire, rxFifoAvail=%d\n",
            rxFifo.avail());

    if (!rxEnable) {
        DPRINTF(Ethernet, "receive disabled...packet dropped\n");
        return true;
    }

    if (!rxFilterEnable) {
        DPRINTF(Ethernet,
            "receive packet filtering disabled . . . packet dropped\n");
        return true;
    }

    if (rxFilter(packet)) {
        DPRINTF(Ethernet, "packet filtered...dropped\n");
        return true;
    }

    if (rxFifo.avail() < packet->length) {
#if TRACING_ON
        IpPtr ip(packet);
        TcpPtr tcp(ip);
        if (ip) {
            DPRINTF(Ethernet,
                    "packet won't fit in receive buffer...pkt ID %d dropped\n",
                    ip->id());
            if (tcp) {
                DPRINTF(Ethernet, "Seq=%d\n", tcp->seq());
            }
        }
#endif
        droppedPackets++;
        devIntrPost(ISR_RXORN);
        return false;
    }

    rxFifo.push(packet);

    rxKick();
    return true;
}


void
NSGigE::drainResume()
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
NSGigE::serialize(CheckpointOut &cp) const
{
    // Serialize the PciDevice base class
    PciDevice::serialize(cp);

    /*
     * Finalize any DMA events now.
     */
    // @todo will mem system save pending dma?

    /*
     * Serialize the device registers
     */
    SERIALIZE_SCALAR(regs.command);
    SERIALIZE_SCALAR(regs.config);
    SERIALIZE_SCALAR(regs.mear);
    SERIALIZE_SCALAR(regs.ptscr);
    SERIALIZE_SCALAR(regs.isr);
    SERIALIZE_SCALAR(regs.imr);
    SERIALIZE_SCALAR(regs.ier);
    SERIALIZE_SCALAR(regs.ihr);
    SERIALIZE_SCALAR(regs.txdp);
    SERIALIZE_SCALAR(regs.txdp_hi);
    SERIALIZE_SCALAR(regs.txcfg);
    SERIALIZE_SCALAR(regs.gpior);
    SERIALIZE_SCALAR(regs.rxdp);
    SERIALIZE_SCALAR(regs.rxdp_hi);
    SERIALIZE_SCALAR(regs.rxcfg);
    SERIALIZE_SCALAR(regs.pqcr);
    SERIALIZE_SCALAR(regs.wcsr);
    SERIALIZE_SCALAR(regs.pcr);
    SERIALIZE_SCALAR(regs.rfcr);
    SERIALIZE_SCALAR(regs.rfdr);
    SERIALIZE_SCALAR(regs.brar);
    SERIALIZE_SCALAR(regs.brdr);
    SERIALIZE_SCALAR(regs.srr);
    SERIALIZE_SCALAR(regs.mibc);
    SERIALIZE_SCALAR(regs.vrcr);
    SERIALIZE_SCALAR(regs.vtcr);
    SERIALIZE_SCALAR(regs.vdr);
    SERIALIZE_SCALAR(regs.ccsr);
    SERIALIZE_SCALAR(regs.tbicr);
    SERIALIZE_SCALAR(regs.tbisr);
    SERIALIZE_SCALAR(regs.tanar);
    SERIALIZE_SCALAR(regs.tanlpar);
    SERIALIZE_SCALAR(regs.taner);
    SERIALIZE_SCALAR(regs.tesr);

    SERIALIZE_ARRAY(rom.perfectMatch, ETH_ADDR_LEN);
    SERIALIZE_ARRAY(rom.filterHash, FHASH_SIZE);

    SERIALIZE_SCALAR(ioEnable);

    /*
     * Serialize the data Fifos
     */
    rxFifo.serialize("rxFifo", cp);
    txFifo.serialize("txFifo", cp);

    /*
     * Serialize the various helper variables
     */
    bool txPacketExists = txPacket != nullptr;
    SERIALIZE_SCALAR(txPacketExists);
    if (txPacketExists) {
        txPacket->simLength = txPacketBufPtr - txPacket->data;
        txPacket->length = txPacketBufPtr - txPacket->data;
        txPacket->serialize("txPacket", cp);
        uint32_t txPktBufPtr = (uint32_t) (txPacketBufPtr - txPacket->data);
        SERIALIZE_SCALAR(txPktBufPtr);
    }

    bool rxPacketExists = rxPacket != nullptr;
    SERIALIZE_SCALAR(rxPacketExists);
    if (rxPacketExists) {
        rxPacket->serialize("rxPacket", cp);
        uint32_t rxPktBufPtr = (uint32_t) (rxPacketBufPtr - rxPacket->data);
        SERIALIZE_SCALAR(rxPktBufPtr);
    }

    SERIALIZE_SCALAR(txXferLen);
    SERIALIZE_SCALAR(rxXferLen);

    /*
     * Serialize Cached Descriptors
     */
    SERIALIZE_SCALAR(rxDesc64.link);
    SERIALIZE_SCALAR(rxDesc64.bufptr);
    SERIALIZE_SCALAR(rxDesc64.cmdsts);
    SERIALIZE_SCALAR(rxDesc64.extsts);
    SERIALIZE_SCALAR(txDesc64.link);
    SERIALIZE_SCALAR(txDesc64.bufptr);
    SERIALIZE_SCALAR(txDesc64.cmdsts);
    SERIALIZE_SCALAR(txDesc64.extsts);
    SERIALIZE_SCALAR(rxDesc32.link);
    SERIALIZE_SCALAR(rxDesc32.bufptr);
    SERIALIZE_SCALAR(rxDesc32.cmdsts);
    SERIALIZE_SCALAR(rxDesc32.extsts);
    SERIALIZE_SCALAR(txDesc32.link);
    SERIALIZE_SCALAR(txDesc32.bufptr);
    SERIALIZE_SCALAR(txDesc32.cmdsts);
    SERIALIZE_SCALAR(txDesc32.extsts);
    SERIALIZE_SCALAR(extstsEnable);

    /*
     * Serialize tx state machine
     */
    int txState = this->txState;
    SERIALIZE_SCALAR(txState);
    SERIALIZE_SCALAR(txEnable);
    SERIALIZE_SCALAR(CTDD);
    SERIALIZE_SCALAR(txFragPtr);
    SERIALIZE_SCALAR(txDescCnt);
    int txDmaState = this->txDmaState;
    SERIALIZE_SCALAR(txDmaState);
    SERIALIZE_SCALAR(txKickTick);

    /*
     * Serialize rx state machine
     */
    int rxState = this->rxState;
    SERIALIZE_SCALAR(rxState);
    SERIALIZE_SCALAR(rxEnable);
    SERIALIZE_SCALAR(CRDD);
    SERIALIZE_SCALAR(rxPktBytes);
    SERIALIZE_SCALAR(rxFragPtr);
    SERIALIZE_SCALAR(rxDescCnt);
    int rxDmaState = this->rxDmaState;
    SERIALIZE_SCALAR(rxDmaState);
    SERIALIZE_SCALAR(rxKickTick);

    /*
     * Serialize EEPROM state machine
     */
    int eepromState = this->eepromState;
    SERIALIZE_SCALAR(eepromState);
    SERIALIZE_SCALAR(eepromClk);
    SERIALIZE_SCALAR(eepromBitsToRx);
    SERIALIZE_SCALAR(eepromOpcode);
    SERIALIZE_SCALAR(eepromAddress);
    SERIALIZE_SCALAR(eepromData);

    /*
     * If there's a pending transmit, store the time so we can
     * reschedule it later
     */
    Tick transmitTick = txEvent.scheduled() ? txEvent.when() - curTick() : 0;
    SERIALIZE_SCALAR(transmitTick);

    /*
     * receive address filter settings
     */
    SERIALIZE_SCALAR(rxFilterEnable);
    SERIALIZE_SCALAR(acceptBroadcast);
    SERIALIZE_SCALAR(acceptMulticast);
    SERIALIZE_SCALAR(acceptUnicast);
    SERIALIZE_SCALAR(acceptPerfect);
    SERIALIZE_SCALAR(acceptArp);
    SERIALIZE_SCALAR(multicastHashEnable);

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
NSGigE::unserialize(CheckpointIn &cp)
{
    // Unserialize the PciDevice base class
    PciDevice::unserialize(cp);

    UNSERIALIZE_SCALAR(regs.command);
    UNSERIALIZE_SCALAR(regs.config);
    UNSERIALIZE_SCALAR(regs.mear);
    UNSERIALIZE_SCALAR(regs.ptscr);
    UNSERIALIZE_SCALAR(regs.isr);
    UNSERIALIZE_SCALAR(regs.imr);
    UNSERIALIZE_SCALAR(regs.ier);
    UNSERIALIZE_SCALAR(regs.ihr);
    UNSERIALIZE_SCALAR(regs.txdp);
    UNSERIALIZE_SCALAR(regs.txdp_hi);
    UNSERIALIZE_SCALAR(regs.txcfg);
    UNSERIALIZE_SCALAR(regs.gpior);
    UNSERIALIZE_SCALAR(regs.rxdp);
    UNSERIALIZE_SCALAR(regs.rxdp_hi);
    UNSERIALIZE_SCALAR(regs.rxcfg);
    UNSERIALIZE_SCALAR(regs.pqcr);
    UNSERIALIZE_SCALAR(regs.wcsr);
    UNSERIALIZE_SCALAR(regs.pcr);
    UNSERIALIZE_SCALAR(regs.rfcr);
    UNSERIALIZE_SCALAR(regs.rfdr);
    UNSERIALIZE_SCALAR(regs.brar);
    UNSERIALIZE_SCALAR(regs.brdr);
    UNSERIALIZE_SCALAR(regs.srr);
    UNSERIALIZE_SCALAR(regs.mibc);
    UNSERIALIZE_SCALAR(regs.vrcr);
    UNSERIALIZE_SCALAR(regs.vtcr);
    UNSERIALIZE_SCALAR(regs.vdr);
    UNSERIALIZE_SCALAR(regs.ccsr);
    UNSERIALIZE_SCALAR(regs.tbicr);
    UNSERIALIZE_SCALAR(regs.tbisr);
    UNSERIALIZE_SCALAR(regs.tanar);
    UNSERIALIZE_SCALAR(regs.tanlpar);
    UNSERIALIZE_SCALAR(regs.taner);
    UNSERIALIZE_SCALAR(regs.tesr);

    UNSERIALIZE_ARRAY(rom.perfectMatch, ETH_ADDR_LEN);
    UNSERIALIZE_ARRAY(rom.filterHash, FHASH_SIZE);

    UNSERIALIZE_SCALAR(ioEnable);

    /*
     * unserialize the data fifos
     */
    rxFifo.unserialize("rxFifo", cp);
    txFifo.unserialize("txFifo", cp);

    /*
     * unserialize the various helper variables
     */
    bool txPacketExists;
    UNSERIALIZE_SCALAR(txPacketExists);
    if (txPacketExists) {
        txPacket = make_shared<EthPacketData>();
        txPacket->unserialize("txPacket", cp);
        uint32_t txPktBufPtr;
        UNSERIALIZE_SCALAR(txPktBufPtr);
        txPacketBufPtr = (uint8_t *) txPacket->data + txPktBufPtr;
    } else
        txPacket = 0;

    bool rxPacketExists;
    UNSERIALIZE_SCALAR(rxPacketExists);
    rxPacket = 0;
    if (rxPacketExists) {
        rxPacket = make_shared<EthPacketData>();
        rxPacket->unserialize("rxPacket", cp);
        uint32_t rxPktBufPtr;
        UNSERIALIZE_SCALAR(rxPktBufPtr);
        rxPacketBufPtr = (uint8_t *) rxPacket->data + rxPktBufPtr;
    } else
        rxPacket = 0;

    UNSERIALIZE_SCALAR(txXferLen);
    UNSERIALIZE_SCALAR(rxXferLen);

    /*
     * Unserialize Cached Descriptors
     */
    UNSERIALIZE_SCALAR(rxDesc64.link);
    UNSERIALIZE_SCALAR(rxDesc64.bufptr);
    UNSERIALIZE_SCALAR(rxDesc64.cmdsts);
    UNSERIALIZE_SCALAR(rxDesc64.extsts);
    UNSERIALIZE_SCALAR(txDesc64.link);
    UNSERIALIZE_SCALAR(txDesc64.bufptr);
    UNSERIALIZE_SCALAR(txDesc64.cmdsts);
    UNSERIALIZE_SCALAR(txDesc64.extsts);
    UNSERIALIZE_SCALAR(rxDesc32.link);
    UNSERIALIZE_SCALAR(rxDesc32.bufptr);
    UNSERIALIZE_SCALAR(rxDesc32.cmdsts);
    UNSERIALIZE_SCALAR(rxDesc32.extsts);
    UNSERIALIZE_SCALAR(txDesc32.link);
    UNSERIALIZE_SCALAR(txDesc32.bufptr);
    UNSERIALIZE_SCALAR(txDesc32.cmdsts);
    UNSERIALIZE_SCALAR(txDesc32.extsts);
    UNSERIALIZE_SCALAR(extstsEnable);

    /*
     * unserialize tx state machine
     */
    int txState;
    UNSERIALIZE_SCALAR(txState);
    this->txState = (TxState) txState;
    UNSERIALIZE_SCALAR(txEnable);
    UNSERIALIZE_SCALAR(CTDD);
    UNSERIALIZE_SCALAR(txFragPtr);
    UNSERIALIZE_SCALAR(txDescCnt);
    int txDmaState;
    UNSERIALIZE_SCALAR(txDmaState);
    this->txDmaState = (DmaState) txDmaState;
    UNSERIALIZE_SCALAR(txKickTick);
    if (txKickTick)
        schedule(txKickEvent, txKickTick);

    /*
     * unserialize rx state machine
     */
    int rxState;
    UNSERIALIZE_SCALAR(rxState);
    this->rxState = (RxState) rxState;
    UNSERIALIZE_SCALAR(rxEnable);
    UNSERIALIZE_SCALAR(CRDD);
    UNSERIALIZE_SCALAR(rxPktBytes);
    UNSERIALIZE_SCALAR(rxFragPtr);
    UNSERIALIZE_SCALAR(rxDescCnt);
    int rxDmaState;
    UNSERIALIZE_SCALAR(rxDmaState);
    this->rxDmaState = (DmaState) rxDmaState;
    UNSERIALIZE_SCALAR(rxKickTick);
    if (rxKickTick)
        schedule(rxKickEvent, rxKickTick);

    /*
     * Unserialize EEPROM state machine
     */
    int eepromState;
    UNSERIALIZE_SCALAR(eepromState);
    this->eepromState = (EEPROMState) eepromState;
    UNSERIALIZE_SCALAR(eepromClk);
    UNSERIALIZE_SCALAR(eepromBitsToRx);
    UNSERIALIZE_SCALAR(eepromOpcode);
    UNSERIALIZE_SCALAR(eepromAddress);
    UNSERIALIZE_SCALAR(eepromData);

    /*
     * If there's a pending transmit, reschedule it now
     */
    Tick transmitTick;
    UNSERIALIZE_SCALAR(transmitTick);
    if (transmitTick)
        schedule(txEvent, curTick() + transmitTick);

    /*
     * unserialize receive address filter settings
     */
    UNSERIALIZE_SCALAR(rxFilterEnable);
    UNSERIALIZE_SCALAR(acceptBroadcast);
    UNSERIALIZE_SCALAR(acceptMulticast);
    UNSERIALIZE_SCALAR(acceptUnicast);
    UNSERIALIZE_SCALAR(acceptPerfect);
    UNSERIALIZE_SCALAR(acceptArp);
    UNSERIALIZE_SCALAR(multicastHashEnable);

    /*
     * Keep track of pending interrupt status.
     */
    UNSERIALIZE_SCALAR(intrTick);
    UNSERIALIZE_SCALAR(cpuPendingIntr);
    Tick intrEventTick;
    UNSERIALIZE_SCALAR(intrEventTick);
    if (intrEventTick) {
        intrEvent = new IntrEvent(this, true);
        schedule(intrEvent, intrEventTick);
    }
}

NSGigE *
NSGigEParams::create()
{
    return new NSGigE(this);
}

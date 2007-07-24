/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

/* @file
 * Device model for Intel's 8254x line of gigabit ethernet controllers.
 * In particular an 82547 revision 2 (82547GI) MAC because it seems to have the
 * fewest workarounds in the driver. It will probably work with most of the
 * other MACs with slight modifications.
 */


/*
 * @todo really there are multiple dma engines.. we should implement them.
 */

#include <algorithm>

#include "base/inet.hh"
#include "base/trace.hh"
#include "dev/i8254xGBe.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/IGbE.hh"
#include "params/IGbEInt.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace iGbReg;
using namespace Net;

IGbE::IGbE(Params *p)
    : PciDev(p), etherInt(NULL),  drainEvent(NULL), useFlowControl(p->use_flow_control),
      rxFifo(p->rx_fifo_size), txFifo(p->tx_fifo_size), rxTick(false),
      txTick(false), txFifoTick(false), rdtrEvent(this), radvEvent(this),
      tadvEvent(this), tidvEvent(this), tickEvent(this), interEvent(this),
      rxDescCache(this, name()+".RxDesc", p->rx_desc_cache_size),
      txDescCache(this, name()+".TxDesc", p->tx_desc_cache_size), clock(p->clock)
{
    // Initialized internal registers per Intel documentation
    // All registers intialized to 0 by per register constructor
    regs.ctrl.fd(1);
    regs.ctrl.lrst(1);
    regs.ctrl.speed(2);
    regs.ctrl.frcspd(1);
    regs.sts.speed(3); // Say we're 1000Mbps
    regs.sts.fd(1); // full duplex
    regs.sts.lu(1); // link up
    regs.eecd.fwe(1);
    regs.eecd.ee_type(1);
    regs.imr = 0;
    regs.iam = 0;
    regs.rxdctl.gran(1);
    regs.rxdctl.wthresh(1);
    regs.fcrth(1);

    regs.pba.rxa(0x30);
    regs.pba.txa(0x10);

    eeOpBits            = 0;
    eeAddrBits          = 0;
    eeDataBits          = 0;
    eeOpcode            = 0;

    // clear all 64 16 bit words of the eeprom
    memset(&flash, 0, EEPROM_SIZE*2);

    // Set the MAC address
    memcpy(flash, p->hardware_address.bytes(), ETH_ADDR_LEN);
    for (int x = 0; x < ETH_ADDR_LEN/2; x++)
        flash[x] = htobe(flash[x]);

    uint16_t csum = 0;
    for (int x = 0; x < EEPROM_SIZE; x++)
        csum += htobe(flash[x]);


    // Magic happy checksum value
    flash[EEPROM_SIZE-1] = htobe((uint16_t)(EEPROM_CSUM - csum));

    rxFifo.clear();
    txFifo.clear();
}


Tick
IGbE::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC)
        PciDev::writeConfig(pkt);
    else
        panic("Device specific PCI config space not implemented.\n");

    ///
    /// Some work may need to be done here based for the pci COMMAND bits.
    ///

    return pioDelay;
}

Tick
IGbE::read(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    // Only 32bit accesses allowed
    assert(pkt->getSize() == 4);

    DPRINTF(Ethernet, "Read device register %#X\n", daddr);

    pkt->allocate();

    ///
    /// Handle read of register here
    ///


    switch (daddr) {
      case REG_CTRL:
        pkt->set<uint32_t>(regs.ctrl());
        break;
      case REG_STATUS:
        pkt->set<uint32_t>(regs.sts());
        break;
      case REG_EECD:
        pkt->set<uint32_t>(regs.eecd());
        break;
      case REG_EERD:
        pkt->set<uint32_t>(regs.eerd());
        break;
      case REG_CTRL_EXT:
        pkt->set<uint32_t>(regs.ctrl_ext());
        break;
      case REG_MDIC:
        pkt->set<uint32_t>(regs.mdic());
        break;
      case REG_ICR:
        DPRINTF(Ethernet, "Reading ICR. ICR=%#x IMR=%#x IAM=%#x IAME=%d\n", regs.icr(),
                regs.imr, regs.iam, regs.ctrl_ext.iame());
        pkt->set<uint32_t>(regs.icr());
        if (regs.icr.int_assert() || regs.imr == 0) {
            regs.icr = regs.icr() & ~mask(30);
            DPRINTF(Ethernet, "Cleared ICR. ICR=%#x\n", regs.icr());
        }
        if (regs.ctrl_ext.iame() && regs.icr.int_assert())
            regs.imr &= ~regs.iam;
        chkInterrupt();
        break;
      case REG_ITR:
        pkt->set<uint32_t>(regs.itr());
        break;
      case REG_RCTL:
        pkt->set<uint32_t>(regs.rctl());
        break;
      case REG_FCTTV:
        pkt->set<uint32_t>(regs.fcttv());
        break;
      case REG_TCTL:
        pkt->set<uint32_t>(regs.tctl());
        break;
      case REG_PBA:
        pkt->set<uint32_t>(regs.pba());
        break;
      case REG_WUC:
      case REG_LEDCTL:
        pkt->set<uint32_t>(0); // We don't care, so just return 0
        break;
      case REG_FCRTL:
        pkt->set<uint32_t>(regs.fcrtl());
        break;
      case REG_FCRTH:
        pkt->set<uint32_t>(regs.fcrth());
        break;
      case REG_RDBAL:
        pkt->set<uint32_t>(regs.rdba.rdbal());
        break;
      case REG_RDBAH:
        pkt->set<uint32_t>(regs.rdba.rdbah());
        break;
      case REG_RDLEN:
        pkt->set<uint32_t>(regs.rdlen());
        break;
      case REG_RDH:
        pkt->set<uint32_t>(regs.rdh());
        break;
      case REG_RDT:
        pkt->set<uint32_t>(regs.rdt());
        break;
      case REG_RDTR:
        pkt->set<uint32_t>(regs.rdtr());
        if (regs.rdtr.fpd()) {
            rxDescCache.writeback(0);
            DPRINTF(EthernetIntr, "Posting interrupt because of RDTR.FPD write\n");
            postInterrupt(IT_RXT);
            regs.rdtr.fpd(0);
        }
        break;
      case REG_RADV:
        pkt->set<uint32_t>(regs.radv());
        break;
      case REG_TDBAL:
        pkt->set<uint32_t>(regs.tdba.tdbal());
        break;
      case REG_TDBAH:
        pkt->set<uint32_t>(regs.tdba.tdbah());
        break;
      case REG_TDLEN:
        pkt->set<uint32_t>(regs.tdlen());
        break;
      case REG_TDH:
        pkt->set<uint32_t>(regs.tdh());
        break;
      case REG_TDT:
        pkt->set<uint32_t>(regs.tdt());
        break;
      case REG_TIDV:
        pkt->set<uint32_t>(regs.tidv());
        break;
      case REG_TXDCTL:
        pkt->set<uint32_t>(regs.txdctl());
        break;
      case REG_TADV:
        pkt->set<uint32_t>(regs.tadv());
        break;
      case REG_RXCSUM:
        pkt->set<uint32_t>(regs.rxcsum());
        break;
      case REG_MANC:
        pkt->set<uint32_t>(regs.manc());
        break;
      default:
        if (!(daddr >= REG_VFTA && daddr < (REG_VFTA + VLAN_FILTER_TABLE_SIZE*4)) &&
            !(daddr >= REG_RAL && daddr < (REG_RAL + RCV_ADDRESS_TABLE_SIZE*8)) &&
            !(daddr >= REG_MTA && daddr < (REG_MTA + MULTICAST_TABLE_SIZE*4)) &&
            !(daddr >= REG_CRCERRS && daddr < (REG_CRCERRS + STATS_REGS_SIZE)))
            panic("Read request to unknown register number: %#x\n", daddr);
        else
            pkt->set<uint32_t>(0);
    };

    pkt->result = Packet::Success;
    return pioDelay;
}

Tick
IGbE::write(PacketPtr pkt)
{
    int bar;
    Addr daddr;


    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    // Only 32bit accesses allowed
    assert(pkt->getSize() == sizeof(uint32_t));

    DPRINTF(Ethernet, "Wrote device register %#X value %#X\n", daddr, pkt->get<uint32_t>());

    ///
    /// Handle write of register here
    ///
    uint32_t val = pkt->get<uint32_t>();

    Regs::RCTL oldrctl;
    Regs::TCTL oldtctl;

    switch (daddr) {
      case REG_CTRL:
        regs.ctrl = val;
        if (regs.ctrl.tfce())
            warn("TX Flow control enabled, should implement\n");
        if (regs.ctrl.rfce())
            warn("RX Flow control enabled, should implement\n");
        break;
      case REG_CTRL_EXT:
        regs.ctrl_ext = val;
        break;
      case REG_STATUS:
        regs.sts = val;
        break;
      case REG_EECD:
        int oldClk;
        oldClk = regs.eecd.sk();
        regs.eecd = val;
        // See if this is a eeprom access and emulate accordingly
        if (!oldClk && regs.eecd.sk()) {
            if (eeOpBits < 8) {
                eeOpcode = eeOpcode << 1 | regs.eecd.din();
                eeOpBits++;
            } else if (eeAddrBits < 8 && eeOpcode == EEPROM_READ_OPCODE_SPI) {
                eeAddr = eeAddr << 1 | regs.eecd.din();
                eeAddrBits++;
            } else if (eeDataBits < 16 && eeOpcode == EEPROM_READ_OPCODE_SPI) {
                assert(eeAddr>>1 < EEPROM_SIZE);
                DPRINTF(EthernetEEPROM, "EEPROM bit read: %d word: %#X\n",
                        flash[eeAddr>>1] >> eeDataBits & 0x1, flash[eeAddr>>1]);
                regs.eecd.dout((flash[eeAddr>>1] >> (15-eeDataBits)) & 0x1);
                eeDataBits++;
            } else if (eeDataBits < 8 && eeOpcode == EEPROM_RDSR_OPCODE_SPI) {
                regs.eecd.dout(0);
                eeDataBits++;
            } else
                panic("What's going on with eeprom interface? opcode:"
                       " %#x:%d addr: %#x:%d, data: %d\n", (uint32_t)eeOpcode,
                       (uint32_t)eeOpBits, (uint32_t)eeAddr,
                       (uint32_t)eeAddrBits, (uint32_t)eeDataBits);

            // Reset everything for the next command
            if ((eeDataBits == 16 && eeOpcode == EEPROM_READ_OPCODE_SPI) ||
               (eeDataBits == 8 && eeOpcode == EEPROM_RDSR_OPCODE_SPI)) {
                eeOpBits = 0;
                eeAddrBits = 0;
                eeDataBits = 0;
               eeOpcode = 0;
                eeAddr = 0;
            }

           DPRINTF(EthernetEEPROM, "EEPROM: opcode: %#X:%d addr: %#X:%d\n",
                    (uint32_t)eeOpcode, (uint32_t) eeOpBits,
                    (uint32_t)eeAddr>>1, (uint32_t)eeAddrBits);
           if (eeOpBits == 8 && !(eeOpcode == EEPROM_READ_OPCODE_SPI ||
                                   eeOpcode == EEPROM_RDSR_OPCODE_SPI ))
                panic("Unknown eeprom opcode: %#X:%d\n", (uint32_t)eeOpcode,
                        (uint32_t)eeOpBits);


        }
        // If driver requests eeprom access, immediately give it to it
        regs.eecd.ee_gnt(regs.eecd.ee_req());
        break;
      case REG_EERD:
        regs.eerd = val;
        break;
      case REG_MDIC:
        regs.mdic = val;
        if (regs.mdic.i())
            panic("No support for interrupt on mdic complete\n");
        if (regs.mdic.phyadd() != 1)
            panic("No support for reading anything but phy\n");
        DPRINTF(Ethernet, "%s phy address %x\n", regs.mdic.op() == 1 ? "Writing"
                : "Reading", regs.mdic.regadd());
        switch (regs.mdic.regadd()) {
            case PHY_PSTATUS:
                regs.mdic.data(0x796D); // link up
                break;
            case PHY_PID:
                regs.mdic.data(0x02A8);
                break;
            case PHY_EPID:
                regs.mdic.data(0x0380);
                break;
            case PHY_GSTATUS:
                regs.mdic.data(0x7C00);
                break;
            case PHY_EPSTATUS:
                regs.mdic.data(0x3000);
                break;
            case PHY_AGC:
                regs.mdic.data(0x180); // some random length
                break;
            default:
                regs.mdic.data(0);
        }
        regs.mdic.r(1);
        break;
      case REG_ICR:
        DPRINTF(Ethernet, "Writing ICR. ICR=%#x IMR=%#x IAM=%#x IAME=%d\n", regs.icr(),
                regs.imr, regs.iam, regs.ctrl_ext.iame());
        if (regs.ctrl_ext.iame())
            regs.imr &= ~regs.iam;
        regs.icr = ~bits(val,30,0) & regs.icr();
        chkInterrupt();
        break;
      case REG_ITR:
        regs.itr = val;
        break;
      case REG_ICS:
        DPRINTF(EthernetIntr, "Posting interrupt because of ICS write\n");
        postInterrupt((IntTypes)val);
        break;
       case REG_IMS:
        regs.imr |= val;
        chkInterrupt();
        break;
      case REG_IMC:
        regs.imr &= ~val;
        chkInterrupt();
        break;
      case REG_IAM:
        regs.iam = val;
        break;
      case REG_RCTL:
        oldrctl = regs.rctl;
        regs.rctl = val;
        if (regs.rctl.rst()) {
            rxDescCache.reset();
            DPRINTF(EthernetSM, "RXS: Got RESET!\n");
            rxFifo.clear();
            regs.rctl.rst(0);
        }
        if (regs.rctl.en())
            rxTick = true;
        restartClock();
        break;
      case REG_FCTTV:
        regs.fcttv = val;
        break;
      case REG_TCTL:
        regs.tctl = val;
        oldtctl = regs.tctl;
        regs.tctl = val;
        if (regs.tctl.en())
           txTick = true;
        restartClock();
        if (regs.tctl.en() && !oldtctl.en()) {
            txDescCache.reset();
        }
         break;
      case REG_PBA:
        regs.pba.rxa(val);
        regs.pba.txa(64 - regs.pba.rxa());
        break;
      case REG_WUC:
      case REG_LEDCTL:
      case REG_FCAL:
      case REG_FCAH:
      case REG_FCT:
      case REG_VET:
      case REG_AIFS:
      case REG_TIPG:
        ; // We don't care, so don't store anything
        break;
      case REG_FCRTL:
        regs.fcrtl = val;
        break;
      case REG_FCRTH:
        regs.fcrth = val;
        break;
      case REG_RDBAL:
        regs.rdba.rdbal( val & ~mask(4));
        rxDescCache.areaChanged();
        break;
      case REG_RDBAH:
        regs.rdba.rdbah(val);
        rxDescCache.areaChanged();
        break;
      case REG_RDLEN:
        regs.rdlen = val & ~mask(7);
        rxDescCache.areaChanged();
        break;
      case REG_RDH:
        regs.rdh = val;
        rxDescCache.areaChanged();
        break;
      case REG_RDT:
        regs.rdt = val;
        rxTick = true;
        restartClock();
        break;
      case REG_RDTR:
        regs.rdtr = val;
        break;
      case REG_RADV:
        regs.radv = val;
        break;
      case REG_TDBAL:
        regs.tdba.tdbal( val & ~mask(4));
        txDescCache.areaChanged();
        break;
      case REG_TDBAH:
        regs.tdba.tdbah(val);
        txDescCache.areaChanged();
        break;
      case REG_TDLEN:
        regs.tdlen = val & ~mask(7);
        txDescCache.areaChanged();
        break;
      case REG_TDH:
        regs.tdh = val;
        txDescCache.areaChanged();
        break;
      case REG_TDT:
        regs.tdt = val;
        txTick = true;
        restartClock();
        break;
      case REG_TIDV:
        regs.tidv = val;
        break;
      case REG_TXDCTL:
        regs.txdctl = val;
        break;
      case REG_TADV:
        regs.tadv = val;
        break;
      case REG_RXCSUM:
        regs.rxcsum = val;
        break;
      case REG_MANC:
        regs.manc = val;
        break;
      default:
       if (!(daddr >= REG_VFTA && daddr < (REG_VFTA + VLAN_FILTER_TABLE_SIZE*4)) &&
           !(daddr >= REG_RAL && daddr < (REG_RAL + RCV_ADDRESS_TABLE_SIZE*8)) &&
           !(daddr >= REG_MTA && daddr < (REG_MTA + MULTICAST_TABLE_SIZE*4)))
           panic("Write request to unknown register number: %#x\n", daddr);
    };

    pkt->result = Packet::Success;
    return pioDelay;
}

void
IGbE::postInterrupt(IntTypes t, bool now)
{
    assert(t);

    // Interrupt is already pending
    if (t & regs.icr())
        return;

    if (regs.icr() & regs.imr)
    {
        regs.icr = regs.icr() | t;
        if (!interEvent.scheduled())
            interEvent.schedule(curTick + Clock::Int::ns * 256 *
                    regs.itr.interval());
    } else {
        regs.icr = regs.icr() | t;
        if (regs.itr.interval() == 0 || now) {
            if (interEvent.scheduled())
                interEvent.deschedule();
            cpuPostInt();
        } else {
           DPRINTF(EthernetIntr, "EINT: Scheduling timer interrupt for %d ticks\n",
                    Clock::Int::ns * 256 * regs.itr.interval());
           if (!interEvent.scheduled())
               interEvent.schedule(curTick + Clock::Int::ns * 256 * regs.itr.interval());
        }
    }
}

void
IGbE::cpuPostInt()
{
    if (rdtrEvent.scheduled()) {
        regs.icr.rxt0(1);
        rdtrEvent.deschedule();
    }
    if (radvEvent.scheduled()) {
        regs.icr.rxt0(1);
        radvEvent.deschedule();
    }
    if (tadvEvent.scheduled()) {
        regs.icr.txdw(1);
        tadvEvent.deschedule();
    }
    if (tidvEvent.scheduled()) {
        regs.icr.txdw(1);
        tidvEvent.deschedule();
    }

    regs.icr.int_assert(1);
    DPRINTF(EthernetIntr, "EINT: Posting interrupt to CPU now. Vector %#x\n",
            regs.icr());
    intrPost();
}

void
IGbE::cpuClearInt()
{
    if (regs.icr.int_assert()) {
        regs.icr.int_assert(0);
        DPRINTF(EthernetIntr, "EINT: Clearing interrupt to CPU now. Vector %#x\n",
                regs.icr());
        intrClear();
    }
}

void
IGbE::chkInterrupt()
{
    // Check if we need to clear the cpu interrupt
    if (!(regs.icr() & regs.imr)) {
        if (interEvent.scheduled())
           interEvent.deschedule();
        if (regs.icr.int_assert())
            cpuClearInt();
    }

    if (regs.icr() & regs.imr) {
        if (regs.itr.interval() == 0)  {
            cpuPostInt();
        } else {
            if (!interEvent.scheduled())
               interEvent.schedule(curTick + Clock::Int::ns * 256 * regs.itr.interval());
        }
    }


}


IGbE::RxDescCache::RxDescCache(IGbE *i, const std::string n, int s)
    : DescCache<RxDesc>(i, n, s), pktDone(false), pktEvent(this)

{
}

bool
IGbE::RxDescCache::writePacket(EthPacketPtr packet)
{
    // We shouldn't have to deal with any of these yet
    DPRINTF(EthernetDesc, "Packet Length: %d Desc Size: %d\n",
            packet->length, igbe->regs.rctl.descSize());
    assert(packet->length < igbe->regs.rctl.descSize());

    if (!unusedCache.size())
        return false;

    pktPtr = packet;
    pktDone = false;
    igbe->dmaWrite(igbe->platform->pciToDma(unusedCache.front()->buf),
            packet->length, &pktEvent, packet->data);
    return true;
}

void
IGbE::RxDescCache::pktComplete()
{
    assert(unusedCache.size());
    RxDesc *desc;
    desc = unusedCache.front();

    uint16_t crcfixup = igbe->regs.rctl.secrc() ? 0 : 4 ;
    desc->len = htole((uint16_t)(pktPtr->length + crcfixup));
    DPRINTF(EthernetDesc, "pktPtr->length: %d stripcrc offset: %d value written: %d %d\n",
            pktPtr->length, crcfixup,
            htole((uint16_t)(pktPtr->length + crcfixup)),
            (uint16_t)(pktPtr->length + crcfixup));

    // no support for anything but starting at 0
    assert(igbe->regs.rxcsum.pcss() == 0);

    DPRINTF(EthernetDesc, "Packet written to memory updating Descriptor\n");

    uint8_t status = RXDS_DD | RXDS_EOP;
    uint8_t err = 0;

    IpPtr ip(pktPtr);

    if (ip) {
        DPRINTF(EthernetDesc, "Proccesing Ip packet with Id=%d\n", ip->id());

        if (igbe->regs.rxcsum.ipofld()) {
            DPRINTF(EthernetDesc, "Checking IP checksum\n");
            status |= RXDS_IPCS;
            desc->csum = htole(cksum(ip));
            if (cksum(ip) != 0) {
                err |= RXDE_IPE;
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
            }
        }
        TcpPtr tcp(ip);
        if (tcp && igbe->regs.rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "Checking TCP checksum\n");
            status |= RXDS_TCPCS;
            desc->csum = htole(cksum(tcp));
            if (cksum(tcp) != 0) {
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
                err |= RXDE_TCPE;
            }
        }

        UdpPtr udp(ip);
        if (udp && igbe->regs.rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "Checking UDP checksum\n");
            status |= RXDS_UDPCS;
            desc->csum = htole(cksum(udp));
            if (cksum(udp) != 0) {
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
                err |= RXDE_TCPE;
            }
        }
    } else { // if ip
        DPRINTF(EthernetSM, "Proccesing Non-Ip packet\n");
    }


    desc->status = htole(status);
    desc->errors = htole(err);

    // No vlan support at this point... just set it to 0
    desc->vlan = 0;

    // Deal with the rx timer interrupts
    if (igbe->regs.rdtr.delay()) {
        DPRINTF(EthernetSM, "RXS: Scheduling DTR for %d\n",
                igbe->regs.rdtr.delay() * igbe->intClock());
        igbe->rdtrEvent.reschedule(curTick + igbe->regs.rdtr.delay() *
                    igbe->intClock(),true);
    }

    if (igbe->regs.radv.idv() && igbe->regs.rdtr.delay()) {
        DPRINTF(EthernetSM, "RXS: Scheduling ADV for %d\n",
                igbe->regs.radv.idv() * igbe->intClock());
        if (!igbe->radvEvent.scheduled())
            igbe->radvEvent.schedule(curTick + igbe->regs.radv.idv() *
                    igbe->intClock());
    }

    // if neither radv or rdtr, maybe itr is set...
    if (!igbe->regs.rdtr.delay()) {
        DPRINTF(EthernetSM, "RXS: Receive interrupt delay disabled, posting IT_RXT\n");
        igbe->postInterrupt(IT_RXT);
    }

    // If the packet is small enough, interrupt appropriately
    // I wonder if this is delayed or not?!
    if (pktPtr->length <= igbe->regs.rsrpd.idv()) {
        DPRINTF(EthernetSM, "RXS: Posting IT_SRPD beacuse small packet received\n");
        igbe->postInterrupt(IT_SRPD);
    }

    DPRINTF(EthernetDesc, "Processing of this descriptor complete\n");
    unusedCache.pop_front();
    usedCache.push_back(desc);
    pktPtr = NULL;
    enableSm();
    pktDone = true;
    igbe->checkDrain();
}

void
IGbE::RxDescCache::enableSm()
{
    igbe->rxTick = true;
    igbe->restartClock();
}

bool
IGbE::RxDescCache::packetDone()
{
    if (pktDone) {
        pktDone = false;
        return true;
    }
    return false;
}

bool
IGbE::RxDescCache::hasOutstandingEvents()
{
    return pktEvent.scheduled() || wbEvent.scheduled() ||
        fetchEvent.scheduled();
}

void
IGbE::RxDescCache::serialize(std::ostream &os)
{
    DescCache<RxDesc>::serialize(os);
    SERIALIZE_SCALAR(pktDone);
}

void
IGbE::RxDescCache::unserialize(Checkpoint *cp, const std::string &section)
{
    DescCache<RxDesc>::unserialize(cp, section);
    UNSERIALIZE_SCALAR(pktDone);
}


///////////////////////////////////// IGbE::TxDesc /////////////////////////////////

IGbE::TxDescCache::TxDescCache(IGbE *i, const std::string n, int s)
    : DescCache<TxDesc>(i,n, s), pktDone(false), isTcp(false), pktWaiting(false),
       pktEvent(this)

{
}

int
IGbE::TxDescCache::getPacketSize()
{
    assert(unusedCache.size());

    TxDesc *desc;

    DPRINTF(EthernetDesc, "Starting processing of descriptor\n");

    while (unusedCache.size() && TxdOp::isContext(unusedCache.front())) {
        DPRINTF(EthernetDesc, "Got context descriptor type... skipping\n");

        // I think we can just ignore these for now?
        desc = unusedCache.front();
        // is this going to be a tcp or udp packet?
        isTcp = TxdOp::tcp(desc) ? true : false;

        // make sure it's ipv4
        assert(TxdOp::ip(desc));

        TxdOp::setDd(desc);
        unusedCache.pop_front();
        usedCache.push_back(desc);
    }

    if (!unusedCache.size())
        return -1;

    DPRINTF(EthernetDesc, "Next TX packet is %d bytes\n",
            TxdOp::getLen(unusedCache.front()));

    return TxdOp::getLen(unusedCache.front());
}

void
IGbE::TxDescCache::getPacketData(EthPacketPtr p)
{
    assert(unusedCache.size());

    TxDesc *desc;
    desc = unusedCache.front();

    assert((TxdOp::isLegacy(desc) || TxdOp::isData(desc)) && TxdOp::getLen(desc));

    pktPtr = p;

    pktWaiting = true;

    DPRINTF(EthernetDesc, "Starting DMA of packet\n");
    igbe->dmaRead(igbe->platform->pciToDma(TxdOp::getBuf(desc)),
            TxdOp::getLen(desc), &pktEvent, p->data + p->length);


}

void
IGbE::TxDescCache::pktComplete()
{

    TxDesc *desc;
    assert(unusedCache.size());
    assert(pktPtr);

    DPRINTF(EthernetDesc, "DMA of packet complete\n");


    desc = unusedCache.front();
    assert((TxdOp::isLegacy(desc) || TxdOp::isData(desc)) && TxdOp::getLen(desc));

    DPRINTF(EthernetDesc, "TxDescriptor data d1: %#llx d2: %#llx\n", desc->d1, desc->d2);

    if (!TxdOp::eop(desc)) {
        // This only supports two descriptors per tx packet
        assert(pktPtr->length == 0);
        pktPtr->length = TxdOp::getLen(desc);
        unusedCache.pop_front();
        usedCache.push_back(desc);
        pktDone = true;
        pktWaiting = false;
        pktPtr = NULL;

        DPRINTF(EthernetDesc, "Partial Packet Descriptor Done\n");
        enableSm();
        return;
    }

    // Set the length of the data in the EtherPacket
    pktPtr->length += TxdOp::getLen(desc);

    // no support for vlans
    assert(!TxdOp::vle(desc));

    // we alway report status
    assert(TxdOp::rs(desc));

    // we only support single packet descriptors at this point
    assert(TxdOp::eop(desc));

    // set that this packet is done
    TxdOp::setDd(desc);

    DPRINTF(EthernetDesc, "TxDescriptor data d1: %#llx d2: %#llx\n", desc->d1, desc->d2);

    if (DTRACE(EthernetDesc)) {
        IpPtr ip(pktPtr);
        if (ip)
            DPRINTF(EthernetDesc, "Proccesing Ip packet with Id=%d\n",
                    ip->id());
        else
            DPRINTF(EthernetSM, "Proccesing Non-Ip packet\n");
    }

    // Checksums are only ofloaded for new descriptor types
    if (TxdOp::isData(desc) && ( TxdOp::ixsm(desc) || TxdOp::txsm(desc)) ) {
        DPRINTF(EthernetDesc, "Calculating checksums for packet\n");
        IpPtr ip(pktPtr);

        if (TxdOp::ixsm(desc)) {
            ip->sum(0);
            ip->sum(cksum(ip));
            DPRINTF(EthernetDesc, "Calculated IP checksum\n");
        }
       if (TxdOp::txsm(desc)) {
           if (isTcp) {
                TcpPtr tcp(ip);
                assert(tcp);
                tcp->sum(0);
                tcp->sum(cksum(tcp));
                DPRINTF(EthernetDesc, "Calculated TCP checksum\n");
           } else {
                UdpPtr udp(ip);
                assert(udp);
                udp->sum(0);
                udp->sum(cksum(udp));
                DPRINTF(EthernetDesc, "Calculated UDP checksum\n");
           }
        }
    }

    if (TxdOp::ide(desc)) {
        // Deal with the rx timer interrupts
        DPRINTF(EthernetDesc, "Descriptor had IDE set\n");
        if (igbe->regs.tidv.idv()) {
            DPRINTF(EthernetDesc, "setting tidv\n");
            igbe->tidvEvent.reschedule(curTick + igbe->regs.tidv.idv() *
                        igbe->intClock(), true);
        }

        if (igbe->regs.tadv.idv() && igbe->regs.tidv.idv()) {
            DPRINTF(EthernetDesc, "setting tadv\n");
            if (!igbe->tadvEvent.scheduled())
                igbe->tadvEvent.schedule(curTick + igbe->regs.tadv.idv() *
                        igbe->intClock());
        }
    }



    unusedCache.pop_front();
    usedCache.push_back(desc);
    pktDone = true;
    pktWaiting = false;
    pktPtr = NULL;

    DPRINTF(EthernetDesc, "Descriptor Done\n");

    if (igbe->regs.txdctl.wthresh() == 0) {
        DPRINTF(EthernetDesc, "WTHRESH == 0, writing back descriptor\n");
        writeback(0);
    } else if (igbe->regs.txdctl.wthresh() >= usedCache.size()) {
        DPRINTF(EthernetDesc, "used > WTHRESH, writing back descriptor\n");
        writeback((igbe->cacheBlockSize()-1)>>4);
    }
    enableSm();
    igbe->checkDrain();
}

void
IGbE::TxDescCache::serialize(std::ostream &os)
{
    DescCache<TxDesc>::serialize(os);
    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(isTcp);
    SERIALIZE_SCALAR(pktWaiting);
}

void
IGbE::TxDescCache::unserialize(Checkpoint *cp, const std::string &section)
{
    DescCache<TxDesc>::unserialize(cp, section);
    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(isTcp);
    UNSERIALIZE_SCALAR(pktWaiting);
}

bool
IGbE::TxDescCache::packetAvailable()
{
    if (pktDone) {
        pktDone = false;
        return true;
    }
    return false;
}

void
IGbE::TxDescCache::enableSm()
{
    igbe->txTick = true;
    igbe->restartClock();
}

bool
IGbE::TxDescCache::hasOutstandingEvents()
{
    return pktEvent.scheduled() || wbEvent.scheduled() ||
        fetchEvent.scheduled();
}


///////////////////////////////////// IGbE /////////////////////////////////

void
IGbE::restartClock()
{
    if (!tickEvent.scheduled() && (rxTick || txTick || txFifoTick) && getState() ==
            SimObject::Running)
        tickEvent.schedule((curTick/cycles(1)) * cycles(1) + cycles(1));
}

unsigned int
IGbE::drain(Event *de)
{
    unsigned int count;
    count = pioPort->drain(de) + dmaPort->drain(de);
    if (rxDescCache.hasOutstandingEvents() ||
            txDescCache.hasOutstandingEvents()) {
        count++;
        drainEvent = de;
    }

    txFifoTick = false;
    txTick = false;
    rxTick = false;

    if (tickEvent.scheduled())
        tickEvent.deschedule();

    if (count)
        changeState(Draining);
    else
        changeState(Drained);

    return count;
}

void
IGbE::resume()
{
    SimObject::resume();

    txFifoTick = true;
    txTick = true;
    rxTick = true;

    restartClock();
}

void
IGbE::checkDrain()
{
    if (!drainEvent)
        return;

    if (rxDescCache.hasOutstandingEvents() ||
            txDescCache.hasOutstandingEvents()) {
        drainEvent->process();
        drainEvent = NULL;
    }
}

void
IGbE::txStateMachine()
{
    if (!regs.tctl.en()) {
        txTick = false;
        DPRINTF(EthernetSM, "TXS: TX disabled, stopping ticking\n");
        return;
    }

    // If we have a packet available and it's length is not 0 (meaning it's not
    // a multidescriptor packet) put it in the fifo, otherwise an the next
    // iteration we'll get the rest of the data
    if (txPacket && txDescCache.packetAvailable() && txPacket->length) {
        bool success;
        DPRINTF(EthernetSM, "TXS: packet placed in TX FIFO\n");
        success = txFifo.push(txPacket);
        txFifoTick = true;
        assert(success);
        txPacket = NULL;
        txDescCache.writeback((cacheBlockSize()-1)>>4);
        return;
    }

    // Only support descriptor granularity
    assert(regs.txdctl.gran());
    if (regs.txdctl.lwthresh() && txDescCache.descLeft() < (regs.txdctl.lwthresh() * 8)) {
        DPRINTF(EthernetSM, "TXS: LWTHRESH caused posting of TXDLOW\n");
        postInterrupt(IT_TXDLOW);
    }

    if (!txPacket) {
        txPacket = new EthPacketData(16384);
    }

    if (!txDescCache.packetWaiting()) {
        if (txDescCache.descLeft() == 0) {
            DPRINTF(EthernetSM, "TXS: No descriptors left in ring, forcing "
                    "writeback stopping ticking and posting TXQE\n");
            txDescCache.writeback(0);
            txTick = false;
            postInterrupt(IT_TXQE, true);
            return;
        }


        if (!(txDescCache.descUnused())) {
            DPRINTF(EthernetSM, "TXS: No descriptors available in cache, fetching and stopping ticking\n");
            txTick = false;
            txDescCache.fetchDescriptors();
            return;
        }

        int size;
        size = txDescCache.getPacketSize();
        if (size > 0 && txFifo.avail() > size) {
            DPRINTF(EthernetSM, "TXS: Reserving %d bytes in FIFO and begining "
                    "DMA of next packet\n", size);
            txFifo.reserve(size);
            txDescCache.getPacketData(txPacket);
        } else if (size <= 0) {
            DPRINTF(EthernetSM, "TXS: No packets to get, writing back used descriptors\n");
            txDescCache.writeback(0);
        } else {
            DPRINTF(EthernetSM, "TXS: FIFO full, stopping ticking until space "
                    "available in FIFO\n");
            txDescCache.writeback((cacheBlockSize()-1)>>4);
            txTick = false;
        }


        return;
    }
    DPRINTF(EthernetSM, "TXS: Nothing to do, stopping ticking\n");
    txTick = false;
}

bool
IGbE::ethRxPkt(EthPacketPtr pkt)
{
    DPRINTF(Ethernet, "RxFIFO: Receiving pcakte from wire\n");
    if (!regs.rctl.en()) {
        DPRINTF(Ethernet, "RxFIFO: RX not enabled, dropping\n");
        return true;
    }

    // restart the state machines if they are stopped
    rxTick = true;
    if ((rxTick || txTick) && !tickEvent.scheduled()) {
        DPRINTF(EthernetSM, "RXS: received packet into fifo, starting ticking\n");
        restartClock();
    }

    if (!rxFifo.push(pkt)) {
        DPRINTF(Ethernet, "RxFIFO: Packet won't fit in fifo... dropped\n");
        postInterrupt(IT_RXO, true);
        return false;
    }
    return true;
}


void
IGbE::rxStateMachine()
{
    if (!regs.rctl.en()) {
        rxTick = false;
        DPRINTF(EthernetSM, "RXS: RX disabled, stopping ticking\n");
        return;
    }

    // If the packet is done check for interrupts/descriptors/etc
    if (rxDescCache.packetDone()) {
        rxDmaPacket = false;
        DPRINTF(EthernetSM, "RXS: Packet completed DMA to memory\n");
        int descLeft = rxDescCache.descLeft();
        switch (regs.rctl.rdmts()) {
            case 2: if (descLeft > .125 * regs.rdlen()) break;
            case 1: if (descLeft > .250 * regs.rdlen()) break;
            case 0: if (descLeft > .500 * regs.rdlen())  break;
                DPRINTF(Ethernet, "RXS: Interrupting (RXDMT) because of descriptors left\n");
                postInterrupt(IT_RXDMT);
                break;
        }

        if (descLeft == 0) {
            DPRINTF(EthernetSM, "RXS: No descriptors left in ring, forcing"
                    " writeback and stopping ticking\n");
            rxDescCache.writeback(0);
            rxTick = false;
        }

        // only support descriptor granulaties
        assert(regs.rxdctl.gran());

        if (regs.rxdctl.wthresh() >= rxDescCache.descUsed()) {
            DPRINTF(EthernetSM, "RXS: Writing back because WTHRESH >= descUsed\n");
            if (regs.rxdctl.wthresh() < (cacheBlockSize()>>4))
                rxDescCache.writeback(regs.rxdctl.wthresh()-1);
            else
                rxDescCache.writeback((cacheBlockSize()-1)>>4);
        }

        if ((rxDescCache.descUnused() < regs.rxdctl.pthresh()) &&
             ((rxDescCache.descLeft() - rxDescCache.descUnused()) > regs.rxdctl.hthresh())) {
            DPRINTF(EthernetSM, "RXS: Fetching descriptors because descUnused < PTHRESH\n");
            rxDescCache.fetchDescriptors();
        }

        if (rxDescCache.descUnused() == 0) {
            DPRINTF(EthernetSM, "RXS: No descriptors available in cache, "
                    "fetching descriptors and stopping ticking\n");
            rxTick = false;
            rxDescCache.fetchDescriptors();
        }
        return;
    }

    if (rxDmaPacket) {
        DPRINTF(EthernetSM, "RXS: stopping ticking until packet DMA completes\n");
        rxTick = false;
        return;
    }

    if (!rxDescCache.descUnused()) {
        DPRINTF(EthernetSM, "RXS: No descriptors available in cache, stopping ticking\n");
        rxTick = false;
        DPRINTF(EthernetSM, "RXS: No descriptors available, fetching\n");
        rxDescCache.fetchDescriptors();
        return;
    }

    if (rxFifo.empty()) {
        DPRINTF(EthernetSM, "RXS: RxFIFO empty, stopping ticking\n");
        rxTick = false;
        return;
    }

    EthPacketPtr pkt;
    pkt = rxFifo.front();

    DPRINTF(EthernetSM, "RXS: Writing packet into memory\n");
    if (!rxDescCache.writePacket(pkt)) {
        return;
    }

    DPRINTF(EthernetSM, "RXS: Removing packet from FIFO\n");
    rxFifo.pop();
    DPRINTF(EthernetSM, "RXS: stopping ticking until packet DMA completes\n");
    rxTick = false;
    rxDmaPacket = true;
}

void
IGbE::txWire()
{
    if (txFifo.empty()) {
        txFifoTick = false;
        return;
    }


    if (etherInt->sendPacket(txFifo.front())) {
        DPRINTF(EthernetSM, "TxFIFO: Successful transmit, bytes available in fifo: %d\n",
                txFifo.avail());
        txFifo.pop();
    } else {
        // We'll get woken up when the packet ethTxDone() gets called
        txFifoTick = false;
    }

}

void
IGbE::tick()
{
    DPRINTF(EthernetSM, "IGbE: -------------- Cycle --------------\n");

    if (rxTick)
        rxStateMachine();

    if (txTick)
        txStateMachine();

    if (txFifoTick)
        txWire();


    if (rxTick || txTick || txFifoTick)
        tickEvent.schedule(curTick + cycles(1));
}

void
IGbE::ethTxDone()
{
    // restart the tx state machines if they are stopped
    // fifo to send another packet
    // tx sm to put more data into the fifo
    txFifoTick = true;
    txTick = true;

    restartClock();
    DPRINTF(EthernetSM, "TxFIFO: Transmission complete\n");
}

void
IGbE::serialize(std::ostream &os)
{
    PciDev::serialize(os);

    regs.serialize(os);
    SERIALIZE_SCALAR(eeOpBits);
    SERIALIZE_SCALAR(eeAddrBits);
    SERIALIZE_SCALAR(eeDataBits);
    SERIALIZE_SCALAR(eeOpcode);
    SERIALIZE_SCALAR(eeAddr);
    SERIALIZE_ARRAY(flash,iGbReg::EEPROM_SIZE);

    rxFifo.serialize("rxfifo", os);
    txFifo.serialize("txfifo", os);

    bool txPktExists = txPacket;
    SERIALIZE_SCALAR(txPktExists);
    if (txPktExists)
        txPacket->serialize("txpacket", os);

    Tick rdtr_time = 0, radv_time = 0, tidv_time = 0, tadv_time = 0,
         inter_time = 0;

    if (rdtrEvent.scheduled())
       rdtr_time = rdtrEvent.when();
    SERIALIZE_SCALAR(rdtr_time);

    if (radvEvent.scheduled())
       radv_time = radvEvent.when();
    SERIALIZE_SCALAR(radv_time);

    if (tidvEvent.scheduled())
       rdtr_time = tidvEvent.when();
    SERIALIZE_SCALAR(tidv_time);

    if (tadvEvent.scheduled())
       rdtr_time = tadvEvent.when();
    SERIALIZE_SCALAR(tadv_time);

    if (interEvent.scheduled())
       rdtr_time = interEvent.when();
    SERIALIZE_SCALAR(inter_time);

    nameOut(os, csprintf("%s.TxDescCache", name()));
    txDescCache.serialize(os);

    nameOut(os, csprintf("%s.RxDescCache", name()));
    rxDescCache.serialize(os);
}

void
IGbE::unserialize(Checkpoint *cp, const std::string &section)
{
    PciDev::unserialize(cp, section);

    regs.unserialize(cp, section);
    UNSERIALIZE_SCALAR(eeOpBits);
    UNSERIALIZE_SCALAR(eeAddrBits);
    UNSERIALIZE_SCALAR(eeDataBits);
    UNSERIALIZE_SCALAR(eeOpcode);
    UNSERIALIZE_SCALAR(eeAddr);
    UNSERIALIZE_ARRAY(flash,iGbReg::EEPROM_SIZE);

    rxFifo.unserialize("rxfifo", cp, section);
    txFifo.unserialize("txfifo", cp, section);

    bool txPktExists;
    UNSERIALIZE_SCALAR(txPktExists);
    if (txPktExists) {
        txPacket = new EthPacketData(16384);
        txPacket->unserialize("txpacket", cp, section);
    }

    rxTick = true;
    txTick = true;
    txFifoTick = true;

    Tick rdtr_time, radv_time, tidv_time, tadv_time, inter_time;
    UNSERIALIZE_SCALAR(rdtr_time);
    UNSERIALIZE_SCALAR(radv_time);
    UNSERIALIZE_SCALAR(tidv_time);
    UNSERIALIZE_SCALAR(tadv_time);
    UNSERIALIZE_SCALAR(inter_time);

    if (rdtr_time)
        rdtrEvent.schedule(rdtr_time);

    if (radv_time)
        radvEvent.schedule(radv_time);

    if (tidv_time)
        tidvEvent.schedule(tidv_time);

    if (tadv_time)
        tadvEvent.schedule(tadv_time);

    if (inter_time)
        interEvent.schedule(inter_time);

    txDescCache.unserialize(cp, csprintf("%s.TxDescCache", section));

    rxDescCache.unserialize(cp, csprintf("%s.RxDescCache", section));
}

IGbEInt *
IGbEIntParams::create()
{
    IGbEInt *dev_int = new IGbEInt(name, device);

    EtherInt *p = (EtherInt *)peer;
    if (p) {
        dev_int->setPeer(p);
        p->setPeer(dev_int);
    }

    return dev_int;
}

IGbE *
IGbEParams::create()
{
    return new IGbE(this);
}

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

#include "dev/net/i8254xGBe.hh"

/*
 * @todo really there are multiple dma engines.. we should implement them.
 */

#include <algorithm>
#include <memory>

#include "base/inet.hh"
#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/EthernetAll.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/IGbE.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace iGbReg;
using namespace Net;

IGbE::IGbE(const Params *p)
    : EtherDevice(p), etherInt(NULL), cpa(NULL),
      rxFifo(p->rx_fifo_size), txFifo(p->tx_fifo_size), inTick(false),
      rxTick(false), txTick(false), txFifoTick(false), rxDmaPacket(false),
      pktOffset(0), fetchDelay(p->fetch_delay), wbDelay(p->wb_delay),
      fetchCompDelay(p->fetch_comp_delay), wbCompDelay(p->wb_comp_delay),
      rxWriteDelay(p->rx_write_delay), txReadDelay(p->tx_read_delay),
      rdtrEvent([this]{ rdtrProcess(); }, name()),
      radvEvent([this]{ radvProcess(); }, name()),
      tadvEvent([this]{ tadvProcess(); }, name()),
      tidvEvent([this]{ tidvProcess(); }, name()),
      tickEvent([this]{ tick(); }, name()),
      interEvent([this]{ delayIntEvent(); }, name()),
      rxDescCache(this, name()+".RxDesc", p->rx_desc_cache_size),
      txDescCache(this, name()+".TxDesc", p->tx_desc_cache_size),
      lastInterrupt(0)
{
    etherInt = new IGbEInt(name() + ".int", this);

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
    regs.tdwba = 0;
    regs.rlpml = 0;
    regs.sw_fw_sync = 0;

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

    // Store the MAC address as queue ID
    macAddr = p->hardware_address;

    rxFifo.clear();
    txFifo.clear();
}

IGbE::~IGbE()
{
    delete etherInt;
}

void
IGbE::init()
{
    cpa = CPA::cpa();
    PciDevice::init();
}

EtherInt*
IGbE::getEthPort(const std::string &if_name, int idx)
{

    if (if_name == "interface") {
        if (etherInt->getPeer())
            panic("Port already connected to\n");
        return etherInt;
    }
    return NULL;
}

Tick
IGbE::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC)
        PciDevice::writeConfig(pkt);
    else
        panic("Device specific PCI config space not implemented.\n");

    //
    // Some work may need to be done here based for the pci COMMAND bits.
    //

    return configDelay;
}

// Handy macro for range-testing register access addresses
#define IN_RANGE(val, base, len) (val >= base && val < (base + len))

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

    //
    // Handle read of register here
    //


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
        DPRINTF(Ethernet, "Reading ICR. ICR=%#x IMR=%#x IAM=%#x IAME=%d\n",
                regs.icr(), regs.imr, regs.iam, regs.ctrl_ext.iame());
        pkt->set<uint32_t>(regs.icr());
        if (regs.icr.int_assert() || regs.imr == 0) {
            regs.icr = regs.icr() & ~mask(30);
            DPRINTF(Ethernet, "Cleared ICR. ICR=%#x\n", regs.icr());
        }
        if (regs.ctrl_ext.iame() && regs.icr.int_assert())
            regs.imr &= ~regs.iam;
        chkInterrupt();
        break;
      case REG_EICR:
        // This is only useful for MSI, but the driver reads it every time
        // Just don't do anything
        pkt->set<uint32_t>(0);
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
      case REG_WUFC:
      case REG_WUS:
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
      case REG_SRRCTL:
        pkt->set<uint32_t>(regs.srrctl());
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
            DPRINTF(EthernetIntr,
                    "Posting interrupt because of RDTR.FPD write\n");
            postInterrupt(IT_RXT);
            regs.rdtr.fpd(0);
        }
        break;
      case REG_RXDCTL:
        pkt->set<uint32_t>(regs.rxdctl());
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
      case REG_TXDCA_CTL:
        pkt->set<uint32_t>(regs.txdca_ctl());
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
      case REG_TDWBAL:
        pkt->set<uint32_t>(regs.tdwba & mask(32));
        break;
      case REG_TDWBAH:
        pkt->set<uint32_t>(regs.tdwba >> 32);
        break;
      case REG_RXCSUM:
        pkt->set<uint32_t>(regs.rxcsum());
        break;
      case REG_RLPML:
        pkt->set<uint32_t>(regs.rlpml);
        break;
      case REG_RFCTL:
        pkt->set<uint32_t>(regs.rfctl());
        break;
      case REG_MANC:
        pkt->set<uint32_t>(regs.manc());
        break;
      case REG_SWSM:
        pkt->set<uint32_t>(regs.swsm());
        regs.swsm.smbi(1);
        break;
      case REG_FWSM:
        pkt->set<uint32_t>(regs.fwsm());
        break;
      case REG_SWFWSYNC:
        pkt->set<uint32_t>(regs.sw_fw_sync);
        break;
      default:
        if (!IN_RANGE(daddr, REG_VFTA, VLAN_FILTER_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_RAL, RCV_ADDRESS_TABLE_SIZE*8) &&
            !IN_RANGE(daddr, REG_MTA, MULTICAST_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_CRCERRS, STATS_REGS_SIZE))
            panic("Read request to unknown register number: %#x\n", daddr);
        else
            pkt->set<uint32_t>(0);
    };

    pkt->makeAtomicResponse();
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

    DPRINTF(Ethernet, "Wrote device register %#X value %#X\n",
            daddr, pkt->get<uint32_t>());

    //
    // Handle write of register here
    //
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
                        flash[eeAddr>>1] >> eeDataBits & 0x1,
                        flash[eeAddr>>1]);
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
        if (regs.eerd.start()) {
            regs.eerd.done(1);
            assert(regs.eerd.addr() < EEPROM_SIZE);
            regs.eerd.data(flash[regs.eerd.addr()]);
            regs.eerd.start(0);
            DPRINTF(EthernetEEPROM, "EEPROM: read addr: %#X data %#x\n",
                    regs.eerd.addr(), regs.eerd.data());
        }
        break;
      case REG_MDIC:
        regs.mdic = val;
        if (regs.mdic.i())
            panic("No support for interrupt on mdic complete\n");
        if (regs.mdic.phyadd() != 1)
            panic("No support for reading anything but phy\n");
        DPRINTF(Ethernet, "%s phy address %x\n",
                regs.mdic.op() == 1 ? "Writing" : "Reading",
                regs.mdic.regadd());
        switch (regs.mdic.regadd()) {
          case PHY_PSTATUS:
            regs.mdic.data(0x796D); // link up
            break;
          case PHY_PID:
            regs.mdic.data(params()->phy_pid);
            break;
          case PHY_EPID:
            regs.mdic.data(params()->phy_epid);
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
        DPRINTF(Ethernet, "Writing ICR. ICR=%#x IMR=%#x IAM=%#x IAME=%d\n",
                regs.icr(), regs.imr, regs.iam, regs.ctrl_ext.iame());
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
      case REG_WUFC:
      case REG_WUS:
      case REG_LEDCTL:
      case REG_FCAL:
      case REG_FCAH:
      case REG_FCT:
      case REG_VET:
      case REG_AIFS:
      case REG_TIPG:
        ; // We don't care, so don't store anything
        break;
      case REG_IVAR0:
        warn("Writing to IVAR0, ignoring...\n");
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
      case REG_SRRCTL:
        regs.srrctl = val;
        break;
      case REG_RDH:
        regs.rdh = val;
        rxDescCache.areaChanged();
        break;
      case REG_RDT:
        regs.rdt = val;
        DPRINTF(EthernetSM, "RXS: RDT Updated.\n");
        if (drainState() == DrainState::Running) {
            DPRINTF(EthernetSM, "RXS: RDT Fetching Descriptors!\n");
            rxDescCache.fetchDescriptors();
        } else {
            DPRINTF(EthernetSM, "RXS: RDT NOT Fetching Desc b/c draining!\n");
        }
        break;
      case REG_RDTR:
        regs.rdtr = val;
        break;
      case REG_RADV:
        regs.radv = val;
        break;
      case REG_RXDCTL:
        regs.rxdctl = val;
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
      case REG_TXDCA_CTL:
        regs.txdca_ctl = val;
        if (regs.txdca_ctl.enabled())
            panic("No support for DCA\n");
        break;
      case REG_TDT:
        regs.tdt = val;
        DPRINTF(EthernetSM, "TXS: TX Tail pointer updated\n");
        if (drainState() == DrainState::Running) {
            DPRINTF(EthernetSM, "TXS: TDT Fetching Descriptors!\n");
            txDescCache.fetchDescriptors();
        } else {
            DPRINTF(EthernetSM, "TXS: TDT NOT Fetching Desc b/c draining!\n");
        }
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
      case REG_TDWBAL:
        regs.tdwba &= ~mask(32);
        regs.tdwba |= val;
        txDescCache.completionWriteback(regs.tdwba & ~mask(1),
                                        regs.tdwba & mask(1));
        break;
      case REG_TDWBAH:
        regs.tdwba &= mask(32);
        regs.tdwba |= (uint64_t)val << 32;
        txDescCache.completionWriteback(regs.tdwba & ~mask(1),
                                        regs.tdwba & mask(1));
        break;
      case REG_RXCSUM:
        regs.rxcsum = val;
        break;
      case REG_RLPML:
        regs.rlpml = val;
        break;
      case REG_RFCTL:
        regs.rfctl = val;
        if (regs.rfctl.exsten())
            panic("Extended RX descriptors not implemented\n");
        break;
      case REG_MANC:
        regs.manc = val;
        break;
      case REG_SWSM:
        regs.swsm = val;
        if (regs.fwsm.eep_fw_semaphore())
            regs.swsm.swesmbi(0);
        break;
      case REG_SWFWSYNC:
        regs.sw_fw_sync = val;
        break;
      default:
        if (!IN_RANGE(daddr, REG_VFTA, VLAN_FILTER_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_RAL, RCV_ADDRESS_TABLE_SIZE*8) &&
            !IN_RANGE(daddr, REG_MTA, MULTICAST_TABLE_SIZE*4))
            panic("Write request to unknown register number: %#x\n", daddr);
    };

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
IGbE::postInterrupt(IntTypes t, bool now)
{
    assert(t);

    // Interrupt is already pending
    if (t & regs.icr() && !now)
        return;

    regs.icr = regs.icr() | t;

    Tick itr_interval = SimClock::Int::ns * 256 * regs.itr.interval();
    DPRINTF(EthernetIntr,
            "EINT: postInterrupt() curTick(): %d itr: %d interval: %d\n",
            curTick(), regs.itr.interval(), itr_interval);

    if (regs.itr.interval() == 0 || now ||
        lastInterrupt + itr_interval <= curTick()) {
        if (interEvent.scheduled()) {
            deschedule(interEvent);
        }
        cpuPostInt();
    } else {
        Tick int_time = lastInterrupt + itr_interval;
        assert(int_time > 0);
        DPRINTF(EthernetIntr, "EINT: Scheduling timer interrupt for tick %d\n",
                int_time);
        if (!interEvent.scheduled()) {
            schedule(interEvent, int_time);
        }
    }
}

void
IGbE::delayIntEvent()
{
    cpuPostInt();
}


void
IGbE::cpuPostInt()
{

    postedInterrupts++;

    if (!(regs.icr() & regs.imr)) {
        DPRINTF(Ethernet, "Interrupt Masked. Not Posting\n");
        return;
    }

    DPRINTF(Ethernet, "Posting Interrupt\n");


    if (interEvent.scheduled()) {
        deschedule(interEvent);
    }

    if (rdtrEvent.scheduled()) {
        regs.icr.rxt0(1);
        deschedule(rdtrEvent);
    }
    if (radvEvent.scheduled()) {
        regs.icr.rxt0(1);
        deschedule(radvEvent);
    }
    if (tadvEvent.scheduled()) {
        regs.icr.txdw(1);
        deschedule(tadvEvent);
    }
    if (tidvEvent.scheduled()) {
        regs.icr.txdw(1);
        deschedule(tidvEvent);
    }

    regs.icr.int_assert(1);
    DPRINTF(EthernetIntr, "EINT: Posting interrupt to CPU now. Vector %#x\n",
            regs.icr());

    intrPost();

    lastInterrupt = curTick();
}

void
IGbE::cpuClearInt()
{
    if (regs.icr.int_assert()) {
        regs.icr.int_assert(0);
        DPRINTF(EthernetIntr,
                "EINT: Clearing interrupt to CPU now. Vector %#x\n",
                regs.icr());
        intrClear();
    }
}

void
IGbE::chkInterrupt()
{
    DPRINTF(Ethernet, "Checking interrupts icr: %#x imr: %#x\n", regs.icr(),
            regs.imr);
    // Check if we need to clear the cpu interrupt
    if (!(regs.icr() & regs.imr)) {
        DPRINTF(Ethernet, "Mask cleaned all interrupts\n");
        if (interEvent.scheduled())
            deschedule(interEvent);
        if (regs.icr.int_assert())
            cpuClearInt();
    }
    DPRINTF(Ethernet, "ITR = %#X itr.interval = %#X\n",
            regs.itr(), regs.itr.interval());

    if (regs.icr() & regs.imr) {
        if (regs.itr.interval() == 0)  {
            cpuPostInt();
        } else {
            DPRINTF(Ethernet,
                    "Possibly scheduling interrupt because of imr write\n");
            if (!interEvent.scheduled()) {
                Tick t = curTick() + SimClock::Int::ns * 256 * regs.itr.interval();
                DPRINTF(Ethernet, "Scheduling for %d\n", t);
                schedule(interEvent, t);
            }
        }
    }
}


///////////////////////////// IGbE::DescCache //////////////////////////////

template<class T>
IGbE::DescCache<T>::DescCache(IGbE *i, const std::string n, int s)
    : igbe(i), _name(n), cachePnt(0), size(s), curFetching(0),
      wbOut(0), moreToWb(false), wbAlignment(0), pktPtr(NULL),
      wbDelayEvent([this]{ writeback1(); }, n),
      fetchDelayEvent([this]{ fetchDescriptors1(); }, n),
      fetchEvent([this]{ fetchComplete(); }, n),
      wbEvent([this]{ wbComplete(); }, n)
{
    fetchBuf = new T[size];
    wbBuf = new T[size];
}

template<class T>
IGbE::DescCache<T>::~DescCache()
{
    reset();
    delete[] fetchBuf;
    delete[] wbBuf;
}

template<class T>
void
IGbE::DescCache<T>::areaChanged()
{
    if (usedCache.size() > 0 || curFetching || wbOut)
        panic("Descriptor Address, Length or Head changed. Bad\n");
    reset();

}

template<class T>
void
IGbE::DescCache<T>::writeback(Addr aMask)
{
    int curHead = descHead();
    int max_to_wb = usedCache.size();

    // Check if this writeback is less restrictive that the previous
    // and if so setup another one immediately following it
    if (wbOut) {
        if (aMask < wbAlignment) {
            moreToWb = true;
            wbAlignment = aMask;
        }
        DPRINTF(EthernetDesc,
                "Writing back already in process, returning\n");
        return;
    }

    moreToWb = false;
    wbAlignment = aMask;


    DPRINTF(EthernetDesc, "Writing back descriptors head: %d tail: "
            "%d len: %d cachePnt: %d max_to_wb: %d descleft: %d\n",
            curHead, descTail(), descLen(), cachePnt, max_to_wb,
            descLeft());

    if (max_to_wb + curHead >= descLen()) {
        max_to_wb = descLen() - curHead;
        moreToWb = true;
        // this is by definition aligned correctly
    } else if (wbAlignment != 0) {
        // align the wb point to the mask
        max_to_wb = max_to_wb & ~wbAlignment;
    }

    DPRINTF(EthernetDesc, "Writing back %d descriptors\n", max_to_wb);

    if (max_to_wb <= 0) {
        if (usedCache.size())
            igbe->anBegin(annSmWb, "Wait Alignment", CPA::FL_WAIT);
        else
            igbe->anWe(annSmWb, annUsedCacheQ);
        return;
    }

    wbOut = max_to_wb;

    assert(!wbDelayEvent.scheduled());
    igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);
    igbe->anBegin(annSmWb, "Prepare Writeback Desc");
}

template<class T>
void
IGbE::DescCache<T>::writeback1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);
        return;
    }

    DPRINTF(EthernetDesc, "Begining DMA of %d descriptors\n", wbOut);

    for (int x = 0; x < wbOut; x++) {
        assert(usedCache.size());
        memcpy(&wbBuf[x], usedCache[x], sizeof(T));
        igbe->anPq(annSmWb, annUsedCacheQ);
        igbe->anPq(annSmWb, annDescQ);
        igbe->anQ(annSmWb, annUsedDescQ);
    }


    igbe->anBegin(annSmWb, "Writeback Desc DMA");

    assert(wbOut);
    igbe->dmaWrite(pciToDma(descBase() + descHead() * sizeof(T)),
                   wbOut * sizeof(T), &wbEvent, (uint8_t*)wbBuf,
                   igbe->wbCompDelay);
}

template<class T>
void
IGbE::DescCache<T>::fetchDescriptors()
{
    size_t max_to_fetch;

    if (curFetching) {
        DPRINTF(EthernetDesc,
                "Currently fetching %d descriptors, returning\n",
                curFetching);
        return;
    }

    if (descTail() >= cachePnt)
        max_to_fetch = descTail() - cachePnt;
    else
        max_to_fetch = descLen() - cachePnt;

    size_t free_cache = size - usedCache.size() - unusedCache.size();

    if (!max_to_fetch)
        igbe->anWe(annSmFetch, annUnusedDescQ);
    else
        igbe->anPq(annSmFetch, annUnusedDescQ, max_to_fetch);

    if (max_to_fetch) {
        if (!free_cache)
            igbe->anWf(annSmFetch, annDescQ);
        else
            igbe->anRq(annSmFetch, annDescQ, free_cache);
    }

    max_to_fetch = std::min(max_to_fetch, free_cache);


    DPRINTF(EthernetDesc, "Fetching descriptors head: %d tail: "
            "%d len: %d cachePnt: %d max_to_fetch: %d descleft: %d\n",
            descHead(), descTail(), descLen(), cachePnt,
            max_to_fetch, descLeft());

    // Nothing to do
    if (max_to_fetch == 0)
        return;

    // So we don't have two descriptor fetches going on at once
    curFetching = max_to_fetch;

    assert(!fetchDelayEvent.scheduled());
    igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
    igbe->anBegin(annSmFetch, "Prepare Fetch Desc");
}

template<class T>
void
IGbE::DescCache<T>::fetchDescriptors1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
        return;
    }

    igbe->anBegin(annSmFetch, "Fetch Desc");

    DPRINTF(EthernetDesc, "Fetching descriptors at %#x (%#x), size: %#x\n",
            descBase() + cachePnt * sizeof(T),
            pciToDma(descBase() + cachePnt * sizeof(T)),
            curFetching * sizeof(T));
    assert(curFetching);
    igbe->dmaRead(pciToDma(descBase() + cachePnt * sizeof(T)),
                  curFetching * sizeof(T), &fetchEvent, (uint8_t*)fetchBuf,
                  igbe->fetchCompDelay);
}

template<class T>
void
IGbE::DescCache<T>::fetchComplete()
{
    T *newDesc;
    igbe->anBegin(annSmFetch, "Fetch Complete");
    for (int x = 0; x < curFetching; x++) {
        newDesc = new T;
        memcpy(newDesc, &fetchBuf[x], sizeof(T));
        unusedCache.push_back(newDesc);
        igbe->anDq(annSmFetch, annUnusedDescQ);
        igbe->anQ(annSmFetch, annUnusedCacheQ);
        igbe->anQ(annSmFetch, annDescQ);
    }


#ifndef NDEBUG
    int oldCp = cachePnt;
#endif

    cachePnt += curFetching;
    assert(cachePnt <= descLen());
    if (cachePnt == descLen())
        cachePnt = 0;

    curFetching = 0;

    DPRINTF(EthernetDesc, "Fetching complete cachePnt %d -> %d\n",
            oldCp, cachePnt);

    if ((descTail() >= cachePnt ? (descTail() - cachePnt) : (descLen() -
                                                             cachePnt)) == 0)
    {
        igbe->anWe(annSmFetch, annUnusedDescQ);
    } else if (!(size - usedCache.size() - unusedCache.size())) {
        igbe->anWf(annSmFetch, annDescQ);
    } else {
        igbe->anBegin(annSmFetch, "Wait", CPA::FL_WAIT);
    }

    enableSm();
    igbe->checkDrain();
}

template<class T>
void
IGbE::DescCache<T>::wbComplete()
{

    igbe->anBegin(annSmWb, "Finish Writeback");

    long  curHead = descHead();
#ifndef NDEBUG
    long oldHead = curHead;
#endif

    for (int x = 0; x < wbOut; x++) {
        assert(usedCache.size());
        delete usedCache[0];
        usedCache.pop_front();

        igbe->anDq(annSmWb, annUsedCacheQ);
        igbe->anDq(annSmWb, annDescQ);
    }

    curHead += wbOut;
    wbOut = 0;

    if (curHead >= descLen())
        curHead -= descLen();

    // Update the head
    updateHead(curHead);

    DPRINTF(EthernetDesc, "Writeback complete curHead %d -> %d\n",
            oldHead, curHead);

    // If we still have more to wb, call wb now
    actionAfterWb();
    if (moreToWb) {
        moreToWb = false;
        DPRINTF(EthernetDesc, "Writeback has more todo\n");
        writeback(wbAlignment);
    }

    if (!wbOut) {
        igbe->checkDrain();
        if (usedCache.size())
            igbe->anBegin(annSmWb, "Wait", CPA::FL_WAIT);
        else
            igbe->anWe(annSmWb, annUsedCacheQ);
    }
    fetchAfterWb();
}

template<class T>
void
IGbE::DescCache<T>::reset()
{
    DPRINTF(EthernetDesc, "Reseting descriptor cache\n");
    for (typename CacheType::size_type x = 0; x < usedCache.size(); x++)
        delete usedCache[x];
    for (typename CacheType::size_type x = 0; x < unusedCache.size(); x++)
        delete unusedCache[x];

    usedCache.clear();
    unusedCache.clear();

    cachePnt = 0;

}

template<class T>
void
IGbE::DescCache<T>::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(cachePnt);
    SERIALIZE_SCALAR(curFetching);
    SERIALIZE_SCALAR(wbOut);
    SERIALIZE_SCALAR(moreToWb);
    SERIALIZE_SCALAR(wbAlignment);

    typename CacheType::size_type usedCacheSize = usedCache.size();
    SERIALIZE_SCALAR(usedCacheSize);
    for (typename CacheType::size_type x = 0; x < usedCacheSize; x++) {
        arrayParamOut(cp, csprintf("usedCache_%d", x),
                      (uint8_t*)usedCache[x],sizeof(T));
    }

    typename CacheType::size_type unusedCacheSize = unusedCache.size();
    SERIALIZE_SCALAR(unusedCacheSize);
    for (typename CacheType::size_type x = 0; x < unusedCacheSize; x++) {
        arrayParamOut(cp, csprintf("unusedCache_%d", x),
                      (uint8_t*)unusedCache[x],sizeof(T));
    }

    Tick fetch_delay = 0, wb_delay = 0;
    if (fetchDelayEvent.scheduled())
        fetch_delay = fetchDelayEvent.when();
    SERIALIZE_SCALAR(fetch_delay);
    if (wbDelayEvent.scheduled())
        wb_delay = wbDelayEvent.when();
    SERIALIZE_SCALAR(wb_delay);


}

template<class T>
void
IGbE::DescCache<T>::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(cachePnt);
    UNSERIALIZE_SCALAR(curFetching);
    UNSERIALIZE_SCALAR(wbOut);
    UNSERIALIZE_SCALAR(moreToWb);
    UNSERIALIZE_SCALAR(wbAlignment);

    typename CacheType::size_type usedCacheSize;
    UNSERIALIZE_SCALAR(usedCacheSize);
    T *temp;
    for (typename CacheType::size_type x = 0; x < usedCacheSize; x++) {
        temp = new T;
        arrayParamIn(cp, csprintf("usedCache_%d", x),
                     (uint8_t*)temp,sizeof(T));
        usedCache.push_back(temp);
    }

    typename CacheType::size_type unusedCacheSize;
    UNSERIALIZE_SCALAR(unusedCacheSize);
    for (typename CacheType::size_type x = 0; x < unusedCacheSize; x++) {
        temp = new T;
        arrayParamIn(cp, csprintf("unusedCache_%d", x),
                     (uint8_t*)temp,sizeof(T));
        unusedCache.push_back(temp);
    }
    Tick fetch_delay = 0, wb_delay = 0;
    UNSERIALIZE_SCALAR(fetch_delay);
    UNSERIALIZE_SCALAR(wb_delay);
    if (fetch_delay)
        igbe->schedule(fetchDelayEvent, fetch_delay);
    if (wb_delay)
        igbe->schedule(wbDelayEvent, wb_delay);


}

///////////////////////////// IGbE::RxDescCache //////////////////////////////

IGbE::RxDescCache::RxDescCache(IGbE *i, const std::string n, int s)
    : DescCache<RxDesc>(i, n, s), pktDone(false), splitCount(0),
    pktEvent([this]{ pktComplete(); }, n),
    pktHdrEvent([this]{ pktSplitDone(); }, n),
    pktDataEvent([this]{ pktSplitDone(); }, n)

{
    annSmFetch = "RX Desc Fetch";
    annSmWb = "RX Desc Writeback";
    annUnusedDescQ = "RX Unused Descriptors";
    annUnusedCacheQ = "RX Unused Descriptor Cache";
    annUsedCacheQ = "RX Used Descriptor Cache";
    annUsedDescQ = "RX Used Descriptors";
    annDescQ = "RX Descriptors";
}

void
IGbE::RxDescCache::pktSplitDone()
{
    splitCount++;
    DPRINTF(EthernetDesc,
            "Part of split packet done: splitcount now %d\n", splitCount);
    assert(splitCount <= 2);
    if (splitCount != 2)
        return;
    splitCount = 0;
    DPRINTF(EthernetDesc,
            "Part of split packet done: calling pktComplete()\n");
    pktComplete();
}

int
IGbE::RxDescCache::writePacket(EthPacketPtr packet, int pkt_offset)
{
    assert(unusedCache.size());
    //if (!unusedCache.size())
    //    return false;

    pktPtr = packet;
    pktDone = false;
    unsigned buf_len, hdr_len;

    RxDesc *desc = unusedCache.front();
    switch (igbe->regs.srrctl.desctype()) {
      case RXDT_LEGACY:
        assert(pkt_offset == 0);
        bytesCopied = packet->length;
        DPRINTF(EthernetDesc, "Packet Length: %d Desc Size: %d\n",
                packet->length, igbe->regs.rctl.descSize());
        assert(packet->length < igbe->regs.rctl.descSize());
        igbe->dmaWrite(pciToDma(desc->legacy.buf),
                       packet->length, &pktEvent, packet->data,
                       igbe->rxWriteDelay);
        break;
      case RXDT_ADV_ONEBUF:
        assert(pkt_offset == 0);
        bytesCopied = packet->length;
        buf_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl.bufLen() :
            igbe->regs.rctl.descSize();
        DPRINTF(EthernetDesc, "Packet Length: %d srrctl: %#x Desc Size: %d\n",
                packet->length, igbe->regs.srrctl(), buf_len);
        assert(packet->length < buf_len);
        igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
                       packet->length, &pktEvent, packet->data,
                       igbe->rxWriteDelay);
        desc->adv_wb.header_len = htole(0);
        desc->adv_wb.sph = htole(0);
        desc->adv_wb.pkt_len = htole((uint16_t)(pktPtr->length));
        break;
      case RXDT_ADV_SPLIT_A:
        int split_point;

        buf_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl.bufLen() :
            igbe->regs.rctl.descSize();
        hdr_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl.hdrLen() : 0;
        DPRINTF(EthernetDesc,
                "lpe: %d Packet Length: %d offset: %d srrctl: %#x "
                "hdr addr: %#x Hdr Size: %d desc addr: %#x Desc Size: %d\n",
                igbe->regs.rctl.lpe(), packet->length, pkt_offset,
                igbe->regs.srrctl(), desc->adv_read.hdr, hdr_len,
                desc->adv_read.pkt, buf_len);

        split_point = hsplit(pktPtr);

        if (packet->length <= hdr_len) {
            bytesCopied = packet->length;
            assert(pkt_offset == 0);
            DPRINTF(EthernetDesc, "Hdr split: Entire packet in header\n");
            igbe->dmaWrite(pciToDma(desc->adv_read.hdr),
                           packet->length, &pktEvent, packet->data,
                           igbe->rxWriteDelay);
            desc->adv_wb.header_len = htole((uint16_t)packet->length);
            desc->adv_wb.sph = htole(0);
            desc->adv_wb.pkt_len = htole(0);
        } else if (split_point) {
            if (pkt_offset) {
                // we are only copying some data, header/data has already been
                // copied
                int max_to_copy =
                    std::min(packet->length - pkt_offset, buf_len);
                bytesCopied += max_to_copy;
                DPRINTF(EthernetDesc,
                        "Hdr split: Continuing data buffer copy\n");
                igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
                               max_to_copy, &pktEvent,
                               packet->data + pkt_offset, igbe->rxWriteDelay);
                desc->adv_wb.header_len = htole(0);
                desc->adv_wb.pkt_len = htole((uint16_t)max_to_copy);
                desc->adv_wb.sph = htole(0);
            } else {
                int max_to_copy =
                    std::min(packet->length - split_point, buf_len);
                bytesCopied += max_to_copy + split_point;

                DPRINTF(EthernetDesc, "Hdr split: splitting at %d\n",
                        split_point);
                igbe->dmaWrite(pciToDma(desc->adv_read.hdr),
                               split_point, &pktHdrEvent,
                               packet->data, igbe->rxWriteDelay);
                igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
                               max_to_copy, &pktDataEvent,
                               packet->data + split_point, igbe->rxWriteDelay);
                desc->adv_wb.header_len = htole(split_point);
                desc->adv_wb.sph = 1;
                desc->adv_wb.pkt_len = htole((uint16_t)(max_to_copy));
            }
        } else {
            panic("Header split not fitting within header buffer or "
                  "undecodable packet not fitting in header unsupported\n");
        }
        break;
      default:
        panic("Unimplemnted RX receive buffer type: %d\n",
              igbe->regs.srrctl.desctype());
    }
    return bytesCopied;

}

void
IGbE::RxDescCache::pktComplete()
{
    assert(unusedCache.size());
    RxDesc *desc;
    desc = unusedCache.front();

    igbe->anBegin("RXS", "Update Desc");

    uint16_t crcfixup = igbe->regs.rctl.secrc() ? 0 : 4 ;
    DPRINTF(EthernetDesc, "pktPtr->length: %d bytesCopied: %d "
            "stripcrc offset: %d value written: %d %d\n",
            pktPtr->length, bytesCopied, crcfixup,
            htole((uint16_t)(pktPtr->length + crcfixup)),
            (uint16_t)(pktPtr->length + crcfixup));

    // no support for anything but starting at 0
    assert(igbe->regs.rxcsum.pcss() == 0);

    DPRINTF(EthernetDesc, "Packet written to memory updating Descriptor\n");

    uint16_t status = RXDS_DD;
    uint8_t err = 0;
    uint16_t ext_err = 0;
    uint16_t csum = 0;
    uint16_t ptype = 0;
    uint16_t ip_id = 0;

    assert(bytesCopied <= pktPtr->length);
    if (bytesCopied == pktPtr->length)
        status |= RXDS_EOP;

    IpPtr ip(pktPtr);

    if (ip) {
        DPRINTF(EthernetDesc, "Proccesing Ip packet with Id=%d\n", ip->id());
        ptype |= RXDP_IPV4;
        ip_id = ip->id();

        if (igbe->regs.rxcsum.ipofld()) {
            DPRINTF(EthernetDesc, "Checking IP checksum\n");
            status |= RXDS_IPCS;
            csum = htole(cksum(ip));
            igbe->rxIpChecksums++;
            if (cksum(ip) != 0) {
                err |= RXDE_IPE;
                ext_err |= RXDEE_IPE;
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
            }
        }
        TcpPtr tcp(ip);
        if (tcp && igbe->regs.rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "Checking TCP checksum\n");
            status |= RXDS_TCPCS;
            ptype |= RXDP_TCP;
            csum = htole(cksum(tcp));
            igbe->rxTcpChecksums++;
            if (cksum(tcp) != 0) {
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
                err |= RXDE_TCPE;
                ext_err |= RXDEE_TCPE;
            }
        }

        UdpPtr udp(ip);
        if (udp && igbe->regs.rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "Checking UDP checksum\n");
            status |= RXDS_UDPCS;
            ptype |= RXDP_UDP;
            csum = htole(cksum(udp));
            igbe->rxUdpChecksums++;
            if (cksum(udp) != 0) {
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
                ext_err |= RXDEE_TCPE;
                err |= RXDE_TCPE;
            }
        }
    } else { // if ip
        DPRINTF(EthernetSM, "Proccesing Non-Ip packet\n");
    }

    switch (igbe->regs.srrctl.desctype()) {
      case RXDT_LEGACY:
        desc->legacy.len = htole((uint16_t)(pktPtr->length + crcfixup));
        desc->legacy.status = htole(status);
        desc->legacy.errors = htole(err);
        // No vlan support at this point... just set it to 0
        desc->legacy.vlan = 0;
        break;
      case RXDT_ADV_SPLIT_A:
      case RXDT_ADV_ONEBUF:
        desc->adv_wb.rss_type = htole(0);
        desc->adv_wb.pkt_type = htole(ptype);
        if (igbe->regs.rxcsum.pcsd()) {
            // no rss support right now
            desc->adv_wb.rss_hash = htole(0);
        } else {
            desc->adv_wb.id = htole(ip_id);
            desc->adv_wb.csum = htole(csum);
        }
        desc->adv_wb.status = htole(status);
        desc->adv_wb.errors = htole(ext_err);
        // no vlan support
        desc->adv_wb.vlan_tag = htole(0);
        break;
      default:
        panic("Unimplemnted RX receive buffer type %d\n",
              igbe->regs.srrctl.desctype());
    }

    DPRINTF(EthernetDesc, "Descriptor complete w0: %#x w1: %#x\n",
            desc->adv_read.pkt, desc->adv_read.hdr);

    if (bytesCopied == pktPtr->length) {
        DPRINTF(EthernetDesc,
                "Packet completely written to descriptor buffers\n");
        // Deal with the rx timer interrupts
        if (igbe->regs.rdtr.delay()) {
            Tick delay = igbe->regs.rdtr.delay() * igbe->intClock();
            DPRINTF(EthernetSM, "RXS: Scheduling DTR for %d\n", delay);
            igbe->reschedule(igbe->rdtrEvent, curTick() + delay);
        }

        if (igbe->regs.radv.idv()) {
            Tick delay = igbe->regs.radv.idv() * igbe->intClock();
            DPRINTF(EthernetSM, "RXS: Scheduling ADV for %d\n", delay);
            if (!igbe->radvEvent.scheduled()) {
                igbe->schedule(igbe->radvEvent, curTick() + delay);
            }
        }

        // if neither radv or rdtr, maybe itr is set...
        if (!igbe->regs.rdtr.delay() && !igbe->regs.radv.idv()) {
            DPRINTF(EthernetSM,
                    "RXS: Receive interrupt delay disabled, posting IT_RXT\n");
            igbe->postInterrupt(IT_RXT);
        }

        // If the packet is small enough, interrupt appropriately
        // I wonder if this is delayed or not?!
        if (pktPtr->length <= igbe->regs.rsrpd.idv()) {
            DPRINTF(EthernetSM,
                    "RXS: Posting IT_SRPD beacuse small packet received\n");
            igbe->postInterrupt(IT_SRPD);
        }
        bytesCopied = 0;
    }

    pktPtr = NULL;
    igbe->checkDrain();
    enableSm();
    pktDone = true;

    igbe->anBegin("RXS", "Done Updating Desc");
    DPRINTF(EthernetDesc, "Processing of this descriptor complete\n");
    igbe->anDq("RXS", annUnusedCacheQ);
    unusedCache.pop_front();
    igbe->anQ("RXS", annUsedCacheQ);
    usedCache.push_back(desc);
}

void
IGbE::RxDescCache::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        igbe->rxTick = true;
        igbe->restartClock();
    }
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
        fetchEvent.scheduled() || pktHdrEvent.scheduled() ||
        pktDataEvent.scheduled();

}

void
IGbE::RxDescCache::serialize(CheckpointOut &cp) const
{
    DescCache<RxDesc>::serialize(cp);
    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(splitCount);
    SERIALIZE_SCALAR(bytesCopied);
}

void
IGbE::RxDescCache::unserialize(CheckpointIn &cp)
{
    DescCache<RxDesc>::unserialize(cp);
    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(splitCount);
    UNSERIALIZE_SCALAR(bytesCopied);
}


///////////////////////////// IGbE::TxDescCache //////////////////////////////

IGbE::TxDescCache::TxDescCache(IGbE *i, const std::string n, int s)
    : DescCache<TxDesc>(i,n, s), pktDone(false), isTcp(false),
      pktWaiting(false), pktMultiDesc(false),
      completionAddress(0), completionEnabled(false),
      useTso(false), tsoHeaderLen(0), tsoMss(0), tsoTotalLen(0), tsoUsedLen(0),
      tsoPrevSeq(0), tsoPktPayloadBytes(0), tsoLoadedHeader(false),
      tsoPktHasHeader(false), tsoDescBytesUsed(0), tsoCopyBytes(0), tsoPkts(0),
    pktEvent([this]{ pktComplete(); }, n),
    headerEvent([this]{ headerComplete(); }, n),
    nullEvent([this]{ nullCallback(); }, n)
{
    annSmFetch = "TX Desc Fetch";
    annSmWb = "TX Desc Writeback";
    annUnusedDescQ = "TX Unused Descriptors";
    annUnusedCacheQ = "TX Unused Descriptor Cache";
    annUsedCacheQ = "TX Used Descriptor Cache";
    annUsedDescQ = "TX Used Descriptors";
    annDescQ = "TX Descriptors";
}

void
IGbE::TxDescCache::processContextDesc()
{
    assert(unusedCache.size());
    TxDesc *desc;

    DPRINTF(EthernetDesc, "Checking and  processing context descriptors\n");

    while (!useTso && unusedCache.size() &&
           TxdOp::isContext(unusedCache.front())) {
        DPRINTF(EthernetDesc, "Got context descriptor type...\n");

        desc = unusedCache.front();
        DPRINTF(EthernetDesc, "Descriptor upper: %#x lower: %#X\n",
                desc->d1, desc->d2);


        // is this going to be a tcp or udp packet?
        isTcp = TxdOp::tcp(desc) ? true : false;

        // setup all the TSO variables, they'll be ignored if we don't use
        // tso for this connection
        tsoHeaderLen = TxdOp::hdrlen(desc);
        tsoMss  = TxdOp::mss(desc);

        if (TxdOp::isType(desc, TxdOp::TXD_CNXT) && TxdOp::tse(desc)) {
            DPRINTF(EthernetDesc, "TCP offload enabled for packet hdrlen: "
                    "%d mss: %d paylen %d\n", TxdOp::hdrlen(desc),
                    TxdOp::mss(desc), TxdOp::getLen(desc));
            useTso = true;
            tsoTotalLen = TxdOp::getLen(desc);
            tsoLoadedHeader = false;
            tsoDescBytesUsed = 0;
            tsoUsedLen = 0;
            tsoPrevSeq = 0;
            tsoPktHasHeader = false;
            tsoPkts = 0;
            tsoCopyBytes = 0;
        }

        TxdOp::setDd(desc);
        unusedCache.pop_front();
        igbe->anDq("TXS", annUnusedCacheQ);
        usedCache.push_back(desc);
        igbe->anQ("TXS", annUsedCacheQ);
    }

    if (!unusedCache.size())
        return;

    desc = unusedCache.front();
    if (!useTso && TxdOp::isType(desc, TxdOp::TXD_ADVDATA) &&
        TxdOp::tse(desc)) {
        DPRINTF(EthernetDesc, "TCP offload(adv) enabled for packet "
                "hdrlen: %d mss: %d paylen %d\n",
                tsoHeaderLen, tsoMss, TxdOp::getTsoLen(desc));
        useTso = true;
        tsoTotalLen = TxdOp::getTsoLen(desc);
        tsoLoadedHeader = false;
        tsoDescBytesUsed = 0;
        tsoUsedLen = 0;
        tsoPrevSeq = 0;
        tsoPktHasHeader = false;
        tsoPkts = 0;
    }

    if (useTso && !tsoLoadedHeader) {
        // we need to fetch a header
        DPRINTF(EthernetDesc, "Starting DMA of TSO header\n");
        assert(TxdOp::isData(desc) && TxdOp::getLen(desc) >= tsoHeaderLen);
        pktWaiting = true;
        assert(tsoHeaderLen <= 256);
        igbe->dmaRead(pciToDma(TxdOp::getBuf(desc)),
                      tsoHeaderLen, &headerEvent, tsoHeader, 0);
    }
}

void
IGbE::TxDescCache::headerComplete()
{
    DPRINTF(EthernetDesc, "TSO: Fetching TSO header complete\n");
    pktWaiting = false;

    assert(unusedCache.size());
    TxDesc *desc = unusedCache.front();
    DPRINTF(EthernetDesc, "TSO: len: %d tsoHeaderLen: %d\n",
            TxdOp::getLen(desc), tsoHeaderLen);

    if (TxdOp::getLen(desc) == tsoHeaderLen) {
        tsoDescBytesUsed = 0;
        tsoLoadedHeader = true;
        unusedCache.pop_front();
        usedCache.push_back(desc);
    } else {
        DPRINTF(EthernetDesc, "TSO: header part of larger payload\n");
        tsoDescBytesUsed = tsoHeaderLen;
        tsoLoadedHeader = true;
    }
    enableSm();
    igbe->checkDrain();
}

unsigned
IGbE::TxDescCache::getPacketSize(EthPacketPtr p)
{
    if (!unusedCache.size())
        return 0;

    DPRINTF(EthernetDesc, "Starting processing of descriptor\n");

    assert(!useTso || tsoLoadedHeader);
    TxDesc *desc = unusedCache.front();

    if (useTso) {
        DPRINTF(EthernetDesc, "getPacket(): TxDescriptor data "
                "d1: %#llx d2: %#llx\n", desc->d1, desc->d2);
        DPRINTF(EthernetDesc, "TSO: use: %d hdrlen: %d mss: %d total: %d "
                "used: %d loaded hdr: %d\n", useTso, tsoHeaderLen, tsoMss,
                tsoTotalLen, tsoUsedLen, tsoLoadedHeader);

        if (tsoPktHasHeader)
            tsoCopyBytes =  std::min((tsoMss + tsoHeaderLen) - p->length,
                                     TxdOp::getLen(desc) - tsoDescBytesUsed);
        else
            tsoCopyBytes =  std::min(tsoMss,
                                     TxdOp::getLen(desc) - tsoDescBytesUsed);
        unsigned pkt_size =
            tsoCopyBytes + (tsoPktHasHeader ? 0 : tsoHeaderLen);

        DPRINTF(EthernetDesc, "TSO: descBytesUsed: %d copyBytes: %d "
                "this descLen: %d\n",
                tsoDescBytesUsed, tsoCopyBytes, TxdOp::getLen(desc));
        DPRINTF(EthernetDesc, "TSO: pktHasHeader: %d\n", tsoPktHasHeader);
        DPRINTF(EthernetDesc, "TSO: Next packet is %d bytes\n", pkt_size);
        return pkt_size;
    }

    DPRINTF(EthernetDesc, "Next TX packet is %d bytes\n",
            TxdOp::getLen(unusedCache.front()));
    return TxdOp::getLen(desc);
}

void
IGbE::TxDescCache::getPacketData(EthPacketPtr p)
{
    assert(unusedCache.size());

    TxDesc *desc;
    desc = unusedCache.front();

    DPRINTF(EthernetDesc, "getPacketData(): TxDescriptor data "
            "d1: %#llx d2: %#llx\n", desc->d1, desc->d2);
    assert((TxdOp::isLegacy(desc) || TxdOp::isData(desc)) &&
           TxdOp::getLen(desc));

    pktPtr = p;

    pktWaiting = true;

    DPRINTF(EthernetDesc, "Starting DMA of packet at offset %d\n", p->length);

    if (useTso) {
        assert(tsoLoadedHeader);
        if (!tsoPktHasHeader) {
            DPRINTF(EthernetDesc,
                    "Loading TSO header (%d bytes) into start of packet\n",
                    tsoHeaderLen);
            memcpy(p->data, &tsoHeader,tsoHeaderLen);
            p->length +=tsoHeaderLen;
            tsoPktHasHeader = true;
        }
    }

    if (useTso) {
        DPRINTF(EthernetDesc,
                "Starting DMA of packet at offset %d length: %d\n",
                p->length, tsoCopyBytes);
        igbe->dmaRead(pciToDma(TxdOp::getBuf(desc))
                      + tsoDescBytesUsed,
                      tsoCopyBytes, &pktEvent, p->data + p->length,
                      igbe->txReadDelay);
        tsoDescBytesUsed += tsoCopyBytes;
        assert(tsoDescBytesUsed <= TxdOp::getLen(desc));
    } else {
        igbe->dmaRead(pciToDma(TxdOp::getBuf(desc)),
                      TxdOp::getLen(desc), &pktEvent, p->data + p->length,
                      igbe->txReadDelay);
    }
}

void
IGbE::TxDescCache::pktComplete()
{

    TxDesc *desc;
    assert(unusedCache.size());
    assert(pktPtr);

    igbe->anBegin("TXS", "Update Desc");

    DPRINTF(EthernetDesc, "DMA of packet complete\n");


    desc = unusedCache.front();
    assert((TxdOp::isLegacy(desc) || TxdOp::isData(desc)) &&
           TxdOp::getLen(desc));

    DPRINTF(EthernetDesc, "TxDescriptor data d1: %#llx d2: %#llx\n",
            desc->d1, desc->d2);

    // Set the length of the data in the EtherPacket
    if (useTso) {
        DPRINTF(EthernetDesc, "TSO: use: %d hdrlen: %d mss: %d total: %d "
            "used: %d loaded hdr: %d\n", useTso, tsoHeaderLen, tsoMss,
            tsoTotalLen, tsoUsedLen, tsoLoadedHeader);
        pktPtr->simLength += tsoCopyBytes;
        pktPtr->length += tsoCopyBytes;
        tsoUsedLen += tsoCopyBytes;
        DPRINTF(EthernetDesc, "TSO: descBytesUsed: %d copyBytes: %d\n",
            tsoDescBytesUsed, tsoCopyBytes);
    } else {
        pktPtr->simLength += TxdOp::getLen(desc);
        pktPtr->length += TxdOp::getLen(desc);
    }



    if ((!TxdOp::eop(desc) && !useTso) ||
        (pktPtr->length < ( tsoMss + tsoHeaderLen) &&
         tsoTotalLen != tsoUsedLen && useTso)) {
        assert(!useTso || (tsoDescBytesUsed == TxdOp::getLen(desc)));
        igbe->anDq("TXS", annUnusedCacheQ);
        unusedCache.pop_front();
        igbe->anQ("TXS", annUsedCacheQ);
        usedCache.push_back(desc);

        tsoDescBytesUsed = 0;
        pktDone = true;
        pktWaiting = false;
        pktMultiDesc = true;

        DPRINTF(EthernetDesc, "Partial Packet Descriptor of %d bytes Done\n",
                pktPtr->length);
        pktPtr = NULL;

        enableSm();
        igbe->checkDrain();
        return;
    }


    pktMultiDesc = false;
    // no support for vlans
    assert(!TxdOp::vle(desc));

    // we only support single packet descriptors at this point
    if (!useTso)
        assert(TxdOp::eop(desc));

    // set that this packet is done
    if (TxdOp::rs(desc))
        TxdOp::setDd(desc);

    DPRINTF(EthernetDesc, "TxDescriptor data d1: %#llx d2: %#llx\n",
            desc->d1, desc->d2);

    if (useTso) {
        IpPtr ip(pktPtr);
        if (ip) {
            DPRINTF(EthernetDesc, "TSO: Modifying IP header. Id + %d\n",
                    tsoPkts);
            ip->id(ip->id() + tsoPkts++);
            ip->len(pktPtr->length - EthPtr(pktPtr)->size());

            TcpPtr tcp(ip);
            if (tcp) {
                DPRINTF(EthernetDesc,
                        "TSO: Modifying TCP header. old seq %d + %d\n",
                        tcp->seq(), tsoPrevSeq);
                tcp->seq(tcp->seq() + tsoPrevSeq);
                if (tsoUsedLen != tsoTotalLen)
                    tcp->flags(tcp->flags() & ~9); // clear fin & psh
            }
            UdpPtr udp(ip);
            if (udp) {
                DPRINTF(EthernetDesc, "TSO: Modifying UDP header.\n");
                udp->len(pktPtr->length - EthPtr(pktPtr)->size());
            }
        }
        tsoPrevSeq = tsoUsedLen;
    }

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
        assert(ip);
        if (TxdOp::ixsm(desc)) {
            ip->sum(0);
            ip->sum(cksum(ip));
            igbe->txIpChecksums++;
            DPRINTF(EthernetDesc, "Calculated IP checksum\n");
        }
        if (TxdOp::txsm(desc)) {
            TcpPtr tcp(ip);
            UdpPtr udp(ip);
            if (tcp) {
                tcp->sum(0);
                tcp->sum(cksum(tcp));
                igbe->txTcpChecksums++;
                DPRINTF(EthernetDesc, "Calculated TCP checksum\n");
            } else if (udp) {
                assert(udp);
                udp->sum(0);
                udp->sum(cksum(udp));
                igbe->txUdpChecksums++;
                DPRINTF(EthernetDesc, "Calculated UDP checksum\n");
            } else {
                panic("Told to checksum, but don't know how\n");
            }
        }
    }

    if (TxdOp::ide(desc)) {
        // Deal with the rx timer interrupts
        DPRINTF(EthernetDesc, "Descriptor had IDE set\n");
        if (igbe->regs.tidv.idv()) {
            Tick delay = igbe->regs.tidv.idv() * igbe->intClock();
            DPRINTF(EthernetDesc, "setting tidv\n");
            igbe->reschedule(igbe->tidvEvent, curTick() + delay, true);
        }

        if (igbe->regs.tadv.idv() && igbe->regs.tidv.idv()) {
            Tick delay = igbe->regs.tadv.idv() * igbe->intClock();
            DPRINTF(EthernetDesc, "setting tadv\n");
            if (!igbe->tadvEvent.scheduled()) {
                igbe->schedule(igbe->tadvEvent, curTick() + delay);
            }
        }
    }


    if (!useTso ||  TxdOp::getLen(desc) == tsoDescBytesUsed) {
        DPRINTF(EthernetDesc, "Descriptor Done\n");
        igbe->anDq("TXS", annUnusedCacheQ);
        unusedCache.pop_front();
        igbe->anQ("TXS", annUsedCacheQ);
        usedCache.push_back(desc);
        tsoDescBytesUsed = 0;
    }

    if (useTso && tsoUsedLen == tsoTotalLen)
        useTso = false;


    DPRINTF(EthernetDesc,
            "------Packet of %d bytes ready for transmission-------\n",
            pktPtr->length);
    pktDone = true;
    pktWaiting = false;
    pktPtr = NULL;
    tsoPktHasHeader = false;

    if (igbe->regs.txdctl.wthresh() == 0) {
        igbe->anBegin("TXS", "Desc Writeback");
        DPRINTF(EthernetDesc, "WTHRESH == 0, writing back descriptor\n");
        writeback(0);
    } else if (!igbe->regs.txdctl.gran() && igbe->regs.txdctl.wthresh() <=
               descInBlock(usedCache.size())) {
        DPRINTF(EthernetDesc, "used > WTHRESH, writing back descriptor\n");
        igbe->anBegin("TXS", "Desc Writeback");
        writeback((igbe->cacheBlockSize()-1)>>4);
    } else if (igbe->regs.txdctl.wthresh() <= usedCache.size()) {
        DPRINTF(EthernetDesc, "used > WTHRESH, writing back descriptor\n");
        igbe->anBegin("TXS", "Desc Writeback");
        writeback((igbe->cacheBlockSize()-1)>>4);
    }

    enableSm();
    igbe->checkDrain();
}

void
IGbE::TxDescCache::actionAfterWb()
{
    DPRINTF(EthernetDesc, "actionAfterWb() completionEnabled: %d\n",
            completionEnabled);
    igbe->postInterrupt(iGbReg::IT_TXDW);
    if (completionEnabled) {
        descEnd = igbe->regs.tdh();
        DPRINTF(EthernetDesc,
                "Completion writing back value: %d to addr: %#x\n", descEnd,
                completionAddress);
        igbe->dmaWrite(pciToDma(mbits(completionAddress, 63, 2)),
                       sizeof(descEnd), &nullEvent, (uint8_t*)&descEnd, 0);
    }
}

void
IGbE::TxDescCache::serialize(CheckpointOut &cp) const
{
    DescCache<TxDesc>::serialize(cp);

    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(isTcp);
    SERIALIZE_SCALAR(pktWaiting);
    SERIALIZE_SCALAR(pktMultiDesc);

    SERIALIZE_SCALAR(useTso);
    SERIALIZE_SCALAR(tsoHeaderLen);
    SERIALIZE_SCALAR(tsoMss);
    SERIALIZE_SCALAR(tsoTotalLen);
    SERIALIZE_SCALAR(tsoUsedLen);
    SERIALIZE_SCALAR(tsoPrevSeq);;
    SERIALIZE_SCALAR(tsoPktPayloadBytes);
    SERIALIZE_SCALAR(tsoLoadedHeader);
    SERIALIZE_SCALAR(tsoPktHasHeader);
    SERIALIZE_ARRAY(tsoHeader, 256);
    SERIALIZE_SCALAR(tsoDescBytesUsed);
    SERIALIZE_SCALAR(tsoCopyBytes);
    SERIALIZE_SCALAR(tsoPkts);

    SERIALIZE_SCALAR(completionAddress);
    SERIALIZE_SCALAR(completionEnabled);
    SERIALIZE_SCALAR(descEnd);
}

void
IGbE::TxDescCache::unserialize(CheckpointIn &cp)
{
    DescCache<TxDesc>::unserialize(cp);

    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(isTcp);
    UNSERIALIZE_SCALAR(pktWaiting);
    UNSERIALIZE_SCALAR(pktMultiDesc);

    UNSERIALIZE_SCALAR(useTso);
    UNSERIALIZE_SCALAR(tsoHeaderLen);
    UNSERIALIZE_SCALAR(tsoMss);
    UNSERIALIZE_SCALAR(tsoTotalLen);
    UNSERIALIZE_SCALAR(tsoUsedLen);
    UNSERIALIZE_SCALAR(tsoPrevSeq);;
    UNSERIALIZE_SCALAR(tsoPktPayloadBytes);
    UNSERIALIZE_SCALAR(tsoLoadedHeader);
    UNSERIALIZE_SCALAR(tsoPktHasHeader);
    UNSERIALIZE_ARRAY(tsoHeader, 256);
    UNSERIALIZE_SCALAR(tsoDescBytesUsed);
    UNSERIALIZE_SCALAR(tsoCopyBytes);
    UNSERIALIZE_SCALAR(tsoPkts);

    UNSERIALIZE_SCALAR(completionAddress);
    UNSERIALIZE_SCALAR(completionEnabled);
    UNSERIALIZE_SCALAR(descEnd);
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
    if (igbe->drainState() != DrainState::Draining) {
        igbe->txTick = true;
        igbe->restartClock();
    }
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
    if (!tickEvent.scheduled() && (rxTick || txTick || txFifoTick) &&
        drainState() == DrainState::Running)
        schedule(tickEvent, clockEdge(Cycles(1)));
}

DrainState
IGbE::drain()
{
    unsigned int count(0);
    if (rxDescCache.hasOutstandingEvents() ||
        txDescCache.hasOutstandingEvents()) {
        count++;
    }

    txFifoTick = false;
    txTick = false;
    rxTick = false;

    if (tickEvent.scheduled())
        deschedule(tickEvent);

    if (count) {
        DPRINTF(Drain, "IGbE not drained\n");
        return DrainState::Draining;
    } else
        return DrainState::Drained;
}

void
IGbE::drainResume()
{
    Drainable::drainResume();

    txFifoTick = true;
    txTick = true;
    rxTick = true;

    restartClock();
    DPRINTF(EthernetSM, "resuming from drain");
}

void
IGbE::checkDrain()
{
    if (drainState() != DrainState::Draining)
        return;

    txFifoTick = false;
    txTick = false;
    rxTick = false;
    if (!rxDescCache.hasOutstandingEvents() &&
        !txDescCache.hasOutstandingEvents()) {
        DPRINTF(Drain, "IGbE done draining, processing drain event\n");
        signalDrainDone();
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
    if (txPacket && txDescCache.packetAvailable()
        && !txDescCache.packetMultiDesc() && txPacket->length) {
        anQ("TXS", "TX FIFO Q");
        DPRINTF(EthernetSM, "TXS: packet placed in TX FIFO\n");
#ifndef NDEBUG
        bool success =
#endif
            txFifo.push(txPacket);
        txFifoTick = true && drainState() != DrainState::Draining;
        assert(success);
        txPacket = NULL;
        anBegin("TXS", "Desc Writeback");
        txDescCache.writeback((cacheBlockSize()-1)>>4);
        return;
    }

    // Only support descriptor granularity
    if (regs.txdctl.lwthresh() &&
        txDescCache.descLeft() < (regs.txdctl.lwthresh() * 8)) {
        DPRINTF(EthernetSM, "TXS: LWTHRESH caused posting of TXDLOW\n");
        postInterrupt(IT_TXDLOW);
    }

    if (!txPacket) {
        txPacket = std::make_shared<EthPacketData>(16384);
    }

    if (!txDescCache.packetWaiting()) {
        if (txDescCache.descLeft() == 0) {
            postInterrupt(IT_TXQE);
            anBegin("TXS", "Desc Writeback");
            txDescCache.writeback(0);
            anBegin("TXS", "Desc Fetch");
            anWe("TXS", txDescCache.annUnusedCacheQ);
            txDescCache.fetchDescriptors();
            DPRINTF(EthernetSM, "TXS: No descriptors left in ring, forcing "
                    "writeback stopping ticking and posting TXQE\n");
            txTick = false;
            return;
        }


        if (!(txDescCache.descUnused())) {
            anBegin("TXS", "Desc Fetch");
            txDescCache.fetchDescriptors();
            anWe("TXS", txDescCache.annUnusedCacheQ);
            DPRINTF(EthernetSM, "TXS: No descriptors available in cache, "
                    "fetching and stopping ticking\n");
            txTick = false;
            return;
        }
        anPq("TXS", txDescCache.annUnusedCacheQ);


        txDescCache.processContextDesc();
        if (txDescCache.packetWaiting()) {
            DPRINTF(EthernetSM,
                    "TXS: Fetching TSO header, stopping ticking\n");
            txTick = false;
            return;
        }

        unsigned size = txDescCache.getPacketSize(txPacket);
        if (size > 0 && txFifo.avail() > size) {
            anRq("TXS", "TX FIFO Q");
            anBegin("TXS", "DMA Packet");
            DPRINTF(EthernetSM, "TXS: Reserving %d bytes in FIFO and "
                    "beginning DMA of next packet\n", size);
            txFifo.reserve(size);
            txDescCache.getPacketData(txPacket);
        } else if (size == 0) {
            DPRINTF(EthernetSM, "TXS: getPacketSize returned: %d\n", size);
            DPRINTF(EthernetSM,
                    "TXS: No packets to get, writing back used descriptors\n");
            anBegin("TXS", "Desc Writeback");
            txDescCache.writeback(0);
        } else {
            anWf("TXS", "TX FIFO Q");
            DPRINTF(EthernetSM, "TXS: FIFO full, stopping ticking until space "
                    "available in FIFO\n");
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
    rxBytes += pkt->length;
    rxPackets++;

    DPRINTF(Ethernet, "RxFIFO: Receiving pcakte from wire\n");
    anBegin("RXQ", "Wire Recv");


    if (!regs.rctl.en()) {
        DPRINTF(Ethernet, "RxFIFO: RX not enabled, dropping\n");
        anBegin("RXQ", "FIFO Drop", CPA::FL_BAD);
        return true;
    }

    // restart the state machines if they are stopped
    rxTick = true && drainState() != DrainState::Draining;
    if ((rxTick || txTick) && !tickEvent.scheduled()) {
        DPRINTF(EthernetSM,
                "RXS: received packet into fifo, starting ticking\n");
        restartClock();
    }

    if (!rxFifo.push(pkt)) {
        DPRINTF(Ethernet, "RxFIFO: Packet won't fit in fifo... dropped\n");
        postInterrupt(IT_RXO, true);
        anBegin("RXQ", "FIFO Drop", CPA::FL_BAD);
        return false;
    }

    if (CPA::available() && cpa->enabled()) {
        assert(sys->numSystemsRunning <= 2);
        System *other_sys;
        if (sys->systemList[0] == sys)
            other_sys = sys->systemList[1];
        else
            other_sys = sys->systemList[0];

        cpa->hwDq(CPA::FL_NONE, sys, macAddr, "RXQ", "WireQ", 0, other_sys);
        anQ("RXQ", "RX FIFO Q");
        cpa->hwWe(CPA::FL_NONE, sys, macAddr, "RXQ", "WireQ", 0, other_sys);
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
        DPRINTF(EthernetSM, "RXS: descLeft: %d rdmts: %d rdlen: %d\n",
                descLeft, regs.rctl.rdmts(), regs.rdlen());

        // rdmts 2->1/8, 1->1/4, 0->1/2
        int ratio = (1ULL << (regs.rctl.rdmts() + 1));
        if (descLeft * ratio <= regs.rdlen()) {
            DPRINTF(Ethernet, "RXS: Interrupting (RXDMT) "
                    "because of descriptors left\n");
            postInterrupt(IT_RXDMT);
        }

        if (rxFifo.empty())
            rxDescCache.writeback(0);

        if (descLeft == 0) {
            anBegin("RXS", "Writeback Descriptors");
            rxDescCache.writeback(0);
            DPRINTF(EthernetSM, "RXS: No descriptors left in ring, forcing"
                    " writeback and stopping ticking\n");
            rxTick = false;
        }

        // only support descriptor granulaties
        assert(regs.rxdctl.gran());

        if (regs.rxdctl.wthresh() >= rxDescCache.descUsed()) {
            DPRINTF(EthernetSM,
                    "RXS: Writing back because WTHRESH >= descUsed\n");
            anBegin("RXS", "Writeback Descriptors");
            if (regs.rxdctl.wthresh() < (cacheBlockSize()>>4))
                rxDescCache.writeback(regs.rxdctl.wthresh()-1);
            else
                rxDescCache.writeback((cacheBlockSize()-1)>>4);
        }

        if ((rxDescCache.descUnused() < regs.rxdctl.pthresh()) &&
            ((rxDescCache.descLeft() - rxDescCache.descUnused()) >
             regs.rxdctl.hthresh())) {
            DPRINTF(EthernetSM, "RXS: Fetching descriptors because "
                    "descUnused < PTHRESH\n");
            anBegin("RXS", "Fetch Descriptors");
            rxDescCache.fetchDescriptors();
        }

        if (rxDescCache.descUnused() == 0) {
            anBegin("RXS", "Fetch Descriptors");
            rxDescCache.fetchDescriptors();
            anWe("RXS", rxDescCache.annUnusedCacheQ);
            DPRINTF(EthernetSM, "RXS: No descriptors available in cache, "
                    "fetching descriptors and stopping ticking\n");
            rxTick = false;
        }
        return;
    }

    if (rxDmaPacket) {
        DPRINTF(EthernetSM,
                "RXS: stopping ticking until packet DMA completes\n");
        rxTick = false;
        return;
    }

    if (!rxDescCache.descUnused()) {
        anBegin("RXS", "Fetch Descriptors");
        rxDescCache.fetchDescriptors();
        anWe("RXS", rxDescCache.annUnusedCacheQ);
        DPRINTF(EthernetSM, "RXS: No descriptors available in cache, "
                "stopping ticking\n");
        rxTick = false;
        DPRINTF(EthernetSM, "RXS: No descriptors available, fetching\n");
        return;
    }
    anPq("RXS", rxDescCache.annUnusedCacheQ);

    if (rxFifo.empty()) {
        anWe("RXS", "RX FIFO Q");
        DPRINTF(EthernetSM, "RXS: RxFIFO empty, stopping ticking\n");
        rxTick = false;
        return;
    }
    anPq("RXS", "RX FIFO Q");
    anBegin("RXS", "Get Desc");

    EthPacketPtr pkt;
    pkt = rxFifo.front();


    pktOffset = rxDescCache.writePacket(pkt, pktOffset);
    DPRINTF(EthernetSM, "RXS: Writing packet into memory\n");
    if (pktOffset == pkt->length) {
        anBegin( "RXS", "FIFO Dequeue");
        DPRINTF(EthernetSM, "RXS: Removing packet from FIFO\n");
        pktOffset = 0;
        anDq("RXS", "RX FIFO Q");
        rxFifo.pop();
    }

    DPRINTF(EthernetSM, "RXS: stopping ticking until packet DMA completes\n");
    rxTick = false;
    rxDmaPacket = true;
    anBegin("RXS", "DMA Packet");
}

void
IGbE::txWire()
{
    txFifoTick = false;

    if (txFifo.empty()) {
        anWe("TXQ", "TX FIFO Q");
        return;
    }


    anPq("TXQ", "TX FIFO Q");
    if (etherInt->sendPacket(txFifo.front())) {
        anQ("TXQ", "WireQ");
        if (DTRACE(EthernetSM)) {
            IpPtr ip(txFifo.front());
            if (ip)
                DPRINTF(EthernetSM, "Transmitting Ip packet with Id=%d\n",
                        ip->id());
            else
                DPRINTF(EthernetSM, "Transmitting Non-Ip packet\n");
        }
        anDq("TXQ", "TX FIFO Q");
        anBegin("TXQ", "Wire Send");
        DPRINTF(EthernetSM,
                "TxFIFO: Successful transmit, bytes available in fifo: %d\n",
                txFifo.avail());

        txBytes += txFifo.front()->length;
        txPackets++;

        txFifo.pop();
    }
}

void
IGbE::tick()
{
    DPRINTF(EthernetSM, "IGbE: -------------- Cycle --------------\n");

    inTick = true;

    if (rxTick)
        rxStateMachine();

    if (txTick)
        txStateMachine();

    // If txWire returns and txFifoTick is still set, that means the data we
    // sent to the other end was already accepted and we can send another
    // frame right away. This is consistent with the previous behavior which
    // would send another frame if one was ready in ethTxDone. This version
    // avoids growing the stack with each frame sent which can cause stack
    // overflow.
    while (txFifoTick)
        txWire();

    if (rxTick || txTick || txFifoTick)
        schedule(tickEvent, curTick() + clockPeriod());

    inTick = false;
}

void
IGbE::ethTxDone()
{
    anBegin("TXQ", "Send Done");
    // restart the tx state machines if they are stopped
    // fifo to send another packet
    // tx sm to put more data into the fifo
    txFifoTick = true && drainState() != DrainState::Draining;
    if (txDescCache.descLeft() != 0 && drainState() != DrainState::Draining)
        txTick = true;

    if (!inTick)
        restartClock();
    DPRINTF(EthernetSM, "TxFIFO: Transmission complete\n");
}

void
IGbE::serialize(CheckpointOut &cp) const
{
    PciDevice::serialize(cp);

    regs.serialize(cp);
    SERIALIZE_SCALAR(eeOpBits);
    SERIALIZE_SCALAR(eeAddrBits);
    SERIALIZE_SCALAR(eeDataBits);
    SERIALIZE_SCALAR(eeOpcode);
    SERIALIZE_SCALAR(eeAddr);
    SERIALIZE_SCALAR(lastInterrupt);
    SERIALIZE_ARRAY(flash,iGbReg::EEPROM_SIZE);

    rxFifo.serialize("rxfifo", cp);
    txFifo.serialize("txfifo", cp);

    bool txPktExists = txPacket != nullptr;
    SERIALIZE_SCALAR(txPktExists);
    if (txPktExists)
        txPacket->serialize("txpacket", cp);

    Tick rdtr_time = 0, radv_time = 0, tidv_time = 0, tadv_time = 0,
        inter_time = 0;

    if (rdtrEvent.scheduled())
        rdtr_time = rdtrEvent.when();
    SERIALIZE_SCALAR(rdtr_time);

    if (radvEvent.scheduled())
        radv_time = radvEvent.when();
    SERIALIZE_SCALAR(radv_time);

    if (tidvEvent.scheduled())
        tidv_time = tidvEvent.when();
    SERIALIZE_SCALAR(tidv_time);

    if (tadvEvent.scheduled())
        tadv_time = tadvEvent.when();
    SERIALIZE_SCALAR(tadv_time);

    if (interEvent.scheduled())
        inter_time = interEvent.when();
    SERIALIZE_SCALAR(inter_time);

    SERIALIZE_SCALAR(pktOffset);

    txDescCache.serializeSection(cp, "TxDescCache");
    rxDescCache.serializeSection(cp, "RxDescCache");
}

void
IGbE::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);

    regs.unserialize(cp);
    UNSERIALIZE_SCALAR(eeOpBits);
    UNSERIALIZE_SCALAR(eeAddrBits);
    UNSERIALIZE_SCALAR(eeDataBits);
    UNSERIALIZE_SCALAR(eeOpcode);
    UNSERIALIZE_SCALAR(eeAddr);
    UNSERIALIZE_SCALAR(lastInterrupt);
    UNSERIALIZE_ARRAY(flash,iGbReg::EEPROM_SIZE);

    rxFifo.unserialize("rxfifo", cp);
    txFifo.unserialize("txfifo", cp);

    bool txPktExists;
    UNSERIALIZE_SCALAR(txPktExists);
    if (txPktExists) {
        txPacket = std::make_shared<EthPacketData>(16384);
        txPacket->unserialize("txpacket", cp);
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
        schedule(rdtrEvent, rdtr_time);

    if (radv_time)
        schedule(radvEvent, radv_time);

    if (tidv_time)
        schedule(tidvEvent, tidv_time);

    if (tadv_time)
        schedule(tadvEvent, tadv_time);

    if (inter_time)
        schedule(interEvent, inter_time);

    UNSERIALIZE_SCALAR(pktOffset);

    txDescCache.unserializeSection(cp, "TxDescCache");
    rxDescCache.unserializeSection(cp, "RxDescCache");
}

IGbE *
IGbEParams::create()
{
    return new IGbE(this);
}

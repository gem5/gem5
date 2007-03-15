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

#include "base/inet.hh"
#include "dev/i8254xGBe.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/builder.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace iGbReg;

IGbE::IGbE(Params *p)
    : PciDev(p), etherInt(NULL), useFlowControl(p->use_flow_control)
{
    // Initialized internal registers per Intel documentation
    regs.tctl(0);
    regs.rctl(0);
    regs.ctrl(0);
    regs.ctrl.fd(1);
    regs.ctrl.lrst(1);
    regs.ctrl.speed(2);
    regs.ctrl.frcspd(1);
    regs.sts(0);
    regs.sts.speed(3); // Say we're 1000Mbps
    regs.sts.fd(1); // full duplex
    regs.eecd(0);
    regs.eecd.fwe(1);
    regs.eecd.ee_type(1);
    regs.eerd(0);
    regs.icr(0);
    regs.rctl(0);
    regs.tctl(0);
    regs.fcrtl(0);
    regs.fcrth(1);
    regs.manc(0);

    regs.pba.rxa(0x30);
    regs.pba.txa(0x10);

    eeOpBits            = 0;
    eeAddrBits          = 0;
    eeDataBits          = 0;
    eeOpcode            = 0;

    // clear all 64 16 bit words of the eeprom
    memset(&flash, 0, EEPROM_SIZE*2);

    //We'll need to instert the MAC address into the flash
    flash[0] = 0xA4A4;
    flash[1] = 0xB6B6;
    flash[2] = 0xC8C8;

    uint16_t csum = 0;
    for (int x = 0; x < EEPROM_SIZE; x++)
        csum += flash[x];

    // Magic happy checksum value
    flash[EEPROM_SIZE-1] = htobe((uint16_t)(EEPROM_CSUM - csum));
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

    //DPRINTF(Ethernet, "Read device register %#X\n", daddr);

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
        pkt->set<uint32_t>(regs.icr());
        // handle auto setting mask from IAM
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

    //DPRINTF(Ethernet, "Wrote device register %#X value %#X\n", daddr, pkt->get<uint32_t>());

    ///
    /// Handle write of register here
    ///
    uint32_t val = pkt->get<uint32_t>();

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
                warn("Accessing unknown phy register %d\n", regs.mdic.regadd());
        }
        regs.mdic.r(1);
        break;
      case REG_ICR:
        regs.icr = val;
        // handle auto setting mask from IAM
        break;
      case REG_ITR:
        regs.itr = val;
        break;
      case REG_ICS:
        regs.icr = val | regs.icr();
        // generate an interrupt if needed here
        break;
       case REG_IMS:
        regs.imr |= val;
        // handle interrupts if needed here
        break;
      case REG_IMC:
        regs.imr |= ~val;
        // handle interrupts if needed here
        break;
      case REG_IAM:
        regs.iam = val;
        break;
      case REG_RCTL:
        regs.rctl = val;
        break;
      case REG_FCTTV:
        regs.fcttv = val;
        break;
      case REG_TCTL:
        regs.tctl = val;
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
        break;
      case REG_RDBAH:
        regs.rdba.rdbah(val);
        break;
      case REG_RDLEN:
        regs.rdlen = val & ~mask(7);
        break;
      case REG_RDH:
        regs.rdh = val;
        break;
      case REG_RDT:
        regs.rdt = val;
        break;
      case REG_RDTR:
        regs.rdtr = val;
        break;
      case REG_RADV:
        regs.radv = val;
        break;
      case REG_TDBAL:
        regs.tdba.tdbal( val & ~mask(4));
        break;
      case REG_TDBAH:
        regs.tdba.tdbah(val);
        break;
      case REG_TDLEN:
        regs.tdlen = val & ~mask(7);
        break;
      case REG_TDH:
        regs.tdh = val;
        break;
      case REG_TDT:
        regs.tdt = val;
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


bool
IGbE::ethRxPkt(EthPacketPtr packet)
{
    panic("Need to implemenet\n");
}


void
IGbE::ethTxDone()
{
    panic("Need to implemenet\n");
}

void
IGbE::serialize(std::ostream &os)
{
    panic("Need to implemenet\n");
}

void
IGbE::unserialize(Checkpoint *cp, const std::string &section)
{
    panic("Need to implemenet\n");
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(IGbEInt)

    SimObjectParam<EtherInt *> peer;
    SimObjectParam<IGbE *> device;

END_DECLARE_SIM_OBJECT_PARAMS(IGbEInt)

BEGIN_INIT_SIM_OBJECT_PARAMS(IGbEInt)

    INIT_PARAM_DFLT(peer, "peer interface", NULL),
    INIT_PARAM(device, "Ethernet device of this interface")

END_INIT_SIM_OBJECT_PARAMS(IGbEInt)

CREATE_SIM_OBJECT(IGbEInt)
{
    IGbEInt *dev_int = new IGbEInt(getInstanceName(), device);

    EtherInt *p = (EtherInt *)peer;
    if (p) {
        dev_int->setPeer(p);
        p->setPeer(dev_int);
    }

    return dev_int;
}

REGISTER_SIM_OBJECT("IGbEInt", IGbEInt)


BEGIN_DECLARE_SIM_OBJECT_PARAMS(IGbE)

    SimObjectParam<System *> system;
    SimObjectParam<Platform *> platform;
    SimObjectParam<PciConfigData *> configdata;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;
    Param<Tick> pio_latency;
    Param<Tick> config_latency;

END_DECLARE_SIM_OBJECT_PARAMS(IGbE)

BEGIN_INIT_SIM_OBJECT_PARAMS(IGbE)

    INIT_PARAM(system, "System pointer"),
    INIT_PARAM(platform, "Platform pointer"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(pci_bus, "PCI bus ID"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code"),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM(config_latency, "Number of cycles for a config read or write")

END_INIT_SIM_OBJECT_PARAMS(IGbE)


CREATE_SIM_OBJECT(IGbE)
{
    IGbE::Params *params = new IGbE::Params;

    params->name = getInstanceName();
    params->platform = platform;
    params->system = system;
    params->configData = configdata;
    params->busNum = pci_bus;
    params->deviceNum = pci_dev;
    params->functionNum = pci_func;
    params->pio_delay = pio_latency;
    params->config_delay = config_latency;

    return new IGbE(params);
}

REGISTER_SIM_OBJECT("IGbE", IGbE)

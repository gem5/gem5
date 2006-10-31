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
    : PciDev(p), etherInt(NULL)
{
    // Initialized internal registers per Intel documentation
    regs.tctl.reg       = 0;
    regs.rctl.reg       = 0;
    regs.ctrl.reg       = 0;
    regs.ctrl.fd        = 1;
    regs.ctrl.lrst      = 1;
    regs.ctrl.speed     = 2;
    regs.ctrl.frcspd    = 1;
    regs.sts.reg        = 0;
    regs.eecd.reg       = 0;
    regs.eecd.fwe       = 1;
    regs.eecd.ee_type   = 1;
    regs.eerd.reg       = 0;
    regs.icd.reg        = 0;
    regs.imc.reg        = 0;
    regs.rctl.reg       = 0;
    regs.tctl.reg       = 0;
    regs.manc.reg       = 0;

    regs.pba.rxa        = 0x30;
    regs.pba.txa        = 0x10;

    eeOpBits            = 0;
    eeAddrBits          = 0;
    eeDataBits          = 0;
    eeOpcode            = 0;

    // clear all 64 16 bit words of the eeprom
    memset(&flash, 0, EEPROM_SIZE*2);

    // Magic happy checksum value
    flash[0] = 0xBABA;
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
      case CTRL:
       pkt->set<uint32_t>(regs.ctrl.reg);
       break;
      case STATUS:
       pkt->set<uint32_t>(regs.sts.reg);
       break;
      case EECD:
       pkt->set<uint32_t>(regs.eecd.reg);
       break;
      case EERD:
       pkt->set<uint32_t>(regs.eerd.reg);
       break;
      case ICR:
       pkt->set<uint32_t>(regs.icd.reg);
       break;
      case IMC:
       pkt->set<uint32_t>(regs.imc.reg);
       break;
      case RCTL:
       pkt->set<uint32_t>(regs.rctl.reg);
       break;
      case TCTL:
       pkt->set<uint32_t>(regs.tctl.reg);
       break;
      case PBA:
       pkt->set<uint32_t>(regs.pba.reg);
       break;
      case WUC:
      case LEDCTL:
       pkt->set<uint32_t>(0); // We don't care, so just return 0
       break;
      case MANC:
       pkt->set<uint32_t>(regs.manc.reg);
       break;
      default:
       if (!(daddr >= VFTA && daddr < (VFTA + VLAN_FILTER_TABLE_SIZE)*4) &&
           !(daddr >= RAL && daddr < (RAL + RCV_ADDRESS_TABLE_SIZE)*4) &&
           !(daddr >= MTA && daddr < (MTA + MULTICAST_TABLE_SIZE)*4))
           pkt->set<uint32_t>(0);
       else
           panic("Read request to unknown register number: %#x\n", daddr);
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
      case CTRL:
       regs.ctrl.reg = val;
       break;
      case STATUS:
       regs.sts.reg = val;
       break;
      case EECD:
       int oldClk;
       oldClk = regs.eecd.sk;
       regs.eecd.reg = val;
       // See if this is a eeprom access and emulate accordingly
       if (!oldClk && regs.eecd.sk) {
           if (eeOpBits < 8) {
               eeOpcode = eeOpcode << 1 | regs.eecd.din;
               eeOpBits++;
           } else if (eeAddrBits < 8 && eeOpcode == EEPROM_READ_OPCODE_SPI) {
               eeAddr = eeAddr << 1 | regs.eecd.din;
               eeAddrBits++;
           } else if (eeDataBits < 16 && eeOpcode == EEPROM_READ_OPCODE_SPI) {
               assert(eeAddr>>1 < EEPROM_SIZE);
               DPRINTF(EthernetEEPROM, "EEPROM bit read: %d word: %#X\n",
                       flash[eeAddr>>1] >> eeDataBits & 0x1, flash[eeAddr>>1]);
               regs.eecd.dout = (flash[eeAddr>>1] >> (15-eeDataBits)) & 0x1;
               eeDataBits++;
           } else if (eeDataBits < 8 && eeOpcode == EEPROM_RDSR_OPCODE_SPI) {
               regs.eecd.dout = 0;
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
       regs.eecd.ee_gnt = regs.eecd.ee_req;
       break;
      case EERD:
       regs.eerd.reg = val;
       break;
      case ICR:
       regs.icd.reg = val;
       break;
      case IMC:
       regs.imc.reg = val;
       break;
      case RCTL:
       regs.rctl.reg = val;
       break;
      case TCTL:
       regs.tctl.reg = val;
       break;
      case PBA:
       regs.pba.rxa = val;
       regs.pba.txa = 64 - regs.pba.rxa;
       break;
      case WUC:
      case LEDCTL:
       ; // We don't care, so don't store anything
       break;
      case MANC:
       regs.manc.reg = val;
       break;
      default:
       if (!(daddr >= VFTA && daddr < (VFTA + VLAN_FILTER_TABLE_SIZE)*4) &&
           !(daddr >= RAL && daddr < (RAL + RCV_ADDRESS_TABLE_SIZE)*4) &&
           !(daddr >= MTA && daddr < (MTA + MULTICAST_TABLE_SIZE)*4))
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

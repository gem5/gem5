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
 * Authors: Andrew Schultz
 *          Ali Saidi
 *          Miguel Serrano
 */

#include <cstddef>
#include <cstdlib>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/intr_control.hh"
#include "dev/ide_ctrl.hh"
#include "dev/ide_disk.hh"
#include "dev/pciconfigall.hh"
#include "dev/pcireg.h"
#include "dev/platform.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/builder.hh"
#include "sim/sim_object.hh"
#include "sim/byteswap.hh"

using namespace std;

////
// Initialization and destruction
////

IdeController::IdeController(Params *p)
    : PciDev(p)
{
    // initialize the PIO interface addresses
    pri_cmd_addr = 0;
    pri_cmd_size = BARSize[0];

    pri_ctrl_addr = 0;
    pri_ctrl_size = BARSize[1];

    sec_cmd_addr = 0;
    sec_cmd_size = BARSize[2];

    sec_ctrl_addr = 0;
    sec_ctrl_size = BARSize[3];

    // initialize the bus master interface (BMI) address to be configured
    // via PCI
    bmi_addr = 0;
    bmi_size = BARSize[4];

    // zero out all of the registers
    memset(bmi_regs.data, 0, sizeof(bmi_regs));
    memset(config_regs.data, 0, sizeof(config_regs.data));

    // setup initial values
    // enable both channels
    config_regs.idetim0 = htole((uint16_t)IDETIM_DECODE_EN);
    config_regs.idetim1 = htole((uint16_t)IDETIM_DECODE_EN);
    bmi_regs.bmis0 = DMA1CAP | DMA0CAP;
    bmi_regs.bmis1 = DMA1CAP | DMA0CAP;

    // reset all internal variables
    io_enabled = false;
    bm_enabled = false;
    memset(cmd_in_progress, 0, sizeof(cmd_in_progress));

    // setup the disks attached to controller
    memset(disks, 0, sizeof(disks));
    dev[0] = 0;
    dev[1] = 0;

    if (params()->disks.size() > 3)
        panic("IDE controllers support a maximum of 4 devices attached!\n");

    for (int i = 0; i < params()->disks.size(); i++) {
        disks[i] = params()->disks[i];
        disks[i]->setController(this);
    }
}

IdeController::~IdeController()
{
    for (int i = 0; i < 4; i++)
        if (disks[i])
            delete disks[i];
}

////
// Utility functions
///

void
IdeController::parseAddr(const Addr &addr, Addr &offset, IdeChannel &channel,
                         IdeRegType &reg_type)
{
    offset = addr;

    if (addr >= pri_cmd_addr && addr < (pri_cmd_addr + pri_cmd_size)) {
        offset -= pri_cmd_addr;
        reg_type = COMMAND_BLOCK;
        channel = PRIMARY;
    } else if (addr >= pri_ctrl_addr &&
               addr < (pri_ctrl_addr + pri_ctrl_size)) {
        offset -= pri_ctrl_addr;
        reg_type = CONTROL_BLOCK;
        channel = PRIMARY;
    } else if (addr >= sec_cmd_addr &&
               addr < (sec_cmd_addr + sec_cmd_size)) {
        offset -= sec_cmd_addr;
        reg_type = COMMAND_BLOCK;
        channel = SECONDARY;
    } else if (addr >= sec_ctrl_addr &&
               addr < (sec_ctrl_addr + sec_ctrl_size)) {
        offset -= sec_ctrl_addr;
        reg_type = CONTROL_BLOCK;
        channel = SECONDARY;
    } else if (addr >= bmi_addr && addr < (bmi_addr + bmi_size)) {
        offset -= bmi_addr;
        reg_type = BMI_BLOCK;
        channel = (offset < BMIC1) ? PRIMARY : SECONDARY;
    } else {
        panic("IDE controller access to invalid address: %#x\n", addr);
    }
}

int
IdeController::getDisk(IdeChannel channel)
{
    int disk = 0;
    uint8_t *devBit = &dev[0];

    if (channel == SECONDARY) {
        disk += 2;
        devBit = &dev[1];
    }

    disk += *devBit;

    assert(*devBit == 0 || *devBit == 1);

    return disk;
}

int
IdeController::getDisk(IdeDisk *diskPtr)
{
    for (int i = 0; i < 4; i++) {
        if ((long)diskPtr == (long)disks[i])
            return i;
    }
    return -1;
}

bool
IdeController::isDiskSelected(IdeDisk *diskPtr)
{
    for (int i = 0; i < 4; i++) {
        if ((long)diskPtr == (long)disks[i]) {
            // is disk is on primary or secondary channel
            int channel = i/2;
            // is disk the master or slave
            int devID = i%2;

            return (dev[channel] == devID);
        }
    }
    panic("Unable to find disk by pointer!!\n");
}

////
// Command completion
////

void
IdeController::setDmaComplete(IdeDisk *disk)
{
    int diskNum = getDisk(disk);

    if (diskNum < 0)
        panic("Unable to find disk based on pointer %#x\n", disk);

    if (diskNum < 2) {
        // clear the start/stop bit in the command register
        bmi_regs.bmic0 &= ~SSBM;
        // clear the bus master active bit in the status register
        bmi_regs.bmis0 &= ~BMIDEA;
        // set the interrupt bit
        bmi_regs.bmis0 |= IDEINTS;
    } else {
        // clear the start/stop bit in the command register
        bmi_regs.bmic1 &= ~SSBM;
        // clear the bus master active bit in the status register
        bmi_regs.bmis1 &= ~BMIDEA;
        // set the interrupt bit
        bmi_regs.bmis1 |= IDEINTS;
    }
}


////
// Read and write handling
////

Tick
IdeController::readConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC)
        return  PciDev::readConfig(pkt);
    assert(offset >= IDE_CTRL_CONF_START && (offset + 1) <= IDE_CTRL_CONF_END);

    pkt->allocate();

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        switch (offset) {
          case IDE_CTRL_CONF_DEV_TIMING:
            pkt->set<uint8_t>(config_regs.sidetim);
            break;
          case IDE_CTRL_CONF_UDMA_CNTRL:
            pkt->set<uint8_t>(config_regs.udmactl);
            break;
          case IDE_CTRL_CONF_PRIM_TIMING+1:
            pkt->set<uint8_t>(htole(config_regs.idetim0) >> 8);
            break;
          case IDE_CTRL_CONF_SEC_TIMING+1:
            pkt->set<uint8_t>(htole(config_regs.idetim1) >> 8);
            break;
          case IDE_CTRL_CONF_IDE_CONFIG:
            pkt->set<uint8_t>(htole(config_regs.ideconfig) & 0xFF);
            break;
          case IDE_CTRL_CONF_IDE_CONFIG+1:
            pkt->set<uint8_t>(htole(config_regs.ideconfig) >> 8);
            break;
          default:
            panic("Invalid PCI configuration read for size 1 at offset: %#x!\n",
                    offset);
        }
        DPRINTF(IdeCtrl, "PCI read offset: %#x size: 1 data: %#x\n", offset,
                (uint32_t)pkt->get<uint8_t>());
        break;
      case sizeof(uint16_t):
        switch (offset) {
          case IDE_CTRL_CONF_PRIM_TIMING:
            pkt->set<uint16_t>(config_regs.idetim0);
            break;
          case IDE_CTRL_CONF_SEC_TIMING:
            pkt->set<uint16_t>(config_regs.idetim1);
            break;
          case IDE_CTRL_CONF_UDMA_TIMING:
            pkt->set<uint16_t>(config_regs.udmatim);
            break;
          case IDE_CTRL_CONF_IDE_CONFIG:
            pkt->set<uint16_t>(config_regs.ideconfig);
            break;
          default:
            panic("Invalid PCI configuration read for size 2 offset: %#x!\n",
                    offset);
        }
        DPRINTF(IdeCtrl, "PCI read offset: %#x size: 2 data: %#x\n", offset,
                (uint32_t)pkt->get<uint16_t>());
        break;
      case sizeof(uint32_t):
        panic("No 32bit reads implemented for this device.");
        DPRINTF(IdeCtrl, "PCI read offset: %#x size: 4 data: %#x\n", offset,
                (uint32_t)pkt->get<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->result = Packet::Success;
    return configDelay;

}


Tick
IdeController::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC) {
        PciDev::writeConfig(pkt);
    } else {
        assert(offset >= IDE_CTRL_CONF_START && (offset + 1) <= IDE_CTRL_CONF_END);

        switch (pkt->getSize()) {
          case sizeof(uint8_t):
            switch (offset) {
              case IDE_CTRL_CONF_DEV_TIMING:
                config_regs.sidetim = pkt->get<uint8_t>();
                break;
              case IDE_CTRL_CONF_UDMA_CNTRL:
                config_regs.udmactl = pkt->get<uint8_t>();
                break;
              case IDE_CTRL_CONF_IDE_CONFIG:
                config_regs.ideconfig = (config_regs.ideconfig & 0xFF00) |
                    (pkt->get<uint8_t>());
                break;
              case IDE_CTRL_CONF_IDE_CONFIG+1:
                config_regs.ideconfig = (config_regs.ideconfig & 0x00FF) |
                    pkt->get<uint8_t>() << 8;
                break;
              default:
                panic("Invalid PCI configuration write for size 1 offset: %#x!\n",
                        offset);
            }
            DPRINTF(IdeCtrl, "PCI write offset: %#x size: 1 data: %#x\n",
                    offset, (uint32_t)pkt->get<uint8_t>());
            break;
          case sizeof(uint16_t):
            switch (offset) {
              case IDE_CTRL_CONF_PRIM_TIMING:
                config_regs.idetim0 = pkt->get<uint16_t>();
                break;
              case IDE_CTRL_CONF_SEC_TIMING:
                config_regs.idetim1 = pkt->get<uint16_t>();
                break;
              case IDE_CTRL_CONF_UDMA_TIMING:
                config_regs.udmatim = pkt->get<uint16_t>();
                break;
              case IDE_CTRL_CONF_IDE_CONFIG:
                config_regs.ideconfig = pkt->get<uint16_t>();
                break;
              default:
                panic("Invalid PCI configuration write for size 2 offset: %#x!\n",
                        offset);
            }
            DPRINTF(IdeCtrl, "PCI write offset: %#x size: 2 data: %#x\n",
                    offset, (uint32_t)pkt->get<uint16_t>());
            break;
          case sizeof(uint32_t):
            panic("Write of unimplemented PCI config. register: %x\n", offset);
            break;
          default:
            panic("invalid access size(?) for PCI configspace!\n");
        }
    }

    /* Trap command register writes and enable IO/BM as appropriate as well as
     * BARs. */
    switch(offset) {
      case PCI0_BASE_ADDR0:
        if (BARAddrs[0] != 0)
            pri_cmd_addr = BARAddrs[0];
        break;

      case PCI0_BASE_ADDR1:
        if (BARAddrs[1] != 0)
            pri_ctrl_addr = BARAddrs[1];
        break;

      case PCI0_BASE_ADDR2:
        if (BARAddrs[2] != 0)
            sec_cmd_addr = BARAddrs[2];
        break;

      case PCI0_BASE_ADDR3:
        if (BARAddrs[3] != 0)
            sec_ctrl_addr = BARAddrs[3];
        break;

      case PCI0_BASE_ADDR4:
        if (BARAddrs[4] != 0)
            bmi_addr = BARAddrs[4];
        break;

      case PCI_COMMAND:
        if (letoh(config.command) & PCI_CMD_IOSE)
            io_enabled = true;
        else
            io_enabled = false;

        if (letoh(config.command) & PCI_CMD_BME)
            bm_enabled = true;
        else
            bm_enabled = false;
        break;
    }
    pkt->result = Packet::Success;
    return configDelay;
}


Tick
IdeController::read(PacketPtr pkt)
{
    Addr offset;
    IdeChannel channel;
    IdeRegType reg_type;
    int disk;

    pkt->allocate();
    if (pkt->getSize() != 1 && pkt->getSize() != 2 && pkt->getSize() !=4)
         panic("Bad IDE read size: %d\n", pkt->getSize());

    parseAddr(pkt->getAddr(), offset, channel, reg_type);

    if (!io_enabled) {
        pkt->result = Packet::Success;
        return pioDelay;
    }

    switch (reg_type) {
      case BMI_BLOCK:
        switch (pkt->getSize()) {
          case sizeof(uint8_t):
            pkt->set(bmi_regs.data[offset]);
            break;
          case sizeof(uint16_t):
            pkt->set(*(uint16_t*)&bmi_regs.data[offset]);
            break;
          case sizeof(uint32_t):
            pkt->set(*(uint32_t*)&bmi_regs.data[offset]);
            break;
          default:
            panic("IDE read of BMI reg invalid size: %#x\n", pkt->getSize());
        }
        break;

      case COMMAND_BLOCK:
      case CONTROL_BLOCK:
        disk = getDisk(channel);

        if (disks[disk] == NULL) {
            pkt->set<uint8_t>(0);
            break;
        }

        switch (offset) {
          case DATA_OFFSET:
            switch (pkt->getSize()) {
              case sizeof(uint16_t):
                disks[disk]->read(offset, reg_type, pkt->getPtr<uint8_t>());
                break;

              case sizeof(uint32_t):
                disks[disk]->read(offset, reg_type, pkt->getPtr<uint8_t>());
                disks[disk]->read(offset, reg_type,
                        pkt->getPtr<uint8_t>() + sizeof(uint16_t));
                break;

              default:
                panic("IDE read of data reg invalid size: %#x\n", pkt->getSize());
            }
            break;
          default:
            if (pkt->getSize() == sizeof(uint8_t)) {
                disks[disk]->read(offset, reg_type, pkt->getPtr<uint8_t>());
            } else
                panic("IDE read of command reg of invalid size: %#x\n", pkt->getSize());
        }
        break;
      default:
        panic("IDE controller read of unknown register block type!\n");
    }
    if (pkt->getSize() == 1)
    DPRINTF(IdeCtrl, "read from offset: %#x size: %#x data: %#x\n",
            offset, pkt->getSize(), (uint32_t)pkt->get<uint8_t>());
    else if (pkt->getSize() == 2)
    DPRINTF(IdeCtrl, "read from offset: %#x size: %#x data: %#x\n",
            offset, pkt->getSize(), pkt->get<uint16_t>());
    else
    DPRINTF(IdeCtrl, "read from offset: %#x size: %#x data: %#x\n",
            offset, pkt->getSize(), pkt->get<uint32_t>());

    pkt->result = Packet::Success;
    return pioDelay;
}

Tick
IdeController::write(PacketPtr pkt)
{
    Addr offset;
    IdeChannel channel;
    IdeRegType reg_type;
    int disk;
    uint8_t oldVal, newVal;

    parseAddr(pkt->getAddr(), offset, channel, reg_type);

    if (!io_enabled) {
        pkt->result = Packet::Success;
        DPRINTF(IdeCtrl, "io not enabled\n");
        return pioDelay;
    }

    switch (reg_type) {
      case BMI_BLOCK:
        if (!bm_enabled) {
            pkt->result = Packet::Success;
            return pioDelay;
        }

        switch (offset) {
            // Bus master IDE command register
          case BMIC1:
          case BMIC0:
            if (pkt->getSize() != sizeof(uint8_t))
                panic("Invalid BMIC write size: %x\n", pkt->getSize());

            // select the current disk based on DEV bit
            disk = getDisk(channel);

            oldVal = bmi_regs.chan[channel].bmic;
            newVal = pkt->get<uint8_t>();

            // if a DMA transfer is in progress, R/W control cannot change
            if (oldVal & SSBM) {
                if ((oldVal & RWCON) ^ (newVal & RWCON)) {
                    (oldVal & RWCON) ? newVal |= RWCON : newVal &= ~RWCON;
                }
            }

            // see if the start/stop bit is being changed
            if ((oldVal & SSBM) ^ (newVal & SSBM)) {
                if (oldVal & SSBM) {
                    // stopping DMA transfer
                    DPRINTF(IdeCtrl, "Stopping DMA transfer\n");

                    // clear the BMIDEA bit
                    bmi_regs.chan[channel].bmis =
                        bmi_regs.chan[channel].bmis & ~BMIDEA;

                    if (disks[disk] == NULL)
                        panic("DMA stop for disk %d which does not exist\n",
                              disk);

                    // inform the disk of the DMA transfer abort
                    disks[disk]->abortDma();
                } else {
                    // starting DMA transfer
                    DPRINTF(IdeCtrl, "Starting DMA transfer\n");

                    // set the BMIDEA bit
                    bmi_regs.chan[channel].bmis =
                        bmi_regs.chan[channel].bmis | BMIDEA;

                    if (disks[disk] == NULL)
                        panic("DMA start for disk %d which does not exist\n",
                              disk);

                    // inform the disk of the DMA transfer start
                    disks[disk]->startDma(letoh(bmi_regs.chan[channel].bmidtp));
                }
            }

            // update the register value
            bmi_regs.chan[channel].bmic = newVal;
            break;

            // Bus master IDE status register
          case BMIS0:
          case BMIS1:
            if (pkt->getSize() != sizeof(uint8_t))
                panic("Invalid BMIS write size: %x\n", pkt->getSize());

            oldVal = bmi_regs.chan[channel].bmis;
            newVal = pkt->get<uint8_t>();

            // the BMIDEA bit is RO
            newVal |= (oldVal & BMIDEA);

            // to reset (set 0) IDEINTS and IDEDMAE, write 1 to each
            if ((oldVal & IDEINTS) && (newVal & IDEINTS))
                newVal &= ~IDEINTS; // clear the interrupt?
            else
                (oldVal & IDEINTS) ? newVal |= IDEINTS : newVal &= ~IDEINTS;

            if ((oldVal & IDEDMAE) && (newVal & IDEDMAE))
                newVal &= ~IDEDMAE;
            else
                (oldVal & IDEDMAE) ? newVal |= IDEDMAE : newVal &= ~IDEDMAE;

            bmi_regs.chan[channel].bmis = newVal;
            break;

            // Bus master IDE descriptor table pointer register
          case BMIDTP0:
          case BMIDTP1:
            {
                if (pkt->getSize() != sizeof(uint32_t))
                    panic("Invalid BMIDTP write size: %x\n", pkt->getSize());

                bmi_regs.chan[channel].bmidtp = htole(pkt->get<uint32_t>() & ~0x3);
            }
            break;

          default:
            if (pkt->getSize() != sizeof(uint8_t) &&
                pkt->getSize() != sizeof(uint16_t) &&
                pkt->getSize() != sizeof(uint32_t))
                panic("IDE controller write of invalid write size: %x\n",
                      pkt->getSize());

            // do a default copy of data into the registers
            memcpy(&bmi_regs.data[offset], pkt->getPtr<uint8_t>(), pkt->getSize());
        }
        break;
      case COMMAND_BLOCK:
        if (offset == IDE_SELECT_OFFSET) {
            uint8_t *devBit = &dev[channel];
            *devBit = (letoh(pkt->get<uint8_t>()) & IDE_SELECT_DEV_BIT) ? 1 : 0;
        }
        // fall-through ok!
      case CONTROL_BLOCK:
        disk = getDisk(channel);

        if (disks[disk] == NULL)
            break;

        switch (offset) {
          case DATA_OFFSET:
            switch (pkt->getSize()) {
              case sizeof(uint16_t):
                disks[disk]->write(offset, reg_type, pkt->getPtr<uint8_t>());
                break;

              case sizeof(uint32_t):
                disks[disk]->write(offset, reg_type, pkt->getPtr<uint8_t>());
                disks[disk]->write(offset, reg_type, pkt->getPtr<uint8_t>() +
                        sizeof(uint16_t));
                break;
              default:
                panic("IDE write of data reg invalid size: %#x\n", pkt->getSize());
            }
            break;
          default:
            if (pkt->getSize() == sizeof(uint8_t)) {
                disks[disk]->write(offset, reg_type, pkt->getPtr<uint8_t>());
            } else
                panic("IDE write of command reg of invalid size: %#x\n", pkt->getSize());
        }
        break;
      default:
        panic("IDE controller write of unknown register block type!\n");
    }

    if (pkt->getSize() == 1)
    DPRINTF(IdeCtrl, "write to offset: %#x size: %#x data: %#x\n",
            offset, pkt->getSize(), (uint32_t)pkt->get<uint8_t>());
    else if (pkt->getSize() == 2)
    DPRINTF(IdeCtrl, "write to offset: %#x size: %#x data: %#x\n",
            offset, pkt->getSize(), pkt->get<uint16_t>());
    else
    DPRINTF(IdeCtrl, "write to offset: %#x size: %#x data: %#x\n",
            offset, pkt->getSize(), pkt->get<uint32_t>());


    pkt->result = Packet::Success;
    return pioDelay;
}

////
// Serialization
////

void
IdeController::serialize(std::ostream &os)
{
    // Serialize the PciDev base class
    PciDev::serialize(os);

    // Serialize register addresses and sizes
    SERIALIZE_SCALAR(pri_cmd_addr);
    SERIALIZE_SCALAR(pri_cmd_size);
    SERIALIZE_SCALAR(pri_ctrl_addr);
    SERIALIZE_SCALAR(pri_ctrl_size);
    SERIALIZE_SCALAR(sec_cmd_addr);
    SERIALIZE_SCALAR(sec_cmd_size);
    SERIALIZE_SCALAR(sec_ctrl_addr);
    SERIALIZE_SCALAR(sec_ctrl_size);
    SERIALIZE_SCALAR(bmi_addr);
    SERIALIZE_SCALAR(bmi_size);

    // Serialize registers
    SERIALIZE_ARRAY(bmi_regs.data,
                    sizeof(bmi_regs.data) / sizeof(bmi_regs.data[0]));
    SERIALIZE_ARRAY(dev, sizeof(dev) / sizeof(dev[0]));
    SERIALIZE_ARRAY(config_regs.data,
                    sizeof(config_regs.data) / sizeof(config_regs.data[0]));

    // Serialize internal state
    SERIALIZE_SCALAR(io_enabled);
    SERIALIZE_SCALAR(bm_enabled);
    SERIALIZE_ARRAY(cmd_in_progress,
                    sizeof(cmd_in_progress) / sizeof(cmd_in_progress[0]));
}

void
IdeController::unserialize(Checkpoint *cp, const std::string &section)
{
    // Unserialize the PciDev base class
    PciDev::unserialize(cp, section);

    // Unserialize register addresses and sizes
    UNSERIALIZE_SCALAR(pri_cmd_addr);
    UNSERIALIZE_SCALAR(pri_cmd_size);
    UNSERIALIZE_SCALAR(pri_ctrl_addr);
    UNSERIALIZE_SCALAR(pri_ctrl_size);
    UNSERIALIZE_SCALAR(sec_cmd_addr);
    UNSERIALIZE_SCALAR(sec_cmd_size);
    UNSERIALIZE_SCALAR(sec_ctrl_addr);
    UNSERIALIZE_SCALAR(sec_ctrl_size);
    UNSERIALIZE_SCALAR(bmi_addr);
    UNSERIALIZE_SCALAR(bmi_size);

    // Unserialize registers
    UNSERIALIZE_ARRAY(bmi_regs.data,
                      sizeof(bmi_regs.data) / sizeof(bmi_regs.data[0]));
    UNSERIALIZE_ARRAY(dev, sizeof(dev) / sizeof(dev[0]));
    UNSERIALIZE_ARRAY(config_regs.data,
                      sizeof(config_regs.data) / sizeof(config_regs.data[0]));

    // Unserialize internal state
    UNSERIALIZE_SCALAR(io_enabled);
    UNSERIALIZE_SCALAR(bm_enabled);
    UNSERIALIZE_ARRAY(cmd_in_progress,
                      sizeof(cmd_in_progress) / sizeof(cmd_in_progress[0]));
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(IdeController)

    SimObjectParam<System *> system;
    SimObjectParam<Platform *> platform;
    SimObjectParam<PciConfigData *> configdata;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;
    Param<Tick> pio_latency;
    Param<Tick> config_latency;
    SimObjectVectorParam<IdeDisk *> disks;

END_DECLARE_SIM_OBJECT_PARAMS(IdeController)

BEGIN_INIT_SIM_OBJECT_PARAMS(IdeController)

    INIT_PARAM(system, "System pointer"),
    INIT_PARAM(platform, "Platform pointer"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(pci_bus, "PCI bus ID"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code"),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM(config_latency, "Number of cycles for a config read or write"),
    INIT_PARAM(disks, "IDE disks attached to this controller")

END_INIT_SIM_OBJECT_PARAMS(IdeController)

CREATE_SIM_OBJECT(IdeController)
{
    IdeController::Params *params = new IdeController::Params;
    params->name = getInstanceName();
    params->platform = platform;
    params->system = system;
    params->configData = configdata;
    params->busNum = pci_bus;
    params->deviceNum = pci_dev;
    params->functionNum = pci_func;
    params->pio_delay = pio_latency;
    params->config_delay = config_latency;
    params->disks = disks;
    return new IdeController(params);
}

REGISTER_SIM_OBJECT("IdeController", IdeController)

#endif //DOXYGEN_SHOULD_SKIP_THIS

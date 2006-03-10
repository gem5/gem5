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

#include <cstddef>
#include <cstdlib>
#include <string>
#include <vector>

#include "arch/alpha/ev5.hh"
#include "base/trace.hh"
#include "cpu/intr_control.hh"
#include "dev/ide_ctrl.hh"
#include "dev/ide_disk.hh"
#include "dev/pciconfigall.hh"
#include "dev/pcireg.h"
#include "dev/platform.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/dma_interface.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "sim/builder.hh"
#include "sim/sim_object.hh"

using namespace std;
using namespace TheISA;

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

    pioInterface = NULL;
    dmaInterface = NULL;
    // create the PIO and DMA interfaces
    if (params()->pio_bus) {
        pioInterface = newPioInterface(name() + ".pio", params()->hier,
                                       params()->pio_bus, this,
                                       &IdeController::cacheAccess);
        pioLatency = params()->pio_latency * params()->pio_bus->clockRate;
    }

    if (params()->dma_bus) {
        dmaInterface = new DMAInterface<Bus>(name() + ".dma",
                                             params()->dma_bus,
                                             params()->dma_bus, 1, true);
    }

    // setup the disks attached to controller
    memset(disks, 0, sizeof(disks));
    dev[0] = 0;
    dev[1] = 0;

    if (params()->disks.size() > 3)
        panic("IDE controllers support a maximum of 4 devices attached!\n");

    for (int i = 0; i < params()->disks.size(); i++) {
        disks[i] = params()->disks[i];
        disks[i]->setController(this, dmaInterface);
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
// Bus timing and bus access functions
////

Tick
IdeController::cacheAccess(MemReqPtr &req)
{
    // @todo Add more accurate timing to cache access
    return curTick + pioLatency;
}

////
// Read and write handling
////

void
IdeController::readConfig(int offset, int size, uint8_t *data)
{
    int config_offset;

    if (offset < PCI_DEVICE_SPECIFIC) {
        PciDev::readConfig(offset, size, data);
    } else if (offset >= IDE_CTRL_CONF_START &&
               (offset + size) <= IDE_CTRL_CONF_END) {

        config_offset = offset - IDE_CTRL_CONF_START;

        switch (size) {
          case sizeof(uint8_t):
            *data = config_regs.data[config_offset];
            break;
          case sizeof(uint16_t):
            *(uint16_t*)data = *(uint16_t*)&config_regs.data[config_offset];
            break;
          case sizeof(uint32_t):
            *(uint32_t*)data = *(uint32_t*)&config_regs.data[config_offset];
            break;
          default:
            panic("Invalid PCI configuration read size!\n");
        }

        DPRINTF(IdeCtrl, "PCI read offset: %#x size: %#x data: %#x\n",
                offset, size, *(uint32_t*)data);

    } else {
        panic("Read of unimplemented PCI config. register: %x\n", offset);
    }
}

void
IdeController::writeConfig(int offset, int size, const uint8_t *data)
{
    int config_offset;

    if (offset < PCI_DEVICE_SPECIFIC) {
        PciDev::writeConfig(offset, size, data);
    } else if (offset >= IDE_CTRL_CONF_START &&
               (offset + size) <= IDE_CTRL_CONF_END) {

        config_offset = offset - IDE_CTRL_CONF_START;

        switch(size) {
          case sizeof(uint8_t):
            config_regs.data[config_offset] = *data;
            break;
          case sizeof(uint16_t):
            *(uint16_t*)&config_regs.data[config_offset] = *(uint16_t*)data;
            break;
          case sizeof(uint32_t):
            *(uint32_t*)&config_regs.data[config_offset] = *(uint32_t*)data;
            break;
          default:
            panic("Invalid PCI configuration write size!\n");
        }
    } else {
        panic("Write of unimplemented PCI config. register: %x\n", offset);
    }

    DPRINTF(IdeCtrl, "PCI write offset: %#x size: %#x data: %#x\n",
            offset, size, data);

    // Catch the writes to specific PCI registers that have side affects
    // (like updating the PIO ranges)
    switch (offset) {
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

      case PCI0_BASE_ADDR0:
        if (BARAddrs[0] != 0) {
            pri_cmd_addr = BARAddrs[0];
            if (pioInterface)
                pioInterface->addAddrRange(RangeSize(pri_cmd_addr,
                                                     pri_cmd_size));

            pri_cmd_addr &= EV5::PAddrUncachedMask;
        }
        break;

      case PCI0_BASE_ADDR1:
        if (BARAddrs[1] != 0) {
            pri_ctrl_addr = BARAddrs[1];
            if (pioInterface)
                pioInterface->addAddrRange(RangeSize(pri_ctrl_addr,
                                                     pri_ctrl_size));

            pri_ctrl_addr &= EV5::PAddrUncachedMask;
        }
        break;

      case PCI0_BASE_ADDR2:
        if (BARAddrs[2] != 0) {
            sec_cmd_addr = BARAddrs[2];
            if (pioInterface)
                pioInterface->addAddrRange(RangeSize(sec_cmd_addr,
                                                     sec_cmd_size));

            sec_cmd_addr &= EV5::PAddrUncachedMask;
        }
        break;

      case PCI0_BASE_ADDR3:
        if (BARAddrs[3] != 0) {
            sec_ctrl_addr = BARAddrs[3];
            if (pioInterface)
                pioInterface->addAddrRange(RangeSize(sec_ctrl_addr,
                                                     sec_ctrl_size));

            sec_ctrl_addr &= EV5::PAddrUncachedMask;
        }
        break;

      case PCI0_BASE_ADDR4:
        if (BARAddrs[4] != 0) {
            bmi_addr = BARAddrs[4];
            if (pioInterface)
                pioInterface->addAddrRange(RangeSize(bmi_addr, bmi_size));

            bmi_addr &= EV5::PAddrUncachedMask;
        }
        break;
    }
}

Fault
IdeController::read(MemReqPtr &req, uint8_t *data)
{
    Addr offset;
    IdeChannel channel;
    IdeRegType reg_type;
    int disk;

    parseAddr(req->paddr, offset, channel, reg_type);

    if (!io_enabled)
        return NoFault;

    switch (reg_type) {
      case BMI_BLOCK:
        switch (req->size) {
          case sizeof(uint8_t):
            *data = bmi_regs.data[offset];
            break;
          case sizeof(uint16_t):
            *(uint16_t*)data = *(uint16_t*)&bmi_regs.data[offset];
            break;
          case sizeof(uint32_t):
            *(uint32_t*)data = *(uint32_t*)&bmi_regs.data[offset];
            break;
          default:
            panic("IDE read of BMI reg invalid size: %#x\n", req->size);
        }
        break;

      case COMMAND_BLOCK:
      case CONTROL_BLOCK:
        disk = getDisk(channel);

        if (disks[disk] == NULL)
            break;

        switch (offset) {
          case DATA_OFFSET:
            switch (req->size) {
              case sizeof(uint16_t):
                disks[disk]->read(offset, reg_type, data);
                break;

              case sizeof(uint32_t):
                disks[disk]->read(offset, reg_type, data);
                disks[disk]->read(offset, reg_type, &data[2]);
                break;

              default:
                panic("IDE read of data reg invalid size: %#x\n", req->size);
            }
            break;
          default:
            if (req->size == sizeof(uint8_t)) {
                disks[disk]->read(offset, reg_type, data);
            } else
                panic("IDE read of command reg of invalid size: %#x\n", req->size);
        }
        break;
      default:
        panic("IDE controller read of unknown register block type!\n");
    }

    DPRINTF(IdeCtrl, "read from offset: %#x size: %#x data: %#x\n",
            offset, req->size, *(uint32_t*)data);

    return NoFault;
}

Fault
IdeController::write(MemReqPtr &req, const uint8_t *data)
{
    Addr offset;
    IdeChannel channel;
    IdeRegType reg_type;
    int disk;
    uint8_t oldVal, newVal;

    parseAddr(req->paddr, offset, channel, reg_type);

    if (!io_enabled)
        return NoFault;

    switch (reg_type) {
      case BMI_BLOCK:
        if (!bm_enabled)
            return NoFault;

        switch (offset) {
            // Bus master IDE command register
          case BMIC1:
          case BMIC0:
            if (req->size != sizeof(uint8_t))
                panic("Invalid BMIC write size: %x\n", req->size);

            // select the current disk based on DEV bit
            disk = getDisk(channel);

            oldVal = bmi_regs.chan[channel].bmic;
            newVal = *data;

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
            if (req->size != sizeof(uint8_t))
                panic("Invalid BMIS write size: %x\n", req->size);

            oldVal = bmi_regs.chan[channel].bmis;
            newVal = *data;

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
                if (req->size != sizeof(uint32_t))
                    panic("Invalid BMIDTP write size: %x\n", req->size);

                uint32_t host_data =  letoh(*(uint32_t*)data);
                host_data &= ~0x3;
                bmi_regs.chan[channel].bmidtp = htole(host_data);
            }
            break;

          default:
            if (req->size != sizeof(uint8_t) &&
                req->size != sizeof(uint16_t) &&
                req->size != sizeof(uint32_t))
                panic("IDE controller write of invalid write size: %x\n",
                      req->size);

            // do a default copy of data into the registers
            memcpy(&bmi_regs.data[offset], data, req->size);
        }
        break;
      case COMMAND_BLOCK:
        if (offset == IDE_SELECT_OFFSET) {
            uint8_t *devBit = &dev[channel];
            *devBit = (letoh(*data) & IDE_SELECT_DEV_BIT) ? 1 : 0;
        }
        // fall-through ok!
      case CONTROL_BLOCK:
        disk = getDisk(channel);

        if (disks[disk] == NULL)
            break;

        switch (offset) {
          case DATA_OFFSET:
            switch (req->size) {
              case sizeof(uint16_t):
                disks[disk]->write(offset, reg_type, data);
                break;

              case sizeof(uint32_t):
                disks[disk]->write(offset, reg_type, data);
                disks[disk]->write(offset, reg_type, &data[2]);
                break;
              default:
                panic("IDE write of data reg invalid size: %#x\n", req->size);
            }
            break;
          default:
            if (req->size == sizeof(uint8_t)) {
                disks[disk]->write(offset, reg_type, data);
            } else
                panic("IDE write of command reg of invalid size: %#x\n", req->size);
        }
        break;
      default:
        panic("IDE controller write of unknown register block type!\n");
    }

    DPRINTF(IdeCtrl, "write to offset: %#x size: %#x data: %#x\n",
            offset, req->size, *(uint32_t*)data);

    return NoFault;
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

    if (pioInterface) {
        pioInterface->addAddrRange(RangeSize(pri_cmd_addr, pri_cmd_size));
        pioInterface->addAddrRange(RangeSize(pri_ctrl_addr, pri_ctrl_size));
        pioInterface->addAddrRange(RangeSize(sec_cmd_addr, sec_cmd_size));
        pioInterface->addAddrRange(RangeSize(sec_ctrl_addr, sec_ctrl_size));
        pioInterface->addAddrRange(RangeSize(bmi_addr, bmi_size));
   }
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(IdeController)

    Param<Addr> addr;
    SimObjectVectorParam<IdeDisk *> disks;
    SimObjectParam<MemoryController *> mmu;
    SimObjectParam<PciConfigAll *> configspace;
    SimObjectParam<PciConfigData *> configdata;
    SimObjectParam<Platform *> platform;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;
    SimObjectParam<Bus *> pio_bus;
    SimObjectParam<Bus *> dma_bus;
    Param<Tick> pio_latency;
    SimObjectParam<HierParams *> hier;

END_DECLARE_SIM_OBJECT_PARAMS(IdeController)

BEGIN_INIT_SIM_OBJECT_PARAMS(IdeController)

    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(disks, "IDE disks attached to this controller"),
    INIT_PARAM(mmu, "Memory controller"),
    INIT_PARAM(configspace, "PCI Configspace"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(platform, "Platform pointer"),
    INIT_PARAM(pci_bus, "PCI bus ID"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code"),
    INIT_PARAM(pio_bus, ""),
    INIT_PARAM(dma_bus, ""),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams)

END_INIT_SIM_OBJECT_PARAMS(IdeController)

CREATE_SIM_OBJECT(IdeController)
{
    IdeController::Params *params = new IdeController::Params;
    params->name = getInstanceName();
    params->mmu = mmu;
    params->configSpace = configspace;
    params->configData = configdata;
    params->plat = platform;
    params->busNum = pci_bus;
    params->deviceNum = pci_dev;
    params->functionNum = pci_func;

    params->disks = disks;
    params->pio_bus = pio_bus;
    params->dma_bus = dma_bus;
    params->pio_latency = pio_latency;
    params->hier = hier;
    return new IdeController(params);
}

REGISTER_SIM_OBJECT("IdeController", IdeController)

#endif //DOXYGEN_SHOULD_SKIP_THIS

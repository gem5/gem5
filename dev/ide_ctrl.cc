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

#include <cstddef>
#include <cstdlib>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/intr_control.hh"
#include "dev/dma.hh"
#include "dev/pcireg.h"
#include "dev/pciconfigall.hh"
#include "dev/ide_disk.hh"
#include "dev/ide_ctrl.hh"
#include "dev/tsunami_cchip.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/bus/dma_interface.hh"
#include "dev/tsunami.hh"
#include "mem/functional_mem/memory_control.hh"
#include "mem/functional_mem/physical_memory.hh"
#include "sim/builder.hh"
#include "sim/sim_object.hh"

using namespace std;

////
// Initialization and destruction
////

IdeController::IdeController(const string &name, IntrControl *ic,
                             const vector<IdeDisk *> &new_disks,
                             MemoryController *mmu, PciConfigAll *cf,
                             PciConfigData *cd, Tsunami *t, uint32_t bus_num,
                             uint32_t dev_num, uint32_t func_num,
                             Bus *host_bus, HierParams *hier)
    : PciDev(name, mmu, cf, cd, bus_num, dev_num, func_num), tsunami(t)
{
    // put back pointer into Tsunami
    tsunami->disk_controller = this;

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
    memset(bmi_regs, 0, sizeof(bmi_regs));
    memset(pci_regs, 0, sizeof(pci_regs));

    // setup initial values
    *(uint32_t *)&pci_regs[IDETIM] = 0x80008000; // enable both channels
    *(uint8_t *)&bmi_regs[BMIS0] = 0x60;
    *(uint8_t *)&bmi_regs[BMIS1] = 0x60;

    // reset all internal variables
    io_enabled = false;
    bm_enabled = false;
    memset(cmd_in_progress, 0, sizeof(cmd_in_progress));

    // create the PIO and DMA interfaces
    if (host_bus) {
        pioInterface = newPioInterface(name, hier, host_bus, this,
                                       &IdeController::cacheAccess);

        dmaInterface = new DMAInterface<Bus>(name + ".dma", host_bus,
                                             host_bus, 1);
    }

    // setup the disks attached to controller
    memset(disks, 0, sizeof(IdeDisk *) * 4);

    if (new_disks.size() > 3)
        panic("IDE controllers support a maximum of 4 devices attached!\n");

    for (int i = 0; i < new_disks.size(); i++) {
        disks[i] = new_disks[i];
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
IdeController::parseAddr(const Addr &addr, Addr &offset, bool &primary,
                         RegType_t &type)
{
    offset = addr;

    if (addr >= pri_cmd_addr && addr < (pri_cmd_addr + pri_cmd_size)) {
        offset -= pri_cmd_addr;
        type = COMMAND_BLOCK;
        primary = true;
    } else if (addr >= pri_ctrl_addr &&
               addr < (pri_ctrl_addr + pri_ctrl_size)) {
        offset -= pri_ctrl_addr;
        type = CONTROL_BLOCK;
        primary = true;
    } else if (addr >= sec_cmd_addr &&
               addr < (sec_cmd_addr + sec_cmd_size)) {
        offset -= sec_cmd_addr;
        type = COMMAND_BLOCK;
        primary = false;
    } else if (addr >= sec_ctrl_addr &&
               addr < (sec_ctrl_addr + sec_ctrl_size)) {
        offset -= sec_ctrl_addr;
        type = CONTROL_BLOCK;
        primary = false;
    } else if (addr >= bmi_addr && addr < (bmi_addr + bmi_size)) {
        offset -= bmi_addr;
        type = BMI_BLOCK;
        primary = (offset < BMIC1) ? true : false;
    } else {
        panic("IDE controller access to invalid address: %#x\n", addr);
    }
}

int
IdeController::getDisk(bool primary)
{
    int disk = 0;
    uint8_t *devBit = &dev[0];

    if (!primary) {
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
        bmi_regs[BMIC0] &= ~SSBM;
        // clear the bus master active bit in the status register
        bmi_regs[BMIS0] &= ~BMIDEA;
        // set the interrupt bit
        bmi_regs[BMIS0] |= IDEINTS;
    } else {
        // clear the start/stop bit in the command register
        bmi_regs[BMIC1] &= ~SSBM;
        // clear the bus master active bit in the status register
        bmi_regs[BMIS1] &= ~BMIDEA;
        // set the interrupt bit
        bmi_regs[BMIS1] |= IDEINTS;
    }
}

////
// Interrupt handling
////

void
IdeController::intrPost()
{
    tsunami->cchip->postDRIR(configData->config.hdr.pci0.interruptLine);
}

void
IdeController::intrClear()
{
    tsunami->cchip->clearDRIR(configData->config.hdr.pci0.interruptLine);
}

////
// Bus timing and bus access functions
////

Tick
IdeController::cacheAccess(MemReqPtr &req)
{
    // @todo Add more accurate timing to cache access
    return curTick + 1000;
}

////
// Read and write handling
////

void
IdeController::ReadConfig(int offset, int size, uint8_t *data)
{
    Addr origOffset = offset;

    if (offset < PCI_DEVICE_SPECIFIC) {
        PciDev::ReadConfig(offset, size, data);
    } else {
        if (offset >= PCI_IDE_TIMING && offset < (PCI_IDE_TIMING + 4)) {
            offset -= PCI_IDE_TIMING;
            offset += IDETIM;

            if ((offset + size) > (IDETIM + 4))
                panic("PCI read of IDETIM with invalid size\n");
        } else if (offset == PCI_SLAVE_TIMING) {
            offset -= PCI_SLAVE_TIMING;
            offset += SIDETIM;

            if ((offset + size) > (SIDETIM + 1))
                panic("PCI read of SIDETIM with invalid size\n");
        } else if (offset == PCI_UDMA33_CTRL) {
            offset -= PCI_UDMA33_CTRL;
            offset += UDMACTL;

            if ((offset + size) > (UDMACTL + 1))
                panic("PCI read of UDMACTL with invalid size\n");
        } else if (offset >= PCI_UDMA33_TIMING &&
                   offset < (PCI_UDMA33_TIMING + 2)) {
            offset -= PCI_UDMA33_TIMING;
            offset += UDMATIM;

            if ((offset + size) > (UDMATIM + 2))
                panic("PCI read of UDMATIM with invalid size\n");
        } else {
            panic("PCI read of unimplemented register: %x\n", offset);
        }

        memcpy((void *)data, (void *)&pci_regs[offset], size);
    }

    DPRINTF(IdeCtrl, "IDE PCI read offset: %#x (%#x) size: %#x data: %#x\n",
                origOffset, offset, size, *(uint32_t *)data);
}

void
IdeController::WriteConfig(int offset, int size, uint32_t data)
{
    DPRINTF(IdeCtrl, "IDE PCI write offset: %#x size: %#x data: %#x\n",
            offset, size, data);

    // do standard write stuff if in standard PCI space
    if (offset < PCI_DEVICE_SPECIFIC) {
        PciDev::WriteConfig(offset, size, data);
    } else {
        if (offset >= PCI_IDE_TIMING && offset < (PCI_IDE_TIMING + 4)) {
            offset -= PCI_IDE_TIMING;
            offset += IDETIM;

            if ((offset + size) > (IDETIM + 4))
                panic("PCI write to IDETIM with invalid size\n");
        } else if (offset == PCI_SLAVE_TIMING) {
            offset -= PCI_SLAVE_TIMING;
            offset += SIDETIM;

            if ((offset + size) > (SIDETIM + 1))
                panic("PCI write to SIDETIM with invalid size\n");
        } else if (offset == PCI_UDMA33_CTRL) {
            offset -= PCI_UDMA33_CTRL;
            offset += UDMACTL;

            if ((offset + size) > (UDMACTL + 1))
                panic("PCI write to UDMACTL with invalid size\n");
        } else if (offset >= PCI_UDMA33_TIMING &&
                   offset < (PCI_UDMA33_TIMING + 2)) {
            offset -= PCI_UDMA33_TIMING;
            offset += UDMATIM;

            if ((offset + size) > (UDMATIM + 2))
                panic("PCI write to UDMATIM with invalid size\n");
        } else {
            panic("PCI write to unimplemented register: %x\n", offset);
        }

        memcpy((void *)&pci_regs[offset], (void *)&data, size);
    }

    if (offset == PCI_COMMAND) {
        if (config.data[offset] & IOSE)
            io_enabled = true;
        else
            io_enabled = false;

        if (config.data[offset] & BME)
            bm_enabled = true;
        else
            bm_enabled = false;

    } else if (data != 0xffffffff) {
        switch (offset) {
          case PCI0_BASE_ADDR0:
            pri_cmd_addr = BARAddrs[0];
            if (pioInterface)
                pioInterface->addAddrRange(pri_cmd_addr,
                                           pri_cmd_addr + pri_cmd_size - 1);

            pri_cmd_addr = pri_cmd_addr & PA_UNCACHED_MASK;
            break;

          case PCI0_BASE_ADDR1:
            pri_ctrl_addr = BARAddrs[1];
            if (pioInterface)
                pioInterface->addAddrRange(pri_ctrl_addr,
                                           pri_ctrl_addr + pri_ctrl_size - 1);

            pri_ctrl_addr = pri_ctrl_addr & PA_UNCACHED_MASK;
            break;

          case PCI0_BASE_ADDR2:
            sec_cmd_addr = BARAddrs[2];
            if (pioInterface)
                pioInterface->addAddrRange(sec_cmd_addr,
                                           sec_cmd_addr + sec_cmd_size - 1);

            sec_cmd_addr = sec_cmd_addr & PA_UNCACHED_MASK;
            break;

          case PCI0_BASE_ADDR3:
            sec_ctrl_addr = BARAddrs[3];
            if (pioInterface)
                pioInterface->addAddrRange(sec_ctrl_addr,
                                           sec_ctrl_addr + sec_ctrl_size - 1);

            sec_ctrl_addr = sec_ctrl_addr & PA_UNCACHED_MASK;
            break;

          case PCI0_BASE_ADDR4:
            bmi_addr = BARAddrs[4];
            if (pioInterface)
                pioInterface->addAddrRange(bmi_addr, bmi_addr + bmi_size - 1);

            bmi_addr = bmi_addr & PA_UNCACHED_MASK;
            break;
        }
    }
}

Fault
IdeController::read(MemReqPtr &req, uint8_t *data)
{
    Addr offset;
    bool primary;
    bool byte;
    bool cmdBlk;
    RegType_t type;
    int disk;

    parseAddr(req->paddr, offset, primary, type);
    byte = (req->size == sizeof(uint8_t)) ? true : false;
    cmdBlk = (type == COMMAND_BLOCK) ? true : false;

    if (!io_enabled)
        return No_Fault;

    // sanity check the size (allows byte, word, or dword access)
    if (req->size != sizeof(uint8_t) && req->size != sizeof(uint16_t) &&
        req->size != sizeof(uint32_t))
        panic("IDE controller read of invalid size: %#x\n", req->size);

    if (type != BMI_BLOCK) {
        assert(req->size != sizeof(uint32_t));

        disk = getDisk(primary);
        if (disks[disk])
            disks[disk]->read(offset, byte, cmdBlk, data);
    } else {
        memcpy((void *)data, &bmi_regs[offset], req->size);
    }

    DPRINTF(IdeCtrl, "IDE read from offset: %#x size: %#x data: %#x\n",
            offset, req->size, *(uint32_t *)data);

    return No_Fault;
}

Fault
IdeController::write(MemReqPtr &req, const uint8_t *data)
{
    Addr offset;
    bool primary;
    bool byte;
    bool cmdBlk;
    RegType_t type;
    int disk;

    parseAddr(req->paddr, offset, primary, type);
    byte = (req->size == sizeof(uint8_t)) ? true : false;
    cmdBlk = (type == COMMAND_BLOCK) ? true : false;

    DPRINTF(IdeCtrl, "IDE write from offset: %#x size: %#x data: %#x\n",
            offset, req->size, *(uint32_t *)data);

    uint8_t oldVal, newVal;

    if (!io_enabled)
        return No_Fault;

    if (type == BMI_BLOCK && !bm_enabled)
        return No_Fault;

    if (type != BMI_BLOCK) {
        // shadow the dev bit
        if (type == COMMAND_BLOCK && offset == IDE_SELECT_OFFSET) {
            uint8_t *devBit = (primary ? &dev[0] : &dev[1]);
            *devBit = ((*data & IDE_SELECT_DEV_BIT) ? 1 : 0);
        }

        assert(req->size != sizeof(uint32_t));

        disk = getDisk(primary);
        if (disks[disk])
            disks[disk]->write(offset, byte, cmdBlk, data);
    } else {
        switch (offset) {
            // Bus master IDE command register
          case BMIC1:
          case BMIC0:
            if (req->size != sizeof(uint8_t))
                panic("Invalid BMIC write size: %x\n", req->size);

            // select the current disk based on DEV bit
            disk = getDisk(primary);

            oldVal = bmi_regs[offset];
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
                    bmi_regs[offset + 0x2] &= ~BMIDEA;

                    if (disks[disk] == NULL)
                        panic("DMA stop for disk %d which does not exist\n",
                              disk);

                    // inform the disk of the DMA transfer abort
                    disks[disk]->abortDma();
                } else {
                    // starting DMA transfer
                    DPRINTF(IdeCtrl, "Starting DMA transfer\n");

                    // set the BMIDEA bit
                    bmi_regs[offset + 0x2] |= BMIDEA;

                    if (disks[disk] == NULL)
                        panic("DMA start for disk %d which does not exist\n",
                              disk);

                    // inform the disk of the DMA transfer start
                    if (primary)
                        disks[disk]->startDma(*(uint32_t *)&bmi_regs[BMIDTP0]);
                    else
                        disks[disk]->startDma(*(uint32_t *)&bmi_regs[BMIDTP1]);
                }
            }

            // update the register value
            bmi_regs[offset] = newVal;
            break;

            // Bus master IDE status register
          case BMIS0:
          case BMIS1:
            if (req->size != sizeof(uint8_t))
                panic("Invalid BMIS write size: %x\n", req->size);

            oldVal = bmi_regs[offset];
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

            bmi_regs[offset] = newVal;
            break;

            // Bus master IDE descriptor table pointer register
          case BMIDTP0:
          case BMIDTP1:
            if (req->size != sizeof(uint32_t))
                panic("Invalid BMIDTP write size: %x\n", req->size);

            *(uint32_t *)&bmi_regs[offset] = *(uint32_t *)data & ~0x3;
            break;

          default:
            if (req->size != sizeof(uint8_t) &&
                req->size != sizeof(uint16_t) &&
                req->size != sizeof(uint32_t))
                panic("IDE controller write of invalid write size: %x\n",
                      req->size);

            // do a default copy of data into the registers
            memcpy((void *)&bmi_regs[offset], data, req->size);
        }
    }

    return No_Fault;
}

////
// Serialization
////

void
IdeController::serialize(std::ostream &os)
{
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
    SERIALIZE_ARRAY(bmi_regs, 16);
    SERIALIZE_ARRAY(dev, 2);
    SERIALIZE_ARRAY(pci_regs, 8);

    // Serialize internal state
    SERIALIZE_SCALAR(io_enabled);
    SERIALIZE_SCALAR(bm_enabled);
    SERIALIZE_ARRAY(cmd_in_progress, 4);
}

void
IdeController::unserialize(Checkpoint *cp, const std::string &section)
{
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
    UNSERIALIZE_ARRAY(bmi_regs, 16);
    UNSERIALIZE_ARRAY(dev, 2);
    UNSERIALIZE_ARRAY(pci_regs, 8);

    // Unserialize internal state
    UNSERIALIZE_SCALAR(io_enabled);
    UNSERIALIZE_SCALAR(bm_enabled);
    UNSERIALIZE_ARRAY(cmd_in_progress, 4);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(IdeController)

    SimObjectParam<IntrControl *> intr_ctrl;
    SimObjectVectorParam<IdeDisk *> disks;
    SimObjectParam<MemoryController *> mmu;
    SimObjectParam<PciConfigAll *> configspace;
    SimObjectParam<PciConfigData *> configdata;
    SimObjectParam<Tsunami *> tsunami;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;
    SimObjectParam<Bus *> host_bus;
    SimObjectParam<HierParams *> hier;

END_DECLARE_SIM_OBJECT_PARAMS(IdeController)

BEGIN_INIT_SIM_OBJECT_PARAMS(IdeController)

    INIT_PARAM(intr_ctrl, "Interrupt Controller"),
    INIT_PARAM(disks, "IDE disks attached to this controller"),
    INIT_PARAM(mmu, "Memory controller"),
    INIT_PARAM(configspace, "PCI Configspace"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(tsunami, "Tsunami chipset pointer"),
    INIT_PARAM(pci_bus, "PCI bus ID"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code"),
    INIT_PARAM_DFLT(host_bus, "Host bus to attach to", NULL),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams)

END_INIT_SIM_OBJECT_PARAMS(IdeController)

CREATE_SIM_OBJECT(IdeController)
{
    return new IdeController(getInstanceName(), intr_ctrl, disks, mmu,
                             configspace, configdata, tsunami, pci_bus,
                             pci_dev, pci_func, host_bus, hier);
}

REGISTER_SIM_OBJECT("IdeController", IdeController)

#endif //DOXYGEN_SHOULD_SKIP_THIS

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
    memset(regs, 0, sizeof(regs));
    memset(pci_regs, 0, sizeof(pci_regs));

    // setup initial values
    *(uint32_t *)&pci_regs[IDETIM] = 0x80008000; // enable both channels
    *(uint8_t *)&regs[BMI + BMIS0] = 0x60;
    *(uint8_t *)&regs[BMI + BMIS1] = 0x60;

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

            pri_cmd_addr = ((pri_cmd_addr | 0xf0000000000) & PA_IMPL_MASK);
            break;

          case PCI0_BASE_ADDR1:
            pri_ctrl_addr = BARAddrs[1];
            if (pioInterface)
                pioInterface->addAddrRange(pri_ctrl_addr,
                                           pri_ctrl_addr + pri_ctrl_size - 1);

            pri_ctrl_addr = ((pri_ctrl_addr | 0xf0000000000) & PA_IMPL_MASK);
            break;

          case PCI0_BASE_ADDR2:
            sec_cmd_addr = BARAddrs[2];
            if (pioInterface)
                pioInterface->addAddrRange(sec_cmd_addr,
                                           sec_cmd_addr + sec_cmd_size - 1);

            sec_cmd_addr = ((sec_cmd_addr | 0xf0000000000) & PA_IMPL_MASK);
            break;

          case PCI0_BASE_ADDR3:
            sec_ctrl_addr = BARAddrs[3];
            if (pioInterface)
                pioInterface->addAddrRange(sec_ctrl_addr,
                                           sec_ctrl_addr + sec_ctrl_size - 1);

            sec_ctrl_addr = ((sec_ctrl_addr | 0xf0000000000) & PA_IMPL_MASK);
            break;

          case PCI0_BASE_ADDR4:
            bmi_addr = BARAddrs[4];
            if (pioInterface)
                pioInterface->addAddrRange(bmi_addr, bmi_addr + bmi_size - 1);

            bmi_addr = ((bmi_addr | 0xf0000000000) & PA_IMPL_MASK);
            break;
        }
    }
}

Fault
IdeController::read(MemReqPtr &req, uint8_t *data)
{
    Addr offset = getOffset(req->paddr);

    if (!io_enabled)
        return No_Fault;

    // sanity check the size (allows byte, word, or dword access)
    if (req->size != sizeof(uint8_t) && req->size != sizeof(uint16_t) &&
        req->size != sizeof(uint32_t))
        panic("IDE controller read of invalid size: %#x\n", req->size);

    DPRINTF(IdeCtrl, "IDE default read from offset: %#x size: %#x data: %#x\n",
            offset, req->size, *(uint32_t *)&regs[offset]);

    // copy the data from the control registers
    memcpy((void *)data, &regs[offset], req->size);

    return No_Fault;
}

Fault
IdeController::write(MemReqPtr &req, const uint8_t *data)
{
    int disk = 0; // selected disk index
    uint8_t oldVal, newVal;

    Addr offset = getOffset(req->paddr);

    if (!io_enabled)
        return No_Fault;

    if (offset >= BMI && !bm_enabled)
        return No_Fault;

    switch (offset) {
        // Bus master IDE command register
      case (BMI + BMIC1):
      case (BMI + BMIC0):
        if (req->size != sizeof(uint8_t))
            panic("Invalid BMIC write size: %x\n", req->size);

        // select the current disk based on DEV bit
        disk = getDisk(offset);

        oldVal = regs[offset];
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
                regs[offset + 0x2] &= ~BMIDEA;

                if (disks[disk] == NULL)
                    panic("DMA stop for disk %d which does not exist\n", disk);

                // inform the disk of the DMA transfer abort
                disks[disk]->dmaStop();
            } else {
                // starting DMA transfer
                DPRINTF(IdeCtrl, "Starting DMA transfer\n");

                // set the BMIDEA bit
                regs[offset + 0x2] |= BMIDEA;

                if (disks[disk] == NULL)
                    panic("DMA start for disk %d which does not exist\n",
                          disk);

                // inform the disk of the DMA transfer start
                disks[disk]->dmaStart();
            }
        }

        // update the register value
        regs[offset] = newVal;
        break;

        // Bus master IDE status register
      case (BMI + BMIS0):
      case (BMI + BMIS1):
        if (req->size != sizeof(uint8_t))
            panic("Invalid BMIS write size: %x\n", req->size);

        oldVal = regs[offset];
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

        regs[offset] = newVal;
        break;

        // Bus master IDE descriptor table pointer register
      case (BMI + BMIDTP0):
      case (BMI + BMIDTP1):
        if (req->size != sizeof(uint32_t))
            panic("Invalid BMIDTP write size: %x\n", req->size);

        *(uint32_t *)&regs[offset] = *(uint32_t *)data & ~0x3;
        break;

        // Write the data word in the command register block
      case (CMD1 + IDE_DATA_OFFSET):
      case (CMD0 + IDE_DATA_OFFSET):
        if (req->size != sizeof(uint16_t))
            panic("Invalid command block data write size: %x\n", req->size);

        break;

        // Write the command byte in command register block
      case (CMD1 + IDE_COMMAND_OFFSET):
      case (CMD0 + IDE_COMMAND_OFFSET):
        if (req->size != sizeof(uint8_t))
            panic("Invalid command block command write size: %x\n", req->size);

        // select the disk based on the DEV bit
        disk = getDisk(offset);

        if (cmd_in_progress[disk])
            panic("Command on disk %d already in progress!\n", disk);
        if (disks[disk] == NULL)
            panic("Specified disk %d does not exist!\n", disk);

        cmd_in_progress[disk] = true;

        // write to both the command/status and alternate status
        regs[offset] = *data;
        regs[offset + 3] = *data;

        // issue the command to the disk
        if (disk < 2)
            disks[disk]->startIO(&regs[CMD0], &regs[BMI + BMIDTP0]);
        else
            disks[disk]->startIO(&regs[CMD1], &regs[BMI + BMIDTP1]);

        break;

      default:
        if (req->size != sizeof(uint8_t) && req->size != sizeof(uint16_t) &&
            req->size != sizeof(uint32_t))
            panic("IDE controller write of invalid write size: %x\n",
                  req->size);

        DPRINTF(IdeCtrl, "IDE default write offset: %#x size: %#x data: %#x\n",
                offset, req->size, *(uint32_t *)data);

        // do a default copy of data into the registers
        memcpy((void *)&regs[offset], data, req->size);
    }

    return No_Fault;
}

////
// Serialization
////

void
IdeController::serialize(std::ostream &os)
{
}

void
IdeController::unserialize(Checkpoint *cp, const std::string &section)
{
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

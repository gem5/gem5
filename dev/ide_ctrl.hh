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

/** @file
 * Simple PCI IDE controller with bus mastering capability
 */

#ifndef __IDE_CTRL_HH__
#define __IDE_CTRL_HH__

#include "dev/pcidev.hh"
#include "dev/pcireg.h"
#include "dev/io_device.hh"

#define CMD0  0x00 // Channel 0 command block offset
#define CTRL0 0x08 // Channel 0 control block offset
#define CMD1  0x0c // Channel 1 command block offset
#define CTRL1 0x14 // Channel 1 control block offset
#define BMI   0x18 // Bus master IDE offset

#define BMIC0    0x0  // Bus master IDE command register
#define BMIS0    0x2  // Bus master IDE status register
#define BMIDTP0  0x4  // Bus master IDE descriptor table pointer register
#define BMIC1    0x8  // Bus master IDE command register
#define BMIS1    0xa  // Bus master IDE status register
#define BMIDTP1  0xc  // Bus master IDE descriptor table pointer register

// Bus master IDE command register bit fields
#define RWCON 0x08 // Bus master read/write control
#define SSBM  0x01 // Start/stop bus master

// Bus master IDE status register bit fields
#define DMA1CAP 0x40 // Drive 1 DMA capable
#define DMA0CAP 0x20 // Drive 0 DMA capable
#define IDEINTS 0x04 // IDE Interrupt Status
#define IDEDMAE 0x02 // IDE DMA error
#define BMIDEA  0x01 // Bus master IDE active

// IDE Command byte fields
// Taken from include/linux/ide.h
#define IDE_DATA_OFFSET         (0)
#define IDE_ERROR_OFFSET        (1)
#define IDE_NSECTOR_OFFSET      (2)
#define IDE_SECTOR_OFFSET       (3)
#define IDE_LCYL_OFFSET         (4)
#define IDE_HCYL_OFFSET         (5)
#define IDE_SELECT_OFFSET       (6)
#define IDE_STATUS_OFFSET       (7)
#define IDE_CONTROL_OFFSET      (8)
#define IDE_IRQ_OFFSET          (9)

#define IDE_FEATURE_OFFSET      IDE_ERROR_OFFSET
#define IDE_COMMAND_OFFSET      IDE_STATUS_OFFSET

// PCI device specific register byte offsets
#define PCI_IDE_TIMING    0x40
#define PCI_SLAVE_TIMING  0x44
#define PCI_UDMA33_CTRL   0x48
#define PCI_UDMA33_TIMING 0x4a

#define IDETIM  (0)
#define SIDETIM (4)
#define UDMACTL (5)
#define UDMATIM (6)

// PCI Command bit fields
#define BME     0x04 // Bus master function enable
#define IOSE    0x01 // I/O space enable

class IntrControl;
class IdeDisk;
class PciConfigAll;
class Tsunami;
class PhysicalMemory;
class BaseInterface;
class HierParams;
class Bus;

/**
 * Device model for an Intel PIIX4 IDE controller
 */

class IdeController : public PciDev
{
  private:
    /** Primary command block registers */
    Addr pri_cmd_addr;
    Addr pri_cmd_size;
    /** Primary control block registers */
    Addr pri_ctrl_addr;
    Addr pri_ctrl_size;
    /** Secondary command block registers */
    Addr sec_cmd_addr;
    Addr sec_cmd_size;
    /** Secondary control block registers */
    Addr sec_ctrl_addr;
    Addr sec_ctrl_size;
    /** Bus master interface (BMI) registers */
    Addr bmi_addr;
    Addr bmi_size;

  private:
    /** Registers used for programmed I/O and bus master interface */
    uint8_t regs[40];
    /** Registers used in PCI configuration */
    uint8_t pci_regs[8];

    // Internal management variables
    bool io_enabled;
    bool bm_enabled;
    bool cmd_in_progress[4];

  private:
    /** Pointer to the chipset */
    Tsunami *tsunami;
    /** IDE disks connected to controller */
    IdeDisk *disks[4];

  private:
    /** Get offset into register file from access address */
    Addr getOffset(const Addr &addr) {
        Addr offset = addr;

        if (addr >= pri_cmd_addr && addr < (pri_cmd_addr + pri_cmd_size)) {
            offset -= pri_cmd_addr;
            offset += CMD0;
        } else if (addr >= pri_ctrl_addr &&
                   addr < (pri_ctrl_addr + pri_ctrl_size)) {
            offset -= pri_ctrl_addr;
            offset += CTRL0;
        } else if (addr >= sec_cmd_addr &&
                   addr < (sec_cmd_addr + sec_cmd_size)) {
            offset -= sec_cmd_addr;
            offset += CMD1;
        } else if (addr >= sec_ctrl_addr &&
                   addr < (sec_ctrl_addr + sec_ctrl_size)) {
            offset -= sec_ctrl_addr;
            offset += CTRL1;
        } else if (addr >= bmi_addr && addr < (bmi_addr + bmi_size)) {
            offset -= bmi_addr;
            offset += BMI;
        } else {
            panic("IDE controller access to invalid address: %#x\n", addr);
        }

        return offset;
    };
    /** Select the disk based on the register offset */
    int getDisk(const Addr &offset) {
        int disk = 0;

        // If the offset is in the channel 1 range, disks are 2 or 3
        if (offset >= CMD1 && offset < BMI && offset >= (BMI + BMIC1))
            disk += 2;

        if (disk < 2) {
            if (regs[CMD0 + IDE_STATUS_OFFSET] & 0x10)
                disk += 1;
        } else {
            if (regs[CMD1 + IDE_STATUS_OFFSET] & 0x10)
                disk += 1;
        }

        return disk;
    };

  public:
    /**
     * Constructs and initializes this controller.
     * @param name The name of this controller.
     * @param ic The interrupt controller.
     * @param mmu The memory controller
     * @param cf PCI config space
     * @param cd PCI config data
     * @param bus_num The PCI bus number
     * @param dev_num The PCI device number
     * @param func_num The PCI function number
     * @param host_bus The host bus to connect to
     * @param hier The hierarchy parameters
     */
    IdeController(const std::string &name, IntrControl *ic,
                  const vector<IdeDisk *> &new_disks,
                  MemoryController *mmu, PciConfigAll *cf,
                  PciConfigData *cd, Tsunami *t,
                  uint32_t bus_num, uint32_t dev_num, uint32_t func_num,
                  Bus *host_bus, HierParams *hier);

    /**
     * Deletes the connected devices.
     */
    ~IdeController();

    virtual void WriteConfig(int offset, int size, uint32_t data);
    virtual void ReadConfig(int offset, int size, uint8_t *data);

    /**
     * Read a done field for a given target.
     * @param req Contains the address of the field to read.
     * @param data Return the field read.
     * @return The fault condition of the access.
     */
    virtual Fault read(MemReqPtr &req, uint8_t *data);

    /**
     * Write to the mmapped I/O control registers.
     * @param req Contains the address to write to.
     * @param data The data to write.
     * @return The fault condition of the access.
     */
    virtual Fault write(MemReqPtr &req, const uint8_t *data);

    /**
     * Cache access timing specific to device
     * @param req Memory request
     */
    Tick cacheAccess(MemReqPtr &req);

    /**
     * Serialize this object to the given output stream.
     * @param os The stream to serialize to.
     */
    virtual void serialize(std::ostream &os);

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    virtual void unserialize(Checkpoint *cp, const std::string &section);

};
#endif // __IDE_CTRL_HH_

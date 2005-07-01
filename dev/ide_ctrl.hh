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

/** @file
 * Simple PCI IDE controller with bus mastering capability and UDMA
 * modeled after controller in the Intel PIIX4 chip
 */

#ifndef __IDE_CTRL_HH__
#define __IDE_CTRL_HH__

#include "dev/pcidev.hh"
#include "dev/pcireg.h"
#include "dev/io_device.hh"

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
#define IDE_SELECT_OFFSET       (6)
#define IDE_SELECT_DEV_BIT      0x10

#define IDE_FEATURE_OFFSET      IDE_ERROR_OFFSET
#define IDE_COMMAND_OFFSET      IDE_STATUS_OFFSET

// PCI device specific register byte offsets
#define IDE_CTRL_CONFIG_START 0x40
#define IDE_CTRL_CONFIG_END ((IDE_CTRL_CONFIG_START) + sizeof(pci_config_regs))


typedef enum RegType {
    COMMAND_BLOCK = 0,
    CONTROL_BLOCK,
    BMI_BLOCK
} RegType_t;

class BaseInterface;
class Bus;
class HierParams;
class IdeDisk;
class IntrControl;
class PciConfigAll;
class PhysicalMemory;
class Platform;

/**
 * Device model for an Intel PIIX4 IDE controller
 */

class IdeController : public PciDev
{
    friend class IdeDisk;

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
    /** Registers used for bus master interface */
    uint8_t bmi_regs[16];
    /** Shadows of the device select bit */
    uint8_t dev[2];
    /** Registers used in device specific PCI configuration */
    union {
        uint8_t data[22];

        struct {
            uint32_t idetim;
            uint8_t sidetim;
            uint8_t reserved_45;
            uint8_t reserved_46;
            uint8_t reserved_47;
            uint8_t udmactl;
            uint8_t reserved_49;
            uint16_t udmatim;
            uint8_t reserved_4c;
            uint8_t reserved_4d;
            uint8_t reserved_4e;
            uint8_t reserved_4f;
            uint8_t reserved_50;
            uint8_t reserved_51;
            uint8_t reserved_52;
            uint8_t reserved_53;
            uint16_t ideconfig;
        };
    } pci_config_regs;

    // Internal management variables
    bool io_enabled;
    bool bm_enabled;
    bool cmd_in_progress[4];

  private:
    /** IDE disks connected to controller */
    IdeDisk *disks[4];

  private:
    /** Parse the access address to pass on to device */
    void parseAddr(const Addr &addr, Addr &offset, bool &primary,
                   RegType_t &type);

    /** Select the disk based on the channel and device bit */
    int getDisk(bool primary);

    /** Select the disk based on a pointer */
    int getDisk(IdeDisk *diskPtr);

  public:
    /** See if a disk is selected based on its pointer */
    bool isDiskSelected(IdeDisk *diskPtr);

  public:
    struct Params : public PciDev::Params
    {
        /** Array of disk objects */
        std::vector<IdeDisk *> disks;
        Bus *host_bus;
        Tick pio_latency;
        HierParams *hier;
    };
    const Params *params() const { return (const Params *)_params; }

  public:
    IdeController(Params *p);
    ~IdeController();

    virtual void WriteConfig(int offset, int size, uint32_t data);
    virtual void ReadConfig(int offset, int size, uint8_t *data);

    void setDmaComplete(IdeDisk *disk);

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

    /**
     * Return how long this access will take.
     * @param req the memory request to calcuate
     * @return Tick when the request is done
     */
    Tick cacheAccess(MemReqPtr &req);
};
#endif // __IDE_CTRL_HH_

/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 */

/** @file
 * Device model for an IDE disk
 */

#ifndef __DEV_STORAGE_IDE_DISK_HH__
#define __DEV_STORAGE_IDE_DISK_HH__

#include "base/statistics.hh"
#include "dev/io_device.hh"
#include "dev/storage/disk_image.hh"
#include "dev/storage/ide_atareg.h"
#include "dev/storage/ide_ctrl.hh"
#include "dev/storage/ide_wdcreg.h"
#include "params/IdeDisk.hh"
#include "sim/eventq.hh"

class ChunkGenerator;

#define DMA_BACKOFF_PERIOD      200

#define MAX_DMA_SIZE            0x20000  // 128K
#define MAX_SINGLE_DMA_SIZE     0x10000
#define MAX_MULTSECT            (128)

#define PRD_BASE_MASK  0xfffffffe
#define PRD_COUNT_MASK 0xfffe
#define PRD_EOT_MASK   0x8000

typedef struct PrdEntry {
    uint32_t baseAddr;
    uint16_t byteCount;
    uint16_t endOfTable;
} PrdEntry_t;

class PrdTableEntry {
  public:
    PrdEntry_t entry;

    uint32_t getBaseAddr()
    {
        return (entry.baseAddr & PRD_BASE_MASK);
    }

    uint32_t getByteCount()
    {
        return ((entry.byteCount == 0) ? MAX_SINGLE_DMA_SIZE :
                (entry.byteCount & PRD_COUNT_MASK));
    }

    uint16_t getEOT()
    {
        return (entry.endOfTable & PRD_EOT_MASK);
    }
};

#define DATA_OFFSET     (0)
#define ERROR_OFFSET    (1)
#define FEATURES_OFFSET (1)
#define NSECTOR_OFFSET  (2)
#define SECTOR_OFFSET   (3)
#define LCYL_OFFSET     (4)
#define HCYL_OFFSET     (5)
#define SELECT_OFFSET   (6)
#define DRIVE_OFFSET    (6)
#define STATUS_OFFSET   (7)
#define COMMAND_OFFSET  (7)

#define CONTROL_OFFSET  (2)
#define ALTSTAT_OFFSET  (2)

#define SELECT_DEV_BIT  0x10
#define CONTROL_RST_BIT 0x04
#define CONTROL_IEN_BIT 0x02
#define STATUS_BSY_BIT  0x80
#define STATUS_DRDY_BIT 0x40
#define STATUS_DRQ_BIT  0x08
#define STATUS_SEEK_BIT 0x10
#define STATUS_DF_BIT   0x20
#define DRIVE_LBA_BIT   0x40

#define DEV0 (0)
#define DEV1 (1)

typedef struct CommandReg {
    uint16_t data;
    uint8_t error;
    uint8_t sec_count;
    uint8_t sec_num;
    uint8_t cyl_low;
    uint8_t cyl_high;
    union {
        uint8_t drive;
        uint8_t head;
    };
    uint8_t command;
} CommandReg_t;

typedef enum Events {
    None = 0,
    Transfer,
    ReadWait,
    WriteWait,
    PrdRead,
    DmaRead,
    DmaWrite
} Events_t;

typedef enum DevAction {
    ACT_NONE = 0,
    ACT_CMD_WRITE,
    ACT_CMD_COMPLETE,
    ACT_CMD_ERROR,
    ACT_SELECT_WRITE,
    ACT_STAT_READ,
    ACT_DATA_READY,
    ACT_DATA_READ_BYTE,
    ACT_DATA_READ_SHORT,
    ACT_DATA_WRITE_BYTE,
    ACT_DATA_WRITE_SHORT,
    ACT_DMA_READY,
    ACT_DMA_DONE,
    ACT_SRST_SET,
    ACT_SRST_CLEAR
} DevAction_t;

typedef enum DevState {
    // Device idle
    Device_Idle_S = 0,
    Device_Idle_SI,
    Device_Idle_NS,

    // Software reset
    Device_Srst,

    // Non-data commands
    Command_Execution,

    // PIO data-in (data to host)
    Prepare_Data_In,
    Data_Ready_INTRQ_In,
    Transfer_Data_In,

    // PIO data-out (data from host)
    Prepare_Data_Out,
    Data_Ready_INTRQ_Out,
    Transfer_Data_Out,

    // DMA protocol
    Prepare_Data_Dma,
    Transfer_Data_Dma,
    Device_Dma_Abort
} DevState_t;

typedef enum DmaState {
    Dma_Idle = 0,
    Dma_Start,
    Dma_Transfer
} DmaState_t;

class IdeController;

/**
 * IDE Disk device model
 */
class IdeDisk : public SimObject
{
  protected:
    /** The IDE controller for this disk. */
    IdeController *ctrl;
    /** The image that contains the data of this disk. */
    DiskImage *image;

  protected:
    /** The disk delay in microseconds. */
    int diskDelay;

  private:
    /** Drive identification structure for this disk */
    struct ataparams driveID;
    /** Data buffer for transfers */
    uint8_t *dataBuffer;
    /** Number of bytes in command data transfer */
    uint32_t cmdBytes;
    /** Number of bytes left in command data transfer */
    uint32_t cmdBytesLeft;
    /** Number of bytes left in DRQ block */
    uint32_t drqBytesLeft;
    /** Current sector in access */
    uint32_t curSector;
    /** Command block registers */
    CommandReg_t cmdReg;
    /** Status register */
    uint8_t status;
    /** Interrupt enable bit */
    bool nIENBit;
    /** Device state */
    DevState_t devState;
    /** Dma state */
    DmaState_t dmaState;
    /** Dma transaction is a read */
    bool dmaRead;
    /** PRD table base address */
    uint32_t curPrdAddr;
    /** PRD entry */
    PrdTableEntry curPrd;
    /** Device ID (master=0/slave=1) */
    int devID;
    /** Interrupt pending */
    bool intrPending;
    /** DMA Aborted */
    bool dmaAborted;

    Stats::Scalar dmaReadFullPages;
    Stats::Scalar dmaReadBytes;
    Stats::Scalar dmaReadTxs;
    Stats::Scalar dmaWriteFullPages;
    Stats::Scalar dmaWriteBytes;
    Stats::Scalar dmaWriteTxs;

  public:
    typedef IdeDiskParams Params;
    IdeDisk(const Params *p);

    /**
     * Delete the data buffer.
     */
    ~IdeDisk();

    /**
     * Reset the device state
     */
    void reset(int id);

    /**
     * Register Statistics
     */
    void regStats() override;

    /**
     * Set the controller for this device
     * @param c The IDE controller
     */
    void setController(IdeController *c) {
        if (ctrl) panic("Cannot change the controller once set!\n");
        ctrl = c;
    }

    // Device register read/write
    void readCommand(const Addr offset, int size, uint8_t *data);
    void readControl(const Addr offset, int size, uint8_t *data);
    void writeCommand(const Addr offset, int size, const uint8_t *data);
    void writeControl(const Addr offset, int size, const uint8_t *data);

    // Start/abort functions
    void startDma(const uint32_t &prdTableBase);
    void abortDma();

  private:
    void startCommand();

    // Interrupt management
    void intrPost();
    void intrClear();

    // DMA stuff
    void doDmaTransfer();
    friend class EventWrapper<IdeDisk, &IdeDisk::doDmaTransfer>;
    EventWrapper<IdeDisk, &IdeDisk::doDmaTransfer> dmaTransferEvent;

    void doDmaDataRead();

    void doDmaRead();
    ChunkGenerator *dmaReadCG;
    friend class EventWrapper<IdeDisk, &IdeDisk::doDmaRead>;
    EventWrapper<IdeDisk, &IdeDisk::doDmaRead> dmaReadWaitEvent;

    void doDmaDataWrite();

    void doDmaWrite();
    ChunkGenerator *dmaWriteCG;
    friend class EventWrapper<IdeDisk, &IdeDisk::doDmaWrite>;
    EventWrapper<IdeDisk, &IdeDisk::doDmaWrite> dmaWriteWaitEvent;

    void dmaPrdReadDone();
    friend class EventWrapper<IdeDisk, &IdeDisk::dmaPrdReadDone>;
    EventWrapper<IdeDisk, &IdeDisk::dmaPrdReadDone> dmaPrdReadEvent;

    void dmaReadDone();
    friend class EventWrapper<IdeDisk, &IdeDisk::dmaReadDone>;
    EventWrapper<IdeDisk, &IdeDisk::dmaReadDone> dmaReadEvent;

    void dmaWriteDone();
    friend class EventWrapper<IdeDisk, &IdeDisk::dmaWriteDone>;
    EventWrapper<IdeDisk, &IdeDisk::dmaWriteDone> dmaWriteEvent;

    // Disk image read/write
    void readDisk(uint32_t sector, uint8_t *data);
    void writeDisk(uint32_t sector, uint8_t *data);

    // State machine management
    void updateState(DevAction_t action);

    // Utility functions
    bool isBSYSet() { return (status & STATUS_BSY_BIT); }
    bool isIENSet() { return nIENBit; }
    bool isDEVSelect();

    void setComplete()
    {
        // clear out the status byte
        status = 0;
        // set the DRDY bit
        status |= STATUS_DRDY_BIT;
        // set the SEEK bit
        status |= STATUS_SEEK_BIT;
    }

    uint32_t getLBABase()
    {
        return  (Addr)(((cmdReg.head & 0xf) << 24) | (cmdReg.cyl_high << 16) |
                       (cmdReg.cyl_low << 8) | (cmdReg.sec_num));
    }

    inline Addr pciToDma(Addr pciAddr);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};


#endif // __DEV_STORAGE_IDE_DISK_HH__

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
 */

/** @file
 * Device model implementation for an IDE disk
 */

#include "dev/storage/ide_disk.hh"

#include <cerrno>
#include <cstring>
#include <deque>
#include <string>

#include "base/bitfield.hh"
#include "base/chunk_generator.hh"
#include "base/compiler.hh"
#include "base/cprintf.hh" // csprintf
#include "base/trace.hh"
#include "debug/IdeDisk.hh"
#include "dev/storage/disk_image.hh"
#include "dev/storage/ide_ctrl.hh"
#include "sim/cur_tick.hh"
#include "sim/sim_object.hh"

namespace gem5
{

IdeDisk::IdeDisk(const Params &p)
    : SimObject(p), image(p.image), diskDelay(p.delay), ideDiskStats(this),
      dmaTransferEvent([this]{ doDmaTransfer(); }, name()),
      dmaReadWaitEvent([this]{ doDmaRead(); }, name()),
      dmaWriteWaitEvent([this]{ doDmaWrite(); }, name()),
      dmaPrdReadEvent([this]{ dmaPrdReadDone(); }, name()),
      dmaReadEvent([this]{ dmaReadDone(); }, name()),
      dmaWriteEvent([this]{ dmaWriteDone(); }, name())
{
    // Reset the device state
    reset(p.driveID);

    // fill out the drive ID structure
    memset(&driveID, 0, sizeof(struct ataparams));

    // Calculate LBA and C/H/S values
    uint16_t cylinders;
    uint8_t heads;
    uint8_t sectors;

    uint32_t lba_size = image->size();
    if (lba_size >= 16383*16*63) {
        cylinders = 16383;
        heads = 16;
        sectors = 63;
    } else {
        if (lba_size >= 63)
            sectors = 63;
        else if (lba_size == 0)
            panic("Bad IDE image size: 0\n");
        else
            sectors = lba_size;

        if ((lba_size / sectors) >= 16)
            heads = 16;
        else
            heads = (lba_size / sectors);

        cylinders = lba_size / (heads * sectors);
    }

    // Setup the model name
    strncpy((char *)driveID.atap_model, "5MI EDD si k",
            sizeof(driveID.atap_model));
    // Set the maximum multisector transfer size
    driveID.atap_multi = MAX_MULTSECT;
    // IORDY supported, IORDY disabled, LBA enabled, DMA enabled
    driveID.atap_capabilities1 = 0x7;
    // UDMA support, EIDE support
    driveID.atap_extensions = 0x6;
    // Setup default C/H/S settings
    driveID.atap_cylinders = cylinders;
    driveID.atap_sectors = sectors;
    driveID.atap_heads = heads;
    // Setup the current multisector transfer size
    driveID.atap_curmulti = MAX_MULTSECT;
    driveID.atap_curmulti_valid = 0x1;
    // Number of sectors on disk
    driveID.atap_capacity = lba_size;
    // Multiword DMA mode 2 and below supported
    driveID.atap_dmamode_supp = 0x4;
    // Set PIO mode 4 and 3 supported
    driveID.atap_piomode_supp = 0x3;
    // Set DMA mode 4 and below supported
    driveID.atap_udmamode_supp = 0x1f;
    // Statically set hardware config word
    driveID.atap_hwreset_res = 0x4001;

    //arbitrary for now...
    driveID.atap_ata_major = WDC_VER_ATA7;
}

IdeDisk::~IdeDisk()
{
    // destroy the data buffer
    delete [] dataBuffer;
}

void
IdeDisk::reset(int id)
{
    // initialize the data buffer and shadow registers
    dataBuffer = new uint8_t[MAX_DMA_SIZE];

    memset(dataBuffer, 0, MAX_DMA_SIZE);
    memset(&cmdReg, 0, sizeof(CommandReg_t));
    memset(&curPrd.entry, 0, sizeof(PrdEntry_t));

    curPrdAddr = 0;
    curSector = 0;
    cmdBytes = 0;
    cmdBytesLeft = 0;
    drqBytesLeft = 0;
    dmaRead = false;
    pendingInterrupt = false;
    dmaAborted = false;

    // set the device state to idle
    dmaState = Dma_Idle;

    if (id == DEV0) {
        devState = Device_Idle_S;
        devID = DEV0;
    } else if (id == DEV1) {
        devState = Device_Idle_NS;
        devID = DEV1;
    } else {
        panic("Invalid device ID: %#x\n", id);
    }

    // set the device ready bit
    status = STATUS_DRDY_BIT;

    /* The error register must be set to 0x1 on start-up to
       indicate that no diagnostic error was detected */
    cmdReg.error = 0x1;
}

////
// Utility functions
////

bool
IdeDisk::isDEVSelect()
{
    return channel->selected() == this;
}

Addr
IdeDisk::pciToDma(Addr pciAddr)
{
    panic_if(!ctrl, "Access to unset controller!");
    return ctrl->pciToDma(pciAddr);
}

////
// Device registers read/write
////

void
IdeDisk::readCommand(const Addr offset, int size, uint8_t *data)
{
    if (offset == DATA_OFFSET) {
        if (size == sizeof(uint16_t)) {
            *(uint16_t *)data = cmdReg.data;
        } else if (size == sizeof(uint32_t)) {
            *(uint16_t *)data = cmdReg.data;
            updateState(ACT_DATA_READ_SHORT);
            *((uint16_t *)data + 1) = cmdReg.data;
        } else {
            panic("Data read of unsupported size %d.\n", size);
        }
        updateState(ACT_DATA_READ_SHORT);
        return;
    }
    assert(size == sizeof(uint8_t));
    switch (offset) {
      case ERROR_OFFSET:
        *data = cmdReg.error;
        break;
      case NSECTOR_OFFSET:
        *data = cmdReg.sec_count;
        break;
      case SECTOR_OFFSET:
        *data = cmdReg.sec_num;
        break;
      case LCYL_OFFSET:
        *data = cmdReg.cyl_low;
        break;
      case HCYL_OFFSET:
        *data = cmdReg.cyl_high;
        break;
      case DRIVE_OFFSET:
        *data = cmdReg.drive;
        break;
      case STATUS_OFFSET:
        *data = status;
        updateState(ACT_STAT_READ);
        break;
      default:
        panic("Invalid IDE command register offset: %#x\n", offset);
    }
    DPRINTF(IdeDisk, "Read to disk at offset: %#x data %#x\n", offset, *data);
}

void
IdeDisk::readControl(const Addr offset, int size, uint8_t *data)
{
    assert(size == sizeof(uint8_t));
    *data = status;
    if (offset != ALTSTAT_OFFSET)
        panic("Invalid IDE control register offset: %#x\n", offset);
    DPRINTF(IdeDisk, "Read to disk at offset: %#x data %#x\n", offset, *data);
}

void
IdeDisk::writeCommand(const Addr offset, int size, const uint8_t *data)
{
    if (offset == DATA_OFFSET) {
        if (size == sizeof(uint16_t)) {
            cmdReg.data = *(const uint16_t *)data;
        } else if (size == sizeof(uint32_t)) {
            cmdReg.data = *(const uint16_t *)data;
            updateState(ACT_DATA_WRITE_SHORT);
            cmdReg.data = *((const uint16_t *)data + 1);
        } else {
            panic("Data write of unsupported size %d.\n", size);
        }
        updateState(ACT_DATA_WRITE_SHORT);
        return;
    }

    assert(size == sizeof(uint8_t));
    switch (offset) {
      case FEATURES_OFFSET:
        break;
      case NSECTOR_OFFSET:
        cmdReg.sec_count = *data;
        break;
      case SECTOR_OFFSET:
        cmdReg.sec_num = *data;
        break;
      case LCYL_OFFSET:
        cmdReg.cyl_low = *data;
        break;
      case HCYL_OFFSET:
        cmdReg.cyl_high = *data;
        break;
      case DRIVE_OFFSET:
        cmdReg.drive = *data;
        updateState(ACT_SELECT_WRITE);
        break;
      case COMMAND_OFFSET:
        cmdReg.command = *data;
        updateState(ACT_CMD_WRITE);
        break;
      default:
        panic("Invalid IDE command register offset: %#x\n", offset);
    }
    DPRINTF(IdeDisk, "Write to disk at offset: %#x data %#x\n", offset,
            (uint32_t)*data);
}

void
IdeDisk::writeControl(const Addr offset, int size, const uint8_t *data)
{
    if (offset != CONTROL_OFFSET)
        panic("Invalid IDE control register offset: %#x\n", offset);

    if (*data & CONTROL_RST_BIT) {
        // force the device into the reset state
        devState = Device_Srst;
        updateState(ACT_SRST_SET);
    } else if (devState == Device_Srst && !(*data & CONTROL_RST_BIT)) {
        updateState(ACT_SRST_CLEAR);
    }

    nIENBit = *data & CONTROL_IEN_BIT;

    DPRINTF(IdeDisk, "Write to disk at offset: %#x data %#x\n", offset,
            (uint32_t)*data);
}

////
// Perform DMA transactions
////

void
IdeDisk::doDmaTransfer()
{
    if (dmaAborted) {
        DPRINTF(IdeDisk, "DMA Aborted before reading PRD entry\n");
        updateState(ACT_CMD_ERROR);
        return;
    }

    if (dmaState != Dma_Transfer || devState != Transfer_Data_Dma) {
        panic("Inconsistent DMA transfer state: dmaState = %d devState = %d\n",
              dmaState, devState);
    }

    if (ctrl->dmaPending() || ctrl->drainState() != DrainState::Running) {
        schedule(dmaTransferEvent, curTick() + DMA_BACKOFF_PERIOD);
        return;
    } else {
        ctrl->dmaRead(curPrdAddr, sizeof(PrdEntry_t), &dmaPrdReadEvent,
                (uint8_t*)&curPrd.entry);
    }
}

void
IdeDisk::dmaPrdReadDone()
{
    if (dmaAborted) {
        DPRINTF(IdeDisk, "DMA Aborted while reading PRD entry\n");
        updateState(ACT_CMD_ERROR);
        return;
    }

    DPRINTF(IdeDisk,
            "PRD: baseAddr:%#x (%#x) byteCount:%d (%d) eot:%#x sector:%d\n",
            curPrd.getBaseAddr(), pciToDma(curPrd.getBaseAddr()),
            curPrd.getByteCount(), (cmdBytesLeft/SectorSize),
            curPrd.getEOT(), curSector);

    // the prd pointer has already been translated, so just do an increment
    curPrdAddr = curPrdAddr + sizeof(PrdEntry_t);

    if (dmaRead)
        doDmaDataRead();
    else
        doDmaDataWrite();
}

void
IdeDisk::doDmaDataRead()
{
    /** @todo we need to figure out what the delay actually will be */
    Tick totalDiskDelay = diskDelay + (curPrd.getByteCount() / SectorSize);

    DPRINTF(IdeDisk, "doDmaRead, diskDelay: %d totalDiskDelay: %d\n",
            diskDelay, totalDiskDelay);

    schedule(dmaReadWaitEvent, curTick() + totalDiskDelay);
}

IdeDisk::
IdeDiskStats::IdeDiskStats(statistics::Group *parent)
    : statistics::Group(parent, "IdeDisk"),
      ADD_STAT(dmaReadFullPages, statistics::units::Count::get(),
               "Number of full page size DMA reads (not PRD)."),
      ADD_STAT(dmaReadBytes, statistics::units::Byte::get(),
               "Number of bytes transfered via DMA reads (not PRD)."),
      ADD_STAT(dmaReadTxs, statistics::units::Count::get(),
               "Number of DMA read transactions (not PRD)."),
      ADD_STAT(dmaWriteFullPages, statistics::units::Count::get(),
               "Number of full page size DMA writes."),
      ADD_STAT(dmaWriteBytes, statistics::units::Byte::get(),
               "Number of bytes transfered via DMA writes."),
      ADD_STAT(dmaWriteTxs, statistics::units::Count::get(),
               "Number of DMA write transactions.")
{
}

void
IdeDisk::doDmaRead()
{
    if (dmaAborted) {
        DPRINTF(IdeDisk, "DMA Aborted in middle of Dma Read\n");
        if (dmaReadCG)
            delete dmaReadCG;
        dmaReadCG = NULL;
        updateState(ACT_CMD_ERROR);
        return;
    }

    if (!dmaReadCG) {
        // clear out the data buffer
        memset(dataBuffer, 0, MAX_DMA_SIZE);
        dmaReadCG = new ChunkGenerator(curPrd.getBaseAddr(),
                curPrd.getByteCount(), chunkBytes);

    }
    if (ctrl->dmaPending() || ctrl->drainState() != DrainState::Running) {
        schedule(dmaReadWaitEvent, curTick() + DMA_BACKOFF_PERIOD);
        return;
    } else if (!dmaReadCG->done()) {
        assert(dmaReadCG->complete() < MAX_DMA_SIZE);
        ctrl->dmaRead(pciToDma(dmaReadCG->addr()), dmaReadCG->size(),
                &dmaReadWaitEvent, dataBuffer + dmaReadCG->complete());
        ideDiskStats.dmaReadBytes += dmaReadCG->size();
        ideDiskStats.dmaReadTxs++;
        if (dmaReadCG->size() == chunkBytes)
            ideDiskStats.dmaReadFullPages++;
        dmaReadCG->next();
    } else {
        assert(dmaReadCG->done());
        delete dmaReadCG;
        dmaReadCG = NULL;
        dmaReadDone();
    }
}

void
IdeDisk::dmaReadDone()
{
    uint32_t bytesWritten = 0;

    // write the data to the disk image
    for (bytesWritten = 0; bytesWritten < curPrd.getByteCount();
         bytesWritten += SectorSize) {

        cmdBytesLeft -= SectorSize;
        writeDisk(curSector++, (uint8_t *)(dataBuffer + bytesWritten));
    }

    // check for the EOT
    if (curPrd.getEOT()) {
        assert(cmdBytesLeft == 0);
        dmaState = Dma_Idle;
        updateState(ACT_DMA_DONE);
    } else {
        doDmaTransfer();
    }
}

void
IdeDisk::doDmaDataWrite()
{
    /** @todo we need to figure out what the delay actually will be */
    Tick totalDiskDelay = diskDelay + (curPrd.getByteCount() / SectorSize);
    uint32_t bytesRead = 0;

    DPRINTF(IdeDisk, "doDmaWrite, diskDelay: %d totalDiskDelay: %d\n",
            diskDelay, totalDiskDelay);

    memset(dataBuffer, 0, MAX_DMA_SIZE);
    assert(cmdBytesLeft <= MAX_DMA_SIZE);
    while (bytesRead < curPrd.getByteCount()) {
        readDisk(curSector++, (uint8_t *)(dataBuffer + bytesRead));
        bytesRead += SectorSize;
        cmdBytesLeft -= SectorSize;
    }
    DPRINTF(IdeDisk, "doDmaWrite, bytesRead: %d cmdBytesLeft: %d\n",
            bytesRead, cmdBytesLeft);

    schedule(dmaWriteWaitEvent, curTick() + totalDiskDelay);
}

void
IdeDisk::doDmaWrite()
{
    if (dmaAborted) {
        DPRINTF(IdeDisk, "DMA Aborted while doing DMA Write\n");
        if (dmaWriteCG)
            delete dmaWriteCG;
        dmaWriteCG = NULL;
        updateState(ACT_CMD_ERROR);
        return;
    }
    if (!dmaWriteCG) {
        // clear out the data buffer
        dmaWriteCG = new ChunkGenerator(curPrd.getBaseAddr(),
                curPrd.getByteCount(), chunkBytes);
    }
    if (ctrl->dmaPending() || ctrl->drainState() != DrainState::Running) {
        schedule(dmaWriteWaitEvent, curTick() + DMA_BACKOFF_PERIOD);
        DPRINTF(IdeDisk, "doDmaWrite: rescheduling\n");
        return;
    } else if (!dmaWriteCG->done()) {
        assert(dmaWriteCG->complete() < MAX_DMA_SIZE);
        ctrl->dmaWrite(pciToDma(dmaWriteCG->addr()), dmaWriteCG->size(),
                &dmaWriteWaitEvent, dataBuffer + dmaWriteCG->complete());
        DPRINTF(IdeDisk, "doDmaWrite: not done curPrd byte count %d, eot %#x\n",
                curPrd.getByteCount(), curPrd.getEOT());
        ideDiskStats.dmaWriteBytes += dmaWriteCG->size();
        ideDiskStats.dmaWriteTxs++;
        if (dmaWriteCG->size() == chunkBytes)
            ideDiskStats.dmaWriteFullPages++;
        dmaWriteCG->next();
    } else {
        DPRINTF(IdeDisk, "doDmaWrite: done curPrd byte count %d, eot %#x\n",
                curPrd.getByteCount(), curPrd.getEOT());
        assert(dmaWriteCG->done());
        delete dmaWriteCG;
        dmaWriteCG = NULL;
        dmaWriteDone();
    }
}

void
IdeDisk::dmaWriteDone()
{
    DPRINTF(IdeDisk,
            "doWriteDone: curPrd byte count %d, eot %#x cmd bytes left:%d\n",
            curPrd.getByteCount(), curPrd.getEOT(), cmdBytesLeft);
    // check for the EOT
    if (curPrd.getEOT()) {
        assert(cmdBytesLeft == 0);
        dmaState = Dma_Idle;
        updateState(ACT_DMA_DONE);
    } else {
        doDmaTransfer();
    }
}

////
// Disk utility routines
///

void
IdeDisk::readDisk(uint32_t sector, uint8_t *data)
{
    uint32_t bytesRead = image->read(data, sector);

    panic_if(bytesRead != SectorSize,
            "Can't read from %s. Only %d of %d read. errno=%d",
            name(), bytesRead, SectorSize, errno);
}

void
IdeDisk::writeDisk(uint32_t sector, uint8_t *data)
{
    uint32_t bytesWritten = image->write(data, sector);

    panic_if(bytesWritten != SectorSize,
            "Can't write to %s. Only %d of %d written. errno=%d",
            name(), bytesWritten, SectorSize, errno);
}

////
// Setup and handle commands
////

void
IdeDisk::startDma(const uint32_t &prdTableBase)
{
    panic_if(dmaState != Dma_Start,
            "Inconsistent DMA state, should be in Dma_Start!");

    panic_if(devState != Transfer_Data_Dma,
            "Inconsistent device state for DMA start!");

    // PRD base address is given by bits 31:2
    curPrdAddr = pciToDma((Addr)(prdTableBase & ~0x3ULL));

    dmaState = Dma_Transfer;

    // schedule dma transfer (doDmaTransfer)
    schedule(dmaTransferEvent, curTick() + 1);
}

void
IdeDisk::abortDma()
{
    panic_if(dmaState == Dma_Idle,
            "Inconsistent DMA state, should be Start or Transfer!");

    panic_if(devState != Transfer_Data_Dma && devState != Prepare_Data_Dma,
            "Inconsistent device state, should be Transfer or Prepare!");

    updateState(ACT_CMD_ERROR);
}

void
IdeDisk::startCommand()
{
    DevAction_t action = ACT_NONE;
    uint32_t size = 0;
    dmaRead = false;

    // Clear any existing errors.
    replaceBits(status, 0, 0);
    replaceBits(cmdReg.error, 2, 0);

    // Decode commands
    switch (cmdReg.command) {
        // Supported non-data commands
      case WDSF_READ_NATIVE_MAX:
        size = (uint32_t)image->size() - 1;
        cmdReg.sec_num = (size & 0xff);
        cmdReg.cyl_low = ((size & 0xff00) >> 8);
        cmdReg.cyl_high = ((size & 0xff0000) >> 16);
        cmdReg.head = ((size & 0xf000000) >> 24);

        devState = Command_Execution;
        action = ACT_CMD_COMPLETE;
        break;

      case WDCC_RECAL:
      case WDCC_IDP:
      case WDCC_STANDBY_IMMED:
      case WDCC_FLUSHCACHE:
      case WDSF_VERIFY:
      case WDSF_SEEK:
      case SET_FEATURES:
      case WDCC_SETMULTI:
      case WDCC_IDLE:
        devState = Command_Execution;
        action = ACT_CMD_COMPLETE;
        break;

        // Supported PIO data-in commands
      case WDCC_IDENTIFY:
        cmdBytes = cmdBytesLeft = sizeof(struct ataparams);
        devState = Prepare_Data_In;
        action = ACT_DATA_READY;
        break;

      case ATAPI_IDENTIFY_DEVICE:
        // We're not an ATAPI device, so this command isn't implemented.
        devState = Command_Execution;
        action = ACT_CMD_ERROR;
        replaceBits(cmdReg.error, 2, 1);
        replaceBits(status, 0, 1);
        break;

      case WDCC_READMULTI:
      case WDCC_READ:
        panic_if(!(cmdReg.drive & DRIVE_LBA_BIT),
                "Attempt to perform CHS access, only supports LBA");

        if (cmdReg.sec_count == 0)
            cmdBytes = cmdBytesLeft = (256 * SectorSize);
        else
            cmdBytes = cmdBytesLeft = (cmdReg.sec_count * SectorSize);

        curSector = getLBABase();

        /** @todo make this a scheduled event to simulate disk delay */
        devState = Prepare_Data_In;
        action = ACT_DATA_READY;
        break;

        // Supported PIO data-out commands
      case WDCC_WRITEMULTI:
      case WDCC_WRITE:
        panic_if(!(cmdReg.drive & DRIVE_LBA_BIT),
                "Attempt to perform CHS access, only supports LBA");

        if (cmdReg.sec_count == 0)
            cmdBytes = cmdBytesLeft = (256 * SectorSize);
        else
            cmdBytes = cmdBytesLeft = (cmdReg.sec_count * SectorSize);
        DPRINTF(IdeDisk, "Setting cmdBytesLeft to %d\n", cmdBytesLeft);
        curSector = getLBABase();

        devState = Prepare_Data_Out;
        action = ACT_DATA_READY;
        break;

        // Supported DMA commands
      case WDCC_WRITEDMA:
        dmaRead = true;  // a write to the disk is a DMA read from memory
        [[fallthrough]];
      case WDCC_READDMA:
        panic_if(!(cmdReg.drive & DRIVE_LBA_BIT),
                "Attempt to perform CHS access, only supports LBA");

        if (cmdReg.sec_count == 0)
            cmdBytes = cmdBytesLeft = (256 * SectorSize);
        else
            cmdBytes = cmdBytesLeft = (cmdReg.sec_count * SectorSize);
        DPRINTF(IdeDisk, "Setting cmdBytesLeft to %d in readdma\n",
                cmdBytesLeft);

        curSector = getLBABase();

        devState = Prepare_Data_Dma;
        action = ACT_DMA_READY;
        break;

      default:
        panic("Unsupported ATA command: %#x\n", cmdReg.command);
    }

    if (action != ACT_NONE) {
        // set the BSY bit
        status |= STATUS_BSY_BIT;
        // clear the DRQ bit
        status &= ~STATUS_DRQ_BIT;
        // clear the DF bit
        status &= ~STATUS_DF_BIT;

        updateState(action);
    }
}

////
// Handle setting and clearing interrupts
////

void
IdeDisk::postInterrupt()
{
    DPRINTF(IdeDisk, "Posting Interrupt\n");
    panic_if(pendingInterrupt,
            "Attempt to post an interrupt with one pending");

    pendingInterrupt = true;

    assert(channel);
    channel->postInterrupt();
}

void
IdeDisk::clearInterrupt()
{
    DPRINTF(IdeDisk, "Clearing Interrupt\n");
    panic_if(!pendingInterrupt, "Attempt to clear a non-pending interrupt");

    pendingInterrupt = false;

    assert(channel);
    channel->clearInterrupt();
}

////
// Manage the device internal state machine
////

void
IdeDisk::updateState(DevAction_t action)
{
    switch (devState) {
      case Device_Srst:
        if (action == ACT_SRST_SET) {
            // set the BSY bit
            status |= STATUS_BSY_BIT;
        } else if (action == ACT_SRST_CLEAR) {
            // clear the BSY bit
            status &= ~STATUS_BSY_BIT;

            // reset the device state
            reset(devID);
        }
        break;

      case Device_Idle_S:
        if (action == ACT_SELECT_WRITE && !isDEVSelect()) {
            devState = Device_Idle_NS;
        } else if (action == ACT_CMD_WRITE) {
            startCommand();
        }

        break;

      case Device_Idle_SI:
        if (action == ACT_SELECT_WRITE && !isDEVSelect()) {
            devState = Device_Idle_NS;
            clearInterrupt();
        } else if (action == ACT_STAT_READ || isIENSet()) {
            devState = Device_Idle_S;
            clearInterrupt();
        } else if (action == ACT_CMD_WRITE) {
            clearInterrupt();
            startCommand();
        }

        break;

      case Device_Idle_NS:
        if (action == ACT_SELECT_WRITE && isDEVSelect()) {
            if (!isIENSet() && pendingInterrupt) {
                devState = Device_Idle_SI;
                postInterrupt();
            }
            if (isIENSet() || !pendingInterrupt) {
                devState = Device_Idle_S;
            }
        }
        break;

      case Command_Execution:
        if (action == ACT_CMD_ERROR || action == ACT_CMD_COMPLETE) {
            // clear the BSY bit
            setComplete();

            if (!isIENSet()) {
                devState = Device_Idle_SI;
                postInterrupt();
            } else {
                devState = Device_Idle_S;
            }
        }
        break;

      case Prepare_Data_In:
        if (action == ACT_CMD_ERROR) {
            // clear the BSY bit
            setComplete();

            if (!isIENSet()) {
                devState = Device_Idle_SI;
                postInterrupt();
            } else {
                devState = Device_Idle_S;
            }
        } else if (action == ACT_DATA_READY) {
            // clear the BSY bit
            status &= ~STATUS_BSY_BIT;
            // set the DRQ bit
            status |= STATUS_DRQ_BIT;

            // copy the data into the data buffer
            if (cmdReg.command == WDCC_IDENTIFY ||
                cmdReg.command == ATAPI_IDENTIFY_DEVICE) {
                // Reset the drqBytes for this block
                drqBytesLeft = sizeof(struct ataparams);

                memcpy((void *)dataBuffer, (void *)&driveID,
                       sizeof(struct ataparams));
            } else {
                // Reset the drqBytes for this block
                drqBytesLeft = SectorSize;

                readDisk(curSector++, dataBuffer);
            }

            // put the first two bytes into the data register
            memcpy((void *)&cmdReg.data, (void *)dataBuffer,
                   sizeof(uint16_t));

            if (!isIENSet()) {
                devState = Data_Ready_INTRQ_In;
                postInterrupt();
            } else {
                devState = Transfer_Data_In;
            }
        }
        break;

      case Data_Ready_INTRQ_In:
        if (action == ACT_STAT_READ) {
            devState = Transfer_Data_In;
            clearInterrupt();
        }
        break;

      case Transfer_Data_In:
        if (action == ACT_DATA_READ_BYTE || action == ACT_DATA_READ_SHORT) {
            if (action == ACT_DATA_READ_BYTE) {
                panic("DEBUG: READING DATA ONE BYTE AT A TIME!\n");
            } else {
                drqBytesLeft -= 2;
                cmdBytesLeft -= 2;

                // copy next short into data registers
                if (drqBytesLeft)
                    memcpy((void *)&cmdReg.data,
                           (void *)&dataBuffer[SectorSize - drqBytesLeft],
                           sizeof(uint16_t));
            }

            if (drqBytesLeft == 0) {
                if (cmdBytesLeft == 0) {
                    // Clear the BSY bit
                    setComplete();
                    devState = Device_Idle_S;
                } else {
                    devState = Prepare_Data_In;
                    // set the BSY_BIT
                    status |= STATUS_BSY_BIT;
                    // clear the DRQ_BIT
                    status &= ~STATUS_DRQ_BIT;

                    /** @todo change this to a scheduled event to simulate
                        disk delay */
                    updateState(ACT_DATA_READY);
                }
            }
        }
        break;

      case Prepare_Data_Out:
        if (action == ACT_CMD_ERROR || cmdBytesLeft == 0) {
            // clear the BSY bit
            setComplete();

            if (!isIENSet()) {
                devState = Device_Idle_SI;
                postInterrupt();
            } else {
                devState = Device_Idle_S;
            }
        } else if (action == ACT_DATA_READY && cmdBytesLeft != 0) {
            // clear the BSY bit
            status &= ~STATUS_BSY_BIT;
            // set the DRQ bit
            status |= STATUS_DRQ_BIT;

            // clear the data buffer to get it ready for writes
            memset(dataBuffer, 0, MAX_DMA_SIZE);

            // reset the drqBytes for this block
            drqBytesLeft = SectorSize;

            if (cmdBytesLeft == cmdBytes || isIENSet()) {
                devState = Transfer_Data_Out;
            } else {
                devState = Data_Ready_INTRQ_Out;
                postInterrupt();
            }
        }
        break;

      case Data_Ready_INTRQ_Out:
        if (action == ACT_STAT_READ) {
            devState = Transfer_Data_Out;
            clearInterrupt();
        }
        break;

      case Transfer_Data_Out:
        if (action == ACT_DATA_WRITE_BYTE ||
            action == ACT_DATA_WRITE_SHORT) {

            if (action == ACT_DATA_READ_BYTE) {
                panic("DEBUG: WRITING DATA ONE BYTE AT A TIME!\n");
            } else {
                // copy the latest short into the data buffer
                memcpy((void *)&dataBuffer[SectorSize - drqBytesLeft],
                       (void *)&cmdReg.data,
                       sizeof(uint16_t));

                drqBytesLeft -= 2;
                cmdBytesLeft -= 2;
            }

            if (drqBytesLeft == 0) {
                // copy the block to the disk
                writeDisk(curSector++, dataBuffer);

                // set the BSY bit
                status |= STATUS_BSY_BIT;
                // set the seek bit
                status |= STATUS_SEEK_BIT;
                // clear the DRQ bit
                status &= ~STATUS_DRQ_BIT;

                devState = Prepare_Data_Out;

                /** @todo change this to a scheduled event to simulate
                    disk delay */
                updateState(ACT_DATA_READY);
            }
        }
        break;

      case Prepare_Data_Dma:
        if (action == ACT_CMD_ERROR) {
            // clear the BSY bit
            setComplete();

            if (!isIENSet()) {
                devState = Device_Idle_SI;
                postInterrupt();
            } else {
                devState = Device_Idle_S;
            }
        } else if (action == ACT_DMA_READY) {
            // clear the BSY bit
            status &= ~STATUS_BSY_BIT;
            // set the DRQ bit
            status |= STATUS_DRQ_BIT;

            devState = Transfer_Data_Dma;

            if (dmaState != Dma_Idle)
                panic("Inconsistent DMA state, should be Dma_Idle\n");

            dmaState = Dma_Start;
            // wait for the write to the DMA start bit
        }
        break;

      case Transfer_Data_Dma:
        if (action == ACT_CMD_ERROR) {
            dmaAborted = true;
            devState = Device_Dma_Abort;
        } else if (action == ACT_DMA_DONE) {
            // clear the BSY bit
            setComplete();
            // set the seek bit
            status |= STATUS_SEEK_BIT;
            // clear the controller state for DMA transfer
            channel->setDmaComplete();

            if (!isIENSet()) {
                devState = Device_Idle_SI;
                postInterrupt();
            } else {
                devState = Device_Idle_S;
            }
        }
        break;

      case Device_Dma_Abort:
        if (action == ACT_CMD_ERROR) {
            setComplete();
            status |= STATUS_SEEK_BIT;
            channel->setDmaComplete();
            dmaAborted = false;
            dmaState = Dma_Idle;

            if (!isIENSet()) {
                devState = Device_Idle_SI;
                postInterrupt();
            } else {
                devState = Device_Idle_S;
            }
        } else {
            DPRINTF(IdeDisk, "Disk still busy aborting previous DMA command\n");
        }
        break;

      default:
        panic("Unknown IDE device state: %#x\n", devState);
    }
}

void
IdeDisk::serialize(CheckpointOut &cp) const
{
    // Check all outstanding events to see if they are scheduled
    // these are all mutually exclusive
    Tick reschedule = 0;
    Events_t event = None;

    int eventCount = 0;

    if (dmaTransferEvent.scheduled()) {
        reschedule = dmaTransferEvent.when();
        event = Transfer;
        eventCount++;
    }
    if (dmaReadWaitEvent.scheduled()) {
        reschedule = dmaReadWaitEvent.when();
        event = ReadWait;
        eventCount++;
    }
    if (dmaWriteWaitEvent.scheduled()) {
        reschedule = dmaWriteWaitEvent.when();
        event = WriteWait;
        eventCount++;
    }
    if (dmaPrdReadEvent.scheduled()) {
        reschedule = dmaPrdReadEvent.when();
        event = PrdRead;
        eventCount++;
    }
    if (dmaReadEvent.scheduled()) {
        reschedule = dmaReadEvent.when();
        event = DmaRead;
        eventCount++;
    }
    if (dmaWriteEvent.scheduled()) {
        reschedule = dmaWriteEvent.when();
        event = DmaWrite;
        eventCount++;
    }

    assert(eventCount <= 1);

    SERIALIZE_SCALAR(reschedule);
    SERIALIZE_ENUM(event);

    // Serialize device registers
    SERIALIZE_SCALAR(cmdReg.data);
    SERIALIZE_SCALAR(cmdReg.sec_count);
    SERIALIZE_SCALAR(cmdReg.sec_num);
    SERIALIZE_SCALAR(cmdReg.cyl_low);
    SERIALIZE_SCALAR(cmdReg.cyl_high);
    SERIALIZE_SCALAR(cmdReg.drive);
    SERIALIZE_SCALAR(cmdReg.command);
    SERIALIZE_SCALAR(status);
    SERIALIZE_SCALAR(nIENBit);
    SERIALIZE_SCALAR(devID);

    // Serialize the PRD related information
    SERIALIZE_SCALAR(curPrd.entry.baseAddr);
    SERIALIZE_SCALAR(curPrd.entry.byteCount);
    SERIALIZE_SCALAR(curPrd.entry.endOfTable);
    SERIALIZE_SCALAR(curPrdAddr);

    /** @todo need to serialized chunk generator stuff!! */
    // Serialize current transfer related information
    SERIALIZE_SCALAR(cmdBytesLeft);
    SERIALIZE_SCALAR(cmdBytes);
    SERIALIZE_SCALAR(drqBytesLeft);
    SERIALIZE_SCALAR(curSector);
    SERIALIZE_SCALAR(dmaRead);
    paramOut(cp, "intrPending", pendingInterrupt);
    SERIALIZE_SCALAR(dmaAborted);
    SERIALIZE_ENUM(devState);
    SERIALIZE_ENUM(dmaState);
    SERIALIZE_ARRAY(dataBuffer, MAX_DMA_SIZE);
}

void
IdeDisk::unserialize(CheckpointIn &cp)
{
    // Reschedule events that were outstanding
    // these are all mutually exclusive
    Tick reschedule = 0;
    Events_t event = None;

    UNSERIALIZE_SCALAR(reschedule);
    UNSERIALIZE_ENUM(event);

    switch (event) {
      case None : break;
      case Transfer : schedule(dmaTransferEvent, reschedule); break;
      case ReadWait : schedule(dmaReadWaitEvent, reschedule); break;
      case WriteWait : schedule(dmaWriteWaitEvent, reschedule); break;
      case PrdRead : schedule(dmaPrdReadEvent, reschedule); break;
      case DmaRead : schedule(dmaReadEvent, reschedule); break;
      case DmaWrite : schedule(dmaWriteEvent, reschedule); break;
    }

    // Unserialize device registers
    UNSERIALIZE_SCALAR(cmdReg.data);
    UNSERIALIZE_SCALAR(cmdReg.sec_count);
    UNSERIALIZE_SCALAR(cmdReg.sec_num);
    UNSERIALIZE_SCALAR(cmdReg.cyl_low);
    UNSERIALIZE_SCALAR(cmdReg.cyl_high);
    UNSERIALIZE_SCALAR(cmdReg.drive);
    UNSERIALIZE_SCALAR(cmdReg.command);
    UNSERIALIZE_SCALAR(status);
    UNSERIALIZE_SCALAR(nIENBit);
    UNSERIALIZE_SCALAR(devID);

    // Unserialize the PRD related information
    UNSERIALIZE_SCALAR(curPrd.entry.baseAddr);
    UNSERIALIZE_SCALAR(curPrd.entry.byteCount);
    UNSERIALIZE_SCALAR(curPrd.entry.endOfTable);
    UNSERIALIZE_SCALAR(curPrdAddr);

    /** @todo need to serialized chunk generator stuff!! */
    // Unserialize current transfer related information
    UNSERIALIZE_SCALAR(cmdBytes);
    UNSERIALIZE_SCALAR(cmdBytesLeft);
    UNSERIALIZE_SCALAR(drqBytesLeft);
    UNSERIALIZE_SCALAR(curSector);
    UNSERIALIZE_SCALAR(dmaRead);
    paramIn(cp, "intrPending", pendingInterrupt);
    UNSERIALIZE_SCALAR(dmaAborted);
    UNSERIALIZE_ENUM(devState);
    UNSERIALIZE_ENUM(dmaState);
    UNSERIALIZE_ARRAY(dataBuffer, MAX_DMA_SIZE);
}

} // namespace gem5

/*
 * Copyright (c) 2013-2015 ARM Limited
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
 * Authors: Rene de Jong
 */


/** @file
 * This is a base class for UFS devices
 * The UFS interface consists out of one host controller which connects a
 * number of devices which together contain up to 8 logic units. A Logical
 * Unit is an externally addressable, independent, processing entity that
 * processes SCSI tasks (commands) and performs task management functions.
 * The decision has been made to abstract the device away, and model the
 * different logic units. Effectively this means that there is one Host layer
 * which handles all the UFS commands (everything UTP, UPIU and UNIpro
 * related) and a SCSI layer, which handles the SCSI interpretation and the
 * interaction with the "disk" (effectively the LBA that that particular
 * Logic Unit controls). Each Logic unit should therefor control a disk image
 * and a timing model. The UFS protocol has three types of commands
 * (explained later). Each has different phases and each phase need to wait
 * for its particular data. This is the reason that this model contains a lot
 * of events. For clarity, a state diagram in doxygen has been provided. To
 * fully apreciate the stages that the model flows through, the states have
 * been broken into three state diagrams. It is best if one imagines the
 * command flow state machine to be happening in the UFSHost layer, and the
 * other two to flow through all the layers of the model (UFS host, SCSI and
 * NVM model). See it as a quarry, one state diagram is moving the crane into
 * place, and the other ones are transporting the dirt down, or the valuables
 * up. For complete information about the working of UFS please refer to
 * http://www.jedec.org/standards-documents/results/jesd220 or
 * http://www.jedec.org/standards-documents/results/jesd223
 * The documents are available free of charge, although you will need to have
 * an acount.
 */

/** UFS command flow state machine
 *digraph CommandFlow{
    node [fontsize=10];
    IDLE -> transferHandler
    [ label=" transfer/task/command request " fontsize=6];
    transferHandler -> command
    [ label=" It is a command " fontsize=6];
    command -> IDLE
    [ label=" Command done, no further action " fontsize=6];
    transferHandler -> taskStart
    [ label=" It is a task " fontsize=6];
    taskStart -> finalUTP
    [ label=" Task handled, now acknowledge (UFS) " fontsize=6];
    transferHandler -> transferStart
    [ label=" It is a transfer " fontsize=6];
    transferStart -> SCSIResume
    [ label=" Transfer, obtain the specific command " fontsize=6];
    SCSIResume -> DiskDataFlowPhase
    [ label=" Disk data transfer (see other graphs) " fontsize=6];
    SCSIResume -> DeviceDataPhase
    [ label=" Device info transfer (handled in SCSIResume) "
        fontsize=6];
    DiskDataFlowPhase -> transferDone
    [ label=" Transfer done, acknowledge SCSI command " fontsize=6];
    DeviceDataPhase -> transferDone
    [ label=" Transfer done, acknowledge SCSI command " fontsize=6];
    transferDone -> finalUTP
    [ label=" Transfer handled, now acknowledge (UFS) " fontsize=6];
    finalUTP -> readDone
    [ label=" All handled, clear data structures " fontsize=6];
    readDone -> IDLE
    [ label=" All handled, nothing outstanding " fontsize=6];
    readDone -> transferHandler
    [ label=" All handled, handle next outstanding " fontsize=6];
    }
 */
/** UFS read transaction flow state machine
 digraph readFlow{
    node [fontsize=10];
    getScatterGather -> commitReadFromDisk
    [ label=" Put the information about the data transfer to the disk "
        fontsize=6];
    commitReadFromDisk -> waitForReads
    [ label=" Push the reads to the flashmodel and wait for callbacks "
        fontsize=6];
    waitForReads -> pushToDMA
    [ label=" Push to the DMA and wait for them to finish " fontsize=6];
    pushToDMA -> waitForReads
    [ label=" Wait for the next disk event " fontsize=6];
    pushToDMA -> waitForDMA
    [ label=" Wait for the last DMA transfer to finish " fontsize=6];
    waitForDMA -> finishTransfer
    [ label=" Continue with the command flow " fontsize=6];
    }
 */
/** UFS write transaction flow state machine
 digraph WriteFlow{
    node [fontsize=10];
    getScatterGather -> getFromDMA
    [ label=" Put the transfer information to the DMA " fontsize=6];
    getFromDMA -> waitForDMA
    [ label=" Wait for dma actions to arrive " fontsize=6];
    waitForDMA -> pushToDisk
    [ label=" Push arrived DMA to disk " fontsize=6];
    pushToDisk -> waitForDMA
    [ label=" Wait for next DMA action " fontsize=6];
    pushToDisk -> waitForDisk
    [ label=" All DMA actions are done, wait for disk " fontsize=6];
    waitForDisk -> finishTransfer
    [ label=" All transactions are done , continue the command flow "
        fontsize=6];
    }
 */

#ifndef __DEV_ARM_UFS_DEVICE_HH__
#define __DEV_ARM_UFS_DEVICE_HH__

#include <deque>

#include "base/addr_range.hh"
#include "base/bitfield.hh"
#include "base/statistics.hh"
#include "debug/UFSHostDevice.hh"
#include "dev/arm/abstract_nvm.hh"
#include "dev/arm/base_gic.hh"
#include "dev/storage/disk_image.hh"
#include "dev/dma_device.hh"
#include "dev/io_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/UFSHostDevice.hh"
#include "sim/serialize.hh"
#include "sim/stats.hh"

/**
 * Host controller layer: This is your Host controller
 * This layer handles the UFS functionality.
 * It tracks all the different transaction stages and uses
 * the device layer and the flash layer to determine the transaction flow.
 */
class UFSHostDevice : public DmaDevice
{
  public:

    UFSHostDevice(const UFSHostDeviceParams* p);

    DrainState drain() override;
    void checkDrain();
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    /**
     * Host Controller Interface
     * This is a set of registers that allow the driver to control the
     * transactions to the flash devices.
     * As defined in:
     * http://www.jedec.org/standards-documents/results/jesd223
     */
    struct HCIMem {
        /**
         * Specify the host capabilities
         */
        uint32_t HCCAP;
        uint32_t HCversion;
        uint32_t HCHCDDID;
        uint32_t HCHCPMID;

        /**
         * Operation and runtime registers
         */
        uint32_t ORInterruptStatus;
        uint32_t ORInterruptEnable;
        uint32_t ORHostControllerStatus;
        uint32_t ORHostControllerEnable;
        uint32_t ORUECPA;
        uint32_t ORUECDL;
        uint32_t ORUECN;
        uint32_t ORUECT;
        uint32_t ORUECDME;
        uint32_t ORUTRIACR;

        /**
         * vendor specific register
         */
        uint32_t vendorSpecific;

        /**
         * Transfer control registers
         */
        uint32_t TRUTRLBA;
        uint32_t TRUTRLBAU;
        uint32_t TRUTRLDBR;
        uint32_t TRUTRLCLR;
        uint32_t TRUTRLRSR;

        /**
         * Task control registers
         */
        uint32_t TMUTMRLBA;
        uint32_t TMUTMRLBAU;
        uint32_t TMUTMRLDBR;
        uint32_t TMUTMRLCLR;
        uint32_t TMUTMRLRSR;

        /**
         * Command registers
         */
        uint32_t CMDUICCMDR;
        uint32_t CMDUCMDARG1;
        uint32_t CMDUCMDARG2;
        uint32_t CMDUCMDARG3;
    };

    /**
     * All the data structures are defined in the UFS standard
     * This standard be found at the JEDEC website free of charge
     * (login required):
     * http://www.jedec.org/standards-documents/results/jesd220
     */

    /**
     * struct UTPUPIUHeader - UPIU header structure
     * dWord0: UPIU header DW-0
     * dWord1: UPIU header DW-1
     * dWord2: UPIU header DW-2
     */
    struct UTPUPIUHeader {
        uint32_t dWord0;
        uint32_t dWord1;
        uint32_t dWord2;
    };

    /**
     * struct UTPUPIURSP - Response UPIU structure
     * header: UPIU header DW-0 to DW-2
     * residualTransferCount: Residual transfer count DW-3
     * reserved: Reserved DW-4 to DW-7
     * senseDataLen: Sense data length DW-8 U16
     * senseData: Sense data field DW-8 to DW-12
     */
    struct UTPUPIURSP {
        struct UTPUPIUHeader header;
        uint32_t residualTransferCount;
        uint32_t reserved[4];
        uint16_t senseDataLen;
        uint8_t senseData[18];
    };

    /**
     * struct UTPUPIUTaskReq - Task request UPIU structure
     * header - UPIU header structure DW0 to DW-2
     * inputParam1: Input param 1 DW-3
     * inputParam2: Input param 2 DW-4
     * inputParam3: Input param 3 DW-5
     * reserved: Reserver DW-6 to DW-7
     */
    struct UTPUPIUTaskReq {
        struct UTPUPIUHeader header;
        uint32_t inputParam1;
        uint32_t inputParam2;
        uint32_t inputParam3;
        uint32_t reserved[2];
    };

    /**
     * struct UFSHCDSGEntry - UFSHCI PRD Entry
     * baseAddr: Lower 32bit physical address DW-0
     * upperAddr: Upper 32bit physical address DW-1
     * reserved: Reserved for future use DW-2
     * size: size of physical segment DW-3
     */
    struct UFSHCDSGEntry {
        uint32_t baseAddr;
        uint32_t upperAddr;
        uint32_t reserved;
        uint32_t size;
    };

    /**
     * struct UTPTransferCMDDesc - UFS Commad Descriptor structure
     * commandUPIU: Command UPIU Frame address
     * responseUPIU: Response UPIU Frame address
     * PRDTable: Physcial Region Descriptor
     * All lengths as defined by JEDEC220
     */
    struct UTPTransferCMDDesc {
        uint8_t commandUPIU[128];
        uint8_t responseUPIU[128];
        struct UFSHCDSGEntry PRDTable[128];
    };

    /**
     * UPIU tranfer message.
     */
    struct UPIUMessage {
        struct UTPUPIUHeader header;
        uint32_t dataOffset;
        uint32_t dataCount;
        std::vector<uint32_t> dataMsg;
    };

    /**
     * struct UTPTransferReqDesc - UTRD structure
     * header: UTRD header DW-0 to DW-3
     * commandDescBaseAddrLo: UCD base address low DW-4
     * commandDescBaseAddrHi: UCD base address high DW-5
     * responseUPIULength: response UPIU length DW-6
     * responseUPIUOffset: response UPIU offset DW-6
     * PRDTableLength: Physical region descriptor length DW-7
     * PRDTableOffset: Physical region descriptor offset DW-7
     */
    struct UTPTransferReqDesc {

        /**
         * struct RequestDescHeader
         * dword0: Descriptor Header DW0
         * dword1: Descriptor Header DW1
         * dword2: Descriptor Header DW2
         * dword3: Descriptor Header DW3
         */
        struct RequestDescHeader {
            uint32_t dWord0;
            uint32_t dWord1;
            uint32_t dWord2;
            uint32_t dWord3;
        } header;

        /* DW 4-5*/
        uint32_t commandDescBaseAddrLo;
        uint32_t commandDescBaseAddrHi;

        /* DW 6 */
        uint16_t responseUPIULength;
        uint16_t responseUPIUOffset;

        /* DW 7 */
        uint16_t PRDTableLength;
        uint16_t PRDTableOffset;
    };

    /**
     * SCSI reply structure. In here is all the information that is needed to
     * build a SCSI reply.
     */
    struct SCSIReply {
        uint8_t status;
        uint32_t msgSize;
        uint8_t LUN;
        struct UPIUMessage message;
        uint8_t senseSize;
        uint8_t expectMore;//0x01 is for writes, 0x02 is for reads
        uint64_t offset;
        uint8_t senseCode[19];
    };

    /**
     * Logic unit information structure. SCSI requires information of each LUN.
     * This structure is defined in the SCSI standard, and can also be found in
     * the UFS standard. http://www.jedec.org/standards-documents/results/jesd220
     */
    struct LUNInfo {
        uint32_t dWord0;
        uint32_t dWord1;
        uint32_t vendor0;
        uint32_t vendor1;
        uint32_t product0;
        uint32_t product1;
        uint32_t product2;
        uint32_t product3;
        uint32_t productRevision;
    };

    /**
     * Different events, and scenarios require different types of information.
     * Keep in mind that for a read-from-disk transaction the host at first a
     * datastructure fetches to determine where and what the command is, then the
     * command fetches and the structure fetches to determine where the
     * different read transactions should be placed and then transfers all the
     * read fragments. It then answers to the original caller with two replies,
     * one for the command, and one for UFS. Each of these stages trigger a
     * different event, and each event needs to know what happened in the
     * previous stage and what is going to happen in the current one. This
     * happens also for writes, SCSI maintanance, UFS maintanance, command
     * management and task management.
     */

    /**
     * Transfer information.
     * @filePointer this does not point to a file, but to a position on the disk
     * image (which is from the software systems perspective a position in a file)
     */
    struct transferInfo {
        std::vector <uint8_t> buffer;
        uint32_t size;
        uint64_t offset;
        uint32_t filePointer;
        uint32_t lunID;
    };

    /**
     * transfer completion info.
     * This information is needed by transferDone to finish the transfer.
     */
    struct transferDoneInfo {
        Addr responseStartAddr;
        uint32_t reqPos;
        struct UTPUPIURSP requestOut;
        uint32_t size;
        Addr address;
        uint8_t *destination;
        bool finished;
        uint32_t lunID;
    };

    /**
     * Transfer start information.
     */
    struct transferStart {
        struct UTPTransferReqDesc* destination;
        uint32_t mask;
        Addr address;
        uint32_t size;
        uint32_t done;
        uint32_t lun_id;
    };

    /**
     * Task start information. This is for the device, so no lun id needed.
     */
    struct taskStart {
        struct UTPUPIUTaskReq destination;
        uint32_t mask;
        Addr address;
        uint32_t size;
        bool done;
    };

    /**
     * After a SCSI command has been identified, the SCSI resume function will
     * handle it. This information will provide context information.
     */
    struct SCSIResumeInfo {
        struct UTPTransferReqDesc* RequestIn;
        int reqPos;
        Addr finalAddress;
        uint32_t finalSize;
        std::vector <uint8_t> destination;
        uint32_t done;
    };

    /**
     * Disk transfer burst information. Needed to allow communication between the
     * disk transactions and dma transactions.
     */
    struct writeToDiskBurst {
        Addr start;
        uint64_t SCSIDiskOffset;
        uint32_t size;
        uint32_t LUN;
    };

    /**
     * Statistics
     */
    struct UFSHostDeviceStats {
        /** Queue lengths */
        Stats::Scalar currentSCSIQueue;
        Stats::Scalar currentReadSSDQueue;
        Stats::Scalar currentWriteSSDQueue;

        /** Amount of data read/written */
        Stats::Scalar totalReadSSD;
        Stats::Scalar totalWrittenSSD;
        Stats::Scalar totalReadDiskTransactions;
        Stats::Scalar totalWriteDiskTransactions;
        Stats::Scalar totalReadUFSTransactions;
        Stats::Scalar totalWriteUFSTransactions;

        /** Average bandwidth for reads and writes */
        Stats::Formula averageReadSSDBW;
        Stats::Formula averageWriteSSDBW;

        /** Average Queue lengths*/
        Stats::Average averageSCSIQueue;
        Stats::Average averageReadSSDQueue;
        Stats::Average averageWriteSSDQueue;

        /** Number of doorbells rung*/
        Stats::Formula curDoorbell;
        Stats::Scalar maxDoorbell;
        Stats::Average averageDoorbell;

        /** Histogram of latencies*/
        Stats::Histogram transactionLatency;
        Stats::Histogram idleTimes;
    };

    /**
     * device layer: This is your Logic unit
     * This layer implements the SCSI functionality of the UFS Device
     * One logic unit controls one or more disk partitions
     */
    class UFSSCSIDevice: SimObject
    {
      public:
        /**
         * Constructor and destructor
         */
        UFSSCSIDevice(const UFSHostDeviceParams* p, uint32_t lun_id, Callback*
                      transfer_cb, Callback *read_cb);
        ~UFSSCSIDevice();

        /**
         * SCSI command handle function; determines what the command is and
         * returns a reply structure that allows the host device to continue
         * with the transfer.
         */
        struct SCSIReply SCSICMDHandle(uint32_t* SCSI_msg);

        /**
         * Disk access functions. These will transfer the data from/to the disk
         */

        /**
         * Read flash. read the data from the disk image. This function
         * doesn't model timing behaviour
         */
        void readFlash(uint8_t* readaddr, uint64_t offset, uint32_t size);

        /**
         * Write flash. write the data to the disk image. This function
         * doesn't model timing behaviour
         */
        void writeFlash(uint8_t* writeaddr, uint64_t offset, uint32_t size);

        /**
         * finished command. Probe to find out wether this logic unit
         * finished its transfer and triggered the callback; The host needs
         * this to handle the final part of the transaction.
         */
        bool finishedCommand() const {return transferCompleted;};

        /**
         * Clear signal. Handle for the host to clear the transfer complete
         * signal.
         */
        void clearSignal() {transferCompleted = false;};

        /**
         * Finished read. Probe to find out which logic unit finished its
         * read. This is needed, because multiple units can do transactions
         * at the same time, and need to push back data at the right time in
         * the right order. (because writes work the other way round, they do
         * not need this mechanism)
         */
        bool finishedRead() const {return readCompleted;};

        /**
         * Clear signal. Handle for the host to clear the read complete
         * signal.
         */
        void clearReadSignal() {readCompleted = false;};

        /**
         * Start the transactions to (and from) the disk
         * The host will queue all the transactions. Once the next phase
         * commences, this function should be started.
         */
        void SSDReadStart(uint32_t total_read);
        void SSDWriteStart();

        /**
         * Sets total amount of write transactions that needs to be made.
         * First they need to be fetched via DMA, so this value is needed in
         * a later stage.
         */
        void setTotalWrite(uint32_t total_write) {totalWrite = total_write;};

        /**
         * End of transfer information
         */
        transferDoneInfo transferInfo;

        /**
         * Information message queues, as there can be multiple messages
         * queued for handling in this system. These are the main
         * communication interfaces between the Host and the device layers
         */

        /**
         * SCSIInfoQueue: each LU handles its own SCSI commands.
         */
        std::deque<struct SCSIResumeInfo> SCSIInfoQueue;

        /**
         * SSDReadInfo: Structure from disk to dma, that contains data, and
         * helper info to get it to the right place in the memory.
         */
        std::deque<struct transferInfo> SSDReadInfo;

        /**
         * SSDWriteDoneInfo: Structure from dma to disk, that contains data,
         * and helper info to get it to the right place in the memory.
         * The done is added because it is going to the last phase of the
         * write transfer.
         */
        std::deque<struct transferInfo> SSDWriteDoneInfo;

      private:
        /**
         * Functions to indicate that the action to the SSD has completed.
         */
        /**
         * Read Call back; This is the callback function for the memory model
         */
        void readCallback();

        /**
         * SSD Read done; Determines if the final callback of the transaction
         * should be made at the end of a read transfer.
         */
        void SSDReadDone();

        /**
         * SSD Write Done; This is the callback function for the memory model.
         */
        void SSDWriteDone();

        /**
         * Status of SCSI. This may be linked to a status check in the future.
         * For now it (mainly) fills a data structure with sense information
         * for a successfull transaction
         */
        void statusCheck(uint8_t status, uint8_t* sensecodelist);

        /**
         * set signal to indicate that the transaction has been completed.
         */
        void setSignal() {transferCompleted = true;};

        /**
         * set signal to indicate that the read action has been completed
         */
        void setReadSignal() {readCompleted = true;};

        /**
         * The objects this model links to.
         * 1: the disk data model
         * 2: the memory timing model
         */
        DiskImage* flashDisk;
        AbstractNVM* flashDevice;

        /**
         * Logic unit dimensions
         */
        const uint32_t blkSize;
        const uint32_t lunAvail;
        const uint64_t diskSize;
        const uint32_t capacityLower;
        const uint32_t capacityUpper;

        /**
         * Logic unit info; needed for SCSI Info messages and LU
         * identification
         */
        struct LUNInfo lunInfo;
        const uint32_t lunID;

        /**
         * Signals to Host layer
         * 1: signal for transaction completion
         * 2: signal for read action completion
         */
        bool transferCompleted;
        bool readCompleted;

        /**
         * Total amount transactions that need to be made
         */
        uint32_t totalRead;
        uint32_t totalWrite;

        /**
         * transaction progress tracking
         */
        uint32_t amountOfWriteTransfers;
        uint32_t amountOfReadTransfers;

        /**
         * Callbacks between Host and Device
         */
        Callback* signalDone;
        Callback* deviceReadCallback;

        /**
         * Callbacks between Device and Memory
         */
        Callback* memReadCallback;
        Callback* memWriteCallback;

        /*
         * Default response header layout. For more information refer to
         * chapter 7 http://www.jedec.org/standards-documents/results/jesd220
         */
        static const unsigned int UPIUHeaderDataIndWord0 = 0x0000C022;
        static const unsigned int UPIUHeaderDataIndWord1 = 0x00000000;
        static const unsigned int UPIUHeaderDataIndWord2 = 0x40000000;

        /*
         * SCSI mode pages values assigned in ufs_device.cc
         * The mode pages give device specific information via the SCSI
         * protocol. They are defined in
         * http://www.jedec.org/standards-documents/results/jesd220
         */
        static const unsigned int controlPage[3];
        static const unsigned int recoveryPage[3];
        static const unsigned int cachingPage[5];

        /*
         * SCSI command set; defined in
         * http://www.jedec.org/standards-documents/results/jesd220
         */
        enum SCSICommandSet {
            SCSIInquiry = 0x12,
            SCSIRead6 = 0x08,
            SCSIRead10 = 0x28,
            SCSIRead16 = 0x88,
            SCSIReadCapacity10 = 0x25,
            SCSIReadCapacity16 = 0x9E,
            SCSIReportLUNs = 0xA0,
            SCSIStartStop = 0x1B,
            SCSITestUnitReady = 0x00,
            SCSIVerify10 = 0x2F,
            SCSIWrite6 = 0x0A,
            SCSIWrite10 = 0x2A,
            SCSIWrite16 = 0x8A,
            SCSIFormatUnit = 0x04,
            SCSISendDiagnostic = 0x1D,
            SCSISynchronizeCache = 0x35,
            //UFS SCSI additional command set for full functionality
            SCSIModeSelect10 = 0x55,
            SCSIModeSense6 = 0x1A,
            SCSIModeSense10 = 0x5A,
            SCSIRequestSense = 0x03,
            SCSIUnmap = 0x42,
            SCSIWriteBuffer = 0x3B,
            SCSIReadBuffer = 0x3C,
            //SCSI commands not supported by UFS; but Linux send them anyway
            SCSIMaintenanceIn = 0xA3
        };

        /*
         * SCSI status codes; defined in
         * http://www.jedec.org/standards-documents/results/jesd220
         */
        enum SCSIStatusCodes {
            SCSIGood = 0x00,
            SCSICheckCondition = 0x02,
            SCSIConditionGood = 0x04,
            SCSIBusy = 0x08,
            SCSIIntermediateGood = 0x10,
            SCSIIntermediatCGood = 0x14,
            SCSIReservationConflict = 0x18,
            SCSICommandTerminated = 0x22,
            SCSITaskSetFull = 0x28,
            SCSIACAActive = 0x30,
            SCSITaskAborted = 0x40
        };

        /*
         * SCSI sense codes; defined in
         * http://www.jedec.org/standards-documents/results/jesd220
         */
        enum SCSISenseCodes {
            SCSINoSense = 0x00,
            SCSIRecoverdError = 0x01,
            SCSINotReady = 0x02,
            SCSIMediumError = 0x03,
            SCSIHardwareError = 0x04,
            SCSIIllegalRequest = 0x05,
            SCSIUnitAttention = 0x06,
            SCSIDataProtect = 0x07,
            SCSIBlankCheck = 0x08,
            SCSIAbortedCommand = 0x0B,
            SCSIVolumeOverflow = 0x0D,
            SCSIMisCompare = 0x0E
        };

    };

    //All access functions are inherrited; no need to make them public
    /**
     * Address range functions
     */
    AddrRangeList getAddrRanges() const override;

    /**
     * register access functions
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    // end of access functions

    /**
     * Initialization function. Sets the sefault HCI register values
     */
    void setValues();

    /**
     * Handler functions. Each function handles a different stage of the
     * transfer. Note that the UFS protocol specifies three types of messages
     * to the host (and devices):
     * 1: Command (to Host specifically)
     * 2: Task (to device; to control flow, not for data)
     * 3: Transfer (to device; to transfer data)
     */
    /**
     * request handler. This function finds the cause of the request and
     * triggers the right follow-up action (command handler, task handler,
     * or transferhandler)
     */
    void requestHandler();

    /**
     * Command handler function. Handles the command send to the Host
     * controller
     */
    void commandHandler();

    /**
     * Task Start function. Starts the task handler once the task data
     * structure has arrived
     */
    void taskStart();

    /**
     * Task handler function. Handles the tasks send to the devices
     * because there are not many tasks implemented yet this is kept in the
     * Host controller layer
     */
    void taskHandler(struct UTPUPIUTaskReq* request_in,
                     uint32_t req_pos, Addr finaladdress, uint32_t finalsize);

    /**
     * Transfer Start function. Starts the transfer handler once the transfer
     * data structure has arrived
     */
    void transferStart();

    /**
     * Transfer handler function. Handles the transfers send to the devices
     * Important to understand here is that a Logic unit is not a device (a
     * device can contain multiple logic units). This function analyses the
     * first data structure that has been transfered. Which will tell the
     * host to expect SCSI frames for the rest of the transaction. Note that
     * the host has no indication whatsoever which LU to address. That will
     * follow in the next transaction.
     */
    void transferHandler(struct UTPTransferReqDesc*
                         request_in, int req_pos, Addr finaladdress,
                         uint32_t finalsize, uint32_t done);

    /**
     * Transfer SCSI function. Determines which Logic unit to address and
     * starts the SCSI resume function
     */
    void SCSIStart();

    /**
     * Starts the scsi handling function in the apropriate Logic unit,
     * prepares the right data transfer scheme and kicks it off.
     */
    void SCSIResume(uint32_t lun_id);

    /**
     * LU callback function to indicate that the action has completed.
     */
    void LUNSignal();

    /**
     * transfer done, the beginning of the final stage of the transfer.
     * Acknowledges UPIU frame and prepares the UTP response frame
     */
    void transferDone(Addr responseStartAddr, uint32_t req_pos,
                      struct UTPUPIURSP request_out, uint32_t size,
                      Addr address, uint8_t* destination, bool finished,
                      uint32_t lun_id);
    /**
     * final UTP, sends the last acknowledge data structure to the system;
     * prepares the clean up functions.
     */
    void finalUTP();

    /**
     * Interrupt control functions
     */
    void clearInterrupt();
    void generateInterrupt();

    /**
     * DMA transfer functions
     * These allow the host to push/pull the data to the memory
     * The provided event indicates what the next phase it that will handle
     * the obtained data, or what the follow up action is once the data has
     * been pushed to the memory
     */
    void writeDevice(Event* additional_action, bool toDisk, Addr start,
                     int size, uint8_t* destination, uint64_t SCSIDiskOffset,
                     uint32_t lun_id);
    void readDevice(bool lastTransfer, Addr SCSIStart, uint32_t SCSISize,
                    uint8_t* SCSIDestination, bool no_cache,
                    Event* additional_action);

    /**
     * Disk transfer management functions
     * these set up the queues, and initiated them, leading to the data
     * transaction timing model based on the scatter gather list constructed
     * in SCSIresume.
     */
    void manageWriteTransfer(uint8_t LUN, uint64_t offset, uint32_t
                             sg_table_length, struct UFSHCDSGEntry* sglist);
    void manageReadTransfer(uint32_t size, uint32_t LUN, uint64_t offset,
                            uint32_t sg_table_length,
                            struct UFSHCDSGEntry* sglist);

    /**
     * Read done
     * Started at the end of a transaction after the last read action. Cleans
     * up UTP descriptor and other remaining data structures. It also raises
     * the interrupt.
     */
    void readDone();

    /**
     * Write done
     * After a DMA write with data intended for the disk, this function is
     * called. It ensures that the disk image is modified, and that the
     * correct timing function is triggered.
     */
    void writeDone();

    /**
     * Read callback
     * Call back function for the logic units to indicate the completion of
     * a read action. Note that this is needed because the read functionality
     * needs to push data structures back to the memory.
     */
    void readCallback();

    /**
     * Read garbage
     * A read from disk data structure can vary in size and is therefor
     * allocated on creation. It can only be destroyed once that particular
     * read action has completed. This function is called on completion of a
     * read from disk action to handle this.
     */
    void readGarbage();

    /**register statistics*/
    void regStats() override;

    /**
     * Host controller information
     */
    const Addr pioAddr;
    const Addr pioSize;
    const Tick pioDelay;
    const int intNum;
    BaseGic* gic;
    const uint32_t lunAvail;
    const uint8_t UFSSlots;

    /**
     * Host controller memory
     */
    HCIMem UFSHCIMem;

    /**
     * Track number of DMA transactions in progress
     */
    int readPendingNum;
    int writePendingNum;

    /**
     * Statistics helper variables
     * Active doorbells indicates how many doorbells are in teh process of
     * being handled.
     * Pending doorbells have been handled and are waiting to be acknowledged
     * by the host system.
     * The doorbell register is 32 bits wide, so one byte is enough to keep
     * track of the numbers
     */
    uint8_t activeDoorbells;
    uint8_t pendingDoorbells;

    /**
     * interrupt verification
     * This keeps track of the number of interrupts generated. It is usefull
     * for debug purposes. Make sure that the implemented driver prints the
     * number of interrupts it has handled so far to fully benefit from this
     * feature.
     */
    uint32_t countInt;

    /**
     * Track the transfer
     * This is allows the driver to "group" certain transfers together by
     * using a tag in the UPIU. The messages with the same tag should be
     * handled together, i.e. their doorbells should be cleared when they are
     * all done. but we need to keep track of the ones we already handled,
     * this integer shadows the doorbells to allow this behaviour.
     */
    uint32_t transferTrack;
    uint32_t taskCommandTrack;

    /**
     * Helper for latency stats
     * These variables keep track of the latency for every doorbell.
     * Eventually the latenmcies will be put in a histogram.
     */
    Tick transactionStart[32];
    Tick idlePhaseStart;

    /**
     * logic units connected to the UFS Host device
     * Note again that the "device" as such is represented by one or multiple
     * logic units.
     */
    std::vector<UFSSCSIDevice*> UFSDevice;

    /**
     * SCSI reply structure, used for direct answering. Might be refered to
     * during the assembly of the reply (data, and response; e.g. if
     * something goes wrong along the way, the reply will be different)
     */
    struct SCSIReply request_out_datain;

    /**
     * SCSI resume info
     * information structure for SCSI resume. it keeps track of all the
     * information that is needed to successfully complete the transaction
     * (response addresses, communicated information so far, etc.).
     */
    struct SCSIResumeInfo SCSIInfo;

    /**
     * To finish the transaction one needs information about the original
     * message. This is stored in this queue
     * transferEnd uses the same structure as transferStartInfo, because all
     * the information it needs is in there. It improves readability in the
     * cc file.
     */
    std::deque<struct transferStart> transferEnd;

    /**
     * When a task/transfer is started it needs information about the
     * task/transfer it is about to perform. This is defined in these
     * structures. If multiple tasks/transfers are issued at the same time,
     * they still need to be fetched one by one. They then need to be
     * executed in the order specified by the UFS standard (least significant
     * doorbell first). The tasks/transfers are placed in the queue in that
     * specific order.
     */
    std::deque<struct taskStart> taskInfo;
    std::deque<struct transferStart> transferStartInfo;

    /**
     * Information to get a DMA transaction
     */
    std::deque<struct writeToDiskBurst> dmaWriteInfo;

    /**
     * Information from DMA transaction to disk
     */
    std::deque<struct transferInfo> SSDWriteinfo;

    /**
     * Information from the Disk, waiting to be pushed to the DMA
     */
    std::deque<struct transferInfo> SSDReadPending;

    /**
     * garbage queue, ensure clearing of the allocated memory
     */
    std::deque<struct UTPTransferReqDesc*> garbage;

    /**
     * RequestHandler stats
     */
    struct UFSHostDeviceStats stats;

    /**
     * Transfer flow events
     * Basically these events form two queues, one from memory to UFS device
     * (DMA) and one from device to flash (SSD). The SSD "queue" is
     * maintained by the flash and the lun classes and does not form a queue
     * of events as such, but rather a queue of information. This can be done
     * because the flow of the events is completely in the control of these
     * classes. (Whereas in the DMA case we rely on an external class)
     */
    std::deque<EventFunctionWrapper> readDoneEvent;
    std::deque<EventFunctionWrapper> writeDoneEvent;

    /**
     * Callbacks for the logic units. One to indicate the completion of a
     * transaction, the other one to indicate the completion of a read
     * action.
     */
    Callback* transferDoneCallback;
    Callback* memReadCallback;

    /**
     * The events that control the functionality.
     * After a doorbell has been set, either a taskevent or a transfer event
     * is scheduled. A transfer event might schedule a SCSI event, all events
     * sequences end with an UTP event, which can be considered as the event
     * which answers the doorbell.
     */
    /**
     * Wait for the SCSI specific data to arive
     */
    EventFunctionWrapper SCSIResumeEvent;

    /**
     * Wait for the moment where we can send the last frame
     */
    EventFunctionWrapper UTPEvent;

    /**
     * Event after a read to clean up the UTP data structures
     */
    std::deque<EventFunctionWrapper> readGarbageEventQueue;

    /**
     * Multiple tasks transfers can be scheduled at once for the device, the
     * only thing we know for sure about them is that they will happen in a
     * first come first serve order; hence we need to queue.
     */
    std::deque<EventFunctionWrapper> taskEventQueue;
    std::deque<EventFunctionWrapper> transferEventQueue;

    /**
     * Bits of interest within UFS data packages
     */
    static const unsigned int UTPTransferREQCOMPL = 0x01;//UFS_BIT(0)
    static const unsigned int UTPTaskREQCOMPL = 0x200;//UFS_BIT(9)
    static const unsigned int UICCommandCOMPL = 0x400;//UFS_BIT(10)
    static const unsigned int UICCommandReady = 0x08;//UFS_BIT(3)

    /*
     * UFSHCI Registers; please refer to
     * http://www.jedec.org/standards-documents/results/jesd223
     * for their definition.
     */
    enum UFSHCIRegisters {
        regControllerCapabilities = 0x00,
        regUFSVersion = 0x08,
        regControllerDEVID = 0x10,
        regControllerPRODID = 0x14,
        regInterruptStatus = 0x20,
        regInterruptEnable = 0x24,
        regControllerStatus = 0x30,
        regControllerEnable = 0x34,
        regUICErrorCodePHYAdapterLayer = 0x38,
        regUICErrorCodeDataLinkLayer = 0x3C,
        regUICErrorCodeNetworkLayer = 0x40,
        regUICErrorCodeTransportLayer = 0x44,
        regUICErrorCodeDME = 0x48,
        regUTPTransferREQINTAGGControl = 0x4C,
        regUTPTransferREQListBaseL = 0x50,
        regUTPTransferREQListBaseH = 0x54,
        regUTPTransferREQDoorbell = 0x58,
        regUTPTransferREQListClear = 0x5C,
        regUTPTransferREQListRunStop = 0x60,
        regUTPTaskREQListBaseL = 0x70,
        regUTPTaskREQListBaseH = 0x74,
        regUTPTaskREQDoorbell = 0x78,
        regUTPTaskREQListClear = 0x7C,
        regUTPTaskREQListRunStop = 0x80,
        regUICCommand = 0x90,
        regUICCommandArg1 = 0x94,
        regUICCommandArg2 = 0x98,
        regUICCommandArg3 = 0x9C
    };
};

#endif //__DEV_ARM_UFS_DEVICE_HH__

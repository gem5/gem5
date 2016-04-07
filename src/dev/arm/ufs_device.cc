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
 * This is a simulation model for a UFS interface
 * The UFS interface consists of a host controller and (at least) one device.
 * The device can contain multiple logic units.
 * To make this interface as usefull as possible for future development, the
 * decision has been made to split the UFS functionality from the SCSI
 * functionality. The class UFS SCSIDevice can therefor be used as a starting
 * point for creating a more generic SCSI device. This has as a consequence
 * that the UFSHostDevice class contains functionality from both the host
 * controller and the device. The current UFS standard (1.1) allows only one
 * device, and up to 8 logic units. the logic units only handle the SCSI part
 * of the command, and the device mainly the UFS part. Yet the split between
 * the SCSIresume function and the SCSICMDHandle might seem a bit awkward.
 * The SCSICMDHandle function is in essence a SCSI reply generator, and it
 * distils the essential information from the command. A disktransfer cannot
 * be made from this position because the scatter gather list is not included
 * in the SCSI command, but in the Transfer Request descriptor. The device
 * needs to manage the data transfer. This file is build up as follows: first
 * the UFSSCSIDevice functions will be presented; then the UFSHostDevice
 * functions. The UFSHostDevice functions are split in three parts: UFS
 * transaction flow, data write transfer and data read transfer. The
 * functions are then ordered in the order in which a transfer takes place.
 */

/**
 * Reference material can be found at the JEDEC website:
 * UFS standard
 * http://www.jedec.org/standards-documents/results/jesd220
 * UFS HCI specification
 * http://www.jedec.org/standards-documents/results/jesd223
 */

#include "dev/arm/ufs_device.hh"

/**
 * Constructor and destructor functions of UFSHCM device
 */
UFSHostDevice::UFSSCSIDevice::UFSSCSIDevice(const UFSHostDeviceParams* p,
                             uint32_t lun_id, Callback *transfer_cb,
                             Callback *read_cb):
    SimObject(p),
    flashDisk(p->image[lun_id]),
    flashDevice(p->internalflash[lun_id]),
    blkSize(p->img_blk_size),
    lunAvail(p->image.size()),
    diskSize(flashDisk->size()),
    capacityLower((diskSize - 1) & 0xffffffff),
    capacityUpper((diskSize - SectorSize) >> 32),
    lunID(lun_id),
    transferCompleted(false),
    readCompleted(false),
    totalRead(0),
    totalWrite(0),
    amountOfWriteTransfers(0),
    amountOfReadTransfers(0)
{
    /**
     * These callbacks are used to communicate the events that are
     * triggered upstream; e.g. from the Memory Device to the UFS SCSI Device
     * or from the UFS SCSI device to the UFS host.
     */
    signalDone = transfer_cb;
    memReadCallback = new MakeCallback<UFSSCSIDevice,
        &UFSHostDevice::UFSSCSIDevice::readCallback>(this);
    deviceReadCallback = read_cb;
    memWriteCallback = new MakeCallback<UFSSCSIDevice,
        &UFSHostDevice::UFSSCSIDevice::SSDWriteDone>(this);

    /**
     * make ascii out of lun_id (and add more characters)
     * UFS allows up to 8 logic units, so the numbering should work out
     */
    uint32_t temp_id = ((lun_id | 0x30) << 24) | 0x3A4449;
    lunInfo.dWord0 = 0x02060000;   //data
    lunInfo.dWord1 = 0x0200001F;
    lunInfo.vendor0 = 0x484D5241;  //ARMH (HMRA)
    lunInfo.vendor1 = 0x424D4143;  //CAMB (BMAC)
    lunInfo.product0 = 0x356D6567; //gem5  (5meg)
    lunInfo.product1 = 0x4D534655; //UFSM (MSFU)
    lunInfo.product2 = 0x4C45444F; //ODEL (LEDO)
    lunInfo.product3 = temp_id;    // ID:"lun_id" ("lun_id":DI)
    lunInfo.productRevision = 0x01000000; //0x01

    DPRINTF(UFSHostDevice, "Logic unit %d assumes that %d logic units are"
            " present in the system\n", lunID, lunAvail);
    DPRINTF(UFSHostDevice,"The disksize of lun: %d should be %d blocks\n",
            lunID, diskSize);
    flashDevice->initializeMemory(diskSize, SectorSize);
}


/**
 * These pages are SCSI specific. For more information refer to:
 * Universal Flash Storage (UFS) JESD220 FEB 2011 (JEDEC)
 * http://www.jedec.org/standards-documents/results/jesd220
 */
const unsigned int UFSHostDevice::UFSSCSIDevice::controlPage[3] =
                                                {0x01400A0A, 0x00000000,
                                                 0x0000FFFF};
const unsigned int UFSHostDevice::UFSSCSIDevice::recoveryPage[3] =
                                                {0x03800A01, 0x00000000,
                                                 0xFFFF0003};
const unsigned int UFSHostDevice::UFSSCSIDevice::cachingPage[5] =
                                                {0x00011208, 0x00000000,
                                                 0x00000000, 0x00000020,
                                                 0x00000000};

UFSHostDevice::UFSSCSIDevice::~UFSSCSIDevice() {}

/**
 * UFS specific SCSI handling function.
 * The following attributes may still be added: SCSI format unit,
 * Send diagnostic and UNMAP;
 * Synchronize Cache and buffer read/write could not be tested yet
 * All parameters can be found in:
 * Universal Flash Storage (UFS) JESD220 FEB 2011 (JEDEC)
 * http://www.jedec.org/standards-documents/results/jesd220
 * (a JEDEC acount may be required {free of charge})
 */

struct UFSHostDevice::SCSIReply
UFSHostDevice::UFSSCSIDevice::SCSICMDHandle(uint32_t* SCSI_msg)
{
    struct SCSIReply scsi_out;
    memset(&scsi_out, 0, sizeof(struct SCSIReply));

    /**
     * Create the standard SCSI reponse information
     * These values might changes over the course of a transfer
     */
    scsi_out.message.header.dWord0 = UPIUHeaderDataIndWord0 |
        lunID << 16;
    scsi_out.message.header.dWord1 = UPIUHeaderDataIndWord1;
    scsi_out.message.header.dWord2 = UPIUHeaderDataIndWord2;
    statusCheck(SCSIGood, scsi_out.senseCode);
    scsi_out.senseSize = scsi_out.senseCode[0];
    scsi_out.LUN = lunID;
    scsi_out.status = SCSIGood;

    DPRINTF(UFSHostDevice, "SCSI command:%2x\n", SCSI_msg[4]);
    /**Determine what the message is and fill the response packet*/

    switch (SCSI_msg[4] & 0xFF) {

      case SCSIInquiry: {
          /**
           * SCSI inquiry: tell about this specific logic unit
           */
          scsi_out.msgSize = 36;
          scsi_out.message.dataMsg.resize(9);

          for (uint8_t count = 0; count < 9; count++)
              scsi_out.message.dataMsg[count] =
                  (reinterpret_cast<uint32_t*> (&lunInfo))[count];
      } break;

      case SCSIRead6: {
          /**
           * Read command. Number indicates the length of the command.
           */
          scsi_out.expectMore = 0x02;
          scsi_out.msgSize = 0;

          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned. Apart from that it only has
           * information in five bits of the first byte that is relevant
           * for this field.
           */
          uint32_t tmp = *reinterpret_cast<uint32_t*>(tempptr);
          uint64_t read_offset = betoh(tmp) & 0x1FFFFF;

          uint32_t read_size = tempptr[4];


          scsi_out.msgSize =  read_size * blkSize;
          scsi_out.offset = read_offset * blkSize;

          if ((read_offset + read_size) > diskSize)
              scsi_out.status = SCSIIllegalRequest;

          DPRINTF(UFSHostDevice, "Read6 offset: 0x%8x, for %d blocks\n",
                  read_offset, read_size);

          /**
           * Renew status check, for the request may have been illegal
           */
          statusCheck(scsi_out.status, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;

      } break;

      case SCSIRead10: {
          scsi_out.expectMore = 0x02;
          scsi_out.msgSize = 0;

          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned.*/
          uint32_t tmp = *reinterpret_cast<uint32_t*>(&tempptr[2]);
          uint64_t read_offset = betoh(tmp);

          uint16_t tmpsize = *reinterpret_cast<uint16_t*>(&tempptr[7]);
          uint32_t read_size = betoh(tmpsize);

          scsi_out.msgSize =  read_size * blkSize;
          scsi_out.offset = read_offset * blkSize;

          if ((read_offset + read_size) > diskSize)
              scsi_out.status = SCSIIllegalRequest;

          DPRINTF(UFSHostDevice, "Read10 offset: 0x%8x, for %d blocks\n",
                  read_offset, read_size);

          /**
           * Renew status check, for the request may have been illegal
           */
          statusCheck(scsi_out.status, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;

      } break;

      case SCSIRead16: {
          scsi_out.expectMore = 0x02;
          scsi_out.msgSize = 0;

          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned.*/
          uint32_t tmp = *reinterpret_cast<uint32_t*>(&tempptr[2]);
          uint64_t read_offset = betoh(tmp);

          tmp = *reinterpret_cast<uint32_t*>(&tempptr[6]);
          read_offset = (read_offset << 32) | betoh(tmp);

          tmp = *reinterpret_cast<uint32_t*>(&tempptr[10]);
          uint32_t read_size = betoh(tmp);

          scsi_out.msgSize =  read_size * blkSize;
          scsi_out.offset = read_offset * blkSize;

          if ((read_offset + read_size) > diskSize)
              scsi_out.status = SCSIIllegalRequest;

          DPRINTF(UFSHostDevice, "Read16 offset: 0x%8x, for %d blocks\n",
                  read_offset, read_size);

          /**
           * Renew status check, for the request may have been illegal
           */
          statusCheck(scsi_out.status, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;

      } break;

      case SCSIReadCapacity10: {
          /**
           * read the capacity of the device
           */
          scsi_out.msgSize = 8;
          scsi_out.message.dataMsg.resize(2);
          scsi_out.message.dataMsg[0] =
              betoh(capacityLower);//last block
          scsi_out.message.dataMsg[1] = betoh(blkSize);//blocksize

      } break;
      case SCSIReadCapacity16: {
          scsi_out.msgSize = 32;
          scsi_out.message.dataMsg.resize(8);
          scsi_out.message.dataMsg[0] =
              betoh(capacityUpper);//last block
          scsi_out.message.dataMsg[1] =
              betoh(capacityLower);//last block
          scsi_out.message.dataMsg[2] = betoh(blkSize);//blocksize
          scsi_out.message.dataMsg[3] = 0x00;//
          scsi_out.message.dataMsg[4] = 0x00;//reserved
          scsi_out.message.dataMsg[5] = 0x00;//reserved
          scsi_out.message.dataMsg[6] = 0x00;//reserved
          scsi_out.message.dataMsg[7] = 0x00;//reserved

      } break;

      case SCSIReportLUNs: {
          /**
           * Find out how many Logic Units this device has.
           */
          scsi_out.msgSize = (lunAvail * 8) + 8;//list + overhead
          scsi_out.message.dataMsg.resize(2 * lunAvail + 2);
          scsi_out.message.dataMsg[0] = (lunAvail * 8) << 24;//LUN listlength
          scsi_out.message.dataMsg[1] = 0x00;

          for (uint8_t count = 0; count < lunAvail; count++) {
              //LUN "count"
              scsi_out.message.dataMsg[2 + 2 * count] = (count & 0x7F) << 8;
              scsi_out.message.dataMsg[3 + 2 * count] = 0x00;
          }

      } break;

      case SCSIStartStop: {
          //Just acknowledge; not deemed relevant ATM
          scsi_out.msgSize = 0;

      } break;

      case SCSITestUnitReady: {
          //Just acknowledge; not deemed relevant ATM
          scsi_out.msgSize = 0;

      } break;

      case SCSIVerify10: {
          /**
           * See if the blocks that the host plans to request are in range of
           * the device.
           */
          scsi_out.msgSize = 0;

          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned.*/
          uint32_t tmp = *reinterpret_cast<uint32_t*>(&tempptr[2]);
          uint64_t read_offset = betoh(tmp);

          uint16_t tmpsize = *reinterpret_cast<uint16_t*>(&tempptr[7]);
          uint32_t read_size = betoh(tmpsize);

          if ((read_offset + read_size) > diskSize)
              scsi_out.status = SCSIIllegalRequest;

          /**
           * Renew status check, for the request may have been illegal
           */
          statusCheck(scsi_out.status, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;

      } break;

      case SCSIWrite6: {
          /**
           * Write command.
           */

          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned. Apart from that it only has
           * information in five bits of the first byte that is relevant
           * for this field.
           */
          uint32_t tmp = *reinterpret_cast<uint32_t*>(tempptr);
          uint64_t write_offset = betoh(tmp) & 0x1FFFFF;

          uint32_t write_size = tempptr[4];

          scsi_out.msgSize =  write_size * blkSize;
          scsi_out.offset = write_offset * blkSize;
          scsi_out.expectMore = 0x01;

          if ((write_offset + write_size) > diskSize)
              scsi_out.status = SCSIIllegalRequest;

          DPRINTF(UFSHostDevice, "Write6 offset: 0x%8x, for %d blocks\n",
                  write_offset, write_size);

          /**
           * Renew status check, for the request may have been illegal
           */
          statusCheck(scsi_out.status, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;

      } break;

      case SCSIWrite10: {
          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned.*/
          uint32_t tmp = *reinterpret_cast<uint32_t*>(&tempptr[2]);
          uint64_t write_offset = betoh(tmp);

          uint16_t tmpsize = *reinterpret_cast<uint16_t*>(&tempptr[7]);
          uint32_t write_size = betoh(tmpsize);

          scsi_out.msgSize =  write_size * blkSize;
          scsi_out.offset = write_offset * blkSize;
          scsi_out.expectMore = 0x01;

          if ((write_offset + write_size) > diskSize)
              scsi_out.status = SCSIIllegalRequest;

          DPRINTF(UFSHostDevice, "Write10 offset: 0x%8x, for %d blocks\n",
                  write_offset, write_size);

          /**
           * Renew status check, for the request may have been illegal
           */
          statusCheck(scsi_out.status, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;

      } break;

      case SCSIWrite16: {
          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned.*/
          uint32_t tmp = *reinterpret_cast<uint32_t*>(&tempptr[2]);
          uint64_t write_offset = betoh(tmp);

          tmp = *reinterpret_cast<uint32_t*>(&tempptr[6]);
          write_offset = (write_offset << 32) | betoh(tmp);

          tmp = *reinterpret_cast<uint32_t*>(&tempptr[10]);
          uint32_t write_size = betoh(tmp);

          scsi_out.msgSize = write_size * blkSize;
          scsi_out.offset = write_offset * blkSize;
          scsi_out.expectMore = 0x01;

          if ((write_offset + write_size) > diskSize)
              scsi_out.status = SCSIIllegalRequest;

          DPRINTF(UFSHostDevice, "Write16 offset: 0x%8x, for %d blocks\n",
                  write_offset, write_size);

          /**
           * Renew status check, for the request may have been illegal
           */
          statusCheck(scsi_out.status, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;

      } break;

      case SCSIFormatUnit: {//not yet verified
          scsi_out.msgSize = 0;
          scsi_out.expectMore = 0x01;

      } break;

      case SCSISendDiagnostic: {//not yet verified
          scsi_out.msgSize = 0;

      } break;

      case SCSISynchronizeCache: {
          //do we have cache (we don't have cache at this moment)
          //TODO: here will synchronization happen when cache is modelled
          scsi_out.msgSize = 0;

      } break;

        //UFS SCSI additional command set for full functionality
      case SCSIModeSelect10:
        //TODO:
        //scsi_out.expectMore = 0x01;//not supported due to modepage support
        //code isn't dead, code suggest what is to be done when implemented
        break;

      case SCSIModeSense6: case SCSIModeSense10: {
          /**
           * Get more discriptive information about the SCSI functionality
           * within this logic unit.
           */
          if ((SCSI_msg[4] & 0x3F0000) >> 16 == 0x0A) {//control page
              scsi_out.message.dataMsg.resize((sizeof(controlPage) >> 2) + 2);
              scsi_out.message.dataMsg[0] = 0x00000A00;//control page code
              scsi_out.message.dataMsg[1] = 0x00000000;//See JEDEC220 ch8

              for (uint8_t count = 0; count < 3; count++)
                  scsi_out.message.dataMsg[2 + count] = controlPage[count];

              scsi_out.msgSize = 20;
              DPRINTF(UFSHostDevice, "CONTROL page\n");

          } else if ((SCSI_msg[4] & 0x3F0000) >> 16 == 0x01) {//recovery page
              scsi_out.message.dataMsg.resize((sizeof(recoveryPage) >> 2)
                                              + 2);

              scsi_out.message.dataMsg[0] = 0x00000100;//recovery page code
              scsi_out.message.dataMsg[1] = 0x00000000;//See JEDEC220 ch8

              for (uint8_t count = 0; count < 3; count++)
                  scsi_out.message.dataMsg[2 + count] = recoveryPage[count];

              scsi_out.msgSize = 20;
              DPRINTF(UFSHostDevice, "RECOVERY page\n");

          } else if ((SCSI_msg[4] & 0x3F0000) >> 16 == 0x08) {//caching page

              scsi_out.message.dataMsg.resize((sizeof(cachingPage) >> 2) + 2);
              scsi_out.message.dataMsg[0] = 0x00001200;//caching page code
              scsi_out.message.dataMsg[1] = 0x00000000;//See JEDEC220 ch8

              for (uint8_t count = 0; count < 5; count++)
                  scsi_out.message.dataMsg[2 + count] = cachingPage[count];

              scsi_out.msgSize = 20;
              DPRINTF(UFSHostDevice, "CACHE page\n");

          } else if ((SCSI_msg[4] & 0x3F0000) >> 16 == 0x3F) {//ALL the pages!

              scsi_out.message.dataMsg.resize(((sizeof(controlPage) +
                                                sizeof(recoveryPage) +
                                                sizeof(cachingPage)) >> 2)
                                              + 2);
              scsi_out.message.dataMsg[0] = 0x00003200;//all page code
              scsi_out.message.dataMsg[1] = 0x00000000;//See JEDEC220 ch8

              for (uint8_t count = 0; count < 3; count++)
                  scsi_out.message.dataMsg[2 + count] = recoveryPage[count];

              for (uint8_t count = 0; count < 5; count++)
                  scsi_out.message.dataMsg[5 + count] = cachingPage[count];

              for (uint8_t count = 0; count < 3; count++)
                  scsi_out.message.dataMsg[10 + count] = controlPage[count];

              scsi_out.msgSize = 52;
              DPRINTF(UFSHostDevice, "Return ALL the pages!!!\n");

          } else inform("Wrong mode page requested\n");

          scsi_out.message.dataCount = scsi_out.msgSize << 24;
      } break;

      case SCSIRequestSense: {
          scsi_out.msgSize = 0;

      } break;

      case SCSIUnmap:break;//not yet verified

      case SCSIWriteBuffer: {
          scsi_out.expectMore = 0x01;

          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned.*/
          uint32_t tmp = *reinterpret_cast<uint32_t*>(&tempptr[2]);
          uint64_t write_offset = betoh(tmp) & 0xFFFFFF;

          tmp = *reinterpret_cast<uint32_t*>(&tempptr[5]);
          uint32_t write_size = betoh(tmp) & 0xFFFFFF;

          scsi_out.msgSize =  write_size;
          scsi_out.offset = write_offset;

      } break;

      case SCSIReadBuffer: {
          /**
           * less trivial than normal read. Size is in bytes instead
           * of blocks, and it is assumed (though not guaranteed) that
           * reading is from cache.
           */
          scsi_out.expectMore = 0x02;

          uint8_t* tempptr = reinterpret_cast<uint8_t*>(&SCSI_msg[4]);

          /**BE and not nicely aligned.*/
          uint32_t tmp = *reinterpret_cast<uint32_t*>(&tempptr[2]);
          uint64_t read_offset = betoh(tmp) & 0xFFFFFF;

          tmp = *reinterpret_cast<uint32_t*>(&tempptr[5]);
          uint32_t read_size = betoh(tmp) & 0xFFFFFF;

          scsi_out.msgSize = read_size;
          scsi_out.offset = read_offset;

          if ((read_offset + read_size) > capacityLower * blkSize)
              scsi_out.status = SCSIIllegalRequest;

          DPRINTF(UFSHostDevice, "Read buffer location: 0x%8x\n",
                  read_offset);
          DPRINTF(UFSHostDevice, "Number of bytes: 0x%8x\n", read_size);

          statusCheck(scsi_out.status, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;

      } break;

      case SCSIMaintenanceIn: {
          /**
           * linux sends this command three times from kernel 3.9 onwards,
           * UFS does not support it, nor does this model. Linux knows this,
           * but tries anyway (useful for some SD card types).
           * Lets make clear we don't want it and just ignore it.
           */
          DPRINTF(UFSHostDevice, "Ignoring Maintenance In command\n");
          statusCheck(SCSIIllegalRequest, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;
          scsi_out.msgSize = 0;
      } break;

      default: {
          statusCheck(SCSIIllegalRequest, scsi_out.senseCode);
          scsi_out.senseSize = scsi_out.senseCode[0];
          scsi_out.status = (scsi_out.status == SCSIGood) ? SCSIGood :
              SCSICheckCondition;
          scsi_out.msgSize = 0;
          inform("Unsupported scsi message type: %2x\n", SCSI_msg[4] & 0xFF);
          inform("0x%8x\n", SCSI_msg[0]);
          inform("0x%8x\n", SCSI_msg[1]);
          inform("0x%8x\n", SCSI_msg[2]);
          inform("0x%8x\n", SCSI_msg[3]);
          inform("0x%8x\n", SCSI_msg[4]);
      } break;
    }

    return scsi_out;
}

/**
 * SCSI status check function. generic device test, creates sense codes
 * Future versions may include TODO: device checks, which is why this is
 * in a separate function.
 */

void
UFSHostDevice::UFSSCSIDevice::statusCheck(uint8_t status,
                                uint8_t* sensecodelist)
{
    for (uint8_t count = 0; count < 19; count++)
        sensecodelist[count] = 0;

    sensecodelist[0] = 18; //sense length
    sensecodelist[1] = 0x70; //we send a valid frame
    sensecodelist[3] = status & 0xF; //mask to be sure + sensecode
    sensecodelist[8] = 0x1F; //data length
}

/**
 * read from the flashdisk
 */

void
UFSHostDevice::UFSSCSIDevice::readFlash(uint8_t* readaddr, uint64_t offset,
                                   uint32_t size)
{
    /** read from image, and get to memory */
    for (int count = 0; count < (size / SectorSize); count++)
        flashDisk->read(&(readaddr[SectorSize*count]), (offset /
                                                        SectorSize) + count);
}

/**
 * Write to the flashdisk
 */

void
UFSHostDevice::UFSSCSIDevice::writeFlash(uint8_t* writeaddr, uint64_t offset,
                                    uint32_t size)
{
    /** Get from fifo and write to image*/
    for (int count = 0; count < (size / SectorSize); count++)
        flashDisk->write(&(writeaddr[SectorSize * count]),
                         (offset / SectorSize) + count);
}

/**
 * Constructor for the UFS Host device
 */

UFSHostDevice::UFSHostDevice(const UFSHostDeviceParams* p) :
    DmaDevice(p),
    pioAddr(p->pio_addr),
    pioSize(0x0FFF),
    pioDelay(p->pio_latency),
    intNum(p->int_num),
    gic(p->gic),
    lunAvail(p->image.size()),
    UFSSlots(p->ufs_slots - 1),
    readPendingNum(0),
    writePendingNum(0),
    activeDoorbells(0),
    pendingDoorbells(0),
    countInt(0),
    transferTrack(0),
    taskCommandTrack(0),
    idlePhaseStart(0),
    SCSIResumeEvent(this),
    UTPEvent(this)
{
    DPRINTF(UFSHostDevice, "The hostcontroller hosts %d Logic units\n",
            lunAvail);
    UFSDevice.resize(lunAvail);

    transferDoneCallback = new MakeCallback<UFSHostDevice,
        &UFSHostDevice::LUNSignal>(this);
    memReadCallback = new MakeCallback<UFSHostDevice,
        &UFSHostDevice::readCallback>(this);

    for (int count = 0; count < lunAvail; count++) {
        UFSDevice[count] = new UFSSCSIDevice(p, count, transferDoneCallback,
                                             memReadCallback);
    }

    if (UFSSlots > 31)
        warn("UFSSlots = %d, this will results in %d command slots",
             UFSSlots, (UFSSlots & 0x1F));

    if ((UFSSlots & 0x1F) == 0)
        fatal("Number of UFS command slots should be between 1 and 32.");

    setValues();
}

/**
 * Create the parameters of this device
 */

UFSHostDevice*
UFSHostDeviceParams::create()
{
    return new UFSHostDevice(this);
}


void
UFSHostDevice::regStats()
{
    using namespace Stats;

    std::string UFSHost_name = name() + ".UFSDiskHost";

    // Register the stats
    /** Queue lengths */
    stats.currentSCSIQueue
        .name(UFSHost_name + ".currentSCSIQueue")
        .desc("Most up to date length of the command queue")
        .flags(none);
    stats.currentReadSSDQueue
        .name(UFSHost_name + ".currentReadSSDQueue")
        .desc("Most up to date length of the read SSD queue")
        .flags(none);
    stats.currentWriteSSDQueue
        .name(UFSHost_name + ".currentWriteSSDQueue")
        .desc("Most up to date length of the write SSD queue")
        .flags(none);

    /** Amount of data read/written */
    stats.totalReadSSD
        .name(UFSHost_name + ".totalReadSSD")
        .desc("Number of bytes read from SSD")
        .flags(none);

    stats.totalWrittenSSD
        .name(UFSHost_name + ".totalWrittenSSD")
        .desc("Number of bytes written to SSD")
        .flags(none);

    stats.totalReadDiskTransactions
        .name(UFSHost_name + ".totalReadDiskTransactions")
        .desc("Number of transactions from disk")
        .flags(none);
    stats.totalWriteDiskTransactions
        .name(UFSHost_name + ".totalWriteDiskTransactions")
        .desc("Number of transactions to disk")
        .flags(none);
    stats.totalReadUFSTransactions
        .name(UFSHost_name + ".totalReadUFSTransactions")
        .desc("Number of transactions from device")
        .flags(none);
    stats.totalWriteUFSTransactions
        .name(UFSHost_name + ".totalWriteUFSTransactions")
        .desc("Number of transactions to device")
        .flags(none);

    /** Average bandwidth for reads and writes */
    stats.averageReadSSDBW
        .name(UFSHost_name + ".averageReadSSDBandwidth")
        .desc("Average read bandwidth (bytes/s)")
        .flags(nozero);

    stats.averageReadSSDBW = stats.totalReadSSD / simSeconds;

    stats.averageWriteSSDBW
        .name(UFSHost_name + ".averageWriteSSDBandwidth")
        .desc("Average write bandwidth (bytes/s)")
        .flags(nozero);

    stats.averageWriteSSDBW = stats.totalWrittenSSD / simSeconds;

    stats.averageSCSIQueue
        .name(UFSHost_name + ".averageSCSIQueueLength")
        .desc("Average command queue length")
        .flags(nozero);
    stats.averageReadSSDQueue
        .name(UFSHost_name + ".averageReadSSDQueueLength")
        .desc("Average read queue length")
        .flags(nozero);
    stats.averageWriteSSDQueue
        .name(UFSHost_name + ".averageWriteSSDQueueLength")
        .desc("Average write queue length")
        .flags(nozero);

    /** Number of doorbells rung*/
    stats.curDoorbell
        .name(UFSHost_name + ".curDoorbell")
        .desc("Most up to date number of doorbells used")
        .flags(none);

    stats.curDoorbell = activeDoorbells;

    stats.maxDoorbell
        .name(UFSHost_name + ".maxDoorbell")
        .desc("Maximum number of doorbells utilized")
        .flags(none);
    stats.averageDoorbell
        .name(UFSHost_name + ".averageDoorbell")
        .desc("Average number of Doorbells used")
        .flags(nozero);

    /** Latency*/
    stats.transactionLatency
        .init(100)
        .name(UFSHost_name + ".transactionLatency")
        .desc("Histogram of transaction times")
        .flags(pdf);

    stats.idleTimes
        .init(100)
        .name(UFSHost_name + ".idlePeriods")
        .desc("Histogram of idle times")
        .flags(pdf);

}

/**
 * Register init
 */
void UFSHostDevice::setValues()
{
    /**
     * The capability register is built up as follows:
     * 31-29 RES; Testmode support; O3 delivery; 64 bit addr;
     * 23-19 RES; 18-16 #TM Req slots; 15-5 RES;4-0 # TR slots
     */
    UFSHCIMem.HCCAP = 0x06070000 | (UFSSlots & 0x1F);
    UFSHCIMem.HCversion = 0x00010000; //version is 1.0
    UFSHCIMem.HCHCDDID = 0xAA003C3C;// Arbitrary number
    UFSHCIMem.HCHCPMID = 0x41524D48; //ARMH (not an official MIPI number)
    UFSHCIMem.TRUTRLDBR = 0x00;
    UFSHCIMem.TMUTMRLDBR = 0x00;
    UFSHCIMem.CMDUICCMDR = 0x00;
    // We can process CMD, TM, TR, device present
    UFSHCIMem.ORHostControllerStatus = 0x08;
    UFSHCIMem.TRUTRLBA = 0x00;
    UFSHCIMem.TRUTRLBAU = 0x00;
    UFSHCIMem.TMUTMRLBA = 0x00;
    UFSHCIMem.TMUTMRLBAU = 0x00;
}

/**
 * Determine address ranges
 */

AddrRangeList
UFSHostDevice::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

/**
 * UFSHCD read register. This function allows the system to read the
 * register entries
 */

Tick
UFSHostDevice::read(PacketPtr pkt)
{
    uint32_t data = 0;

    switch (pkt->getAddr() & 0xFF)
    {

      case regControllerCapabilities:
        data = UFSHCIMem.HCCAP;
        break;

      case regUFSVersion:
        data = UFSHCIMem.HCversion;
        break;

      case regControllerDEVID:
        data = UFSHCIMem.HCHCDDID;
        break;

      case regControllerPRODID:
        data = UFSHCIMem.HCHCPMID;
        break;

      case regInterruptStatus:
        data = UFSHCIMem.ORInterruptStatus;
        UFSHCIMem.ORInterruptStatus = 0x00;
        //TODO: Revise and extend
        clearInterrupt();
        break;

      case regInterruptEnable:
        data = UFSHCIMem.ORInterruptEnable;
        break;

      case regControllerStatus:
        data = UFSHCIMem.ORHostControllerStatus;
        break;

      case regControllerEnable:
        data = UFSHCIMem.ORHostControllerEnable;
        break;

      case regUICErrorCodePHYAdapterLayer:
        data = UFSHCIMem.ORUECPA;
        break;

      case regUICErrorCodeDataLinkLayer:
        data = UFSHCIMem.ORUECDL;
        break;

      case regUICErrorCodeNetworkLayer:
        data = UFSHCIMem.ORUECN;
        break;

      case regUICErrorCodeTransportLayer:
        data = UFSHCIMem.ORUECT;
        break;

      case regUICErrorCodeDME:
        data = UFSHCIMem.ORUECDME;
        break;

      case regUTPTransferREQINTAGGControl:
        data = UFSHCIMem.ORUTRIACR;
        break;

      case regUTPTransferREQListBaseL:
        data = UFSHCIMem.TRUTRLBA;
        break;

      case regUTPTransferREQListBaseH:
        data = UFSHCIMem.TRUTRLBAU;
        break;

      case regUTPTransferREQDoorbell:
        data = UFSHCIMem.TRUTRLDBR;
        break;

      case regUTPTransferREQListClear:
        data = UFSHCIMem.TRUTRLCLR;
        break;

      case regUTPTransferREQListRunStop:
        data = UFSHCIMem.TRUTRLRSR;
        break;

      case regUTPTaskREQListBaseL:
        data = UFSHCIMem.TMUTMRLBA;
        break;

      case regUTPTaskREQListBaseH:
        data = UFSHCIMem.TMUTMRLBAU;
        break;

      case regUTPTaskREQDoorbell:
        data = UFSHCIMem.TMUTMRLDBR;
        break;

      case regUTPTaskREQListClear:
        data = UFSHCIMem.TMUTMRLCLR;
        break;

      case regUTPTaskREQListRunStop:
        data = UFSHCIMem.TMUTMRLRSR;
        break;

      case regUICCommand:
        data = UFSHCIMem.CMDUICCMDR;
        break;

      case regUICCommandArg1:
        data = UFSHCIMem.CMDUCMDARG1;
        break;

      case regUICCommandArg2:
        data = UFSHCIMem.CMDUCMDARG2;
        break;

      case regUICCommandArg3:
        data = UFSHCIMem.CMDUCMDARG3;
        break;

      default:
        data = 0x00;
        break;
    }

    pkt->set<uint32_t>(data);
    pkt->makeResponse();
    return pioDelay;
}

/**
 * UFSHCD write function. This function allows access to the writeable
 * registers. If any function attempts to write value to an unwriteable
 * register entry, then the value will not be written.
 */
Tick
UFSHostDevice::write(PacketPtr pkt)
{
    uint32_t data = 0;

    switch (pkt->getSize()) {

      case 1:
        data = pkt->get<uint8_t>();
        break;

      case 2:
        data = pkt->get<uint16_t>();
        break;

      case 4:
        data = pkt->get<uint32_t>();
        break;

      default:
        panic("Undefined UFSHCD controller write size!\n");
        break;
    }

    switch (pkt->getAddr() & 0xFF)
    {
      case regControllerCapabilities://you shall not write to this
        break;

      case regUFSVersion://you shall not write to this
        break;

      case regControllerDEVID://you shall not write to this
        break;

      case regControllerPRODID://you shall not write to this
        break;

      case regInterruptStatus://you shall not write to this
        break;

      case regInterruptEnable:
        UFSHCIMem.ORInterruptEnable = data;
        break;

      case regControllerStatus:
        UFSHCIMem.ORHostControllerStatus = data;
        break;

      case regControllerEnable:
        UFSHCIMem.ORHostControllerEnable = data;
        break;

      case regUICErrorCodePHYAdapterLayer:
        UFSHCIMem.ORUECPA = data;
        break;

      case regUICErrorCodeDataLinkLayer:
        UFSHCIMem.ORUECDL = data;
        break;

      case regUICErrorCodeNetworkLayer:
        UFSHCIMem.ORUECN = data;
        break;

      case regUICErrorCodeTransportLayer:
        UFSHCIMem.ORUECT = data;
        break;

      case regUICErrorCodeDME:
        UFSHCIMem.ORUECDME = data;
        break;

      case regUTPTransferREQINTAGGControl:
        UFSHCIMem.ORUTRIACR = data;
        break;

      case regUTPTransferREQListBaseL:
        UFSHCIMem.TRUTRLBA = data;
        if (((UFSHCIMem.TRUTRLBA | UFSHCIMem.TRUTRLBAU) != 0x00) &&
            ((UFSHCIMem.TMUTMRLBA | UFSHCIMem.TMUTMRLBAU)!= 0x00))
            UFSHCIMem.ORHostControllerStatus |= UICCommandReady;
        break;

      case regUTPTransferREQListBaseH:
        UFSHCIMem.TRUTRLBAU = data;
        if (((UFSHCIMem.TRUTRLBA | UFSHCIMem.TRUTRLBAU) != 0x00) &&
            ((UFSHCIMem.TMUTMRLBA | UFSHCIMem.TMUTMRLBAU) != 0x00))
            UFSHCIMem.ORHostControllerStatus |= UICCommandReady;
        break;

      case regUTPTransferREQDoorbell:
        if (!(UFSHCIMem.TRUTRLDBR) && data)
            stats.idleTimes.sample(curTick() - idlePhaseStart);
        UFSHCIMem.TRUTRLDBR |= data;
        requestHandler();
        break;

      case regUTPTransferREQListClear:
        UFSHCIMem.TRUTRLCLR = data;
        break;

      case regUTPTransferREQListRunStop:
        UFSHCIMem.TRUTRLRSR = data;
        break;

      case regUTPTaskREQListBaseL:
        UFSHCIMem.TMUTMRLBA = data;
        if (((UFSHCIMem.TRUTRLBA | UFSHCIMem.TRUTRLBAU) != 0x00) &&
            ((UFSHCIMem.TMUTMRLBA | UFSHCIMem.TMUTMRLBAU) != 0x00))
            UFSHCIMem.ORHostControllerStatus |= UICCommandReady;
        break;

      case regUTPTaskREQListBaseH:
        UFSHCIMem.TMUTMRLBAU = data;
        if (((UFSHCIMem.TRUTRLBA | UFSHCIMem.TRUTRLBAU) != 0x00) &&
            ((UFSHCIMem.TMUTMRLBA | UFSHCIMem.TMUTMRLBAU) != 0x00))
            UFSHCIMem.ORHostControllerStatus |= UICCommandReady;
        break;

      case regUTPTaskREQDoorbell:
        UFSHCIMem.TMUTMRLDBR |= data;
        requestHandler();
        break;

      case regUTPTaskREQListClear:
        UFSHCIMem.TMUTMRLCLR = data;
        break;

      case regUTPTaskREQListRunStop:
        UFSHCIMem.TMUTMRLRSR = data;
        break;

      case regUICCommand:
        UFSHCIMem.CMDUICCMDR = data;
        requestHandler();
        break;

      case regUICCommandArg1:
        UFSHCIMem.CMDUCMDARG1 = data;
        break;

      case regUICCommandArg2:
        UFSHCIMem.CMDUCMDARG2 = data;
        break;

      case regUICCommandArg3:
        UFSHCIMem.CMDUCMDARG3 = data;
        break;

      default:break;//nothing happens, you try to access a register that
                    //does not exist

    }

    pkt->makeResponse();
    return pioDelay;
}

/**
 * Request handler. Determines where the request comes from and initiates the
 * appropriate actions accordingly.
 */

void
UFSHostDevice::requestHandler()
{
    Addr address = 0x00;
    int mask = 0x01;
    int size;
    int count = 0;
    struct taskStart task_info;
    struct transferStart transferstart_info;
    transferstart_info.done = 0;

    /**
     * step1 determine what called us
     * step2 determine where to get it
     * Look for any request of which we where not yet aware
     */
    while (((UFSHCIMem.CMDUICCMDR > 0x00) |
            ((UFSHCIMem.TMUTMRLDBR ^ taskCommandTrack) > 0x00) |
            ((UFSHCIMem.TRUTRLDBR ^ transferTrack) > 0x00)) ) {

        if (UFSHCIMem.CMDUICCMDR > 0x00) {
            /**
             * Command; general control of the Host controller.
             * no DMA transfer needed
             */
            commandHandler();
            UFSHCIMem.ORInterruptStatus |= UICCommandCOMPL;
            generateInterrupt();
            UFSHCIMem.CMDUICCMDR = 0x00;
            return; //command, nothing more we can do

        } else if ((UFSHCIMem.TMUTMRLDBR ^ taskCommandTrack) > 0x00) {
            /**
             * Task; flow control, meant for the device/Logic unit
             * DMA transfer is needed, flash will not be approached
             */
            size = sizeof(UTPUPIUTaskReq);
            /**Find the position that is not handled yet*/
            count = findLsbSet((UFSHCIMem.TMUTMRLDBR ^ taskCommandTrack));
            address = UFSHCIMem.TMUTMRLBAU;
            //<-64 bit
            address = (count * size) + (address << 32) +
                UFSHCIMem.TMUTMRLBA;
            taskCommandTrack |= mask << count;

            inform("UFSmodel received a task from the system; this might"
                   " lead to untested behaviour.\n");

            task_info.mask = mask << count;
            task_info.address = address;
            task_info.size = size;
            task_info.done = UFSHCIMem.TMUTMRLDBR;
            taskInfo.push_back(task_info);
            taskEventQueue.push_back(this);
            writeDevice(&taskEventQueue.back(), false, address, size,
                        reinterpret_cast<uint8_t*>
                        (&taskInfo.back().destination), 0, 0);

        } else if ((UFSHCIMem.TRUTRLDBR ^ transferTrack) > 0x00) {
            /**
             * Transfer; Data transfer from or to the disk. There will be DMA
             * transfers, and the flash might be approached. Further
             * commands, are needed to specify the exact command.
             */
            size = sizeof(UTPTransferReqDesc);
            /**Find the position that is not handled yet*/
            count = findLsbSet((UFSHCIMem.TRUTRLDBR ^ transferTrack));
            address = UFSHCIMem.TRUTRLBAU;
            //<-64 bit
            address = (count * size) + (address << 32) + UFSHCIMem.TRUTRLBA;

            transferTrack |= mask << count;
            DPRINTF(UFSHostDevice, "Doorbell register: 0x%8x select #:"
                    " 0x%8x completion info: 0x%8x\n", UFSHCIMem.TRUTRLDBR,
                    count, transferstart_info.done);

            transferstart_info.done = UFSHCIMem.TRUTRLDBR;

            /**stats**/
            transactionStart[count] = curTick(); //note the start time
            ++activeDoorbells;
            stats.maxDoorbell = (stats.maxDoorbell.value() < activeDoorbells)
                ? activeDoorbells : stats.maxDoorbell.value();
            stats.averageDoorbell = stats.maxDoorbell.value();

            /**
             * step3 start transfer
             * step4 register information; allowing the host to respond in
             * the end
             */
            transferstart_info.mask = mask << count;
            transferstart_info.address = address;
            transferstart_info.size = size;
            transferstart_info.done = UFSHCIMem.TRUTRLDBR;
            transferStartInfo.push_back(transferstart_info);

            /**Deleted in readDone, queued in finalUTP*/
            transferStartInfo.back().destination = new struct
                UTPTransferReqDesc;
            DPRINTF(UFSHostDevice, "Initial transfer start: 0x%8x\n",
                    transferstart_info.done);
            transferEventQueue.push_back(this);

            if (transferEventQueue.size() < 2) {
                writeDevice(&transferEventQueue.front(), false,
                            address, size, reinterpret_cast<uint8_t*>
                            (transferStartInfo.front().destination),0, 0);
                DPRINTF(UFSHostDevice, "Transfer scheduled\n");
            }
        }
    }
}

/**
 * Task start event
 */

void
UFSHostDevice::taskStart()
{
    DPRINTF(UFSHostDevice, "Task start");
    taskHandler(&taskInfo.front().destination, taskInfo.front().mask,
                taskInfo.front().address, taskInfo.front().size);
    taskInfo.pop_front();
    taskEventQueue.pop_front();
}

/**
 * Transfer start event
 */

void
UFSHostDevice::transferStart()
{
    DPRINTF(UFSHostDevice, "Enter transfer event\n");
    transferHandler(transferStartInfo.front().destination,
                    transferStartInfo.front().mask,
                    transferStartInfo.front().address,
                    transferStartInfo.front().size,
                    transferStartInfo.front().done);

    transferStartInfo.pop_front();
    DPRINTF(UFSHostDevice, "Transfer queue size at end of event: "
            "0x%8x\n", transferEventQueue.size());
}

/**
 * Handles the commands that are given. At this point in time, not many
 * commands have been implemented in the driver.
 */

void
UFSHostDevice::commandHandler()
{
    if (UFSHCIMem.CMDUICCMDR == 0x16) {
        UFSHCIMem.ORHostControllerStatus |= 0x0F;//link startup
    }

}

/**
 * Handles the tasks that are given. At this point in time, not many tasks
 * have been implemented in the driver.
 */

void
UFSHostDevice::taskHandler(struct UTPUPIUTaskReq* request_in,
                           uint32_t req_pos, Addr finaladdress, uint32_t
                           finalsize)
{
    /**
     * For now, just unpack and acknowledge the task without doing anything.
     * TODO Implement UFS tasks.
     */
    inform("taskHandler\n");
    inform("%8x\n", request_in->header.dWord0);
    inform("%8x\n", request_in->header.dWord1);
    inform("%8x\n", request_in->header.dWord2);

    request_in->header.dWord2 &= 0xffffff00;

    UFSHCIMem.TMUTMRLDBR &= ~(req_pos);
    taskCommandTrack &= ~(req_pos);
    UFSHCIMem.ORInterruptStatus |= UTPTaskREQCOMPL;

    readDevice(true, finaladdress, finalsize, reinterpret_cast<uint8_t*>
               (request_in), true, NULL);

}

/**
 * Obtains the SCSI command (if any)
 * Two possibilities: if it contains a SCSI command, then it is a usable
 * message; if it doesnt contain a SCSI message, then it can't be handeld
 * by this code.
 * This is the second stage of the transfer. We have the information about
 * where the next command can be found and what the type of command is. The
 * actions that are needed from the device its side are: get the information
 * and store the information such that we can reply.
 */

void
UFSHostDevice::transferHandler(struct UTPTransferReqDesc* request_in,
                               int req_pos, Addr finaladdress, uint32_t
                               finalsize, uint32_t done)
{

    Addr cmd_desc_addr = 0x00;


    //acknowledge handling of the message
    DPRINTF(UFSHostDevice, "SCSI message detected\n");
    request_in->header.dWord2 &= 0xffffff00;
    SCSIInfo.RequestIn = request_in;
    SCSIInfo.reqPos = req_pos;
    SCSIInfo.finalAddress = finaladdress;
    SCSIInfo.finalSize = finalsize;
    SCSIInfo.destination.resize(request_in->PRDTableOffset * 4
        + request_in->PRDTableLength * sizeof(UFSHCDSGEntry));
    SCSIInfo.done = done;

    assert(!SCSIResumeEvent.scheduled());
    /**
     *Get the UTP command that has the SCSI command
     */
    cmd_desc_addr = request_in->commandDescBaseAddrHi;
    cmd_desc_addr = (cmd_desc_addr << 32) |
        (request_in->commandDescBaseAddrLo & 0xffffffff);

    writeDevice(&SCSIResumeEvent, false, cmd_desc_addr,
                SCSIInfo.destination.size(), &SCSIInfo.destination[0],0, 0);

    DPRINTF(UFSHostDevice, "SCSI scheduled\n");

    transferEventQueue.pop_front();
}

/**
 * Obtain LUN and put it in the right LUN queue. Each LUN has its own queue
 * of commands that need to be executed. This is the first instance where it
 * can be determined which Logic unit should handle the transfer. Then check
 * wether it should wait and queue or if it can continue.
 */

void
UFSHostDevice::SCSIStart()
{
    DPRINTF(UFSHostDevice, "SCSI message on hold until ready\n");
    uint32_t LUN = SCSIInfo.destination[2];
    UFSDevice[LUN]->SCSIInfoQueue.push_back(SCSIInfo);

    DPRINTF(UFSHostDevice, "SCSI queue %d has %d elements\n", LUN,
            UFSDevice[LUN]->SCSIInfoQueue.size());

    /**There are 32 doorbells, so at max there can be 32 transactions*/
    if (UFSDevice[LUN]->SCSIInfoQueue.size() < 2) //LUN is available
        SCSIResume(LUN);

    else if (UFSDevice[LUN]->SCSIInfoQueue.size() > 32)
        panic("SCSI queue is getting too big %d\n", UFSDevice[LUN]->
              SCSIInfoQueue.size());

    /**
     * First transfer is done, fetch the next;
     * At this point, the device is busy, not the HC
     */
    if (!transferEventQueue.empty()) {

        /**
         * loading next data packet in case Another LUN
         * is approached in the mean time
         */
        writeDevice(&transferEventQueue.front(), false,
                    transferStartInfo.front().address,
                    transferStartInfo.front().size, reinterpret_cast<uint8_t*>
                    (transferStartInfo.front().destination), 0, 0);

        DPRINTF(UFSHostDevice, "Transfer scheduled");
    }
}

/**
 * Handles the transfer requests that are given.
 * There can be three types of transfer. SCSI specific, Reads and writes
 * apart from the data transfer, this also generates its own reply (UPIU
 * response). Information for this reply is stored in transferInfo and will
 * be used in transferDone
 */

void
UFSHostDevice::SCSIResume(uint32_t lun_id)
{
    DPRINTF(UFSHostDevice, "SCSIresume\n");
    if (UFSDevice[lun_id]->SCSIInfoQueue.empty())
        panic("No SCSI message scheduled lun:%d Doorbell: 0x%8x", lun_id,
              UFSHCIMem.TRUTRLDBR);

    /**old info, lets form it such that we can understand it*/
    struct UTPTransferReqDesc* request_in = UFSDevice[lun_id]->
        SCSIInfoQueue.front().RequestIn;

    uint32_t req_pos = UFSDevice[lun_id]->SCSIInfoQueue.front().reqPos;

    Addr finaladdress = UFSDevice[lun_id]->SCSIInfoQueue.front().
        finalAddress;

    uint32_t finalsize = UFSDevice[lun_id]->SCSIInfoQueue.front().finalSize;

    uint32_t* transfercommand = reinterpret_cast<uint32_t*>
        (&(UFSDevice[lun_id]->SCSIInfoQueue.front().destination[0]));

    DPRINTF(UFSHostDevice, "Task tag: 0x%8x\n", transfercommand[0]>>24);
    /**call logic unit to handle SCSI command*/
    request_out_datain = UFSDevice[(transfercommand[0] & 0xFF0000) >> 16]->
        SCSICMDHandle(transfercommand);

    DPRINTF(UFSHostDevice, "LUN: %d\n", request_out_datain.LUN);

    /**
     * build response stating that it was succesful
     * command completion, Logic unit number, and Task tag
     */
    request_in->header.dWord0 = ((request_in->header.dWord0 >> 24) == 0x21)
        ? 0x36 : 0x21;
    UFSDevice[lun_id]->transferInfo.requestOut.header.dWord0 =
        request_in->header.dWord0 | (request_out_datain.LUN << 8)
        | (transfercommand[0] & 0xFF000000);
    /**SCSI status reply*/
    UFSDevice[lun_id]->transferInfo.requestOut.header.dWord1 = 0x00000000 |
        (request_out_datain.status << 24);
    /**segment size + EHS length (see UFS standard ch7)*/
    UFSDevice[lun_id]->transferInfo.requestOut.header.dWord2 = 0x00000000 |
        ((request_out_datain.senseSize + 2) << 24) | 0x05;
    /**amount of data that will follow*/
    UFSDevice[lun_id]->transferInfo.requestOut.senseDataLen =
        request_out_datain.senseSize;

    //data
    for (uint8_t count = 0; count<request_out_datain.senseSize; count++) {
        UFSDevice[lun_id]->transferInfo.requestOut.senseData[count] =
            request_out_datain.senseCode[count + 1];
    }

    /*
     * At position defined by "request_in->PRDTableOffset" (counting 32 bit
     * words) in array "transfercommand" we have a scatter gather list, which
     * is usefull to us if we interpreted it as a UFSHCDSGEntry structure.
     */
    struct UFSHCDSGEntry* sglist =  reinterpret_cast<UFSHCDSGEntry*>
        (&(transfercommand[(request_in->PRDTableOffset)]));

    uint32_t length = request_in->PRDTableLength;
    DPRINTF(UFSHostDevice, "# PRDT entries: %d\n", length);

    Addr response_addr = request_in->commandDescBaseAddrHi;
    response_addr = (response_addr << 32) |
        ((request_in->commandDescBaseAddrLo +
          (request_in->responseUPIULength << 2)) & 0xffffffff);

    /**transferdone information packet filling*/
    UFSDevice[lun_id]->transferInfo.responseStartAddr = response_addr;
    UFSDevice[lun_id]->transferInfo.reqPos = req_pos;
    UFSDevice[lun_id]->transferInfo.size = finalsize;
    UFSDevice[lun_id]->transferInfo.address = finaladdress;
    UFSDevice[lun_id]->transferInfo.destination = reinterpret_cast<uint8_t*>
        (UFSDevice[lun_id]->SCSIInfoQueue.front().RequestIn);
    UFSDevice[lun_id]->transferInfo.finished = true;
    UFSDevice[lun_id]->transferInfo.lunID = request_out_datain.LUN;

    /**
     * In this part the data that needs to be transfered will be initiated
     * and the chain of DMA (and potentially) disk transactions will be
     * started.
     */
    if (request_out_datain.expectMore == 0x01) {
        /**write transfer*/
        manageWriteTransfer(request_out_datain.LUN, request_out_datain.offset,
                            length, sglist);

    } else if (request_out_datain.expectMore == 0x02) {
        /**read transfer*/
        manageReadTransfer(request_out_datain.msgSize, request_out_datain.LUN,
                           request_out_datain.offset, length, sglist);

    } else {
        /**not disk related transfer, SCSI maintanance*/
        uint32_t count = 0;
        uint32_t size_accum = 0;
        DPRINTF(UFSHostDevice, "Data DMA size: 0x%8x\n",
                request_out_datain.msgSize);

        /**Transport the SCSI reponse data according to the SG list*/
        while ((length > count) && size_accum
               < (request_out_datain.msgSize - 1) &&
               (request_out_datain.msgSize != 0x00)) {
            Addr SCSI_start = sglist[count].upperAddr;
            SCSI_start = (SCSI_start << 32) |
                (sglist[count].baseAddr & 0xFFFFFFFF);
            DPRINTF(UFSHostDevice, "Data DMA start: 0x%8x\n", SCSI_start);
            DPRINTF(UFSHostDevice, "Data DMA size: 0x%8x\n",
                    (sglist[count].size + 1));
            /**
             * safetynet; it has been shown that sg list may be optimistic in
             * the amount of data allocated, which can potentially lead to
             * some garbage data being send over. Hence this construction
             * that finds the least amount of data that needs to be
             * transfered.
             */
            uint32_t size_to_send = sglist[count].size + 1;

            if (request_out_datain.msgSize < (size_to_send + size_accum))
                size_to_send = request_out_datain.msgSize - size_accum;

            readDevice(false, SCSI_start, size_to_send,
                       reinterpret_cast<uint8_t*>
                       (&(request_out_datain.message.dataMsg[size_accum])),
                       false, NULL);

            size_accum += size_to_send;
            DPRINTF(UFSHostDevice, "Total remaining: 0x%8x,accumulated so far"
                    " : 0x%8x\n", (request_out_datain.msgSize - size_accum),
                    size_accum);

            ++count;
            DPRINTF(UFSHostDevice, "Transfer #: %d\n", count);
        }

        /**Go to the next stage of the answering process*/
        transferDone(response_addr, req_pos, UFSDevice[lun_id]->
                     transferInfo.requestOut, finalsize, finaladdress,
                     reinterpret_cast<uint8_t*>(request_in), true, lun_id);
    }

    DPRINTF(UFSHostDevice, "SCSI resume done\n");
}

/**
 * Find finished transfer. Callback function. One of the LUNs is done with
 * the disk transfer and reports back to the controller. This function finds
 * out who it was, and calls transferDone.
 */
void
UFSHostDevice::LUNSignal()
{
    uint8_t this_lun = 0;

    //while we haven't found the right lun, keep searching
    while ((this_lun < lunAvail) && !UFSDevice[this_lun]->finishedCommand())
        ++this_lun;

    if (this_lun < lunAvail) {
        //Clear signal.
        UFSDevice[this_lun]->clearSignal();
        //found it; call transferDone
        transferDone(UFSDevice[this_lun]->transferInfo.responseStartAddr,
                     UFSDevice[this_lun]->transferInfo.reqPos,
                     UFSDevice[this_lun]->transferInfo.requestOut,
                     UFSDevice[this_lun]->transferInfo.size,
                     UFSDevice[this_lun]->transferInfo.address,
                     UFSDevice[this_lun]->transferInfo.destination,
                     UFSDevice[this_lun]->transferInfo.finished,
                     UFSDevice[this_lun]->transferInfo.lunID);
    }

    else
        panic("no LUN finished in tick %d\n", curTick());
}

/**
 * Transfer done. When the data transfer is done, this function ensures
 * that the application is notified.
 */

void
UFSHostDevice::transferDone(Addr responseStartAddr, uint32_t req_pos,
                            struct UTPUPIURSP request_out, uint32_t size,
                            Addr address, uint8_t* destination,
                            bool finished, uint32_t lun_id)
{
    /**Test whether SCSI queue hasn't popped prematurely*/
    if (UFSDevice[lun_id]->SCSIInfoQueue.empty())
        panic("No SCSI message scheduled lun:%d Doorbell: 0x%8x", lun_id,
              UFSHCIMem.TRUTRLDBR);

    DPRINTF(UFSHostDevice, "DMA start: 0x%8x; DMA size: 0x%8x\n",
            responseStartAddr, sizeof(request_out));

    struct transferStart lastinfo;
    lastinfo.mask = req_pos;
    lastinfo.done = finished;
    lastinfo.address = address;
    lastinfo.size = size;
    lastinfo.destination = reinterpret_cast<UTPTransferReqDesc*>
        (destination);
    lastinfo.lun_id = lun_id;

    transferEnd.push_back(lastinfo);

    DPRINTF(UFSHostDevice, "Transfer done start\n");

    readDevice(false, responseStartAddr, sizeof(request_out),
               reinterpret_cast<uint8_t*>
               (&(UFSDevice[lun_id]->transferInfo.requestOut)),
               true, &UTPEvent);
}

/**
 * finalUTP. Second part of the transfer done event.
 * this sends the final response: the UTP response. After this transaction
 * the doorbell shall be cleared, and the interupt shall be set.
 */

void
UFSHostDevice::finalUTP()
{
    uint32_t lun_id = transferEnd.front().lun_id;

    UFSDevice[lun_id]->SCSIInfoQueue.pop_front();
    DPRINTF(UFSHostDevice, "SCSIInfoQueue size: %d, lun: %d\n",
            UFSDevice[lun_id]->SCSIInfoQueue.size(), lun_id);

    /**stats**/
    if (UFSHCIMem.TRUTRLDBR & transferEnd.front().mask) {
        uint8_t count = 0;
        while (!(transferEnd.front().mask & (0x1 << count)))
            ++count;
        stats.transactionLatency.sample(curTick() -
                                        transactionStart[count]);
    }

    /**Last message that will be transfered*/
    readDevice(true, transferEnd.front().address,
               transferEnd.front().size, reinterpret_cast<uint8_t*>
               (transferEnd.front().destination), true, NULL);

    /**clean and ensure that the tracker is updated*/
    transferTrack &= ~(transferEnd.front().mask);
    --activeDoorbells;
    ++pendingDoorbells;
    garbage.push_back(transferEnd.front().destination);
    transferEnd.pop_front();
    DPRINTF(UFSHostDevice, "UTP handled\n");

    /**stats**/
    stats.averageDoorbell = stats.maxDoorbell.value();

    DPRINTF(UFSHostDevice, "activeDoorbells: %d, pendingDoorbells: %d,"
            " garbage: %d, TransferEvent: %d\n", activeDoorbells,
            pendingDoorbells, garbage.size(), transferEventQueue.size());

    /**This is the moment that the device is available again*/
    if (!UFSDevice[lun_id]->SCSIInfoQueue.empty())
        SCSIResume(lun_id);
}

/**
 * Read done handling function, is only initiated at the end of a transaction
 */
void
UFSHostDevice::readDone()
{
    DPRINTF(UFSHostDevice, "Read done start\n");
    --readPendingNum;

    /**Garbage collection; sort out the allocated UTP descriptor*/
    if (garbage.size() > 0) {
        delete garbage.front();
        garbage.pop_front();
        }

    /**done, generate interrupt if we havent got one already*/
    if (!(UFSHCIMem.ORInterruptStatus & 0x01)) {
        UFSHCIMem.ORInterruptStatus |= UTPTransferREQCOMPL;
        generateInterrupt();
    }


    if (!readDoneEvent.empty()) {
        readDoneEvent.pop_front();
    }
}

/**
 * set interrupt and sort out the doorbell register.
 */

void
UFSHostDevice::generateInterrupt()
{
    /**just to keep track of the transactions*/
    countInt++;

    /**step5 clear doorbell*/
    UFSHCIMem.TRUTRLDBR &= transferTrack;
    pendingDoorbells = 0;
    DPRINTF(UFSHostDevice, "Clear doorbell %X\n", UFSHCIMem.TRUTRLDBR);

    checkDrain();

    /**step6 raise interrupt*/
    gic->sendInt(intNum);
    DPRINTF(UFSHostDevice, "Send interrupt @ transaction: 0x%8x!\n",
            countInt);
}

/**
 * Clear interrupt
 */

void
UFSHostDevice::clearInterrupt()
{
    gic->clearInt(intNum);
    DPRINTF(UFSHostDevice, "Clear interrupt: 0x%8x!\n", countInt);

    checkDrain();

    if (!(UFSHCIMem.TRUTRLDBR)) {
        idlePhaseStart = curTick();
    }
    /**end of a transaction*/
}

/**
 * Important to understand about the transfer flow:
 * We have basically three stages, The "system memory" stage, the "device
 * buffer" stage and the "disk" stage. In this model we assume an infinite
 * buffer, or a buffer that is big enough to store all the data in the
 * biggest transaction. Between the three stages are two queues. Those queues
 * store the messages to simulate their transaction from one stage to the
 * next. The manage{Action} function fills up one of the queues and triggers
 * the first event, which causes a chain reaction of events executed once
 * they pass through their queues. For a write action the stages are ordered
 * "system memory", "device buffer" and "disk", whereas the read transfers
 * happen "disk", "device buffer" and "system memory". The dma action in the
 * dma device is written from a bus perspective whereas this model is written
 * from a device perspective. To avoid confusion, the translation between the
 * two has been made in the writeDevice and readDevice funtions.
 */


/**
 * Dma transaction function: write device. Note that the dma action is
 * from a device perspective, while this function is from an initiator
 * perspective
 */

void
UFSHostDevice::writeDevice(Event* additional_action, bool toDisk, Addr
                           start, int size, uint8_t* destination, uint64_t
                           SCSIDiskOffset, uint32_t lun_id)
{
    DPRINTF(UFSHostDevice, "Write transaction Start: 0x%8x; Size: %d\n",
            start, size);

    /**check whether transfer is all the way to the flash*/
    if (toDisk) {
        ++writePendingNum;

        while (!writeDoneEvent.empty() && (writeDoneEvent.front().when()
                                          < curTick()))
            writeDoneEvent.pop_front();

        writeDoneEvent.push_back(this);
        assert(!writeDoneEvent.back().scheduled());

        /**destination is an offset here since we are writing to a disk*/
        struct transferInfo new_transfer;
        new_transfer.offset = SCSIDiskOffset;
        new_transfer.size = size;
        new_transfer.lunID = lun_id;
        new_transfer.filePointer = 0;
        SSDWriteinfo.push_back(new_transfer);

        /**allocate appropriate buffer*/
        SSDWriteinfo.back().buffer.resize(size);

        /**transaction*/
        dmaPort.dmaAction(MemCmd::ReadReq, start, size,
                          &writeDoneEvent.back(),
                          &SSDWriteinfo.back().buffer[0], 0);
        //yes, a readreq at a write device function is correct.
        DPRINTF(UFSHostDevice, "Write to disk scheduled\n");

    } else {
        assert(!additional_action->scheduled());
        dmaPort.dmaAction(MemCmd::ReadReq, start, size,
                          additional_action, destination, 0);
        DPRINTF(UFSHostDevice, "Write scheduled\n");
    }
}

/**
 * Manage write transfer. Manages correct transfer flow and makes sure that
 * the queues are filled on time
 */

void
UFSHostDevice::manageWriteTransfer(uint8_t LUN, uint64_t offset, uint32_t
                                   sg_table_length, struct UFSHCDSGEntry*
                                   sglist)
{
    struct writeToDiskBurst next_packet;

    next_packet.SCSIDiskOffset = offset;

    UFSDevice[LUN]->setTotalWrite(sg_table_length);

    /**
     * Break-up the transactions into actions defined by the scatter gather
     * list.
     */
    for (uint32_t count = 0; count < sg_table_length; count++) {
        next_packet.start = sglist[count].upperAddr;
        next_packet.start = (next_packet.start << 32) |
            (sglist[count].baseAddr & 0xFFFFFFFF);
        next_packet.LUN = LUN;
        DPRINTF(UFSHostDevice, "Write data DMA start: 0x%8x\n",
                next_packet.start);
        DPRINTF(UFSHostDevice, "Write data DMA size: 0x%8x\n",
                (sglist[count].size + 1));
        assert(sglist[count].size > 0);

        if (count != 0)
            next_packet.SCSIDiskOffset = next_packet.SCSIDiskOffset +
                (sglist[count - 1].size + 1);

        next_packet.size = sglist[count].size + 1;

        /**If the queue is empty, the transaction should be initiated*/
        if (dmaWriteInfo.empty())
            writeDevice(NULL, true, next_packet.start, next_packet.size,
                        NULL, next_packet.SCSIDiskOffset, next_packet.LUN);
        else
            DPRINTF(UFSHostDevice, "Write not initiated queue: %d\n",
                    dmaWriteInfo.size());

        dmaWriteInfo.push_back(next_packet);
        DPRINTF(UFSHostDevice, "Write Location: 0x%8x\n",
                next_packet.SCSIDiskOffset);

        DPRINTF(UFSHostDevice, "Write transfer #: 0x%8x\n", count + 1);

        /** stats **/
        stats.totalWrittenSSD += (sglist[count].size + 1);
    }

    /**stats**/
    ++stats.totalWriteUFSTransactions;
}

/**
 * Write done handling function. Is only initiated when the flash is directly
 * approached
 */

void
UFSHostDevice::writeDone()
{
    /**DMA is done, information no longer needed*/
    assert(dmaWriteInfo.size() > 0);
    dmaWriteInfo.pop_front();
    assert(SSDWriteinfo.size() > 0);
    uint32_t lun = SSDWriteinfo.front().lunID;

    /**If there is nothing on the way, we need to start the events*/
    DPRINTF(UFSHostDevice, "Write done entered, queue: %d\n",
            UFSDevice[lun]->SSDWriteDoneInfo.size());
    /**Write the disk*/
    UFSDevice[lun]->writeFlash(&SSDWriteinfo.front().buffer[0],
                               SSDWriteinfo.front().offset,
                               SSDWriteinfo.front().size);

    /**
     * Move to the second queue, enter the logic unit
     * This is where the disk is approached and the flash transaction is
     * handled SSDWriteDone will take care of the timing
     */
    UFSDevice[lun]->SSDWriteDoneInfo.push_back(SSDWriteinfo.front());
    SSDWriteinfo.pop_front();

    --writePendingNum;
    /**so far, only the DMA part has been handled, lets do the disk delay*/
    UFSDevice[lun]->SSDWriteStart();

    /** stats **/
    stats.currentWriteSSDQueue = UFSDevice[lun]->SSDWriteDoneInfo.size();
    stats.averageWriteSSDQueue = UFSDevice[lun]->SSDWriteDoneInfo.size();
    ++stats.totalWriteDiskTransactions;

    /**initiate the next dma action (if any)*/
    if (!dmaWriteInfo.empty())
        writeDevice(NULL, true, dmaWriteInfo.front().start,
                    dmaWriteInfo.front().size,  NULL,
                    dmaWriteInfo.front().SCSIDiskOffset,
                    dmaWriteInfo.front().LUN);
    DPRINTF(UFSHostDevice, "Write done end\n");
}

/**
 * SSD write start. Starts the write action in the timing model
 */
void
UFSHostDevice::UFSSCSIDevice::SSDWriteStart()
{
    assert(SSDWriteDoneInfo.size() > 0);
    flashDevice->writeMemory(
        SSDWriteDoneInfo.front().offset,
        SSDWriteDoneInfo.front().size, memWriteCallback);

    SSDWriteDoneInfo.pop_front();

    DPRINTF(UFSHostDevice, "Write is started; left in queue: %d\n",
            SSDWriteDoneInfo.size());
}


/**
 * SSDisk write done
 */

void
UFSHostDevice::UFSSCSIDevice::SSDWriteDone()
{
    DPRINTF(UFSHostDevice, "Write disk, aiming for %d messages, %d so far\n",
            totalWrite, amountOfWriteTransfers);

    //we have done one extra transfer
    ++amountOfWriteTransfers;

    /**test whether call was correct*/
    assert(totalWrite >= amountOfWriteTransfers && totalWrite != 0);

    /**are we there yet? (did we do everything)*/
    if (totalWrite == amountOfWriteTransfers) {
        DPRINTF(UFSHostDevice, "Write transactions finished\n");
        totalWrite = 0;
        amountOfWriteTransfers = 0;

        //Callback UFS Host
        setSignal();
        signalDone->process();
    }

}

/**
 * Dma transaction function: read device. Notice that the dma action is from
 * a device perspective, while this function is from an initiator perspective
 */

void
UFSHostDevice::readDevice(bool lastTransfer, Addr start, uint32_t size,
                          uint8_t* destination, bool no_cache, Event*
                          additional_action)
{
    DPRINTF(UFSHostDevice, "Read start: 0x%8x; Size: %d, data[0]: 0x%8x\n",
            start, size, (reinterpret_cast<uint32_t *>(destination))[0]);

    /** check wether interrupt is needed */
    if (lastTransfer) {
        ++readPendingNum;
        readDoneEvent.push_back(this);
        assert(!readDoneEvent.back().scheduled());
        dmaPort.dmaAction(MemCmd::WriteReq, start, size,
                          &readDoneEvent.back(), destination, 0);
        //yes, a writereq at a read device function is correct.

    } else {
        if (additional_action != NULL)
            assert(!additional_action->scheduled());

        dmaPort.dmaAction(MemCmd::WriteReq, start, size,
                          additional_action, destination, 0);

    }

}

/**
 * Manage read transfer. Manages correct transfer flow and makes sure that
 * the queues are filled on time
 */

void
UFSHostDevice::manageReadTransfer(uint32_t size, uint32_t LUN, uint64_t
                                  offset, uint32_t sg_table_length,
                                  struct UFSHCDSGEntry* sglist)
{
    uint32_t size_accum = 0;

    DPRINTF(UFSHostDevice, "Data READ size: %d\n", size);

    /**
     * Break-up the transactions into actions defined by the scatter gather
     * list.
     */
    for (uint32_t count = 0; count < sg_table_length; count++) {
        struct transferInfo new_transfer;
        new_transfer.offset = sglist[count].upperAddr;
        new_transfer.offset = (new_transfer.offset << 32) |
            (sglist[count].baseAddr & 0xFFFFFFFF);
        new_transfer.filePointer = offset + size_accum;
        new_transfer.size = (sglist[count].size + 1);
        new_transfer.lunID = LUN;

        DPRINTF(UFSHostDevice, "Data READ start: 0x%8x; size: %d\n",
                new_transfer.offset, new_transfer.size);

        UFSDevice[LUN]->SSDReadInfo.push_back(new_transfer);
        UFSDevice[LUN]->SSDReadInfo.back().buffer.resize(sglist[count].size
                                                         + 1);

        /**
         * The disk image is read here; but the action is simultated later
         * You can see this as the preparation stage, whereas later is the
         * simulation phase.
         */
        UFSDevice[LUN]->readFlash(&UFSDevice[LUN]->
                                  SSDReadInfo.back().buffer[0],
                                  offset + size_accum,
                                  sglist[count].size + 1);

        size_accum += (sglist[count].size + 1);

        DPRINTF(UFSHostDevice, "Transfer %d; Remaining: 0x%8x, Accumulated:"
                " 0x%8x\n", (count + 1), (size-size_accum), size_accum);

        /** stats **/
        stats.totalReadSSD += (sglist[count].size + 1);
        stats.currentReadSSDQueue = UFSDevice[LUN]->SSDReadInfo.size();
        stats.averageReadSSDQueue = UFSDevice[LUN]->SSDReadInfo.size();
    }

    UFSDevice[LUN]->SSDReadStart(sg_table_length);

    /** stats **/
    ++stats.totalReadUFSTransactions;

}



/**
 * SSDisk start read; this function was created to keep the interfaces
 * between the layers simpler. Without this function UFSHost would need to
 * know about the flashdevice.
 */

void
UFSHostDevice::UFSSCSIDevice::SSDReadStart(uint32_t total_read)
{
    totalRead = total_read;
    for (uint32_t number_handled = 0; number_handled < SSDReadInfo.size();
         number_handled++) {
        /**
         * Load all the read request to the Memory device.
         * It will call back when done.
         */
        flashDevice->readMemory(SSDReadInfo.front().filePointer,
                                SSDReadInfo.front().size, memReadCallback);
    }

}


/**
 * SSDisk read done
 */

void
UFSHostDevice::UFSSCSIDevice::SSDReadDone()
{
    DPRINTF(UFSHostDevice, "SSD read done at lun %d, Aiming for %d messages,"
            " %d so far\n", lunID, totalRead, amountOfReadTransfers);

    if (totalRead == amountOfReadTransfers) {
        totalRead = 0;
        amountOfReadTransfers = 0;

        /**Callback: transferdone*/
        setSignal();
        signalDone->process();
    }

}

/**
 * Read callback, on the way from the disk to the DMA. Called by the flash
 * layer. Intermediate step to the host layer
 */
void
UFSHostDevice::UFSSCSIDevice::readCallback()
{
    ++amountOfReadTransfers;

    /**Callback; make sure data is transfered upstream:
     * UFSHostDevice::readCallback
     */
    setReadSignal();
    deviceReadCallback->process();

    //Are we done yet?
    SSDReadDone();
}

/**
 * Read callback, on the way from the disk to the DMA. Called by the UFSSCSI
 * layer.
 */

void
UFSHostDevice::readCallback()
{
    DPRINTF(UFSHostDevice, "Read Callback\n");
    uint8_t this_lun = 0;

    //while we haven't found the right lun, keep searching
    while ((this_lun < lunAvail) && !UFSDevice[this_lun]->finishedRead())
        ++this_lun;

    DPRINTF(UFSHostDevice, "Found LUN %d messages pending for clean: %d\n",
            this_lun, SSDReadPending.size());

    if (this_lun < lunAvail) {
        //Clear signal.
        UFSDevice[this_lun]->clearReadSignal();
        SSDReadPending.push_back(UFSDevice[this_lun]->SSDReadInfo.front());
        UFSDevice[this_lun]->SSDReadInfo.pop_front();
        readGarbageEventQueue.push_back(this);

        //make sure the queue is popped a the end of the dma transaction
        readDevice(false, SSDReadPending.front().offset,
                   SSDReadPending.front().size,
                   &SSDReadPending.front().buffer[0], false,
                   &readGarbageEventQueue.back());

        /**stats*/
        ++stats.totalReadDiskTransactions;
    }
    else
        panic("no read finished in tick %d\n", curTick());
}

/**
 * After a disk read DMA transfer, the structure needs to be freed. This is
 * done in this function.
 */
void
UFSHostDevice::readGarbage()
{
    DPRINTF(UFSHostDevice, "Clean read data, %d\n", SSDReadPending.size());
    SSDReadPending.pop_front();
    readGarbageEventQueue.pop_front();
}

/**
 * Serialize; needed to make checkpoints
 */

void
UFSHostDevice::serialize(CheckpointOut &cp) const
{
    DmaDevice::serialize(cp);

    const uint8_t* temp_HCI_mem = reinterpret_cast<const uint8_t*>(&UFSHCIMem);
    SERIALIZE_ARRAY(temp_HCI_mem, sizeof(HCIMem));

    uint32_t lun_avail = lunAvail;
    SERIALIZE_SCALAR(lun_avail);
}


/**
 * Unserialize; needed to restore from checkpoints
 */

void
UFSHostDevice::unserialize(CheckpointIn &cp)
{
    DmaDevice::unserialize(cp);
    uint8_t* temp_HCI_mem = reinterpret_cast<uint8_t*>(&UFSHCIMem);
    UNSERIALIZE_ARRAY(temp_HCI_mem, sizeof(HCIMem));

    uint32_t lun_avail;
    UNSERIALIZE_SCALAR(lun_avail);
    assert(lunAvail == lun_avail);
}


/**
 * Drain; needed to enable checkpoints
 */

DrainState
UFSHostDevice::drain()
{
    if (UFSHCIMem.TRUTRLDBR) {
        DPRINTF(UFSHostDevice, "UFSDevice is draining...\n");
        return DrainState::Draining;
    } else {
        DPRINTF(UFSHostDevice, "UFSDevice drained\n");
        return DrainState::Drained;
    }
}

/**
 * Checkdrain; needed to enable checkpoints
 */

void
UFSHostDevice::checkDrain()
{
    if (drainState() != DrainState::Draining)
        return;

    if (UFSHCIMem.TRUTRLDBR) {
        DPRINTF(UFSHostDevice, "UFSDevice is still draining; with %d active"
            " doorbells\n", activeDoorbells);
    } else {
        DPRINTF(UFSHostDevice, "UFSDevice is done draining\n");
        signalDrainDone();
    }
}

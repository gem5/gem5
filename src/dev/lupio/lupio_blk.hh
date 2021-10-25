/*
 * Copyright (c) 2021 The Regents of the University of California
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

#ifndef __DEV_LUPIO_LUPIO_BLK_HH__
#define __DEV_LUPIO_LUPIO_BLK_HH__

#include "debug/LupioBLK.hh"
#include "dev/dma_device.hh"
#include "dev/io_device.hh"
#include "dev/lupio/lupio_pic.hh"
#include "dev/platform.hh"
#include "dev/storage/disk_image.hh"
#include "params/LupioBLK.hh"
#include "sim/system.hh"

namespace gem5
{

/**
 * LupioBLK:
 * A virtual block device which aims to provide a disk-like interface for
 * second-level storage. It enables the transfer of blocks from the block
 * device to the system's main memory, and vice-versa.
 */
class LupioBLK : public DmaDevice
{
  protected:
    const ByteOrder byteOrder = ByteOrder::little;
    Platform *platform;
    EventFunctionWrapper dmaEvent;
    Addr pioAddr;
    Tick pioDelay;
    Addr pioSize;
    DiskImage &image;
    int lupioBLKIntID;

    void dmaEventDone();

  // Register map
  private:
    enum
    {
        LUPIO_BLK_CONF,

        LUPIO_BLK_NBLK,
        LUPIO_BLK_BLKA,
        LUPIO_BLK_MEMA,

        LUPIO_BLK_CTRL,
        LUPIO_BLK_STAT
    };

    int reqLen = 0;
    uint8_t *reqData = nullptr;

    uint64_t nbBlocks = 0;
    uint32_t nblk = 0;
    uint32_t lba = 0;
    Addr mem = 0;

    bool writeOp = false;
    bool err = false;
    bool busy = false;
    /**
     * Function to return data pertaining to blocks being transferred
     */
    uint64_t lupioBLKRead(const uint8_t addr);
    /**
     * Function to write to registers containing data pertaining to
     * block transfers
     */
    void lupioBLKWrite(const uint8_t addr, uint64_t val64);
    /**
     * Function to initiate and direct the transfer of data by the
     *  DMA device
     **/
    void lupioBLKCmd(void);

  public:
    PARAMS(LupioBLK);
    LupioBLK(const Params &params);

    /**
     * Implement BasicPioDevice virtual functions
     */
    AddrRangeList getAddrRanges() const override;
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace gem5

#endif // __DEV_LUPIO_LUPIO_BLK_HH_

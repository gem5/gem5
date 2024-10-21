/*
 * Copyright (c) 2012-2013, 2021 ARM Limited
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
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

#ifndef __MEM_FLASH_MEM_HH__
#define __MEM_FLASH_MEM_HH__

#include "base/random.hh"
#include "mem/abstract_mem.hh"
#include "params/CfiMemory.hh"

namespace gem5
{

namespace memory
{

/**
 * CfiMemory: This is modelling a flash memory adhering to the
 * Common Flash Interface (CFI):
 *
 * JEDEC JESD68.01
 * JEDEC JEP137B
 * Intel Application Note 646
 *
 * This is as of now a pure functional model of a flash controller:
 * no timing/power information has been encoded in it and it is therefore
 * not representive of a real device. Some voltage/timing values have
 * nevertheless been encoded in the CFI table.
 * This is just a requirement from the CFI specification: guest software
 * might query those entries, but they are not reflected in gem5 statistics.
 *
 * The model is meant to be used to allow execution of flash drivers
 * (e.g. UEFI firmware storing EFI variables in non volatile memory)
 */
class CfiMemory : public AbstractMemory
{
  private:
    enum class CfiCommand
    {
        NO_CMD = 0,
        LOCK_BLOCK = 0x1,
        ERASE_BLOCK_SETUP = 0x20,
        WORD_PROGRAM = 0x40,
        CLEAR_STATUS_REG = 0x50,
        LOCK_BLOCK_SETUP = 0x60,
        READ_STATUS_REG = 0x70,
        READ_DEVICE_ID = 0x90,
        READ_CFI_QUERY = 0x98,
        BUFFERED_PROGRAM_SETUP = 0xE8,
        BUFFERED_PROGRAM_CONFIRM = 0xD0,
        BLOCK_ERASE_CONFIRM = 0xD0,
        UNLOCK_BLOCK = 0xD0,
        AMD_RESET=0xF0,
        READ_ARRAY = 0xFF,
        /** This is not a real command, but it is used by the internal
         * model only to represent the 2nd write cycle state for a buffered
         * program (when the buffer size is supplied) */
        BUFFER_SIZE_READ,
    };

    /** Possible in the status register */
    static const uint8_t STATUS_ERASE_ERROR = 0x30;
    static const uint8_t STATUS_LOCK_ERROR = 0x12;
    static const uint8_t STATUS_READY = 0x80;
    static const uint8_t STATUS_PROGRAM_LOCK_BIT = 0x10;

    /** Metadata about the erase blocks in flash */
    struct BlockData : public Serializable
    {
        BlockData(const CfiMemory &_parent, ssize_t number, ssize_t size)
          : Serializable(), locked(number, false), blockSize(size),
            parent(_parent)
        {}

        /**
         * Return true if the block pointed by the block_address
         * parameter is locked
         *
         * @params block_address address of the erase block in flash
         *         memory: first block starts ad address 0x0
         * @return true if block is locked
         */
        bool isLocked(Addr block_address) const;

        /**
         * Lock the block pointed by the block_address
         * parameter
         *
         * @params block_address address of the erase block in flash
         *         memory: first block starts ad address 0x0
         */
        void lock(Addr block_address);

        /**
         * Unlock the block pointed by the block_address
         * parameter
         *
         * @params block_address address of the erase block in flash
         *         memory: first block starts ad address 0x0
         */
        void unlock(Addr block_address);

        /** Erase a single block. The address of the block
         * is supplied by the packet address.
         *
         * @params pkt memory packet targeting the erase block
         */
        void erase(PacketPtr pkt);

        /** Number of erase blocks in flash memory */
        ssize_t number() const { return locked.size(); }

        /** Size in bytes of a single erase block */
        ssize_t size() const { return blockSize; }

      private: // Serializable
        void serialize(CheckpointOut &cp) const override;

        void unserialize(CheckpointIn &cp) override;

      private:
        uint32_t blockIdx(Addr block_address) const;

        // Per block flag. True if the block is locked
        std::vector<bool> locked;

        // Size of the block in bytes
        const ssize_t blockSize;

        const CfiMemory &parent;
    };

    /**
     * Word Buffer used by the BUFFERED PROGRAM command
     * to write (program) chunks of words to flash
     */
    struct ProgramBuffer : public Serializable
    {
      public:
        // program buffer max size = 32 words
        static const ssize_t MAX_BUFFER_SIZE = 32 * 4;

        ProgramBuffer(const CfiMemory &_parent)
          : Serializable(), parent(_parent)
        {}

        /**
         * Start buffering
         * @param buffer_size new size (in bytes) of the program buffer
         */
        void setup(ssize_t buffer_size);

        /**
         * Write data into the buffer. If the buffer is full, the
         * method will return true, meaning it's time to write
         * back the buffer into memory
         *
         * @params flash_address address in flash (relative to the start)
         * @params data_ptr pointer to the data
         * @params size number of bytes to be written to the buffer
         *
         * @return true if buffer needs to be written back to flash
         */
        bool write(Addr flash_address, void *data_ptr, ssize_t size);

        bool writeback();

      private:
        void serialize(CheckpointOut &cp) const override;

        void unserialize(CheckpointIn &cp) override;

      private:
        // program buffer
        std::vector<uint8_t> buffer;

        // Number of bytes written in the buffer
        ssize_t bytesWritten = 0;

        // Pointing to the latest written word in the buffer
        Addr blockPointer = 0;

        const CfiMemory &parent;
    };

    /**
     * A deferred packet stores a packet along with its scheduled
     * transmission time
     */
    class DeferredPacket
    {

      public:

        const Tick tick;
        const PacketPtr pkt;

        DeferredPacket(PacketPtr _pkt, Tick _tick) : tick(_tick), pkt(_pkt)
        { }
    };

    class MemoryPort : public ResponsePort
    {
      private:
        CfiMemory& mem;

      public:
        MemoryPort(const std::string& _name, CfiMemory& _memory);

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        Tick recvAtomicBackdoor(
                PacketPtr pkt, MemBackdoorPtr &_backdoor) override;
        void recvFunctional(PacketPtr pkt) override;
        void recvMemBackdoorReq(const MemBackdoorReq &req,
                MemBackdoorPtr &_backdoor) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override;
    };

    MemoryPort port;

    /**
     * Latency from that a request is accepted until the response is
     * ready to be sent.
     */
    const Tick latency;

    /**
     * Fudge factor added to the latency.
     */
    const Tick latency_var;

    /**
     * Internal (unbounded) storage to mimic the delay caused by the
     * actual memory access. Note that this is where the packet spends
     * the memory latency.
     */
    std::list<DeferredPacket> packetQueue;

    /**
     * Bandwidth in ticks per byte. The regulation affects the
     * acceptance rate of requests and the queueing takes place after
     * the regulation.
     */
    const double bandwidth;

    /**
     * Track the state of the memory as either idle or busy, no need
     * for an enum with only two states.
     */
    bool isBusy;

    /**
     * Remember if we have to retry an outstanding request that
     * arrived while we were busy.
     */
    bool retryReq;

    /**
     * Remember if we failed to send a response and are awaiting a
     * retry. This is only used as a check.
     */
    bool retryResp;

    /**
     * Release the memory after being busy and send a retry if a
     * request was rejected in the meanwhile.
     */
    void release();

    EventFunctionWrapper releaseEvent;

    /**
     * Dequeue a packet from our internal packet queue and move it to
     * the port where it will be sent as soon as possible.
     */
    void dequeue();

    EventFunctionWrapper dequeueEvent;

    /**
     * Detemine the latency.
     *
     * @return the latency seen by the current packet
     */
    Tick getLatency() const;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

    const uint8_t numberOfChips;

    const uint16_t vendorID;
    const uint16_t deviceID;
    const uint16_t bankWidth;

    /** Previous command (issued in the previous write cycle) */
    CfiCommand readState;
    CfiCommand writeState;

    uint8_t statusRegister;

    BlockData blocks;

    ProgramBuffer programBuffer;

    uint8_t cfiQueryTable[61];

    mutable Random::RandomPtr rng = Random::genRandom();

  public:
    CfiMemory(const CfiMemoryParams &p);

    DrainState drain() override;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;
    void init() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected:
    Tick recvAtomic(PacketPtr pkt);
    Tick recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &_backdoor);
    void recvFunctional(PacketPtr pkt);
    void recvMemBackdoorReq(const MemBackdoorReq &req,
            MemBackdoorPtr &_backdoor);
    bool recvTimingReq(PacketPtr pkt);
    void recvRespRetry();

    /** Make a read/write access to the CFI Memory */
    void cfiAccess(PacketPtr pkt);

    /** Write request to the CFI Memory */
    void write(PacketPtr pkt);

    /** Read request to the CFI Memory */
    void read(PacketPtr pkt);

    /**
     * Helper function to read the device identifier after the
     * read state machine is put in the CfiCommand::READ_DEVICE_ID
     * mode.
     *
     * @param flash_address: The flash address LSBits encode the
     *                       the information the software is trying
     *                       to read
     */
    uint64_t readDeviceID(Addr flash_address) const;

    /**
     * Service a new command issued to the flash device
     *
     * @param command: new command issued to the flash device
     */
    void handleCommand(CfiCommand command);

    /** Return the selected entry in the CFI table
     *
     * @param addr: offset in the CFI table
     */
    uint64_t cfiQuery(Addr addr);
};

} // namespace memory
} // namespace gem5

#endif

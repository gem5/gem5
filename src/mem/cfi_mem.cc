/*
 * Copyright (c) 2010-2013, 2015, 2021 ARM Limited
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

#include "mem/cfi_mem.hh"

#include <cmath>

#include "base/intmath.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/CFI.hh"
#include "debug/Drain.hh"

namespace gem5
{

namespace memory
{

bool
CfiMemory::BlockData::isLocked(Addr block_address) const
{
    return locked[blockIdx(block_address)];
}

void
CfiMemory::BlockData::lock(Addr block_address)
{
    locked[blockIdx(block_address)] = true;
}

void
CfiMemory::BlockData::unlock(Addr block_address)
{
    locked[blockIdx(block_address)] = false;
}

void
CfiMemory::BlockData::serialize(CheckpointOut &cp) const
{
    SERIALIZE_CONTAINER(locked);
}

void
CfiMemory::BlockData::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_CONTAINER(locked);
}

uint32_t
CfiMemory::BlockData::blockIdx(Addr block_address) const
{
    return block_address / blockSize;
}

void
CfiMemory::ProgramBuffer::setup(ssize_t buffer_size)
{
    /** Clipping the size to its limit */
    if (buffer_size > MAX_BUFFER_SIZE) {
        buffer_size = MAX_BUFFER_SIZE;
    }

    buffer.resize(buffer_size);
    std::fill(buffer.begin(), buffer.end(), 0);
    bytesWritten = 0;
}

bool
CfiMemory::ProgramBuffer::write(Addr flash_address, void *data_ptr,
                                ssize_t size)
{
    if (bytesWritten >= buffer.size())
        return true;

    if (bytesWritten == 0) {
        blockPointer = flash_address;
    }

    const Addr offset = flash_address - blockPointer;

    if (flash_address < blockPointer || offset >= MAX_BUFFER_SIZE)
        return true;

    std::memcpy(buffer.data() + offset, data_ptr, size);
    bytesWritten += size;

    return false;
}

bool
CfiMemory::ProgramBuffer::writeback()
{
    if (parent.blocks.isLocked(blockPointer)) {
        return false;
    } else {
        std::memcpy(parent.toHostAddr(parent.start() + blockPointer),
                    buffer.data(), bytesWritten);
        return true;
    }
}

void
CfiMemory::ProgramBuffer::serialize(CheckpointOut &cp) const
{
    SERIALIZE_CONTAINER(buffer);
    SERIALIZE_SCALAR(bytesWritten);
    SERIALIZE_SCALAR(blockPointer);
}

void
CfiMemory::ProgramBuffer::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_CONTAINER(buffer);
    UNSERIALIZE_SCALAR(bytesWritten);
    UNSERIALIZE_SCALAR(blockPointer);
}

CfiMemory::CfiMemory(const CfiMemoryParams &p)
    : AbstractMemory(p),
      port(name() + ".port", *this),
      latency(p.latency),
      latency_var(p.latency_var),
      bandwidth(p.bandwidth),
      isBusy(false),
      retryReq(false),
      retryResp(false),
      releaseEvent([this] { release(); }, name()),
      dequeueEvent([this] { dequeue(); }, name()),
      numberOfChips(2),
      vendorID(p.vendor_id),
      deviceID(p.device_id),
      bankWidth(p.bank_width),
      readState(CfiCommand::READ_ARRAY),
      writeState(CfiCommand::NO_CMD),
      statusRegister(STATUS_READY),
      blocks(*this, size() / p.blk_size, p.blk_size),
      programBuffer(*this),
      cfiQueryTable{
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00,
          /* Query-unique ASCII string */
          'Q', 'R', 'Y',
          /* Primary Algorithm Command Set and Control = Intel/Sharp */
          0x01, 0x00,
          /* Address for Primary Algorithm extended Query */
          0x31, 0x00,
          /* Alternative Algorithm Command Set and Control Interface */
          0x00, 0x00,
          /* Address for Alternative Algorithm extended Query */
          0x00, 0x00,
          /* Vcc Minimum Program/Erase or Write voltage ([7:4].[3-0]V) */
          0x45,
          /* Vcc Maximum Program/Erase or Write voltage ([7:4].[3-0]V) */
          0x55,
          /* Vpp Minimum Program/Erase voltage (0 = No Vpp pin) */
          0x00,
          /* Vpp Minimum Program/Erase voltage (0 = No Vpp pin) */
          0x00,
          /* Typical timeout per single byte/word/D-word program: (2^N us) */
          0x01,
          /* Typical timeout for maximum-size multi-byte program: (2^N us) */
          0x01,
          /* Typical timeout per individual block erase: (2^N ms) */
          0x01,
          /* Typical timeout for full chip erase: (2^N ms) */
          0x00,
          /* Maximum timeout for byte/word/D-word program (2^N typical) */
          0x00,
          /* Maximum timeout for multi-byte program (2^N typical) */
          0x00,
          /* Maximum timeout per individual block erase (2^N typical) */
          0x00,
          /* Maximum timeout for chip erase (2^N typical) */
          0x00,
          /* Device Size in number of bytes (2^N) */
          static_cast<uint8_t>(log2(size())),
          /* Flash Device Interface Code description */
          0x05, 0x00,
          /* Maximum number of bytes in multi-byte program (2^N) */
          static_cast<uint8_t>(
              bits(log2i(ProgramBuffer::MAX_BUFFER_SIZE), 7, 0)),
          static_cast<uint8_t>(
              bits(log2i(ProgramBuffer::MAX_BUFFER_SIZE), 15, 8)),
          /* Number of Erase Block Regions within device */
          0x01,
          /* Erase Block Region Information */
          static_cast<uint8_t>(bits(blocks.number(), 7, 0)),
          static_cast<uint8_t>(bits(blocks.number(), 15, 8)),
          static_cast<uint8_t>(bits(blocks.size(), 7, 0)),
          static_cast<uint8_t>(bits(blocks.size(), 15, 8)), 0x00, 0x00, 0x00,
          0x00,                   // empty Block region 2 info
          0x00, 0x00, 0x00, 0x00, // empty Block region 3 info
          0x00, 0x00, 0x00, 0x00  // empty Block region 4 info
      }
{}

void
CfiMemory::init()
{
    AbstractMemory::init();

    // allow unconnected memories as this is used in several ruby
    // systems at the moment
    if (port.isConnected()) {
        port.sendRangeChange();
    }
}

Tick
CfiMemory::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
                                     "is responding");

    cfiAccess(pkt);
    return getLatency();
}

Tick
CfiMemory::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &_backdoor)
{
    Tick latency = recvAtomic(pkt);

    if (backdoor.ptr())
        _backdoor = &backdoor;
    return latency;
}

void
CfiMemory::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    functionalAccess(pkt);

    bool done = false;
    auto p = packetQueue.begin();
    // potentially update the packets in our packet queue as well
    while (!done && p != packetQueue.end()) {
        done = pkt->trySatisfyFunctional(p->pkt);
        ++p;
    }

    pkt->popLabel();
}

void
CfiMemory::recvMemBackdoorReq(const MemBackdoorReq &req,
                              MemBackdoorPtr &_backdoor)
{
    if (backdoor.ptr())
        _backdoor = &backdoor;
}

bool
CfiMemory::recvTimingReq(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
                                     "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at memory controller, "
             "saw %s to %#llx\n",
             pkt->cmdString(), pkt->getAddr());

    // we should not get a new request after committing to retry the
    // current one, but unfortunately the CPU violates this rule, so
    // simply ignore it for now
    if (retryReq)
        return false;

    // if we are busy with a read or write, remember that we have to
    // retry
    if (isBusy) {
        retryReq = true;
        return false;
    }

    // technically the packet only reaches us after the header delay,
    // and since this is a memory controller we also need to
    // deserialise the payload before performing any write operation
    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;

    // update the release time according to the bandwidth limit, and
    // do so with respect to the time it takes to finish this request
    // rather than long term as it is the short term data rate that is
    // limited for any real memory

    // calculate an appropriate tick to release to not exceed
    // the bandwidth limit
    Tick duration = pkt->getSize() * bandwidth;

    // only consider ourselves busy if there is any need to wait
    // to avoid extra events being scheduled for (infinitely) fast
    // memories
    if (duration != 0) {
        schedule(releaseEvent, curTick() + duration);
        isBusy = true;
    }

    // go ahead and deal with the packet and put the response in the
    // queue if there is one
    bool needs_response = pkt->needsResponse();
    recvAtomic(pkt);
    // turn packet around to go back to requester if response expected
    if (needs_response) {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());

        Tick when_to_send = curTick() + receive_delay + getLatency();

        // typically this should be added at the end, so start the
        // insertion sort with the last element, also make sure not to
        // re-order in front of some existing packet with the same
        // address, the latter is important as this memory effectively
        // hands out exclusive copies (shared is not asserted)
        auto i = packetQueue.end();
        --i;
        while (i != packetQueue.begin() && when_to_send < i->tick &&
               !i->pkt->matchAddr(pkt)) {
            --i;
        }

        // emplace inserts the element before the position pointed to by
        // the iterator, so advance it one step
        packetQueue.emplace(++i, pkt, when_to_send);

        if (!retryResp && !dequeueEvent.scheduled())
            schedule(dequeueEvent, packetQueue.back().tick);
    } else {
        pendingDelete.reset(pkt);
    }

    return true;
}

void
CfiMemory::release()
{
    assert(isBusy);
    isBusy = false;
    if (retryReq) {
        retryReq = false;
        port.sendRetryReq();
    }
}

void
CfiMemory::dequeue()
{
    assert(!packetQueue.empty());
    DeferredPacket deferred_pkt = packetQueue.front();

    retryResp = !port.sendTimingResp(deferred_pkt.pkt);

    if (!retryResp) {
        packetQueue.pop_front();

        // if the queue is not empty, schedule the next dequeue event,
        // otherwise signal that we are drained if we were asked to do so
        if (!packetQueue.empty()) {
            // if there were packets that got in-between then we
            // already have an event scheduled, so use re-schedule
            reschedule(dequeueEvent,
                       std::max(packetQueue.front().tick, curTick()), true);
        } else if (drainState() == DrainState::Draining) {
            DPRINTF(Drain, "Draining of CfiMemory complete\n");
            signalDrainDone();
        }
    }
}

Tick
CfiMemory::getLatency() const
{
    return latency +
           (latency_var ? random_mt.random<Tick>(0, latency_var) : 0);
}

void
CfiMemory::recvRespRetry()
{
    assert(retryResp);

    dequeue();
}

Port &
CfiMemory::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "port") {
        return AbstractMemory::getPort(if_name, idx);
    } else {
        return port;
    }
}

DrainState
CfiMemory::drain()
{
    if (!packetQueue.empty()) {
        DPRINTF(Drain, "CfiMemory Queue has requests, waiting to drain\n");
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

void
CfiMemory::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ENUM(readState);
    SERIALIZE_ENUM(writeState);

    SERIALIZE_SCALAR(statusRegister);

    SERIALIZE_OBJ(blocks);
    SERIALIZE_OBJ(programBuffer);
}

void
CfiMemory::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ENUM(readState);
    UNSERIALIZE_ENUM(writeState);

    UNSERIALIZE_SCALAR(statusRegister);

    UNSERIALIZE_OBJ(blocks);
    UNSERIALIZE_OBJ(programBuffer);
}

CfiMemory::MemoryPort::MemoryPort(const std::string &_name, CfiMemory &_memory)
    : ResponsePort(_name), mem(_memory)
{}

AddrRangeList
CfiMemory::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(mem.getAddrRange());
    return ranges;
}

Tick
CfiMemory::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return mem.recvAtomic(pkt);
}

Tick
CfiMemory::MemoryPort::recvAtomicBackdoor(PacketPtr pkt,
                                          MemBackdoorPtr &_backdoor)
{
    return mem.recvAtomicBackdoor(pkt, _backdoor);
}

void
CfiMemory::MemoryPort::recvFunctional(PacketPtr pkt)
{
    mem.recvFunctional(pkt);
}

void
CfiMemory::MemoryPort::recvMemBackdoorReq(const MemBackdoorReq &req,
                                          MemBackdoorPtr &_backdoor)
{
    mem.recvMemBackdoorReq(req, _backdoor);
}

bool
CfiMemory::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    return mem.recvTimingReq(pkt);
}

void
CfiMemory::MemoryPort::recvRespRetry()
{
    mem.recvRespRetry();
}

void
CfiMemory::cfiAccess(PacketPtr pkt)
{
    if (pkt->isWrite()) {
        write(pkt);
    } else {
        read(pkt);
    }
}

void
CfiMemory::write(PacketPtr pkt)
{
    DPRINTF(CFI, "write, address: %#x, val: %#x\n", pkt->getAddr(),
            pkt->getUintX(ByteOrder::little));

    const Addr flash_address = pkt->getAddr() - start();

    const uint16_t value = pkt->getUintX(ByteOrder::little) & 0xffff;
    const auto new_cmd = static_cast<CfiCommand>(value & 0xff);

    switch (writeState) {
    case CfiCommand::NO_CMD:
        handleCommand(new_cmd);
        break;

    case CfiCommand::ERASE_BLOCK_SETUP:
        if (new_cmd == CfiCommand::BLOCK_ERASE_CONFIRM) {
            // Erasing the block
            // Check if block is locked
            if (blocks.isLocked(flash_address)) {
                statusRegister |= STATUS_LOCK_ERROR;
            } else {
                blocks.erase(pkt);
            }
        } else {
            statusRegister |= STATUS_ERASE_ERROR;
        }
        writeState = CfiCommand::NO_CMD;
        readState = CfiCommand::READ_STATUS_REG;
        break;

    case CfiCommand::LOCK_BLOCK_SETUP:
        if (new_cmd == CfiCommand::LOCK_BLOCK) {
            // Lock the addressed block
            blocks.lock(flash_address);
            readState = CfiCommand::READ_STATUS_REG;

        } else if (new_cmd == CfiCommand::UNLOCK_BLOCK) {
            // Unlock the addressed block
            blocks.unlock(flash_address);
            readState = CfiCommand::READ_STATUS_REG;

        } else {
            statusRegister |= STATUS_ERASE_ERROR;
        }

        writeState = CfiCommand::NO_CMD;
        break;

    case CfiCommand::WORD_PROGRAM:
        readState = CfiCommand::READ_STATUS_REG;
        writeState = CfiCommand::NO_CMD;

        if (blocks.isLocked(flash_address)) {
            statusRegister |= STATUS_LOCK_ERROR;
        } else {
            AbstractMemory::access(pkt);
            return;
        }
        break;

    case CfiCommand::BUFFERED_PROGRAM_SETUP: {
        // Buffer size in bytes
        auto buffer_size = (value + 1) * sizeof(uint32_t);

        // Clearing the program buffer
        programBuffer.setup(buffer_size);

        readState = CfiCommand::READ_STATUS_REG;
        writeState = CfiCommand::BUFFER_SIZE_READ;
        break;
    }

    case CfiCommand::BUFFER_SIZE_READ: {
        // Write to the buffer and check if a writeback is needed
        // (if the buffer is full)
        auto writeback = programBuffer.write(
            flash_address, pkt->getPtr<void>(), pkt->getSize());

        if (writeback) {
            if (new_cmd == CfiCommand::BUFFERED_PROGRAM_CONFIRM) {
                auto success = programBuffer.writeback();
                if (!success)
                    statusRegister |= STATUS_LOCK_ERROR;

                readState = CfiCommand::READ_STATUS_REG;
            } else {
                statusRegister |= STATUS_PROGRAM_LOCK_BIT;
            }
            writeState = CfiCommand::NO_CMD;
        }
        break;
    }

    default:
        panic("Invalid Write State\n");
        return;
    }

    pkt->makeResponse();
}

void
CfiMemory::read(PacketPtr pkt)
{
    const Addr flash_address = pkt->getAddr() - start();
    uint64_t value = 0;

    switch (readState) {
    case CfiCommand::READ_STATUS_REG:
        value = statusRegister;
        break;
    case CfiCommand::READ_DEVICE_ID:
        value = readDeviceID(flash_address);
        break;
    case CfiCommand::READ_CFI_QUERY:
        value = cfiQuery(flash_address);
        break;
    case CfiCommand::READ_ARRAY:
        AbstractMemory::access(pkt);
        return;
    default:
        panic("Invalid Read State\n");
        return;
    }

    if (numberOfChips == 2) {
        value |= (value << 16);
    }

    pkt->setUintX(value, ByteOrder::little);
    pkt->makeResponse();

    DPRINTF(CFI, "read, address: %#x, val: %#x\n", pkt->getAddr(),
            pkt->getUintX(ByteOrder::little));
}

uint64_t
CfiMemory::readDeviceID(Addr flash_address) const
{
    switch ((flash_address & 0xff) / bankWidth) {
    case 0x00: // vendor ID
        return vendorID;
    case 0x01: // device ID
        return deviceID;
    case 0x02: // lock bit
        return blocks.isLocked(flash_address);
    default:
        // Unsupported entries
        warn("Invalid Device Identifier code: %d\n", flash_address & 0xff);
        return 0;
    }
}

void
CfiMemory::handleCommand(CfiCommand new_cmd)
{
    switch (new_cmd) {
    case CfiCommand::READ_ARRAY:
        DPRINTF(CFI, "CFI Command: Read Array\n");
        readState = CfiCommand::READ_ARRAY;
        break;
    case CfiCommand::READ_DEVICE_ID:
        DPRINTF(CFI, "CFI Command: Read Device Identifier\n");
        readState = CfiCommand::READ_DEVICE_ID;
        break;
    case CfiCommand::READ_CFI_QUERY:
        DPRINTF(CFI, "CFI Command: CFI Query\n");
        readState = CfiCommand::READ_CFI_QUERY;
        break;
    case CfiCommand::READ_STATUS_REG:
        DPRINTF(CFI, "CFI Command: Read Status Register\n");
        readState = CfiCommand::READ_STATUS_REG;
        break;
    case CfiCommand::CLEAR_STATUS_REG:
        DPRINTF(CFI, "CFI Command: Clear Status Register\n");
        statusRegister = STATUS_READY;
        break;
    case CfiCommand::BUFFERED_PROGRAM_CONFIRM:
        DPRINTF(CFI, "CFI Command: Buffered Program Confirm\n");
        break;
    case CfiCommand::ERASE_BLOCK_SETUP:
        DPRINTF(CFI, "CFI Command: Erase Block Setup\n");
        writeState = CfiCommand::ERASE_BLOCK_SETUP;
        readState = CfiCommand::READ_STATUS_REG;
        break;
    case CfiCommand::LOCK_BLOCK_SETUP:
        DPRINTF(CFI, "CFI Command: Lock Block Setup\n");
        writeState = CfiCommand::LOCK_BLOCK_SETUP;
        break;
    case CfiCommand::WORD_PROGRAM:
        DPRINTF(CFI, "CFI Command: Word Program\n");
        writeState = CfiCommand::WORD_PROGRAM;
        readState = CfiCommand::READ_STATUS_REG;
        break;
    case CfiCommand::BUFFERED_PROGRAM_SETUP:
        DPRINTF(CFI, "CFI Command: Buffered Program Setup\n");
        writeState = CfiCommand::BUFFERED_PROGRAM_SETUP;
        readState = CfiCommand::READ_STATUS_REG;
        break;
    case CfiCommand::AMD_RESET:
        // because of how u-boot works and reset the flash
        // we have to ignore the AMD RESET explicitly
        //  (see the function __flash_cmd_reset in drivers/mtd/cfi_flash.c)
        break;
    default:
        panic("Don't know what to do with %#x\n",
              static_cast<uint16_t>(new_cmd));
    }
}

uint64_t
CfiMemory::cfiQuery(Addr flash_address)
{
    flash_address /= bankWidth;

    panic_if(flash_address >= sizeof(cfiQueryTable),
             "Acessing invalid entry in CFI query table (addr=%#x)",
             flash_address);

    return cfiQueryTable[flash_address];
}

void
CfiMemory::BlockData::erase(PacketPtr pkt)
{
    auto host_address = parent.toHostAddr(pkt->getAddr());
    std::memset(host_address, 0xff, blockSize);
}

} // namespace memory
} // namespace gem5

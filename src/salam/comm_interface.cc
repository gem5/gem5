/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "salam/comm_interface.hh"

#include <cstdio>
#include <cstdlib>
#include <iomanip>

#include "base/trace.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

using namespace std;

/*****************************************************************************
 * CommInterface serves as the general system interface for hardware
 * accelerators. It provides a set of memory-mapped registers, as well as
 * master ports for accessing both local busses/SPMs and system memory.
 ****************************************************************************/
CommInterface::CommInterface(const CommInterfaceParams &p)
    : BasicPioDevice(p, p.pio_size),
      io_addr(p.pio_addr),
      io_size(p.pio_size),
      flag_size(p.flags_size),
      config_size(p.config_size),
      devname(p.devicename),
      gic(p.gic),
      int_num(p.int_num),
      use_premap_data(p.premap_data),
      endian(p.system->getGuestByteOrder()),
      debugEnabled(p.enable_debug_msgs),
      masterId(p.system->getRequestorId(this, name())),
      tickEvent(this),
      cacheLineSize(p.cache_line_size),
      clock_period(p.clock_period),
      reset_spm(p.reset_spm)
{
    processDelay = 1000 * clock_period;
    FLAG_OFFSET = 0;
    CONFIG_OFFSET = flag_size;
    VAR_OFFSET = CONFIG_OFFSET + config_size;
    processingDone = false;
    computationNeeded = false;
    int_flag = false;

    mmreg = new uint8_t[io_size];
    for (int i = 0; i < io_size; i++) {
        mmreg[i] = 0;
    }
    cu = nullptr;

    if (use_premap_data) {
        for (auto i = 0; i < p.data_bases.size(); i++) {
            data_base_ptrs.push_back(p.data_bases[i]);
        }
    }
}

bool
CommInterface::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    owner->recvPacket(pkt);
    return true;
}

void
CommInterface::MemSidePort::recvReqRetry()
{
    assert(outstandingPkts.size());

    if (debug()) {
        DPRINTF(CommInterface, "Got a retry...\n");
    }
    while (outstandingPkts.size() && sendTimingReq(outstandingPkts.front())) {
        if (debug()) {
            DPRINTF(CommInterface, "Unblocked, sent blocked packet.\n");
        }
        outstandingPkts.pop();
        // TODO: This should just signal the engine that the packet completed
        // engine should schedule tick as necessary. Need a test case
        if (!owner->tickEvent.scheduled()) {
            owner->schedule(owner->tickEvent, curTick() + owner->processDelay);
            // owner->schedule(owner->tickEvent, owner->nextCycle());
        }
    }
}

void
CommInterface::MemSidePort::sendPacket(PacketPtr pkt)
{
    if (isStalled() || !sendTimingReq(pkt)) {
        if (debug()) {
            DPRINTF(
                CommInterface,
                "sendTiming failed in sendPacket(pkt->req->getPaddr()=0x%x)\n",
                (unsigned int)pkt->req->getPaddr());
        }
        setStalled(pkt);
    }
}

bool
CommInterface::SPMPort::recvTimingResp(PacketPtr pkt)
{
    owner->recvPacket(pkt);
    return true;
}

void
CommInterface::SPMPort::recvReqRetry()
{
    assert(outstandingPkts.size());

    if (debug()) {
        DPRINTF(CommInterface, "Got a retry...\n");
    }
    while (outstandingPkts.size() && sendTimingReq(outstandingPkts.front())) {
        if (debug()) {
            DPRINTF(CommInterface, "Unblocked, sent blocked packet.\n");
        }
        outstandingPkts.pop();
        // TODO: This should just signal the engine that the packet completed
        // engine should schedule tick as necessary. Need a test case
        if (!owner->tickEvent.scheduled()) {
            owner->schedule(owner->tickEvent, curTick() + owner->processDelay);
            // owner->schedule(owner->tickEvent, owner->nextCycle());
        }
    }
}

void
CommInterface::SPMPort::sendPacket(PacketPtr pkt)
{
    if (isStalled() || !sendTimingReq(pkt)) {
        if (debug()) {
            DPRINTF(
                CommInterface,
                "sendTiming failed in sendPacket(pkt->req->getPaddr()=0x%x)\n",
                (unsigned int)pkt->req->getPaddr());
        }
        setStalled(pkt);
    }
}

bool
CommInterface::RegPort::recvTimingResp(PacketPtr pkt)
{
    owner->recvPacket(pkt);
    return true;
}

void
CommInterface::RegPort::recvReqRetry()
{
    panic("We should not be receiving retries from Register Banks.\n");
}

void
CommInterface::RegPort::sendPacket(PacketPtr pkt)
{
    sendTimingReq(pkt);
}

void
CommInterface::recvPacket(PacketPtr pkt)
{
    if (pkt->isRead()) {
        MemoryRequest *readReq = findMemRequest(pkt, true);
        RequestPort *carrier = readReq->getCarrierPort();
        if (MemSidePort *port = dynamic_cast<MemSidePort *>(carrier)) {
            port->readReq = nullptr;
        }
        if (SPMPort *port = dynamic_cast<SPMPort *>(carrier)) {
            port->readReq = nullptr;
        }
        if (debug()) {
            DPRINTF(CommInterface, "Done with a read. addr: 0x%x, size: %d\n",
                    pkt->req->getPaddr(), pkt->getSize());
        }
        pkt->writeData(readReq->buffer +
                       (pkt->req->getPaddr() - readReq->beginAddr));
        if (debug()) {
            DPRINTF(CommInterface, "Read:%s\n", readReq->printBuffer());
        }
        for (int i = pkt->req->getPaddr() - readReq->beginAddr;
             i < pkt->req->getPaddr() - readReq->beginAddr + pkt->getSize();
             i++) {
            readReq->readsDone[i] = true;
        }

        // mark readDone as only the contiguous region
        while (readReq->readDone < readReq->totalLength &&
               readReq->readsDone[readReq->readDone]) {
            readReq->readDone++;
        }

        if (!readReq->needToRead) {
            if (debug()) {
                DPRINTF(CommInterface, "Done reading \n");
            }
            cu->readCommit(readReq);
            if (debug()) {
                DPRINTF(CommInterface, "Clearing Request \n");
            }
            clearMemRequest(readReq, true);
            delete readReq;
        } else {
            readQueue.push_front(readReq);
            // Clear the request from the in-flight queue
            clearMemRequest(readReq, true);
        }
    } else if (pkt->isWrite()) {
        MemoryRequest *writeReq = findMemRequest(pkt, false);
        RequestPort *carrier = writeReq->getCarrierPort();
        if (MemSidePort *port = dynamic_cast<MemSidePort *>(carrier)) {
            port->writeReq = nullptr;
        }
        if (SPMPort *port = dynamic_cast<SPMPort *>(carrier)) {
            port->writeReq = nullptr;
        }
        if (debug()) {
            DPRINTF(CommInterface, "Done with a write. addr: 0x%x, size: %d\n",
                    pkt->req->getPaddr(), pkt->getSize());
        }
        writeReq->writeDone += pkt->getSize();
        if (!(writeReq->needToWrite)) {
            if (debug()) {
                DPRINTF(CommInterface, "Done writing\n");
            }
            cu->writeCommit(writeReq);
            clearMemRequest(writeReq, false);
            delete writeReq;
        } else {
            writeQueue.push_front(writeReq);
            // Clear the request from the in-flight queue
            clearMemRequest(writeReq, false);
        }
    } else {
        panic("Something went very wrong!");
    }
    if (!tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + processDelay);
    }
    delete pkt;
}

void
CommInterface::checkMMR()
{
    if (!computationNeeded) {
        if (debug()) {
            DPRINTF(CommInterface, "Checking MMR to see if Run bit set\n");
        }
        if (*mmreg & 0x01) {
            *mmreg &= 0xfe;
            *mmreg |= 0x02;
            computationNeeded = true;
            cu->initialize();
        }

        if (processingDone && !tickEvent.scheduled()) {
            processingDone = false;
            schedule(tickEvent, curTick() + processDelay);
        }
    }
}

bool
CommInterface::inStreamRange(Addr add)
{
    for (auto port : streamPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add)) {
                return true;
            }
        }
    }
    return false;
}

bool
CommInterface::inSPMRange(Addr add)
{
    for (auto port : spmPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add)) {
                return true;
            }
        }
    }
    return false;
}

bool
CommInterface::inRegRange(Addr add)
{
    for (auto port : regPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add)) {
                return true;
            }
        }
    }
    return false;
}

bool
CommInterface::inLocalRange(Addr add)
{
    for (auto port : localPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add)) {
                return true;
            }
        }
    }
    return false;
}

bool
CommInterface::inGlobalRange(Addr add)
{
    for (auto port : globalPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add)) {
                return true;
            }
        }
    }
    return false;
}

CommInterface::MemSidePort *
CommInterface::getValidLocalPort(Addr add, bool read)
{
    for (auto port : localPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add) && !(port->isStalled())) {
                if ((read && !(port->readReq)) ||
                    (!read && !(port->writeReq))) {
                    return port;
                }
            }
        }
    }
    return nullptr;
}

CommInterface::MemSidePort *
CommInterface::getValidGlobalPort(Addr add, bool read)
{
    for (auto port : globalPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add) && !(port->isStalled())) {
                if ((read && !(port->readReq)) ||
                    (!read && !(port->writeReq))) {
                    return port;
                }
            }
        }
    }
    return nullptr;
}

CommInterface::MemSidePort *
CommInterface::getValidStreamPort(Addr add, size_t len, bool read)
{
    for (auto port : streamPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add) && !(port->isStalled())) {
                if (((read && !(port->readReq)) ||
                     (!read && !(port->writeReq))) &&
                    port->streamValid(len, read)) {
                    return port;
                }
            }
        }
    }
    return nullptr;
}

CommInterface::SPMPort *
CommInterface::getValidSPMPort(Addr add, size_t len, bool read)
{
    for (auto port : spmPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add) && !(port->isStalled())) {
                if (((read && !(port->readReq)) ||
                     (!read && !(port->writeReq)))) {
                    if (port->canAccess(add, len, read)) {
                        return port;
                    }
                }
            }
        }
    }
    return nullptr;
}

CommInterface::RegPort *
CommInterface::getValidRegPort(Addr add)
{
    for (auto port : regPorts) {
        AddrRangeList adl = port->getAddrRanges();
        for (auto address : adl) {
            if (address.contains(add)) {
                return port;
            }
        }
    }
    return nullptr;
}

void
CommInterface::processMemoryRequests()
{
    if (!allPortsStalled()) {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Checking read requests. %d requests in queue.\n",
                    readQueue.size());
        }
        for (auto it = readQueue.begin(); it != readQueue.end();) {
            Addr address = (*it)->currentReadAddr;
            if (debug()) {
                DPRINTF(CommInterfaceQueues, "Request Address: %lx\n",
                        address);
            }
            RequestPort *mport;
            if (inStreamRange(address)) {
                mport = getValidStreamPort(address, (*it)->readLeft, true);
            } else if (inSPMRange(address)) {
                mport = getValidSPMPort(address, (*it)->readLeft, true);
            } else if (inLocalRange(address)) {
                mport = getValidLocalPort(address, true);
            } else if (inGlobalRange(address)) {
                mport = getValidGlobalPort(address, true);
            } else {
                panic("Address %lx is not reachable by any ports\n", address);
            }
            if (SPMPort *port = dynamic_cast<SPMPort *>(mport)) {
                if (debug()) {
                    DPRINTF(CommInterfaceQueues,
                            "Found available memory port\n");
                }
                port->readReq = (*it);
                port->readReq->setCarrierPort(port);
                it = readQueue.erase(it);
                if (port->readReq && port->readReq->needToRead) {
                    if (debug()) {
                        DPRINTF(CommInterfaceQueues,
                                "Trying read on available memory port\n");
                    }
                    tryRead(port);
                    accRdQ.push_back(port->readReq);
                }
            } else if (MemSidePort *port =
                           dynamic_cast<MemSidePort *>(mport)) {
                if (debug()) {
                    DPRINTF(CommInterfaceQueues,
                            "Found available memory port\n");
                }
                port->readReq = (*it);
                port->readReq->setCarrierPort(port);
                it = readQueue.erase(it);
                if (port->readReq && port->readReq->needToRead) {
                    if (debug()) {
                        DPRINTF(CommInterfaceQueues,
                                "Trying read on available memory port\n");
                    }
                    tryRead(port);
                    accRdQ.push_back(port->readReq);
                }
            } else {
                if (debug()) {
                    DPRINTF(CommInterfaceQueues,
                            "Found no ports able to read %d bytes from %lx\n",
                            (*it)->length, address);
                }
                ++it;
            }
        }
        if (debug()) {
            DPRINTF(CommInterface,
                    "Checking write requests. %d requests in queue.\n",
                    writeQueue.size());
        }
        for (auto it = writeQueue.begin(); it != writeQueue.end();) {
            Addr address = (*it)->currentWriteAddr;
            if (debug()) {
                DPRINTF(CommInterfaceQueues, "Request Address: %lx\n",
                        address);
            }
            RequestPort *mport;
            if (inStreamRange(address)) {
                mport = getValidStreamPort(address, (*it)->writeLeft, false);
            } else if (inSPMRange(address)) {
                mport = getValidSPMPort(address, (*it)->writeLeft, false);
            } else if (inLocalRange(address)) {
                mport = getValidLocalPort(address, false);
            } else if (inGlobalRange(address)) {
                mport = getValidGlobalPort(address, false);
            } else {
                panic("Address %lx is not reachable by any ports\n", address);
            }
            if (SPMPort *port = dynamic_cast<SPMPort *>(mport)) {
                if (debug()) {
                    DPRINTF(CommInterfaceQueues,
                            "Found available memory port\n");
                }
                port->writeReq = (*it);
                port->writeReq->setCarrierPort(port);
                it = writeQueue.erase(it);
                if (port->writeReq && port->writeReq->needToWrite) {
                    if (debug()) {
                        DPRINTF(CommInterfaceQueues,
                                "Trying write on available memory port\n");
                    }
                    tryWrite(port);
                    accWrQ.push_back(port->writeReq);
                }
            } else if (MemSidePort *port =
                           dynamic_cast<MemSidePort *>(mport)) {
                if (debug()) {
                    DPRINTF(CommInterfaceQueues,
                            "Found available memory port\n");
                }
                port->writeReq = (*it);
                port->writeReq->setCarrierPort(port);
                it = writeQueue.erase(it);
                if (port->writeReq && port->writeReq->needToWrite) {
                    if (debug()) {
                        DPRINTF(CommInterfaceQueues,
                                "Trying write on available memory port\n");
                    }
                    tryWrite(port);
                    accWrQ.push_back(port->writeReq);
                }
            } else {
                if (debug()) {
                    DPRINTF(CommInterfaceQueues,
                            "Found no ports able to write %d bytes to %lx\n",
                            (*it)->length, address);
                }
                ++it;
            }
        }
    } else {
        if (debug()) {
            DPRINTF(CommInterface, "All ports are stalled\n");
        }
    }
    requestsInQueues = readQueue.size() + writeQueue.size();
    if (!tickEvent.scheduled() && requestsInQueues > 0) {
        schedule(tickEvent, curTick() + processDelay);
    }
}

void
CommInterface::tick()
{
    if (debug()) {
        DPRINTF(CommInterface, "Tick!\n");
    }
    checkMMR();
    requestsInQueues = readQueue.size() + writeQueue.size();
    if (requestsInQueues > 0) {
        processMemoryRequests();
    }
}

void
CommInterface::tryRead(MemSidePort *port)
{
    MemoryRequest *readReq = port->readReq;
    Request::Flags flags;
    if (readReq->readLeft <= 0) {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Something went wrong."
                    "Shouldn't try to read if there aren't reads left\n");
        }
        return;
    }
    int size;
    if (readReq->currentReadAddr % cacheLineSize) {
        size = cacheLineSize - (readReq->currentReadAddr % cacheLineSize);
        if (debug()) {
            DPRINTF(CommInterface, "Aligning\n");
        }
    } else {
        size = cacheLineSize;
    }
    size = readReq->readLeft > (size - 1) ? size : readReq->readLeft;
    RequestPtr req =
        make_shared<Request>(readReq->currentReadAddr, size, flags, masterId);
    if (debug()) {
        DPRINTF(CommInterface,
                "Trying to read addr: 0x%016x, %d bytes through port: %s\n",
                req->getPaddr(), size, port->name());
    }

    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();
    readReq->pkt = pkt;
    port->sendPacket(pkt);

    readReq->currentReadAddr += size;

    readReq->readLeft -= size;

    if (!(readReq->readLeft > 0)) {
        readReq->needToRead = false;
        if (!tickEvent.scheduled()) {
            schedule(tickEvent, curTick() + processDelay);
        }
    } else {
        if (!port->isStalled() && !tickEvent.scheduled()) {
            schedule(tickEvent, curTick() + processDelay);
        }
    }
}

void
CommInterface::tryWrite(MemSidePort *port)
{
    MemoryRequest *writeReq = port->writeReq;
    if (writeReq->writeLeft <= 0) {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Something went wrong."
                    "Shouldn't try to write if there aren't writes left\n");
        }
        return;
    }

    int size;
    if (writeReq->currentWriteAddr % cacheLineSize) {
        size = cacheLineSize - (writeReq->currentWriteAddr % cacheLineSize);
        if (debug()) {
            DPRINTF(CommInterface, "Aligning\n");
        }
    } else {
        size = cacheLineSize;
    }
    size = writeReq->writeLeft > size - 1 ? size : writeReq->writeLeft;

    Request::Flags flags;
    uint8_t *data = new uint8_t[size];
    std::memcpy(
        data, &(writeReq->buffer[writeReq->totalLength - writeReq->writeLeft]),
        size);
    RequestPtr req = make_shared<Request>(writeReq->currentWriteAddr, size,
                                          flags, masterId);
    req->setExtraData((uint64_t)data);

    if (debug()) {
        DPRINTF(CommInterface, "totalLength: %d, writeLeft: %d\n",
                writeReq->totalLength, writeReq->writeLeft);
    }
    if (debug()) {
        DPRINTF(CommInterface,
                "Trying to write to addr: 0x%016x, %d bytes, data 0x%08x"
                "through port: %s\n",
                writeReq->currentWriteAddr, size,
                *((uint64_t *)(&(writeReq->buffer[writeReq->totalLength -
                                                  writeReq->writeLeft]))),
                port->name());
    }

    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    uint8_t *pkt_data = (uint8_t *)req->getExtraData();
    pkt->dataDynamic(pkt_data);
    writeReq->pkt = pkt;
    port->sendPacket(pkt);

    writeReq->currentWriteAddr += size;
    writeReq->writeLeft -= size;

    if (!(writeReq->writeLeft > 0)) {
        writeReq->needToWrite = false;
        if (!tickEvent.scheduled()) {
            schedule(tickEvent, curTick() + processDelay);
        }
    } else if (!port->isStalled() && !tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + processDelay);
    }
}

void
CommInterface::tryRead(SPMPort *port)
{
    MemoryRequest *readReq = port->readReq;
    Request::Flags flags;
    if (readReq->readLeft <= 0) {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Something went wrong."
                    "Shouldn't try to read if there aren't reads left\n");
        }
        return;
    }
    int size;
    if (readReq->currentReadAddr % cacheLineSize) {
        size = cacheLineSize - (readReq->currentReadAddr % cacheLineSize);
        if (debug()) {
            DPRINTF(CommInterface, "Aligning\n");
        }
    } else {
        size = cacheLineSize;
    }
    size = readReq->readLeft > (size - 1) ? size : readReq->readLeft;
    RequestPtr req =
        make_shared<Request>(readReq->currentReadAddr, size, flags, masterId);
    if (debug()) {
        DPRINTF(CommInterface,
                "Trying to read addr: 0x%016x, %d bytes through port: %s\n",
                req->getPaddr(), size, port->name());
    }

    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();
    readReq->pkt = pkt;
    port->sendPacket(pkt);

    readReq->currentReadAddr += size;

    readReq->readLeft -= size;

    if (!(readReq->readLeft > 0)) {
        readReq->needToRead = false;
        if (!tickEvent.scheduled()) {
            schedule(tickEvent, curTick() + processDelay);
        }
    } else {
        if (!port->isStalled() && !tickEvent.scheduled()) {
            schedule(tickEvent, curTick() + processDelay);
        }
    }
}

void
CommInterface::tryWrite(SPMPort *port)
{
    MemoryRequest *writeReq = port->writeReq;
    if (writeReq->writeLeft <= 0) {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Something went wrong."
                    "Shouldn't try to write if there aren't writes left\n");
        }
        return;
    }

    int size;
    if (writeReq->currentWriteAddr % cacheLineSize) {
        size = cacheLineSize - (writeReq->currentWriteAddr % cacheLineSize);
        if (debug()) {
            DPRINTF(CommInterface, "Aligning\n");
        }
    } else {
        size = cacheLineSize;
    }
    size = writeReq->writeLeft > size - 1 ? size : writeReq->writeLeft;

    Request::Flags flags;
    uint8_t *data = new uint8_t[size];
    std::memcpy(
        data, &(writeReq->buffer[writeReq->totalLength - writeReq->writeLeft]),
        size);
    RequestPtr req = make_shared<Request>(writeReq->currentWriteAddr, size,
                                          flags, masterId);
    req->setExtraData((uint64_t)data);

    if (debug()) {
        DPRINTF(CommInterface, "totalLength: %d, writeLeft: %d\n",
                writeReq->totalLength, writeReq->writeLeft);
    }
    if (debug()) {
        DPRINTF(CommInterface,
                "Trying to write to addr: 0x%016x, %d bytes,"
                " data 0x%08x through port: %s\n",
                writeReq->currentWriteAddr, size,
                *((uint64_t *)(&(writeReq->buffer[writeReq->totalLength -
                                                  writeReq->writeLeft]))),
                port->name());
    }

    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    uint8_t *pkt_data = (uint8_t *)req->getExtraData();
    pkt->dataDynamic(pkt_data);
    writeReq->pkt = pkt;
    port->sendPacket(pkt);

    writeReq->currentWriteAddr += size;
    writeReq->writeLeft -= size;

    if (!(writeReq->writeLeft > 0)) {
        writeReq->needToWrite = false;
        if (!tickEvent.scheduled()) {
            schedule(tickEvent, curTick() + processDelay);
        }
    } else if (!port->isStalled() && !tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + processDelay);
    }
}

void
CommInterface::tryRead(RegPort *port)
{
    MemoryRequest *readReq = port->readReq;
    Request::Flags flags;
    if (readReq->readLeft <= 0) {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Something went wrong."
                    "Shouldn't try to read if there aren't reads left\n");
        }
        return;
    }
    int size = readReq->readLeft;
    RequestPtr req =
        make_shared<Request>(readReq->currentReadAddr, size, flags, masterId);
    if (debug()) {
        DPRINTF(CommInterface,
                "Trying to read addr: 0x%016x, "
                "%d bytes through port: %s\n",
                req->getPaddr(), size, port->name());
    }
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();
    readReq->pkt = pkt;
    readReq->currentReadAddr += size;
    readReq->readLeft -= size;
    if (readReq->readLeft <= 0) {
        readReq->needToRead = false;
    }
    port->sendPacket(pkt);

    if (!(readReq->readLeft > 0)) {
        if (!tickEvent.scheduled()) {
            schedule(tickEvent, curTick() + processDelay);
        }
    }
}

void
CommInterface::tryWrite(RegPort *port)
{
    MemoryRequest *writeReq = port->writeReq;
    if (writeReq->writeLeft <= 0) {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Something went wrong."
                    "Shouldn't try to write if there aren't writes left\n");
        }
        return;
    }

    int size = writeReq->writeLeft;

    Request::Flags flags;
    uint8_t *data = new uint8_t[size];
    std::memcpy(
        data, &(writeReq->buffer[writeReq->totalLength - writeReq->writeLeft]),
        size);
    RequestPtr req = make_shared<Request>(writeReq->currentWriteAddr, size,
                                          flags, masterId);
    req->setExtraData((uint64_t)data);

    if (debug()) {
        DPRINTF(CommInterface, "totalLength: %d, writeLeft: %d\n",
                writeReq->totalLength, writeReq->writeLeft);
    }
    if (debug()) {
        DPRINTF(CommInterface,
                "Trying to write to addr: 0x%016x, %d bytes, "
                "data 0x%08x through port: %s\n",
                writeReq->currentWriteAddr, size,
                *((uint64_t *)(&(writeReq->buffer[writeReq->totalLength -
                                                  writeReq->writeLeft]))),
                port->name());
    }

    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    uint8_t *pkt_data = (uint8_t *)req->getExtraData();
    pkt->dataDynamic(pkt_data);
    writeReq->pkt = pkt;
    writeReq->currentWriteAddr += size;
    writeReq->writeLeft -= size;
    if (writeReq->writeLeft <= 0) {
        writeReq->needToWrite = false;
    }
    port->sendPacket(pkt);

    if (!(writeReq->writeLeft > 0)) {
        if (!tickEvent.scheduled()) {
            schedule(tickEvent, curTick() + processDelay);
        }
    }
}

void
CommInterface::enqueueRead(MemoryRequest *req)
{
    if (inRegRange(req->getAddress())) {
        // We want to immediately handle register requests
        // and bypass memory queues
        auto regport = getValidRegPort(req->getAddress());
        accRdQ.push_back(req); // Add it to active read queue
        regport->setReadReq(req);
        req->setCarrierPort(regport);
        tryRead(regport);
    } else {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Read from 0x%lx of Size:%d Bytes Enqueued:\n",
                    req->address, req->length);
        }
        readQueue.push_back(req);
        if (debug()) {
            DPRINTF(CommInterfaceQueues, "Current Queue:\n");
            for (auto it = readQueue.begin(); it != readQueue.end(); ++it) {
                DPRINTF(CommInterfaceQueues, "Read Request: %lx\n",
                        (*it)->address);
            }
        }
    }
    if (!tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + processDelay);
    }
}

void
CommInterface::enqueueWrite(MemoryRequest *req)
{
    if (inRegRange(req->getAddress())) {
        // We want to immediately handle register requests
        // and bypass memory queues
        auto regport = getValidRegPort(req->getAddress());
        accWrQ.push_back(req); // Add it to active write queue
        regport->setWriteReq(req);
        req->setCarrierPort(regport);
        tryWrite(regport);
    } else {
        if (debug()) {
            DPRINTF(CommInterface,
                    "Write to 0x%lx of size:%d bytes "
                    "enqueued\n",
                    req->address, req->length);
        }
        writeQueue.push_back(req);
        if (debug()) {
            DPRINTF(CommInterfaceQueues, "Current Queue:\n");
            for (auto it = writeQueue.begin(); it != writeQueue.end(); ++it) {
                DPRINTF(CommInterfaceQueues, "Write Request: %lx\n",
                        (*it)->address);
            }
        }
    }
    if (!tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + processDelay);
    }
}

void
CommInterface::finish()
{
    *mmreg &= 0xfc;
    *mmreg |= 0x04;
    computationNeeded = false;
    if (int_num > 0) {
        int_flag = true;
        gic->sendInt(int_num);
    }
    if (reset_spm) {
        for (auto port : spmPorts) {
            port->setReadyStatus(false);
        }
    }
}

Tick
CommInterface::read(PacketPtr pkt)
{
    if (debug()) {
        DPRINTF(DeviceMMR,
                "The address range associated with this ACC was read!\n");
    }

    Addr offset = pkt->req->getPaddr() - io_addr;

    uint64_t data;

    data = *(uint64_t *)(mmreg + offset);

    switch (pkt->getSize()) {
        case 1:
            pkt->set<uint8_t>(data, endian);
            break;
        case 2:
            pkt->set<uint16_t>(data, endian);
            break;
        case 4:
            pkt->set<uint32_t>(data, endian);
            break;
        case 8:
            pkt->set<uint64_t>(data, endian);
            break;
        default:
            panic("Read size too big?\n");
            break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
CommInterface::write(PacketPtr pkt)
{
    if (debug()) {
        DPRINTF(
            DeviceMMR,
            "The address range associated with this ACC was written to!\n");
    }

    if (debug()) {
        DPRINTF(DeviceMMR, "Packet addr 0x%lx\n", pkt->req->getPaddr());
    }
    if (debug()) {
        DPRINTF(DeviceMMR, "IO addr 0x%lx\n", io_addr);
    }
    if (debug()) {
        DPRINTF(DeviceMMR, "Diff addr 0x%lx\n",
                pkt->req->getPaddr() - io_addr);
    }
    if (debug()) {
        DPRINTF(DeviceMMR, "Packet val (LE) %d\n", pkt->getLE<uint8_t>());
    }
    if (debug()) {
        DPRINTF(DeviceMMR, "Packet val (BE) %d\n", pkt->getBE<uint8_t>());
    }
    if (debug()) {
        DPRINTF(DeviceMMR, "Packet val %d\n", pkt->get<uint8_t>(endian));
    }
    pkt->writeData(mmreg + (pkt->req->getPaddr() - io_addr));

    std::stringstream mm;
    for (int i = io_size - 1; i >= 0; i--) {
        if ((i >= flag_size + config_size) &&
            ((i - flag_size - config_size) % 8 == 0)) {
            mm << std::setfill('0') << std::setw(2) << std::hex
               << (uint32_t)mmreg[i] << "|";
        } else if (i == flag_size + config_size) {
            mm << "|" << std::setfill('0') << std::setw(2) << std::hex
               << (uint32_t)mmreg[i];
        } else if (i == flag_size) {
            mm << "|" << std::setfill('0') << std::setw(2) << std::hex
               << (uint32_t)mmreg[i];
        } else {
            mm << std::setfill('0') << std::setw(2) << std::hex
               << (uint32_t)mmreg[i];
        }
    }
    std::string mmr = mm.str();
    if (debug()) {
        DPRINTF(DeviceMMR, "MMReg value: %s\n", mmr);
    }

    pkt->makeAtomicResponse();

    if (((*mmreg & 0x04) == 0x00) && int_flag) {
        if (int_num > 0) {
            gic->clearInt(int_num);
        }
        int_flag = false;
    }
    if (!tickEvent.scheduled()) {
        schedule(tickEvent, nextCycle());
    }

    return pioDelay;
}

uint64_t
CommInterface::getGlobalVar(unsigned offset, unsigned size)
{
    if (use_premap_data) {
        return data_base_ptrs.at(offset / 8);
    } else {
        uint64_t value;
        switch (size) {
            case 1:
                value = *(uint64_t *)(uint8_t *)(mmreg + VAR_OFFSET + offset);
                break;
            case 2:
                value = *(uint64_t *)(uint16_t *)(mmreg + VAR_OFFSET + offset);
                break;
            case 4:
                value = *(uint64_t *)(uint32_t *)(mmreg + VAR_OFFSET + offset);
                break;
            case 8:
                value = *(uint64_t *)(mmreg + VAR_OFFSET + offset);
                break;
            default:
                panic("Data size: %d is not supported as a global variable!");
        }
        return value;
    }
}

Port &
CommInterface::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "local") {
        if (idx >= localPorts.size()) {
            localPorts.resize((idx + 1));
        }
        if (localPorts[idx] == nullptr) {
            const std::string portName =
                name() + ".local[" + std::to_string(idx) + "]";
            localPorts[idx] = new MemSidePort(portName, this, idx);
        }
        return *localPorts[idx];
    } else if (if_name == "acp") {
        if (idx >= globalPorts.size()) {
            globalPorts.resize((idx + 1));
        }
        if (globalPorts[idx] == nullptr) {
            const std::string portName =
                name() + ".acp[" + std::to_string(idx) + "]";
            globalPorts[idx] = new MemSidePort(portName, this, idx);
        }
        return *globalPorts[idx];
    } else if (if_name == "stream") {
        if (idx >= streamPorts.size()) {
            streamPorts.resize((idx + 1));
        }
        if (streamPorts[idx] == nullptr) {
            const std::string portName =
                name() + ".stream[" + std::to_string(idx) + "]";
            streamPorts[idx] = new MemSidePort(portName, this, idx);
        }
        return *streamPorts[idx];
    } else if (if_name == "spm") {
        if (idx >= spmPorts.size()) {
            spmPorts.resize((idx + 1));
        }
        if (spmPorts[idx] == nullptr) {
            const std::string portName =
                name() + ".spm[" + std::to_string(idx) + "]";
            spmPorts[idx] = new SPMPort(portName, this, idx);
        }
        return *spmPorts[idx];
    } else if (if_name == "reg") {
        if (idx >= regPorts.size()) {
            regPorts.resize((idx + 1));
        }
        if (regPorts[idx] == nullptr) {
            const std::string portName =
                name() + ".reg[" + std::to_string(idx) + "]";
            regPorts[idx] = new RegPort(portName, this, idx);
        }
        return *regPorts[idx];
    } else if (if_name == "pio") {
        return pioPort;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

MemoryRequest *
CommInterface::findMemRequest(PacketPtr pkt, bool isRead)
{
    if (isRead) {
        for (auto it = accRdQ.begin(); it != accRdQ.end(); ++it) {
            if ((*it)->pkt == pkt) {
                return (*it);
            }
        }
    } else {
        for (auto it = accWrQ.begin(); it != accWrQ.end(); ++it) {
            if ((*it)->pkt == pkt) {
                return (*it);
            }
        }
    }
    panic("Could not find memory request in request queues");
    return NULL;
}

void
CommInterface::clearMemRequest(MemoryRequest *req, bool isRead)
{
    if (isRead) {
        for (auto it = accRdQ.begin(); it != accRdQ.end(); ++it) {
            if ((*it) == req) {
                it = accRdQ.erase(it);
                break;
            }
        }
    } else {
        for (auto it = accWrQ.begin(); it != accWrQ.end(); ++it) {
            if ((*it) == req) {
                it = accWrQ.erase(it);
                break;
            }
        }
    }
}

void
CommInterface::startup()
{}

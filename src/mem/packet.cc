/*
 * Copyright (c) 2011-2019, 2021 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2010,2015 Advanced Micro Devices, Inc.
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

/**
 * @file
 * Definition of the Packet Class, a packet is a transaction occuring
 * between a single level of the memory heirarchy (ie L1->L2).
 */

#include "mem/packet.hh"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "mem/packet_access.hh"
#include "sim/bufval.hh"

namespace gem5
{

const MemCmd::CommandInfo MemCmd::commandInfo[] = {
    /* InvalidCmd */
    { {}, InvalidCmd, "InvalidCmd" },
    /* ReadReq - Read issued by a non-caching agent such as a CPU or
     * device, with no restrictions on alignment. */
    { { IsRead, IsRequest, NeedsResponse }, ReadResp, "ReadReq" },
    /* ReadResp */
    { { IsRead, IsResponse, HasData }, InvalidCmd, "ReadResp" },
    /* ReadRespWithInvalidate */
    { { IsRead, IsResponse, HasData, IsInvalidate },
      InvalidCmd,
      "ReadRespWithInvalidate" },
    /* WriteReq */
    { { IsWrite, NeedsWritable, IsRequest, NeedsResponse, HasData },
      WriteResp,
      "WriteReq" },
    /* WriteResp */
    { { IsWrite, IsResponse }, InvalidCmd, "WriteResp" },
    /* WriteCompleteResp - The WriteCompleteResp command is needed
     * because in the GPU memory model we use a WriteResp to indicate
     * that a write has reached the cache controller so we can free
     * resources at the coalescer. Later, when the write succesfully
     * completes we send a WriteCompleteResp to the CU so its wait
     * counters can be updated. Wait counters in the CU is how memory
     * dependences are handled in the GPU ISA. */
    { { IsWrite, IsResponse }, InvalidCmd, "WriteCompleteResp" },
    /* WritebackDirty */
    { { IsWrite, IsRequest, IsEviction, HasData, FromCache },
      InvalidCmd,
      "WritebackDirty" },
    /* WritebackClean - This allows the upstream cache to writeback a
     * line to the downstream cache without it being considered
     * dirty. */
    { { IsWrite, IsRequest, IsEviction, HasData, FromCache },
      InvalidCmd,
      "WritebackClean" },
    /* WriteClean - This allows a cache to write a dirty block to a memory
       below without evicting its copy. */
    { { IsWrite, IsRequest, HasData, FromCache }, InvalidCmd, "WriteClean" },
    /* CleanEvict */
    { { IsRequest, IsEviction, FromCache }, InvalidCmd, "CleanEvict" },
    /* SoftPFReq */
    { { IsRead, IsRequest, IsSWPrefetch, NeedsResponse },
      SoftPFResp,
      "SoftPFReq" },
    /* SoftPFExReq */
    { { IsRead, NeedsWritable, IsInvalidate, IsRequest, IsSWPrefetch,
        NeedsResponse },
      SoftPFResp,
      "SoftPFExReq" },
    /* HardPFReq */
    { { IsRead, IsRequest, IsHWPrefetch, NeedsResponse, FromCache },
      HardPFResp,
      "HardPFReq" },
    /* SoftPFResp */
    { { IsRead, IsResponse, IsSWPrefetch, HasData },
      InvalidCmd,
      "SoftPFResp" },
    /* HardPFResp */
    { { IsRead, IsResponse, IsHWPrefetch, HasData },
      InvalidCmd,
      "HardPFResp" },
    /* WriteLineReq */
    { { IsWrite, NeedsWritable, IsRequest, NeedsResponse, HasData },
      WriteResp,
      "WriteLineReq" },
    /* UpgradeReq */
    { { IsInvalidate, NeedsWritable, IsUpgrade, IsRequest, NeedsResponse,
        FromCache },
      UpgradeResp,
      "UpgradeReq" },
    /* SCUpgradeReq: response could be UpgradeResp or UpgradeFailResp */
    { { IsInvalidate, NeedsWritable, IsUpgrade, IsLlsc, IsRequest,
        NeedsResponse, FromCache },
      UpgradeResp,
      "SCUpgradeReq" },
    /* UpgradeResp */
    { { IsUpgrade, IsResponse }, InvalidCmd, "UpgradeResp" },
    /* SCUpgradeFailReq: generates UpgradeFailResp but still gets the data */
    { { IsRead, NeedsWritable, IsInvalidate, IsLlsc, IsRequest, NeedsResponse,
        FromCache },
      UpgradeFailResp,
      "SCUpgradeFailReq" },
    /* UpgradeFailResp - Behaves like a ReadExReq, but notifies an SC
     * that it has failed, acquires line as Dirty*/
    { { IsRead, IsResponse, HasData }, InvalidCmd, "UpgradeFailResp" },
    /* ReadExReq - Read issues by a cache, always cache-line aligned,
     * and the response is guaranteed to be writeable (exclusive or
     * even modified} */
    { { IsRead, NeedsWritable, IsInvalidate, IsRequest, NeedsResponse,
        FromCache },
      ReadExResp,
      "ReadExReq" },
    /* ReadExResp - Response matching a read exclusive, as we check
     * the need for exclusive also on responses */
    { { IsRead, IsResponse, HasData }, InvalidCmd, "ReadExResp" },
    /* ReadCleanReq - Read issued by a cache, always cache-line
     * aligned, and the response is guaranteed to not contain dirty data
     * (exclusive or shared}.*/
    { { IsRead, IsRequest, NeedsResponse, FromCache },
      ReadResp,
      "ReadCleanReq" },
    /* ReadSharedReq - Read issued by a cache, always cache-line
     * aligned, response is shared, possibly exclusive, owned or even
     * modified. */
    { { IsRead, IsRequest, NeedsResponse, FromCache },
      ReadResp,
      "ReadSharedReq" },
    /* LoadLockedReq: note that we use plain ReadResp as response, so that
     *                we can also use ReadRespWithInvalidate when needed */
    { { IsRead, IsLlsc, IsRequest, NeedsResponse },
      ReadResp,
      "LoadLockedReq" },
    /* StoreCondReq */
    { { IsWrite, NeedsWritable, IsLlsc, IsRequest, NeedsResponse, HasData },
      StoreCondResp,
      "StoreCondReq" },
    /* StoreCondFailReq: generates failing StoreCondResp */
    { { IsWrite, NeedsWritable, IsLlsc, IsRequest, NeedsResponse, HasData },
      StoreCondResp,
      "StoreCondFailReq" },
    /* StoreCondResp */
    { { IsWrite, IsLlsc, IsResponse }, InvalidCmd, "StoreCondResp" },
    /* LockedRMWReadReq */
    { { IsRead, IsLockedRMW, NeedsWritable, IsRequest, NeedsResponse },
      LockedRMWReadResp,
      "LockedRMWReadReq" },
    /* LockedRMWReadResp */
    { { IsRead, IsLockedRMW, NeedsWritable, IsResponse, HasData },
      InvalidCmd,
      "LockedRMWReadResp" },
    /* LockedRMWWriteReq */
    { { IsWrite, IsLockedRMW, NeedsWritable, IsRequest, NeedsResponse,
        HasData },
      LockedRMWWriteResp,
      "LockedRMWWriteReq" },
    /* LockedRMWWriteResp */
    { { IsWrite, IsLockedRMW, NeedsWritable, IsResponse },
      InvalidCmd,
      "LockedRMWWriteResp" },
    /* SwapReq -- for Swap ldstub type operations */
    { { IsRead, IsWrite, NeedsWritable, IsRequest, HasData, NeedsResponse },
      SwapResp,
      "SwapReq" },
    /* SwapResp -- for Swap ldstub type operations */
    { { IsRead, IsWrite, IsResponse, HasData }, InvalidCmd, "SwapResp" },
    { {}, InvalidCmd, "Deprecated_MessageReq" },
    { {}, InvalidCmd, "Deprecated_MessageResp" },
    /* MemFenceReq -- for synchronization requests */
    { { IsRequest, NeedsResponse }, MemFenceResp, "MemFenceReq" },
    /* MemSyncReq */
    { { IsRequest, NeedsResponse }, MemSyncResp, "MemSyncReq" },
    /* MemSyncResp */
    { { IsResponse }, InvalidCmd, "MemSyncResp" },
    /* MemFenceResp -- for synchronization responses */
    { { IsResponse }, InvalidCmd, "MemFenceResp" },
    /* Cache Clean Request -- Update with the latest data all existing
       copies of the block down to the point indicated by the
       request */
    { { IsRequest, IsClean, NeedsResponse, FromCache },
      CleanSharedResp,
      "CleanSharedReq" },
    /* Cache Clean Response - Indicates that all caches up to the
       specified point of reference have a up-to-date copy of the
       cache block or no copy at all */
    { { IsResponse, IsClean }, InvalidCmd, "CleanSharedResp" },
    /* Cache Clean and Invalidate Request -- Invalidate all existing
       copies down to the point indicated by the request */
    { { IsRequest, IsInvalidate, IsClean, NeedsResponse, FromCache },
      CleanInvalidResp,
      "CleanInvalidReq" },
    /* Cache Clean and Invalidate Respose -- Indicates that no cache
       above the specified point holds the block and that the block
       was written to a memory below the specified point. */
    { { IsResponse, IsInvalidate, IsClean }, InvalidCmd, "CleanInvalidResp" },
    /* InvalidDestError  -- packet dest field invalid */
    { { IsResponse, IsError }, InvalidCmd, "InvalidDestError" },
    /* BadAddressError   -- memory address invalid */
    { { IsResponse, IsError }, InvalidCmd, "BadAddressError" },
    /* ReadError -- packet dest unable to fulfill read command */
    { { IsRead, IsResponse, IsError }, InvalidCmd, "ReadError" },
    /* WriteError -- packet dest unable to fulfill write command */
    { { IsWrite, IsResponse, IsError }, InvalidCmd, "WriteError" },
    /* FunctionalReadError */
    { { IsRead, IsResponse, IsError }, InvalidCmd, "FunctionalReadError" },
    /* FunctionalWriteError */
    { { IsWrite, IsResponse, IsError }, InvalidCmd, "FunctionalWriteError" },
    /* PrintReq */
    { { IsRequest, IsPrint }, InvalidCmd, "PrintReq" },
    /* Flush Request */
    { { IsRequest, IsFlush, NeedsWritable }, InvalidCmd, "FlushReq" },
    /* Invalidation Request */
    { { IsInvalidate, IsRequest, NeedsWritable, NeedsResponse, FromCache },
      InvalidateResp,
      "InvalidateReq" },
    /* Invalidation Response */
    { { IsInvalidate, IsResponse }, InvalidCmd, "InvalidateResp" },
    // hardware transactional memory
    { { IsRead, IsRequest, NeedsResponse }, HTMReqResp, "HTMReq" },
    { { IsRead, IsResponse }, InvalidCmd, "HTMReqResp" },
    { { IsRead, IsRequest }, InvalidCmd, "HTMAbort" },
    { { IsRequest }, InvalidCmd, "TlbiExtSync" },
};

AddrRange
Packet::getAddrRange() const
{
    return RangeSize(getAddr(), getSize());
}

bool
Packet::trySatisfyFunctional(Printable *obj, Addr addr, bool is_secure,
                             int size, uint8_t *_data)
{
    const Addr func_start = getAddr();
    const Addr func_end = getAddr() + getSize() - 1;
    const Addr val_start = addr;
    const Addr val_end = val_start + size - 1;

    if (is_secure != _isSecure || func_start > val_end ||
        val_start > func_end) {
        // no intersection
        return false;
    }

    // check print first since it doesn't require data
    if (isPrint()) {
        assert(!_data);
        safe_cast<PrintReqState *>(senderState)->printObj(obj);
        return false;
    }

    // we allow the caller to pass NULL to signify the other packet
    // has no data
    if (!_data) {
        return false;
    }

    const Addr val_offset =
        func_start > val_start ? func_start - val_start : 0;
    const Addr func_offset =
        func_start < val_start ? val_start - func_start : 0;
    const Addr overlap_size =
        std::min(val_end, func_end) + 1 - std::max(val_start, func_start);

    if (isRead()) {
        std::memcpy(getPtr<uint8_t>() + func_offset, _data + val_offset,
                    overlap_size);

        // initialise the tracking of valid bytes if we have not
        // used it already
        if (bytesValid.empty())
            bytesValid.resize(getSize(), false);

        // track if we are done filling the functional access
        bool all_bytes_valid = true;

        int i = 0;

        // check up to func_offset
        for (; all_bytes_valid && i < func_offset; ++i)
            all_bytes_valid &= bytesValid[i];

        // update the valid bytes
        for (i = func_offset; i < func_offset + overlap_size; ++i)
            bytesValid[i] = true;

        // check the bit after the update we just made
        for (; all_bytes_valid && i < getSize(); ++i)
            all_bytes_valid &= bytesValid[i];

        return all_bytes_valid;
    } else if (isWrite()) {
        std::memcpy(_data + val_offset, getConstPtr<uint8_t>() + func_offset,
                    overlap_size);
    } else {
        panic("Don't know how to handle command %s\n", cmdString());
    }

    // keep going with request by default
    return false;
}

void
Packet::copyResponderFlags(const PacketPtr pkt)
{
    assert(isRequest());
    // If we have already found a responder, no other cache should
    // commit to responding
    assert(!pkt->cacheResponding() || !cacheResponding());
    flags.set(pkt->flags & RESPONDER_FLAGS);
}

void
Packet::pushSenderState(Packet::SenderState *sender_state)
{
    assert(sender_state != NULL);
    sender_state->predecessor = senderState;
    senderState = sender_state;
}

Packet::SenderState *
Packet::popSenderState()
{
    assert(senderState != NULL);
    SenderState *sender_state = senderState;
    senderState = sender_state->predecessor;
    sender_state->predecessor = NULL;
    return sender_state;
}

uint64_t
Packet::getUintX(ByteOrder endian) const
{
    auto [val, success] =
        gem5::getUintX(getConstPtr<void>(), getSize(), endian);
    panic_if(!success, "%i isn't a supported word size.\n", getSize());
    return val;
}

void
Packet::setUintX(uint64_t w, ByteOrder endian)
{
    bool success = gem5::setUintX(w, getPtr<void>(), getSize(), endian);
    panic_if(!success, "%i isn't a supported word size.\n", getSize());
}

void
Packet::print(std::ostream &o, const int verbosity,
              const std::string &prefix) const
{
    ccprintf(o, "%s%s [%x:%x]%s%s%s%s%s%s", prefix, cmdString(), getAddr(),
             getAddr() + getSize() - 1, req->isSecure() ? " (s)" : "",
             req->isInstFetch() ? " IF" : "",
             req->isUncacheable() ? " UC" : "", isExpressSnoop() ? " ES" : "",
             req->isToPOC() ? " PoC" : "", req->isToPOU() ? " PoU" : "");
}

std::string
Packet::print() const
{
    std::ostringstream str;
    print(str);
    return str.str();
}

bool
Packet::matchBlockAddr(const Addr addr, const bool is_secure,
                       const int blk_size) const
{
    return (getBlockAddr(blk_size) == addr) && (isSecure() == is_secure);
}

bool
Packet::matchBlockAddr(const PacketPtr pkt, const int blk_size) const
{
    return matchBlockAddr(pkt->getBlockAddr(blk_size), pkt->isSecure(),
                          blk_size);
}

bool
Packet::matchAddr(const Addr addr, const bool is_secure) const
{
    return (getAddr() == addr) && (isSecure() == is_secure);
}

bool
Packet::matchAddr(const PacketPtr pkt) const
{
    return matchAddr(pkt->getAddr(), pkt->isSecure());
}

Packet::PrintReqState::PrintReqState(std::ostream &_os, int _verbosity)
    : curPrefixPtr(new std::string("")), os(_os), verbosity(_verbosity)
{
    labelStack.push_back(LabelStackEntry("", curPrefixPtr));
}

Packet::PrintReqState::~PrintReqState()
{
    labelStack.pop_back();
    assert(labelStack.empty());
    delete curPrefixPtr;
}

Packet::PrintReqState::LabelStackEntry::LabelStackEntry(
    const std::string &_label, std::string *_prefix)
    : label(_label), prefix(_prefix), labelPrinted(false)
{}

void
Packet::PrintReqState::pushLabel(const std::string &lbl,
                                 const std::string &prefix)
{
    labelStack.push_back(LabelStackEntry(lbl, curPrefixPtr));
    curPrefixPtr = new std::string(*curPrefixPtr);
    *curPrefixPtr += prefix;
}

void
Packet::PrintReqState::popLabel()
{
    delete curPrefixPtr;
    curPrefixPtr = labelStack.back().prefix;
    labelStack.pop_back();
    assert(!labelStack.empty());
}

void
Packet::PrintReqState::printLabels()
{
    if (!labelStack.back().labelPrinted) {
        LabelStack::iterator i = labelStack.begin();
        LabelStack::iterator end = labelStack.end();
        while (i != end) {
            if (!i->labelPrinted) {
                ccprintf(os, "%s%s\n", *(i->prefix), i->label);
                i->labelPrinted = true;
            }
            i++;
        }
    }
}

void
Packet::PrintReqState::printObj(Printable *obj)
{
    printLabels();
    obj->print(os, verbosity, curPrefix());
}

void
Packet::makeHtmTransactionalReqResponse(const HtmCacheFailure htm_return_code)
{
    assert(needsResponse());
    assert(isRequest());

    cmd = cmd.responseCommand();

    setHtmTransactionFailedInCache(htm_return_code);

    // responses are never express, even if the snoop that
    // triggered them was
    flags.clear(EXPRESS_SNOOP);
}

void
Packet::setHtmTransactionFailedInCache(const HtmCacheFailure htm_return_code)
{
    if (htm_return_code != HtmCacheFailure::NO_FAIL)
        flags.set(FAILS_TRANSACTION);

    htmReturnReason = htm_return_code;
}

bool
Packet::htmTransactionFailedInCache() const
{
    return flags.isSet(FAILS_TRANSACTION);
}

HtmCacheFailure
Packet::getHtmTransactionFailedInCacheRC() const
{
    assert(htmTransactionFailedInCache());
    return htmReturnReason;
}

void
Packet::setHtmTransactional(uint64_t htm_uid)
{
    flags.set(FROM_TRANSACTION);
    htmTransactionUid = htm_uid;
}

bool
Packet::isHtmTransactional() const
{
    return flags.isSet(FROM_TRANSACTION);
}

uint64_t
Packet::getHtmTransactionUid() const
{
    assert(flags.isSet(FROM_TRANSACTION));
    return htmTransactionUid;
}

} // namespace gem5

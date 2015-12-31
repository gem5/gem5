/*
 * Copyright (c) 2011-2015 ARM Limited
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
 *
 * Authors: Ali Saidi
 *          Steve Reinhardt
 */

/**
 * @file
 * Definition of the Packet Class, a packet is a transaction occuring
 * between a single level of the memory heirarchy (ie L1->L2).
 */

#include <cstring>
#include <iostream>

#include "base/cprintf.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "mem/packet.hh"

using namespace std;

// The one downside to bitsets is that static initializers can get ugly.
#define SET1(a1)                     (1 << (a1))
#define SET2(a1, a2)                 (SET1(a1) | SET1(a2))
#define SET3(a1, a2, a3)             (SET2(a1, a2) | SET1(a3))
#define SET4(a1, a2, a3, a4)         (SET3(a1, a2, a3) | SET1(a4))
#define SET5(a1, a2, a3, a4, a5)     (SET4(a1, a2, a3, a4) | SET1(a5))
#define SET6(a1, a2, a3, a4, a5, a6) (SET5(a1, a2, a3, a4, a5) | SET1(a6))

const MemCmd::CommandInfo
MemCmd::commandInfo[] =
{
    /* InvalidCmd */
    { 0, InvalidCmd, "InvalidCmd" },
    /* ReadReq - Read issued by a non-caching agent such as a CPU or
     * device, with no restrictions on alignment. */
    { SET3(IsRead, IsRequest, NeedsResponse), ReadResp, "ReadReq" },
    /* ReadResp */
    { SET3(IsRead, IsResponse, HasData), InvalidCmd, "ReadResp" },
    /* ReadRespWithInvalidate */
    { SET4(IsRead, IsResponse, HasData, IsInvalidate),
            InvalidCmd, "ReadRespWithInvalidate" },
    /* WriteReq */
    { SET5(IsWrite, NeedsWritable, IsRequest, NeedsResponse, HasData),
            WriteResp, "WriteReq" },
    /* WriteResp */
    { SET2(IsWrite, IsResponse), InvalidCmd, "WriteResp" },
    /* WritebackDirty */
    { SET4(IsWrite, IsRequest, IsEviction, HasData),
            InvalidCmd, "WritebackDirty" },
    /* WritebackClean - This allows the upstream cache to writeback a
     * line to the downstream cache without it being considered
     * dirty. */
    { SET4(IsWrite, IsRequest, IsEviction, HasData),
            InvalidCmd, "WritebackClean" },
    /* CleanEvict */
    { SET2(IsRequest, IsEviction), InvalidCmd, "CleanEvict" },
    /* SoftPFReq */
    { SET4(IsRead, IsRequest, IsSWPrefetch, NeedsResponse),
            SoftPFResp, "SoftPFReq" },
    /* HardPFReq */
    { SET4(IsRead, IsRequest, IsHWPrefetch, NeedsResponse),
            HardPFResp, "HardPFReq" },
    /* SoftPFResp */
    { SET4(IsRead, IsResponse, IsSWPrefetch, HasData),
            InvalidCmd, "SoftPFResp" },
    /* HardPFResp */
    { SET4(IsRead, IsResponse, IsHWPrefetch, HasData),
            InvalidCmd, "HardPFResp" },
    /* WriteLineReq */
    { SET5(IsWrite, NeedsWritable, IsRequest, NeedsResponse, HasData),
            WriteResp, "WriteLineReq" },
    /* UpgradeReq */
    { SET5(IsInvalidate, NeedsWritable, IsUpgrade, IsRequest, NeedsResponse),
            UpgradeResp, "UpgradeReq" },
    /* SCUpgradeReq: response could be UpgradeResp or UpgradeFailResp */
    { SET6(IsInvalidate, NeedsWritable, IsUpgrade, IsLlsc,
           IsRequest, NeedsResponse),
            UpgradeResp, "SCUpgradeReq" },
    /* UpgradeResp */
    { SET2(IsUpgrade, IsResponse),
            InvalidCmd, "UpgradeResp" },
    /* SCUpgradeFailReq: generates UpgradeFailResp but still gets the data */
    { SET6(IsRead, NeedsWritable, IsInvalidate,
           IsLlsc, IsRequest, NeedsResponse),
            UpgradeFailResp, "SCUpgradeFailReq" },
    /* UpgradeFailResp - Behaves like a ReadExReq, but notifies an SC
     * that it has failed, acquires line as Dirty*/
    { SET3(IsRead, IsResponse, HasData),
            InvalidCmd, "UpgradeFailResp" },
    /* ReadExReq - Read issues by a cache, always cache-line aligned,
     * and the response is guaranteed to be writeable (exclusive or
     * even modified) */
    { SET5(IsRead, NeedsWritable, IsInvalidate, IsRequest, NeedsResponse),
            ReadExResp, "ReadExReq" },
    /* ReadExResp - Response matching a read exclusive, as we check
     * the need for exclusive also on responses */
    { SET3(IsRead, IsResponse, HasData),
            InvalidCmd, "ReadExResp" },
    /* ReadCleanReq - Read issued by a cache, always cache-line
     * aligned, and the response is guaranteed to not contain dirty data
     * (exclusive or shared).*/
    { SET3(IsRead, IsRequest, NeedsResponse), ReadResp, "ReadCleanReq" },
    /* ReadSharedReq - Read issued by a cache, always cache-line
     * aligned, response is shared, possibly exclusive, owned or even
     * modified. */
    { SET3(IsRead, IsRequest, NeedsResponse), ReadResp, "ReadSharedReq" },
    /* LoadLockedReq: note that we use plain ReadResp as response, so that
     *                we can also use ReadRespWithInvalidate when needed */
    { SET4(IsRead, IsLlsc, IsRequest, NeedsResponse),
            ReadResp, "LoadLockedReq" },
    /* StoreCondReq */
    { SET6(IsWrite, NeedsWritable, IsLlsc,
           IsRequest, NeedsResponse, HasData),
            StoreCondResp, "StoreCondReq" },
    /* StoreCondFailReq: generates failing StoreCondResp */
    { SET6(IsWrite, NeedsWritable, IsLlsc,
           IsRequest, NeedsResponse, HasData),
            StoreCondResp, "StoreCondFailReq" },
    /* StoreCondResp */
    { SET3(IsWrite, IsLlsc, IsResponse),
            InvalidCmd, "StoreCondResp" },
    /* SwapReq -- for Swap ldstub type operations */
    { SET6(IsRead, IsWrite, NeedsWritable, IsRequest, HasData, NeedsResponse),
        SwapResp, "SwapReq" },
    /* SwapResp -- for Swap ldstub type operations */
    { SET4(IsRead, IsWrite, IsResponse, HasData),
            InvalidCmd, "SwapResp" },
    /* IntReq -- for interrupts */
    { SET4(IsWrite, IsRequest, NeedsResponse, HasData),
        MessageResp, "MessageReq" },
    /* IntResp -- for interrupts */
    { SET2(IsWrite, IsResponse), InvalidCmd, "MessageResp" },
    /* MemFenceReq -- for synchronization requests */
    {SET2(IsRequest, NeedsResponse), MemFenceResp, "MemFenceReq"},
    /* MemFenceResp -- for synchronization responses */
    {SET1(IsResponse), InvalidCmd, "MemFenceResp"},
    /* InvalidDestError  -- packet dest field invalid */
    { SET2(IsResponse, IsError), InvalidCmd, "InvalidDestError" },
    /* BadAddressError   -- memory address invalid */
    { SET2(IsResponse, IsError), InvalidCmd, "BadAddressError" },
    /* FunctionalReadError */
    { SET3(IsRead, IsResponse, IsError), InvalidCmd, "FunctionalReadError" },
    /* FunctionalWriteError */
    { SET3(IsWrite, IsResponse, IsError), InvalidCmd, "FunctionalWriteError" },
    /* PrintReq */
    { SET2(IsRequest, IsPrint), InvalidCmd, "PrintReq" },
    /* Flush Request */
    { SET3(IsRequest, IsFlush, NeedsWritable), InvalidCmd, "FlushReq" },
    /* Invalidation Request */
    { SET4(IsInvalidate, IsRequest, NeedsWritable, NeedsResponse),
      InvalidateResp, "InvalidateReq" },
    /* Invalidation Response */
    { SET2(IsInvalidate, IsResponse),
      InvalidCmd, "InvalidateResp" }
};

bool
Packet::checkFunctional(Printable *obj, Addr addr, bool is_secure, int size,
                        uint8_t *_data)
{
    Addr func_start = getAddr();
    Addr func_end   = getAddr() + getSize() - 1;
    Addr val_start  = addr;
    Addr val_end    = val_start + size - 1;

    if (is_secure != _isSecure || func_start > val_end ||
        val_start > func_end) {
        // no intersection
        return false;
    }

    // check print first since it doesn't require data
    if (isPrint()) {
        assert(!_data);
        safe_cast<PrintReqState*>(senderState)->printObj(obj);
        return false;
    }

    // we allow the caller to pass NULL to signify the other packet
    // has no data
    if (!_data) {
        return false;
    }

    // offset of functional request into supplied value (could be
    // negative if partial overlap)
    int offset = func_start - val_start;

    if (isRead()) {
        if (func_start >= val_start && func_end <= val_end) {
            memcpy(getPtr<uint8_t>(), _data + offset, getSize());
            if (bytesValid.empty())
                bytesValid.resize(getSize(), true);
            // complete overlap, and as the current packet is a read
            // we are done
            return true;
        } else {
            // Offsets and sizes to copy in case of partial overlap
            int func_offset;
            int val_offset;
            int overlap_size;

            // calculate offsets and copy sizes for the two byte arrays
            if (val_start < func_start && val_end <= func_end) {
                // the one we are checking against starts before and
                // ends before or the same
                val_offset = func_start - val_start;
                func_offset = 0;
                overlap_size = val_end - func_start;
            } else if (val_start >= func_start && val_end > func_end) {
                // the one we are checking against starts after or the
                // same, and ends after
                val_offset = 0;
                func_offset = val_start - func_start;
                overlap_size = func_end - val_start;
            } else if (val_start >= func_start && val_end <= func_end) {
                // the one we are checking against is completely
                // subsumed in the current packet, possibly starting
                // and ending at the same address
                val_offset = 0;
                func_offset = val_start - func_start;
                overlap_size = size;
            } else if (val_start < func_start && val_end > func_end) {
                // the current packet is completely subsumed in the
                // one we are checking against
                val_offset = func_start - val_start;
                func_offset = 0;
                overlap_size = func_end - func_start;
            } else {
                panic("Missed a case for checkFunctional with "
                      " %s 0x%x size %d, against 0x%x size %d\n",
                      cmdString(), getAddr(), getSize(), addr, size);
            }

            // copy partial data into the packet's data array
            uint8_t *dest = getPtr<uint8_t>() + func_offset;
            uint8_t *src = _data + val_offset;
            memcpy(dest, src, overlap_size);

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
        }
    } else if (isWrite()) {
        if (offset >= 0) {
            memcpy(_data + offset, getConstPtr<uint8_t>(),
                   (min(func_end, val_end) - func_start) + 1);
        } else {
            // val_start > func_start
            memcpy(_data, getConstPtr<uint8_t>() - offset,
                   (min(func_end, val_end) - val_start) + 1);
        }
    } else {
        panic("Don't know how to handle command %s\n", cmdString());
    }

    // keep going with request by default
    return false;
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

void
Packet::print(ostream &o, const int verbosity, const string &prefix) const
{
    ccprintf(o, "%s[%x:%x] %s\n", prefix,
             getAddr(), getAddr() + getSize() - 1, cmdString());
}

std::string
Packet::print() const {
    ostringstream str;
    print(str);
    return str.str();
}

Packet::PrintReqState::PrintReqState(ostream &_os, int _verbosity)
    : curPrefixPtr(new string("")), os(_os), verbosity(_verbosity)
{
    labelStack.push_back(LabelStackEntry("", curPrefixPtr));
}

Packet::PrintReqState::~PrintReqState()
{
    labelStack.pop_back();
    assert(labelStack.empty());
    delete curPrefixPtr;
}

Packet::PrintReqState::
LabelStackEntry::LabelStackEntry(const string &_label, string *_prefix)
    : label(_label), prefix(_prefix), labelPrinted(false)
{
}

void
Packet::PrintReqState::pushLabel(const string &lbl, const string &prefix)
{
    labelStack.push_back(LabelStackEntry(lbl, curPrefixPtr));
    curPrefixPtr = new string(*curPrefixPtr);
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

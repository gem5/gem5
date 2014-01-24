/*
 * Copyright (c) 2011-2013 ARM Limited
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
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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
    /* ReadReq */
    { SET3(IsRead, IsRequest, NeedsResponse), ReadResp, "ReadReq" },
    /* ReadResp */
    { SET3(IsRead, IsResponse, HasData), InvalidCmd, "ReadResp" },
    /* ReadRespWithInvalidate */
    { SET4(IsRead, IsResponse, HasData, IsInvalidate),
            InvalidCmd, "ReadRespWithInvalidate" },
    /* WriteReq */
    { SET5(IsWrite, NeedsExclusive, IsRequest, NeedsResponse, HasData),
            WriteResp, "WriteReq" },
    /* WriteResp */
    { SET3(IsWrite, NeedsExclusive, IsResponse), InvalidCmd, "WriteResp" },
    /* Writeback */
    { SET4(IsWrite, NeedsExclusive, IsRequest, HasData),
            InvalidCmd, "Writeback" },
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
    /* WriteInvalidateReq (currently unused, see packet.hh) */
    { SET6(IsWrite, NeedsExclusive, IsInvalidate,
           IsRequest, HasData, NeedsResponse),
            WriteInvalidateResp, "WriteInvalidateReq" },
    /* WriteInvalidateResp (currently unused, see packet.hh) */
    { SET3(IsWrite, NeedsExclusive, IsResponse),
            InvalidCmd, "WriteInvalidateResp" },
    /* UpgradeReq */
    { SET5(IsInvalidate, NeedsExclusive, IsUpgrade, IsRequest, NeedsResponse),
            UpgradeResp, "UpgradeReq" },
    /* SCUpgradeReq: response could be UpgradeResp or UpgradeFailResp */
    { SET6(IsInvalidate, NeedsExclusive, IsUpgrade, IsLlsc,
           IsRequest, NeedsResponse),
            UpgradeResp, "SCUpgradeReq" },
    /* UpgradeResp */
    { SET3(NeedsExclusive, IsUpgrade, IsResponse),
            InvalidCmd, "UpgradeResp" },
    /* SCUpgradeFailReq: generates UpgradeFailResp ASAP */
    { SET5(IsInvalidate, NeedsExclusive, IsLlsc,
           IsRequest, NeedsResponse),
            UpgradeFailResp, "SCUpgradeFailReq" },
    /* UpgradeFailResp */
    { SET2(NeedsExclusive, IsResponse),
            InvalidCmd, "UpgradeFailResp" },
    /* ReadExReq */
    { SET5(IsRead, NeedsExclusive, IsInvalidate, IsRequest, NeedsResponse),
            ReadExResp, "ReadExReq" },
    /* ReadExResp */
    { SET4(IsRead, NeedsExclusive, IsResponse, HasData),
            InvalidCmd, "ReadExResp" },
    /* LoadLockedReq: note that we use plain ReadResp as response, so that
     *                we can also use ReadRespWithInvalidate when needed */
    { SET4(IsRead, IsLlsc, IsRequest, NeedsResponse),
            ReadResp, "LoadLockedReq" },
    /* StoreCondReq */
    { SET6(IsWrite, NeedsExclusive, IsLlsc,
           IsRequest, NeedsResponse, HasData),
            StoreCondResp, "StoreCondReq" },
    /* StoreCondFailReq: generates failing StoreCondResp ASAP */
    { SET6(IsWrite, NeedsExclusive, IsLlsc,
           IsRequest, NeedsResponse, HasData),
            StoreCondResp, "StoreCondFailReq" },
    /* StoreCondResp */
    { SET4(IsWrite, NeedsExclusive, IsLlsc, IsResponse),
            InvalidCmd, "StoreCondResp" },
    /* SwapReq -- for Swap ldstub type operations */
    { SET6(IsRead, IsWrite, NeedsExclusive, IsRequest, HasData, NeedsResponse),
        SwapResp, "SwapReq" },
    /* SwapResp -- for Swap ldstub type operations */
    { SET5(IsRead, IsWrite, NeedsExclusive, IsResponse, HasData),
            InvalidCmd, "SwapResp" },
    /* IntReq -- for interrupts */
    { SET4(IsWrite, IsRequest, NeedsResponse, HasData),
        MessageResp, "MessageReq" },
    /* IntResp -- for interrupts */
    { SET2(IsWrite, IsResponse), InvalidCmd, "MessageResp" },
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
    { SET3(IsRequest, IsFlush, NeedsExclusive), InvalidCmd, "FlushReq" },
    /* Invalidation Request */
    { SET3(NeedsExclusive, IsInvalidate, IsRequest),
      InvalidCmd, "InvalidationReq" },
};

bool
Packet::checkFunctional(Printable *obj, Addr addr, bool is_secure, int size,
                        uint8_t *data)
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
        dynamic_cast<PrintReqState*>(senderState)->printObj(obj);
        return false;
    }

    // if there's no data, there's no need to look further
    if (!data) {
        return false;
    }

    // offset of functional request into supplied value (could be
    // negative if partial overlap)
    int offset = func_start - val_start;

    if (isRead()) {
        if (func_start >= val_start && func_end <= val_end) {
            allocate();
            memcpy(getPtr<uint8_t>(), data + offset, getSize());
            return true;
        } else {
            // Offsets and sizes to copy in case of partial overlap
            int func_offset;
            int val_offset;
            int overlap_size;

            // calculate offsets and copy sizes for the two byte arrays
            if (val_start < func_start && val_end <= func_end) {
                val_offset = func_start - val_start;
                func_offset = 0;
                overlap_size = val_end - func_start;
            } else if (val_start >= func_start && val_end > func_end) {
                val_offset = 0;
                func_offset = val_start - func_start;
                overlap_size = func_end - val_start;
            } else if (val_start >= func_start && val_end <= func_end) {
                val_offset = 0;
                func_offset = val_start - func_start;
                overlap_size = size;
            } else {
                panic("BUG: Missed a case for a partial functional request");
            }

            // Figure out how much of the partial overlap should be copied
            // into the packet and not overwrite previously found bytes.
            if (bytesValidStart == 0 && bytesValidEnd == 0) {
                // No bytes have been copied yet, just set indices
                // to found range
                bytesValidStart = func_offset;
                bytesValidEnd = func_offset + overlap_size;
            } else {
                // Some bytes have already been copied. Use bytesValid
                // indices and offset values to figure out how much data
                // to copy and where to copy it to.

                // Indice overlap conditions to check
                int a = func_offset - bytesValidStart;
                int b = (func_offset + overlap_size) - bytesValidEnd;
                int c = func_offset - bytesValidEnd;
                int d = (func_offset + overlap_size) - bytesValidStart;

                if (a >= 0 && b <= 0) {
                    // bytes already in pkt data array are superset of
                    // found bytes, will not copy any bytes
                    overlap_size = 0;
                } else if (a < 0 && d >= 0 && b <= 0) {
                    // found bytes will move bytesValidStart towards 0
                    overlap_size = bytesValidStart - func_offset;
                    bytesValidStart = func_offset;
                } else if (b > 0 && c <= 0 && a >= 0) {
                    // found bytes will move bytesValidEnd
                    // towards end of pkt data array
                    overlap_size =
                        (func_offset + overlap_size) - bytesValidEnd;
                    val_offset += bytesValidEnd - func_offset;
                    func_offset = bytesValidEnd;
                    bytesValidEnd += overlap_size;
                } else if (a < 0 && b > 0) {
                    // Found bytes are superset of copied range. Will move
                    // bytesValidStart towards 0 and bytesValidEnd towards
                    // end of pkt data array.  Need to break copy into two
                    // pieces so as to not overwrite previously found data.

                    // copy the first half
                    uint8_t *dest = getPtr<uint8_t>() + func_offset;
                    uint8_t *src = data + val_offset;
                    memcpy(dest, src, (bytesValidStart - func_offset));

                    // re-calc the offsets and indices to do the copy
                    // required for the second half
                    val_offset += (bytesValidEnd - func_offset);
                    bytesValidStart = func_offset;
                    overlap_size =
                        (func_offset + overlap_size) - bytesValidEnd;
                    func_offset = bytesValidEnd;
                    bytesValidEnd += overlap_size;
                } else if ((c > 0 && b > 0)
                           || (a < 0 && d < 0)) {
                    // region to be copied is discontiguous! Not supported.
                    panic("BUG: Discontiguous bytes found"
                          "for functional copying!");
                }
            }
            assert(bytesValidEnd <= getSize());

            // copy partial data into the packet's data array
            uint8_t *dest = getPtr<uint8_t>() + func_offset;
            uint8_t *src = data + val_offset;
            memcpy(dest, src, overlap_size);

            // check if we're done filling the functional access
            bool done = (bytesValidStart == 0) && (bytesValidEnd == getSize());
            return done;
        }
    } else if (isWrite()) {
        if (offset >= 0) {
            memcpy(data + offset, getPtr<uint8_t>(),
                   (min(func_end, val_end) - func_start) + 1);
        } else {
            // val_start > func_start
            memcpy(data, getPtr<uint8_t>() - offset,
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

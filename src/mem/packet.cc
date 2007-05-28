/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include <iostream>
#include <cstring>
#include "base/misc.hh"
#include "base/trace.hh"
#include "mem/packet.hh"

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
    /* WriteReq */
    { SET4(IsWrite, IsRequest, NeedsResponse, HasData),
            WriteResp, "WriteReq" },
    /* WriteResp */
    { SET2(IsWrite, IsResponse), InvalidCmd, "WriteResp" },
    /* Writeback */
    { SET4(IsWrite, IsRequest, HasData, NeedsResponse),
            WritebackAck, "Writeback" },
    /* WritebackAck */
    { SET2(IsWrite, IsResponse), InvalidCmd, "WritebackAck" },
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
    /* InvalidateReq */
    { SET2(IsInvalidate, IsRequest), InvalidCmd, "InvalidateReq" },
    /* WriteInvalidateReq */
    { SET5(IsWrite, IsInvalidate, IsRequest, HasData, NeedsResponse),
            WriteInvalidateResp, "WriteInvalidateReq" },
    /* WriteInvalidateResp */
    { SET3(IsWrite, IsInvalidate, IsResponse),
            InvalidCmd, "WriteInvalidateResp" },
    /* UpgradeReq */
    { SET3(IsInvalidate, IsRequest, IsUpgrade), InvalidCmd, "UpgradeReq" },
    /* ReadExReq */
    { SET4(IsRead, IsInvalidate, IsRequest, NeedsResponse),
            ReadExResp, "ReadExReq" },
    /* ReadExResp */
    { SET4(IsRead, IsInvalidate, IsResponse, HasData),
            InvalidCmd, "ReadExResp" },
    /* SwapReq -- for Swap ldstub type operations */
    { SET4(IsReadWrite, IsRequest, HasData, NeedsResponse),
        SwapResp, "SwapReq" },
    /* SwapResp -- for Swap ldstub type operations */
    { SET3(IsReadWrite, IsResponse, HasData),
        InvalidCmd, "SwapResp" }
};


/** delete the data pointed to in the data pointer. Ok to call to matter how
 * data was allocted. */
void
Packet::deleteData()
{
    assert(staticData || dynamicData);
    if (staticData)
        return;

    if (arrayData)
        delete [] data;
    else
        delete data;
}

/** If there isn't data in the packet, allocate some. */
void
Packet::allocate()
{
    if (data)
        return;
    assert(!staticData);
    dynamicData = true;
    arrayData = true;
    data = new uint8_t[getSize()];
}

/** Do the packet modify the same addresses. */
bool
Packet::intersect(PacketPtr p)
{
    Addr s1 = getAddr();
    Addr e1 = getAddr() + getSize() - 1;
    Addr s2 = p->getAddr();
    Addr e2 = p->getAddr() + p->getSize() - 1;

    return !(s1 > e2 || e1 < s2);
}

bool
fixDelayedResponsePacket(PacketPtr func, PacketPtr timing)
{
    bool result;

    if (timing->isRead() || timing->isWrite()) {
        // Ugly hack to deal with the fact that we queue the requests
        // and don't convert them to responses until we issue them on
        // the bus.  I tried to avoid this by converting packets to
        // responses right away, but this breaks during snoops where a
        // responder may do the conversion before other caches have
        // done the snoop.  Would work if we copied the packet instead
        // of just hanging on to a pointer.
        MemCmd oldCmd = timing->cmd;
        timing->cmd = timing->cmd.responseCommand();
        result = fixPacket(func, timing);
        timing->cmd = oldCmd;
    }
    else {
        //Don't toggle if it isn't a read/write response
        result = fixPacket(func, timing);
    }

    return result;
}

bool
fixPacket(PacketPtr func, PacketPtr timing)
{
    Addr funcStart      = func->getAddr();
    Addr funcEnd        = func->getAddr() + func->getSize() - 1;
    Addr timingStart    = timing->getAddr();
    Addr timingEnd      = timing->getAddr() + timing->getSize() - 1;

    assert(!(funcStart > timingEnd || timingStart > funcEnd));

    // this packet can't solve our problem, continue on
    if (!timing->hasData())
        return true;

    if (func->isRead()) {
        if (funcStart >= timingStart && funcEnd <= timingEnd) {
            func->allocate();
            std::memcpy(func->getPtr<uint8_t>(), timing->getPtr<uint8_t>() +
                    funcStart - timingStart, func->getSize());
            func->result = Packet::Success;
            func->flags |= SATISFIED;
            return false;
        } else {
            // In this case the timing packet only partially satisfies
            // the request, so we would need more information to make
            // this work.  Like bytes valid in the packet or
            // something, so the request could continue and get this
            // bit of possibly newer data along with the older data
            // not written to yet.
            panic("Timing packet only partially satisfies the functional"
                    "request. Now what?");
        }
    } else if (func->isWrite()) {
        if (funcStart >= timingStart) {
            std::memcpy(timing->getPtr<uint8_t>() + (funcStart - timingStart),
                   func->getPtr<uint8_t>(),
                   (std::min(funcEnd, timingEnd) - funcStart) + 1);
        } else { // timingStart > funcStart
            std::memcpy(timing->getPtr<uint8_t>(),
                   func->getPtr<uint8_t>() + (timingStart - funcStart),
                   (std::min(funcEnd, timingEnd) - timingStart) + 1);
        }
        // we always want to keep going with a write
        return true;
    } else
        panic("Don't know how to handle command type %#x\n",
                func->cmdToIndex());

}


std::ostream &
operator<<(std::ostream &o, const Packet &p)
{

    o << "[0x";
    o.setf(std::ios_base::hex, std::ios_base::showbase);
    o <<  p.getAddr();
    o.unsetf(std::ios_base::hex| std::ios_base::showbase);
    o <<  ":";
    o.setf(std::ios_base::hex, std::ios_base::showbase);
    o <<  p.getAddr() + p.getSize() - 1 << "] ";
    o.unsetf(std::ios_base::hex| std::ios_base::showbase);

    if (p.result == Packet::Success)
        o << "Successful ";
    if (p.result == Packet::BadAddress)
        o << "BadAddress ";
    if (p.result == Packet::Nacked)
        o << "Nacked ";
    if (p.result == Packet::Unknown)
        o << "Inflight ";

    if (p.isRead())
        o << "Read ";
    if (p.isWrite())
        o << "Write ";
    if (p.isReadWrite())
        o << "Read/Write ";
    if (p.isInvalidate())
        o << "Invalidate ";
    if (p.isRequest())
        o << "Request ";
    if (p.isResponse())
        o << "Response ";
    if (p.hasData())
        o << "w/Data ";

    o << std::endl;
    return o;
}


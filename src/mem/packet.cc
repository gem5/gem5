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

#include "base/misc.hh"
#include "base/trace.hh"
#include "mem/packet.hh"

static const std::string ReadReqString("ReadReq");
static const std::string WriteReqString("WriteReq");
static const std::string WriteReqNoAckString("WriteReqNoAck|Writeback");
static const std::string ReadRespString("ReadResp");
static const std::string WriteRespString("WriteResp");
static const std::string SoftPFReqString("SoftPFReq");
static const std::string SoftPFRespString("SoftPFResp");
static const std::string HardPFReqString("HardPFReq");
static const std::string HardPFRespString("HardPFResp");
static const std::string InvalidateReqString("InvalidateReq");
static const std::string WriteInvalidateReqString("WriteInvalidateReq");
static const std::string WriteInvalidateRespString("WriteInvalidateResp");
static const std::string UpgradeReqString("UpgradeReq");
static const std::string ReadExReqString("ReadExReq");
static const std::string ReadExRespString("ReadExResp");
static const std::string OtherCmdString("<other>");

const std::string &
Packet::cmdString() const
{
    switch (cmd) {
      case ReadReq:         return ReadReqString;
      case WriteReq:        return WriteReqString;
      case WriteReqNoAck:   return WriteReqNoAckString;
      case ReadResp:        return ReadRespString;
      case WriteResp:       return WriteRespString;
      case SoftPFReq:       return SoftPFReqString;
      case SoftPFResp:      return SoftPFRespString;
      case HardPFReq:       return HardPFReqString;
      case HardPFResp:      return HardPFRespString;
      case InvalidateReq:   return InvalidateReqString;
      case WriteInvalidateReq:return WriteInvalidateReqString;
      case WriteInvalidateResp:return WriteInvalidateRespString;
      case UpgradeReq:      return UpgradeReqString;
      case ReadExReq:       return ReadExReqString;
      case ReadExResp:      return ReadExRespString;
      default:              return OtherCmdString;
    }
}

const std::string &
Packet::cmdIdxToString(Packet::Command idx)
{
    switch (idx) {
      case ReadReq:         return ReadReqString;
      case WriteReq:        return WriteReqString;
      case WriteReqNoAck:   return WriteReqNoAckString;
      case ReadResp:        return ReadRespString;
      case WriteResp:       return WriteRespString;
      case SoftPFReq:       return SoftPFReqString;
      case SoftPFResp:      return SoftPFRespString;
      case HardPFReq:       return HardPFReqString;
      case HardPFResp:      return HardPFRespString;
      case InvalidateReq:   return InvalidateReqString;
      case WriteInvalidateReq:return WriteInvalidateReqString;
      case WriteInvalidateResp:return WriteInvalidateRespString;
      case UpgradeReq:      return UpgradeReqString;
      case ReadExReq:       return ReadExReqString;
      case ReadExResp:      return ReadExRespString;
      default:              return OtherCmdString;
    }
}

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
fixPacket(PacketPtr func, PacketPtr timing)
{
    Addr funcStart      = func->getAddr();
    Addr funcEnd        = func->getAddr() + func->getSize() - 1;
    Addr timingStart    = timing->getAddr();
    Addr timingEnd      = timing->getAddr() + timing->getSize() - 1;

    assert(!(funcStart > timingEnd || timingStart < funcEnd));

    if (DTRACE(FunctionalAccess)) {
       DebugOut() << func;
       DebugOut() << timing;
    }

    // this packet can't solve our problem, continue on
    if (!timing->hasData())
        return true;

    if (func->isRead()) {
        if (funcStart >= timingStart && funcEnd <= timingEnd) {
            func->allocate();
            memcpy(func->getPtr<uint8_t>(), timing->getPtr<uint8_t>() +
                    funcStart - timingStart, func->getSize());
            func->result = Packet::Success;
            return false;
        } else {
            // In this case the timing packet only partially satisfies the
            // requset, so we would need more information to make this work.
            // Like bytes valid in the packet or something, so the request could
            // continue and get this bit of possibly newer data along with the
            // older data not written to yet.
            panic("Timing packet only partially satisfies the functional"
                    "request. Now what?");
        }
    } else if (func->isWrite()) {
        if (funcStart >= timingStart) {
            memcpy(timing->getPtr<uint8_t>() + (funcStart - timingStart),
                   func->getPtr<uint8_t>(),
                   funcStart - std::min(funcEnd, timingEnd));
        } else { // timingStart > funcStart
            memcpy(timing->getPtr<uint8_t>(),
                   func->getPtr<uint8_t>() + (timingStart - funcStart),
                   timingStart - std::min(funcEnd, timingEnd));
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
        o << "Read ";
    if (p.isInvalidate())
        o << "Read ";
    if (p.isRequest())
        o << "Request ";
    if (p.isResponse())
        o << "Response ";
    if (p.hasData())
        o << "w/Data ";

    o << std::endl;
    return o;
}


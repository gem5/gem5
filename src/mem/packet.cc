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
#include "base/misc.hh"
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
Packet::intersect(Packet *p)
{
    Addr s1 = getAddr();
    Addr e1 = getAddr() + getSize() - 1;
    Addr s2 = p->getAddr();
    Addr e2 = p->getAddr() + p->getSize() - 1;

    return !(s1 > e2 || e1 < s2);
}

bool
fixPacket(Packet *func, Packet *timing)
{
    panic("Need to implement!");
}

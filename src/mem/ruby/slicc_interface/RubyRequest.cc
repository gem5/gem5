/*
 * Copyright (c) 2011 Mark D. Hill and David A. Wood
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

#include <iostream>

#include "mem/ruby/slicc_interface/RubyRequest.hh"

using namespace std;

void
RubyRequest::print(ostream& out) const
{
  out << "[RubyRequest: ";
  out << "LineAddress = " << m_LineAddress << " ";
  out << "PhysicalAddress = " << m_PhysicalAddress << " ";
  out << "Type = " << m_Type << " ";
  out << "ProgramCounter = " << m_ProgramCounter << " ";
  out << "AccessMode = " << m_AccessMode << " ";
  out << "Size = " << m_Size << " ";
  out << "Prefetch = " << m_Prefetch << " ";
//  out << "Time = " << getTime() << " ";
  out << "]";
}

bool
RubyRequest::functionalRead(Packet *pkt)
{
    // This needs a little explanation. Initially I thought that this
    // message should be read. But the way the memtester works for now,
    // we should not be reading this message as memtester updates the
    // functional memory only after a write has actually taken place.
    return false;
}

bool
RubyRequest::functionalWrite(Packet *pkt)
{
    // This needs a little explanation. I am not sure if this message
    // should be written. Essentially the question is how are writes
    // ordered. I am assuming that if a functional write is issued after
    // a timing write to the same address, then the functional write
    // has to overwrite the data for the timing request, even if the
    // timing request has still not been ordered globally.

    Addr wBase = pkt->getAddr();
    Addr wTail = wBase + pkt->getSize();
    Addr mBase = m_PhysicalAddress.getAddress();
    Addr mTail = mBase + m_Size;

    uint8_t * pktData = pkt->getPtr<uint8_t>(true);

    Addr cBase = std::max(wBase, mBase);
    Addr cTail = std::min(wTail, mTail);

    for (Addr i = cBase; i < cTail; ++i) {
        data[i - mBase] = pktData[i - wBase];
    }

    return cBase < cTail;
}

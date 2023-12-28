/*
 * Copyright (c) 2019,2021 ARM Limited
 * All rights reserved.
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

#include "mem/ruby/slicc_interface/RubyRequest.hh"

#include <iostream>

#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"

namespace gem5
{

namespace ruby
{

void
RubyRequest::print(std::ostream& out) const
{
  out << "[RubyRequest: ";
  out << std::hex << "LineAddress = 0x" << m_LineAddress << std::dec << " ";
  out << std::hex << "PhysicalAddress = 0x" << m_PhysicalAddress;
  out << std::dec << " " << "Type = " << m_Type << " ";
  out << std::hex << "ProgramCounter = 0x" << m_ProgramCounter << std::dec;
  out << " " << "AccessMode = " << m_AccessMode << " ";
  out << "Size = " << m_Size << " ";
  out << "Prefetch = " << m_Prefetch << " ";
  out << "isGLCSet = " << m_isGLCSet << "";
  out << "isSLCSet = " << m_isSLCSet << "";
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
RubyRequest::functionalRead(Packet *pkt, WriteMask &mask)
{
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

    if (!pkt->hasData() || !m_pkt->hasData())
        return false;

    uint8_t *data =  m_pkt->getPtr<uint8_t>();

    if (pkt->isMaskedWrite() || m_pkt->isMaskedWrite()) {
        warn("Skiping functional write to/from a masked write packet"
            " (addr: %#x, other addr: %#x).\n", m_PhysicalAddress,
              pkt->getAddr());
        return false;
    }

    Addr wBase = pkt->getAddr();
    Addr wTail = wBase + pkt->getSize();
    Addr mBase = m_PhysicalAddress;
    Addr mTail = mBase + m_Size;

    const uint8_t * pktData = pkt->getConstPtr<uint8_t>();

    Addr cBase = std::max(wBase, mBase);
    Addr cTail = std::min(wTail, mTail);

    for (Addr i = cBase; i < cTail; ++i) {
        data[i - mBase] = pktData[i - wBase];
    }

    // also overwrite the WTData
    testAndWrite(m_PhysicalAddress, m_WTData, pkt);

    return cBase < cTail;
}

void
RubyRequest::setWriteMask(uint32_t offset, uint32_t len,
        std::vector< std::pair<int,AtomicOpFunctor*>> atomicOps)
{
    m_writeMask.setMask(offset, len);
    m_writeMask.setAtomicOps(atomicOps);
}


} // namespace ruby
} // namespace gem5

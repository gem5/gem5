/*
 * Copyright (c) 2009 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SLICC_INTERFACE_RUBYREQUEST_HH__
#define __MEM_RUBY_SLICC_INTERFACE_RUBYREQUEST_HH__

#include <ostream>
#include <vector>

#include "mem/protocol/HSAScope.hh"
#include "mem/protocol/HSASegment.hh"
#include "mem/protocol/Message.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/protocol/RubyAccessMode.hh"
#include "mem/protocol/RubyRequestType.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/common/WriteMask.hh"

class RubyRequest : public Message
{
  public:
    Addr m_PhysicalAddress;
    Addr m_LineAddress;
    RubyRequestType m_Type;
    Addr m_ProgramCounter;
    RubyAccessMode m_AccessMode;
    int m_Size;
    PrefetchBit m_Prefetch;
    uint8_t* data;
    PacketPtr m_pkt;
    ContextID m_contextId;
    WriteMask m_writeMask;
    DataBlock m_WTData;
    int m_wfid;
    HSAScope m_scope;
    HSASegment m_segment;


    RubyRequest(Tick curTime, uint64_t _paddr, uint8_t* _data, int _len,
        uint64_t _pc, RubyRequestType _type, RubyAccessMode _access_mode,
        PacketPtr _pkt, PrefetchBit _pb = PrefetchBit_No,
        ContextID _proc_id = 100, ContextID _core_id = 99,
        HSAScope _scope = HSAScope_UNSPECIFIED,
        HSASegment _segment = HSASegment_GLOBAL)
        : Message(curTime),
          m_PhysicalAddress(_paddr),
          m_Type(_type),
          m_ProgramCounter(_pc),
          m_AccessMode(_access_mode),
          m_Size(_len),
          m_Prefetch(_pb),
          data(_data),
          m_pkt(_pkt),
          m_contextId(_core_id),
          m_scope(_scope),
          m_segment(_segment)
    {
        m_LineAddress = makeLineAddress(m_PhysicalAddress);
    }

    RubyRequest(Tick curTime, uint64_t _paddr, uint8_t* _data, int _len,
        uint64_t _pc, RubyRequestType _type,
        RubyAccessMode _access_mode, PacketPtr _pkt, PrefetchBit _pb,
        unsigned _proc_id, unsigned _core_id,
        int _wm_size, std::vector<bool> & _wm_mask,
        DataBlock & _Data,
        HSAScope _scope = HSAScope_UNSPECIFIED,
        HSASegment _segment = HSASegment_GLOBAL)
        : Message(curTime),
          m_PhysicalAddress(_paddr),
          m_Type(_type),
          m_ProgramCounter(_pc),
          m_AccessMode(_access_mode),
          m_Size(_len),
          m_Prefetch(_pb),
          data(_data),
          m_pkt(_pkt),
          m_contextId(_core_id),
          m_writeMask(_wm_size,_wm_mask),
          m_WTData(_Data),
          m_wfid(_proc_id),
          m_scope(_scope),
          m_segment(_segment)
    {
        m_LineAddress = makeLineAddress(m_PhysicalAddress);
    }

    RubyRequest(Tick curTime, uint64_t _paddr, uint8_t* _data, int _len,
        uint64_t _pc, RubyRequestType _type,
        RubyAccessMode _access_mode, PacketPtr _pkt, PrefetchBit _pb,
        unsigned _proc_id, unsigned _core_id,
        int _wm_size, std::vector<bool> & _wm_mask,
        DataBlock & _Data,
        std::vector< std::pair<int,AtomicOpFunctor*> > _atomicOps,
        HSAScope _scope = HSAScope_UNSPECIFIED,
        HSASegment _segment = HSASegment_GLOBAL)
        : Message(curTime),
          m_PhysicalAddress(_paddr),
          m_Type(_type),
          m_ProgramCounter(_pc),
          m_AccessMode(_access_mode),
          m_Size(_len),
          m_Prefetch(_pb),
          data(_data),
          m_pkt(_pkt),
          m_contextId(_core_id),
          m_writeMask(_wm_size,_wm_mask,_atomicOps),
          m_WTData(_Data),
          m_wfid(_proc_id),
          m_scope(_scope),
          m_segment(_segment)
    {
        m_LineAddress = makeLineAddress(m_PhysicalAddress);
    }


    RubyRequest(Tick curTime) : Message(curTime) {}
    MsgPtr clone() const
    { return std::shared_ptr<Message>(new RubyRequest(*this)); }

    Addr getLineAddress() const { return m_LineAddress; }
    Addr getPhysicalAddress() const { return m_PhysicalAddress; }
    const RubyRequestType& getType() const { return m_Type; }
    Addr getProgramCounter() const { return m_ProgramCounter; }
    const RubyAccessMode& getAccessMode() const { return m_AccessMode; }
    const int& getSize() const { return m_Size; }
    const PrefetchBit& getPrefetch() const { return m_Prefetch; }

    void print(std::ostream& out) const;
    bool functionalRead(Packet *pkt);
    bool functionalWrite(Packet *pkt);
};

inline std::ostream&
operator<<(std::ostream& out, const RubyRequest& obj)
{
  obj.print(out);
  out << std::flush;
  return out;
}

#endif  //__MEM_RUBY_SLICC_INTERFACE_RUBYREQUEST_HH__

/*
 * Copyright (c) 2020-2021, 2024 Arm Limited
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

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/common/WriteMask.hh"
#include "mem/ruby/protocol/Message.hh"
#include "mem/ruby/protocol/PrefetchBit.hh"
#include "mem/ruby/protocol/RubyAccessMode.hh"
#include "mem/ruby/protocol/RubyRequestType.hh"

namespace gem5
{

namespace ruby
{

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
    PacketPtr m_pkt;
    ContextID m_contextId;
    WriteMask m_writeMask;
    DataBlock m_WTData;
    int m_wfid;
    uint64_t m_instSeqNum;
    bool m_htmFromTransaction;
    uint64_t m_htmTransactionUid;
    bool m_isTlbi;
    // Should be uint64, but SLICC complains about casts
    Addr m_tlbiTransactionUid;
    // GPU cache bypass flags. GLC bypasses L1 while SLC bypasses both L1 and
    // L2 if set to true. They are set to false by default and they must be
    // explicitly set to true in the program in order to bypass caches
    bool m_isGLCSet;
    bool m_isSLCSet;
    bool m_isSecure;

    RubyRequest(Tick curTime, int block_size, uint64_t _paddr, int _len,
        uint64_t _pc, RubyRequestType _type, RubyAccessMode _access_mode,
        PacketPtr _pkt, PrefetchBit _pb = PrefetchBit_No,
        ContextID _proc_id = 100, ContextID _core_id = 99)
        : Message(curTime, block_size),
          m_PhysicalAddress(_paddr),
          m_Type(_type),
          m_ProgramCounter(_pc),
          m_AccessMode(_access_mode),
          m_Size(_len),
          m_Prefetch(_pb),
          m_pkt(_pkt),
          m_contextId(_core_id),
          m_writeMask(block_size),
          m_WTData(block_size),
          m_htmFromTransaction(false),
          m_htmTransactionUid(0),
          m_isTlbi(false),
          m_tlbiTransactionUid(0),
          m_isSecure(m_pkt ? m_pkt->req->isSecure() : false)
    {
        int block_size_bits = floorLog2(block_size);
        m_LineAddress = makeLineAddress(m_PhysicalAddress, block_size_bits);
        if (_pkt) {
            m_isGLCSet = m_pkt->req->isGLCSet();
            m_isSLCSet = m_pkt->req->isSLCSet();
        } else {
            m_isGLCSet = 0;
            m_isSLCSet = 0;
        }
    }

    /** RubyRequest for memory management commands */
    RubyRequest(Tick curTime, int block_size,
        uint64_t _pc, RubyRequestType _type, RubyAccessMode _access_mode,
        PacketPtr _pkt, ContextID _proc_id, ContextID _core_id)
        : Message(curTime, block_size),
          m_PhysicalAddress(0),
          m_Type(_type),
          m_ProgramCounter(_pc),
          m_AccessMode(_access_mode),
          m_Size(0),
          m_Prefetch(PrefetchBit_No),
          m_pkt(_pkt),
          m_contextId(_core_id),
          m_writeMask(block_size),
          m_WTData(block_size),
          m_htmFromTransaction(false),
          m_htmTransactionUid(0),
          m_isTlbi(false),
          m_tlbiTransactionUid(0),
          m_isSecure(m_pkt->req->isSecure())
    {
        assert(m_pkt->req->isMemMgmt());
        if (_pkt) {
            m_isGLCSet = m_pkt->req->isGLCSet();
            m_isSLCSet = m_pkt->req->isSLCSet();
        } else {
            m_isGLCSet = 0;
            m_isSLCSet = 0;
        }
    }

    RubyRequest(Tick curTime, int block_size, uint64_t _paddr, int _len,
        uint64_t _pc, RubyRequestType _type,
        RubyAccessMode _access_mode, PacketPtr _pkt, PrefetchBit _pb,
        unsigned _proc_id, unsigned _core_id,
        int _wm_size, std::vector<bool> & _wm_mask,
        DataBlock & _Data,
        uint64_t _instSeqNum = 0)
        : Message(curTime, block_size),
          m_PhysicalAddress(_paddr),
          m_Type(_type),
          m_ProgramCounter(_pc),
          m_AccessMode(_access_mode),
          m_Size(_len),
          m_Prefetch(_pb),
          m_pkt(_pkt),
          m_contextId(_core_id),
          m_writeMask(_wm_size,_wm_mask),
          m_WTData(_Data),
          m_wfid(_proc_id),
          m_instSeqNum(_instSeqNum),
          m_htmFromTransaction(false),
          m_htmTransactionUid(0),
          m_isTlbi(false),
          m_tlbiTransactionUid(0),
          m_isSecure(m_pkt->req->isSecure())
    {
        int block_size_bits = floorLog2(block_size);
        m_LineAddress = makeLineAddress(m_PhysicalAddress, block_size_bits);
        if (_pkt) {
            m_isGLCSet = m_pkt->req->isGLCSet();
            m_isSLCSet = m_pkt->req->isSLCSet();
        } else {
            m_isGLCSet = 0;
            m_isSLCSet = 0;
        }
    }

    RubyRequest(Tick curTime, int block_size, uint64_t _paddr, int _len,
        uint64_t _pc, RubyRequestType _type,
        RubyAccessMode _access_mode, PacketPtr _pkt, PrefetchBit _pb,
        unsigned _proc_id, unsigned _core_id,
        int _wm_size, std::vector<bool> & _wm_mask,
        DataBlock & _Data,
        std::vector< std::pair<int,AtomicOpFunctor*> > _atomicOps,
        uint64_t _instSeqNum = 0)
        : Message(curTime, block_size),
          m_PhysicalAddress(_paddr),
          m_Type(_type),
          m_ProgramCounter(_pc),
          m_AccessMode(_access_mode),
          m_Size(_len),
          m_Prefetch(_pb),
          m_pkt(_pkt),
          m_contextId(_core_id),
          m_writeMask(_wm_size,_wm_mask,_atomicOps),
          m_WTData(_Data),
          m_wfid(_proc_id),
          m_instSeqNum(_instSeqNum),
          m_htmFromTransaction(false),
          m_htmTransactionUid(0),
          m_isTlbi(false),
          m_tlbiTransactionUid(0),
          m_isSecure(m_pkt->req->isSecure())
    {
        int block_size_bits = floorLog2(block_size);
        m_LineAddress = makeLineAddress(m_PhysicalAddress, block_size_bits);
        if (_pkt) {
            m_isGLCSet = m_pkt->req->isGLCSet();
            m_isSLCSet = m_pkt->req->isSLCSet();

        } else {
            m_isGLCSet = 0;
            m_isSLCSet = 0;
        }
    }

    RubyRequest(Tick curTime, int block_size)
        : Message(curTime, block_size),
          m_writeMask(block_size),
          m_WTData(block_size)
    {
    }
    MsgPtr clone() const
    { return std::shared_ptr<Message>(new RubyRequest(*this)); }

    Addr getLineAddress() const { return m_LineAddress; }
    Addr getPhysicalAddress() const { return m_PhysicalAddress; }
    const RubyRequestType& getType() const { return m_Type; }
    Addr getProgramCounter() const { return m_ProgramCounter; }
    const RubyAccessMode& getAccessMode() const { return m_AccessMode; }
    const int& getSize() const { return m_Size; }
    const PrefetchBit& getPrefetch() const { return m_Prefetch; }
    RequestPtr getRequestPtr() const { return m_pkt->req; }

    void setWriteMask(uint32_t offset, uint32_t len,
        std::vector< std::pair<int,AtomicOpFunctor*>> atomicOps);
    void print(std::ostream& out) const;
    bool functionalRead(Packet *pkt);
    bool functionalRead(Packet *pkt, WriteMask &mask);
    bool functionalWrite(Packet *pkt);
};

inline std::ostream&
operator<<(std::ostream& out, const RubyRequest& obj)
{
  obj.print(out);
  out << std::flush;
  return out;
}

} // namespace ruby
} // namespace gem5

#endif  //__MEM_RUBY_SLICC_INTERFACE_RUBYREQUEST_HH__

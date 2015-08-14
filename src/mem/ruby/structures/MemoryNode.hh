/*
 * Copyright (c) 2008 Mark D. Hill and David A. Wood
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

/*
 * Description:
 * This structure records everything known about a single
 * memory request that is queued in the memory controller.
 * It is created when the memory request first arrives
 * at a memory controller and is deleted when the underlying
 * message is enqueued to be sent back to the directory.
 */

#ifndef __MEM_RUBY_STRUCTURES_MEMORYNODE_HH__
#define __MEM_RUBY_STRUCTURES_MEMORYNODE_HH__

#include <iostream>

#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/slicc_interface/Message.hh"

class MemoryNode
{
  public:
    // old constructor
    MemoryNode(const Cycles& time, int counter, const PacketPtr p,
               Addr addr, const bool is_mem_read)
        : m_time(time), pkt(p)
    {
        m_msg_counter = counter;
        m_addr = addr;
        m_is_mem_read = is_mem_read;
        m_is_dirty_wb = !is_mem_read;
    }

    // new constructor
    MemoryNode(const Cycles& time, const PacketPtr p,
               Addr addr, const bool is_mem_read,
               const bool is_dirty_wb)
        : m_time(time), pkt(p)
    {
        m_msg_counter = 0;
        m_addr = addr;
        m_is_mem_read = is_mem_read;
        m_is_dirty_wb = is_dirty_wb;
    }

    void print(std::ostream& out) const;

    Cycles m_time;
    int m_msg_counter;
    PacketPtr pkt;
    Addr m_addr;
    bool m_is_mem_read;
    bool m_is_dirty_wb;
};

inline std::ostream&
operator<<(std::ostream& out, const MemoryNode& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_STRUCTURES_MEMORYNODE_HH__

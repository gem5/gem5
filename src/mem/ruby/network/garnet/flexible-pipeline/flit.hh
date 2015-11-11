/*
 * Copyright (c) 2008 Princeton University
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
 * Authors: Niket Agarwal
 */

#include <cassert>
#include <iostream>

#include "base/types.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/slicc_interface/Message.hh"

#ifndef __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_FLIT_HH__
#define __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_FLIT_HH__

class flit
{
  public:
    flit(int id, int vc, int vnet, int size, MsgPtr msg_ptr, Cycles curTime);

    int get_size() const { return m_size; }
    int get_id() const { return m_id; }
    Cycles get_time() const { return m_time; }
    Cycles get_creation_time() const { return m_creation_time; }
    void set_time(Cycles time) { m_time = time; }
    int get_vnet() const { return m_vnet; }
    int get_vc() const { return m_vc; }
    void set_vc(int vc) { m_vc = vc; }
    MsgPtr& get_msg_ptr() { return m_msg_ptr; }
    flit_type get_type() const { return m_type; }
    void set_delay(Cycles delay) { src_delay = delay; }
    Cycles get_delay() const { return src_delay; }
    void print(std::ostream& out) const;

    static bool
    greater(flit* n1, flit* n2)
    {
        if (n1->get_time() == n2->get_time())
            //assert(n1->flit_id != n2->flit_id);
            return (n1->get_id() > n2->get_id());
        else
            return (n1->get_time() > n2->get_time());
    }

    bool functionalRead(Packet *pkt);
    bool functionalWrite(Packet *pkt);

  private:
    const int m_id;
    const int m_vnet;
    int m_vc;
    const int m_size;
    const Cycles m_creation_time;
    Cycles m_time;
    flit_type m_type;
    MsgPtr m_msg_ptr;
    Cycles src_delay;
};

inline std::ostream&
operator<<(std::ostream& out, const flit& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_FLIT_HH__

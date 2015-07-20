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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_FLIT_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_FLIT_D_HH__

#include <cassert>
#include <iostream>

#include "base/types.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/slicc_interface/Message.hh"

class flit_d
{
  public:
    flit_d(int id, int vc, int vnet, int size, MsgPtr msg_ptr, Cycles curTime);
    flit_d(int vc, bool is_free_signal, Cycles curTime);
    void set_outport(int port) { m_outport = port; }
    int get_outport() {return m_outport; }
    void print(std::ostream& out) const;
    bool is_free_signal() { return m_is_free_signal; }
    int get_size() { return m_size; }
    Cycles get_enqueue_time() { return m_enqueue_time; }
    int get_id() { return m_id; }
    Cycles get_time() { return m_time; }
    void set_time(Cycles time) { m_time = time; }
    int get_vnet() { return m_vnet; }
    int get_vc() { return m_vc; }
    void set_vc(int vc) { m_vc = vc; }
    MsgPtr& get_msg_ptr() { return m_msg_ptr; }
    flit_type get_type() { return m_type; }

    bool
    is_stage(flit_stage t_stage, Cycles curTime)
    {
        return (m_stage.first == t_stage &&
                curTime >= m_stage.second);
    }

    void
    advance_stage(flit_stage t_stage, Cycles newTime)
    {
        m_stage.first = t_stage;
        m_stage.second = newTime;
    }

    std::pair<flit_stage, Cycles> get_stage() { return m_stage; }

    void set_delay(Cycles delay) { src_delay = delay; }
    Cycles get_delay() { return src_delay; }

    static bool
    greater(flit_d* n1, flit_d* n2)
    {
        if (n1->get_time() == n2->get_time()) {
            //assert(n1->flit_id != n2->flit_id);
            return (n1->get_id() > n2->get_id());
        } else {
            return (n1->get_time() > n2->get_time());
        }
    }

    bool functionalWrite(Packet *pkt);

  private:
    int m_id;
    int m_vnet;
    int m_vc;
    int m_size;
    bool m_is_free_signal;
    Cycles m_enqueue_time, m_time;
    flit_type m_type;
    MsgPtr m_msg_ptr;
    int m_outport;
    Cycles src_delay;
    std::pair<flit_stage, Cycles> m_stage;
};

inline std::ostream&
operator<<(std::ostream& out, const flit_d& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_FLIT_D_HH__

/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
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
 *          Tushar Krishna
 */


#ifndef __MEM_RUBY_NETWORK_GARNET_VIRTUAL_CHANNEL_HH__
#define __MEM_RUBY_NETWORK_GARNET_VIRTUAL_CHANNEL_HH__

#include <utility>

#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/flitBuffer.hh"

class VirtualChannel
{
  public:
    VirtualChannel(int id);
    ~VirtualChannel();

    bool need_stage(flit_stage stage, Cycles time);
    void set_idle(Cycles curTime);
    void set_active(Cycles curTime);
    void set_outvc(int outvc)               { m_output_vc = outvc; }
    inline int get_outvc()                  { return m_output_vc; }
    void set_outport(int outport)           { m_output_port = outport; };
    inline int get_outport()                  { return m_output_port; }

    inline Cycles get_enqueue_time()          { return m_enqueue_time; }
    inline void set_enqueue_time(Cycles time) { m_enqueue_time = time; }
    inline VC_state_type get_state()        { return m_vc_state.first; }

    inline bool isReady(Cycles curTime)
    {
        return m_input_buffer->isReady(curTime);
    }

    inline void
    insertFlit(flit *t_flit)
    {
        m_input_buffer->insert(t_flit);
    }

    inline void
    set_state(VC_state_type m_state, Cycles curTime)
    {
        m_vc_state.first = m_state;
        m_vc_state.second = curTime;
    }

    inline flit*
    peekTopFlit()
    {
        return m_input_buffer->peekTopFlit();
    }

    inline flit*
    getTopFlit()
    {
        return m_input_buffer->getTopFlit();
    }

    uint32_t functionalWrite(Packet *pkt);

  private:
    int m_id;
    flitBuffer *m_input_buffer;
    std::pair<VC_state_type, Cycles> m_vc_state;
    int m_output_port;
    Cycles m_enqueue_time;
    int m_output_vc;
};

#endif // __MEM_RUBY_NETWORK_GARNET_VIRTUAL_CHANNEL_HH__

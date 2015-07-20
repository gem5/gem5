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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_VIRTUAL_CHANNEL_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_VIRTUAL_CHANNEL_D_HH__

#include <utility>

#include "mem/ruby/network/garnet/fixed-pipeline/flitBuffer_d.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"

class VirtualChannel_d
{
  public:
    VirtualChannel_d(int id);
    ~VirtualChannel_d();

    bool need_stage(VC_state_type state, flit_stage stage, Cycles curTime);
    void set_outport(int outport);
    void grant_vc(int out_vc, Cycles curTime);

    inline Cycles get_enqueue_time()          { return m_enqueue_time; }
    inline void set_enqueue_time(Cycles time) { m_enqueue_time = time; }
    inline VC_state_type get_state()        { return m_vc_state.first; }
    inline int get_outvc()                  { return m_output_vc; }
    inline bool has_credits()               { return (m_credit_count > 0); }
    inline int get_route()                  { return route; }
    inline void update_credit(int credit)   { m_credit_count = credit; }
    inline void increment_credit()          { m_credit_count++; }

    inline bool isReady(Cycles curTime)
    {
        return m_input_buffer->isReady(curTime);
    }

    inline void
    insertFlit(flit_d *t_flit)
    {
        m_input_buffer->insert(t_flit);
    }

    inline void
    set_state(VC_state_type m_state, Cycles curTime)
    {
        m_vc_state.first = m_state;
        m_vc_state.second = curTime;
    }

    inline flit_d*
    peekTopFlit()
    {
        return m_input_buffer->peekTopFlit();
    }

    inline flit_d*
    getTopFlit()
    {
        return m_input_buffer->getTopFlit();
    }

    uint32_t functionalWrite(Packet *pkt);

  private:
    int m_id;
    flitBuffer_d *m_input_buffer;
    std::pair<VC_state_type, Cycles> m_vc_state; // I/R/V/A/C
    int route;
    Cycles m_enqueue_time;
    int m_output_vc;
    int m_credit_count;
};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_VIRTUAL_CHANNEL_D_HH__

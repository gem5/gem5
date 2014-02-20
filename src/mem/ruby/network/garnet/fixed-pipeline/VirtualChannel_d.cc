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

#include "mem/ruby/network/garnet/fixed-pipeline/VirtualChannel_d.hh"

VirtualChannel_d::VirtualChannel_d(int id)
    : m_enqueue_time(INFINITE_)
{
    m_id = id;
    m_input_buffer = new flitBuffer_d();
    m_vc_state.first = IDLE_;
    m_vc_state.second = Cycles(0);
}

VirtualChannel_d::~VirtualChannel_d()
{
    delete m_input_buffer;
}

void
VirtualChannel_d::set_outport(int outport)
{
    route = outport;
}

void
VirtualChannel_d::grant_vc(int out_vc, Cycles curTime)
{
    m_output_vc = out_vc;
    m_vc_state.first = ACTIVE_;
    m_vc_state.second = curTime + Cycles(1);
    flit_d *t_flit = m_input_buffer->peekTopFlit();
    t_flit->advance_stage(SA_, curTime);
}

bool
VirtualChannel_d::need_stage(VC_state_type state, flit_stage stage,
                             Cycles ct)
{
    if ((m_vc_state.first == state) && (ct >= m_vc_state.second)) {
        if (m_input_buffer->isReady(ct)) {
            flit_d *t_flit = m_input_buffer->peekTopFlit();
            return(t_flit->is_stage(stage, ct)) ;
        }
    }
    return false;
}

uint32_t
VirtualChannel_d::functionalWrite(Packet *pkt)
{
    return m_input_buffer->functionalWrite(pkt);
}

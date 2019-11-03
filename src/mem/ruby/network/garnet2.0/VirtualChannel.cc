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


#include "mem/ruby/network/garnet2.0/VirtualChannel.hh"

VirtualChannel::VirtualChannel(int id)
    : m_enqueue_time(INFINITE_)
{
    m_id = id;
    m_input_buffer = new flitBuffer();
    m_vc_state.first = IDLE_;
    m_vc_state.second = Cycles(0);
    m_output_vc = -1;
    m_output_port = -1;
}

VirtualChannel::~VirtualChannel()
{
    delete m_input_buffer;
}

void
VirtualChannel::set_idle(Cycles curTime)
{
    m_vc_state.first = IDLE_;
    m_vc_state.second = curTime;
    m_enqueue_time = Cycles(INFINITE_);
    m_output_port = -1;
    m_output_vc = -1;
}

void
VirtualChannel::set_active(Cycles curTime)
{
    m_vc_state.first = ACTIVE_;
    m_vc_state.second = curTime;
    m_enqueue_time = curTime;
}

bool
VirtualChannel::need_stage(flit_stage stage, Cycles time)
{
    if (m_input_buffer->isReady(time)) {
        assert(m_vc_state.first == ACTIVE_ && m_vc_state.second <= time);
        flit *t_flit = m_input_buffer->peekTopFlit();
        return(t_flit->is_stage(stage, time));
    }
    return false;
}

uint32_t
VirtualChannel::functionalWrite(Packet *pkt)
{
    return m_input_buffer->functionalWrite(pkt);
}

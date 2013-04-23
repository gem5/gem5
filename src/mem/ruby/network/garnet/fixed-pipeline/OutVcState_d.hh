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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_OUT_VC_STATE_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_OUT_VC_STATE_D_HH__

#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"

class OutVcState_d
{
  public:
    OutVcState_d(int id, GarnetNetwork_d *network_ptr);

    int get_inport()                { return m_in_port; }
    int get_invc()                  { return m_in_vc; }
    int get_credit_count()          { return m_credit_count; }

    void set_inport(int port)       { m_in_port = port; }
    void set_invc(int vc)           { m_in_vc = vc; }
    inline bool
    isInState(VC_state_type state, Cycles request_time)
    {
        return ((m_vc_state == state) && (request_time >= m_time) );
    }
    inline void
    setState(VC_state_type state, Cycles time)
    {
        m_vc_state = state;
        m_time = time;
    }
    inline bool has_credits()       { return (m_credit_count > 0); }
    inline void increment_credit()  { m_credit_count++; }
    inline void decrement_credit()  { m_credit_count--; }

  private:
    int m_id ;
    Cycles m_time;
    VC_state_type m_vc_state;
    int m_in_port;
    int m_in_vc;
    int m_credit_count;
};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_OUT_VC_STATE_D_HH__

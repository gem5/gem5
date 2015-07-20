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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_OUTPUT_UNIT_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_OUTPUT_UNIT_D_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/OutVcState_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/flitBuffer_d.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"

class OutputUnit_d : public Consumer
{
  public:
    OutputUnit_d(int id, Router_d *router);
    ~OutputUnit_d();
    void set_out_link(NetworkLink_d *link);
    void set_credit_link(CreditLink_d *credit_link);
    void wakeup();
    flitBuffer_d* getOutQueue();
    void update_vc(int vc, int in_port, int in_vc);
    void print(std::ostream& out) const {};
    void decrement_credit(int out_vc);

    int
    get_credit_cnt(int vc)
    {
        return m_outvc_state[vc]->get_credit_count();
    }

    inline int
    get_outlink_id()
    {
        return m_out_link->get_id();
    }

    inline void
    set_vc_state(VC_state_type state, int vc, Cycles curTime)
    {
      m_outvc_state[vc]->setState(state, curTime);
    }

    inline bool
    is_vc_idle(int vc, Cycles curTime)
    {
        return (m_outvc_state[vc]->isInState(IDLE_, curTime));
    }

    inline void
    insert_flit(flit_d *t_flit)
    {
        m_out_buffer->insert(t_flit);
        m_out_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
    }

    uint32_t functionalWrite(Packet *pkt);

  private:
    int m_id;
    int m_num_vcs;
    Router_d *m_router;
    NetworkLink_d *m_out_link;
    CreditLink_d *m_credit_link;

    flitBuffer_d *m_out_buffer; // This is for the network link to consume
    std::vector<OutVcState_d *> m_outvc_state; // vc state of downstream router

};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_OUTPUT_UNIT_D_HH__

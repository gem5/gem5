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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_INPUT_UNIT_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_INPUT_UNIT_D_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/VirtualChannel_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/flitBuffer_d.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"

class Router_d;

class InputUnit_d : public Consumer
{
  public:
    InputUnit_d(int id, Router_d *router);
    ~InputUnit_d();

    void wakeup();
    void printConfig(std::ostream& out);
    flitBuffer_d* getCreditQueue() { return creditQueue; }
    void print(std::ostream& out) const {};

    inline int get_inlink_id() { return m_in_link->get_id(); }

    inline void
    set_vc_state(VC_state_type state, int vc)
    {
        m_vcs[vc]->set_state(state);
    }

    inline void
    set_enqueue_time(int invc, Time time)
    {
        m_vcs[invc]->set_enqueue_time(time);
    }

    inline Time
    get_enqueue_time(int invc)
    {
        return m_vcs[invc]->get_enqueue_time();
    }

    inline void
    update_credit(int in_vc, int credit)
    {
        m_vcs[in_vc]->update_credit(credit);
    }

    inline bool
    has_credits(int vc)
    {
        return m_vcs[vc]->has_credits();
    }

    inline void
    increment_credit(int in_vc, bool free_signal)
    {
        flit_d *t_flit = new flit_d(in_vc, free_signal);
        creditQueue->insert(t_flit);
        g_eventQueue_ptr->scheduleEvent(m_credit_link, 1);
    }

    inline int
    get_outvc(int invc)
    {
        return m_vcs[invc]->get_outvc();
    }

    inline void
    updateRoute(int vc, int outport)
    {
        m_vcs[vc]->set_outport(outport);
        m_vcs[vc]->set_state(VC_AB_);
    }

    inline void
    grant_vc(int in_vc, int out_vc)
    {
        m_vcs[in_vc]->grant_vc(out_vc);
    }

    inline flit_d*
    peekTopFlit(int vc)
    {
        return m_vcs[vc]->peekTopFlit();
    }

    inline flit_d*
    getTopFlit(int vc)
    {
        return m_vcs[vc]->getTopFlit();
    }

    inline bool
    need_stage(int vc, VC_state_type state, flit_stage stage)
    {
        return m_vcs[vc]->need_stage(state, stage);
    }

    inline bool
    need_stage_nextcycle(int vc, VC_state_type state, flit_stage stage)
    {
        return m_vcs[vc]->need_stage_nextcycle(state, stage);
    }

    inline bool
    isReady(int invc)
    {
        return m_vcs[invc]->isReady();
    }

    inline int
    get_route(int vc)
    {
        return m_vcs[vc]->get_route();
    }

    inline void
    set_in_link(NetworkLink_d *link)
    {
        m_in_link = link;
    }

    inline void
    set_credit_link(CreditLink_d *credit_link)
    {
        m_credit_link = credit_link;
    }

    inline double
    get_buf_read_count(int vnet)
    {
        return m_num_buffer_reads[vnet];
    }

    inline double
    get_buf_write_count(int vnet)
    {
        return m_num_buffer_writes[vnet];
    }

  private:
    int m_id;
    int m_num_vcs;
    int m_vc_per_vnet;
    std::vector<double> m_num_buffer_writes;
    std::vector<double> m_num_buffer_reads;

    Router_d *m_router;
    NetworkLink_d *m_in_link;
    CreditLink_d *m_credit_link;
    flitBuffer_d *creditQueue;

    // Virtual channels
    std::vector<VirtualChannel_d *> m_vcs;
};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_INPUT_UNIT_D_HH__

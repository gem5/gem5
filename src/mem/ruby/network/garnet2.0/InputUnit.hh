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


#ifndef __MEM_RUBY_NETWORK_GARNET_INPUT_UNIT_HH__
#define __MEM_RUBY_NETWORK_GARNET_INPUT_UNIT_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"
#include "mem/ruby/network/garnet2.0/VirtualChannel.hh"
#include "mem/ruby/network/garnet2.0/flitBuffer.hh"

class InputUnit : public Consumer
{
  public:
    InputUnit(int id, PortDirection direction, Router *router);
    ~InputUnit();

    void wakeup();
    void print(std::ostream& out) const {};

    inline PortDirection get_direction() { return m_direction; }

    inline void
    set_vc_idle(int vc, Cycles curTime)
    {
        m_vcs[vc]->set_idle(curTime);
    }

    inline void
    set_vc_active(int vc, Cycles curTime)
    {
        m_vcs[vc]->set_active(curTime);
    }

    inline void
    grant_outport(int vc, int outport)
    {
        m_vcs[vc]->set_outport(outport);
    }

    inline void
    grant_outvc(int vc, int outvc)
    {
        m_vcs[vc]->set_outvc(outvc);
    }

    inline int
    get_outport(int invc)
    {
        return m_vcs[invc]->get_outport();
    }

    inline int
    get_outvc(int invc)
    {
        return m_vcs[invc]->get_outvc();
    }

    inline Cycles
    get_enqueue_time(int invc)
    {
        return m_vcs[invc]->get_enqueue_time();
    }

    void increment_credit(int in_vc, bool free_signal, Cycles curTime);

    inline flit*
    peekTopFlit(int vc)
    {
        return m_vcs[vc]->peekTopFlit();
    }

    inline flit*
    getTopFlit(int vc)
    {
        return m_vcs[vc]->getTopFlit();
    }

    inline bool
    need_stage(int vc, flit_stage stage, Cycles time)
    {
        return m_vcs[vc]->need_stage(stage, time);
    }

    inline bool
    isReady(int invc, Cycles curTime)
    {
        return m_vcs[invc]->isReady(curTime);
    }

    flitBuffer* getCreditQueue() { return creditQueue; }

    inline void
    set_in_link(NetworkLink *link)
    {
        m_in_link = link;
    }

    inline int get_inlink_id() { return m_in_link->get_id(); }

    inline void
    set_credit_link(CreditLink *credit_link)
    {
        m_credit_link = credit_link;
    }

    double get_buf_read_activity(unsigned int vnet) const
    { return m_num_buffer_reads[vnet]; }
    double get_buf_write_activity(unsigned int vnet) const
    { return m_num_buffer_writes[vnet]; }

    uint32_t functionalWrite(Packet *pkt);
    void resetStats();

  private:
    int m_id;
    PortDirection m_direction;
    int m_num_vcs;
    int m_vc_per_vnet;

    Router *m_router;
    NetworkLink *m_in_link;
    CreditLink *m_credit_link;
    flitBuffer *creditQueue;

    // Input Virtual channels
    std::vector<VirtualChannel *> m_vcs;

    // Statistical variables
    std::vector<double> m_num_buffer_writes;
    std::vector<double> m_num_buffer_reads;
};

#endif // __MEM_RUBY_NETWORK_GARNET_INPUT_UNIT_HH__

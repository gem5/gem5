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

#include "base/stl_helpers.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/OutputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

OutputUnit_d::OutputUnit_d(int id, Router_d *router)
    : Consumer(router)
{
    m_id = id;
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_out_buffer = new flitBuffer_d();

    for (int i = 0; i < m_num_vcs; i++) {
        m_outvc_state.push_back(new OutVcState_d(i, m_router->get_net_ptr()));
    }
}

OutputUnit_d::~OutputUnit_d()
{
    delete m_out_buffer;
    deletePointers(m_outvc_state);
}

void
OutputUnit_d::decrement_credit(int out_vc)
{
    m_outvc_state[out_vc]->decrement_credit();
    m_router->update_incredit(m_outvc_state[out_vc]->get_inport(),
                              m_outvc_state[out_vc]->get_invc(),
                              m_outvc_state[out_vc]->get_credit_count());
}

void
OutputUnit_d::wakeup()
{
    if (m_credit_link->isReady(m_router->curCycle())) {
        flit_d *t_flit = m_credit_link->consumeLink();
        int out_vc = t_flit->get_vc();
        m_outvc_state[out_vc]->increment_credit();
        m_router->update_incredit(m_outvc_state[out_vc]->get_inport(),
                                  m_outvc_state[out_vc]->get_invc(),
                                  m_outvc_state[out_vc]->get_credit_count());

        if (t_flit->is_free_signal())
            set_vc_state(IDLE_, out_vc, m_router->curCycle());

        delete t_flit;
    }
}

flitBuffer_d*
OutputUnit_d::getOutQueue()
{
    return m_out_buffer;
}

void
OutputUnit_d::set_out_link(NetworkLink_d *link)
{
    m_out_link = link;
}

void
OutputUnit_d::set_credit_link(CreditLink_d *credit_link)
{
    m_credit_link = credit_link;
}

void
OutputUnit_d::update_vc(int vc, int in_port, int in_vc)
{
  m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());
    m_outvc_state[vc]->set_inport(in_port);
    m_outvc_state[vc]->set_invc(in_vc);
    m_router->update_incredit(in_port, in_vc,
                              m_outvc_state[vc]->get_credit_count());
}

uint32_t
OutputUnit_d::functionalWrite(Packet *pkt)
{
    return m_out_buffer->functionalWrite(pkt);
}

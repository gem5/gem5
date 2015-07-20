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
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/OutputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Switch_d.hh"

using m5::stl_helpers::deletePointers;

Switch_d::Switch_d(Router_d *router)
    : Consumer(router)
{
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_crossbar_activity = 0;
}

Switch_d::~Switch_d()
{
    deletePointers(m_switch_buffer);
}

void
Switch_d::init()
{
    m_output_unit = m_router->get_outputUnit_ref();

    m_num_inports = m_router->get_num_inports();
    m_switch_buffer.resize(m_num_inports);
    for (int i = 0; i < m_num_inports; i++) {
        m_switch_buffer[i] = new flitBuffer_d();
    }
}

void
Switch_d::wakeup()
{
    DPRINTF(RubyNetwork, "Switch woke up at time: %lld\n",
            m_router->curCycle());

    for (int inport = 0; inport < m_num_inports; inport++) {
        if (!m_switch_buffer[inport]->isReady(m_router->curCycle()))
            continue;
        flit_d *t_flit = m_switch_buffer[inport]->peekTopFlit();
        if (t_flit->is_stage(ST_, m_router->curCycle())) {
            int outport = t_flit->get_outport();
            t_flit->advance_stage(LT_, m_router->curCycle());
            t_flit->set_time(m_router->curCycle());

            // This will take care of waking up the Network Link
            m_output_unit[outport]->insert_flit(t_flit);
            m_switch_buffer[inport]->getTopFlit();
            m_crossbar_activity++;
        }
    }
    check_for_wakeup();
}

void
Switch_d::check_for_wakeup()
{
    Cycles nextCycle = m_router->curCycle() + Cycles(1);

    for (int inport = 0; inport < m_num_inports; inport++) {
        if (m_switch_buffer[inport]->isReady(nextCycle)) {
            m_router->vcarb_req();
            break;
        }
    }
}

uint32_t
Switch_d::functionalWrite(Packet *pkt)
{
   uint32_t num_functional_writes = 0;

   for (uint32_t i = 0; i < m_switch_buffer.size(); ++i) {
       num_functional_writes += m_switch_buffer[i]->functionalWrite(pkt);
   }

   return num_functional_writes;
}

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

#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/InputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/OutputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SWallocator_d.hh"

SWallocator_d::SWallocator_d(Router_d *router)
    : Consumer(router)
{
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();

    m_local_arbiter_activity = 0;
    m_global_arbiter_activity = 0;
}

void
SWallocator_d::init()
{
    m_input_unit = m_router->get_inputUnit_ref();
    m_output_unit = m_router->get_outputUnit_ref();

    m_num_inports = m_router->get_num_inports();
    m_num_outports = m_router->get_num_outports();
    m_round_robin_outport.resize(m_num_outports);
    m_round_robin_inport.resize(m_num_inports);
    m_port_req.resize(m_num_outports);
    m_vc_winners.resize(m_num_outports);

    for (int i = 0; i < m_num_inports; i++) {
        m_round_robin_inport[i] = 0;
    }

    for (int i = 0; i < m_num_outports; i++) {
        m_port_req[i].resize(m_num_inports);
        m_vc_winners[i].resize(m_num_inports);

        m_round_robin_outport[i] = 0;

        for (int j = 0; j < m_num_inports; j++) {
            m_port_req[i][j] = false; // [outport][inport]
        }
    }
}

void
SWallocator_d::wakeup()
{
    arbitrate_inports(); // First stage of allocation
    arbitrate_outports(); // Second stage of allocation

    clear_request_vector();
    check_for_wakeup();
    m_router->call_switch();

}

void
SWallocator_d::arbitrate_inports()
{
    // First do round robin arbitration on a set of input vc requests
    for (int inport = 0; inport < m_num_inports; inport++) {
        int invc = m_round_robin_inport[inport];

        // Select next round robin vc candidate within valid vnet
        int next_round_robin_invc = invc;
        next_round_robin_invc++;
        if (next_round_robin_invc >= m_num_vcs)
            next_round_robin_invc = 0;
        m_round_robin_inport[inport] = next_round_robin_invc;

        for (int invc_iter = 0; invc_iter < m_num_vcs; invc_iter++) {
            invc++;
            if (invc >= m_num_vcs)
                invc = 0;

            if (m_input_unit[inport]->need_stage(invc, ACTIVE_, SA_,
                                                 m_router->curCycle()) &&
                m_input_unit[inport]->has_credits(invc)) {

                if (is_candidate_inport(inport, invc)) {
                    int outport = m_input_unit[inport]->get_route(invc);
                    m_local_arbiter_activity++;
                    m_port_req[outport][inport] = true;
                    m_vc_winners[outport][inport]= invc;
                    break; // got one vc winner for this port
                }
            }
        }
    }
}

bool
SWallocator_d::is_candidate_inport(int inport, int invc)
{
    int outport = m_input_unit[inport]->get_route(invc);
    Cycles t_enqueue_time = m_input_unit[inport]->get_enqueue_time(invc);
    int t_vnet = get_vnet(invc);
    int vc_base = t_vnet*m_vc_per_vnet;
    if ((m_router->get_net_ptr())->isVNetOrdered(t_vnet)) {
        for (int vc_offset = 0; vc_offset < m_vc_per_vnet; vc_offset++) {
            int temp_vc = vc_base + vc_offset;
            if (m_input_unit[inport]->need_stage(temp_vc, ACTIVE_, SA_,
                                                 m_router->curCycle()) &&
               (m_input_unit[inport]->get_route(temp_vc) == outport) &&
               (m_input_unit[inport]->get_enqueue_time(temp_vc) <
                    t_enqueue_time)) {
                return false;
                break;
            }
        }
    }
    return true;
}


void
SWallocator_d::arbitrate_outports()
{
    // Now there are a set of input vc requests for output vcs.
    // Again do round robin arbitration on these requests
    for (int outport = 0; outport < m_num_outports; outport++) {
        int inport = m_round_robin_outport[outport];
        m_round_robin_outport[outport]++;

        if (m_round_robin_outport[outport] >= m_num_outports)
            m_round_robin_outport[outport] = 0;

        for (int inport_iter = 0; inport_iter < m_num_inports; inport_iter++) {
            inport++;
            if (inport >= m_num_inports)
                inport = 0;

            // inport has a request this cycle for outport:
            if (m_port_req[outport][inport]) {
                m_port_req[outport][inport] = false;
                int invc = m_vc_winners[outport][inport];
                int outvc = m_input_unit[inport]->get_outvc(invc);

                // remove flit from Input Unit
                flit_d *t_flit = m_input_unit[inport]->getTopFlit(invc);
                t_flit->advance_stage(ST_, m_router->curCycle());
                t_flit->set_vc(outvc);
                t_flit->set_outport(outport);
                t_flit->set_time(m_router->curCycle());

                m_output_unit[outport]->decrement_credit(outvc);
                m_router->update_sw_winner(inport, t_flit);
                m_global_arbiter_activity++;

                if ((t_flit->get_type() == TAIL_) ||
                    t_flit->get_type() == HEAD_TAIL_) {

                    // Send a credit back
                    // along with the information that this VC is now idle
                    m_input_unit[inport]->increment_credit(invc, true,
                        m_router->curCycle());

                    // This Input VC should now be empty
                    assert(m_input_unit[inport]->isReady(invc,
                        m_router->curCycle()) == false);

                    m_input_unit[inport]->set_vc_state(IDLE_, invc,
                        m_router->curCycle());
                    m_input_unit[inport]->set_enqueue_time(invc,
                        Cycles(INFINITE_));
                } else {
                    // Send a credit back
                    // but do not indicate that the VC is idle
                    m_input_unit[inport]->increment_credit(invc, false,
                        m_router->curCycle());
                }
                break; // got a in request for this outport
            }
        }
    }
}

void
SWallocator_d::check_for_wakeup()
{
    Cycles nextCycle = m_router->curCycle() + Cycles(1);

    for (int i = 0; i < m_num_inports; i++) {
        for (int j = 0; j < m_num_vcs; j++) {
            if (m_input_unit[i]->need_stage(j, ACTIVE_, SA_, nextCycle)) {
                m_router->vcarb_req();
                return;
            }
        }
    }
}

int
SWallocator_d::get_vnet(int invc)
{
    int vnet = invc/m_vc_per_vnet;
    assert(vnet < m_router->get_num_vnets());
    return vnet;
}

void
SWallocator_d::clear_request_vector()
{
    for (int i = 0; i < m_num_outports; i++) {
        for (int j = 0; j < m_num_inports; j++) {
            m_port_req[i][j] = false;
        }
    }
}

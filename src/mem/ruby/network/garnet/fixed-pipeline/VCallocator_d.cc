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
#include "mem/ruby/network/garnet/fixed-pipeline/VCallocator_d.hh"

VCallocator_d::VCallocator_d(Router_d *router)
    : Consumer(router)
{
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();

    m_local_arbiter_activity.resize(m_num_vcs/m_vc_per_vnet);
    m_global_arbiter_activity.resize(m_num_vcs/m_vc_per_vnet);
    for (int i = 0; i < m_local_arbiter_activity.size(); i++) {
        m_local_arbiter_activity[i] = 0;
        m_global_arbiter_activity[i] = 0;
    }
}

void
VCallocator_d::init()
{
    m_input_unit = m_router->get_inputUnit_ref();
    m_output_unit = m_router->get_outputUnit_ref();

    m_num_inports = m_router->get_num_inports();
    m_num_outports = m_router->get_num_outports();
    m_round_robin_invc.resize(m_num_inports);
    m_round_robin_outvc.resize(m_num_outports);
    m_outvc_req.resize(m_num_outports);
    m_outvc_is_req.resize(m_num_outports);

    for (int i = 0; i < m_num_inports; i++) {
        m_round_robin_invc[i].resize(m_num_vcs);

        for (int j = 0; j < m_num_vcs; j++) {
            m_round_robin_invc[i][j] = 0;
        }
    }

    for (int i = 0; i < m_num_outports; i++) {
        m_round_robin_outvc[i].resize(m_num_vcs);
        m_outvc_req[i].resize(m_num_vcs);
        m_outvc_is_req[i].resize(m_num_vcs);

        for (int j = 0; j < m_num_vcs; j++) {
            m_round_robin_outvc[i][j].first = 0;
            m_round_robin_outvc[i][j].second = 0;
            m_outvc_is_req[i][j] = false;

            m_outvc_req[i][j].resize(m_num_inports);

            for (int k = 0; k < m_num_inports; k++) {
                m_outvc_req[i][j][k].resize(m_num_vcs);
                for (int l = 0; l < m_num_vcs; l++) {
                    m_outvc_req[i][j][k][l] = false;
                }
            }
        }
    }
}

void
VCallocator_d::clear_request_vector()
{
    for (int i = 0; i < m_num_outports; i++) {
        for (int j = 0; j < m_num_vcs; j++) {
            if (!m_outvc_is_req[i][j])
                continue;
            m_outvc_is_req[i][j] = false;
            for (int k = 0; k < m_num_inports; k++) {
                for (int l = 0; l < m_num_vcs; l++) {
                    m_outvc_req[i][j][k][l] = false;
                }
            }
        }
    }
}

void
VCallocator_d::wakeup()
{
    arbitrate_invcs(); // First stage of allocation
    arbitrate_outvcs(); // Second stage of allocation

    clear_request_vector();
    check_for_wakeup();
    m_router->call_sw_alloc();
}

bool
VCallocator_d::is_invc_candidate(int inport_iter, int invc_iter)
{
    int outport = m_input_unit[inport_iter]->get_route(invc_iter);
    int vnet = get_vnet(invc_iter);
    Cycles t_enqueue_time =
        m_input_unit[inport_iter]->get_enqueue_time(invc_iter);

    int invc_base = vnet*m_vc_per_vnet;

    if ((m_router->get_net_ptr())->isVNetOrdered(vnet)) {
        for (int vc_offset = 0; vc_offset < m_vc_per_vnet; vc_offset++) {
            int temp_vc = invc_base + vc_offset;
            if (m_input_unit[inport_iter]->need_stage(temp_vc, VC_AB_, VA_,
                                                      m_router->curCycle()) &&
               (m_input_unit[inport_iter]->get_route(temp_vc) == outport) &&
               (m_input_unit[inport_iter]->get_enqueue_time(temp_vc) <
                    t_enqueue_time)) {
                return false;
            }
        }
    }
    return true;
}

void
VCallocator_d::select_outvc(int inport_iter, int invc_iter)
{
    int outport = m_input_unit[inport_iter]->get_route(invc_iter);
    int vnet = get_vnet(invc_iter);
    int outvc_base = vnet*m_vc_per_vnet;
    int num_vcs_per_vnet = m_vc_per_vnet;

    int outvc_offset = m_round_robin_invc[inport_iter][invc_iter];
    m_round_robin_invc[inport_iter][invc_iter]++;

    if (m_round_robin_invc[inport_iter][invc_iter] >= num_vcs_per_vnet)
        m_round_robin_invc[inport_iter][invc_iter] = 0;

    for (int outvc_offset_iter = 0; outvc_offset_iter < num_vcs_per_vnet;
            outvc_offset_iter++) {
        outvc_offset++;
        if (outvc_offset >= num_vcs_per_vnet)
            outvc_offset = 0;
        int outvc = outvc_base + outvc_offset;
        if (m_output_unit[outport]->is_vc_idle(outvc, m_router->curCycle())) {
            m_local_arbiter_activity[vnet]++;
            m_outvc_req[outport][outvc][inport_iter][invc_iter] = true;
            if (!m_outvc_is_req[outport][outvc])
                m_outvc_is_req[outport][outvc] = true;
            return; // out vc acquired
        }
    }
}

void
VCallocator_d::arbitrate_invcs()
{
    for (int inport_iter = 0; inport_iter < m_num_inports; inport_iter++) {
        for (int invc_iter = 0; invc_iter < m_num_vcs; invc_iter++) {
            if (m_input_unit[inport_iter]->need_stage(invc_iter, VC_AB_,
                    VA_, m_router->curCycle())) {
                if (!is_invc_candidate(inport_iter, invc_iter))
                    continue;

                select_outvc(inport_iter, invc_iter);
            }
        }
    }
}

void
VCallocator_d::arbitrate_outvcs()
{
    for (int outport_iter = 0; outport_iter < m_num_outports; outport_iter++) {
        for (int outvc_iter = 0; outvc_iter < m_num_vcs; outvc_iter++) {
            if (!m_outvc_is_req[outport_iter][outvc_iter]) {
                // No requests for this outvc in this cycle
                continue;
            }

            int inport = m_round_robin_outvc[outport_iter][outvc_iter].first;
            int invc_offset =
                m_round_robin_outvc[outport_iter][outvc_iter].second;
            int vnet = get_vnet(outvc_iter);
            int invc_base = vnet*m_vc_per_vnet;
            int num_vcs_per_vnet = m_vc_per_vnet;

            m_round_robin_outvc[outport_iter][outvc_iter].second++;
            if (m_round_robin_outvc[outport_iter][outvc_iter].second >=
               num_vcs_per_vnet) {
                m_round_robin_outvc[outport_iter][outvc_iter].second = 0;
                m_round_robin_outvc[outport_iter][outvc_iter].first++;
                if (m_round_robin_outvc[outport_iter][outvc_iter].first >=
                   m_num_inports)
                    m_round_robin_outvc[outport_iter][outvc_iter].first = 0;
            }
            for (int in_iter = 0; in_iter < m_num_inports*num_vcs_per_vnet;
                    in_iter++) {
                invc_offset++;
                if (invc_offset >= num_vcs_per_vnet) {
                    invc_offset = 0;
                    inport++;
                    if (inport >= m_num_inports)
                        inport = 0;
                }
                int invc = invc_base + invc_offset;
                if (m_outvc_req[outport_iter][outvc_iter][inport][invc]) {
                    m_global_arbiter_activity[vnet]++;
                    m_input_unit[inport]->grant_vc(invc, outvc_iter,
                        m_router->curCycle());
                    m_output_unit[outport_iter]->update_vc(
                        outvc_iter, inport, invc);
                    break;
                }
            }
        }
    }
}

int
VCallocator_d::get_vnet(int invc)
{
    int vnet = invc/m_vc_per_vnet;
    assert(vnet < m_router->get_num_vnets());

    return vnet;
}

void
VCallocator_d::check_for_wakeup()
{
    Cycles nextCycle = m_router->curCycle() + Cycles(1);

    for (int i = 0; i < m_num_inports; i++) {
        for (int j = 0; j < m_num_vcs; j++) {
            if (m_input_unit[i]->need_stage(j, VC_AB_, VA_, nextCycle)) {
                m_router->vcarb_req();
                return;
            }
        }
    }
}

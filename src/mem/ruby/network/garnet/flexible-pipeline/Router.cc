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

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/InVcState.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/OutVcState.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/Router.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/VCarbiter.hh"
#include "mem/ruby/slicc_interface/Message.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

Router::Router(const Params *p)
    : BasicRouter(p), FlexibleConsumer(this)
{
    m_virtual_networks = p->virt_nets;
    m_vc_per_vnet = p->vcs_per_vnet;
    m_round_robin_inport = 0;
    m_round_robin_start = 0;
    m_num_vcs = m_vc_per_vnet * m_virtual_networks;
    m_vc_arbiter = new VCarbiter(this);
}

Router::~Router()
{
    for (int i = 0; i < m_in_link.size(); i++) {
        deletePointers(m_in_vc_state[i]);
    }
    for (int i = 0; i < m_out_link.size(); i++) {
        deletePointers(m_out_vc_state[i]);
        deletePointers(m_router_buffers[i]);
    }
    deletePointers(m_out_src_queue);
    delete m_vc_arbiter;
}

void
Router::addInPort(NetworkLink *in_link)
{
    int port = m_in_link.size();
    vector<InVcState *> in_vc_vector;
    for (int i = 0; i < m_num_vcs; i++) {
        in_vc_vector.push_back(new InVcState(i));
        in_vc_vector[i]->setState(IDLE_, curCycle());
    }
    m_in_vc_state.push_back(in_vc_vector);
    m_in_link.push_back(in_link);
    in_link->setLinkConsumer(this);
    in_link->setInPort(port);

    int start = 0;
    m_round_robin_invc.push_back(start);
}

void
Router::addOutPort(NetworkLink *out_link, const NetDest& routing_table_entry,
                   int link_weight)
{
    int port = m_out_link.size();
    out_link->setOutPort(port);
    int start = 0;
    m_vc_round_robin.push_back(start);

    m_out_src_queue.push_back(new flitBuffer());

    m_out_link.push_back(out_link);
    m_routing_table.push_back(routing_table_entry);
    out_link->setSourceQueue(m_out_src_queue[port]);
    out_link->setSource(this);

    vector<flitBuffer *> intermediateQueues;
    for (int i = 0; i < m_num_vcs; i++) {
        int buffer_size = m_net_ptr->getBufferSize();
        if (buffer_size > 0) // finite size
            intermediateQueues.push_back(new flitBuffer(buffer_size));
        else // infinite size
            intermediateQueues.push_back(new flitBuffer());
    }
    m_router_buffers.push_back(intermediateQueues);

    vector<OutVcState *> out_vc_vector;
    for (int i = 0; i < m_num_vcs; i++) {
        out_vc_vector.push_back(new OutVcState(i));
        out_vc_vector[i]->setState(IDLE_, curCycle());
    }
    m_out_vc_state.push_back(out_vc_vector);
    m_link_weights.push_back(link_weight);
}

bool
Router::isBufferNotFull(int vc, int inport)
{
    int outport = m_in_vc_state[inport][vc]->get_outport();
    int outvc = m_in_vc_state[inport][vc]->get_outvc();

    return (!m_router_buffers[outport][outvc]->isFull());
}

// A request for an output vc has been placed by an upstream Router/NI.
// This has to be updated and arbitration performed
void
Router::request_vc(int in_vc, int in_port, NetDest destination,
                   Cycles request_time)
{
    assert(m_in_vc_state[in_port][in_vc]->isInState(IDLE_, request_time));

    int outport = getRoute(destination);
    m_in_vc_state[in_port][in_vc]->setRoute(outport);
    m_in_vc_state[in_port][in_vc]->setState(VC_AB_, request_time);
    assert(request_time >= curCycle());
    if (request_time > curCycle())
        m_vc_arbiter->scheduleEventAbsolute(clockPeriod() * request_time);
    else
        vc_arbitrate();
}

void
Router::vc_arbitrate()
{
    int inport = m_round_robin_inport;
    m_round_robin_inport++;
    if (m_round_robin_inport == m_in_link.size())
        m_round_robin_inport = 0;

    for (int port_iter = 0; port_iter < m_in_link.size(); port_iter++) {
        inport++;
        if (inport >= m_in_link.size())
            inport = 0;
        int invc = m_round_robin_invc[inport];
        m_round_robin_invc[inport] = get_next_round_robin_vc(invc);

        for (int vc_iter = 0; vc_iter < m_num_vcs; vc_iter++) {
            invc++;
            if (invc >= m_num_vcs)
                invc = 0;

            InVcState *in_vc_state = m_in_vc_state[inport][invc];

            if (in_vc_state->isInState(VC_AB_, curCycle())) {
                int outport = in_vc_state->get_outport();
                vector<int> valid_vcs = get_valid_vcs(invc);
                for (int valid_vc_iter = 0; valid_vc_iter < valid_vcs.size();
                        valid_vc_iter++) {
                    if (m_out_vc_state[outport][valid_vcs[valid_vc_iter]]
                            ->isInState(IDLE_, curCycle())) {

                        in_vc_state->grant_vc(valid_vcs[valid_vc_iter],
                                curCycle());

                        m_in_link[inport]->grant_vc_link(invc, curCycle());

                        m_out_vc_state[outport][valid_vcs[valid_vc_iter]]
                            ->setState(VC_AB_, curCycle());
                        break;
                    }
                }
            }
        }
    }
}

vector<int>
Router::get_valid_vcs(int invc)
{
    vector<int> vc_list;

    for (int vnet = 0; vnet < m_virtual_networks; vnet++) {
        if (invc >= (vnet*m_vc_per_vnet) && invc < ((vnet+1)*m_vc_per_vnet)) {
            int base = vnet*m_vc_per_vnet;
            int vc_per_vnet;
            if (m_net_ptr->isVNetOrdered(vnet))
                vc_per_vnet = 1;
            else
                vc_per_vnet = m_vc_per_vnet;

            for (int offset = 0; offset < vc_per_vnet; offset++) {
                vc_list.push_back(base+offset);
            }
            break;
        }
    }
    return vc_list;
}

void
Router::grant_vc(int out_port, int vc, Cycles grant_time)
{
    assert(m_out_vc_state[out_port][vc]->isInState(VC_AB_, grant_time));
    m_out_vc_state[out_port][vc]->grant_vc(grant_time);
    scheduleEvent(Cycles(1));
}

void
Router::release_vc(int out_port, int vc, Cycles release_time)
{
    assert(m_out_vc_state[out_port][vc]->isInState(ACTIVE_, release_time));
    m_out_vc_state[out_port][vc]->setState(IDLE_, release_time);
    scheduleEvent(Cycles(1));
}

// This function calculated the output port for a particular destination.
int
Router::getRoute(NetDest destination)
{
    int output_link = -1;
    int min_weight = INFINITE_;
    for (int link = 0; link < m_routing_table.size(); link++) {
        if (destination.intersectionIsNotEmpty(m_routing_table[link])) {
            if ((m_link_weights[link] >= min_weight))
                continue;
            output_link = link;
            min_weight = m_link_weights[link];
        }
    }
    return output_link;
}

void
Router::routeCompute(flit *m_flit, int inport)
{
    int invc = m_flit->get_vc();
    int outport = m_in_vc_state[inport][invc]->get_outport();
    int outvc = m_in_vc_state[inport][invc]->get_outvc();

    assert(m_net_ptr->getNumPipeStages() >= 1);

    // Subtract 1 as 1 cycle will be consumed in scheduling the output link
    m_flit->set_time(curCycle() + Cycles((m_net_ptr->getNumPipeStages() - 1)));
    m_flit->set_vc(outvc);
    m_router_buffers[outport][outvc]->insert(m_flit);

    if (m_net_ptr->getNumPipeStages() > 1)
        scheduleEvent(Cycles(m_net_ptr->getNumPipeStages() - 1));

    if ((m_flit->get_type() == HEAD_) || (m_flit->get_type() == HEAD_TAIL_)) {
        Message *nm = m_flit->get_msg_ptr().get();
        NetDest destination = nm->getDestination();

        if (m_net_ptr->getNumPipeStages() > 1) {
            m_out_vc_state[outport][outvc]->setState(VC_AB_, curCycle() +
                                                     Cycles(1));
            m_out_link[outport]->request_vc_link(outvc, destination,
                                                 curCycle() + Cycles(1));
        } else {
            m_out_vc_state[outport][outvc]->setState(VC_AB_, curCycle());
            m_out_link[outport]->request_vc_link(outvc, destination,
                curCycle());
        }
    }

    if ((m_flit->get_type() == TAIL_) || (m_flit->get_type() == HEAD_TAIL_)) {
        m_in_vc_state[inport][invc]->setState(IDLE_, curCycle() + Cycles(1));
        m_in_link[inport]->release_vc_link(invc, curCycle() + Cycles(1));
    }
}

void
Router::wakeup()
{
    flit *t_flit;

    // This is for round-robin scheduling of incoming ports
    int incoming_port = m_round_robin_start;
    m_round_robin_start++;
    if (m_round_robin_start >= m_in_link.size()) {
        m_round_robin_start = 0;
    }

    for (int port = 0; port < m_in_link.size(); port++) {
        // Round robin scheduling
        incoming_port++;
        if (incoming_port >= m_in_link.size())
            incoming_port = 0;

        // checking the incoming link
        if (m_in_link[incoming_port]->isReady()) {
            DPRINTF(RubyNetwork, "m_id: %d, Time: %lld\n", m_id, curCycle());
            t_flit = m_in_link[incoming_port]->peekLink();
            routeCompute(t_flit, incoming_port);
            m_in_link[incoming_port]->consumeLink();
        }
    }
    scheduleOutputLinks();
    checkReschedule(); // This is for flits lying in the router buffers
    vc_arbitrate();
    check_arbiter_reschedule();
}

void
Router::scheduleOutputLinks()
{
    for (int port = 0; port < m_out_link.size(); port++) {
        int vc_tolookat = m_vc_round_robin[port];
        m_vc_round_robin[port] = get_next_round_robin_vc(vc_tolookat);

        for (int i = 0; i < m_num_vcs; i++) {
            vc_tolookat++;
            if (vc_tolookat == m_num_vcs)
                vc_tolookat = 0;

            if (m_router_buffers[port][vc_tolookat]->isReady(curCycle())) {

                // models buffer backpressure
                if (m_out_vc_state[port][vc_tolookat]->isInState(ACTIVE_,
                   curCycle()) &&
                   m_out_link[port]->isBufferNotFull_link(vc_tolookat)) {

                    flit *t_flit =
                        m_router_buffers[port][vc_tolookat]->getTopFlit();
                    t_flit->set_time(curCycle() + Cycles(1));
                    m_out_src_queue[port]->insert(t_flit);

                    m_out_link[port]->
                        scheduleEventAbsolute(clockEdge(Cycles(1)));
                    break; // done for this port
                }
            }
        }
    }
}

int
Router::get_vnet(int vc) const
{
    int vnet = vc/m_vc_per_vnet;
    assert(vnet < m_virtual_networks);
    return vnet;
}

int
Router::get_next_round_robin_vc(int vc) const
{
    vc++;
    if (vc == m_num_vcs)
        vc = 0;
    return vc;
}

void
Router::checkReschedule()
{
    for (int port = 0; port < m_out_link.size(); port++) {
        for (int vc = 0; vc < m_num_vcs; vc++) {
            if (m_router_buffers[port][vc]->isReady(curCycle() + Cycles(1))) {
                scheduleEvent(Cycles(1));
                return;
            }
        }
    }
}

void
Router::check_arbiter_reschedule()
{
    for (int port = 0; port < m_in_link.size(); port++) {
        for (int vc = 0; vc < m_num_vcs; vc++) {
            if (m_in_vc_state[port][vc]->isInState(VC_AB_, curCycle() +
                                                   Cycles(1))) {
                m_vc_arbiter->scheduleEventAbsolute(clockEdge(Cycles(1)));
                return;
            }
        }
    }
}

bool
Router::functionalRead(Packet *pkt)
{
    // Access the buffers in the router for performing a functional read
    for (unsigned int i = 0; i < m_router_buffers.size(); i++) {
        for (unsigned int j = 0; j < m_router_buffers[i].size(); ++j) {
            if (m_router_buffers[i][j]->functionalRead(pkt)) {
                return true;
            }
        }
    }

    // Access the link queues for performing a functional read
    for (unsigned int i = 0; i < m_out_src_queue.size(); i++) {
        if (m_out_src_queue[i]->functionalRead(pkt)) {
            return true;
        }
    }
    return false;
}

uint32_t
Router::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    // Access the buffers in the router for performing a functional write
    for (unsigned int i = 0; i < m_router_buffers.size(); i++) {
        for (unsigned int j = 0; j < m_router_buffers[i].size(); ++j) {
            num_functional_writes +=
                m_router_buffers[i][j]->functionalWrite(pkt);
        }
    }

    // Access the link queues for performing a functional write
    for (unsigned int i = 0; i < m_out_src_queue.size(); i++) {
        num_functional_writes += m_out_src_queue[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

void
Router::print(ostream& out) const
{
    out << "[Router]";
}

Router *
GarnetRouterParams::create()
{
    return new Router(this);
}

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

#include <cassert>
#include <cmath>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/NetworkInterface.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/flitBuffer.hh"
#include "mem/ruby/slicc_interface/NetworkMessage.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

NetworkInterface::NetworkInterface(int id, int virtual_networks,
                                   GarnetNetwork *network_ptr)
    : FlexibleConsumer(network_ptr)
{
    m_id = id;
    m_net_ptr = network_ptr;
    m_virtual_networks  = virtual_networks;
    m_vc_per_vnet = m_net_ptr->getVCsPerVnet();
    m_num_vcs = m_vc_per_vnet*m_virtual_networks;

    m_vc_round_robin = 0;
    m_ni_buffers.resize(m_num_vcs);
    inNode_ptr.resize(m_virtual_networks);
    outNode_ptr.resize(m_virtual_networks);

    // instantiating the NI flit buffers
    for (int i =0; i < m_num_vcs; i++)
        m_ni_buffers[i] = new flitBuffer();

    m_vc_allocator.resize(m_virtual_networks);
    for (int i = 0; i < m_virtual_networks; i++) {
        m_vc_allocator[i] = 0;
    }

    for (int i = 0; i < m_num_vcs; i++) {
        m_out_vc_state.push_back(new OutVcState(i));
    }
}

NetworkInterface::~NetworkInterface()
{
    deletePointers(m_out_vc_state);
    deletePointers(m_ni_buffers);
    delete outSrcQueue;
}

void
NetworkInterface::addInPort(NetworkLink *in_link)
{
    inNetLink = in_link;
    in_link->setLinkConsumer(this);
}

void
NetworkInterface::addOutPort(NetworkLink *out_link)
{
    outNetLink = out_link;
    outSrcQueue = new flitBuffer();
    out_link->setSourceQueue(outSrcQueue);
    out_link->setSource(this);
}

void
NetworkInterface::addNode(vector<MessageBuffer*>& in,
    vector<MessageBuffer*>& out)
{
    assert(in.size() == m_virtual_networks);
    inNode_ptr = in;
    outNode_ptr = out;

    // protocol injects messages into the NI
    for (int j = 0; j < m_virtual_networks; j++) {
        inNode_ptr[j]->setConsumer(this);
        inNode_ptr[j]->setReceiver(m_net_ptr);

        outNode_ptr[j]->setSender(m_net_ptr);
    }
}

void
NetworkInterface::request_vc(int in_vc, int in_port, NetDest destination,
                             Cycles request_time)
{
    inNetLink->grant_vc_link(in_vc, request_time);
}

bool
NetworkInterface::flitisizeMessage(MsgPtr msg_ptr, int vnet)
{
    NetworkMessage *net_msg_ptr = safe_cast<NetworkMessage *>(msg_ptr.get());
    NetDest net_msg_dest = net_msg_ptr->getInternalDestination();

    // get all the destinations associated with this message.
    vector<NodeID> dest_nodes = net_msg_dest.getAllDest();

    // Number of flits is dependent on the link bandwidth available.
    // This is expressed in terms of bytes/cycle or the flit size

    int num_flits = (int) ceil((double) m_net_ptr->MessageSizeType_to_int(
                net_msg_ptr->getMessageSize())/m_net_ptr->getNiFlitSize());

    // loop to convert all multicast messages into unicast messages
    for (int ctr = 0; ctr < dest_nodes.size(); ctr++) {
        int vc = calculateVC(vnet); // this will return a free output vc

        if (vc == -1) {
            // did not find a free output vc
            return false ;
        }
        MsgPtr new_msg_ptr = msg_ptr->clone();
        NodeID destID = dest_nodes[ctr];

        NetworkMessage *new_net_msg_ptr =
            safe_cast<NetworkMessage *>(new_msg_ptr.get());
        if (dest_nodes.size() > 1) {
            NetDest personal_dest;
            for (int m = 0; m < (int) MachineType_NUM; m++) {
                if ((destID >= MachineType_base_number((MachineType) m)) &&
                    destID < MachineType_base_number((MachineType) (m+1))) {
                    // calculating the NetDest associated with this destID
                    personal_dest.clear();
                    personal_dest.add((MachineID) {(MachineType) m, (destID -
                        MachineType_base_number((MachineType) m))});
                    new_net_msg_ptr->getInternalDestination() = personal_dest;
                    break;
                }
            }
            net_msg_dest.removeNetDest(personal_dest);

            // removing the destination from the original message to reflect
            // that a message with this particular destination has been
            // flitisized and an output vc is acquired
            net_msg_ptr->getInternalDestination().removeNetDest(personal_dest);
        }
        for (int i = 0; i < num_flits; i++) {
            m_net_ptr->increment_injected_flits(vnet);
            flit *fl = new flit(i, vc, vnet, num_flits, new_msg_ptr,
                                m_net_ptr->curCycle());
            fl->set_delay(m_net_ptr->curCycle() -
                          m_net_ptr->ticksToCycles(msg_ptr->getTime()));
            m_ni_buffers[vc]->insert(fl);
        }

        m_out_vc_state[vc]->setState(VC_AB_, m_net_ptr->curCycle());

        // setting an output vc request for the next hop.
        // This flit will be ready to traverse the link and into the next hop
        // only when an output vc is acquired at the next hop
        outNetLink->request_vc_link(vc,
                                    new_net_msg_ptr->getInternalDestination(),
                                    m_net_ptr->curCycle());
    }

    return true ;
}

// An output vc has been granted at the next hop to one of the vc's.
// We have to update the state of the vc to reflect this
void
NetworkInterface::grant_vc(int out_port, int vc, Cycles grant_time)
{
    assert(m_out_vc_state[vc]->isInState(VC_AB_, grant_time));
    m_out_vc_state[vc]->grant_vc(grant_time);
    scheduleEvent(Cycles(1));
}

// The tail flit corresponding to this vc has been buffered at the next hop
// and thus this vc is now free
void
NetworkInterface::release_vc(int out_port, int vc, Cycles release_time)
{
    assert(m_out_vc_state[vc]->isInState(ACTIVE_, release_time));
    m_out_vc_state[vc]->setState(IDLE_, release_time);
    scheduleEvent(Cycles(1));
}

// Looking for a free output vc
int
NetworkInterface::calculateVC(int vnet)
{
    int vc_per_vnet;
    if (m_net_ptr->isVNetOrdered(vnet))
        vc_per_vnet = 1;
    else
        vc_per_vnet = m_vc_per_vnet;

    for (int i = 0; i < vc_per_vnet; i++) {
        int delta = m_vc_allocator[vnet];
        m_vc_allocator[vnet]++;
        if (m_vc_allocator[vnet] == vc_per_vnet)
            m_vc_allocator[vnet] = 0;

        if (m_out_vc_state[(vnet*m_vc_per_vnet) + delta]->isInState(IDLE_,
            m_net_ptr->curCycle())) {
            return ((vnet*m_vc_per_vnet) + delta);
        }
    }
    return -1;
}

/*
 * The NI wakeup checks whether there are any ready messages in the protocol
 * buffer. If yes, it picks that up, flitisizes it into a number of flits and
 * puts it into an output buffer and schedules the output link.
 * On a wakeup it also checks whether there are flits in the input link.
 * If yes, it picks them up and if the flit is a tail, the NI inserts the
 * corresponding message into the protocol buffer.
 */

void
NetworkInterface::wakeup()
{
    MsgPtr msg_ptr;

    //Checking for messages coming from the protocol
    // can pick up a message/cycle for each virtual net
    for (int vnet = 0; vnet < m_virtual_networks; vnet++) {
        while (inNode_ptr[vnet]->isReady()) // Is there a message waiting
        {
            msg_ptr = inNode_ptr[vnet]->peekMsgPtr();
            if (flitisizeMessage(msg_ptr, vnet)) {
                inNode_ptr[vnet]->dequeue();
            } else {
                break;
            }
        }
    }

    scheduleOutputLink();
    checkReschedule();

    /*********** Picking messages destined for this NI **********/

    if (inNetLink->isReady()) {
        flit *t_flit = inNetLink->consumeLink();
        if (t_flit->get_type() == TAIL_ || t_flit->get_type() == HEAD_TAIL_) {
            DPRINTF(RubyNetwork, "m_id: %d, Message delivered at time: %lld\n",
                    m_id, m_net_ptr->curCycle());

            outNode_ptr[t_flit->get_vnet()]->enqueue(
                t_flit->get_msg_ptr(), Cycles(1));

            // signal the upstream router that this vc can be freed now
            inNetLink->release_vc_link(t_flit->get_vc(),
                m_net_ptr->curCycle() + Cycles(1));
        }

        int vnet = t_flit->get_vnet();
        m_net_ptr->increment_received_flits(vnet);
        Cycles network_delay = m_net_ptr->curCycle() -
                               t_flit->get_enqueue_time();
        Cycles queueing_delay = t_flit->get_delay();

        m_net_ptr->increment_network_latency(network_delay, vnet);
        m_net_ptr->increment_queueing_latency(queueing_delay, vnet);
        delete t_flit;
    }
}

/* This function looks at the NI buffers and if some buffer has flits which
 * are ready to traverse the link in the next cycle and also the downstream
 * output vc associated with this flit has buffers left, the link is scheduled
 * for the next cycle
 */

void
NetworkInterface::scheduleOutputLink()
{
    int vc = m_vc_round_robin;
    m_vc_round_robin++;
    if (m_vc_round_robin == m_num_vcs)
        m_vc_round_robin = 0;

    for (int i = 0; i < m_num_vcs; i++) {
        vc++;
        if (vc == m_num_vcs)
            vc = 0;
        if (m_ni_buffers[vc]->isReady(m_net_ptr->curCycle())) {
            if (m_out_vc_state[vc]->isInState(ACTIVE_,
               m_net_ptr->curCycle()) &&
               outNetLink->isBufferNotFull_link(vc)) {  // buffer backpressure

                // Just removing the flit
                flit *t_flit = m_ni_buffers[vc]->getTopFlit();
                t_flit->set_time(m_net_ptr->curCycle() + Cycles(1));
                outSrcQueue->insert(t_flit);

                // schedule the out link
                outNetLink->
                    scheduleEventAbsolute(m_net_ptr->clockEdge(Cycles(1)));
                return;
            }
        }
    }
}

void
NetworkInterface::checkReschedule()
{
    for (int vnet = 0; vnet < m_virtual_networks; vnet++) {
        if (inNode_ptr[vnet]->isReady()) { // Is there a message waiting
            scheduleEvent(Cycles(1));
            return;
        }
    }
    for (int vc = 0; vc < m_num_vcs; vc++) {
        if (m_ni_buffers[vc]->isReadyForNext(m_net_ptr->curCycle())) {
            scheduleEvent(Cycles(1));
            return;
        }
    }
}

bool
NetworkInterface::functionalRead(Packet *pkt)
{
    // Go through the internal buffers
    for (unsigned int i = 0; i < m_ni_buffers.size(); ++i) {
        if (m_ni_buffers[i]->functionalRead(pkt)) {
            return true;
        }
    }

    // Go through the buffer between this network interface and the router
    if (outSrcQueue->functionalRead(pkt)) {
        return true;
    }

    return false;
}

uint32_t
NetworkInterface::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (unsigned int i = 0; i < m_ni_buffers.size(); ++i) {
        num_functional_writes += m_ni_buffers[i]->functionalWrite(pkt);
    }

    num_functional_writes += outSrcQueue->functionalWrite(pkt);
    return num_functional_writes;
}

void
NetworkInterface::print(std::ostream& out) const
{
    out << "[Network Interface]";
}

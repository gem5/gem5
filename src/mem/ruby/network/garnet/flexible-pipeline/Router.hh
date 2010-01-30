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

#ifndef ROUTER_H
#define ROUTER_H

#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/FlexibleConsumer.hh"
#include "mem/gems_common/PrioHeap.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/NetworkLink.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/flitBuffer.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/InVcState.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/OutVcState.hh"

class VCarbiter;

class Router : public FlexibleConsumer {
public:
        Router(int id, GarnetNetwork *network_ptr);

        ~Router();

        void addInPort(NetworkLink *in_link);
        void addOutPort(NetworkLink *out_link, const NetDest& routing_table_entry, int link_weight);
        void wakeup();
        void request_vc(int in_vc, int in_port, NetDest destination, Time request_time);
        bool isBufferNotFull(int vc, int inport);
        void grant_vc(int out_port, int vc, Time grant_time);
        void release_vc(int out_port, int vc, Time release_time);
        void vc_arbitrate();

        void printConfig(ostream& out) const;
        void print(ostream& out) const;

private:
/***************Data Members******************/
        int m_id;
        int m_virtual_networks, m_num_vcs, m_vc_per_vnet;
        GarnetNetwork *m_net_ptr;
        Vector<int > m_vc_round_robin; // For scheduling of out source queues
        int m_round_robin_inport, m_round_robin_start; // for vc arbitration
        Vector<int > m_round_robin_invc; // For every outport. for vc arbitration

        Vector<Vector<flitBuffer *> > m_router_buffers; // These are essentially output buffers
        Vector<flitBuffer *> m_out_src_queue; // These are source queues for the output link
        Vector<NetworkLink *> m_in_link;
        Vector<NetworkLink *> m_out_link;
        Vector<Vector<InVcState * > > m_in_vc_state;
        Vector<Vector<OutVcState * > > m_out_vc_state;
        Vector<NetDest> m_routing_table;
        Vector<int > m_link_weights;
        VCarbiter *m_vc_arbiter;

/*********** Private methods *************/
        int getRoute(NetDest destination);
        Vector<int > get_valid_vcs(int invc);
        void routeCompute(flit *m_flit, int inport);
        void checkReschedule();
        void check_arbiter_reschedule();
        void scheduleOutputLinks();
};

#endif


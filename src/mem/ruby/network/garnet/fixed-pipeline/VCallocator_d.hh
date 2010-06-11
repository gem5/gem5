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

#ifndef VC_ALLOCATOR_D_H
#define VC_ALLOCATOR_D_H

#include <iostream>
#include <vector>
#include <utility>

#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/common/Consumer.hh"

class Router_d;
class InputUnit_d;
class OutputUnit_d;

class VCallocator_d : public Consumer {
public:
        VCallocator_d(Router_d *router);
        void init();
        void wakeup();
        void check_for_wakeup();
        void clear_request_vector();
        int get_vnet(int invc);
        void print(std::ostream& out) const {};
        void arbitrate_invcs();
        void arbitrate_outvcs();
        bool is_invc_candidate(int inport_iter, int invc_iter);
        void select_outvc(int inport_iter, int invc_iter);
        inline double get_local_arbit_count()
        {
                return m_local_arbiter_activity;
        }
        inline double get_global_arbit_count()
        {
                return m_global_arbiter_activity;
        }

private:
        int m_num_vcs, m_vc_per_vnet;
        int m_num_inports;
        int m_num_outports;

        double m_local_arbiter_activity, m_global_arbiter_activity;

        Router_d *m_router;
        std::vector<std::vector<int > > m_round_robin_invc; // First stage of arbitration where all vcs select an output vc to content for
        std::vector<std::vector<std::pair<int, int> > > m_round_robin_outvc; // Arbiter for every output vc
        std::vector<std::vector<std::vector<std::vector<bool> > > > m_outvc_req; // [outport][outvc][inpotr][invc]. set true in the first phase of allocation
        std::vector<std::vector<bool> > m_outvc_is_req;

        std::vector<InputUnit_d *> m_input_unit ;
        std::vector<OutputUnit_d *> m_output_unit ;

};

#endif

/*
 * Copyright (c) 2009 Princeton University
 * Copyright (c) 2009 The Regents of the University of California
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
 * Authors:  Hangsheng Wang (Orion 1.0, Princeton)
 *           Xinping Zhu (Orion 1.0, Princeton)
 *           Xuning Chen (Orion 1.0, Princeton)
 *           Bin Li (Orion 2.0, Princeton)
 *           Kambiz Samadi (Orion 2.0, UC San Diego)
 */

#include <cassert>
#include <iostream>

#include "mem/ruby/network/orion/FlipFlop.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

using namespace std;

FlipFlop::FlipFlop(
  const string& ff_model_str_,
  double load_,
  const TechParameter* tech_param_ptr_
)
{
    if (ff_model_str_ == string("NEG_DFF"))
    {
        m_ff_model = NEG_DFF;
    }
    else
    {
        m_ff_model = NO_MODEL;
    }

    if (m_ff_model != NO_MODEL)
    {
        assert(load_ == load_);

        m_load = load_;
        m_tech_param_ptr = tech_param_ptr_;

        init();
    }
}

FlipFlop::~FlipFlop()
{}

void FlipFlop::init()
{
    double c1, c2, c3, c4, c5, c6;

    double e_factor = m_tech_param_ptr->get_EnergyFactor();
    switch(m_ff_model)
    {
        case NEG_DFF:
            c1 = c5 = c6 = calc_node_cap(2, 1);
            c2 = calc_node_cap(2, 3);
            c3 = calc_node_cap(3, 2);
            c4 = calc_node_cap(2, 3);

            m_e_switch = (c1+c2+c3+c4+c5+c6+m_load)*e_factor/2.0;
            // no 1/2 for e_keep and e_clock because clock signal switches twice in one cycle
            m_e_keep_1 = c3*e_factor;
            m_e_keep_0 = c2*e_factor;
            m_e_clock = calc_clock_cap()*e_factor;

            m_i_static = calc_i_static();
            break;
        default:
            cerr << "error" << endl;
    }
    return;
}


//this model is based on the gate-level design given by Randy H. Katz "Contemporary Logic Design"
//Figure 6.24, node numbers (1-6) are assigned to all gate outputs, left to right, top to bottom
//
//We should have pure cap functions and leave the decision of whether or not to have coefficient
//1/2 in init function.
double FlipFlop::calc_node_cap(uint32_t num_fanin_, uint32_t num_fanout_)
{
    double total_cap = 0;

    //FIXME: all need actual size
    //part 1: drain cap of NOR gate
    double WdecNORn = m_tech_param_ptr->get_WdecNORn();
    double WdecNORp = m_tech_param_ptr->get_WdecNORp();
    total_cap += num_fanin_*m_tech_param_ptr->calc_draincap(WdecNORn, TechParameter::NCH, 1) + m_tech_param_ptr->calc_draincap(WdecNORp, TechParameter::PCH, num_fanin_);

    //part 2: gate cap of NOR gate
    total_cap += num_fanout_*m_tech_param_ptr->calc_gatecap(WdecNORn+WdecNORp, 0);
    return total_cap;
}

double FlipFlop::calc_clock_cap()
{
    double WdecNORn = m_tech_param_ptr->get_WdecNORn();
    double WdecNORp = m_tech_param_ptr->get_WdecNORp();
    return (2*m_tech_param_ptr->calc_gatecap(WdecNORn+WdecNORp, 0));
}

double FlipFlop::calc_i_static()
{
    double WdecNORn = m_tech_param_ptr->get_WdecNORn();
    double WdecNORp = m_tech_param_ptr->get_WdecNORp();
    double NOR2_TAB_0 = m_tech_param_ptr->get_NOR2_TAB(0);
    double NOR2_TAB_1 = m_tech_param_ptr->get_NOR2_TAB(1);
    double NOR2_TAB_2 = m_tech_param_ptr->get_NOR2_TAB(2);
    double NOR2_TAB_3 = m_tech_param_ptr->get_NOR2_TAB(3);

    return (WdecNORp*NOR2_TAB_0 + WdecNORn*(NOR2_TAB_1+NOR2_TAB_2+NOR2_TAB_3))/4*6;
}

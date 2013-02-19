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

#include "mem/ruby/network/orion/Crossbar/Crossbar.hh"
#include "mem/ruby/network/orion/Crossbar/MatrixCrossbar.hh"
#include "mem/ruby/network/orion/Crossbar/MultreeCrossbar.hh"
#include "mem/ruby/network/orion/OrionConfig.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

using namespace std;

Crossbar::Crossbar(
        CrossbarModel xbar_model_,
        const string& conn_type_str_,
        const string& trans_type_str_,
        uint32_t num_in_,
        uint32_t num_out_,
        uint32_t data_width_,
        uint32_t num_in_seg_,
        uint32_t num_out_seg_,
        uint32_t degree_,
        const TechParameter* tech_param_ptr_
        )
{
    m_xbar_model = xbar_model_;
    if (m_xbar_model != NO_MODEL)
    {
        assert((num_in_ == num_in_) && (num_in_ != 0));
        assert((num_out_ == num_out_) && (num_out_ != 0));
        assert((data_width_ == data_width_) && (data_width_ != 0));
        assert(num_in_seg_ == num_in_seg_);
        assert(num_out_seg_ == num_out_seg_);

        set_conn_type(conn_type_str_);
        set_trans_type(trans_type_str_);
        m_num_in = num_in_;
        m_num_out = num_out_;
        m_num_in_seg = num_in_seg_;
        m_num_out_seg = num_out_seg_;
        m_data_width = data_width_;
        m_degree = degree_;
        m_tech_param_ptr = tech_param_ptr_;
    }
    else
    {
        cerr << "ERROR at " << __FILE__ << " " << __LINE__ << endl;
    }
}

Crossbar::~Crossbar()
{}

double Crossbar::get_static_power() const
{
    double vdd = m_tech_param_ptr->get_vdd();
    double SCALE_S = m_tech_param_ptr->get_SCALE_S();
    return (m_i_static*vdd*SCALE_S);
}

void Crossbar::print_all() const
{
    cout << "Crossbar" << endl;
    cout << "\t" << "Traversal = " << get_dynamic_energy(false) << endl;
    cout << "\t" << "Static power = " << get_static_power() << endl;
    return;
}

void Crossbar::set_conn_type(const string& conn_type_str_)
{
    if (conn_type_str_ == string("TRANS_GATE"))
    {
        m_conn_type = TRANS_GATE;
    }
    else if (conn_type_str_ == string("TRISTATE_GATE"))
    {
        m_conn_type = TRISTATE_GATE;
    }
    else
    {
        cerr << "Invalid connect type: '" << conn_type_str_ << "'. Use TRANS_GATE as default." << endl;
        m_conn_type = TRANS_GATE;
    }
    return;
}

void Crossbar::set_trans_type(const string& trans_type_str_)
{
    if (trans_type_str_ == string("NP_GATE"))
    {
        m_trans_type = NP_GATE;
    }
    else if (trans_type_str_ == string("N_GATE"))
    {
        m_trans_type = N_GATE;
    }
    else
    {
        cerr << "Invalid trans type: '" << trans_type_str_ << "'. Use N_GATE as default." << endl;
        m_trans_type = N_GATE;
    }
}

double Crossbar::calc_in_cap()
{
    double total_cap = 0;

    // part 1: wire cap
    total_cap += m_cap_in_wire;

    double trans_cap = 0;
    // part 2: drain cap of transmission gate or gate cap of tri-state gate
    if (m_conn_type == TRANS_GATE)
    {
        //FIXME: resizing strategy
        double Wmemcellr = m_tech_param_ptr->get_Wmemcellr();
        double nsize = Wmemcellr;
        double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
        double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
        double psize = nsize*Wdecinvp/Wdecinvn;
        trans_cap = m_tech_param_ptr->calc_draincap(nsize, TechParameter::NCH, 1);
        if (m_trans_type == NP_GATE)
        {
            trans_cap += m_tech_param_ptr->calc_draincap(psize, TechParameter::PCH, 1);
        }
    }
    else if (m_conn_type == TRISTATE_GATE)
    {
        double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
        double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
        double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
        double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();
        trans_cap = m_tech_param_ptr->calc_gatecap(Woutdrvnandn+Woutdrvnandp, 0)
            + m_tech_param_ptr->calc_gatecap(Woutdrvnorn+Woutdrvnorp, 0);
    }
    total_cap += trans_cap*m_num_out;

    // segmented crossbar
    if (m_num_in_seg > 1)
    {
        total_cap *= (m_num_in_seg+1)/(m_num_in_seg*2);
        // input capacitance of tri-state buffer
        double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
        double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
        double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
        double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();
        total_cap += (m_num_in_seg+2)*(m_num_in_seg-1)/(m_num_in_seg*2)*(m_tech_param_ptr->calc_gatecap(Woutdrvnandn+Woutdrvnandp, 0)+m_tech_param_ptr->calc_gatecap(Woutdrvnorn+Woutdrvnorp, 0));
        // output capacitance of tri-state buffer
        double Woutdrivern = m_tech_param_ptr->get_Woutdrivern();
        double Woutdriverp = m_tech_param_ptr->get_Woutdriverp();
        total_cap += (m_num_in_seg-1)/2*(m_tech_param_ptr->calc_draincap(Woutdrivern, TechParameter::NCH, 1)+m_tech_param_ptr->calc_draincap(Woutdriverp, TechParameter::PCH, 1));
    }

    // part 3: input driver
    //FIXME: how to specify timing
    double period = m_tech_param_ptr->get_period();
    double psize = m_tech_param_ptr->calc_driver_psize(total_cap, period/3.0);
    double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
    double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
    double nsize = psize*Wdecinvn/Wdecinvp;
    total_cap += m_tech_param_ptr->calc_draincap(nsize, TechParameter::NCH, 1)+m_tech_param_ptr->calc_draincap(psize, TechParameter::PCH, 1)+m_tech_param_ptr->calc_gatecap(nsize+psize, 0);

    return total_cap/2.0;
}

double Crossbar::calc_out_cap(uint32_t num_in_)
{
    double total_cap = 0;

    // part 1: wire cap
    total_cap += m_cap_out_wire;

    double trans_cap = 0;
    // part 2: drain cap of transmission gate or tri-state gate
    if (m_conn_type == TRANS_GATE)
    {
        // FIXME: resizing strategy
        double Wmemcellr = m_tech_param_ptr->get_Wmemcellr();
        double nsize = Wmemcellr;
        double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
        double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
        double psize = nsize*Wdecinvp/Wdecinvn;

        trans_cap = m_tech_param_ptr->calc_draincap(nsize, TechParameter::NCH, 1);
        if (m_trans_type == NP_GATE)
        {
            trans_cap += m_tech_param_ptr->calc_draincap(psize, TechParameter::PCH, 1);
        }
    }
    else if (m_conn_type == TRISTATE_GATE)
    {
        double Woutdrivern = m_tech_param_ptr->get_Woutdrivern();
        double Woutdriverp = m_tech_param_ptr->get_Woutdriverp();
        trans_cap = m_tech_param_ptr->calc_draincap(Woutdrivern, TechParameter::NCH, 1)+m_tech_param_ptr->calc_draincap(Woutdriverp, TechParameter::PCH, 1);
    }
    total_cap += trans_cap*num_in_;

    // segmented crossbar
    if (m_num_out_seg > 1)
    {
        total_cap *= (m_num_out_seg+1)/(m_num_out_seg*2);
        // input capacitance of tri-state buffer
        double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
        double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
        double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
        double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();
        total_cap += (m_num_out_seg+2)*(m_num_out_seg-1)/(m_num_out_seg*2)*(m_tech_param_ptr->calc_gatecap(Woutdrvnandn+Woutdrvnandp, 0)+m_tech_param_ptr->calc_gatecap(Woutdrvnorn+Woutdrvnorp, 0));
        // output capacitance of tri-state buffer
        double Woutdrivern = m_tech_param_ptr->get_Woutdrivern();
        double Woutdriverp = m_tech_param_ptr->get_Woutdriverp();
        total_cap += (m_num_out_seg-1)/2*(m_tech_param_ptr->calc_draincap(Woutdrivern, TechParameter::NCH, 1)+m_tech_param_ptr->calc_draincap(Woutdriverp, TechParameter::PCH, 1));
    }

    // part 3: output driver
    double Woutdrivern = m_tech_param_ptr->get_Woutdrivern();
    double Woutdriverp = m_tech_param_ptr->get_Woutdriverp();
    total_cap += m_tech_param_ptr->calc_draincap(Woutdrivern, TechParameter::NCH, 1)+m_tech_param_ptr->calc_draincap(Woutdriverp, TechParameter::PCH, 1)+m_tech_param_ptr->calc_gatecap(Woutdrivern+Woutdriverp, 0);

    return total_cap/2.0;
}

double Crossbar::calc_int_cap()
{
    double total_cap = 0;

    if (m_conn_type == TRANS_GATE)
    {
        // part 1: drain cap of transmission gate
        //FIXME: Wmemcellr and resize
        double Wmemcellr = m_tech_param_ptr->get_Wmemcellr();
        double nsize = Wmemcellr;
        double trans_cap = m_tech_param_ptr->calc_draincap(nsize, TechParameter::NCH, 1);
        if (m_trans_type == NP_GATE)
        {
            double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
            double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
            double psize = nsize*Wdecinvp/Wdecinvn;
            trans_cap += m_tech_param_ptr->calc_draincap(psize, TechParameter::PCH, 1);
        }
        total_cap += trans_cap*(m_degree+1);
    }
    else if (m_conn_type == TRISTATE_GATE)
    {
        // part 1: drain cap of tri-state gate
        double Woutdrivern = m_tech_param_ptr->get_Woutdrivern();
        double Woutdriverp = m_tech_param_ptr->get_Woutdriverp();

        double trans_cap = (m_tech_param_ptr->calc_draincap(Woutdrivern, TechParameter::NCH, 1)+m_tech_param_ptr->calc_draincap(Woutdriverp, TechParameter::PCH, 1))*m_degree;
        // part 2: gate cap of tri-state gate
        double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
        double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
        double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
        double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();
        trans_cap += m_tech_param_ptr->calc_gatecap(Woutdrvnandn+Woutdrvnandp, 0)+m_tech_param_ptr->calc_gatecap(Woutdrvnorn+Woutdrvnorp, 0);
        total_cap += trans_cap;
    }

    return total_cap/2.0;
}

double Crossbar::calc_ctr_cap(double cap_wire_, bool prev_ctr_, bool next_ctr_)
{
    double total_cap = 0;

    // part 1: wire cap
    total_cap += cap_wire_;

    double trans_cap = 0;
    // part 2: gate cap of transmission gate or tri-state gate
    if (m_conn_type == TRANS_GATE)
    {
        //FIXME Wmemcellr and resize
        double Wmemcellr = m_tech_param_ptr->get_Wmemcellr();
        double nsize = Wmemcellr;
        trans_cap = m_tech_param_ptr->calc_gatecap(nsize, 0);
        if (m_trans_type == NP_GATE)
        {
            double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
            double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
            double psize = nsize*Wdecinvp/Wdecinvn;
            trans_cap += m_tech_param_ptr->calc_gatecap(psize, 0);
        }
    }
    else if (m_conn_type == TRISTATE_GATE)
    {
        double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
        double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
        double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
        double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();
        trans_cap = m_tech_param_ptr->calc_gatecap(Woutdrvnandn+Woutdrvnandp, 0)
            + m_tech_param_ptr->calc_gatecap(Woutdrvnorn+Woutdrvnorp, 0);
    }
    total_cap += trans_cap*m_data_width;

    // part 3: inverter
    if (!((m_conn_type == TRANS_GATE) && (m_trans_type == N_GATE) && (!prev_ctr_)))
    {
        double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
        double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
        total_cap += m_tech_param_ptr->calc_draincap(Wdecinvn, TechParameter::NCH, 1)+m_tech_param_ptr->calc_draincap(Wdecinvp, TechParameter::PCH, 1)
            + m_tech_param_ptr->calc_gatecap(Wdecinvn+Wdecinvp, 0);
    }

    double WdecNORn = m_tech_param_ptr->get_WdecNORn();
    double WdecNORp = m_tech_param_ptr->get_WdecNORp();
    // part 4: drain cap of previous level control signal
    if (prev_ctr_)
    {
        // FIXME: need actual size, use decoder data for now
        total_cap += m_degree*m_tech_param_ptr->calc_draincap(WdecNORn, TechParameter::NCH, 1)
            +m_tech_param_ptr->calc_draincap(WdecNORp, TechParameter::PCH, m_degree);
    }

    // part 5: gate cap of next level control signal
    if (next_ctr_)
    {
        // FIXME: need actual size, use decoder data for now
        total_cap += m_tech_param_ptr->calc_gatecap(WdecNORn+WdecNORp, m_degree*40+20);
    }

    return total_cap;
}

Crossbar* Crossbar::create_crossbar(
        const string& xbar_model_str_,
        uint32_t num_in_,
        uint32_t num_out_,
        uint32_t data_width_,
        const OrionConfig* orion_cfg_ptr_
        )
{
    if (xbar_model_str_ == string("MATRIX_CROSSBAR"))
    {
        const string& conn_type_str = orion_cfg_ptr_->get<string>("CROSSBAR_CONNECT_TYPE");
        const string& trans_type_str = orion_cfg_ptr_->get<string>("CROSSBAR_TRANS_GATE_TYPE");
        uint32_t num_in_seg = orion_cfg_ptr_->get<uint32_t>("CROSSBAR_NUM_IN_SEG");
        uint32_t num_out_seg = orion_cfg_ptr_->get<uint32_t>("CROSSBAR_NUM_OUT_SEG");
        double len_in_wire = orion_cfg_ptr_->get<double>("CROSSBAR_LEN_IN_WIRE");
        double len_out_wire = orion_cfg_ptr_->get<double>("CROSSBAR_LEN_OUT_WIRE");
        const TechParameter* tech_param_ptr = orion_cfg_ptr_->get_tech_param_ptr();
        return new MatrixCrossbar(conn_type_str, trans_type_str, 
                num_in_, num_out_, data_width_, num_in_seg, num_out_seg,
                len_in_wire, len_out_wire, tech_param_ptr);
    }
    else if (xbar_model_str_ == string("MULTREE_CROSSBAR"))
    {
        const string& conn_type_str = orion_cfg_ptr_->get<string>("CROSSBAR_CONNECT_TYPE");
        const string& trans_type_str = orion_cfg_ptr_->get<string>("CROSSBAR_TRANS_GATE_TYPE");
        uint32_t degree = orion_cfg_ptr_->get<uint32_t>("CROSSBAR_MUX_DEGREE");
        const TechParameter* tech_param_ptr = orion_cfg_ptr_->get_tech_param_ptr();
        return new MultreeCrossbar(conn_type_str, trans_type_str, 
                num_in_, num_out_, data_width_, degree, tech_param_ptr);
    }
    else
    {
        cerr << "WARNING: No Crossbar model" << endl;
        return (Crossbar*)NULL;
    }
}

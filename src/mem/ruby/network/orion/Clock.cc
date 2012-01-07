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

#include "mem/ruby/network/orion/Clock.hh"
#include "mem/ruby/network/orion/OrionConfig.hh"
#include "mem/ruby/network/orion/TechParameter.hh"
#include "mem/ruby/network/orion/Wire.hh"

Clock::Clock(
        bool is_in_buf_,
        bool is_in_shared_switch_,
        bool is_out_buf_,
        bool is_out_shared_switch_,
        const OrionConfig* orion_cfg_ptr_
        )
{
    m_is_in_buf = is_in_buf_;
    m_is_in_shared_switch = is_in_shared_switch_;
    m_is_out_buf = is_out_buf_;
    m_is_out_shared_switch = is_out_shared_switch_;
    m_orion_cfg_ptr = orion_cfg_ptr_;

    init();
}

Clock::~Clock()
{}

double Clock::get_dynamic_energy() const
{
    return (m_e_pipe_reg + m_e_htree);
}

double Clock::get_static_power() const
{
    double vdd = m_tech_param_ptr->get_vdd();
    return (m_i_static*vdd);
}

void Clock::init()
{
    m_tech_param_ptr = m_orion_cfg_ptr->get_tech_param_ptr();

    double e_factor = m_tech_param_ptr->get_EnergyFactor();

    // Pipeline registers capacitive load on clock network
    uint32_t num_in_port = m_orion_cfg_ptr->get_num_in_port();
    uint32_t num_out_port = m_orion_cfg_ptr->get_num_out_port();
    uint32_t num_vclass = m_orion_cfg_ptr->get_num_vclass();
    uint32_t num_vchannel = m_orion_cfg_ptr->get_num_vchannel();
    uint32_t flit_width = m_orion_cfg_ptr->get_flit_width();

    uint32_t num_pipe_reg = 0;

    // pipeline registers after the link traversal stage
    num_pipe_reg += num_in_port*flit_width;

    // pipeline registers for input buffer
    if (m_is_in_buf)
    {
        if (m_is_in_shared_switch)
        {
            num_pipe_reg += num_in_port*flit_width;
        }
        else
        {
            num_pipe_reg += num_in_port*num_vclass*num_vchannel*flit_width;
        }
    }

    // pipeline registers for crossbar
    if (m_is_out_shared_switch)
    {
        num_pipe_reg += num_out_port*flit_width;
    }
    else
    {
        num_pipe_reg += num_out_port*num_vclass*num_vchannel*flit_width;
    }

    // pipeline registers for output buffer
    if (m_is_out_buf) // assume output buffers share links
    {
        num_pipe_reg += num_out_port*flit_width;
    }

    double cap_clock = m_tech_param_ptr->get_ClockCap();
    m_e_pipe_reg = num_pipe_reg*cap_clock*e_factor;

    //========================H_tree wiring load ========================*/
    // The 1e-6 factor is to convert the "router_diagonal" back to meters.
    // To be consistent we use micro-meters unit for our inputs, but 
    // the functions, internally, use meters. */

    double i_static_nmos = 0;
    double i_static_pmos = 0;

    bool is_htree = m_orion_cfg_ptr->get<bool>("IS_HTREE_CLOCK");
    if(is_htree)
    {
        const string& width_spacing_model_str = m_orion_cfg_ptr->get<string>("WIRE_WIDTH_SPACING");
        const string& buf_scheme_str = m_orion_cfg_ptr->get<string>("WIRE_BUFFERING_MODEL");
        bool is_shielding = m_orion_cfg_ptr->get<bool>("WIRE_IS_SHIELDING");
        Wire wire(width_spacing_model_str, buf_scheme_str, is_shielding, m_tech_param_ptr);

        double router_diagonal = m_orion_cfg_ptr->get<double>("ROUTER_DIAGONAL");
        double Clockwire = m_tech_param_ptr->get_ClockCap();

        double htree_clockcap;
        int k;
        double h;

        double BufferNMOSOffCurrent = m_tech_param_ptr->get_BufferNMOSOffCurrent();
        double BufferPMOSOffCurrent = m_tech_param_ptr->get_BufferPMOSOffCurrent();

        if (m_tech_param_ptr->is_trans_type_hvt() || m_tech_param_ptr->is_trans_type_nvt())
        {
            htree_clockcap = (4+4+2+2)*(router_diagonal*1e-6)*Clockwire;

            wire.calc_opt_buffering(&k, &h, ((4+4+2+2)*router_diagonal*1e-6));
            i_static_nmos = BufferNMOSOffCurrent*h*k*15;
            i_static_pmos = BufferPMOSOffCurrent*h*k*15;
        }
        else
        {
            htree_clockcap = (8+4+4+4+4)*(router_diagonal*1e-6)*Clockwire;

            wire.calc_opt_buffering(&k, &h, ((4+4+2+2)*router_diagonal*1e-6));  
            i_static_nmos = BufferNMOSOffCurrent*h*k*29;
            i_static_pmos = BufferPMOSOffCurrent*h*k*15;
        }

        m_e_htree = (htree_clockcap+cap_clock)*e_factor;
    }
    else
    {
        m_e_htree = 0;
    }

    double SCALE_S = m_tech_param_ptr->get_SCALE_S();
    double DFF_TAB_0 = m_tech_param_ptr->get_DFF_TAB(0);
    double Wdff = m_tech_param_ptr->get_Wdff();
    m_i_static = (((i_static_nmos+i_static_pmos)/2)/SCALE_S + (num_pipe_reg*DFF_TAB_0*Wdff));
}

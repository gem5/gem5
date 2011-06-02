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

#include "base/misc.hh"
#include "mem/ruby/network/orion/Buffer/PrechargeUnit.hh"
#include "mem/ruby/network/orion/Buffer/SRAM.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

PrechargeUnit::PrechargeUnit(
        const string& pre_model_str_,
        double pre_load_,
        const SRAM* sram_ptr_,
        const TechParameter* tech_param_ptr_
        )
{
    if (pre_model_str_ == "SINGLE_BITLINE")
    {
        m_pre_model = SINGLE_BITLINE;
    }
    else if (pre_model_str_ == "EQU_BITLINE")
    {
        m_pre_model = EQU_BITLINE;
    }
    else if (pre_model_str_ == "SINGLE_OTHER")
    {
        m_pre_model = SINGLE_OTHER;
    }
    else
    {
        m_pre_model = NO_MODEL;
    }

    if (m_pre_model != NO_MODEL)
    {
        m_pre_load = pre_load_;
        m_sram_ptr = sram_ptr_;
        m_tech_param_ptr = tech_param_ptr_;

        init();
    }
}

PrechargeUnit::~PrechargeUnit()
{
}

void PrechargeUnit::init()
{
    double period = m_tech_param_ptr->get_period();
    m_pre_size = m_tech_param_ptr->calc_driver_psize(m_pre_load, period/8.0);
    //FIXME - shouldn't be added
    double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
    double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
    m_pre_size += m_pre_size*Wdecinvn/Wdecinvp;

    uint32_t num_gate = calc_num_pre_gate();

    // WHS: 10 should go to PARM 
    double e_factor = m_tech_param_ptr->get_EnergyFactor();
    m_e_charge_gate = calc_pre_cap(m_pre_size, 10) * num_gate * e_factor;

    uint32_t num_data_end = m_sram_ptr->get_num_data_end();
    if (num_data_end == 2)
    {
        e_factor = m_tech_param_ptr->get_SenseEnergyFactor();
    }
    else
    {
        e_factor = m_tech_param_ptr->get_EnergyFactor();
    }
    uint32_t num_drain = calc_num_pre_drain();
    m_e_charge_drain = m_tech_param_ptr->calc_draincap(m_pre_size, TechParameter::PCH, 1)*num_drain*e_factor;

    // static power
    double PMOS_TAB_0 = m_tech_param_ptr->get_PMOS_TAB(0);
    m_i_static = num_gate*m_pre_size*PMOS_TAB_0;
}

uint32_t PrechargeUnit::calc_num_pre_gate()
{
    switch(m_pre_model)
    {
        case SINGLE_BITLINE: return 2;
        case EQU_BITLINE:    return 3;
        case SINGLE_OTHER:   return 1;
        default: fatal("Incorrect Precharge Unit model.\n");
    }
}

uint32_t PrechargeUnit::calc_num_pre_drain()
{
    switch(m_pre_model)
    {
        case SINGLE_BITLINE: return 1;
        case EQU_BITLINE:    return 2;
        case SINGLE_OTHER:   return 1;
        default: fatal("Incorrect Precharge Unit model.\n");
    }
}

double PrechargeUnit::calc_pre_cap(double width_, double length_)
{
    return m_tech_param_ptr->calc_gatecap(width_, length_);
}


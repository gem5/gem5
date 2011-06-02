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

#include <iostream>

#include "mem/ruby/network/orion/Buffer/MemUnit.hh"
#include "mem/ruby/network/orion/Buffer/SRAM.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

using namespace std;

MemUnit::MemUnit(
        const string& mem_model_str_,
        const SRAM* sram_ptr_,
        const TechParameter* tech_param_ptr_
        )
{
    if (mem_model_str_.compare("NORMAL_MEM") == 0)
    {
        m_mem_model = NORMAL_MEM;
    }
    else 
    {
        m_mem_model = NO_MODEL;
    }

    if (m_mem_model != NO_MODEL)
    {
        m_sram_ptr = sram_ptr_;
        m_tech_param_ptr = tech_param_ptr_;

        init();
    }
}

MemUnit::~MemUnit()
{}

void MemUnit::init()
{
    double e_factor = m_tech_param_ptr->get_EnergyFactor();

    m_e_switch = calc_mem_cap() * e_factor;

    m_i_static = calc_i_static();
}

double MemUnit::calc_mem_cap()
{
    double Wmemcella = m_tech_param_ptr->get_Wmemcella();
    double Wmemcellbscale = m_tech_param_ptr->get_Wmemcellbscale();
    double Wmemcellr = m_tech_param_ptr->get_Wmemcellr();
    double Wmemcellw = m_tech_param_ptr->get_Wmemcellw();
    uint32_t num_data_end = m_sram_ptr->get_num_data_end();
    uint32_t num_read_port = m_sram_ptr->get_num_read_port();
    uint32_t num_write_port = m_sram_ptr->get_num_write_port();

    const TechParameter* tp = m_tech_param_ptr;

    double total_cap = 0;
    // part 1: drain capacitance of pass transistors
    total_cap += tp->calc_draincap(Wmemcellr, TechParameter::NCH, 1)*num_read_port*num_data_end/2.0;
    total_cap += tp->calc_draincap(Wmemcellw, TechParameter::NCH, 1)*num_write_port;

    // has coefficient (1/2 * 2)
    // part 2: drain capacitance of memory cell
    total_cap += tp->calc_draincap(Wmemcella, TechParameter::NCH, 1) + tp->calc_draincap(Wmemcella*Wmemcellbscale, TechParameter::PCH, 1
            );

    // has coefficient (1/2 * 2)
    // part 3: gate capacitance of memory cell
    total_cap += tp->calc_gatecap(Wmemcella, 1) + tp->calc_gatecap(Wmemcella*Wmemcellbscale, 1);

    return total_cap;
}

double MemUnit::calc_i_static()
{
    double Wmemcella = m_tech_param_ptr->get_Wmemcella();
    double Wmemcellbscale = m_tech_param_ptr->get_Wmemcellbscale();
    double Wmemcellr = m_tech_param_ptr->get_Wmemcellr();
    double Wmemcellw = m_tech_param_ptr->get_Wmemcellw();
    uint32_t num_data_end = m_sram_ptr->get_num_data_end();
    uint32_t num_read_port = m_sram_ptr->get_num_read_port();
    uint32_t num_write_port = m_sram_ptr->get_num_write_port();

    const TechParameter* tp = m_tech_param_ptr;

    double ret = 0;
    // memory cell
    //FIXME - why
    ret += (Wmemcella*tp->get_NMOS_TAB(0) + Wmemcella*Wmemcellbscale*tp->get_PMOS_TAB(0))*2;
    // read port pass tx
    ret += (Wmemcellr*tp->get_NMOS_TAB(0)*num_data_end*num_read_port);
    // write port pass tx
    ret += (Wmemcellw*tp->get_NMOS_TAB(0)*2*num_write_port);

    return ret;
}


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
#include "mem/ruby/network/orion/Buffer/SRAM.hh"
#include "mem/ruby/network/orion/Buffer/WordlineUnit.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

WordlineUnit::WordlineUnit(
        const string& wl_model_str_,
        const SRAM* sram_ptr_,
        const TechParameter* tech_param_ptr_
        )
{
    if (wl_model_str_ == string("RW_WORDLINE"))
    {
        m_wl_model = RW_WORDLINE;
    }
    else if (wl_model_str_ == string("WO_WORDLINE"))
    {
        m_wl_model = WO_WORDLINE;
    }
    else 
    {
        m_wl_model = NO_MODEL;
    }

    if (m_wl_model != NO_MODEL)
    {
        m_sram_ptr = sram_ptr_;
        m_tech_param_ptr = tech_param_ptr_;

        init();
    }
}

WordlineUnit::~WordlineUnit()
{}

void WordlineUnit::init()
{
    uint32_t num_port = m_sram_ptr->get_num_port();
    uint32_t num_read_port = m_sram_ptr->get_num_read_port();
    uint32_t num_col = m_sram_ptr->get_num_col();
    uint32_t num_data_end = m_sram_ptr->get_num_data_end();
    double RegCellWidth = m_tech_param_ptr->get_RegCellWidth();
    double BitlineSpacing = m_tech_param_ptr->get_BitlineSpacing();

    if (num_data_end == 2)
    {
        m_wl_len = num_col*(RegCellWidth + 2*num_port*BitlineSpacing);
    }
    else
    {
        m_wl_len = num_col*(RegCellWidth + (2*num_port-num_read_port)*BitlineSpacing);
    }

    double wl_cmetal;
    if (num_port > 1)
    {
        wl_cmetal = m_tech_param_ptr->get_CC3M3metal();
    }
    else
    {
        wl_cmetal = m_tech_param_ptr->get_CM3metal();
    }

    m_wl_wire_cap = m_wl_len*wl_cmetal;

    double e_factor = m_tech_param_ptr->get_EnergyFactor();
    double Wmemcellr = m_tech_param_ptr->get_Wmemcellr();
    double Wmemcellw = m_tech_param_ptr->get_Wmemcellw();
    double Woutdrivern = m_tech_param_ptr->get_Woutdrivern();
    double Woutdriverp = m_tech_param_ptr->get_Woutdriverp();
    double NMOS_TAB_0 = m_tech_param_ptr->get_NMOS_TAB(0);
    double PMOS_TAB_0 = m_tech_param_ptr->get_PMOS_TAB(0);
    switch(m_wl_model)
    {
        case RW_WORDLINE:
            m_e_read = calc_wordline_cap(num_col*num_data_end, Wmemcellr) * e_factor;
            m_e_write = calc_wordline_cap(num_col*2, Wmemcellw) * e_factor;
            m_i_static = (Woutdrivern*NMOS_TAB_0 + Woutdriverp*PMOS_TAB_0);
            break;
        case WO_WORDLINE:
            m_e_read = 0;
            m_e_write = calc_wordline_cap(num_col*2, Wmemcellw)*e_factor;
            m_i_static = 0;
            break;
        default:
            fatal("Incorrect Wordline model.\n");
    }
    return;
}

double WordlineUnit::calc_wordline_cap(
        uint32_t num_mos_,
        double mos_width_
        ) const
{
    double total_cap;

    // part 1: line cap, including gate cap of pass tx's and metal cap
    double BitWidth = m_tech_param_ptr->get_BitWidth();
    total_cap = m_tech_param_ptr->calc_gatecappass(mos_width_, BitWidth/2.0-mos_width_)*num_mos_ + m_wl_wire_cap;

    // part 2: input driver
    double period = m_tech_param_ptr->get_period();
    double psize, nsize;
    psize = m_tech_param_ptr->calc_driver_psize(total_cap, period/16.0);
    double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
    double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
    nsize = psize*Wdecinvn/Wdecinvp;

    // WHS: 20 should go to PARM
    total_cap += m_tech_param_ptr->calc_draincap(nsize, TechParameter::NCH, 1) + m_tech_param_ptr->calc_draincap(psize, TechParameter::PCH, 1) + m_tech_param_ptr->calc_gatecap(psize+nsize, 20);

    return total_cap;
}


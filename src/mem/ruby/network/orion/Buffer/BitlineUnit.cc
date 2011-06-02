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
 * Authors:   Hangsheng Wang (Orion 1.0, Princeton)
 *           Xinping Zhu (Orion 1.0, Princeton)
 *           Xuning Chen (Orion 1.0, Princeton)
 *           Bin Li (Orion 2.0, Princeton)
 *           Kambiz Samadi (Orion 2.0, UC San Diego)
 */

#include "base/misc.hh"
#include "mem/ruby/network/orion/Buffer/BitlineUnit.hh"
#include "mem/ruby/network/orion/Buffer/SRAM.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

BitlineUnit::BitlineUnit(
        const string bl_model_str_,
        const SRAM* sram_ptr_,
        const TechParameter* tech_param_ptr_
        )
{
    if (bl_model_str_ == "RW_BITLINE")
    {
        m_bl_model = RW_BITLINE;
    }
    else if (bl_model_str_ == "WO_BITLINE")
    {
        m_bl_model = WO_BITLINE;
    }
    else 
    {
        m_bl_model = NO_MODEL;
    }

    if (m_bl_model != NO_MODEL)
    {
        m_sram_ptr = sram_ptr_;
        m_tech_param_ptr = tech_param_ptr_;

        init();
    }
}

BitlineUnit::~BitlineUnit()
{}

void BitlineUnit::init()
{
    uint32_t num_port = m_sram_ptr->get_num_port();
    uint32_t num_data_end = m_sram_ptr->get_num_data_end();
    double bl_cmetal;
    if ((num_port > 1) || (num_data_end == 2))
    {
        bl_cmetal = m_tech_param_ptr->get_CC3M2metal();
    }
    else
    {
        bl_cmetal = m_tech_param_ptr->get_CM2metal();
    }
    uint32_t num_row = m_sram_ptr->get_num_row();
    double RegCellHeight = m_tech_param_ptr->get_RegCellHeight();
    double WordlineSpacing = m_tech_param_ptr->get_WordlineSpacing();
    m_bl_len = num_row*(RegCellHeight + num_port*WordlineSpacing);
    m_bl_wire_cap = m_bl_len * bl_cmetal;

    double e_factor = m_tech_param_ptr->get_EnergyFactor();
    double sense_e_factor = m_tech_param_ptr->get_SenseEnergyFactor();
    switch(m_bl_model)
    {
        case RW_BITLINE:
            if (num_data_end == 2)
            {
                m_e_col_sel = calc_col_select_cap() * e_factor;
                m_e_col_read = calc_col_read_cap() * sense_e_factor;
            }
            else
            {
                m_e_col_sel = 0;
                m_e_col_read = calc_col_read_cap() * e_factor;
            }
            m_e_col_write = calc_col_write_cap() * e_factor;

            m_i_static = calc_i_static();
            break;
        case WO_BITLINE:
            m_e_col_sel = m_e_col_read = 0;
            m_e_col_write = calc_col_write_cap() * e_factor;
            //FIXME - no static power?
            break;
        default:
            fatal("Error in BITLINE model.\n");
    }
    return;
}

double BitlineUnit::calc_col_select_cap()
{
    double Wbitmuxn = m_tech_param_ptr->get_Wbitmuxn();
    return m_tech_param_ptr->calc_gatecap(Wbitmuxn, 1);
}

double BitlineUnit::calc_col_read_cap()
{
    double total_cap = 0;

    // part 1: drain cap of precharge tx's
    //total_cap = m_num_bl_pre * Util::calc_draincap(m_pre_size, Util::PCH, 1);

    // part 2: drain cap of pass tx's
    double Wmemcellr = m_tech_param_ptr->get_Wmemcellr();
    uint32_t num_row = m_sram_ptr->get_num_row();
    total_cap = num_row * m_tech_param_ptr->calc_draincap(Wmemcellr, TechParameter::NCH, 1);

    // part 3: metal cap
    total_cap += m_bl_wire_cap;
    m_pre_unit_load = total_cap;

    // part 4: bitline inverter
    uint32_t num_data_end = m_sram_ptr->get_num_data_end();
    if (num_data_end == 1)
    {
        // FIXME: magic numbers
        double MSCALE = m_tech_param_ptr->get_MSCALE();
        total_cap += m_tech_param_ptr->calc_gatecap(MSCALE * (29.9 + 7.8), 0) + m_tech_param_ptr->calc_gatecap(MSCALE * (47.0 + 12.0), 0);
    }

    // part 5: gate cap of sense amplifier or output driver
    bool is_outdrv = m_sram_ptr->get_is_outdrv();
    if (num_data_end == 2)
    { // sense amplifier
        double WsenseQ1to4 = m_tech_param_ptr->get_WsenseQ1to4();
        total_cap += 2 * m_tech_param_ptr->calc_gatecap(WsenseQ1to4, 10);
    }
    else if (is_outdrv)
    {
        double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
        double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
        double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
        double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();

        total_cap += m_tech_param_ptr->calc_gatecap(Woutdrvnandn, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnandp, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnorn, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnorp, 1);
    }

    return total_cap;
}

double BitlineUnit::calc_col_write_cap()
{
    double total_cap, psize, nsize;

    // part 1: line cap, including drain cap of pass tx's and metal cap
    uint32_t num_row = m_sram_ptr->get_num_row();
    double Wmemcellw = m_tech_param_ptr->get_Wmemcellw();
    total_cap = num_row * m_tech_param_ptr->calc_draincap(Wmemcellw, TechParameter::NCH, 1) + m_bl_wire_cap;

    // part 2: write driver
    double period = m_tech_param_ptr->get_period();
    psize = m_tech_param_ptr->calc_driver_psize(total_cap, period / 8.0);
    double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
    double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
    nsize = psize * Wdecinvn / Wdecinvp;
    total_cap += m_tech_param_ptr->calc_draincap(psize, TechParameter::PCH, 1) + m_tech_param_ptr->calc_draincap(nsize, TechParameter::NCH, 1) + m_tech_param_ptr->calc_gatecap(psize + nsize, 1);

    return total_cap;
}

double BitlineUnit::calc_i_static()
{
    double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
    double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
    double NMOS_TAB_0 = m_tech_param_ptr->get_NMOS_TAB(0);
    double PMOS_TAB_0 = m_tech_param_ptr->get_PMOS_TAB(0);

    return (2*(Wdecinvn*NMOS_TAB_0+Wdecinvp*PMOS_TAB_0));
}

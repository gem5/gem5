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
#include "mem/ruby/network/orion/Buffer/OutdrvUnit.hh"
#include "mem/ruby/network/orion/Buffer/SRAM.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

OutdrvUnit::OutdrvUnit(
        const string& outdrv_model_str_,
        const SRAM* sram_ptr_,
        const TechParameter* tech_param_ptr_
        )
{
    if (outdrv_model_str_ == string("CACHE_OUTDRV"))
    {
        m_outdrv_model = CACHE_OUTDRV;
    }
    else if (outdrv_model_str_ == string("REG_OUTDRV"))
    {
        m_outdrv_model = REG_OUTDRV;
    }
    else
    {
        m_outdrv_model = NO_MODEL;
    }

    if (m_outdrv_model != NO_MODEL)
    {
        m_sram_ptr = sram_ptr_;
        m_tech_param_ptr = tech_param_ptr_;
        init();
    }
    else
    {
        m_e_select = 0;
        m_e_out_1 = 0;
        m_e_out_0 = 0;
        m_e_chg_data = 0;
    }
}

OutdrvUnit::~OutdrvUnit()
{}

void OutdrvUnit::init()
{
    double e_factor = m_tech_param_ptr->get_EnergyFactor();

    m_e_select = calc_select_cap() * e_factor;
    m_e_out_1 = calc_outdata_cap(1) * e_factor;
    m_e_out_0 = calc_outdata_cap(0) * e_factor;

    switch(m_outdrv_model)
    {
        case CACHE_OUTDRV:
            m_e_chg_data = calc_chgdata_cap() * e_factor;
            break;
        case REG_OUTDRV:
            m_e_chg_data = 0;
            break;
        default:
            fatal("Incorrect OUTDRIVE model.\n");
    }

    m_i_static = calc_i_static();
    return;
}

double OutdrvUnit::calc_select_cap()
{
    double total_cap;

    // stage 1: inverter
    double Woutdrvseln = m_tech_param_ptr->get_Woutdrvseln();
    double Woutdrvselp = m_tech_param_ptr->get_Woutdrvselp();
    total_cap = m_tech_param_ptr->calc_gatecap(Woutdrvseln, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvselp, 1) + m_tech_param_ptr->calc_draincap(Woutdrvseln, TechParameter::NCH, 1) + m_tech_param_ptr->calc_draincap(Woutdrvselp, TechParameter::PCH, 1);

    // stage 2: gate cap of nand gate and nor gate
    // only consider 1 gate cap because another and drain cap switch depends on data value
    uint32_t line_width = m_sram_ptr->get_line_width();
    double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
    double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
    double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
    double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();
    total_cap += line_width * (m_tech_param_ptr->calc_gatecap(Woutdrvnandn, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnandp, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnorn, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnorp, 1));
    return total_cap;
}

double OutdrvUnit::calc_chgdata_cap()
{
    double total_cap;
    double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
    double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
    double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
    double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();

    total_cap = (m_tech_param_ptr->calc_gatecap(Woutdrvnandn, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnandp, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnorn, 1) + m_tech_param_ptr->calc_gatecap(Woutdrvnorp, 1)) / 2.0;
    return total_cap;
}

double OutdrvUnit::calc_outdata_cap(bool value_)
{
    double total_cap = 0;

    // stage 1: drain cap of nand gate or nor gate
    if (value_)
    {
        //drain cap of nand gate
        double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
        double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
        total_cap = m_tech_param_ptr->calc_draincap(Woutdrvnandn, TechParameter::NCH, 2) + 2 * m_tech_param_ptr->calc_draincap(Woutdrvnandp, TechParameter::PCH, 1);
    }
    else
    {
        //drain cap of nor gate
        double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
        double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();
        total_cap = 2 * m_tech_param_ptr->calc_draincap(Woutdrvnorn, TechParameter::NCH, 1) + m_tech_param_ptr->calc_draincap(Woutdrvnorp, TechParameter::PCH, 2);
    }

    // stage 2: gate cap of output inverter
    if (value_)
    {
        double Woutdriverp = m_tech_param_ptr->get_Woutdriverp();
        total_cap += m_tech_param_ptr->calc_gatecap(Woutdriverp, 1);
    }
    else
    {
        double Woutdrivern = m_tech_param_ptr->get_Woutdrivern();
        total_cap += m_tech_param_ptr->calc_gatecap(Woutdrivern, 1);
    }

    //drian cap of output inverter should be included into bus cap
    //TODO
    return total_cap;
}

double OutdrvUnit::calc_i_static()
{
    //FIXME - add static power
    return 0;
}


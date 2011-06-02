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

#include <cmath>

#include "mem/ruby/network/orion/Buffer/DecoderUnit.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

using namespace std;

DecoderUnit::DecoderUnit(
        const string& dec_model_str_,
        uint32_t dec_width_,
        const TechParameter* tech_param_ptr_
        )
{
    if (dec_model_str_.compare("GENERIC_DEC") == 0)
    {
        m_dec_model = GENERIC_DEC;
    }
    else
    {
        m_dec_model = NO_MODEL;
    }

    if (m_dec_model != NO_MODEL)
    {
        m_dec_width = dec_width_;
        m_tech_param_ptr = tech_param_ptr_;

        init();
    }
}

DecoderUnit::~DecoderUnit()
{
}

void DecoderUnit::init()
{
    if (m_dec_width >= 4)
    { // 2-level decoder
        m_num_in_1st = (m_dec_width == 4)? 2:3;
        m_num_out_0th = 1 << (m_num_in_1st - 1);
        m_num_in_2nd = (uint32_t)ceil((double)m_dec_width/(double)m_num_in_1st);
        m_num_out_1st = 1 << (m_dec_width - m_num_in_1st);
    }
    else if (m_dec_width >= 2)
    { // 1-level decoder
        m_num_in_1st = m_dec_width;
        m_num_out_0th = 1 << (m_num_in_1st - 1);
        m_num_in_2nd = m_num_out_1st = 0;
    }
    else
    {
        m_num_in_1st = m_num_out_0th = m_num_in_2nd = m_num_out_1st = 0;
    }

    // compute energy constants
    double e_factor = m_tech_param_ptr->get_vdd() * m_tech_param_ptr->get_vdd();
    if (m_dec_width >= 4)
    {
        m_e_chg_l1 = calc_chgl1_cap() * e_factor;
        m_e_chg_output = calc_select_cap() * e_factor;
    }
    else if (m_dec_width >= 2)
    {
        m_e_chg_l1 = calc_chgl1_cap() * e_factor;
        m_e_chg_output = 0;
    }
    else
    {
        m_e_chg_l1 = m_e_chg_output = 0;
    }
    m_e_chg_addr = calc_chgaddr_cap() * e_factor;

    return;
}

double DecoderUnit::calc_chgl1_cap()
{
    double total_cap;

    // part 1: drain cap of level-1 decoder
    double Wdec3to8p = m_tech_param_ptr->get_Wdec3to8p();
    double Wdec3to8n = m_tech_param_ptr->get_Wdec3to8n();
    total_cap = m_num_in_1st * m_tech_param_ptr->calc_draincap(Wdec3to8p, TechParameter::PCH, 1) + m_tech_param_ptr->calc_draincap(Wdec3to8n, TechParameter::NCH, m_num_in_1st);

    /* part 2: gate cap of level-2 decoder */
    /* WHS: 40 and 20 should go to PARM */
    double WdecNORn = m_tech_param_ptr->get_WdecNORn();
    double WdecNORp = m_tech_param_ptr->get_WdecNORp();
    total_cap += m_num_out_0th*m_tech_param_ptr->calc_gatecap((WdecNORn+WdecNORp), m_num_in_2nd*40 + 20);

    return total_cap;
}

double DecoderUnit::calc_select_cap()
{
    double total_cap;

    // part 1: drain cap of last level decoders 
    double WdecNORp = m_tech_param_ptr->get_WdecNORp();
    double WdecNORn = m_tech_param_ptr->get_WdecNORn();
    total_cap = m_num_in_2nd * m_tech_param_ptr->calc_draincap(WdecNORn, TechParameter::NCH, 1) + m_tech_param_ptr->calc_draincap(WdecNORp, TechParameter::PCH, m_num_in_2nd);

    // part 2: output inverter
    // WHS: 20 should go to PARM
    double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
    double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
    total_cap += m_tech_param_ptr->calc_draincap(Wdecinvn, TechParameter::NCH, 1) + m_tech_param_ptr->calc_draincap(Wdecinvp, TechParameter::PCH, 1) + m_tech_param_ptr->calc_gatecap(Wdecinvn + Wdecinvp, 20);

    return total_cap;
}

double DecoderUnit::calc_chgaddr_cap()
{
    double total_cap;

    // stage 1: input driver
    double Wdecdrivep = m_tech_param_ptr->get_Wdecdrivep();
    double Wdecdriven = m_tech_param_ptr->get_Wdecdriven();
    total_cap = m_tech_param_ptr->calc_draincap(Wdecdrivep, TechParameter::PCH, 1) + m_tech_param_ptr->calc_draincap(Wdecdriven, TechParameter::NCH, 1) + m_tech_param_ptr->calc_gatecap(Wdecdriven, 1);

    /* inverter to produce complement addr, this needs 1/2 */
    /* WHS: assume Wdecinv(np) for this inverter */
    double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
    double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
    total_cap += (m_tech_param_ptr->calc_draincap(Wdecinvp, TechParameter::PCH, 1) + m_tech_param_ptr->calc_draincap(Wdecinvn, TechParameter::NCH, 1) + m_tech_param_ptr->calc_gatecap(Wdecinvp, 1) + m_tech_param_ptr->calc_gatecap(Wdecinvn, 1)) / 2;

    /* stage 2: gate cap of level-1 decoder */
    /* WHS: 10 should go to PARM */
    double Wdec3to8p = m_tech_param_ptr->get_Wdec3to8p();
    double Wdec3to8n = m_tech_param_ptr->get_Wdec3to8n();
    total_cap += m_num_out_0th*m_tech_param_ptr->calc_gatecap( Wdec3to8n + Wdec3to8p, 10 );

    return total_cap;
}


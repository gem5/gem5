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

#include "mem/ruby/network/orion/Crossbar/MatrixCrossbar.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

using namespace std;

MatrixCrossbar::MatrixCrossbar(
        const string& conn_type_str_, 
        const string& trans_type_str_, 
        uint32_t num_in_,
        uint32_t num_out_,
        uint32_t data_width_,
        uint32_t num_in_seg_,
        uint32_t num_out_seg_,
        double len_in_wire_,
        double len_out_wire_,
        const TechParameter* tech_param_ptr_
        ) : Crossbar(
            MATRIX_CROSSBAR, conn_type_str_, trans_type_str_, 
            num_in_, num_out_, data_width_, num_in_seg_, num_out_seg_, 
            0, tech_param_ptr_)
{
    assert(len_in_wire_ == len_in_wire_);
    assert(len_out_wire_ == len_out_wire_);

    m_len_in_wire = len_in_wire_;
    m_len_out_wire = len_out_wire_;
    init();
}

MatrixCrossbar::~MatrixCrossbar()
{}

double MatrixCrossbar::get_dynamic_energy(bool is_max_) const
{
    double e_atomic;
    double e_access = 0;

    e_atomic = m_e_chg_in*m_data_width*(is_max_? 1:0.5);
    e_access += e_atomic;

    e_atomic = m_e_chg_out*m_data_width*(is_max_? 1:0.5);
    e_access += e_atomic;

    e_atomic = m_e_chg_ctr;
    e_access += e_atomic;

    return e_access;
}

void MatrixCrossbar::init()
{
    // FIXME: need accurate spacing
    double CrsbarCellWidth = m_tech_param_ptr->get_CrsbarCellWidth();
    double CrsbarCellHeight = m_tech_param_ptr->get_CrsbarCellHeight();
    double len_in = m_num_out*m_data_width*CrsbarCellWidth;
    double len_out = m_num_in*m_data_width*CrsbarCellHeight;
    if(len_in > m_len_in_wire) m_len_in_wire = len_in;
    if(len_out > m_len_out_wire) m_len_out_wire = len_out;
    double CC3metal = m_tech_param_ptr->get_CC3metal();
    m_cap_in_wire = CC3metal*m_len_in_wire;
    m_cap_out_wire = CC3metal*m_len_out_wire;
    double Cmetal = m_tech_param_ptr->get_Cmetal();
    m_cap_ctr_wire = Cmetal*m_len_in_wire/2.0;
    m_len_req_wire = m_len_in_wire;

    double e_factor = m_tech_param_ptr->get_EnergyFactor();
    m_e_chg_in = calc_in_cap()*e_factor;
    m_e_chg_out = calc_out_cap(m_num_out)*e_factor;
    //FIXME: wire length estimation, really reset?
    //control signal should reset after transmission is done, so no 1/2
    m_e_chg_ctr = calc_ctr_cap(m_cap_ctr_wire, 0, 0)*e_factor;
    m_e_chg_int = 0;

    m_i_static = calc_i_static();
    return;
}

double MatrixCrossbar::calc_i_static()
{
    double Woutdrvnandn = m_tech_param_ptr->get_Woutdrvnandn();
    double Woutdrvnandp = m_tech_param_ptr->get_Woutdrvnandp();
    double Woutdrvnorn = m_tech_param_ptr->get_Woutdrvnorn();
    double Woutdrvnorp = m_tech_param_ptr->get_Woutdrvnorp();
    double Wdecinvn = m_tech_param_ptr->get_Wdecinvn();
    double Wdecinvp = m_tech_param_ptr->get_Wdecinvp();
    double Woutdrivern = m_tech_param_ptr->get_Woutdrivern();
    double Woutdriverp = m_tech_param_ptr->get_Woutdriverp();
    double NAND2_TAB_0 = m_tech_param_ptr->get_NAND2_TAB(0);
    double NAND2_TAB_1 = m_tech_param_ptr->get_NAND2_TAB(1);
    double NAND2_TAB_2 = m_tech_param_ptr->get_NAND2_TAB(2);
    double NAND2_TAB_3 = m_tech_param_ptr->get_NAND2_TAB(3);
    double NOR2_TAB_0 = m_tech_param_ptr->get_NOR2_TAB(0);
    double NOR2_TAB_1 = m_tech_param_ptr->get_NOR2_TAB(1);
    double NOR2_TAB_2 = m_tech_param_ptr->get_NOR2_TAB(2);
    double NOR2_TAB_3 = m_tech_param_ptr->get_NOR2_TAB(3);
    double NMOS_TAB_0 = m_tech_param_ptr->get_NMOS_TAB(0);
    double PMOS_TAB_0 = m_tech_param_ptr->get_PMOS_TAB(0);

    double i_static = 0;
    // tri-state buffers
    i_static += ((Woutdrvnandp*(NAND2_TAB_0+NAND2_TAB_1+NAND2_TAB_2)+Woutdrvnandn*NAND2_TAB_3)/4
            + (Woutdrvnorp*NOR2_TAB_0+Woutdrvnorn*(NOR2_TAB_1+NOR2_TAB_2+NOR2_TAB_3))/4
            + Woutdrivern*NMOS_TAB_0+Woutdriverp*PMOS_TAB_0)*m_num_in*m_num_out*m_data_width;

    // input driver
    i_static += (Wdecinvn*NMOS_TAB_0+Wdecinvp*PMOS_TAB_0)*m_num_in*m_data_width;

    // output driver
    i_static += (Woutdrivern*NMOS_TAB_0+Woutdriverp*PMOS_TAB_0)*m_num_out*m_data_width;

    // control siganl inverter
    i_static += (Wdecinvn*NMOS_TAB_0+Wdecinvp*PMOS_TAB_0)*m_num_in*m_num_out;
    return i_static;
}

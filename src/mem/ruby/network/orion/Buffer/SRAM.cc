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
#include <cmath>
#include <iostream>

#include "mem/ruby/network/orion/Buffer/AmpUnit.hh"
#include "mem/ruby/network/orion/Buffer/BitlineUnit.hh"
#include "mem/ruby/network/orion/Buffer/DecoderUnit.hh"
#include "mem/ruby/network/orion/Buffer/MemUnit.hh"
#include "mem/ruby/network/orion/Buffer/OutdrvUnit.hh"
#include "mem/ruby/network/orion/Buffer/PrechargeUnit.hh"
#include "mem/ruby/network/orion/Buffer/SRAM.hh"
#include "mem/ruby/network/orion/Buffer/WordlineUnit.hh"

using namespace std;

SRAM::SRAM(
        uint32_t num_entry_,
        uint32_t line_width_,
        bool is_fifo_,
        bool is_outdrv_,
        uint32_t num_read_port_,
        uint32_t num_write_port_,
        uint32_t num_data_end_,
        const string& rowdec_model_str_,
        const string& wl_model_str_,
        const string& bl_pre_model_str_,
        const string& mem_model_str_,
        const string& bl_model_str_,
        const string& amp_model_str_,
        const string& outdrv_model_str_,
        const TechParameter* tech_param_ptr_
        )
{
    assert(num_entry_ == num_entry_);
    assert(line_width_ == line_width_);
    assert(num_read_port_ == num_read_port_);
    assert(num_write_port_ == num_write_port_);
    assert(num_data_end_ == num_data_end_);

    m_num_entry = num_entry_;
    m_line_width = line_width_;
    m_is_fifo = is_fifo_;
    m_is_outdrv = is_outdrv_;

    m_num_read_port = num_read_port_;
    m_num_write_port = num_write_port_;
    m_num_data_end = num_data_end_;

    m_rowdec_model_str =  rowdec_model_str_;
    m_wl_model_str = wl_model_str_;
    m_bl_pre_model_str = bl_pre_model_str_;
    m_mem_model_str = mem_model_str_;
    m_bl_model_str = bl_model_str_;
    m_amp_model_str = amp_model_str_;
    m_outdrv_model_str = outdrv_model_str_;
    m_tech_param_ptr = tech_param_ptr_;

    init();
}

SRAM::~SRAM()
{
    delete m_outdrv_unit_ptr;
    delete m_amp_unit_ptr;
    delete m_bl_unit_ptr;
    delete m_mem_unit_ptr;
    delete m_bl_pre_unit_ptr;
    delete m_wl_unit_ptr;
    delete m_rowdec_unit_ptr;
}

double SRAM::calc_e_read(
        bool is_max_
        ) const
{
    double e_atomic;
    double e_read = 0;

    // decoder
    if (m_rowdec_unit_ptr != NULL)
    {
        e_atomic = m_rowdec_unit_ptr->get_e_chg_addr()*m_rowdec_unit_ptr->get_dec_width()*(is_max_? 1:0.5);
        e_atomic += m_rowdec_unit_ptr->get_e_chg_output();
        // assume all 1st-level decoders change output
        e_atomic += m_rowdec_unit_ptr->get_e_chg_l1()*m_rowdec_unit_ptr->get_num_in_2nd();
        e_read += e_atomic;
    }

    //wordline
    e_atomic = m_wl_unit_ptr->get_e_read();
    e_read += e_atomic;

    //bitline pre
    e_atomic = m_bl_pre_unit_ptr->get_e_charge_gate()*m_line_width;
    if (m_num_data_end == 2)
    {
        e_atomic += m_bl_pre_unit_ptr->get_e_charge_drain()*m_line_width;
    }
    else
    {
        e_atomic += m_bl_pre_unit_ptr->get_e_charge_drain()*m_line_width*(is_max_? 1:0.5);
    }
    e_read += e_atomic;

    //bitline
    if (m_num_data_end == 2)
    {
        e_atomic = m_bl_unit_ptr->get_e_col_read()*m_line_width;
    }
    else
    {
        e_atomic = m_bl_unit_ptr->get_e_col_read()*m_line_width*(is_max_? 1:0.5);
    }
    e_read += e_atomic;

    if (m_num_data_end == 2)
    {
        e_atomic = m_amp_unit_ptr->get_e_access()*m_line_width;
        e_read += e_atomic;
    }

    if (m_outdrv_unit_ptr != NULL)
    {
        e_atomic = m_outdrv_unit_ptr->get_e_select();

        e_atomic += m_outdrv_unit_ptr->get_e_chg_data()*m_line_width*(is_max_? 1:0.5);

        //assume 1 and 0 are uniformly distributed
        if ((m_outdrv_unit_ptr->get_e_out_1() >= m_outdrv_unit_ptr->get_e_out_0()) || (!is_max_))
        {
            e_atomic += m_outdrv_unit_ptr->get_e_out_1()*m_line_width*(is_max_? 1:0.5);
        }
        if ((m_outdrv_unit_ptr->get_e_out_1() < m_outdrv_unit_ptr->get_e_out_0()) || (!is_max_))
        {
            e_atomic += m_outdrv_unit_ptr->get_e_out_0()*m_line_width*(is_max_? 1:0.5);
        }

        e_read += e_atomic;
    }

    return e_read;
}

double SRAM::calc_e_write(
        bool is_max_
        ) const
{
    double e_atomic;
    double e_write = 0;

    // decoder
    if (m_rowdec_unit_ptr != NULL)
    {
        e_atomic = m_rowdec_unit_ptr->get_e_chg_addr()*m_rowdec_unit_ptr->get_dec_width()*(is_max_? 1:0.5);
        e_atomic += m_rowdec_unit_ptr->get_e_chg_output();
        // assume all 1st-level decoders change output
        e_atomic += m_rowdec_unit_ptr->get_e_chg_l1()*m_rowdec_unit_ptr->get_num_in_2nd();
        e_write += e_atomic;
    }

    //wordline
    e_atomic = m_wl_unit_ptr->get_e_write();
    e_write += e_atomic;

    //bitline
    e_atomic = m_bl_unit_ptr->get_e_col_wrtie()*m_line_width*(is_max_? 1:0.5);
    e_write += e_atomic;

    //mem cell
    e_atomic = m_mem_unit_ptr->get_e_switch()*m_line_width*(is_max_? 1:0.5);
    e_write += e_atomic;

    return e_write;
}

double SRAM::calc_i_static() const
{
    double i_static = 0;

    i_static += m_bl_unit_ptr->get_i_static()*m_line_width*m_num_write_port;
    i_static += m_mem_unit_ptr->get_i_static()*m_num_entry*m_line_width;
    i_static += m_bl_pre_unit_ptr->get_i_static()*m_line_width*m_num_read_port;
    i_static += m_wl_unit_ptr->get_i_static()*m_num_entry*(m_num_read_port+m_num_write_port);

    return i_static;
}

void SRAM::init()
{
    // output driver unit
    if (m_is_outdrv)
    {
        m_outdrv_unit_ptr = new OutdrvUnit(m_outdrv_model_str, this, m_tech_param_ptr);
    }
    else
    {
        m_outdrv_unit_ptr = NULL;
    }

    // sense amplifier unit
    if (m_num_data_end == 2)
    {
        m_amp_unit_ptr = new AmpUnit(m_amp_model_str, m_tech_param_ptr);
    }
    else
    {
        m_amp_unit_ptr = NULL;
    }

    // bitline unit
    m_bl_unit_ptr = new BitlineUnit(m_bl_model_str, this, m_tech_param_ptr);

    // mem unit
    m_mem_unit_ptr = new MemUnit(m_mem_model_str, this, m_tech_param_ptr);

    // precharge unit
    double bl_pre_unit_load = m_bl_unit_ptr->get_pre_unit_load();
    m_bl_pre_unit_ptr = new PrechargeUnit(m_bl_pre_model_str, bl_pre_unit_load, this, m_tech_param_ptr);

    // wordline unit
    m_wl_unit_ptr = new WordlineUnit(m_wl_model_str, this, m_tech_param_ptr);

    // decode unit
    if (!m_is_fifo)
    {
        m_rowdec_width = (uint32_t)log2((double)m_num_entry);
        m_rowdec_unit_ptr = new DecoderUnit(m_rowdec_model_str, m_rowdec_width, m_tech_param_ptr);
    }
    else
    {
        m_rowdec_unit_ptr = NULL;
    }
    return;
}



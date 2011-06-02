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

#include "mem/ruby/network/orion/Buffer/Buffer.hh"
#include "mem/ruby/network/orion/Buffer/Register.hh"
#include "mem/ruby/network/orion/Buffer/SRAM.hh"
#include "mem/ruby/network/orion/OrionConfig.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

using namespace std;

Buffer::Buffer(
        const string& buffer_model_str_,
        bool is_fifo_,
        bool is_outdrv_,
        uint32_t num_entry_,
        uint32_t line_width_,
        uint32_t num_read_port_,
        uint32_t num_write_port_,
        const OrionConfig* orion_cfg_ptr_
        )
{
    if (buffer_model_str_ == string("SRAM"))
    {
        m_buffer_model = BUF_SRAM;
    }
    else if(buffer_model_str_ == string("REGISTER"))
    {
        m_buffer_model = BUF_REG;
    }
    else
    {
        m_buffer_model = NO_MODEL;
    }

    if (m_buffer_model != NO_MODEL)
    {
        assert(num_entry_ == num_entry_);
        assert(line_width_ == line_width_);
        assert(num_read_port_ == num_read_port_);
        assert(num_write_port_ == num_write_port_);

        m_num_entry = num_entry_;
        m_line_width = line_width_;
        m_is_fifo = is_fifo_;
        m_is_outdrv = is_outdrv_;
        m_num_read_port = num_read_port_;
        m_num_write_port = num_write_port_;

        m_orion_cfg_ptr = orion_cfg_ptr_;
        m_tech_param_ptr = m_orion_cfg_ptr->get_tech_param_ptr();

        init();
    }
}

Buffer::~Buffer()
{
    delete m_sram_ptr;
}

double Buffer::get_dynamic_energy(
        bool is_read_,
        bool is_max_
        ) const
{
    if (m_buffer_model == BUF_SRAM)
    {
        if (is_read_)
        {
            return m_sram_ptr->calc_e_read(is_max_);
        }
        else
        {
            return m_sram_ptr->calc_e_write(is_max_);
        }
    }
    else if (m_buffer_model == BUF_REG)
    {
        if (is_read_)
        {
            return m_reg_ptr->calc_e_read();
        }
        else
        {
            return m_reg_ptr->calc_e_write();
        }
    }
    else
    {
        return 0;
    }
}

double Buffer::get_static_power() const
{
    double vdd = m_tech_param_ptr->get_vdd();
    double SCALE_S = m_tech_param_ptr->get_SCALE_S();
    if (m_buffer_model == BUF_SRAM)
    {
        return m_sram_ptr->calc_i_static()*vdd*SCALE_S;
    }
    else if (m_buffer_model == BUF_REG)
    {
        return m_reg_ptr->calc_i_static()*vdd*SCALE_S;
    }
    else
    {
        return 0;
    }
}

void Buffer::print_all() const
{
    cout << "Buffer" << endl;
    cout << "\t" << "Read = " << get_dynamic_energy(true, false) << endl;
    cout << "\t" << "Write = " << get_dynamic_energy(false, false) << endl;
    cout << "\t" << "Static power = " << get_static_power() << endl;
    return;
}

void Buffer::init()
{
    if(m_buffer_model == BUF_SRAM)
    {
        uint32_t num_data_end = m_orion_cfg_ptr->get<uint32_t>("SRAM_NUM_DATA_END");
        const string& rowdec_model_str = m_orion_cfg_ptr->get<string>("SRAM_ROWDEC_MODEL");
        const string& wl_model_str = m_orion_cfg_ptr->get<string>("SRAM_WORDLINE_MODEL");
        const string& bl_pre_model_str = m_orion_cfg_ptr->get<string>("SRAM_BITLINE_PRE_MODEL");
        const string& mem_model_str = "NORMAL_MEM";
        const string& bl_model_str = m_orion_cfg_ptr->get<string>("SRAM_BITLINE_MODEL");
        const string& amp_model_str = m_orion_cfg_ptr->get<string>("SRAM_AMP_MODEL");
        const string& outdrv_model_str = m_orion_cfg_ptr->get<string>("SRAM_OUTDRV_MODEL");
        m_sram_ptr = new SRAM(
                m_num_entry, m_line_width, m_is_fifo, m_is_outdrv, 
                m_num_read_port, m_num_write_port, num_data_end, 
                rowdec_model_str, wl_model_str, bl_pre_model_str, mem_model_str, 
                bl_model_str, amp_model_str, outdrv_model_str, m_tech_param_ptr);
    }
    else if (m_buffer_model == BUF_REG)
    {
        m_sram_ptr = NULL;
        m_reg_ptr = new Register(m_num_entry, m_line_width, m_tech_param_ptr);
    }
    else
    {
        m_sram_ptr = NULL;
        m_reg_ptr = NULL;
    }
    return;
}


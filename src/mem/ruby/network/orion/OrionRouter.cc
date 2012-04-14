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

#include "mem/ruby/network/orion/Allocator/SWAllocator.hh"
#include "mem/ruby/network/orion/Allocator/VCAllocator.hh"
#include "mem/ruby/network/orion/Buffer/Buffer.hh"
#include "mem/ruby/network/orion/Crossbar/Crossbar.hh"
#include "mem/ruby/network/orion/Clock.hh"
#include "mem/ruby/network/orion/OrionConfig.hh"
#include "OrionRouter.hh"

using namespace std;

OrionRouter::OrionRouter(
    uint32_t num_in_port_,
    uint32_t num_out_port_,
    uint32_t num_vclass_,
    std::vector<uint32_t > vclass_type_ary_,
    uint32_t num_vc_per_vclass_,
    uint32_t in_buf_per_data_vc_,
    uint32_t in_buf_per_ctrl_vc_,
    uint32_t flit_width_,
    OrionConfig* orion_cfg_ptr_
)
{
    assert((num_in_port_ == num_in_port_) && (num_in_port_ != 0));
    assert((num_out_port_ == num_out_port_) && (num_out_port_ != 0));
    assert((num_vclass_ == num_vclass_) && (num_vclass_ != 0));
    assert((num_vc_per_vclass_ == num_vc_per_vclass_) && (num_vc_per_vclass_ != 0));
    assert(in_buf_per_data_vc_ != 0);
    assert(in_buf_per_ctrl_vc_ != 0);
    assert((flit_width_ == flit_width_) && (flit_width_ != 0));

    orion_cfg_ptr_->set_num_in_port(num_in_port_);
    orion_cfg_ptr_->set_num_out_port(num_out_port_);
    orion_cfg_ptr_->set_num_vclass(num_vclass_);
    orion_cfg_ptr_->set_flit_width(flit_width_);
    m_orion_cfg_ptr = orion_cfg_ptr_;

    m_num_in_port = m_orion_cfg_ptr->get<uint32_t>("NUM_INPUT_PORT");
    m_num_out_port = m_orion_cfg_ptr->get<uint32_t>("NUM_OUTPUT_PORT");
    m_flit_width = m_orion_cfg_ptr->get<uint32_t>("FLIT_WIDTH");
    m_num_vclass = m_orion_cfg_ptr->get<uint32_t>("NUM_VIRTUAL_CLASS");

    m_num_vc_per_vclass_ary = new uint32_t [m_num_vclass];
    m_in_buf_num_set_ary = new uint32_t [m_num_vclass];
    for (int i = 0; i < m_num_vclass; i++)
    {
        // can also suppport different vcs per vclass
        m_num_vc_per_vclass_ary[i] = num_vc_per_vclass_;

        if (vclass_type_ary_[i] == 0) // ctrl
            m_in_buf_num_set_ary[i] = in_buf_per_ctrl_vc_;
        else if (vclass_type_ary_[i] == 1) // data
            m_in_buf_num_set_ary[i] = in_buf_per_data_vc_;
        else
            assert(0);
    }

    init();
}

OrionRouter::~OrionRouter()
{
    delete[] m_num_vc_per_vclass_ary;
    delete[] m_in_buf_num_set_ary;

    if (m_in_buf_ary_ptr)
    {
        for (uint32_t i = 0; i < m_num_vclass; i++)
        {
            delete m_in_buf_ary_ptr[i];
        }
        delete[] m_in_buf_ary_ptr;
    }

    if (m_va_ary_ptr)
    {
        for (uint32_t i = 0; i < m_num_vclass; i++)
        {
            delete m_va_ary_ptr[i];
        }
        delete[] m_va_ary_ptr;
    }

    delete m_xbar_ptr;
    delete m_sa_ptr;
    delete m_clk_ptr;
}

double OrionRouter::calc_dynamic_energy_buf(uint32_t vclass_id_, bool is_read_, bool is_max_) const
{
    assert(vclass_id_ < m_num_vclass);
    if (m_in_buf_ary_ptr)
    {
        if (m_in_buf_ary_ptr[vclass_id_])
        {
            return m_in_buf_ary_ptr[vclass_id_]->get_dynamic_energy(is_read_, is_max_);
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

double OrionRouter::calc_dynamic_energy_xbar(bool is_max_) const
{
    if (m_xbar_ptr)
    {
        return m_xbar_ptr->get_dynamic_energy(is_max_);
    }
    else
    {
        return 0;
    }
}

double OrionRouter::calc_dynamic_energy_local_vc_arb(uint32_t vclass_id_, double num_req_, bool is_max_) const
{
    assert(vclass_id_ < m_num_vclass);

    if (m_va_ary_ptr)
    {
        if (m_va_ary_ptr[vclass_id_])
        {
            return m_va_ary_ptr[vclass_id_]->get_dynamic_energy_local_vc_arb(num_req_, is_max_);
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

double OrionRouter::calc_dynamic_energy_global_vc_arb(uint32_t vclass_id_, double num_req_, bool is_max_) const
{
    assert(vclass_id_ < m_num_vclass);

    if (m_va_ary_ptr)
    {
        if (m_va_ary_ptr[vclass_id_])
        {
            return m_va_ary_ptr[vclass_id_]->get_dynamic_energy_global_vc_arb(num_req_, is_max_);
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

//double OrionRouter::calc_dynamic_energy_vc_select(bool is_read_, bool is_max_) const
//{
//  if (m_vc_select_ptr)
//  {
//    return m_vc_select_ptr->get_dynamic_energy_vc_select(is_read_, is_max_);
//  }
//  else
//  {
//    return 0;
//  }
//}

double OrionRouter::calc_dynamic_energy_local_sw_arb(double num_req_, bool is_max_) const
{
    if (m_sa_ptr)
    {
        return m_sa_ptr->get_dynamic_energy_local_sw_arb(num_req_, is_max_);
    }
    else
    {
        return 0;
    }
}

double OrionRouter::calc_dynamic_energy_global_sw_arb(double num_req_, bool is_max_) const
{
    if (m_sa_ptr)
    {
        return m_sa_ptr->get_dynamic_energy_global_sw_arb(num_req_, is_max_);
    }
    else
    {
        return 0;
    }
}

double OrionRouter::calc_dynamic_energy_clock() const
{
    if (m_clk_ptr)
    {
        return m_clk_ptr->get_dynamic_energy();
    }
    else
    {
        return 0;
    }
}

double OrionRouter::get_static_power_buf() const
{
    if (m_in_buf_ary_ptr)
    {
        double total_static_power = 0;

        for (uint32_t i = 0; i < m_num_vclass; i++)
        {
            uint32_t num_in_buf;
            if (m_is_in_shared_buf)
            {
                num_in_buf = m_num_in_port;
            }
            else
            {
                num_in_buf = m_num_vc_per_vclass_ary[i]*m_num_in_port;
            }
            total_static_power += m_in_buf_ary_ptr[i]->get_static_power()*(double)num_in_buf;
        }
        return total_static_power;
    }
    else
    {
        return 0;
    }
}

double OrionRouter::get_static_power_xbar() const
{
    if (m_xbar_ptr)
    {
        return m_xbar_ptr->get_static_power();
    }
    else
    {
        return 0;
    }
}

double OrionRouter::get_static_power_va() const
{
    if (m_va_ary_ptr)
    {
        double total_static_power = 0;

        for (uint32_t i = 0; i < m_num_vclass; i++)
        {
            total_static_power += m_va_ary_ptr[i]->get_static_power();
        }
        return total_static_power;
    }
    else
    {
        return 0;
    }
}

//double OrionRouter::get_static_power_vc_select() const
//{
//  if (m_vc_select_ptr)
//  {
//    return m_vc_select_ptr->get_static_power();
//  }
//  else
//  {
//    return 0;
//  }
//}

double OrionRouter::get_static_power_sa() const
{
    if (m_sa_ptr)
    {
        return m_sa_ptr->get_static_power();
    }
    else
    {
        return 0;
    }
}

double OrionRouter::get_static_power_clock() const
{
    if (m_clk_ptr)
    {
        return m_clk_ptr->get_static_power();
    }
    else
    {
        return 0;
    }
}

void OrionRouter::init()
{
    m_total_num_vc = 0;
    for (uint32_t i = 0; i < m_num_vclass; i++)
    {
        m_total_num_vc += m_num_vc_per_vclass_ary[i];
    }

    if (m_total_num_vc > 1)
    {
        m_is_in_shared_buf = m_orion_cfg_ptr->get<bool>("IS_IN_SHARED_BUFFER");
        m_is_out_shared_buf = m_orion_cfg_ptr->get<bool>("IS_OUT_SHARED_BUFFER");
        m_is_in_shared_switch = m_orion_cfg_ptr->get<bool>("IS_IN_SHARED_SWITCH");
        m_is_out_shared_switch = m_orion_cfg_ptr->get<bool>("IS_OUT_SHARED_SWITCH");
    }
    else
    {
        m_is_in_shared_buf = false;
        m_is_out_shared_buf = false;
        m_is_in_shared_switch = false;
        m_is_out_shared_switch = false;
    }

    //input buffer
    bool is_in_buf = m_orion_cfg_ptr->get<bool>("IS_INPUT_BUFFER");
    if (is_in_buf)
    {
        bool is_fifo = true;
        bool is_outdrv = (!m_is_in_shared_buf) && (m_is_in_shared_switch);
        const string& in_buf_model_str = m_orion_cfg_ptr->get<string>("IN_BUF_MODEL");
        m_in_buf_ary_ptr = new Buffer* [m_num_vclass];
        for (uint32_t i = 0; i < m_num_vclass; i++)
        {
            uint32_t in_buf_num_read_port = m_orion_cfg_ptr->get<uint32_t>("IN_BUF_NUM_READ_PORT");
            uint32_t in_buf_num_set = m_in_buf_num_set_ary[i];
            m_in_buf_ary_ptr[i] = new Buffer(in_buf_model_str, is_fifo, is_outdrv, 
                    in_buf_num_set, m_flit_width, in_buf_num_read_port, 1, m_orion_cfg_ptr);
        }
    }
    else
    {
        m_in_buf_ary_ptr = NULL;
    }

    bool is_out_buf = m_orion_cfg_ptr->get<bool>("IS_OUTPUT_BUFFER");

    //crossbar
    uint32_t num_switch_in;
    if (is_in_buf)
    {
        if (m_is_in_shared_buf)
        {
            uint32_t in_buf_num_read_port = m_orion_cfg_ptr->get<uint32_t>("IN_BUF_NUM_READ_PORT");
            num_switch_in = in_buf_num_read_port*m_num_in_port;
        }
        else if (m_is_in_shared_switch)
        {
            num_switch_in = 1*m_num_in_port;
        }
        else
        {
            num_switch_in = m_total_num_vc*m_num_in_port;
        }
    }
    else
    {
        num_switch_in = 1*m_num_in_port;
    }
    uint32_t num_switch_out;
    if (is_out_buf)
    {
        if (m_is_out_shared_buf)
        {
            uint32_t out_buf_num_write_port = m_orion_cfg_ptr->get<uint32_t>("OUT_BUF_NUM_WRITE_PORT");
            num_switch_out = out_buf_num_write_port*m_num_out_port;
        }
        else if (m_is_out_shared_switch)
        {
            num_switch_out = 1*m_num_out_port;
        }
        else
        {
            num_switch_out = m_total_num_vc*m_num_out_port;
        }
    }
    else
    {
        num_switch_out = 1*m_num_out_port;
    }
    const string& xbar_model_str = m_orion_cfg_ptr->get<string>("CROSSBAR_MODEL");
    m_xbar_ptr = Crossbar::create_crossbar(xbar_model_str, 
            num_switch_in, num_switch_out, m_flit_width, m_orion_cfg_ptr);

    //vc allocator
    const string& va_model_str = m_orion_cfg_ptr->get<string>("VA_MODEL");
    m_va_ary_ptr = new VCAllocator* [m_num_vclass];
    //m_vc_select_ary_ptr = new VCAllocator* [m_num_vclass];
    for (uint32_t i = 0; i < m_num_vclass; i++)
    {
        m_va_ary_ptr[i] = VCAllocator::create_vcallocator(va_model_str,
                m_num_in_port, m_num_out_port, 1, m_num_vc_per_vclass_ary[i],
                m_orion_cfg_ptr);
        //m_vc_select_ary_ptr[i] = VCAllocator::create_vcallocator("VC_SELECT",
        //  m_num_in_port, m_num_out_port, 1, m_num_vc_per_vclass_ary[i], m_orion_cfg_ptr);
    }

    //sw allocator
    m_sa_ptr = SWAllocator::create_swallocator(
            m_num_in_port, m_num_out_port, 1, m_total_num_vc, 
            m_xbar_ptr, m_orion_cfg_ptr);

    //cloc
    m_clk_ptr = new Clock(is_in_buf, m_is_in_shared_switch, is_out_buf, m_is_out_shared_switch, m_orion_cfg_ptr);

    return;
}

void OrionRouter::print() const
{
    if (m_in_buf_ary_ptr)
    {
        for (uint32_t i = 0; i < m_num_vclass; i++)
        {
            cout << "VClass " << i << endl;
            if (m_in_buf_ary_ptr[i]) m_in_buf_ary_ptr[i]->print_all();
        }
    }
    m_xbar_ptr->print_all();
    for (uint32_t i = 0; i < m_num_vclass; i++)
    {
        cout << "VClass " << i << endl;
        m_va_ary_ptr[i]->print_all();
        //m_vc_select_ary_ptr[i]->print_all();
    }
    m_sa_ptr->print_all();

    //cout << "Router - Dynamic Energy" << endl;
    //cout << "\t" << "Buffer Read = " << calc_dynamic_energy_buf(true) << endl;
    //cout << "\t" << "Buffer Write = " << calc_dynamic_energy_buf(false) << endl;
    //cout << "\t" << "Crossbar = " << calc_dynamic_energy_xbar() << endl;
    //cout << "\t" << "Local VC Allocator(1) = " << calc_dynamic_energy_local_vc_arb(1) << endl;
    //cout << "\t" << "Global VC Allocator(1) = " << calc_dynamic_energy_global_vc_arb(1) << endl;
    //cout << "\t" << "VC Select Read = " << calc_dynamic_energy_vc_select(true) << endl;
    //cout << "\t" << "VC Select Write = " << calc_dynamic_energy_vc_select(false) << endl;
    //cout << "\t" << "Local SW Allocator(2) = " << calc_dynamic_energy_local_sw_arb(1) << endl;
    //cout << "\t" << "Global SW Allocator(2) = " << calc_dynamic_energy_global_sw_arb(1) << endl;
    //cout << "\t" << "Clock = " << calc_dynamic_energy_clock() << endl;
    //cout << endl;
    //cout << "Router - Static Power" << endl;
    //cout << "\t" << "Buffer = " << get_static_power_buf() << endl;
    //cout << "\t" << "Crossbar = " << get_static_power_xbar() << endl;
    //cout << "\t" << "VC Allocator = " << get_static_power_va() << endl;
    //cout << "\t" << "SW Allocator = " << get_static_power_sa() << endl;
    //cout << "\t" << "Clock = " << get_static_power_clock() << endl;
    //cout << endl;
    return;
}


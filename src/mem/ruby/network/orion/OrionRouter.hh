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

#ifndef __ORIONROUTER_H__
#define __ORIONROUTER_H__

#include <vector>

#include "mem/ruby/network/orion/Type.hh"

class OrionConfig;
class Buffer;
class Crossbar;
class VCAllocator;
class SWAllocator;
class Clock;

class OrionRouter
{
  public:
    OrionRouter(
        const OrionConfig* orion_cfg_ptr_
    );

    //values in cfg file will be modified
    OrionRouter(
      uint32_t num_in_port_,
      uint32_t num_out_port_,
      uint32_t num_vclass_,
      std::vector<uint32_t > vclass_type_,
      uint32_t num_vc_per_vclass_,
      uint32_t in_buf_per_data_vc_,
      uint32_t in_buf_per_ctrl_vc_,
      uint32_t flit_width_,
      OrionConfig* orion_cfg_ptr_
    );
    ~OrionRouter();

  public:
    //double calc_dynamic_energy(double e_fin_, bool is_max_ = false) const;
    //double calc_dynamic_energy_in_buf(bool is_read_, bool is_max_ = false) const;
    //double calc_dynamic_energy_out_buf(bool is_read_, bool is_max_ = false) const;
    double calc_dynamic_energy_buf(uint32_t vclass_id_, bool is_read_, bool is_max_ = false) const;
    double calc_dynamic_energy_xbar(bool is_max_ = false) const;
    double calc_dynamic_energy_local_vc_arb(uint32_t vclass_id_, double num_req_, bool is_max_ = false) const;
    double calc_dynamic_energy_global_vc_arb(uint32_t vclass_id_, double num_req_, bool is_max_ = false) const;
    //double calc_dynamic_energy_vc_select(uint32_t vclass_id_, bool is_read_, bool is_max_ = false) const;
    double calc_dynamic_energy_local_sw_arb(double num_req_, bool is_max_ = false) const;
    double calc_dynamic_energy_global_sw_arb(double num_req_, bool is_max_ = false) const;
    double calc_dynamic_energy_clock() const;

    double get_static_power_buf() const;
    double get_static_power_xbar() const;
    double get_static_power_va() const;
    double get_static_power_sa() const;
    double get_static_power_clock() const;

    void print() const;

    void init();

  private:
    const OrionConfig* m_orion_cfg_ptr;

    uint32_t m_num_in_port;
    uint32_t m_num_out_port;
    uint32_t m_flit_width;
    uint32_t m_num_vclass;
    uint32_t num_vc_per_vclass;
    uint32_t m_total_num_vc;
    uint32_t* m_num_vc_per_vclass_ary;
    uint32_t* m_in_buf_num_set_ary;
    bool m_is_in_shared_buf;
    bool m_is_out_shared_buf;
    bool m_is_in_shared_switch;
    bool m_is_out_shared_switch;

    Buffer** m_in_buf_ary_ptr;
    Crossbar* m_xbar_ptr;
    VCAllocator** m_va_ary_ptr;
    //VCAllocator** m_vc_select_ary_ptr;
    SWAllocator* m_sa_ptr;
    Clock* m_clk_ptr;
};

#endif


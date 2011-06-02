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

#ifndef __SWALLOCATOR_H__
#define __SWALLOCATOR_H__

#include "mem/ruby/network/orion/Type.hh"

class TechParameter;
class OrionConfig;
class Arbiter;
class Crossbar;

class SWAllocator
{
  protected:
    SWAllocator(
      uint32_t num_in_port_,
      uint32_t num_out_port_,
      uint32_t num_vclass_,
      uint32_t num_vchannel_,
      double len_in_wire_,
      const string& local_arb_model_str_,
      const string& local_arb_ff_model_str_,
      const string& global_arb_model_str_,
      const string& global_arb_ff_model_str_,
      const TechParameter* tech_param_ptr_
    );

  public:
    ~SWAllocator();

  public:
    double get_dynamic_energy_local_sw_arb(double num_req_, bool is_max_) const;
    double get_dynamic_energy_global_sw_arb(double num_req_, bool is_max_) const;
    double get_static_power() const;

    void print_all() const;

  protected:
    void init();

  protected:
    uint32_t m_num_in_port;
    uint32_t m_num_out_port;
    uint32_t m_num_vclass;
    uint32_t m_num_vchannel;

    Arbiter* m_local_arb_ptr;
    Arbiter* m_global_arb_ptr;

  public:
    static SWAllocator* create_swallocator(
      uint32_t num_in_port_,
      uint32_t num_out_port_,
      uint32_t num_vclass_,
      uint32_t num_vchannel_,
      const Crossbar* xbar_ptr_,
      const OrionConfig* orion_cfg_ptr_
    );
};

#endif

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

#ifndef __WIRE_H__
#define __WIRE_H__

#include "mem/ruby/network/orion/Type.hh"

class TechParameter;

class Wire
{
  public:
    enum WidthSpacingModel
    {
      SWIDTH_SSPACE,
      SWIDTH_DSPACE,
      DWIDTH_SSPACE,
      DWIDTH_DSPACE
    };
    enum BufferScheme
    {
      MIN_DELAY,
      STAGGERED
    };

  public:
    Wire(
      const string& wire_spacing_model_str_,
      const string& buf_scheme_str_,
      bool is_shielding_,
      const TechParameter* tech_param_ptr_
    );
    ~Wire();

  public:
    void calc_opt_buffering(int* k_, double* h_, double len_) const;
    double calc_dynamic_energy(double len_) const;
    double calc_static_power(double len_) const;

  private:
    void init();
    void set_width_spacing_model(const string& wire_spacing_model_str_);
    void set_buf_scheme(const string& buf_scheme_str_);
    double calc_res_unit_len();
    double calc_gnd_cap_unit_len();
    double calc_couple_cap_unit_len();

  private:
    const TechParameter* m_tech_param_ptr;
    WidthSpacingModel m_width_spacing_model;
    BufferScheme m_buf_scheme;
    bool m_is_shielding;

    double m_res_unit_len;
    double m_gnd_cap_unit_len;
    double m_couple_cap_unit_len;
};

#endif

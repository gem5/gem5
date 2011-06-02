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

#ifndef __ARBITER_H__
#define __ARBITER_H__

#include "mem/ruby/network/orion/Type.hh"

class TechParameter;
class FlipFlop;

class Arbiter
{
  public:
    enum ArbiterModel
    {
      NO_MODEL = 0,
      RR_ARBITER,
      MATRIX_ARBITER
    };

  public:
    Arbiter(const ArbiterModel arb_model_,
            const uint32_t req_width_,
            const double len_in_wire_,
            const TechParameter* tech_param_ptr_);
    virtual ~Arbiter() = 0;

  public:
    virtual double calc_dynamic_energy(double num_req_, bool is_max_) const = 0;
    double get_static_power() const;

  protected:
    ArbiterModel m_arb_model;
    uint32_t m_req_width;
    double m_len_in_wire;
    const TechParameter* m_tech_param_ptr;

    FlipFlop* m_ff_ptr;

    double m_e_chg_req;
    double m_e_chg_grant;

    double m_i_static;

  public:
    static Arbiter* create_arbiter(const string& arb_model_str_,
                                   const string& ff_model_str_,
                                   uint32_t req_width_,
                                   double len_in_wire_,
                                   const TechParameter* tech_param_ptr_);
};

#endif


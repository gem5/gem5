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

#ifndef __DECODERUNIT_H__
#define __DECODERUNIT_H__

#include "mem/ruby/network/orion/TechParameter.hh"
#include "mem/ruby/network/orion/Type.hh"

class DecoderUnit
{
  public:
    enum DecModel
    {
      NO_MODEL = 0,
      GENERIC_DEC
    };

  public:
    DecoderUnit(
      const string& dec_model_str_, 
      uint32_t dec_width_, 
      const TechParameter* tech_param_ptr_
    );
    ~DecoderUnit();

  public:
    uint32_t get_dec_width() const { return m_dec_width; }
    uint32_t get_num_in_2nd() const { return m_num_in_2nd; }
    double get_e_chg_addr() const { return m_e_chg_addr; }
    double get_e_chg_output() const { return m_e_chg_output; }
    double get_e_chg_l1() const { return m_e_chg_l1; }

  private:
    void init();
    double calc_chgl1_cap();
    double calc_select_cap();
    double calc_chgaddr_cap();

  private:
    DecModel m_dec_model;
    uint32_t m_dec_width;
    const TechParameter* m_tech_param_ptr;

    uint32_t m_num_in_1st;
    uint32_t m_num_in_2nd;
    uint32_t m_num_out_0th;
    uint32_t m_num_out_1st;

    double m_e_chg_l1;
    double m_e_chg_output;
    double m_e_chg_addr;
};

#endif


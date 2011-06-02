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

#include "mem/ruby/network/orion/OrionConfig.hh"
#include "mem/ruby/network/orion/OrionLink.hh"
#include "mem/ruby/network/orion/Wire.hh"

using namespace std;

OrionLink::OrionLink(
  double len_,
  uint32_t line_width_,
  const OrionConfig* orion_cfg_ptr_
)
{
    assert(len_ == len_);
    assert(line_width_ == line_width_);

    m_len = len_;
    m_line_width = line_width_;
    m_orion_cfg_ptr = orion_cfg_ptr_;

    init();
}

OrionLink::~OrionLink()
{}

double OrionLink::calc_dynamic_energy(uint32_t num_bit_flip_) const
{
    assert(num_bit_flip_ <= m_line_width);
    return (num_bit_flip_*(m_dynamic_energy_per_bit/2));
}

double OrionLink::get_static_power() const
{
    return (m_line_width*m_static_power_per_bit);
}

void OrionLink::init()
{
    const TechParameter* tech_param_ptr = m_orion_cfg_ptr->get_tech_param_ptr();

    const string& width_spacing_model_str = m_orion_cfg_ptr->get<string>("WIRE_WIDTH_SPACING");
    const string& buf_scheme_str = m_orion_cfg_ptr->get<string>("WIRE_BUFFERING_MODEL");
    bool is_shielding = m_orion_cfg_ptr->get<bool>("WIRE_IS_SHIELDING");
    Wire wire(width_spacing_model_str, buf_scheme_str, is_shielding, tech_param_ptr);

    m_dynamic_energy_per_bit = wire.calc_dynamic_energy(m_len);
    m_static_power_per_bit = wire.calc_static_power(m_len);
    return;
}

void OrionLink::print() const
{
    cout << "Link - Dynamic Energy" << endl;
    cout << "\t" << "One Bit = " << calc_dynamic_energy(1) << endl;
    cout << endl;
    cout << "Link - Static Power" << endl;
    cout << "\t" << "One Bit = " << get_static_power() << endl;
    cout << endl;
    return;
}

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

#include "mem/ruby/network/orion/Allocator/Arbiter.hh"
#include "mem/ruby/network/orion/Allocator/MatrixArbiter.hh"
#include "mem/ruby/network/orion/Allocator/RRArbiter.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

using namespace std;

Arbiter::Arbiter(const ArbiterModel arb_model_,
                 const uint32_t req_width_,
                 const double len_in_wire_,
                 const TechParameter* tech_param_ptr_)
{
    assert(req_width_ == req_width_);
    assert(len_in_wire_ == len_in_wire_);

    m_arb_model = arb_model_;
    m_req_width = req_width_;
    m_len_in_wire = len_in_wire_;
    m_tech_param_ptr = tech_param_ptr_;
}

Arbiter::~Arbiter()
{}

double 
Arbiter::get_static_power() const
{
    double vdd = m_tech_param_ptr->get_vdd();
    double SCALE_S = m_tech_param_ptr->get_SCALE_S();

    return m_i_static*vdd*SCALE_S;
}

Arbiter* 
Arbiter::create_arbiter(const string& arb_model_str_, 
                        const string& ff_model_str_,
                        uint32_t req_width_, 
                        double len_in_wire_, 
                        const TechParameter* tech_param_ptr_)
{
    if (arb_model_str_ == string("RR_ARBITER")) {

        return new RRArbiter(ff_model_str_, req_width_, 
                             len_in_wire_, tech_param_ptr_);

    } else if (arb_model_str_ == string("MATRIX_ARBITER")) {

        return new MatrixArbiter(ff_model_str_, req_width_, 
                                 len_in_wire_, tech_param_ptr_);

    } else {
        cerr << "WARNING: No Arbiter model" << endl;
        return (Arbiter*)NULL;
    }
}

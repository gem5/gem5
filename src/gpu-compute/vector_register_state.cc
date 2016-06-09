/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: John Kalamatianos
 */

#include "gpu-compute/vector_register_state.hh"

#include <limits>

#include "gpu-compute/compute_unit.hh"

VecRegisterState::VecRegisterState() : computeUnit(nullptr)
{
    s_reg.clear();
    d_reg.clear();
}

void
VecRegisterState::setParent(ComputeUnit *_computeUnit)
{
    computeUnit = _computeUnit;
    _name = computeUnit->name() + ".VecRegState";
}

void
VecRegisterState::init(uint32_t _size, uint32_t wf_size)
{
    s_reg.resize(_size);
    fatal_if(wf_size > std::numeric_limits<unsigned long long>::digits ||
             wf_size <= 0,
             "WF size is larger than the host can support or is zero");
    fatal_if((wf_size & (wf_size - 1)) != 0,
             "Wavefront size should be a power of 2");
    for (int i = 0; i < s_reg.size(); ++i) {
        s_reg[i].resize(wf_size, 0);
    }
    d_reg.resize(_size);
    for (int i = 0; i < d_reg.size(); ++i) {
        d_reg[i].resize(wf_size, 0);
    }
}

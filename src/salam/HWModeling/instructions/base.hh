/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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
 */

#ifndef __HWMODEL_INST_CONFIG_BASE_HH__
#define __HWMODEL_INST_CONFIG_BASE_HH__

#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>

class InstConfigBase
{
  private:
  protected:
    uint32_t _functional_unit;
    uint32_t _functional_unit_limit;
    uint32_t _opcode_num;
    uint32_t _runtime_cycles;

  public:
    InstConfigBase();
    InstConfigBase(uint32_t functional_unit, uint32_t functional_unit_limit,
                   uint32_t opcode_num, uint32_t runtime_cycles)
    {}
    uint32_t
    get_functional_unit()
    {
        return _functional_unit;
    }
    uint32_t
    get_functional_unit_limit()
    {
        return _functional_unit_limit;
    }
    uint32_t
    get_opcode_num()
    {
        return _opcode_num;
    }
    uint32_t
    get_runtime_cycles()
    {
        return _runtime_cycles;
    }
};
#endif // __HWMODEL_INST_CONFIG_BASE_HH__

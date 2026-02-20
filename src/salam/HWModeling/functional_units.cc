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

#include "functional_units.hh"

// GENERATED CONSTRUCTOR - DO NOT MODIFY
FunctionalUnits::FunctionalUnits(const FunctionalUnitsParams &params)
    : SimObject(params),
      _float_divider(params.float_divider),
      _float_adder(params.float_adder),
      _double_multiplier(params.double_multiplier),
      _double_adder(params.double_adder),
      _bit_shifter(params.bit_shifter),
      _integer_multiplier(params.integer_multiplier),
      _bit_register(params.bit_register),
      _double_divider(params.double_divider),
      _float_multiplier(params.float_multiplier),
      _integer_adder(params.integer_adder),
      _bitwise_operations(params.bitwise_operations)
{
    functional_unit_list.push_back(_float_divider);
    functional_unit_list.push_back(_float_adder);
    functional_unit_list.push_back(_double_multiplier);
    functional_unit_list.push_back(_double_adder);
    functional_unit_list.push_back(_bit_shifter);
    functional_unit_list.push_back(_integer_multiplier);
    functional_unit_list.push_back(_bit_register);
    functional_unit_list.push_back(_double_divider);
    functional_unit_list.push_back(_float_multiplier);
    functional_unit_list.push_back(_integer_adder);
    functional_unit_list.push_back(_bitwise_operations);
}
// END OF GENERATED CONSTRUCTOR

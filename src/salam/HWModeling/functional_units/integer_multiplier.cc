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

#include "integer_multiplier.hh"

// AUTO-GENERATED FILE (See util/SALAM-docs/README_SALAM.md for details)

IntegerMultiplier::IntegerMultiplier(const IntegerMultiplierParams &params)
    : SimObject(params),
      FunctionalUnitBase(
          params.alias, params.stages, params.cycles, params.enum_value,
          params.int_size, params.int_sign, params.int_apmode, params.fp_size,
          params.fp_sign, params.fp_apmode, params.ptr_size, params.ptr_sign,
          params.ptr_apmode, params.limit, params.power_units,
          params.energy_units, params.time_units, params.area_units,
          params.fu_latency, params.internal_power, params.switch_power,
          params.dynamic_power, params.dynamic_energy, params.leakage_power,
          params.area, params.path_delay)
{}

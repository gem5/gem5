/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 */

#ifndef __HWMODEL_FUNCTIONAL_UNITS_HH__
#define __HWMODEL_FUNCTIONAL_UNITS_HH__

#include "params/FunctionalUnits.hh"
#include "sim/sim_object.hh"

// AUTO-GENERATED HEADERS (See util/SALAM-docs/README_SALAM.md for details)
#include <cstdlib>
#include <iostream>
#include <vector>

#include "functional_units/base.hh"
#include "functional_units/bit_register.hh"
#include "functional_units/bit_shifter.hh"
#include "functional_units/bitwise_operations.hh"
#include "functional_units/double_adder.hh"
#include "functional_units/double_divider.hh"
#include "functional_units/double_multiplier.hh"
#include "functional_units/float_adder.hh"
#include "functional_units/float_divider.hh"
#include "functional_units/float_multiplier.hh"
#include "functional_units/integer_adder.hh"
#include "functional_units/integer_multiplier.hh"

using namespace gem5;

class FunctionalUnitBase;

class FunctionalUnits : public SimObject
{
  private:
  protected:
  public:
    // AUTO-GENERATED CLASS MEMBERS (See util/SALAM-docs/README_SALAM.md for
    // details)
    FloatDivider *_float_divider;
    FloatAdder *_float_adder;
    DoubleMultiplier *_double_multiplier;
    DoubleAdder *_double_adder;
    BitShifter *_bit_shifter;
    IntegerMultiplier *_integer_multiplier;
    BitRegister *_bit_register;
    DoubleDivider *_double_divider;
    FloatMultiplier *_float_multiplier;
    IntegerAdder *_integer_adder;
    BitwiseOperations *_bitwise_operations;
    FunctionalUnits();
    // DEFAULT CONSTRUCTOR (See util/SALAM-docs/README_SALAM.md for details)
    FunctionalUnits(const FunctionalUnitsParams &params);
    // END DEFAULT CONSTRUCTOR
    std::vector<FunctionalUnitBase *> functional_unit_list;
};
#endif //__HWMODEL_FUNCTIONAL_UNITS_HH__

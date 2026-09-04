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

#include "hw_interface.hh"

HWInterface::HWInterface(const HWInterfaceParams &params)
    : SimObject(params),
      cycle_counts(params.cycle_counts),
      functional_units(params.functional_units),
      hw_statistics(params.hw_statistics),
      inst_config(params.inst_config),
      opcodes(params.opcodes),
      salam_power_model(params.salam_power_model),
      simulator_config(params.simulator_config)
{}

bool
HWInterface::availableFunctionalUnit(uint64_t functional_unit)
{
    switch (functional_unit) {
        case INTADDER: {
            if (functional_units->_integer_adder->is_available()) {
                functional_units->_integer_adder->use_functional_unit();
                return true;
            }
            break;
        }
        case INTMULTI: {
            if (functional_units->_integer_multiplier->is_available()) {
                functional_units->_integer_multiplier->use_functional_unit();
                return true;
            }
            break;
        }
        case INTSHIFTER: {
            if (functional_units->_bit_shifter->is_available()) {
                functional_units->_bit_shifter->use_functional_unit();
                return true;
            }
            break;
        }
        case INTBITWISE: {
            if (functional_units->_bitwise_operations->is_available()) {
                functional_units->_bitwise_operations->use_functional_unit();
                return true;
            }
            break;
        }
        case FPSPADDER: {
            if (functional_units->_float_adder->is_available()) {
                functional_units->_float_adder->use_functional_unit();
                return true;
            }
            break;
        }
        case FPDPADDER: {
            if (functional_units->_double_adder->is_available()) {
                functional_units->_double_adder->use_functional_unit();
                return true;
            }
            break;
        }
        case FPSPMULTI: {
            if (functional_units->_float_multiplier->is_available()) {
                functional_units->_float_multiplier->use_functional_unit();
                return true;
            }
            break;
        }
        case FPSPDIVID: {
            if (functional_units->_float_divider->is_available()) {
                functional_units->_float_divider->use_functional_unit();
                return true;
            }
            break;
        }
        case FPDPMULTI: {
            if (functional_units->_double_multiplier->is_available()) {
                functional_units->_double_multiplier->use_functional_unit();
                return true;
            }
            break;
        }
        case FPDPDIVID: {
            if (functional_units->_double_divider->is_available()) {
                functional_units->_double_divider->use_functional_unit();
                return true;
            }
            break;
        }
        case COMPARE: {
            break;
        }
        case GETELEMENTPTR: {
            break;
        }
        case CONVERSION: {
            break;
        }
        case OTHERINST: {
            break;
        }
        case REGISTER: {
            if (functional_units->_bit_register->is_available()) {
                functional_units->_bit_register->use_functional_unit();
                return true;
            }
            break;
        }
        case COUNTER: {
            break;
        }
        default: {
            return false;
        }
    }
    return false;
}

void
HWInterface::clearFunctionalUnit(uint64_t unit)
{
    switch (unit) {
        case INTADDER: {
            functional_units->_integer_adder->clear_functional_unit();
            break;
        }
        case INTMULTI: {
            functional_units->_integer_multiplier->clear_functional_unit();
            break;
        }
        case INTSHIFTER: {
            functional_units->_bit_shifter->clear_functional_unit();
            break;
        }
        case INTBITWISE: {
            functional_units->_bitwise_operations->clear_functional_unit();
            break;
        }
        case FPSPADDER: {
            functional_units->_float_adder->clear_functional_unit();
            break;
        }
        case FPDPADDER: {
            functional_units->_double_adder->clear_functional_unit();
            break;
        }
        case FPSPMULTI: {
            functional_units->_float_multiplier->clear_functional_unit();
            break;
        }
        case FPSPDIVID: {
            functional_units->_float_divider->clear_functional_unit();
            break;
        }
        case FPDPMULTI: {
            functional_units->_double_multiplier->clear_functional_unit();
            break;
        }
        case FPDPDIVID: {
            functional_units->_double_divider->clear_functional_unit();
            break;
        }
        case COMPARE: {
            break;
        }
        case GETELEMENTPTR: {
            break;
        }
        case CONVERSION: {
            break;
        }
        case OTHERINST: {
            break;
        }
        case REGISTER: {
            functional_units->_bit_register->clear_functional_unit();
            break;
        }
        case COUNTER: {
            break;
        }
        default: {
        }
    }
}

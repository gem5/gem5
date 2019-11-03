/*
 * Copyright 2018 Google, Inc.
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
 * Authors: Gabe Black
 */

#include "systemc/ext/dt/bit/messages.hh"
#include "systemc/utils/report.hh"

namespace sc_core
{

const char SC_ID_LENGTH_MISMATCH_[] =
    "length mismatch in bit/logic vector assignment";
const char SC_ID_INCOMPATIBLE_TYPES_[] = "incompatible types";
const char SC_ID_CANNOT_CONVERT_[] = "cannot perform conversion";
const char SC_ID_INCOMPATIBLE_VECTORS_[] = "incompatible vectors";
const char SC_ID_VALUE_NOT_VALID_[] = "value is not valid";
const char SC_ID_ZERO_LENGTH_[] = "zero length";
const char SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_[] =
    "vector contains 4-value logic";
const char SC_ID_SC_BV_CANNOT_CONTAIN_X_AND_Z_[] =
    "sc_bv cannot contain values X and Z";
const char SC_ID_VECTOR_TOO_LONG_[] = "vector is too long: truncated";
const char SC_ID_VECTOR_TOO_SHORT_[] = "vector is too short: 0-padded";
const char SC_ID_WRONG_VALUE_[] = "wrong value";
const char SC_ID_LOGIC_Z_TO_BOOL_[] =
    "sc_logic value 'Z' cannot be converted to bool";
const char SC_ID_LOGIC_X_TO_BOOL_[] =
    "sc_logic value 'X' cannot be converted to bool";

namespace
{

sc_gem5::DefaultReportMessages predefinedMessages{
    {200, SC_ID_LENGTH_MISMATCH_},
    {201, SC_ID_INCOMPATIBLE_TYPES_},
    {202, SC_ID_CANNOT_CONVERT_},
    {203, SC_ID_INCOMPATIBLE_VECTORS_},
    {204, SC_ID_VALUE_NOT_VALID_},
    {205, SC_ID_ZERO_LENGTH_},
    {206, SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_},
    {207, SC_ID_SC_BV_CANNOT_CONTAIN_X_AND_Z_},
    {208, SC_ID_VECTOR_TOO_LONG_},
    {209, SC_ID_VECTOR_TOO_SHORT_},
    {210, SC_ID_WRONG_VALUE_},
    {211, SC_ID_LOGIC_Z_TO_BOOL_},
    {212, SC_ID_LOGIC_X_TO_BOOL_}
};

} // anonymous namespace

} // namespace sc_core

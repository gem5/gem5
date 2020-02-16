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
 */

#include "systemc/ext/dt/fx/messages.hh"
#include "systemc/utils/report.hh"

namespace sc_core
{

const char SC_ID_INVALID_WL_[] = "total wordlength <= 0 is not valid";
const char SC_ID_INVALID_N_BITS_[] = "number of bits < 0 is not valid";
const char SC_ID_INVALID_DIV_WL_[] = "division wordlength <= 0 is not valid";
const char SC_ID_INVALID_CTE_WL_[] = "constant wordlength <= 0 is not valid";
const char SC_ID_INVALID_MAX_WL_[] =
    "maximum wordlength <= 0 and != -1 is not valid";
const char SC_ID_INVALID_FX_VALUE_[] = "invalid fixed-point value";
const char SC_ID_INVALID_O_MODE_[] = "invalid overflow mode";
const char SC_ID_OUT_OF_RANGE_[] = "index out of range";
const char SC_ID_CONTEXT_BEGIN_FAILED_[] = "context begin failed";
const char SC_ID_CONTEXT_END_FAILED_[] = "context end failed";
const char SC_ID_WRAP_SM_NOT_DEFINED_[] =
    "SC_WRAP_SM not defined for unsigned numbers";

namespace
{

sc_gem5::DefaultReportMessages predefinedMessages{
    {300, SC_ID_INVALID_WL_},
    {301, SC_ID_INVALID_N_BITS_},
    {302, SC_ID_INVALID_DIV_WL_},
    {303, SC_ID_INVALID_CTE_WL_},
    {304, SC_ID_INVALID_MAX_WL_},
    {305, SC_ID_INVALID_FX_VALUE_},
    {306, SC_ID_INVALID_O_MODE_},
    {307, SC_ID_OUT_OF_RANGE_},
    {308, SC_ID_CONTEXT_BEGIN_FAILED_},
    {309, SC_ID_CONTEXT_END_FAILED_},
    {310, SC_ID_WRAP_SM_NOT_DEFINED_}
};

} // anonymous namespace

} // namespace sc_core

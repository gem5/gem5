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

#include "systemc/ext/utils/messages.hh"
#include "systemc/utils/report.hh"

namespace sc_core
{

const char SC_ID_REGISTER_ID_FAILED_[] = "register_id failed";
const char SC_ID_UNKNOWN_ERROR_[] = "unknown error";
const char SC_ID_WITHOUT_MESSAGE_[] = "";
const char SC_ID_NOT_IMPLEMENTED_[] = "not implemented";
const char SC_ID_INTERNAL_ERROR_[] = "internal error";
const char SC_ID_ASSERTION_FAILED_[] = "assertion failed";
const char SC_ID_OUT_OF_BOUNDS_[] = "out of bounds";
const char SC_ID_ABORT_[] = "simulation aborted";

const char SC_ID_STRING_TOO_LONG_[] = "string is too long";
const char SC_ID_FRONT_ON_EMPTY_LIST_[] =
    "attempt to take front() on an empty list";
const char SC_ID_BACK_ON_EMPTY_LIST_[] =
    "attempt to take back() on an empty list";
const char SC_ID_IEEE_1666_DEPRECATION_[] = "/IEEE_Std_1666/deprecated";
const char SC_ID_VECTOR_INIT_CALLED_TWICE_[] =
    "sc_vector::init has already been called";
const char SC_ID_VECTOR_BIND_EMPTY_[] =
    "sc_vector::bind called with empty range";
const char SC_ID_VECTOR_NONOBJECT_ELEMENTS_[] =
    "sc_vector::get_elements called for element type "
    "not derived from sc_object";

namespace
{

sc_gem5::DefaultReportMessages predefinedMessages{
    { 800, SC_ID_REGISTER_ID_FAILED_ },
    { 0, SC_ID_UNKNOWN_ERROR_ },
    { 1, SC_ID_WITHOUT_MESSAGE_ },
    { 2, SC_ID_NOT_IMPLEMENTED_ },
    { 3, SC_ID_INTERNAL_ERROR_ },
    { 4, SC_ID_ASSERTION_FAILED_ },
    { 5, SC_ID_OUT_OF_BOUNDS_ },

    { 99, SC_ID_ABORT_ },

    { 801, SC_ID_STRING_TOO_LONG_ },
    { 802, SC_ID_FRONT_ON_EMPTY_LIST_ },
    { 803, SC_ID_BACK_ON_EMPTY_LIST_ },
    { 804, SC_ID_IEEE_1666_DEPRECATION_ },
    { 805, SC_ID_VECTOR_INIT_CALLED_TWICE_ },
    { 807, SC_ID_VECTOR_BIND_EMPTY_ },
    { 808, SC_ID_VECTOR_NONOBJECT_ELEMENTS_ }
};

} // anonymous namespace

} // namespace sc_core

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

#ifndef __SYSTEMC_EXT_UTILS_MESSAGES_HH__
#define __SYSTEMC_EXT_UTILS_MESSAGES_HH__

namespace sc_core
{

extern const char SC_ID_UNKNOWN_ERROR_[];
extern const char SC_ID_WITHOUT_MESSAGE_[];
extern const char SC_ID_NOT_IMPLEMENTED_[];
extern const char SC_ID_INTERNAL_ERROR_[];
extern const char SC_ID_ASSERTION_FAILED_[];
extern const char SC_ID_OUT_OF_BOUNDS_[];
extern const char SC_ID_ABORT_[];

extern const char SC_ID_REGISTER_ID_FAILED_[];
extern const char SC_ID_STRING_TOO_LONG_[];
extern const char SC_ID_FRONT_ON_EMPTY_LIST_[];
extern const char SC_ID_BACK_ON_EMPTY_LIST_[];
extern const char SC_ID_IEEE_1666_DEPRECATION_[];
extern const char SC_ID_VECTOR_INIT_CALLED_TWICE_[];
extern const char SC_ID_VECTOR_BIND_EMPTY_[];
extern const char SC_ID_VECTOR_NONOBJECT_ELEMENTS_[];

} // namespace sc_core

#endif // __SYSTEMC_EXT_UTILS_MESSAGES_HH__

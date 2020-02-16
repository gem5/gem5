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

#ifndef __SYSTEMC_EXT_CHANNEL_MESSAGES_HH__
#define __SYSTEMC_EXT_CHANNEL_MESSAGES_HH__

namespace sc_core
{

extern const char SC_ID_PORT_OUTSIDE_MODULE_[];
extern const char SC_ID_CLOCK_PERIOD_ZERO_[];
extern const char SC_ID_CLOCK_HIGH_TIME_ZERO_[];
extern const char SC_ID_CLOCK_LOW_TIME_ZERO_[];
extern const char SC_ID_MORE_THAN_ONE_FIFO_READER_[];
extern const char SC_ID_MORE_THAN_ONE_FIFO_WRITER_[];
extern const char SC_ID_INVALID_FIFO_SIZE_[];
extern const char SC_ID_BIND_IF_TO_PORT_[];
extern const char SC_ID_BIND_PORT_TO_PORT_[];
extern const char SC_ID_COMPLETE_BINDING_[];
extern const char SC_ID_INSERT_PORT_[];
extern const char SC_ID_REMOVE_PORT_[];
extern const char SC_ID_GET_IF_[];
extern const char SC_ID_INSERT_PRIM_CHANNEL_[];
extern const char SC_ID_REMOVE_PRIM_CHANNEL_[];
extern const char SC_ID_MORE_THAN_ONE_SIGNAL_DRIVER_[];
extern const char SC_ID_NO_DEFAULT_EVENT_[];
extern const char SC_ID_RESOLVED_PORT_NOT_BOUND_[];
extern const char SC_ID_FIND_EVENT_[];
extern const char SC_ID_INVALID_SEMAPHORE_VALUE_[];
extern const char SC_ID_SC_EXPORT_HAS_NO_INTERFACE_[];
extern const char SC_ID_INSERT_EXPORT_[];
extern const char SC_ID_EXPORT_OUTSIDE_MODULE_[];
extern const char SC_ID_SC_EXPORT_NOT_REGISTERED_[];
extern const char SC_ID_SC_EXPORT_NOT_BOUND_AFTER_CONSTRUCTION_[];
extern const char SC_ID_ATTEMPT_TO_WRITE_TO_CLOCK_[];
extern const char SC_ID_SC_EXPORT_ALREADY_BOUND_[];
extern const char SC_ID_OPERATION_ON_NON_SPECIALIZED_SIGNAL_[];
extern const char SC_ID_ATTEMPT_TO_BIND_CLOCK_TO_OUTPUT_[];
extern const char SC_ID_NO_ASYNC_UPDATE_[];

} // namespace sc_core

#endif  // __SYSTEMC_EXT_CHANNEL_MESSAGES_HH__

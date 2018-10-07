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

#ifndef __SYSTEMC_EXT_CHANNEL__USING_HH__
#define __SYSTEMC_EXT_CHANNEL__USING_HH__

#include "_channel.hh"

using sc_core::sc_buffer;

using sc_core::sc_clock;
using sc_core::sc_in_clk;
using sc_core::sc_inout_clk;
using sc_core::sc_out_clk;

using sc_core::sc_event_queue_if;
using sc_core::sc_event_queue;
using sc_core::sc_event_queue_port;

using sc_core::sc_fifo;

using sc_core::sc_fifo_in;

using sc_core::sc_fifo_nonblocking_in_if;
using sc_core::sc_fifo_blocking_in_if;
using sc_core::sc_fifo_in_if;

using sc_core::sc_fifo_out;

using sc_core::sc_fifo_nonblocking_out_if;
using sc_core::sc_fifo_blocking_out_if;
using sc_core::sc_fifo_out_if;

using sc_core::sc_in;

using sc_core::sc_in_resolved;

using sc_core::sc_in_rv;

using sc_core::sc_inout;

using sc_core::sc_inout_resolved;

using sc_core::sc_inout_rv;

using sc_core::sc_mutex;

using sc_core::sc_mutex_if;

using sc_core::sc_out;

using sc_core::sc_out_resolved;

using sc_core::sc_out_rv;

using sc_core::sc_semaphore;

using sc_core::sc_semaphore_if;

using sc_core::sc_signal;

using sc_core::sc_signal_in_if;

using sc_core::SC_ONE_WRITER;
using sc_core::SC_MANY_WRITERS;
using sc_core::sc_signal_write_if;
using sc_core::sc_signal_inout_if;
using sc_core::sc_signal_out_if;

using sc_core::sc_signal_resolved;

using sc_core::sc_signal_rv;

using sc_core::SC_ID_PORT_OUTSIDE_MODULE_;
using sc_core::SC_ID_CLOCK_PERIOD_ZERO_;
using sc_core::SC_ID_CLOCK_HIGH_TIME_ZERO_;
using sc_core::SC_ID_CLOCK_LOW_TIME_ZERO_;
using sc_core::SC_ID_MORE_THAN_ONE_FIFO_READER_;
using sc_core::SC_ID_MORE_THAN_ONE_FIFO_WRITER_;
using sc_core::SC_ID_INVALID_FIFO_SIZE_;
using sc_core::SC_ID_BIND_IF_TO_PORT_;
using sc_core::SC_ID_BIND_PORT_TO_PORT_;
using sc_core::SC_ID_COMPLETE_BINDING_;
using sc_core::SC_ID_INSERT_PORT_;
using sc_core::SC_ID_REMOVE_PORT_;
using sc_core::SC_ID_GET_IF_;
using sc_core::SC_ID_INSERT_PRIM_CHANNEL_;
using sc_core::SC_ID_REMOVE_PRIM_CHANNEL_;
using sc_core::SC_ID_MORE_THAN_ONE_SIGNAL_DRIVER_;
using sc_core::SC_ID_NO_DEFAULT_EVENT_;
using sc_core::SC_ID_RESOLVED_PORT_NOT_BOUND_;
using sc_core::SC_ID_FIND_EVENT_;
using sc_core::SC_ID_INVALID_SEMAPHORE_VALUE_;
using sc_core::SC_ID_SC_EXPORT_HAS_NO_INTERFACE_;
using sc_core::SC_ID_INSERT_EXPORT_;
using sc_core::SC_ID_EXPORT_OUTSIDE_MODULE_;
using sc_core::SC_ID_SC_EXPORT_NOT_REGISTERED_;
using sc_core::SC_ID_SC_EXPORT_NOT_BOUND_AFTER_CONSTRUCTION_;
using sc_core::SC_ID_ATTEMPT_TO_WRITE_TO_CLOCK_;
using sc_core::SC_ID_SC_EXPORT_ALREADY_BOUND_;
using sc_core::SC_ID_OPERATION_ON_NON_SPECIALIZED_SIGNAL_;
using sc_core::SC_ID_ATTEMPT_TO_BIND_CLOCK_TO_OUTPUT_;
using sc_core::SC_ID_NO_ASYNC_UPDATE_;

#endif  //__SYSTEMC_EXT_CHANNEL__USING_HH__

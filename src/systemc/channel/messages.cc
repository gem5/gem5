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

#include "systemc/ext/channel/messages.hh"
#include "systemc/utils/report.hh"

namespace sc_core
{

const char SC_ID_PORT_OUTSIDE_MODULE_[] = "port specified outside of module";
const char SC_ID_CLOCK_PERIOD_ZERO_[] = "sc_clock period is zero";
const char SC_ID_CLOCK_HIGH_TIME_ZERO_[] = "sc_clock high time is zero";
const char SC_ID_CLOCK_LOW_TIME_ZERO_[] = "sc_clock low time is zero";
const char SC_ID_MORE_THAN_ONE_FIFO_READER_[] =
    "sc_fifo<T> cannot have more than one reader";
const char SC_ID_MORE_THAN_ONE_FIFO_WRITER_[] =
    "sc_fifo<T> cannot have more than one writer";
const char SC_ID_INVALID_FIFO_SIZE_[] =
    "sc_fifo<T> must have a size of at least 1";
const char SC_ID_BIND_IF_TO_PORT_[] = "bind interface to port failed";
const char SC_ID_BIND_PORT_TO_PORT_[] = "bind parent port to port failed";
const char SC_ID_COMPLETE_BINDING_[] = "complete binding failed";
const char SC_ID_INSERT_PORT_[] = "insert port failed";
const char SC_ID_REMOVE_PORT_[] = "remove port failed";
const char SC_ID_GET_IF_[] = "get interface failed";
const char SC_ID_INSERT_PRIM_CHANNEL_[] = "insert primitive channel failed";
const char SC_ID_REMOVE_PRIM_CHANNEL_[] = "remove primitive channel failed";
const char SC_ID_MORE_THAN_ONE_SIGNAL_DRIVER_[] =
    "sc_signal<T> cannot have more than one driver";
const char SC_ID_NO_DEFAULT_EVENT_[] = "channel doesn't have a default event";
const char SC_ID_RESOLVED_PORT_NOT_BOUND_[] =
    "resolved port not bound to resolved signal";
const char SC_ID_FIND_EVENT_[] = "find event failed";
const char SC_ID_INVALID_SEMAPHORE_VALUE_[] =
    "sc_semaphore requires an initial value >= 0";
const char SC_ID_SC_EXPORT_HAS_NO_INTERFACE_[] =
    "sc_export instance has no interface";
const char SC_ID_INSERT_EXPORT_[] = "insert sc_export failed";
const char SC_ID_EXPORT_OUTSIDE_MODULE_[] =
    "sc_export specified outside of module";
const char SC_ID_SC_EXPORT_NOT_REGISTERED_[] =
    "remove sc_export failed, sc_export not registered";
const char SC_ID_SC_EXPORT_NOT_BOUND_AFTER_CONSTRUCTION_[] =
    "sc_export instance not bound to interface at end of construction";
const char SC_ID_ATTEMPT_TO_WRITE_TO_CLOCK_[] =
    "attempt to write the value of an sc_clock instance";
const char SC_ID_SC_EXPORT_ALREADY_BOUND_[] =
    "sc_export instance already bound";
const char SC_ID_OPERATION_ON_NON_SPECIALIZED_SIGNAL_[] =
    "attempted specalized signal operation on non-specialized signal";
const char SC_ID_ATTEMPT_TO_BIND_CLOCK_TO_OUTPUT_[] =
    "attempted to bind sc_clock instance to sc_inout or sc_out";
const char SC_ID_NO_ASYNC_UPDATE_[] =
    "this build has no asynchronous update support";

namespace
{

sc_gem5::DefaultReportMessages predefinedMessages{
    {100, SC_ID_PORT_OUTSIDE_MODULE_},
    {101, SC_ID_CLOCK_PERIOD_ZERO_},
    {102, SC_ID_CLOCK_HIGH_TIME_ZERO_},
    {103, SC_ID_CLOCK_LOW_TIME_ZERO_},
    {104, SC_ID_MORE_THAN_ONE_FIFO_READER_},
    {105, SC_ID_MORE_THAN_ONE_FIFO_WRITER_},
    {106, SC_ID_INVALID_FIFO_SIZE_},
    {107, SC_ID_BIND_IF_TO_PORT_},
    {108, SC_ID_BIND_PORT_TO_PORT_},
    {109, SC_ID_COMPLETE_BINDING_},
    {110, SC_ID_INSERT_PORT_},
    {111, SC_ID_REMOVE_PORT_},
    {112, SC_ID_GET_IF_},
    {113, SC_ID_INSERT_PRIM_CHANNEL_},
    {114, SC_ID_REMOVE_PRIM_CHANNEL_},
    {115, SC_ID_MORE_THAN_ONE_SIGNAL_DRIVER_},
    {116, SC_ID_NO_DEFAULT_EVENT_},
    {117, SC_ID_RESOLVED_PORT_NOT_BOUND_},
    {118, SC_ID_FIND_EVENT_},
    {119, SC_ID_INVALID_SEMAPHORE_VALUE_},
    {120, SC_ID_SC_EXPORT_HAS_NO_INTERFACE_},
    {121, SC_ID_INSERT_EXPORT_},
    {122, SC_ID_EXPORT_OUTSIDE_MODULE_},
    {123, SC_ID_SC_EXPORT_NOT_REGISTERED_},
    {124, SC_ID_SC_EXPORT_NOT_BOUND_AFTER_CONSTRUCTION_},
    {125, SC_ID_ATTEMPT_TO_WRITE_TO_CLOCK_},
    {126, SC_ID_SC_EXPORT_ALREADY_BOUND_},
    {127, SC_ID_OPERATION_ON_NON_SPECIALIZED_SIGNAL_},
    {128, SC_ID_ATTEMPT_TO_BIND_CLOCK_TO_OUTPUT_},
    {129, SC_ID_NO_ASYNC_UPDATE_}
};

} // anonymous namespace

} // namespace sc_core

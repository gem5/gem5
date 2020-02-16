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

#include "systemc/ext/core/messages.hh"
#include "systemc/utils/report.hh"

namespace sc_core
{

const char SC_ID_NO_BOOL_RETURNED_[] = "operator does not return boolean";
const char SC_ID_NO_INT_RETURNED_[] = "operator does not return int";
const char SC_ID_NO_SC_LOGIC_RETURNED_[] = "operator does not return sc_logic";
const char SC_ID_OPERAND_NOT_SC_LOGIC_[] = "operand is not sc_logic";
const char SC_ID_OPERAND_NOT_BOOL_[] = "operand is not bool";
const char SC_ID_INSTANCE_EXISTS_[] = "object already exists";
const char SC_ID_ILLEGAL_CHARACTERS_[] = "illegal characters";
const char SC_ID_VC6_PROCESS_HELPER_[] =
    "internal error: sc_vc6_process_helper";
const char SC_ID_VC6_MAX_PROCESSES_EXCEEDED_[] =
    "maximum number of processes per module exceeded (VC6)";
const char SC_ID_END_MODULE_NOT_CALLED_[] =
    "module construction not properly completed: did "
    "you forget to add a sc_module_name parameter to "
    "your module constructor?";
const char SC_ID_HIER_NAME_INCORRECT_[] =
    "hierarchical name as shown may be incorrect due to previous errors";
const char SC_ID_SET_STACK_SIZE_[] =
    "set_stack_size() is only allowed for SC_THREADs and SC_CTHREADs";
const char SC_ID_SC_MODULE_NAME_USE_[] = "incorrect use of sc_module_name";
const char SC_ID_SC_MODULE_NAME_REQUIRED_[] =
    "an sc_module_name parameter for your constructor is required";
const char SC_ID_SET_TIME_RESOLUTION_[] = "set time resolution failed";
const char SC_ID_SET_DEFAULT_TIME_UNIT_[] = "set default time unit failed";
const char SC_ID_DEFAULT_TIME_UNIT_CHANGED_[] =
    "default time unit changed to time resolution";
const char SC_ID_INCONSISTENT_API_CONFIG_[] =
    "inconsistent library configuration detected";
const char SC_ID_WAIT_NOT_ALLOWED_[] =
    "wait() is only allowed in SC_THREADs and SC_CTHREADs";
const char SC_ID_NEXT_TRIGGER_NOT_ALLOWED_[] =
    "next_trigger() is only allowed in SC_METHODs";
const char SC_ID_IMMEDIATE_NOTIFICATION_[] =
    "immediate notification is not allowed during update phase or elaboration";
const char SC_ID_HALT_NOT_ALLOWED_[] = "halt() is only allowed in SC_CTHREADs";
const char SC_ID_WATCHING_NOT_ALLOWED_[] =
    "watching() has been deprecated, use reset_signal_is()";
const char SC_ID_DONT_INITIALIZE_[] =
    "dont_initialize() has no effect for SC_CTHREADs";
const char SC_ID_WAIT_N_INVALID_[] = "wait(n) is only valid for n > 0";
const char SC_ID_MAKE_SENSITIVE_[] = "make sensitive failed";
const char SC_ID_MAKE_SENSITIVE_POS_[] = "make sensitive pos failed";
const char SC_ID_MAKE_SENSITIVE_NEG_[] = "make sensitive neg failed";
const char SC_ID_INSERT_MODULE_[] = "insert module failed";
const char SC_ID_REMOVE_MODULE_[] = "remove module failed";
const char SC_ID_NOTIFY_DELAYED_[] =
    "notify_delayed() cannot be called on events "
    "that have pending notifications";
const char SC_ID_GEN_UNIQUE_NAME_[] =
    "cannot generate unique name from null string";
const char SC_ID_MODULE_NAME_STACK_EMPTY_[] =
    "module name stack is empty: did you forget to "
    "add a sc_module_name parameter to your module "
    "constructor?";
const char SC_ID_NAME_EXISTS_[] = "name already exists";
const char SC_ID_IMMEDIATE_SELF_NOTIFICATION_[] =
    "immediate self-notification ignored as of IEEE 1666-2011";
const char SC_ID_WAIT_DURING_UNWINDING_[] =
    "wait() not allowed during unwinding";
const char SC_ID_CYCLE_MISSES_EVENTS_[] =
    "the simulation contains timed-events but they are "
    "ignored by sc_cycle() ==> the simulation will be "
    "incorrect";
const char SC_ID_RETHROW_UNWINDING_[] =
    "sc_unwind_exception not re-thrown during kill/reset";
const char SC_ID_PROCESS_ALREADY_UNWINDING_[] =
    "kill/reset ignored during unwinding";
const char SC_ID_MODULE_METHOD_AFTER_START_[] =
    "call to SC_METHOD in sc_module while simulation running";
const char SC_ID_MODULE_THREAD_AFTER_START_[] =
    "call to SC_THREAD in sc_module while simulation running";
const char SC_ID_MODULE_CTHREAD_AFTER_START_[] =
    "call to SC_CTHREAD in sc_module while simulation running";
const char SC_ID_SIMULATION_TIME_OVERFLOW_[] =
    "simulation time value overflow, simulation aborted";
const char SC_ID_SIMULATION_STOP_CALLED_TWICE_[] =
    "sc_stop has already been called";
const char SC_ID_SIMULATION_START_AFTER_STOP_[] =
    "sc_start called after sc_stop has been called";
const char SC_ID_STOP_MODE_AFTER_START_[] =
    "attempt to set sc_stop mode  after start will be ignored";
const char SC_ID_SIMULATION_START_AFTER_ERROR_[] =
    "attempt to restart simulation after error";
const char SC_ID_SIMULATION_UNCAUGHT_EXCEPTION_[] = "uncaught exception";
const char SC_ID_PHASE_CALLBACKS_UNSUPPORTED_[] =
    "simulation phase callbacks not enabled";
const char SC_ID_PHASE_CALLBACK_NOT_IMPLEMENTED_[] =
    "empty simulation phase callback called";
const char SC_ID_PHASE_CALLBACK_REGISTER_[] =
    "register simulation phase callback";
const char SC_ID_PHASE_CALLBACK_FORBIDDEN_[] =
    "forbidden action in simulation phase callback";
const char SC_ID_SIMULATION_START_UNEXPECTED_[] =
    "sc_start called unexpectedly";
const char SC_ID_THROW_IT_IGNORED_[] =
    "throw_it on method/non-running process is being ignored ";
const char SC_ID_NOT_EXPECTING_DYNAMIC_EVENT_NOTIFY_[] =
    "dynamic event notification encountered when sensitivity is static";
const char SC_ID_DISABLE_WILL_ORPHAN_PROCESS_[] =
    "disable() or dont_initialize() called on process with no static "
    "sensitivity, it will be orphaned";
const char SC_ID_PROCESS_CONTROL_CORNER_CASE_[] =
    "Undefined process control interaction";
const char SC_ID_METHOD_TERMINATION_EVENT_[] =
    "Attempt to get terminated event for a method process";
const char SC_ID_JOIN_ON_METHOD_HANDLE_[] =
    "Attempt to register method process with sc_join object";
const char SC_ID_NO_PROCESS_SEMANTICS_[] =
    "Attempt to invoke process with no semantics() method";
const char SC_ID_EVENT_ON_NULL_PROCESS_[] =
    "Attempt to get an event for non-existent process";
const char SC_ID_EVENT_LIST_FAILED_[] =
    "invalid use of sc_(and|or)_event list";
const char SC_ID_UNKNOWN_PROCESS_TYPE_[] = "Unknown process type";
const char SC_ID_TIME_CONVERSION_FAILED_[] = "sc_time conversion failed";
const char SC_ID_NEGATIVE_SIMULATION_TIME_[] =
    "negative simulation interval specified in sc_start call";
const char SC_ID_BAD_SC_MODULE_CONSTRUCTOR_[] =
    "sc_module(const char*), sc_module(const std::string&) "
    "have been deprecated, use sc_module(const sc_module_name&)";
const char SC_ID_EMPTY_PROCESS_HANDLE_[] =
    "attempt to use an empty process handle ignored";
const char SC_ID_NO_SC_START_ACTIVITY_[] =
    "no activity or clock movement for sc_start() invocation";
const char SC_ID_KILL_PROCESS_WHILE_UNITIALIZED_[] =
    "a process may not be killed before it is initialized";
const char SC_ID_RESET_PROCESS_WHILE_NOT_RUNNING_[] =
    "a process may not be asynchronously reset while the "
    "simulation is not running";
const char SC_ID_THROW_IT_WHILE_NOT_RUNNING_[] =
    "throw_it not allowed unless simulation is running ";

namespace {

sc_gem5::DefaultReportMessages predfinedMessages{
    {500, SC_ID_NO_BOOL_RETURNED_},
    {501, SC_ID_NO_INT_RETURNED_},
    {502, SC_ID_NO_SC_LOGIC_RETURNED_},
    {503, SC_ID_OPERAND_NOT_SC_LOGIC_},
    {504, SC_ID_OPERAND_NOT_BOOL_},
    {505, SC_ID_INSTANCE_EXISTS_},
    {506, SC_ID_ILLEGAL_CHARACTERS_},
    {507, SC_ID_VC6_PROCESS_HELPER_},
    {508, SC_ID_VC6_MAX_PROCESSES_EXCEEDED_},
    {509, SC_ID_END_MODULE_NOT_CALLED_},
    {510, SC_ID_HIER_NAME_INCORRECT_},
    {511, SC_ID_SET_STACK_SIZE_},
    {512, SC_ID_SC_MODULE_NAME_USE_},
    {513, SC_ID_SC_MODULE_NAME_REQUIRED_},
    {514, SC_ID_SET_TIME_RESOLUTION_},
    {515, SC_ID_SET_DEFAULT_TIME_UNIT_},
    {516, SC_ID_DEFAULT_TIME_UNIT_CHANGED_},
    {517, SC_ID_INCONSISTENT_API_CONFIG_},
    {519, SC_ID_WAIT_NOT_ALLOWED_},
    {520, SC_ID_NEXT_TRIGGER_NOT_ALLOWED_},
    {521, SC_ID_IMMEDIATE_NOTIFICATION_},
    {522, SC_ID_HALT_NOT_ALLOWED_},
    {523, SC_ID_WATCHING_NOT_ALLOWED_},
    {524, SC_ID_DONT_INITIALIZE_},
    {525, SC_ID_WAIT_N_INVALID_},
    {526, SC_ID_MAKE_SENSITIVE_},
    {527, SC_ID_MAKE_SENSITIVE_POS_},
    {528, SC_ID_MAKE_SENSITIVE_NEG_},
    {529, SC_ID_INSERT_MODULE_},
    {530, SC_ID_REMOVE_MODULE_},
    {531, SC_ID_NOTIFY_DELAYED_},
    {532, SC_ID_GEN_UNIQUE_NAME_},
    {533, SC_ID_MODULE_NAME_STACK_EMPTY_},
    {534, SC_ID_NAME_EXISTS_},
    {536, SC_ID_IMMEDIATE_SELF_NOTIFICATION_},
    {537, SC_ID_WAIT_DURING_UNWINDING_},
    {538, SC_ID_CYCLE_MISSES_EVENTS_},
    {539, SC_ID_RETHROW_UNWINDING_},
    {540, SC_ID_PROCESS_ALREADY_UNWINDING_},
    {541, SC_ID_MODULE_METHOD_AFTER_START_},
    {542, SC_ID_MODULE_THREAD_AFTER_START_},
    {543, SC_ID_MODULE_CTHREAD_AFTER_START_},
    {544, SC_ID_SIMULATION_TIME_OVERFLOW_},
    {545, SC_ID_SIMULATION_STOP_CALLED_TWICE_},
    {546, SC_ID_SIMULATION_START_AFTER_STOP_},
    {547, SC_ID_STOP_MODE_AFTER_START_},
    {548, SC_ID_SIMULATION_START_AFTER_ERROR_},
    {549, SC_ID_SIMULATION_UNCAUGHT_EXCEPTION_},
    {550, SC_ID_PHASE_CALLBACKS_UNSUPPORTED_},
    {551, SC_ID_PHASE_CALLBACK_NOT_IMPLEMENTED_},
    {552, SC_ID_PHASE_CALLBACK_REGISTER_},
    {553, SC_ID_PHASE_CALLBACK_FORBIDDEN_},
    {554, SC_ID_SIMULATION_START_UNEXPECTED_},
    {556, SC_ID_THROW_IT_IGNORED_},
    {557, SC_ID_NOT_EXPECTING_DYNAMIC_EVENT_NOTIFY_},
    {558, SC_ID_DISABLE_WILL_ORPHAN_PROCESS_},
    {559, SC_ID_PROCESS_CONTROL_CORNER_CASE_},
    {560, SC_ID_METHOD_TERMINATION_EVENT_},
    {561, SC_ID_JOIN_ON_METHOD_HANDLE_},
    {563, SC_ID_NO_PROCESS_SEMANTICS_},
    {564, SC_ID_EVENT_ON_NULL_PROCESS_},
    {565, SC_ID_EVENT_LIST_FAILED_},
    {566, SC_ID_UNKNOWN_PROCESS_TYPE_},
    {567, SC_ID_TIME_CONVERSION_FAILED_},
    {568, SC_ID_NEGATIVE_SIMULATION_TIME_},
    {569, SC_ID_BAD_SC_MODULE_CONSTRUCTOR_},
    {570, SC_ID_EMPTY_PROCESS_HANDLE_},
    {571, SC_ID_NO_SC_START_ACTIVITY_},
    {572, SC_ID_KILL_PROCESS_WHILE_UNITIALIZED_},
    {573, SC_ID_RESET_PROCESS_WHILE_NOT_RUNNING_},
    {574, SC_ID_THROW_IT_WHILE_NOT_RUNNING_}
};

} // anonymous namespace

} // namespace sc_core

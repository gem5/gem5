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

#ifndef __SYSTEMC_EXT_CORE_MESSAGES_HH__
#define __SYSTEMC_EXT_CORE_MESSAGES_HH__

namespace sc_core
{

extern const char SC_ID_NO_BOOL_RETURNED_[];
extern const char SC_ID_NO_INT_RETURNED_[];
extern const char SC_ID_NO_SC_LOGIC_RETURNED_[];
extern const char SC_ID_OPERAND_NOT_SC_LOGIC_[];
extern const char SC_ID_OPERAND_NOT_BOOL_[];
extern const char SC_ID_INSTANCE_EXISTS_[];
extern const char SC_ID_ILLEGAL_CHARACTERS_[];
extern const char SC_ID_VC6_PROCESS_HELPER_[];
extern const char SC_ID_VC6_MAX_PROCESSES_EXCEEDED_[];
extern const char SC_ID_END_MODULE_NOT_CALLED_[];
extern const char SC_ID_HIER_NAME_INCORRECT_[];
extern const char SC_ID_SET_STACK_SIZE_[];
extern const char SC_ID_SC_MODULE_NAME_USE_[];
extern const char SC_ID_SC_MODULE_NAME_REQUIRED_[];
extern const char SC_ID_SET_TIME_RESOLUTION_[];
extern const char SC_ID_SET_DEFAULT_TIME_UNIT_[];
extern const char SC_ID_DEFAULT_TIME_UNIT_CHANGED_[];
extern const char SC_ID_INCONSISTENT_API_CONFIG_[];
extern const char SC_ID_WAIT_NOT_ALLOWED_[];
extern const char SC_ID_NEXT_TRIGGER_NOT_ALLOWED_[];
extern const char SC_ID_IMMEDIATE_NOTIFICATION_[];
extern const char SC_ID_HALT_NOT_ALLOWED_[];
extern const char SC_ID_WATCHING_NOT_ALLOWED_[];
extern const char SC_ID_DONT_INITIALIZE_[];
extern const char SC_ID_WAIT_N_INVALID_[];
extern const char SC_ID_MAKE_SENSITIVE_[];
extern const char SC_ID_MAKE_SENSITIVE_POS_[];
extern const char SC_ID_MAKE_SENSITIVE_NEG_[];
extern const char SC_ID_INSERT_MODULE_[];
extern const char SC_ID_REMOVE_MODULE_[];
extern const char SC_ID_NOTIFY_DELAYED_[];
extern const char SC_ID_GEN_UNIQUE_NAME_[];
extern const char SC_ID_MODULE_NAME_STACK_EMPTY_[];
extern const char SC_ID_NAME_EXISTS_[];
extern const char SC_ID_IMMEDIATE_SELF_NOTIFICATION_[];
extern const char SC_ID_WAIT_DURING_UNWINDING_[];
extern const char SC_ID_CYCLE_MISSES_EVENTS_[];
extern const char SC_ID_RETHROW_UNWINDING_[];
extern const char SC_ID_PROCESS_ALREADY_UNWINDING_[];
extern const char SC_ID_MODULE_METHOD_AFTER_START_[];
extern const char SC_ID_MODULE_THREAD_AFTER_START_[];
extern const char SC_ID_MODULE_CTHREAD_AFTER_START_[];
extern const char SC_ID_SIMULATION_TIME_OVERFLOW_[];
extern const char SC_ID_SIMULATION_STOP_CALLED_TWICE_[];
extern const char SC_ID_SIMULATION_START_AFTER_STOP_[];
extern const char SC_ID_STOP_MODE_AFTER_START_[];
extern const char SC_ID_SIMULATION_START_AFTER_ERROR_[];
extern const char SC_ID_SIMULATION_UNCAUGHT_EXCEPTION_[];
extern const char SC_ID_PHASE_CALLBACKS_UNSUPPORTED_[];
extern const char SC_ID_PHASE_CALLBACK_NOT_IMPLEMENTED_[];
extern const char SC_ID_PHASE_CALLBACK_REGISTER_[];
extern const char SC_ID_PHASE_CALLBACK_FORBIDDEN_[];
extern const char SC_ID_SIMULATION_START_UNEXPECTED_[];
extern const char SC_ID_THROW_IT_IGNORED_[];
extern const char SC_ID_NOT_EXPECTING_DYNAMIC_EVENT_NOTIFY_[];
extern const char SC_ID_DISABLE_WILL_ORPHAN_PROCESS_[];
extern const char SC_ID_PROCESS_CONTROL_CORNER_CASE_[];
extern const char SC_ID_METHOD_TERMINATION_EVENT_[];
extern const char SC_ID_JOIN_ON_METHOD_HANDLE_[];
extern const char SC_ID_NO_PROCESS_SEMANTICS_[];
extern const char SC_ID_EVENT_ON_NULL_PROCESS_[];
extern const char SC_ID_EVENT_LIST_FAILED_[];
extern const char SC_ID_UNKNOWN_PROCESS_TYPE_[];
extern const char SC_ID_TIME_CONVERSION_FAILED_[];
extern const char SC_ID_NEGATIVE_SIMULATION_TIME_[];
extern const char SC_ID_BAD_SC_MODULE_CONSTRUCTOR_[];
extern const char SC_ID_EMPTY_PROCESS_HANDLE_[];
extern const char SC_ID_NO_SC_START_ACTIVITY_[];
extern const char SC_ID_KILL_PROCESS_WHILE_UNITIALIZED_[];
extern const char SC_ID_RESET_PROCESS_WHILE_NOT_RUNNING_[];
extern const char SC_ID_THROW_IT_WHILE_NOT_RUNNING_[];

} // namespace sc_core

#endif // __SYSTEMC_EXT_CORE_MESSAGES_HH__

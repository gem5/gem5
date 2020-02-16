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

#ifndef __SYSTEMC_EXT_CORE__USING_HH__
#define __SYSTEMC_EXT_CORE__USING_HH__

#include "_core.hh"

using sc_core::sc_attr_base;
using sc_core::sc_attribute;
using sc_core::sc_attr_cltn;

using sc_core::sc_event_finder;
using sc_core::sc_event_finder_t;
using sc_core::sc_event_and_list;
using sc_core::sc_event_or_list;
using sc_core::sc_event_and_expr;
using sc_core::sc_event_or_expr;
using sc_core::sc_event;
using sc_core::sc_get_top_level_events;
using sc_core::sc_find_event;

using sc_core::sc_export_base;
using sc_core::sc_export;

using sc_core::sc_interface;

using sc_core::sc_thread_handle;
using sc_core::sc_join;

using sc_core::sc_argc;
using sc_core::sc_argv;
using sc_core::sc_starvation_policy;
using sc_core::SC_RUN_TO_TIME;
using sc_core::SC_EXIT_ON_STARVATION;
using sc_core::sc_start;
using sc_core::sc_pause;
using sc_core::sc_set_stop_mode;
using sc_core::sc_get_stop_mode;
using sc_core::sc_stop_mode;
using sc_core::SC_STOP_FINISH_DELTA;
using sc_core::SC_STOP_IMMEDIATE;
using sc_core::sc_stop;
using sc_core::sc_time_stamp;
using sc_core::sc_delta_count;
using sc_core::sc_is_running;
using sc_core::sc_pending_activity_at_current_time;
using sc_core::sc_pending_activity_at_future_time;
using sc_core::sc_pending_activity;
using sc_core::sc_time_to_pending_activity;
using sc_core::sc_get_status;
using sc_core::SC_ELABORATION;
using sc_core::SC_BEFORE_END_OF_ELABORATION;
using sc_core::SC_END_OF_ELABORATION;
using sc_core::SC_START_OF_SIMULATION;
using sc_core::SC_RUNNING;
using sc_core::SC_PAUSED;
using sc_core::SC_STOPPED;
using sc_core::SC_END_OF_SIMULATION;
using sc_core::SC_END_OF_INITIALIZATION;
using sc_core::SC_END_OF_UPDATE;
using sc_core::SC_BEFORE_TIMESTEP;
using sc_core::SC_STATUS_ANY;
using sc_core::sc_status;

using sc_core::sc_bind_proxy;
using sc_core::SC_BIND_PROXY_NIL;
using sc_core::sc_module;
using sc_core::next_trigger;
using sc_core::wait;
using sc_core::halt;
using sc_core::sc_gen_unique_name;
using sc_core::sc_hierarchical_name_exists;
using sc_core::sc_behavior;
using sc_core::sc_channel;
using sc_core::sc_start_of_simulation_invoked;
using sc_core::sc_end_of_simulation_invoked;

using sc_core::sc_module_name;

using sc_core::sc_object;
using sc_core::sc_get_top_level_objects;
using sc_core::sc_find_object;

using sc_core::sc_port_policy;
using sc_core::SC_ONE_OR_MORE_BOUND;
using sc_core::SC_ZERO_OR_MORE_BOUND;
using sc_core::SC_ALL_BOUND;
using sc_core::sc_port_base;
using sc_core::sc_port_b;
using sc_core::sc_port;

using sc_core::sc_prim_channel;

using sc_core::sc_curr_proc_kind;
using sc_core::SC_NO_PROC_;
using sc_core::SC_METHOD_PROC_;
using sc_core::SC_THREAD_PROC_;
using sc_core::SC_CTHREAD_PROC_;
using sc_core::sc_descendent_inclusion_info;
using sc_core::SC_NO_DESCENDANTS;
using sc_core::SC_INCLUDE_DESCENDANTS;
using sc_core::sc_unwind_exception;
using sc_core::sc_process_b;
using sc_core::sc_get_curr_process_handle;
using sc_core::sc_get_current_process_b;
using sc_core::sc_curr_proc_info;
using sc_core::sc_curr_proc_handle;
using sc_core::sc_process_handle;
using sc_core::sc_get_current_process_handle;
using sc_core::sc_is_unwinding;

using sc_core::sc_sensitive;

using sc_core::sc_simcontext;
using sc_core::sc_get_curr_simcontext;

using sc_core::sc_spawn_options;
using sc_core::sc_spawn;

using sc_core::sc_time_unit;
using sc_core::SC_FS;
using sc_core::SC_PS;
using sc_core::SC_NS;
using sc_core::SC_US;
using sc_core::SC_MS;
using sc_core::SC_SEC;
using sc_core::sc_time;
using sc_core::sc_time_tuple;
using sc_core::SC_ZERO_TIME;
using sc_core::sc_set_time_resolution;
using sc_core::sc_get_time_resolution;
using sc_core::sc_max_time;
using sc_core::sc_get_default_time_unit;
using sc_core::sc_set_default_time_unit;

using sc_core::SC_ID_NO_BOOL_RETURNED_;
using sc_core::SC_ID_NO_INT_RETURNED_;
using sc_core::SC_ID_NO_SC_LOGIC_RETURNED_;
using sc_core::SC_ID_OPERAND_NOT_SC_LOGIC_;
using sc_core::SC_ID_OPERAND_NOT_BOOL_;
using sc_core::SC_ID_INSTANCE_EXISTS_;
using sc_core::SC_ID_ILLEGAL_CHARACTERS_;
using sc_core::SC_ID_VC6_PROCESS_HELPER_;
using sc_core::SC_ID_VC6_MAX_PROCESSES_EXCEEDED_;
using sc_core::SC_ID_END_MODULE_NOT_CALLED_;
using sc_core::SC_ID_HIER_NAME_INCORRECT_;
using sc_core::SC_ID_SET_STACK_SIZE_;
using sc_core::SC_ID_SC_MODULE_NAME_USE_;
using sc_core::SC_ID_SC_MODULE_NAME_REQUIRED_;
using sc_core::SC_ID_SET_TIME_RESOLUTION_;
using sc_core::SC_ID_SET_DEFAULT_TIME_UNIT_;
using sc_core::SC_ID_DEFAULT_TIME_UNIT_CHANGED_;
using sc_core::SC_ID_INCONSISTENT_API_CONFIG_;
using sc_core::SC_ID_WAIT_NOT_ALLOWED_;
using sc_core::SC_ID_NEXT_TRIGGER_NOT_ALLOWED_;
using sc_core::SC_ID_IMMEDIATE_NOTIFICATION_;
using sc_core::SC_ID_HALT_NOT_ALLOWED_;
using sc_core::SC_ID_WATCHING_NOT_ALLOWED_;
using sc_core::SC_ID_DONT_INITIALIZE_;
using sc_core::SC_ID_WAIT_N_INVALID_;
using sc_core::SC_ID_MAKE_SENSITIVE_;
using sc_core::SC_ID_MAKE_SENSITIVE_POS_;
using sc_core::SC_ID_MAKE_SENSITIVE_NEG_;
using sc_core::SC_ID_INSERT_MODULE_;
using sc_core::SC_ID_REMOVE_MODULE_;
using sc_core::SC_ID_NOTIFY_DELAYED_;
using sc_core::SC_ID_GEN_UNIQUE_NAME_;
using sc_core::SC_ID_MODULE_NAME_STACK_EMPTY_;
using sc_core::SC_ID_NAME_EXISTS_;
using sc_core::SC_ID_IMMEDIATE_SELF_NOTIFICATION_;
using sc_core::SC_ID_WAIT_DURING_UNWINDING_;
using sc_core::SC_ID_CYCLE_MISSES_EVENTS_;
using sc_core::SC_ID_RETHROW_UNWINDING_;
using sc_core::SC_ID_PROCESS_ALREADY_UNWINDING_;
using sc_core::SC_ID_MODULE_METHOD_AFTER_START_;
using sc_core::SC_ID_MODULE_THREAD_AFTER_START_;
using sc_core::SC_ID_MODULE_CTHREAD_AFTER_START_;
using sc_core::SC_ID_SIMULATION_TIME_OVERFLOW_;
using sc_core::SC_ID_SIMULATION_STOP_CALLED_TWICE_;
using sc_core::SC_ID_SIMULATION_START_AFTER_STOP_;
using sc_core::SC_ID_STOP_MODE_AFTER_START_;
using sc_core::SC_ID_SIMULATION_START_AFTER_ERROR_;
using sc_core::SC_ID_SIMULATION_UNCAUGHT_EXCEPTION_;
using sc_core::SC_ID_PHASE_CALLBACKS_UNSUPPORTED_;
using sc_core::SC_ID_PHASE_CALLBACK_NOT_IMPLEMENTED_;
using sc_core::SC_ID_PHASE_CALLBACK_REGISTER_;
using sc_core::SC_ID_PHASE_CALLBACK_FORBIDDEN_;
using sc_core::SC_ID_SIMULATION_START_UNEXPECTED_;
using sc_core::SC_ID_THROW_IT_IGNORED_;
using sc_core::SC_ID_NOT_EXPECTING_DYNAMIC_EVENT_NOTIFY_;
using sc_core::SC_ID_DISABLE_WILL_ORPHAN_PROCESS_;
using sc_core::SC_ID_PROCESS_CONTROL_CORNER_CASE_;
using sc_core::SC_ID_METHOD_TERMINATION_EVENT_;
using sc_core::SC_ID_JOIN_ON_METHOD_HANDLE_;
using sc_core::SC_ID_NO_PROCESS_SEMANTICS_;
using sc_core::SC_ID_EVENT_ON_NULL_PROCESS_;
using sc_core::SC_ID_EVENT_LIST_FAILED_;
using sc_core::SC_ID_UNKNOWN_PROCESS_TYPE_;
using sc_core::SC_ID_TIME_CONVERSION_FAILED_;
using sc_core::SC_ID_NEGATIVE_SIMULATION_TIME_;
using sc_core::SC_ID_BAD_SC_MODULE_CONSTRUCTOR_;
using sc_core::SC_ID_EMPTY_PROCESS_HANDLE_;
using sc_core::SC_ID_NO_SC_START_ACTIVITY_;
using sc_core::SC_ID_KILL_PROCESS_WHILE_UNITIALIZED_;
using sc_core::SC_ID_RESET_PROCESS_WHILE_NOT_RUNNING_;
using sc_core::SC_ID_THROW_IT_WHILE_NOT_RUNNING_;

#endif  //__SYSTEMC_EXT_CORE__USING_HH__

/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_kernel_ids.h -- Report ids for the kernel code.

  Original Author: Martin Janssen, Synopsys, Inc., 2002-01-17

 CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_KERNEL_IDS_H
#define SC_KERNEL_IDS_H


#include "sysc/utils/sc_report.h"


// ----------------------------------------------------------------------------
//  Report ids (kernel)
//
//  Report ids in the range of 500-599.
// ----------------------------------------------------------------------------

#ifndef SC_DEFINE_MESSAGE
#define SC_DEFINE_MESSAGE(id,unused1,unused2) \
    namespace sc_core { extern const char id[]; }
namespace sc_core {
    extern const char SC_ID_REGISTER_ID_FAILED_[]; // in sc_report_handler.cpp
}
#endif

SC_DEFINE_MESSAGE(SC_ID_NO_BOOL_RETURNED_            , 500, 
	"operator does not return boolean")
SC_DEFINE_MESSAGE(SC_ID_NO_INT_RETURNED_             , 501,
	"operator does not return int")
SC_DEFINE_MESSAGE(SC_ID_NO_SC_LOGIC_RETURNED_        , 502,
	"operator does not return sc_logic")
SC_DEFINE_MESSAGE(SC_ID_OPERAND_NOT_SC_LOGIC_        , 503,
	"operand is not sc_logic")
SC_DEFINE_MESSAGE(SC_ID_OPERAND_NOT_BOOL_            , 504,
	"operand is not bool")
SC_DEFINE_MESSAGE(SC_ID_INSTANCE_EXISTS_             , 505,
	"object already exists")       
SC_DEFINE_MESSAGE(SC_ID_ILLEGAL_CHARACTERS_          , 506,
	"illegal characters" )
SC_DEFINE_MESSAGE(SC_ID_VC6_PROCESS_HELPER_          , 507,
	"internal error: sc_vc6_process_helper" )
SC_DEFINE_MESSAGE(SC_ID_VC6_MAX_PROCESSES_EXCEEDED_  , 508,
	"maximum number of processes per module exceeded (VC6)" )
SC_DEFINE_MESSAGE(SC_ID_END_MODULE_NOT_CALLED_       , 509,
	"module construction not properly completed: did "
	"you forget to add a sc_module_name parameter to "
	"your module constructor?" )
SC_DEFINE_MESSAGE(SC_ID_HIER_NAME_INCORRECT_         , 510,
	"hierarchical name as shown may be incorrect due to previous errors" )
SC_DEFINE_MESSAGE(SC_ID_SET_STACK_SIZE_              , 511,
	"set_stack_size() is only allowed for SC_THREADs and SC_CTHREADs" )
SC_DEFINE_MESSAGE(SC_ID_SC_MODULE_NAME_USE_          , 512,
	"incorrect use of sc_module_name" )
SC_DEFINE_MESSAGE(SC_ID_SC_MODULE_NAME_REQUIRED_     , 513,
	"an sc_module_name parameter for your constructor is required" )
SC_DEFINE_MESSAGE(SC_ID_SET_TIME_RESOLUTION_         , 514,
	"set time resolution failed" )
SC_DEFINE_MESSAGE(SC_ID_SET_DEFAULT_TIME_UNIT_       , 515,
	"set default time unit failed" )
SC_DEFINE_MESSAGE(SC_ID_DEFAULT_TIME_UNIT_CHANGED_   , 516,
	"default time unit changed to time resolution" )
SC_DEFINE_MESSAGE(SC_ID_INCONSISTENT_API_CONFIG_     , 517,
	"inconsistent library configuration detected" )
// available message number 518
SC_DEFINE_MESSAGE(SC_ID_WAIT_NOT_ALLOWED_            , 519,
	"wait() is only allowed in SC_THREADs and SC_CTHREADs" )
SC_DEFINE_MESSAGE(SC_ID_NEXT_TRIGGER_NOT_ALLOWED_    , 520,
	"next_trigger() is only allowed in SC_METHODs" )
SC_DEFINE_MESSAGE(SC_ID_IMMEDIATE_NOTIFICATION_      , 521,
	"immediate notification is not allowed during the update phase" )
SC_DEFINE_MESSAGE(SC_ID_HALT_NOT_ALLOWED_            , 522,
	"halt() is only allowed in SC_CTHREADs" )
SC_DEFINE_MESSAGE(SC_ID_WATCHING_NOT_ALLOWED_        , 523,
	"watching() has been deprecated, use reset_signal_is()" )
SC_DEFINE_MESSAGE(SC_ID_DONT_INITIALIZE_             , 524,
	"dont_initialize() has no effect for SC_CTHREADs" )
SC_DEFINE_MESSAGE(SC_ID_WAIT_N_INVALID_              , 525,
	"wait(n) is only valid for n > 0" )
SC_DEFINE_MESSAGE(SC_ID_MAKE_SENSITIVE_              , 526,
	"make sensitive failed" )
SC_DEFINE_MESSAGE(SC_ID_MAKE_SENSITIVE_POS_          , 527,
	"make sensitive pos failed" )
SC_DEFINE_MESSAGE(SC_ID_MAKE_SENSITIVE_NEG_          , 528,
	"make sensitive neg failed" )
SC_DEFINE_MESSAGE(SC_ID_INSERT_MODULE_               , 529,
	"insert module failed" )
SC_DEFINE_MESSAGE(SC_ID_REMOVE_MODULE_               , 530,
	"remove module failed" )
SC_DEFINE_MESSAGE(SC_ID_NOTIFY_DELAYED_              , 531,
	"notify_delayed() cannot be called on events "
	"that have pending notifications" )
SC_DEFINE_MESSAGE(SC_ID_GEN_UNIQUE_NAME_             , 532,
	"cannot generate unique name from null string" )
SC_DEFINE_MESSAGE(SC_ID_MODULE_NAME_STACK_EMPTY_     , 533,
	"module name stack is empty: did you forget to "
	"add a sc_module_name parameter to your module "
	"constructor?" )
// available message number 534
// available message number 535
SC_DEFINE_MESSAGE( SC_ID_IMMEDIATE_SELF_NOTIFICATION_, 536,
         "immediate self-notification ignored as of IEEE 1666-2011" )
SC_DEFINE_MESSAGE( SC_ID_WAIT_DURING_UNWINDING_      , 537,
         "wait() not allowed during unwinding" )
SC_DEFINE_MESSAGE(SC_ID_CYCLE_MISSES_EVENTS_         , 538,
         "the simulation contains timed-events but they are "
         "ignored by sc_cycle() ==> the simulation will be "
	 "incorrect" )
SC_DEFINE_MESSAGE( SC_ID_RETHROW_UNWINDING_          , 539,
         "sc_unwind_exception not re-thrown during kill/reset" )
SC_DEFINE_MESSAGE( SC_ID_PROCESS_ALREADY_UNWINDING_  , 540,
         "kill/reset ignored during unwinding" )
SC_DEFINE_MESSAGE(SC_ID_MODULE_METHOD_AFTER_START_   , 541,
	"call to SC_METHOD in sc_module while simulation running" )
SC_DEFINE_MESSAGE(SC_ID_MODULE_THREAD_AFTER_START_   , 542,
	"call to SC_THREAD in sc_module while simulation running" )
SC_DEFINE_MESSAGE(SC_ID_MODULE_CTHREAD_AFTER_START_   , 543,
	"call to SC_CTHREAD in sc_module while simulation running" )
SC_DEFINE_MESSAGE(SC_ID_SIMULATION_TIME_OVERFLOW_   , 544,
	"simulation time value overflow, simulation aborted" )
SC_DEFINE_MESSAGE(SC_ID_SIMULATION_STOP_CALLED_TWICE_ , 545,
	"sc_stop has already been called" ) 
SC_DEFINE_MESSAGE(SC_ID_SIMULATION_START_AFTER_STOP_  , 546,
	"sc_start called after sc_stop has been called" ) 
SC_DEFINE_MESSAGE(SC_ID_STOP_MODE_AFTER_START_        , 547,
	"attempt to set sc_stop mode  after start will be ignored" ) 
SC_DEFINE_MESSAGE( SC_ID_SIMULATION_START_AFTER_ERROR_, 548,
       "attempt to restart simulation after error" )
SC_DEFINE_MESSAGE( SC_ID_SIMULATION_UNCAUGHT_EXCEPTION_, 549,
       "uncaught exception" )
SC_DEFINE_MESSAGE(SC_ID_PHASE_CALLBACKS_UNSUPPORTED_   , 550,
       "simulation phase callbacks not enabled")
SC_DEFINE_MESSAGE(SC_ID_PHASE_CALLBACK_NOT_IMPLEMENTED_, 551,
       "empty simulation phase callback called" )
SC_DEFINE_MESSAGE(SC_ID_PHASE_CALLBACK_REGISTER_,        552,
       "register simulation phase callback" )
SC_DEFINE_MESSAGE(SC_ID_PHASE_CALLBACK_FORBIDDEN_,       553,
       "forbidden action in simulation phase callback" )
// available message number 554
// available message number 555
SC_DEFINE_MESSAGE(SC_ID_THROW_IT_IGNORED_  , 556,
        "throw_it on method/non-running process is being ignored " )
SC_DEFINE_MESSAGE(SC_ID_NOT_EXPECTING_DYNAMIC_EVENT_NOTIFY_ , 557,
	"dynamic event notification encountered when sensitivity is static" )
SC_DEFINE_MESSAGE(SC_ID_DISABLE_WILL_ORPHAN_PROCESS_     , 558,
       "disable() or dont_initialize() called on process with no static sensitivity, it will be orphaned" )
SC_DEFINE_MESSAGE(SC_ID_PROCESS_CONTROL_CORNER_CASE_     , 559,
	"Undefined process control interaction" ) 
SC_DEFINE_MESSAGE(SC_ID_METHOD_TERMINATION_EVENT_        , 560,
	"Attempt to get terminated event for a method process" ) 
SC_DEFINE_MESSAGE(SC_ID_JOIN_ON_METHOD_HANDLE_        , 561,
	"Attempt to register method process with sc_join object" ) 
SC_DEFINE_MESSAGE(SC_ID_NO_PROCESS_SEMANTICS_         , 563,
	"Attempt to invoke process with no semantics() method" )
SC_DEFINE_MESSAGE(SC_ID_EVENT_ON_NULL_PROCESS_         , 564,
	"Attempt to get an event for non-existent process" )
// available message number 565
SC_DEFINE_MESSAGE(SC_ID_UNKNOWN_PROCESS_TYPE_,       566,
	"Unknown process type" )
// available message number 567
SC_DEFINE_MESSAGE(SC_ID_NEGATIVE_SIMULATION_TIME_, 568,
        "negative simulation interval specified in sc_start call" )
SC_DEFINE_MESSAGE(SC_ID_BAD_SC_MODULE_CONSTRUCTOR_  , 569,
        "sc_module(const char*), sc_module(const std::string&) "
        "have been deprecated, use sc_module(const sc_module_name&)" )
SC_DEFINE_MESSAGE(SC_ID_EMPTY_PROCESS_HANDLE_  , 570,
        "attempt to use an empty process handle ignored" )
SC_DEFINE_MESSAGE(SC_ID_NO_SC_START_ACTIVITY_  , 571,
        "no activity or clock movement for sc_start() invocation" )
SC_DEFINE_MESSAGE(SC_ID_KILL_PROCESS_WHILE_UNITIALIZED_  , 572,
        "a process may not be killed before it is initialized" )
SC_DEFINE_MESSAGE(SC_ID_RESET_PROCESS_WHILE_NOT_RUNNING_  , 573,
        "a process may not be asynchronously reset while the simulation is not running" )
SC_DEFINE_MESSAGE(SC_ID_THROW_IT_WHILE_NOT_RUNNING_  , 574,
        "throw_it not allowed unless simulation is running " )


/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:
    
 *****************************************************************************/

// $Log: sc_kernel_ids.h,v $
// Revision 1.25  2011/08/26 22:06:34  acg
//  Torsten Maehne: formating fix.
//
// Revision 1.24  2011/08/07 19:08:04  acg
//  Andy Goodrich: moved logs to end of file so line number synching works
//  better between versions.
//
// Revision 1.23  2011/07/24 11:15:47  acg
//  Philipp A. Hartmann: Improvements to error/warning messages related to
//  process control.
//
// Revision 1.22  2011/05/09 04:07:48  acg
//  Philipp A. Hartmann:
//    (1) Restore hierarchy in all phase callbacks.
//    (2) Ensure calls to before_end_of_elaboration.
//
// Revision 1.21  2011/04/19 19:15:41  acg
//  Andy Goodrich: fix so warning message is always issued for a throw_it()
//  on a method process.
//
// Revision 1.20  2011/04/19 15:04:27  acg
//  Philipp A. Hartmann: clean up SC_ID messages.
//
// Revision 1.19  2011/04/19 02:39:09  acg
//  Philipp A. Hartmann: added checks for additional throws during stack unwinds.
//
// Revision 1.18  2011/04/05 06:23:45  acg
//  Andy Goodrich: comments for throws while the simulator is not running.
//
// Revision 1.17  2011/04/01 22:30:39  acg
//  Andy Goodrich: change hard assertion to warning for trigger_dynamic()
//  getting called when there is only STATIC sensitivity. This can result
//  because of sc_process_handle::throw_it().
//
// Revision 1.16  2011/03/28 13:02:51  acg
//  Andy Goodrich: Changes for disable() interactions.
//
// Revision 1.15  2011/03/07 17:34:21  acg
//  Andy Goodrich: changed process control corner case message. Added more
//  place holders for unused message numbers.
//
// Revision 1.14  2011/03/06 19:57:11  acg
//  Andy Goodrich: refinements for the illegal suspend - synchronous reset
//  interaction.
//
// Revision 1.13  2011/03/06 15:56:29  acg
//  Andy Goodrich: added process control corner case error message, remove
//  unused messages.
//
// Revision 1.12  2011/03/05 19:44:20  acg
//  Andy Goodrich: changes for object and event naming and structures.
//
// Revision 1.11  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.10  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.9  2011/02/13 21:29:16  acg
//  Andy Goodrich: added error messages for throws that occur before
//  simulator intialization.
//
// Revision 1.8  2011/02/11 13:25:24  acg
//  Andy Goodrich: Philipp A. Hartmann's changes:
//    (1) Removal of SC_CTHREAD method overloads.
//    (2) New exception processing code.
//
// Revision 1.7  2011/02/07 19:17:20  acg
//  Andy Goodrich: changes for IEEE 1666 compatibility.
//
// Revision 1.6  2011/01/19 23:21:50  acg
//  Andy Goodrich: changes for IEEE 1666 2011
//
// Revision 1.5  2010/07/30 05:21:22  acg
//  Andy Goodrich: release 2.3 fixes.
//
// Revision 1.4  2009/02/28 00:26:58  acg
//  Andy Goodrich: changed boost name space to sc_boost to allow use with
//  full boost library applications.
//
// Revision 1.3  2008/11/17 15:57:15  acg
//  Andy Goodrich: added deprecation message for sc_module(const char*)
//
// Revision 1.2  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.7  2006/08/29 23:37:13  acg
//  Andy Goodrich: Added check for negative time.
//
// Revision 1.6  2006/04/20 17:08:16  acg
//  Andy Goodrich: 3.0 style process changes.
//
// Revision 1.5  2006/01/25 00:31:19  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.4  2006/01/24 20:49:04  acg
// Andy Goodrich: changes to remove the use of deprecated features within the
// simulator, and to issue warning messages when deprecated features are used.
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.
//

#endif

// Taf!

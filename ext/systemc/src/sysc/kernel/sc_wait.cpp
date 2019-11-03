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

  sc_wait.cpp -- Wait() and related functions.

  Original Author: Stan Y. Liao, Synopsys, Inc.
                   Martin Janssen, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#include "sysc/kernel/sc_except.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_cthread_process.h"
#include "sysc/kernel/sc_thread_process.h"
#include "sysc/kernel/sc_simcontext_int.h"
#include "sysc/kernel/sc_wait.h"
#include "sysc/utils/sc_utils_ids.h"

namespace sc_core {

// static sensitivity for SC_THREADs and SC_CTHREADs

void warn_cthread_wait()
{
    static bool warn_wait = true;
    if ( warn_wait )
    {
        warn_wait = false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "all waits except wait() and wait(N)\n" \
	    "             are deprecated for SC_CTHREAD, " \
	    "use an SC_THREAD instead");
    }
}

void
wait( sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    switch( cpi->kind ) {
    case SC_THREAD_PROC_: 
    case SC_CTHREAD_PROC_: {
	RCAST<sc_cthread_handle>( cpi->process_handle )->wait_cycles();
        break;
    }
    default:
	SC_REPORT_ERROR( SC_ID_WAIT_NOT_ALLOWED_, "\n        "
			 "in SC_METHODs use next_trigger() instead" );
        break;
    }
}

// dynamic sensitivity for SC_THREADs and SC_CTHREADs

void
wait( const sc_event& e, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    switch( cpi->kind ) {
    case SC_THREAD_PROC_: {
	RCAST<sc_thread_handle>( cpi->process_handle )->wait( e );
	break;
    }
    case SC_CTHREAD_PROC_: {
        warn_cthread_wait();
	sc_cthread_handle cthread_h =
            RCAST<sc_cthread_handle>( cpi->process_handle );
	cthread_h->wait( e );
	cthread_h->wait_cycles();
	break;
    }
    default:
	SC_REPORT_ERROR( SC_ID_WAIT_NOT_ALLOWED_, "\n        "
			 "in SC_METHODs use next_trigger() instead" );
        break;
    }
}

void
wait( const sc_event_or_list& el, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    switch( cpi->kind ) {
    case SC_THREAD_PROC_: {
	RCAST<sc_thread_handle>( cpi->process_handle )->wait( el );
	break;
    }
    case SC_CTHREAD_PROC_: {
        warn_cthread_wait();
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "wait(event_list) is deprecated for SC_CTHREAD, use SC_THREAD");
	sc_cthread_handle cthread_h =
            RCAST<sc_cthread_handle>( cpi->process_handle );
	cthread_h->wait( el );
	cthread_h->wait_cycles();
	break;
    }
    default:
	SC_REPORT_ERROR( SC_ID_WAIT_NOT_ALLOWED_, "\n        "
			 "in SC_METHODs use next_trigger() instead" );
        break;
    }
}

void
wait( const sc_event_and_list& el, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    switch( cpi->kind ) {
    case SC_THREAD_PROC_: {
	RCAST<sc_thread_handle>( cpi->process_handle )->wait( el );
	break;
    }
    case SC_CTHREAD_PROC_: {
        warn_cthread_wait();
	sc_cthread_handle cthread_h =
            RCAST<sc_cthread_handle>( cpi->process_handle );
	cthread_h->wait( el );
	cthread_h->wait_cycles();
	break;
    }
    default:
	SC_REPORT_ERROR( SC_ID_WAIT_NOT_ALLOWED_, "\n        "
			 "in SC_METHODs use next_trigger() instead" );
        break;
    }
}

void
wait( const sc_time& t, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    switch( cpi->kind ) {
    case SC_THREAD_PROC_: {
	RCAST<sc_thread_handle>( cpi->process_handle )->wait( t );
	break;
    }
    case SC_CTHREAD_PROC_: {
        warn_cthread_wait();
	sc_cthread_handle cthread_h =
            RCAST<sc_cthread_handle>( cpi->process_handle );
	cthread_h->wait( t );
	cthread_h->wait_cycles();
	break;
    }
    default:
	SC_REPORT_ERROR( SC_ID_WAIT_NOT_ALLOWED_, "\n        "
			 "in SC_METHODs use next_trigger() instead" );
        break;
    }
}

void
wait( const sc_time& t, const sc_event& e, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    switch( cpi->kind ) {
    case SC_THREAD_PROC_: {
	RCAST<sc_thread_handle>( cpi->process_handle )->wait( t, e );
	break;
    }
    case SC_CTHREAD_PROC_: {
        warn_cthread_wait();
	sc_cthread_handle cthread_h =
            RCAST<sc_cthread_handle>( cpi->process_handle );
	cthread_h->wait( t, e );
	cthread_h->wait_cycles();
	break;
    }
    default:
	SC_REPORT_ERROR( SC_ID_WAIT_NOT_ALLOWED_, "\n        "
			 "in SC_METHODs use next_trigger() instead" );
        break;
    }
}

void
wait( const sc_time& t, const sc_event_or_list& el, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    switch( cpi->kind ) {
    case SC_THREAD_PROC_: {
	RCAST<sc_thread_handle>( cpi->process_handle )->wait( t, el );
	break;
    }
    case SC_CTHREAD_PROC_: {
        warn_cthread_wait();
	sc_cthread_handle cthread_h =
            RCAST<sc_cthread_handle>( cpi->process_handle );
	cthread_h->wait( t, el );
	cthread_h->wait_cycles();
	break;
    }
    default:
	SC_REPORT_ERROR( SC_ID_WAIT_NOT_ALLOWED_, "\n        "
			 "in SC_METHODs use next_trigger() instead" );
        break;
    }
}

void
wait( const sc_time& t, const sc_event_and_list& el, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    switch( cpi->kind ) {
    case SC_THREAD_PROC_: {
	RCAST<sc_thread_handle>( cpi->process_handle )->wait( t, el );
	break;
    }
    case SC_CTHREAD_PROC_: {
        warn_cthread_wait();
	sc_cthread_handle cthread_h =
            RCAST<sc_cthread_handle>( cpi->process_handle );
	cthread_h->wait( t, el );
	cthread_h->wait_cycles();
	break;
    }
    default:
	SC_REPORT_ERROR( SC_ID_WAIT_NOT_ALLOWED_, "\n        "
			 "in SC_METHODs use next_trigger() instead" );
        break;
    }
}


// static sensitivity for SC_METHODs

void
next_trigger( sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    if( cpi->kind == SC_METHOD_PROC_ ) {
	RCAST<sc_method_handle>( cpi->process_handle )->clear_trigger();
    } else {
	SC_REPORT_ERROR( SC_ID_NEXT_TRIGGER_NOT_ALLOWED_, "\n        "
			 "in SC_THREADs and SC_CTHREADs use wait() instead" );
    }
}


// dynamic sensitivity for SC_METHODs

void
next_trigger( const sc_event& e, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    if( cpi->kind == SC_METHOD_PROC_ ) {
	RCAST<sc_method_handle>( cpi->process_handle )->next_trigger( e );
    } else {
	SC_REPORT_ERROR( SC_ID_NEXT_TRIGGER_NOT_ALLOWED_, "\n        "
			 "in SC_THREADs and SC_CTHREADs use wait() instead" );
    }
}

void
next_trigger( const sc_event_or_list& el, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    if( cpi->kind == SC_METHOD_PROC_ ) {
	RCAST<sc_method_handle>( cpi->process_handle )->next_trigger( el );
    } else {
	SC_REPORT_ERROR( SC_ID_NEXT_TRIGGER_NOT_ALLOWED_, "\n        "
			 "in SC_THREADs and SC_CTHREADs use wait() instead" );
    }
}

void
next_trigger( const sc_event_and_list& el, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    if( cpi->kind == SC_METHOD_PROC_ ) {
	RCAST<sc_method_handle>( cpi->process_handle )->next_trigger( el );
    } else {
	SC_REPORT_ERROR( SC_ID_NEXT_TRIGGER_NOT_ALLOWED_, "\n        "
			 "in SC_THREADs and SC_CTHREADs use wait() instead" );
    }
}

void
next_trigger( const sc_time& t, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    if( cpi->kind == SC_METHOD_PROC_ ) {
	RCAST<sc_method_handle>( cpi->process_handle )->next_trigger( t );
    } else {
	SC_REPORT_ERROR( SC_ID_NEXT_TRIGGER_NOT_ALLOWED_, "\n        "
			 "in SC_THREADs and SC_CTHREADs use wait() instead" );
    }
}

void
next_trigger( const sc_time& t, const sc_event& e, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    if( cpi->kind == SC_METHOD_PROC_ ) {
	RCAST<sc_method_handle>( cpi->process_handle )->next_trigger( t, e );
    } else {
	SC_REPORT_ERROR( SC_ID_NEXT_TRIGGER_NOT_ALLOWED_, "\n        "
			 "in SC_THREADs and SC_CTHREADs use wait() instead" );
    }
}

void
next_trigger( const sc_time& t, const sc_event_or_list& el, sc_simcontext* simc)
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    if( cpi->kind == SC_METHOD_PROC_ ) {
	RCAST<sc_method_handle>( cpi->process_handle )->next_trigger( t, el );
    } else {
	SC_REPORT_ERROR( SC_ID_NEXT_TRIGGER_NOT_ALLOWED_, "\n        "
			 "in SC_THREADs and SC_CTHREADs use wait() instead" );
    }
}

void
next_trigger(const sc_time& t, const sc_event_and_list& el, sc_simcontext* simc)
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    if( cpi->kind == SC_METHOD_PROC_ ) {
	RCAST<sc_method_handle>( cpi->process_handle )->next_trigger( t, el );
    } else {
	SC_REPORT_ERROR( SC_ID_NEXT_TRIGGER_NOT_ALLOWED_, "\n        "
			 "in SC_THREADs and SC_CTHREADs use wait() instead" );
    }
}


// for SC_METHODs and SC_THREADs and SC_CTHREADs

bool
timed_out( sc_simcontext* simc )
{
    static bool warn_timed_out=true; 
    if ( warn_timed_out )
    {
        warn_timed_out = false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "timed_out() function is deprecated" );
    }

    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    return cpi->process_handle->timed_out();
}



// misc.

void
sc_set_location( const char* file, int lineno, sc_simcontext* simc )
{
    sc_curr_proc_handle cpi = simc->get_curr_proc_info();
    sc_process_b* handle = cpi->process_handle;
    handle->file = file;
    handle->lineno = lineno;
}

} // namespace sc_core

/* 
$Log: sc_wait.cpp,v $
Revision 1.6  2011/08/26 20:46:11  acg
 Andy Goodrich: moved the modification log to the end of the file to
 eliminate source line number skew when check-ins are done.

Revision 1.5  2011/02/18 20:27:14  acg
 Andy Goodrich: Updated Copyrights.

Revision 1.4  2011/02/13 21:47:38  acg
 Andy Goodrich: update copyright notice.

Revision 1.3  2011/01/18 20:10:45  acg
 Andy Goodrich: changes for IEEE1666_2011 semantics.

Revision 1.2  2008/05/22 17:06:27  acg
 Andy Goodrich: updated copyright notice to include 2008.

Revision 1.1.1.1  2006/12/15 20:20:05  acg
SystemC 2.3

Revision 1.7  2006/02/02 20:20:39  acg
 Andy Goodrich: warnings for SC_THREAD waits.

Revision 1.6  2006/02/01 01:36:54  acg
 Andy Goodrich: addition of deprecation comments for SC_CTHREAD waits other
 than wait() and wait(N).

Revision 1.5  2006/01/31 22:17:40  acg
 Andy Goodrich: added deprecation warnings for SC_CTHREAD waits other than
 wait() and wait(N).

Revision 1.4  2006/01/25 00:31:20  acg
 Andy Goodrich: Changed over to use a standard message id of
 SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.

Revision 1.3  2006/01/24 20:49:05  acg
Andy Goodrich: changes to remove the use of deprecated features within the
simulator, and to issue warning messages when deprecated features are used.

Revision 1.2  2006/01/03 23:18:45  acg
Changed copyright to include 2006.

Revision 1.1.1.1  2005/12/19 23:16:44  acg
First check in of SystemC 2.1 into its own archive.

Revision 1.10  2005/09/02 19:03:30  acg
Changes for dynamic processes. Removal of lambda support.

Revision 1.9  2005/07/30 03:45:05  acg
Changes from 2.1, including changes for sc_process_handle.

Revision 1.8  2005/04/04 00:16:07  acg
Changes for directory name change to sys from systemc.
Changes for sc_string going to std::string.
Changes for sc_pvector going to std::vector.
Changes for reference pools for bit and part selections.
Changes for const sc_concatref support.

Revision 1.5  2004/09/27 20:49:10  acg
Andy Goodrich, Forte Design Systems, Inc.
   - Added a $Log comment so that CVS checkin comments appear in the
     checkout source.

*/

// Taf!

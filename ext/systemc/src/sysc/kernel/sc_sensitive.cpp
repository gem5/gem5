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

  sc_sensitive.cpp --

  Original Author: Stan Y. Liao, Synopsys, Inc.
                   Martin Janssen, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#include "sysc/kernel/sc_event.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_module.h"
#include "sysc/kernel/sc_cthread_process.h"
#include "sysc/kernel/sc_method_process.h"
#include "sysc/kernel/sc_thread_process.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/kernel/sc_sensitive.h"
#include "sysc/communication/sc_signal_ports.h"
#include "sysc/utils/sc_utils_ids.h"

namespace sc_core {

// support functions

static
sc_method_handle
as_method_handle( sc_process_b* handle_ )
{
    return DCAST<sc_method_handle>( handle_ );
}

static
sc_thread_handle
as_thread_handle( sc_process_b* handle_ )
{
    return DCAST<sc_thread_handle>( handle_ );
}

static 
void
warn_no_parens()
{
    static bool warn_no_parentheses=true;
    if ( warn_no_parentheses )
    {
	warn_no_parentheses=false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "use of () to specify sensitivity is deprecated, use << instead" );
    }
}

// ----------------------------------------------------------------------------
//  CLASS : sc_sensitive
//
//  Static sensitivity class for events.
// ----------------------------------------------------------------------------

// constructor

sc_sensitive::sc_sensitive( sc_module* module_ )
: m_module( module_ ),
  m_mode( SC_NONE_ ),
  m_handle( 0 )
{}


// destructor

sc_sensitive::~sc_sensitive()
{}


// changing between process handles

sc_sensitive&
sc_sensitive::operator << ( sc_process_handle handle_ )
{
    switch ( handle_.proc_kind() )
	{
      case SC_CTHREAD_PROC_:
      case SC_THREAD_PROC_:
        m_mode = SC_THREAD_;
	break;
      case SC_METHOD_PROC_:
	m_mode = SC_METHOD_;
	break;
      default:
	assert(0);
    }
    m_handle = (sc_process_b*)handle_;
    return *this;
}

sc_sensitive&
sc_sensitive::operator << ( const sc_event& event_ )
{
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_:
    case SC_THREAD_: {
	m_handle->add_static_event( event_ );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

void
sc_sensitive::make_static_sensitivity(
    sc_process_b* handle_, const sc_event& event_)
{
    handle_->add_static_event( event_ );
}


sc_sensitive&
sc_sensitive::operator << ( const sc_interface& interface_ )
{
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_:
    case SC_THREAD_: {
	m_handle->add_static_event( interface_.default_event() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

void
sc_sensitive::make_static_sensitivity(
    sc_process_b* handle_, const sc_interface& interface_)
{
    handle_->add_static_event( interface_.default_event() );
}

sc_sensitive&
sc_sensitive::operator << ( const sc_port_base& port_ )
{
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ) );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ) );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

void
sc_sensitive::make_static_sensitivity(
    sc_process_b* handle_, const sc_port_base& port_)
{
    sc_method_handle handle_m = as_method_handle( handle_ );
    if ( handle_m ) {
	port_.make_sensitive( handle_m );
	return;
    }
    sc_thread_handle handle_t = as_thread_handle( handle_ );
    // assert(handle_t);
    port_.make_sensitive( handle_t );
}

sc_sensitive&
sc_sensitive::operator << ( sc_event_finder& event_finder_ )
{
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	event_finder_.port().make_sensitive( as_method_handle( m_handle ),
					     &event_finder_ );
	break;
    }
    case SC_THREAD_: {
	event_finder_.port().make_sensitive( as_thread_handle( m_handle ),
					     &event_finder_ );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

void 
sc_sensitive::make_static_sensitivity(
    sc_process_b* handle_, sc_event_finder& event_finder_)
{
    if (sc_is_running()) {
      handle_->add_static_event( event_finder_.find_event() );
    } else {
	sc_method_handle handle_m = as_method_handle( handle_ );
	if ( handle_m ) {
	    event_finder_.port().make_sensitive( handle_m, &event_finder_ );
	    return;
        }
	sc_thread_handle handle_t = as_thread_handle( handle_ );
	// assert(handle_t);
	event_finder_.port().make_sensitive( handle_t, &event_finder_);
    }
}


sc_sensitive&
sc_sensitive::operator () ( const sc_event& event_ )
{
    warn_no_parens();
    return operator << ( event_ );
}

sc_sensitive&
sc_sensitive::operator () ( const sc_interface& interface_ )
{
    warn_no_parens();
    return operator << ( interface_ );
}

sc_sensitive&
sc_sensitive::operator () ( const sc_port_base& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

sc_sensitive&
sc_sensitive::operator () ( sc_event_finder& event_finder_ )
{
    warn_no_parens();
    return operator << ( event_finder_ );
}


sc_sensitive&
sc_sensitive::operator () ( sc_cthread_handle handle_,
			    sc_event_finder& event_finder_ )
{
    event_finder_.port().make_sensitive( handle_, &event_finder_ );
    return *this;
}

sc_sensitive&
sc_sensitive::operator () ( sc_cthread_handle handle_,
			    const in_if_b_type& interface_ )
{
    handle_->add_static_event( interface_.posedge_event() );
    return *this;
}

sc_sensitive&
sc_sensitive::operator () ( sc_cthread_handle handle_,
			    const in_if_l_type& interface_ )
{
    handle_->add_static_event( interface_.posedge_event() );
    return *this;
}

sc_sensitive&
sc_sensitive::operator () ( sc_cthread_handle handle_,
			    const in_port_b_type& port_ )
{
    port_.make_sensitive( handle_, &port_.pos() );
    return *this;
}

sc_sensitive&
sc_sensitive::operator () ( sc_cthread_handle handle_,
			    const in_port_l_type& port_ )
{
    port_.make_sensitive( handle_, &port_.pos() );
    return *this;
}

sc_sensitive&
sc_sensitive::operator () ( sc_cthread_handle handle_,
			    const inout_port_b_type& port_ )
{
    port_.make_sensitive( handle_, &port_.pos() );
    return *this;
}

sc_sensitive&
sc_sensitive::operator () ( sc_cthread_handle handle_,
			    const inout_port_l_type& port_ )
{
    port_.make_sensitive( handle_, &port_.pos() );
    return *this;
}

void sc_sensitive::reset()
{
    m_mode = SC_NONE_;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_sensitive_pos
//
//  Static sensitivity class for positive edge events.
// ----------------------------------------------------------------------------

static void sc_deprecated_sensitive_pos()
{
    static bool warn_sensitive_pos=true;
    if ( warn_sensitive_pos )
    {
        warn_sensitive_pos=false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	 "sc_sensitive_pos is deprecated use sc_sensitive << with pos() instead" );
    }
}

// constructor

sc_sensitive_pos::sc_sensitive_pos( sc_module* module_ )
: m_module( module_ ),
  m_mode( SC_NONE_ ),
  m_handle( 0 )
{}


// destructor

sc_sensitive_pos::~sc_sensitive_pos()
{}


// changing between process handles

sc_sensitive_pos&
sc_sensitive_pos::operator << ( sc_process_handle handle_ )
{
    switch ( handle_.proc_kind() )
	{
      case SC_CTHREAD_PROC_:
      case SC_THREAD_PROC_:
        m_mode = SC_THREAD_;
	break;
      case SC_METHOD_PROC_:
	m_mode = SC_METHOD_;
	break;
      default:
	assert(0);
    }
    m_handle = (sc_process_b*)handle_;
    return *this;
}

sc_sensitive_pos&
sc_sensitive_pos::operator << ( sc_method_handle handle_ )
{
    m_mode = SC_METHOD_;
    m_handle = handle_;
    return *this;
}

sc_sensitive_pos&
sc_sensitive_pos::operator << ( sc_thread_handle handle_ )
{
    m_mode = SC_THREAD_;
    m_handle = handle_;
    return *this;
}


sc_sensitive_pos&
sc_sensitive_pos::operator << ( const in_if_b_type& interface_ )
{
    sc_deprecated_sensitive_pos();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_POS_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_:
    case SC_THREAD_: {
	m_handle->add_static_event( interface_.posedge_event() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_pos&
sc_sensitive_pos::operator << ( const in_if_l_type& interface_ )
{
    sc_deprecated_sensitive_pos();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_POS_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_:
    case SC_THREAD_: {
	m_handle->add_static_event( interface_.posedge_event() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_pos&
sc_sensitive_pos::operator << ( const in_port_b_type& port_ )
{
    sc_deprecated_sensitive_pos();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_POS_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ), &port_.pos() );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ), &port_.pos() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_pos&
sc_sensitive_pos::operator << ( const in_port_l_type& port_ )
{
    sc_deprecated_sensitive_pos();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_POS_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ), &port_.pos() );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ), &port_.pos() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_pos&
sc_sensitive_pos::operator << ( const inout_port_b_type& port_ )
{
    sc_deprecated_sensitive_pos();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_POS_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ), &port_.pos() );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ), &port_.pos() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_pos&
sc_sensitive_pos::operator << ( const inout_port_l_type& port_ )
{
    sc_deprecated_sensitive_pos();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_POS_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ), &port_.pos() );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ), &port_.pos() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}


sc_sensitive_pos&
sc_sensitive_pos::operator () ( const in_if_b_type& interface_ )
{
    warn_no_parens();
    return operator << ( interface_ );
}

sc_sensitive_pos&
sc_sensitive_pos::operator () ( const in_if_l_type& interface_ )
{
    warn_no_parens();
    return operator << ( interface_ );
}

sc_sensitive_pos&
sc_sensitive_pos::operator () ( const in_port_b_type& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

sc_sensitive_pos&
sc_sensitive_pos::operator () ( const in_port_l_type& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

sc_sensitive_pos&
sc_sensitive_pos::operator () ( const inout_port_b_type& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

sc_sensitive_pos&
sc_sensitive_pos::operator () ( const inout_port_l_type& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

void sc_sensitive_pos::reset()
{
    m_mode = SC_NONE_;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_sensitive_neg
//
//  Static sensitivity class for negative edge events.
// ----------------------------------------------------------------------------

static void sc_deprecated_sensitive_neg()
{
    static bool warn_sensitive_neg=true;
    if ( warn_sensitive_neg )
    {
        warn_sensitive_neg=false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	 "sc_sensitive_neg is deprecated use sc_sensitive << with neg() instead" );
    }
}

// constructor

sc_sensitive_neg::sc_sensitive_neg( sc_module* module_ )
: m_module( module_ ),
  m_mode( SC_NONE_ ),
  m_handle( 0 )
{}


// destructor

sc_sensitive_neg::~sc_sensitive_neg()
{}


// changing between process handles

sc_sensitive_neg&
sc_sensitive_neg::operator << ( sc_process_handle handle_ )
{
    switch ( handle_.proc_kind() )
	{
      case SC_CTHREAD_PROC_:
      case SC_THREAD_PROC_:
        m_mode = SC_THREAD_;
	break;
      case SC_METHOD_PROC_:
	m_mode = SC_METHOD_;
	break;
      default:
	assert(0);
    }
    m_handle = (sc_process_b*)handle_;
    return *this;
}

sc_sensitive_neg&
sc_sensitive_neg::operator << ( sc_method_handle handle_ )
{
    m_mode = SC_METHOD_;
    m_handle = handle_;
    return *this;
}

sc_sensitive_neg&
sc_sensitive_neg::operator << ( sc_thread_handle handle_ )
{
    m_mode = SC_THREAD_;
    m_handle = handle_;
    return *this;
}


sc_sensitive_neg&
sc_sensitive_neg::operator << ( const in_if_b_type& interface_ )
{
    sc_deprecated_sensitive_neg();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_NEG_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_:
    case SC_THREAD_: {
	m_handle->add_static_event( interface_.negedge_event() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_neg&
sc_sensitive_neg::operator << ( const in_if_l_type& interface_ )
{
    sc_deprecated_sensitive_neg();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_NEG_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_:
    case SC_THREAD_: {
	m_handle->add_static_event( interface_.negedge_event() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_neg&
sc_sensitive_neg::operator << ( const in_port_b_type& port_ )
{
    sc_deprecated_sensitive_neg();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_NEG_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ), &port_.neg() );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ), &port_.neg() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_neg&
sc_sensitive_neg::operator << ( const in_port_l_type& port_ )
{
    sc_deprecated_sensitive_neg();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_NEG_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ), &port_.neg() );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ), &port_.neg() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_neg&
sc_sensitive_neg::operator << ( const inout_port_b_type& port_ )
{
    sc_deprecated_sensitive_neg();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_NEG_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ), &port_.neg() );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ), &port_.neg() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}

sc_sensitive_neg&
sc_sensitive_neg::operator << ( const inout_port_l_type& port_ )
{
    sc_deprecated_sensitive_neg();
    // check
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_MAKE_SENSITIVE_NEG_, "simulation running" );
    }

    // make sensitive
    switch( m_mode ) {
    case SC_METHOD_: {
	port_.make_sensitive( as_method_handle( m_handle ), &port_.neg() );
	break;
    }
    case SC_THREAD_: {
	port_.make_sensitive( as_thread_handle( m_handle ), &port_.neg() );
	break;
    }
    case SC_NONE_:
        /* do nothing */
        break;
    }

    return *this;
}


sc_sensitive_neg&
sc_sensitive_neg::operator () ( const in_if_b_type& interface_ )
{
    warn_no_parens();
    return operator << ( interface_ );
}

sc_sensitive_neg&
sc_sensitive_neg::operator () ( const in_if_l_type& interface_ )
{
    warn_no_parens();
    return operator << ( interface_ );
}

sc_sensitive_neg&
sc_sensitive_neg::operator () ( const in_port_b_type& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

sc_sensitive_neg&
sc_sensitive_neg::operator () ( const in_port_l_type& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

sc_sensitive_neg&
sc_sensitive_neg::operator () ( const inout_port_b_type& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

sc_sensitive_neg&
sc_sensitive_neg::operator () ( const inout_port_l_type& port_ )
{
    warn_no_parens();
    return operator << ( port_ );
}

void sc_sensitive_neg::reset()
{
    m_mode = SC_NONE_;
}

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Bishnupriya Bhattacharya, Cadence Design Systems,
                               25 August, 2003
  Description of Modification: add make_static_sensitivity() routines to support
                               dynamic method process creation with static
                               sensitivity

 *****************************************************************************/

// $Log: sc_sensitive.cpp,v $
// Revision 1.5  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.3  2011/02/13 21:47:38  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.8  2006/04/11 23:13:21  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.7  2006/01/27 17:31:24  acg
//  Andy Goodrich: removed debugging comments from << operator code for types
//  that are deprecated.
//
// Revision 1.6  2006/01/26 21:04:54  acg
//  Andy Goodrich: deprecation message changes and additional messages.
//
// Revision 1.5  2006/01/25 00:31:19  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.4  2006/01/24 20:49:05  acg
// Andy Goodrich: changes to remove the use of deprecated features within the
// simulator, and to issue warning messages when deprecated features are used.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//

// Taf!

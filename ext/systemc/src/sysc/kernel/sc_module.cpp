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

  sc_module.cpp -- Base class of all sequential and combinational processes.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#include <cassert>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "sysc/kernel/sc_event.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_module.h"
#include "sysc/kernel/sc_module_registry.h"
#include "sysc/kernel/sc_name_gen.h"
#include "sysc/kernel/sc_object_manager.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_simcontext_int.h"
#include "sysc/kernel/sc_object_int.h"
#include "sysc/kernel/sc_reset.h"
#include "sysc/communication/sc_communication_ids.h"
#include "sysc/communication/sc_interface.h"
#include "sysc/communication/sc_port.h"
#include "sysc/communication/sc_signal.h"
#include "sysc/communication/sc_signal_ports.h"
#include "sysc/utils/sc_utils_ids.h"
#include "sysc/utils/sc_iostream.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_module_dynalloc_list
//
//  Garbage collection for modules dynamically allocated with SC_NEW.
// ----------------------------------------------------------------------------

class sc_module_dynalloc_list
{
public:

    sc_module_dynalloc_list() : m_list()
        {}

    ~sc_module_dynalloc_list();

    void add( sc_module* p )
        { m_list.push_back( p ); }

private:

    sc_plist<sc_module*> m_list;
};


//------------------------------------------------------------------------------
//"~sc_module_dynalloc_list"
//
// Note we clear the m_parent field for the module being deleted. This because
// we process the list front to back so the parent has already been deleted, 
// and we don't want ~sc_object() to try to access the parent which may 
// contain garbage.
//------------------------------------------------------------------------------
sc_module_dynalloc_list::~sc_module_dynalloc_list()
{
    sc_plist<sc_module*>::iterator it( m_list );
    while( ! it.empty() ) {
        (*it)->m_parent = 0;
        delete *it;
        it ++;
    }
}


// ----------------------------------------------------------------------------

sc_module*
sc_module_dynalloc( sc_module* module_ )
{
    static sc_module_dynalloc_list dynalloc_list;
    dynalloc_list.add( module_ );
    return module_;
}


// ----------------------------------------------------------------------------
//  STRUCT : sc_bind_proxy
//
//  Struct for temporarily storing a pointer to an interface or port.
//  Used for positional binding.
// ----------------------------------------------------------------------------
    
sc_bind_proxy::sc_bind_proxy()
: iface( 0 ),
  port( 0 )
{}

sc_bind_proxy::sc_bind_proxy( sc_interface& iface_ )
: iface( &iface_ ),
  port( 0 )
{}

sc_bind_proxy::sc_bind_proxy( sc_port_base& port_ )
: iface( 0 ),
  port( &port_ )
{}


const sc_bind_proxy SC_BIND_PROXY_NIL;


// ----------------------------------------------------------------------------
//  CLASS : sc_module
//
//  Base class for all structural entities.
// ----------------------------------------------------------------------------

void
sc_module::sc_module_init()
{
    simcontext()->get_module_registry()->insert( *this );
    simcontext()->hierarchy_push( this );
    m_end_module_called = false;
    m_module_name_p = 0;
    m_port_vec = new std::vector<sc_port_base*>;
    m_port_index = 0;
    m_name_gen = new sc_name_gen;
}

/*
 *  This form of the constructor assumes that the user has
 *  used an sc_module_name parameter for his/her constructor.
 *  That parameter object has been pushed onto the stack,
 *  and can be looked up by calling the 
 *  top_of_module_name_stack() function of the object manager.
 *  This technique has two advantages:
 *
 *  1) The user no longer has to write sc_module(name) in the
 *     constructor initializer.
 *  2) The user no longer has to call end_module() at the end
 *     of the constructor -- a common negligence.
 *
 *  But it is important to note that sc_module_name may be used
 *  in the user's constructor's parameter. If it is used anywhere
 *  else, unexpected things will happen. The error-checking
 *  mechanism builtin here cannot hope to catch all misuses.
 *
 */

sc_module::sc_module()
: sc_object(::sc_core::sc_get_curr_simcontext()
                  ->get_object_manager()
                  ->top_of_module_name_stack()
                  ->operator const char*()),
  sensitive(this),
  sensitive_pos(this),
  sensitive_neg(this),
  m_end_module_called(false),
  m_port_vec(),
  m_port_index(0),
  m_name_gen(0),
  m_module_name_p(0)
{
    /* When this form is used, we better have a fresh sc_module_name
       on the top of the stack */
    sc_module_name* mod_name = 
        simcontext()->get_object_manager()->top_of_module_name_stack();
    if (0 == mod_name || 0 != mod_name->m_module_p)
        SC_REPORT_ERROR( SC_ID_SC_MODULE_NAME_REQUIRED_, 0 );
    sc_module_init();
    mod_name->set_module( this );
    m_module_name_p = mod_name; // must come after sc_module_init call.
}

sc_module::sc_module( const sc_module_name& )
: sc_object(::sc_core::sc_get_curr_simcontext()
                  ->get_object_manager()
                  ->top_of_module_name_stack()
                  ->operator const char*()),
  sensitive(this),
  sensitive_pos(this),
  sensitive_neg(this),
  m_end_module_called(false),
  m_port_vec(),
  m_port_index(0),
  m_name_gen(0),
  m_module_name_p(0)
{
    /* For those used to the old style of passing a name to sc_module,
       this constructor will reduce the chance of making a mistake */

    /* When this form is used, we better have a fresh sc_module_name
       on the top of the stack */
    sc_module_name* mod_name = 
        simcontext()->get_object_manager()->top_of_module_name_stack();
    if (0 == mod_name || 0 != mod_name->m_module_p)
      SC_REPORT_ERROR( SC_ID_SC_MODULE_NAME_REQUIRED_, 0 );
    sc_module_init();
    mod_name->set_module( this );
    m_module_name_p = mod_name; // must come after sc_module_init call.
}

/* --------------------------------------------------------------------
 *
 * Deprecated constructors:
 *   sc_module( const char* )
 *   sc_module( const std::string& )
 */
sc_module::sc_module( const char* nm )
: sc_object(nm),
  sensitive(this),
  sensitive_pos(this),
  sensitive_neg(this),
  m_end_module_called(false),
  m_port_vec(),
  m_port_index(0),
  m_name_gen(0),
  m_module_name_p(0)
{
    SC_REPORT_WARNING( SC_ID_BAD_SC_MODULE_CONSTRUCTOR_, nm );
    sc_module_init();
}

sc_module::sc_module( const std::string& s )
: sc_object( s.c_str() ),
  sensitive(this),
  sensitive_pos(this),
  sensitive_neg(this),
  m_end_module_called(false),
  m_port_vec(),
  m_port_index(0),
  m_name_gen(0),
  m_module_name_p(0)
{
    SC_REPORT_WARNING( SC_ID_BAD_SC_MODULE_CONSTRUCTOR_, s.c_str() );
    sc_module_init();
}

/* -------------------------------------------------------------------- */

sc_module::~sc_module()
{
    delete m_port_vec;
    delete m_name_gen;
    orphan_child_objects();
    if ( m_module_name_p )
    {
	m_module_name_p->clear_module( this ); // must be before end_module()
    	end_module();
    }
    simcontext()->get_module_registry()->remove( *this );
}


const ::std::vector<sc_object*>&
sc_module::get_child_objects() const
{
    return m_child_objects;
}

// set SC_THREAD asynchronous reset sensitivity

void
sc_module::async_reset_signal_is( const sc_in<bool>& port, bool level )
{
	sc_reset::reset_signal_is(true, port, level);
}

void
sc_module::async_reset_signal_is( const sc_inout<bool>& port, bool level )
{
	sc_reset::reset_signal_is(true, port, level);
}

void
sc_module::async_reset_signal_is( const sc_out<bool>& port, bool level )
{
	sc_reset::reset_signal_is(true, port, level);
}

void
sc_module::async_reset_signal_is(const sc_signal_in_if<bool>& iface, bool level)
{
	sc_reset::reset_signal_is(true, iface, level);
}

void
sc_module::end_module()
{
    if( ! m_end_module_called ) {
	/* TBD: Can check here to alert the user that end_module
                was not called for a previous module. */
	(void)sc_get_curr_simcontext()->hierarchy_pop();
	sc_get_curr_simcontext()->reset_curr_proc(); 
	sensitive.reset();
	sensitive_pos.reset();
	sensitive_neg.reset();
	m_end_module_called = true;
	m_module_name_p = 0; // make sure we are not called in ~sc_module().
    }
}


// to prevent initialization for SC_METHODs and SC_THREADs

void
sc_module::dont_initialize()
{
    sc_process_handle last_proc = sc_get_last_created_process_handle();
    last_proc.dont_initialize( true );
}

// set SC_THREAD synchronous reset sensitivity

void
sc_module::reset_signal_is( const sc_in<bool>& port, bool level )
{
	sc_reset::reset_signal_is(false, port, level);
}

void
sc_module::reset_signal_is( const sc_inout<bool>& port, bool level )
{
	sc_reset::reset_signal_is(false, port, level);
}

void
sc_module::reset_signal_is( const sc_out<bool>& port, bool level )
{
	sc_reset::reset_signal_is(false, port, level);
}

void
sc_module::reset_signal_is( const sc_signal_in_if<bool>& iface, bool level )
{
	sc_reset::reset_signal_is(false, iface, level);
}

// to generate unique names for objects in an MT-Safe way

const char*
sc_module::gen_unique_name( const char* basename_, bool preserve_first )
{
    return m_name_gen->gen_unique_name( basename_, preserve_first );
}


// called by construction_done 

void
sc_module::before_end_of_elaboration()
{}

// We push the sc_module instance onto the stack of open objects so 
// that any objects that are created in before_end_of_elaboration have
// the proper parent. After the call we pop the hierarchy.
void
sc_module::construction_done()
{
    hierarchy_scope scope(this);
    before_end_of_elaboration();
}

// called by elaboration_done (does nothing by default)

void
sc_module::end_of_elaboration()
{}


// We push the sc_module instance onto the stack of open objects so 
// that any objects that are created in end_of_elaboration have
// the proper parent. After the call we pop the hierarchy.
void
sc_module::elaboration_done( bool& error_ )
{
    if( ! m_end_module_called ) {
	char msg[BUFSIZ];
	std::sprintf( msg, "module '%s'", name() );
	SC_REPORT_WARNING( SC_ID_END_MODULE_NOT_CALLED_, msg );
	if( error_ ) {
	    SC_REPORT_WARNING( SC_ID_HIER_NAME_INCORRECT_, 0 );
	}
	error_ = true;
    }
    hierarchy_scope scope(this);
    end_of_elaboration();
}

// called by start_simulation (does nothing by default)

void
sc_module::start_of_simulation()
{}

void
sc_module::start_simulation()
{
    hierarchy_scope scope(this);
    start_of_simulation();
}

// called by simulation_done (does nothing by default)

void
sc_module::end_of_simulation()
{}

void
sc_module::simulation_done()
{
    hierarchy_scope scope(this);
    end_of_simulation();
}

void
sc_module::set_stack_size( std::size_t size )
{
    sc_process_handle  proc_h(
    	sc_is_running() ?
	sc_get_current_process_handle() :
	sc_get_last_created_process_handle()
    );
    sc_thread_handle thread_h;  // Current process as thread.


    thread_h = (sc_thread_handle)proc_h;
    if ( thread_h ) 
    {
	thread_h->set_stack_size( size );
    }
    else
    {
	SC_REPORT_WARNING( SC_ID_SET_STACK_SIZE_, 0 );
    }
}


int
sc_module::append_port( sc_port_base* port_ )
{
    int index = m_port_vec->size();
    m_port_vec->push_back( port_ );
    return index;
}


// positional binding methods

static void sc_warn_arrow_arrow_bind()
{
    static bool warn_arrow_arrow_bind=true;
    if ( warn_arrow_arrow_bind )
    {
    	warn_arrow_arrow_bind = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "positional binding using << or , is deprecated, use () instead.");
    }
}

sc_module&
sc_module::operator << ( sc_interface& interface_ )
{
    sc_warn_arrow_arrow_bind();
    positional_bind(interface_);
    return *this;
}

sc_module&
sc_module::operator << ( sc_port_base& port_ )
{
    sc_warn_arrow_arrow_bind();
    positional_bind(port_);
    return *this;
}


void
sc_module::positional_bind( sc_interface& interface_ )
{
    if( m_port_index == (int)m_port_vec->size() ) {
	char msg[BUFSIZ];
	if( m_port_index == 0 ) {
	    std::sprintf( msg, "module `%s' has no ports", name() );
	} else {
	    std::sprintf( msg, "all ports of module `%s' are bound", name() );
	}
	SC_REPORT_ERROR( SC_ID_BIND_IF_TO_PORT_, msg );
    }
    int status = (*m_port_vec)[m_port_index]->pbind( interface_ );
    if( status != 0 ) {
	char msg[BUFSIZ];
	switch( status ) {
	case 1:
	    std::sprintf( msg, "port %d of module `%s' is already bound",
		     m_port_index, name() );
	    break;
	case 2:
	    std::sprintf( msg, "type mismatch on port %d of module `%s'",
		     m_port_index, name() );
	    break;
	default:
	    std::sprintf( msg, "unknown error" );
	    break;
	}
	SC_REPORT_ERROR( SC_ID_BIND_IF_TO_PORT_, msg );
    }
    ++ m_port_index;
}

void
sc_module::positional_bind( sc_port_base& port_ )
{
    if( m_port_index == (int)m_port_vec->size() ) {
	char msg[BUFSIZ];
	if( m_port_index == 0 ) {
	    std::sprintf( msg, "module `%s' has no ports", name() );
	} else {
	    std::sprintf( msg, "all ports of module `%s' are bound", name() );
	}
	SC_REPORT_ERROR( SC_ID_BIND_PORT_TO_PORT_, msg );
    }
    int status = (*m_port_vec)[m_port_index]->pbind( port_ );
    if( status != 0 ) {
	char msg[BUFSIZ];
	switch( status ) {
	case 1:
	    std::sprintf( msg, "port %d of module `%s' is already bound",
		     m_port_index, name() );
	    break;
	case 2:
	    std::sprintf( msg, "type mismatch on port %d of module `%s'",
		     m_port_index, name() );
	    break;
	default:
	    std::sprintf( msg, "unknown error" );
	    break;
	}
	SC_REPORT_ERROR( SC_ID_BIND_PORT_TO_PORT_, msg );
    }
    ++ m_port_index;
}


#define TRY_BIND( p )                                                         \
    if( (p).iface != 0 ) {                                                    \
        positional_bind( *(p).iface );                                        \
    } else if( (p).port != 0 ) {                                              \
        positional_bind( *(p).port );                                         \
    } else {                                                                  \
        return;                                                               \
    }


void
sc_module::operator () ( const sc_bind_proxy& p001,
			 const sc_bind_proxy& p002,
			 const sc_bind_proxy& p003,
			 const sc_bind_proxy& p004,
			 const sc_bind_proxy& p005,
			 const sc_bind_proxy& p006,
			 const sc_bind_proxy& p007,
			 const sc_bind_proxy& p008,
			 const sc_bind_proxy& p009,
			 const sc_bind_proxy& p010,
			 const sc_bind_proxy& p011,
			 const sc_bind_proxy& p012,
			 const sc_bind_proxy& p013,
			 const sc_bind_proxy& p014,
			 const sc_bind_proxy& p015,
			 const sc_bind_proxy& p016,
			 const sc_bind_proxy& p017,
			 const sc_bind_proxy& p018,
			 const sc_bind_proxy& p019,
			 const sc_bind_proxy& p020,
			 const sc_bind_proxy& p021,
			 const sc_bind_proxy& p022,
			 const sc_bind_proxy& p023,
			 const sc_bind_proxy& p024,
			 const sc_bind_proxy& p025,
			 const sc_bind_proxy& p026,
			 const sc_bind_proxy& p027,
			 const sc_bind_proxy& p028,
			 const sc_bind_proxy& p029,
			 const sc_bind_proxy& p030,
			 const sc_bind_proxy& p031,
			 const sc_bind_proxy& p032,
			 const sc_bind_proxy& p033,
			 const sc_bind_proxy& p034,
			 const sc_bind_proxy& p035,
			 const sc_bind_proxy& p036,
			 const sc_bind_proxy& p037,
			 const sc_bind_proxy& p038,
			 const sc_bind_proxy& p039,
			 const sc_bind_proxy& p040,
			 const sc_bind_proxy& p041,
			 const sc_bind_proxy& p042,
			 const sc_bind_proxy& p043,
			 const sc_bind_proxy& p044,
			 const sc_bind_proxy& p045,
			 const sc_bind_proxy& p046,
			 const sc_bind_proxy& p047,
			 const sc_bind_proxy& p048,
			 const sc_bind_proxy& p049,
			 const sc_bind_proxy& p050,
			 const sc_bind_proxy& p051,
			 const sc_bind_proxy& p052,
			 const sc_bind_proxy& p053,
			 const sc_bind_proxy& p054,
			 const sc_bind_proxy& p055,
			 const sc_bind_proxy& p056,
			 const sc_bind_proxy& p057,
			 const sc_bind_proxy& p058,
			 const sc_bind_proxy& p059,
			 const sc_bind_proxy& p060,
			 const sc_bind_proxy& p061,
			 const sc_bind_proxy& p062,
			 const sc_bind_proxy& p063,
			 const sc_bind_proxy& p064 )
{
    static bool warn_only_once=true;
    if ( m_port_index > 0 && warn_only_once )
    {
        warn_only_once = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	 "multiple () binding deprecated, use explicit port binding instead." );
    }

    TRY_BIND( p001 );
    TRY_BIND( p002 );
    TRY_BIND( p003 );
    TRY_BIND( p004 );
    TRY_BIND( p005 );
    TRY_BIND( p006 );
    TRY_BIND( p007 );
    TRY_BIND( p008 );
    TRY_BIND( p009 );
    TRY_BIND( p010 );
    TRY_BIND( p011 );
    TRY_BIND( p012 );
    TRY_BIND( p013 );
    TRY_BIND( p014 );
    TRY_BIND( p015 );
    TRY_BIND( p016 );
    TRY_BIND( p017 );
    TRY_BIND( p018 );
    TRY_BIND( p019 );
    TRY_BIND( p020 );
    TRY_BIND( p021 );
    TRY_BIND( p022 );
    TRY_BIND( p023 );
    TRY_BIND( p024 );
    TRY_BIND( p025 );
    TRY_BIND( p026 );
    TRY_BIND( p027 );
    TRY_BIND( p028 );
    TRY_BIND( p029 );
    TRY_BIND( p030 );
    TRY_BIND( p031 );
    TRY_BIND( p032 );
    TRY_BIND( p033 );
    TRY_BIND( p034 );
    TRY_BIND( p035 );
    TRY_BIND( p036 );
    TRY_BIND( p037 );
    TRY_BIND( p038 );
    TRY_BIND( p039 );
    TRY_BIND( p040 );
    TRY_BIND( p041 );
    TRY_BIND( p042 );
    TRY_BIND( p043 );
    TRY_BIND( p044 );
    TRY_BIND( p045 );
    TRY_BIND( p046 );
    TRY_BIND( p047 );
    TRY_BIND( p048 );
    TRY_BIND( p049 );
    TRY_BIND( p050 );
    TRY_BIND( p051 );
    TRY_BIND( p052 );
    TRY_BIND( p053 );
    TRY_BIND( p054 );
    TRY_BIND( p055 );
    TRY_BIND( p056 );
    TRY_BIND( p057 );
    TRY_BIND( p058 );
    TRY_BIND( p059 );
    TRY_BIND( p060 );
    TRY_BIND( p061 );
    TRY_BIND( p062 );
    TRY_BIND( p063 );
    TRY_BIND( p064 );
}

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Ali Dasdan, Synopsys, Inc.
  Description of Modification: - Implementation of operator() and operator,
                                 positional connection method.
                               - Implementation of error checking in
                                 operator<<'s.
                               - Implementation of the function test_module_prm.
                               - Implementation of set_stack_size().

      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems 20 May 2003
  Description of Modification: Inherit from sc_process_host

      Name, Affiliation, Date: Bishnupriya Bhattacharya, Cadence Design Systems,
                               25 August, 2003
  Description of Modification: dont_initialize() uses 
                               sc_get_last_created_process_handle() instead of
                               sc_get_current_process_b()

      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems 25 Mar 2003
  Description of Modification: Fixed bug in SC_NEW, see comments on 
                               ~sc_module_dynalloc_list below.


 *****************************************************************************/


// $Log: sc_module.cpp,v $
// Revision 1.14  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.13  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.12  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.11  2011/03/05 19:44:20  acg
//  Andy Goodrich: changes for object and event naming and structures.
//
// Revision 1.10  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.9  2011/02/16 22:37:30  acg
//  Andy Goodrich: clean up to remove need for ps_disable_pending.
//
// Revision 1.8  2011/02/14 17:51:40  acg
//  Andy Goodrich: proper pushing an poppping of the module hierarchy for
//  start_of_simulation() and end_of_simulation.
//
// Revision 1.7  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.6  2011/01/25 20:50:37  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.5  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.4  2008/11/17 15:57:15  acg
//  Andy Goodrich: added deprecation message for sc_module(const char*)
//
// Revision 1.3  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.2  2007/05/17 20:16:33  acg
//  Andy Goodrich: changes for beta release to LWG.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.9  2006/12/02 20:58:18  acg
//  Andy Goodrich: updates from 2.2 for IEEE 1666 support.
//
// Revision 1.8  2006/03/21 00:00:34  acg
//   Andy Goodrich: changed name of sc_get_current_process_base() to be
//   sc_get_current_process_b() since its returning an sc_process_b instance.
//
// Revision 1.7  2006/03/14 23:56:58  acg
//   Andy Goodrich: This fixes a bug when an exception is thrown in
//   sc_module::sc_module() for a dynamically allocated sc_module
//   object. We are calling sc_module::end_module() on a module that has
//   already been deleted. The scenario runs like this:
//
//   a) the sc_module constructor is entered
//   b) the exception is thrown
//   c) the exception processor deletes the storage for the sc_module
//   d) the stack is unrolled causing the sc_module_name instance to be deleted
//   e) ~sc_module_name() calls end_module() with its pointer to the sc_module
//   f) because the sc_module has been deleted its storage is corrupted,
//      either by linking it to a free space chain, or by reuse of some sort
//   g) the m_simc field is garbage
//   h) the m_object_manager field is also garbage
//   i) an exception occurs
//
//   This does not happen for automatic sc_module instances since the
//   storage for the module is not reclaimed its just part of the stack.
//
//   I am fixing this by having the destructor for sc_module clear the
//   module pointer in its sc_module_name instance. That cuts things at
//   step (e) above, since the pointer will be null if the module has
//   already been deleted. To make sure the module stack is okay, I call
//   end-module() in ~sc_module in the case where there is an
//   sc_module_name pointer lying around.
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
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.
//

// Taf!

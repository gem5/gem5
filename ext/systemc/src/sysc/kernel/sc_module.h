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

  sc_module.h -- Base class of all hierarchical modules and channels.

  Original Author: Stan Y. Liao, Synopsys, Inc.
                   Martin Janssen, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_MODULE_H
#define SC_MODULE_H

#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_module_name.h"
#include "sysc/kernel/sc_sensitive.h"
#include "sysc/kernel/sc_time.h"
#include "sysc/kernel/sc_wait.h"
#include "sysc/kernel/sc_wait_cthread.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/utils/sc_list.h"

namespace sc_core {

class sc_name_gen;
template<class T> class sc_in;
template<class T> class sc_inout;
template<class T> class sc_out;

// ----------------------------------------------------------------------------
//  STRUCT : sc_bind_proxy
//
//  Struct for temporarily storing a pointer to an interface or port.
//  Used for positional binding.
// ----------------------------------------------------------------------------

struct sc_bind_proxy
{
    sc_interface* iface;
    sc_port_base* port;
    
    sc_bind_proxy();
    sc_bind_proxy( sc_interface& );
    sc_bind_proxy( sc_port_base& );
};


extern const sc_bind_proxy SC_BIND_PROXY_NIL;


// ----------------------------------------------------------------------------
//  CLASS : sc_module
//
//  Base class for all structural entities.
// ----------------------------------------------------------------------------

class sc_module
: public sc_object, public sc_process_host
{
    friend class sc_module_name;
    friend class sc_module_registry;
    friend class sc_object;
    friend class sc_port_registry;
	friend class sc_process_b;
    friend class sc_simcontext;

public:

    sc_simcontext* sc_get_curr_simcontext()
	{ return simcontext(); }

    // to generate unique names for objects in an MT-Safe way
    const char* gen_unique_name( const char* basename_, bool preserve_first );

    virtual const char* kind() const
        { return "sc_module"; }

protected:
  
    // called by construction_done 
    virtual void before_end_of_elaboration();

    void construction_done();

    // called by elaboration_done (does nothing by default)
    virtual void end_of_elaboration();

    void elaboration_done( bool& );

    // called by start_simulation (does nothing by default)
    virtual void start_of_simulation();

    void start_simulation();

    // called by simulation_done (does nothing by default)
    virtual void end_of_simulation();

    void simulation_done();

    void sc_module_init();

    // constructor
    sc_module();
    sc_module( const sc_module_name& nm ); /* for those used to old style */

    /* DEPRECATED */ sc_module( const char* nm ); 
    /* DEPRECATED */ sc_module( const std::string& nm );

public:

    // destructor
    virtual ~sc_module();

    // positional binding methods

    sc_module& operator << ( sc_interface& );
    sc_module& operator << ( sc_port_base& );

    sc_module& operator , ( sc_interface& interface_ )
        { return operator << ( interface_ ); }

    sc_module& operator , ( sc_port_base& port_ )
        { return operator << ( port_ ); }

    // operator() is declared at the end of the class.

    const ::std::vector<sc_object*>& get_child_objects() const;

protected:

    // this must be called by user-defined modules
    void end_module();


    // to prevent initialization for SC_METHODs and SC_THREADs
    void dont_initialize();

    // positional binding code - used by operator ()

    void positional_bind( sc_interface& );
    void positional_bind( sc_port_base& );

    // set reset sensitivity for SC_xTHREADs
    void async_reset_signal_is( const sc_in<bool>& port, bool level );
    void async_reset_signal_is( const sc_inout<bool>& port, bool level );
    void async_reset_signal_is( const sc_out<bool>& port, bool level );
    void async_reset_signal_is( const sc_signal_in_if<bool>& iface, bool level);
    void reset_signal_is( const sc_in<bool>& port, bool level );
    void reset_signal_is( const sc_inout<bool>& port, bool level );
    void reset_signal_is( const sc_out<bool>& port, bool level );
    void reset_signal_is( const sc_signal_in_if<bool>& iface, bool level );

    // static sensitivity for SC_THREADs and SC_CTHREADs

    void wait()
        { ::sc_core::wait( simcontext() ); }

    // dynamic sensitivity for SC_THREADs and SC_CTHREADs

    void wait( const sc_event& e )
        { ::sc_core::wait( e, simcontext() ); }

    void wait( const sc_event_or_list& el )
	{ ::sc_core::wait( el, simcontext() ); }

    void wait( const sc_event_and_list& el )
	{ ::sc_core::wait( el, simcontext() ); }

    void wait( const sc_time& t )
        { ::sc_core::wait( t, simcontext() ); }

    void wait( double v, sc_time_unit tu )
        { ::sc_core::wait( sc_time( v, tu, simcontext() ), simcontext() ); }

    void wait( const sc_time& t, const sc_event& e )
        { ::sc_core::wait( t, e, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event& e )
        { ::sc_core::wait( 
		sc_time( v, tu, simcontext() ), e, simcontext() ); }

    void wait( const sc_time& t, const sc_event_or_list& el )
        { ::sc_core::wait( t, el, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event_or_list& el )
        { ::sc_core::wait( sc_time( v, tu, simcontext() ), el, simcontext() ); }

    void wait( const sc_time& t, const sc_event_and_list& el )
        { ::sc_core::wait( t, el, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event_and_list& el )
        { ::sc_core::wait( sc_time( v, tu, simcontext() ), el, simcontext() ); }


    // static sensitivity for SC_METHODs

    void next_trigger()
	{ ::sc_core::next_trigger( simcontext() ); }


    // dynamic sensitivty for SC_METHODs

    void next_trigger( const sc_event& e )
        { ::sc_core::next_trigger( e, simcontext() ); }

    void next_trigger( const sc_event_or_list& el )
        { ::sc_core::next_trigger( el, simcontext() ); }

    void next_trigger( const sc_event_and_list& el )
        { ::sc_core::next_trigger( el, simcontext() ); }

    void next_trigger( const sc_time& t )
        { ::sc_core::next_trigger( t, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu )
        { ::sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), simcontext() ); }

    void next_trigger( const sc_time& t, const sc_event& e )
        { ::sc_core::next_trigger( t, e, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event& e )
        { ::sc_core::next_trigger( 
		sc_time( v, tu, simcontext() ), e, simcontext() ); }

    void next_trigger( const sc_time& t, const sc_event_or_list& el )
        { ::sc_core::next_trigger( t, el, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event_or_list& el )
        { ::sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), el, simcontext() ); }

    void next_trigger( const sc_time& t, const sc_event_and_list& el )
        { ::sc_core::next_trigger( t, el, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event_and_list& el )
        { ::sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), el, simcontext() ); }


    // for SC_METHODs and SC_THREADs and SC_CTHREADs

    bool timed_out()
        { return ::sc_core::timed_out(); }


    // for SC_CTHREADs

    void halt()
        { ::sc_core::halt( simcontext() ); }

    void wait( int n )
        { ::sc_core::wait( n, simcontext() ); }

    void at_posedge( const sc_signal_in_if<bool>& s )
	{ ::sc_core::at_posedge( s, simcontext() ); }

    void at_posedge( const sc_signal_in_if<sc_dt::sc_logic>& s )
	{ ::sc_core::at_posedge( s, simcontext() ); }

    void at_negedge( const sc_signal_in_if<bool>& s )
	{ ::sc_core::at_negedge( s, simcontext() ); }

    void at_negedge( const sc_signal_in_if<sc_dt::sc_logic>& s )
	{ ::sc_core::at_negedge( s, simcontext() ); }

    // Catch uses of watching:
    void watching( bool /* expr */ )
        { SC_REPORT_ERROR(SC_ID_WATCHING_NOT_ALLOWED_,""); }

    // These are protected so that user derived classes can refer to them.
    sc_sensitive     sensitive;
    sc_sensitive_pos sensitive_pos;
    sc_sensitive_neg sensitive_neg;

    // Function to set the stack size of the current (c)thread process.
    void set_stack_size( std::size_t );

    int append_port( sc_port_base* );

private:
    sc_module( const sc_module& );
    const sc_module& operator = ( const sc_module& );

private:

    bool                        m_end_module_called;
    std::vector<sc_port_base*>* m_port_vec;
    int                         m_port_index;
    sc_name_gen*                m_name_gen;
    sc_module_name*             m_module_name_p;

public:

    void defunct() { }

    // positional binding methods (cont'd)

    void operator () ( const sc_bind_proxy& p001,
		       const sc_bind_proxy& p002 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p003 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p004 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p005 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p006 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p007 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p008 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p009 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p010 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p011 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p012 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p013 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p014 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p015 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p016 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p017 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p018 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p019 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p020 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p021 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p022 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p023 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p024 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p025 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p026 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p027 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p028 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p029 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p030 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p031 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p032 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p033 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p034 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p035 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p036 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p037 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p038 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p039 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p040 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p041 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p042 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p043 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p044 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p045 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p046 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p047 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p048 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p049 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p050 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p051 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p052 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p053 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p054 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p055 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p056 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p057 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p058 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p059 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p060 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p061 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p062 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p063 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p064 = SC_BIND_PROXY_NIL );

};

extern sc_module* sc_module_dynalloc(sc_module*);
#define SC_NEW(x)  ::sc_core::sc_module_dynalloc(new x);


// -----------------------------------------------------------------------------
// SOME MACROS TO SIMPLIFY SYNTAX:
// -----------------------------------------------------------------------------

#define SC_MODULE(user_module_name)                                           \
    struct user_module_name : ::sc_core::sc_module

#define SC_CTOR(user_module_name)                                             \
    typedef user_module_name SC_CURRENT_USER_MODULE;                          \
    user_module_name( ::sc_core::sc_module_name )

// the SC_HAS_PROCESS macro call must be followed by a ;
#define SC_HAS_PROCESS(user_module_name)                                      \
    typedef user_module_name SC_CURRENT_USER_MODULE

// The this-> construct on sensitive operators in the macros below is
// required for gcc 4.x when a templated class has a templated parent that is
// derived from sc_module:
//
// template<typename X>
// class B : public sc_module;
// template<typename X>
// class A : public B<X>

#define declare_method_process(handle, name, host_tag, func)        \
    {		                                                    \
        ::sc_core::sc_process_handle handle =                      \
	    sc_core::sc_get_curr_simcontext()->create_method_process( \
		name,  false, SC_MAKE_FUNC_PTR( host_tag, func ), \
		this, 0 ); \
        this->sensitive << handle;                                        \
        this->sensitive_pos << handle;                                    \
        this->sensitive_neg << handle;                                    \
    }

#define declare_thread_process(handle, name, host_tag, func)        \
    {                                                               \
        ::sc_core::sc_process_handle handle =                      \
	     sc_core::sc_get_curr_simcontext()->create_thread_process( \
                 name,  false,           \
                 SC_MAKE_FUNC_PTR( host_tag, func ), this, 0 ); \
        this->sensitive << handle;                                        \
        this->sensitive_pos << handle;                                    \
        this->sensitive_neg << handle;                                    \
    }

#define declare_cthread_process(handle, name, host_tag, func, edge) \
    {                                                               \
        ::sc_core::sc_process_handle handle =                     \
	     sc_core::sc_get_curr_simcontext()->create_cthread_process( \
            name,  false,          \
                     SC_MAKE_FUNC_PTR( host_tag, func ), this, 0 ); \
        this->sensitive.operator() ( handle, edge );\
    }

#define SC_CTHREAD(func, edge)                                                \
    declare_cthread_process( func ## _handle,                                 \
                             #func,                                           \
                             SC_CURRENT_USER_MODULE,                          \
                             func,                                            \
                             edge )

#define SC_METHOD(func)                                                       \
    declare_method_process( func ## _handle,                                  \
                            #func,                                            \
                            SC_CURRENT_USER_MODULE,                           \
                            func )

#define SC_THREAD(func)                                                       \
    declare_thread_process( func ## _handle,                                  \
                            #func,                                            \
                            SC_CURRENT_USER_MODULE,                           \
                            func )



// ----------------------------------------------------------------------------
//  TYPEDEFS
// ----------------------------------------------------------------------------

typedef sc_module sc_channel;
typedef sc_module sc_behavior;

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

      Name, Affiliation, Date: Gene Bushuyev, Synopsys, Inc.
  Description of Modification: - Change implementation for VC6.

      Name, Affiliation, Date: Andy Godorich, Forte
                               Bishnupriya Bhattacharya, Cadence Design Systems,
                               25 August, 2003
  Description of Modification: inherit from sc_process_host as a part of
                               implementing dynamic processes

 *****************************************************************************/

// $Log: sc_module.h,v $
// Revision 1.11  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.10  2011/08/15 16:43:24  acg
//  Torsten Maehne: changes to remove unused argument warnings.
//
// Revision 1.9  2011/03/05 19:44:20  acg
//  Andy Goodrich: changes for object and event naming and structures.
//
// Revision 1.8  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.7  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.6  2011/01/18 20:10:44  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.5  2010/12/07 20:09:12  acg
// Andy Goodrich: remove unused signal declaration
//
// Revision 1.4  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.3  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.2  2007/01/24 20:14:40  acg
//  Andy Goodrich: improved comments about the use of this-> in the macros
//  that access sensitive.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.10  2006/12/02 20:58:18  acg
//  Andy Goodrich: updates from 2.2 for IEEE 1666 support.
//
// Revision 1.7  2006/04/11 23:13:21  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.6  2006/03/15 17:53:34  acg
//  Andy Goodrich, Forte Design
//  Reordered includes to pick up <cassert> for use by sc_process_name.h
//
// Revision 1.5  2006/03/14 23:56:58  acg
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
// Revision 1.4  2006/01/24 20:49:05  acg
// Andy Goodrich: changes to remove the use of deprecated features within the
// simulator, and to issue warning messages when deprecated features are used.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.

#endif

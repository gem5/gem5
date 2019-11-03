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

  sc_simcontext.cpp -- Provides a simulation context for use with multiple
                       simulations.

  Original Author: Stan Y. Liao, Synopsys, Inc.
                   Martin Janssen, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#include <algorithm>

#define SC_DISABLE_API_VERSION_CHECK // for in-library sc_ver.h inclusion

#include "sysc/kernel/sc_cor_fiber.h"
#include "sysc/kernel/sc_cor_pthread.h"
#include "sysc/kernel/sc_cor_qt.h"
#include "sysc/kernel/sc_event.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_module.h"
#include "sysc/kernel/sc_module_registry.h"
#include "sysc/kernel/sc_name_gen.h"
#include "sysc/kernel/sc_object_manager.h"
#include "sysc/kernel/sc_cthread_process.h"
#include "sysc/kernel/sc_method_process.h"
#include "sysc/kernel/sc_thread_process.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_simcontext_int.h"
#include "sysc/kernel/sc_reset.h"
#include "sysc/kernel/sc_ver.h"
#include "sysc/kernel/sc_boost.h"
#include "sysc/kernel/sc_spawn.h"
#include "sysc/kernel/sc_phase_callback_registry.h"
#include "sysc/communication/sc_port.h"
#include "sysc/communication/sc_export.h"
#include "sysc/communication/sc_prim_channel.h"
#include "sysc/tracing/sc_trace.h"
#include "sysc/utils/sc_mempool.h"
#include "sysc/utils/sc_list.h"
#include "sysc/utils/sc_utils_ids.h"

// DEBUGGING MACROS:
//
// DEBUG_MSG(NAME,P,MSG)
//     MSG  = message to print
//     NAME = name that must match the process for the message to print, or
//            null if the message should be printed unconditionally.
//     P    = pointer to process message is for, or NULL in which case the
//            message will not print.
#if 0
#   define DEBUG_NAME ""
#   define DEBUG_MSG(NAME,P,MSG) \
    { \
        if ( P && ( (strlen(NAME)==0) || !strcmp(NAME,P->name())) ) \
          std::cout << "**** " << sc_time_stamp() << " ("  \
	            << sc_get_current_process_name() << "): " << MSG \
		    << " - " << P->name() << std::endl; \
    }
#else
#   define DEBUG_MSG(NAME,P,MSG) 
#endif

#if SC_HAS_PHASE_CALLBACKS_
#  define SC_DO_PHASE_CALLBACK_( Kind ) \
    m_phase_cb_registry->Kind()
#else
#  define SC_DO_PHASE_CALLBACK_( Kind ) \
    ((void)0) /* do nothing */
#endif

#if defined( SC_ENABLE_SIMULATION_PHASE_CALLBACKS_TRACING )
// use callback based tracing
#  define SC_SIMCONTEXT_TRACING_  0
#else
// enable tracing via explicit trace_cycle calls from simulator loop
#  define SC_SIMCONTEXT_TRACING_  1
#endif

namespace sc_core {

sc_stop_mode stop_mode = SC_STOP_FINISH_DELTA;

// ----------------------------------------------------------------------------
//  CLASS : sc_process_table
//
//  Container class that keeps track of all method processes,
//  (c)thread processes.
// ----------------------------------------------------------------------------

class sc_process_table
{
  public:

    sc_process_table();
    ~sc_process_table();
    void push_front( sc_method_handle );
    void push_front( sc_thread_handle );
    sc_method_handle method_q_head();
    sc_method_handle remove( sc_method_handle );
    sc_thread_handle thread_q_head();
    sc_thread_handle remove( sc_thread_handle );


  private:

    sc_method_handle  m_method_q;  // Queue of existing method processes.
    sc_thread_handle  m_thread_q;  // Queue of existing thread processes.
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

sc_process_table::sc_process_table() : m_method_q(0), m_thread_q(0)
{}

sc_process_table::~sc_process_table()
{

    sc_method_handle  method_next_p;	// Next method to delete.
    sc_method_handle  method_now_p;	// Method now deleting.

    for( method_now_p = m_method_q; method_now_p; method_now_p = method_next_p )
    {
	method_next_p = method_now_p->next_exist();
	delete method_now_p;
    }

    if ( m_thread_q )
    {
        ::std::cout << ::std::endl 
             << "WATCH OUT!! In sc_process_table destructor. "
             << "Threads and cthreads are not actually getting deleted here. "
	     << "Some memory may leak. Look at the comments here in "
	     << "kernel/sc_simcontext.cpp for more details."
	     << ::std::endl;
    }

    // don't delete threads and cthreads. If a (c)thread
    // has died, then it has already been deleted. Only (c)threads created
    // before simulation-start are in this table. Due to performance
    // reasons, we don't look up the dying thread in the process table
    // and remove it from there. simcontext::reset and ~simcontext invoke this
    // destructor. At present none of these routines are ever invoked. 
    // We can delete threads and cthreads here if a dying thread figured out
    // it was created before simulation-start and took itself off the 
    // process_table. 

#if 0
    sc_thread_handle  thread_next_p;	// Next thread to delete.
    sc_thread_handle  thread_now_p;	// Thread now deleting.

    for( thread_now_p=m_thread_q; thread_now_p; thread_now_p=thread_next_p )
    {
	thread_next_p = thread_now_p->next_exist();
	delete thread_now_p;
    }
#endif // 0
}

inline
sc_method_handle 
sc_process_table::method_q_head()
{
    return m_method_q;
}

inline
void
sc_process_table::push_front( sc_method_handle handle_ )
{
    handle_->set_next_exist(m_method_q);
    m_method_q = handle_;
}

inline
void
sc_process_table::push_front( sc_thread_handle handle_ )
{
    handle_->set_next_exist(m_thread_q);
    m_thread_q = handle_;
}

sc_method_handle
sc_process_table::remove( sc_method_handle handle_ )
{
    sc_method_handle now_p;	// Entry now examining.
    sc_method_handle prior_p;	// Entry prior to one now examining.

    prior_p = 0;
    for ( now_p = m_method_q; now_p; now_p = now_p->next_exist() )
    {
	if ( now_p == handle_ )
	{
	    if ( prior_p )
		prior_p->set_next_exist( now_p->next_exist() );
	    else
		m_method_q = now_p->next_exist();
	    return handle_;
	}
    }
    return 0;
}

sc_thread_handle
sc_process_table::remove( sc_thread_handle handle_ )
{
    sc_thread_handle now_p;	// Entry now examining.
    sc_thread_handle prior_p;	// Entry prior to one now examining.

    prior_p = 0;
    for ( now_p = m_thread_q; now_p; now_p = now_p->next_exist() )
    {
	if ( now_p == handle_ )
	{
	    if ( prior_p )
		prior_p->set_next_exist( now_p->next_exist() );
	    else
		m_thread_q = now_p->next_exist();
	    return handle_;
	}
    }
    return 0;
}

inline
sc_thread_handle 
sc_process_table::thread_q_head()
{
    return m_thread_q;
}

int
sc_notify_time_compare( const void* p1, const void* p2 )
{
    const sc_event_timed* et1 = static_cast<const sc_event_timed*>( p1 );
    const sc_event_timed* et2 = static_cast<const sc_event_timed*>( p2 );

    const sc_time& t1 = et1->notify_time();
    const sc_time& t2 = et2->notify_time();
    
    if( t1 < t2 ) {
	return 1;
    } else if( t1 > t2 ) {
	return -1;
    } else {
	return 0;
    }
}


// +============================================================================
// | CLASS sc_invoke_method - class to invoke sc_method's to support 
// |                          sc_simcontext::preempt_with().
// +============================================================================
SC_MODULE(sc_invoke_method)
{
    SC_CTOR(sc_invoke_method)
    {
      // remove from object hierarchy
      detach();
    }

    virtual ~sc_invoke_method()
    {
	m_invokers.resize(0);
    }

    // Method to call to execute a method's semantics. 
    
    void invoke_method( sc_method_handle method_h )
    {
	sc_process_handle invoker_h;  // handle for invocation thread to use.
	std::vector<sc_process_handle>::size_type invokers_n; // number of invocation threads available.

	m_method = method_h;

	// There is not an invocation thread to use, so allocate one.

	invokers_n = m_invokers.size();
	if ( invokers_n == 0 )
	{
	    sc_spawn_options options;
	    options.dont_initialize();
	    options.set_stack_size(0x100000);
	    options.set_sensitivity(&m_dummy);
	    invoker_h = sc_spawn(sc_bind(&sc_invoke_method::invoker,this), 
				 sc_gen_unique_name("invoker"), &options);
	    ((sc_process_b*)invoker_h)->detach();
	}

	// There is an invocation thread to use, use the last one on the list.

	else
	{
	    invoker_h = m_invokers[invokers_n-1];
	    m_invokers.pop_back();
	}

	// Fire off the invocation thread to invoke the method's semantics,
	// When it blocks put it onto the list of invocation threads that
	// are available.

        sc_get_curr_simcontext()->preempt_with( (sc_thread_handle)invoker_h );
	DEBUG_MSG( DEBUG_NAME, m_method, "back from preemption" ); 
	m_invokers.push_back(invoker_h);
    }

    // Thread to call method from:

    void invoker()
    {
	sc_simcontext* csc_p = sc_get_curr_simcontext();
	sc_process_b*  me = sc_get_current_process_b();

	DEBUG_MSG( DEBUG_NAME, me, "invoker initialization" );
        for (;; )
        {
            DEBUG_MSG( DEBUG_NAME, m_method, "invoker executing method" );
	    csc_p->set_curr_proc( (sc_process_b*)m_method );
	    csc_p->get_active_invokers().push_back((sc_thread_handle)me);
	    m_method->run_process();
	    csc_p->set_curr_proc( me );
	    csc_p->get_active_invokers().pop_back();
            DEBUG_MSG( DEBUG_NAME, m_method, "back from executing method" );
	    wait();
	}
    }

    sc_event                       m_dummy;    // dummy event to wait on.
    sc_method_handle               m_method;   // method to be invoked.
    std::vector<sc_process_handle> m_invokers; // list of invoking threads.
};

// ----------------------------------------------------------------------------
//  CLASS : sc_simcontext
//
//  The simulation context.
// ----------------------------------------------------------------------------

void
sc_simcontext::init()
{

    // ALLOCATE VARIOUS MANAGERS AND REGISTRIES:

    m_object_manager = new sc_object_manager;
    m_module_registry = new sc_module_registry( *this );
    m_port_registry = new sc_port_registry( *this );
    m_export_registry = new sc_export_registry( *this );
    m_prim_channel_registry = new sc_prim_channel_registry( *this );
    m_phase_cb_registry = new sc_phase_callback_registry( *this );
    m_name_gen = new sc_name_gen;
    m_process_table = new sc_process_table;
    m_current_writer = 0;


    // CHECK FOR ENVIRONMENT VARIABLES THAT MODIFY SIMULATOR EXECUTION:

    const char* write_check = std::getenv("SC_SIGNAL_WRITE_CHECK");
    m_write_check = ( (write_check==0) || strcmp(write_check,"DISABLE") ) ?
      true : false;


    // FINISH INITIALIZATIONS:

    reset_curr_proc();
    m_next_proc_id = -1;
    m_timed_events = new sc_ppq<sc_event_timed*>( 128, sc_notify_time_compare );
    m_something_to_trace = false;
    m_runnable = new sc_runnable;
    m_collectable = new sc_process_list;
    m_time_params = new sc_time_params;
    m_curr_time = SC_ZERO_TIME;
    m_max_time = SC_ZERO_TIME;
    m_change_stamp = 0;
    m_delta_count = 0;
    m_forced_stop = false;
    m_paused = false;
    m_ready_to_simulate = false;
    m_elaboration_done = false;
    m_execution_phase = phase_initialize;
    m_error = NULL;
    m_cor_pkg = 0;
    m_method_invoker_p = NULL;
    m_cor = 0;
    m_in_simulator_control = false;
    m_start_of_simulation_called = false;
    m_end_of_simulation_called = false;
    m_simulation_status = SC_ELABORATION;
}

void
sc_simcontext::clean()
{
    delete m_object_manager;
    delete m_module_registry;
    delete m_port_registry;
    delete m_export_registry;
    delete m_prim_channel_registry;
    delete m_phase_cb_registry;
    delete m_name_gen;
    delete m_process_table;
    m_child_objects.resize(0);
    m_delta_events.resize(0);
    delete m_timed_events;
    for( int i = m_trace_files.size() - 1; i >= 0; -- i ) {
	delete m_trace_files[i];
    }
    m_trace_files.resize(0);
    delete m_runnable;
    delete m_collectable;
    delete m_time_params;
    delete m_cor_pkg;
    delete m_error;
}


sc_simcontext::sc_simcontext() :
    m_object_manager(0), m_module_registry(0), m_port_registry(0),
    m_export_registry(0), m_prim_channel_registry(0),
    m_phase_cb_registry(0), m_name_gen(0),
    m_process_table(0), m_curr_proc_info(), m_current_writer(0),
    m_write_check(false), m_next_proc_id(-1), m_child_events(),
    m_child_objects(), m_delta_events(), m_timed_events(0), m_trace_files(),
    m_something_to_trace(false), m_runnable(0), m_collectable(0), 
    m_time_params(), m_curr_time(SC_ZERO_TIME), m_max_time(SC_ZERO_TIME), 
    m_change_stamp(0), m_delta_count(0), m_forced_stop(false), m_paused(false),
    m_ready_to_simulate(false), m_elaboration_done(false),
    m_execution_phase(phase_initialize), m_error(0),
    m_in_simulator_control(false), m_end_of_simulation_called(false),
    m_simulation_status(SC_ELABORATION), m_start_of_simulation_called(false),
    m_cor_pkg(0), m_cor(0)
{
    init();
}

sc_simcontext::~sc_simcontext()
{
    clean();
}

// +----------------------------------------------------------------------------
// |"sc_simcontext::active_object"
// | 
// | This method returns the currently active object with respect to 
// | additions to the hierarchy. It will be the top of the object hierarchy
// | stack if it is non-empty, or it will be the active process, or NULL 
// | if there is no active process.
// +----------------------------------------------------------------------------
sc_object*
sc_simcontext::active_object() 
{
    sc_object* result_p; // pointer to return.

    result_p = m_object_manager->hierarchy_curr();
    if ( !result_p )
        result_p = (sc_object*)get_curr_proc_info()->process_handle;
    return result_p;
}

// +----------------------------------------------------------------------------
// |"sc_simcontext::crunch"
// | 
// | This method implements the simulator's execution of processes. It performs
// | one or more "delta" cycles. Each delta cycle consists of an evaluation,
// | an update phase, and a notification phase. During the evaluation phase any 
// | processes that are ready to run are executed. After all the processes have
// | been executed the update phase is entered. During the update phase the 
// | values of any signals that have changed are updated. After the updates
// | have been performed the notification phase is entered. During that phase
// | any notifications that need to occur because of of signal values changes
// | are performed. This will result in the queueing of processes for execution
// | that are sensitive to those notifications. At that point a delta cycle
// | is complete, and the process is started again unless 'once' is true.
// |
// | Arguments:
// |     once = true if only one delta cycle is to be performed.
// +----------------------------------------------------------------------------
inline void
sc_simcontext::crunch( bool once )
{
#ifdef DEBUG_SYSTEMC
    int num_deltas = 0;  // number of delta cycles
#endif

    while ( true ) 
    {

	// EVALUATE PHASE
	
	m_execution_phase = phase_evaluate;
	bool empty_eval_phase = true;
	while( true ) 
	{

	    // execute method processes

	    m_runnable->toggle_methods();
	    sc_method_handle method_h = pop_runnable_method();
	    while( method_h != 0 ) {
		empty_eval_phase = false;
		if ( !method_h->run_process() )
		{
		    goto out;
		}
		method_h = pop_runnable_method();
	    }

	    // execute (c)thread processes

	    m_runnable->toggle_threads();
	    sc_thread_handle thread_h = pop_runnable_thread();
	    while( thread_h != 0 ) {
                if ( thread_h->m_cor_p != NULL ) break;
		thread_h = pop_runnable_thread();
	    }

	    if( thread_h != 0 ) {
	        empty_eval_phase = false;
		m_cor_pkg->yield( thread_h->m_cor_p );
	    }
	    if( m_error ) {
		goto out;
	    }

	    // check for call(s) to sc_stop
	    if( m_forced_stop ) {
		if ( stop_mode == SC_STOP_IMMEDIATE ) goto out;
	    }

	    // no more runnable processes

	    if( m_runnable->is_empty() ) {
		break;
	    }
	}

	// remove finally dead zombies:

        while( ! m_collectable->empty() )
        {
	    sc_process_b* del_p = m_collectable->front();
	    m_collectable->pop_front();
	    del_p->reference_decrement();
        }


	// UPDATE PHASE
	//
	// The change stamp must be updated first so that event_occurred()
	// will work.

	m_execution_phase = phase_update;
	if ( !empty_eval_phase ) 
	{
//	    SC_DO_PHASE_CALLBACK_(evaluation_done);
	    m_change_stamp++;
	    m_delta_count ++;
	}
	m_prim_channel_registry->perform_update();
	SC_DO_PHASE_CALLBACK_(update_done);
	m_execution_phase = phase_notify;

#if SC_SIMCONTEXT_TRACING_
	if( m_something_to_trace ) {
	    trace_cycle( /* delta cycle? */ true );
	}
#endif

        // check for call(s) to sc_stop
        if( m_forced_stop ) {
            break;
        }

#ifdef DEBUG_SYSTEMC
        // check for possible infinite loops
        if( ++ num_deltas > SC_MAX_NUM_DELTA_CYCLES ) {
	    ::std::cerr << "SystemC warning: "
		 << "the number of delta cycles exceeds the limit of "
		 << SC_MAX_NUM_DELTA_CYCLES
		 << ", defined in sc_constants.h.\n"
		 << "This is a possible sign of an infinite loop.\n"
		 << "Increase the limit if this warning is invalid.\n";
	    break;
	}
#endif

	// NOTIFICATION PHASE:
	//
	// Process delta notifications which will queue processes for 
	// subsequent execution.

        int size = m_delta_events.size();
	if ( size != 0 )
	{
	    sc_event** l_events = &m_delta_events[0];
	    int i = size - 1;
	    do {
		l_events[i]->trigger();
	    } while( -- i >= 0 );
	    m_delta_events.resize(0);
	}

	if( m_runnable->is_empty() ) {
	    // no more runnable processes
	    break;
	}

	// if sc_pause() was called we are done.

	if ( m_paused ) break;

        // IF ONLY DOING ONE CYCLE, WE ARE DONE. OTHERWISE EXECUTE NEW CALLBACKS

        if ( once ) break;
    }

    // When this point is reached the processing of delta cycles is complete,
    // if the completion was because of an error throw the exception specified
    // by '*m_error'.
out:
    this->reset_curr_proc();
    if( m_error ) throw *m_error; // re-throw propagated error
}

inline
void
sc_simcontext::cycle( const sc_time& t)
{
    sc_time next_event_time;

    m_in_simulator_control = true;
    crunch(); 
    SC_DO_PHASE_CALLBACK_(before_timestep);
#if SC_SIMCONTEXT_TRACING_
    if( m_something_to_trace ) {
        trace_cycle( /* delta cycle? */ false );
    }
#endif
    m_curr_time += t;
    if ( next_time(next_event_time) && next_event_time <= m_curr_time) {
        SC_REPORT_WARNING(SC_ID_CYCLE_MISSES_EVENTS_, "");
    }
    m_in_simulator_control = false;
    SC_DO_PHASE_CALLBACK_(simulation_paused);
}

void
sc_simcontext::elaborate()
{
    if( m_elaboration_done || sim_status() != SC_SIM_OK ) {
        return;
    }

    // Instantiate the method invocation module
    // (not added to public object hierarchy)

    m_method_invoker_p =
      new sc_invoke_method("$$$$kernel_module$$$$_invoke_method" );

    m_simulation_status = SC_BEFORE_END_OF_ELABORATION;
    for( int cd = 0; cd != 4; /* empty */ )
    {
        cd  = m_port_registry->construction_done();
        cd += m_export_registry->construction_done();
        cd += m_prim_channel_registry->construction_done();
        cd += m_module_registry->construction_done();

        // check for call(s) to sc_stop
        if( m_forced_stop ) {
            do_sc_stop_action();
            return;
        }

    }
    SC_DO_PHASE_CALLBACK_(construction_done);

    // SIGNAL THAT ELABORATION IS DONE
    //
    // We set the switch before the calls in case someone creates a process 
    // in an end_of_elaboration callback. We need the information to flag 
    // the process as being dynamic.

    m_elaboration_done = true;
    m_simulation_status = SC_END_OF_ELABORATION;

    m_port_registry->elaboration_done();
    m_export_registry->elaboration_done();
    m_prim_channel_registry->elaboration_done();
    m_module_registry->elaboration_done();
    SC_DO_PHASE_CALLBACK_(elaboration_done);
    sc_reset::reconcile_resets();

    // check for call(s) to sc_stop
    if( m_forced_stop ) {
        do_sc_stop_action();
        return;
    }
}

void
sc_simcontext::prepare_to_simulate()
{
    sc_method_handle  method_p;  // Pointer to method process accessing.
    sc_thread_handle  thread_p;  // Pointer to thread process accessing.

    if( m_ready_to_simulate || sim_status() != SC_SIM_OK ) {
        return;
    }

    // instantiate the coroutine package
    m_cor_pkg = new sc_cor_pkg_t( this );
    m_cor = m_cor_pkg->get_main();

    // NOTIFY ALL OBJECTS THAT SIMULATION IS ABOUT TO START:

    m_simulation_status = SC_START_OF_SIMULATION;
    m_port_registry->start_simulation();
    m_export_registry->start_simulation();
    m_prim_channel_registry->start_simulation();
    m_module_registry->start_simulation();
    SC_DO_PHASE_CALLBACK_(start_simulation);
    m_start_of_simulation_called = true;

    // CHECK FOR CALL(S) TO sc_stop 

    if( m_forced_stop ) {
        do_sc_stop_action();
        return;
    }

    // PREPARE ALL (C)THREAD PROCESSES FOR SIMULATION:

    for ( thread_p = m_process_table->thread_q_head(); 
	  thread_p; thread_p = thread_p->next_exist() )
    {
	thread_p->prepare_for_simulation();
    }

    m_simulation_status = SC_RUNNING;
    m_ready_to_simulate = true;
    m_runnable->init();

    // update phase

    m_execution_phase = phase_update;
    m_prim_channel_registry->perform_update();
    m_execution_phase = phase_notify;

    int size;

    // make all method processes runnable

    for ( method_p = m_process_table->method_q_head(); 
	  method_p; method_p = method_p->next_exist() )
    {
	if ( ((method_p->m_state & sc_process_b::ps_bit_disabled) != 0) ||
	     method_p->dont_initialize() ) 
	{
	    if ( method_p->m_static_events.size() == 0 )
	    {
	        SC_REPORT_WARNING( SC_ID_DISABLE_WILL_ORPHAN_PROCESS_, 
		                   method_p->name() );
	    }
	}
	else if ( (method_p->m_state & sc_process_b::ps_bit_suspended) == 0) 
	{
	    push_runnable_method_front( method_p );
        }
	else
	{
	    method_p->m_state |= sc_process_b::ps_bit_ready_to_run;
	}
    }

    // make thread processes runnable
    // (cthread processes always have the dont_initialize flag set)

    for ( thread_p = m_process_table->thread_q_head(); 
	  thread_p; thread_p = thread_p->next_exist() )
    {
	if ( ((thread_p->m_state & sc_process_b::ps_bit_disabled) != 0) || 
	     thread_p->dont_initialize() ) 
	{
	    if ( thread_p->m_static_events.size() == 0 )
	    {
	        SC_REPORT_WARNING( SC_ID_DISABLE_WILL_ORPHAN_PROCESS_, 
		                   thread_p->name() );
	    }
	}
	else if ( (thread_p->m_state & sc_process_b::ps_bit_suspended) == 0) 
	{
            push_runnable_thread_front( thread_p );
        }
	else
	{
	    thread_p->m_state |= sc_process_b::ps_bit_ready_to_run;
	}
    }


    // process delta notifications

    if( ( size = m_delta_events.size() ) != 0 ) {
        sc_event** l_delta_events = &m_delta_events[0];
        int i = size - 1;
        do {
            l_delta_events[i]->trigger();
        } while( -- i >= 0 );
        m_delta_events.resize(0);
    }

    SC_DO_PHASE_CALLBACK_(initialization_done);
}

void
sc_simcontext::initial_crunch( bool no_crunch )
{
    if( no_crunch || m_runnable->is_empty() ) {
        return;
    }

    // run the delta cycle loop

    crunch();
    if( m_error ) {
        return;
    }

#if SC_SIMCONTEXT_TRACING_
    if( m_something_to_trace ) {
        trace_cycle( false );
    }
#endif

    // check for call(s) to sc_stop
    if( m_forced_stop ) {
        do_sc_stop_action();
    }
}

void
sc_simcontext::initialize( bool no_crunch )
{
    m_in_simulator_control = true;
    elaborate();

    prepare_to_simulate();
    initial_crunch(no_crunch);
    m_in_simulator_control = false;
}

// +----------------------------------------------------------------------------
// |"sc_simcontext::simulate"
// | 
// | This method runs the simulation for the specified amount of time.
// |
// | Notes:
// |   (1) This code always run with an SC_EXIT_ON_STARVATION starvation policy,
// |       so the simulation time on return will be the minimum of the 
// |       simulation on entry plus the duration, and the maximum time of any 
// |       event present in the simulation. If the simulation policy is
// |       SC_RUN_TO_TIME starvation it is implemented by the caller of this 
// |       method, e.g., sc_start(), by artificially setting the simulation
// |       time forward after this method completes.
// |
// | Arguments:
// |     duration = amount of time to simulate.
// +----------------------------------------------------------------------------
void
sc_simcontext::simulate( const sc_time& duration )
{
    initialize( true );

    if (sim_status() != SC_SIM_OK) {
	return;
    }

    sc_time non_overflow_time = sc_max_time() - m_curr_time;
    if ( duration > non_overflow_time )
    {
	SC_REPORT_ERROR(SC_ID_SIMULATION_TIME_OVERFLOW_, "");
	return;
    }
    else if ( duration < SC_ZERO_TIME )
    {
        SC_REPORT_ERROR(SC_ID_NEGATIVE_SIMULATION_TIME_,"");
    }

    m_in_simulator_control = true;
    m_paused = false;

    sc_time until_t = m_curr_time + duration;
    sc_time t;            // current simulaton time.

    // IF DURATION WAS ZERO WE ONLY CRUNCH ONCE:
    //
    // We duplicate the code so that we don't add the overhead of the
    // check to each loop in the do below.
    if ( duration == SC_ZERO_TIME ) 
    {
	m_in_simulator_control = true;
  	crunch( true );
	if( m_error ) {
	    m_in_simulator_control = false;
	    return;
	}
#if SC_SIMCONTEXT_TRACING_
        if( m_something_to_trace )
            trace_cycle( /* delta cycle? */ false );
#endif
        if( m_forced_stop ) {
            do_sc_stop_action();
            return;
        }
        // return via implicit pause
        goto exit_pause;
    }

    // NON-ZERO DURATION: EXECUTE UP TO THAT TIME, OR UNTIL EVENT STARVATION:

    do {

	crunch();
	if( m_error ) {
	    m_in_simulator_control = false;
	    return;
	}
#if SC_SIMCONTEXT_TRACING_
	if( m_something_to_trace ) {
	    trace_cycle( false );
	}
#endif
        // check for call(s) to sc_stop() or sc_pause().
        if( m_forced_stop ) {
            do_sc_stop_action();
            return;
        }
        if( m_paused ) goto exit_pause; // return explicit pause

	t = m_curr_time; 

	do {
	    // See note 1 above:

            if ( !next_time(t) || (t > until_t ) ) goto exit_time;
	    if ( t > m_curr_time ) 
	    {
		SC_DO_PHASE_CALLBACK_(before_timestep);
		m_curr_time = t;
		m_change_stamp++;
	    }

	    // PROCESS TIMED NOTIFICATIONS AT THE CURRENT TIME

	    do {
		sc_event_timed* et = m_timed_events->extract_top();
		sc_event* e = et->event();
		delete et;
		if( e != 0 ) {
		    e->trigger();
		}
	    } while( m_timed_events->size() &&
		     m_timed_events->top()->notify_time() == t );

	} while( m_runnable->is_empty() );
    } while ( t < until_t ); // hold off on the delta for the until_t time.

exit_time:  // final simulation time update, if needed
    if ( t > m_curr_time && t <= until_t ) 
    {
        SC_DO_PHASE_CALLBACK_(before_timestep);
        m_curr_time = t;
        m_change_stamp++;
    }
exit_pause: // call pause callback upon implicit or explicit pause
    m_execution_phase      = phase_evaluate;
    m_in_simulator_control = false;
    SC_DO_PHASE_CALLBACK_(simulation_paused);
}

void
sc_simcontext::do_sc_stop_action()
{
    SC_REPORT_INFO("/OSCI/SystemC","Simulation stopped by user.");
    if (m_start_of_simulation_called) {
	end();
	m_in_simulator_control = false;
    }
    m_simulation_status = SC_STOPPED;
    SC_DO_PHASE_CALLBACK_(simulation_stopped);
}

void
sc_simcontext::mark_to_collect_process( sc_process_b* zombie )
{
    m_collectable->push_back( zombie );
}


//------------------------------------------------------------------------------
//"sc_simcontext::stop"
//
// This method stops the simulator after some amount of further processing.
// How much processing is done depends upon the value of the global variable
// stop_mode:
//     SC_STOP_IMMEDIATE - aborts the execution phase of the current delta
//                         cycle and performs whatever updates are pending.
//     SC_STOP_FINISH_DELTA - finishes the current delta cycle - both execution
//                            and updates.
// If sc_stop is called outside of the purview of the simulator kernel 
// (e.g., directly from sc_main), the end of simulation notifications 
// are performed. From within the purview of the simulator kernel, these
// will be performed at a later time.
//------------------------------------------------------------------------------

void
sc_simcontext::stop()
{
    static bool stop_warning_issued = false;
    if (m_forced_stop)
    {
        if ( !stop_warning_issued )
        {
            stop_warning_issued = true; // This must be before the WARNING!!!
            SC_REPORT_WARNING(SC_ID_SIMULATION_STOP_CALLED_TWICE_, "");
        }
        return;
    }
    if ( stop_mode == SC_STOP_IMMEDIATE ) m_runnable->init();
    m_forced_stop = true;
    if ( !m_in_simulator_control  )
    {
        do_sc_stop_action();
    } 
}

void
sc_simcontext::reset()
{
    clean();
    init();
}

void
sc_simcontext::end()
{
    m_simulation_status = SC_END_OF_SIMULATION;
    m_ready_to_simulate = false;
    m_port_registry->simulation_done();
    m_export_registry->simulation_done();
    m_prim_channel_registry->simulation_done();
    m_module_registry->simulation_done();
    SC_DO_PHASE_CALLBACK_(simulation_done);
    m_end_of_simulation_called = true;
}

void
sc_simcontext::hierarchy_push( sc_module* mod )
{
    m_object_manager->hierarchy_push( mod );
}

sc_module*
sc_simcontext::hierarchy_pop()
{
	return static_cast<sc_module*>( m_object_manager->hierarchy_pop() );
}

sc_module*
sc_simcontext::hierarchy_curr() const
{
    return static_cast<sc_module*>( m_object_manager->hierarchy_curr() );
}
    
sc_object*
sc_simcontext::first_object()
{
    return m_object_manager->first_object();
}

sc_object*
sc_simcontext::next_object()
{
    return m_object_manager->next_object();
}

sc_object*
sc_simcontext::find_object( const char* name )
{
    static bool warn_find_object=true;
    if ( warn_find_object )
    {
	warn_find_object = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_simcontext::find_object() is deprecated,\n" \
            " use sc_find_object()" );
    }
    return m_object_manager->find_object( name );
}

// to generate unique names for objects in an MT-Safe way

const char*
sc_simcontext::gen_unique_name( const char* basename_, bool preserve_first )
{
    return m_name_gen->gen_unique_name( basename_, preserve_first );
}


sc_process_handle 
sc_simcontext::create_cthread_process( 
    const char* name_p, bool free_host, SC_ENTRY_FUNC method_p,         
    sc_process_host* host_p, const sc_spawn_options* opt_p )
{
    sc_thread_handle handle = 
        new sc_cthread_process(name_p, free_host, method_p, host_p, opt_p);
    if ( m_ready_to_simulate ) 
    {
	handle->prepare_for_simulation();
    } else {
	m_process_table->push_front( handle );
    }
    return sc_process_handle(handle);
}


sc_process_handle 
sc_simcontext::create_method_process( 
    const char* name_p, bool free_host, SC_ENTRY_FUNC method_p,         
    sc_process_host* host_p, const sc_spawn_options* opt_p )
{
    sc_method_handle handle = 
        new sc_method_process(name_p, free_host, method_p, host_p, opt_p);
    if ( m_ready_to_simulate ) { // dynamic process
	if ( !handle->dont_initialize() )
        {
#ifdef SC_HAS_PHASE_CALLBACKS_
            if( SC_UNLIKELY_( m_simulation_status
                            & (SC_END_OF_UPDATE|SC_BEFORE_TIMESTEP) ) )
            {
                std::stringstream msg;
                msg << m_simulation_status 
                    << ":\n\t immediate method spawning of "
                       "`" << handle->name() << "' ignored";
                SC_REPORT_WARNING( SC_ID_PHASE_CALLBACK_FORBIDDEN_
                                 , msg.str().c_str() );
            }
            else
#endif // SC_HAS_PHASE_CALLBACKS_
            {
                push_runnable_method( handle );
            }
        }
        else if ( handle->m_static_events.size() == 0 )
        {
            SC_REPORT_WARNING( SC_ID_DISABLE_WILL_ORPHAN_PROCESS_,
                               handle->name() );
        }

    } else {
	m_process_table->push_front( handle );
    }
    return sc_process_handle(handle);
}


sc_process_handle 
sc_simcontext::create_thread_process( 
    const char* name_p, bool free_host, SC_ENTRY_FUNC method_p,         
    sc_process_host* host_p, const sc_spawn_options* opt_p )
{
    sc_thread_handle handle = 
        new sc_thread_process(name_p, free_host, method_p, host_p, opt_p);
    if ( m_ready_to_simulate ) { // dynamic process
	handle->prepare_for_simulation();
        if ( !handle->dont_initialize() )
        {
#ifdef SC_HAS_PHASE_CALLBACKS_
            if( SC_UNLIKELY_( m_simulation_status
                            & (SC_END_OF_UPDATE|SC_BEFORE_TIMESTEP) ) )
            {
                std::stringstream msg;
                msg << m_simulation_status 
                    << ":\n\t immediate thread spawning of "
                       "`" << handle->name() << "' ignored";
                SC_REPORT_WARNING( SC_ID_PHASE_CALLBACK_FORBIDDEN_
                                 , msg.str().c_str() );
            }
            else
#endif // SC_HAS_PHASE_CALLBACKS_
            {
                push_runnable_thread( handle );
            }
        }
        else if ( handle->m_static_events.size() == 0 )
        {
            SC_REPORT_WARNING( SC_ID_DISABLE_WILL_ORPHAN_PROCESS_,
                               handle->name() );
        }

    } else {
	m_process_table->push_front( handle );
    }
    return sc_process_handle(handle);
}

void
sc_simcontext::add_trace_file( sc_trace_file* tf )
{
    m_trace_files.push_back( tf );
    m_something_to_trace = true;
}

void
sc_simcontext::remove_trace_file( sc_trace_file* tf )
{
    m_trace_files.erase(
        std::remove( m_trace_files.begin(), m_trace_files.end(), tf )
    );
    m_something_to_trace = ( m_trace_files.size() > 0 );
}

sc_cor*
sc_simcontext::next_cor()
{
    if( m_error ) {
	return m_cor;
    }
    
    sc_thread_handle thread_h = pop_runnable_thread();
    while( thread_h != 0 ) {
	if ( thread_h->m_cor_p != NULL ) break;
	thread_h = pop_runnable_thread();
    }
    
    if( thread_h != 0 ) {
	return thread_h->m_cor_p;
    } else {
	return m_cor;
    }
}

const ::std::vector<sc_object*>&
sc_simcontext::get_child_objects() const
{
    static bool warn_get_child_objects=true;
    if ( warn_get_child_objects )
    {
	warn_get_child_objects = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_simcontext::get_child_objects() is deprecated,\n" \
            " use sc_get_top_level_objects()" );
    }
    return m_child_objects;
}

void
sc_simcontext::add_child_event( sc_event* event_ )
{
    // no check if object_ is already in the set
    m_child_events.push_back( event_ );
}

void
sc_simcontext::add_child_object( sc_object* object_ )
{
    // no check if object_ is already in the set
    m_child_objects.push_back( object_ );
}

void
sc_simcontext::remove_child_event( sc_event* event_ )
{
    int size = m_child_events.size();
    for( int i = 0; i < size; ++ i ) {
	if( event_ == m_child_events[i] ) {
	    m_child_events[i] = m_child_events[size - 1];
	    m_child_events.resize(size-1);
	    return;
	}
    }
    // no check if event_ is really in the set
}

void
sc_simcontext::remove_child_object( sc_object* object_ )
{
    int size = m_child_objects.size();
    for( int i = 0; i < size; ++ i ) {
	if( object_ == m_child_objects[i] ) {
	    m_child_objects[i] = m_child_objects[size - 1];
	    m_child_objects.resize(size-1);
	    return;
	}
    }
    // no check if object_ is really in the set
}

sc_dt::uint64
sc_simcontext::delta_count() const
{
    static bool warn_delta_count=true;
    if ( warn_delta_count )
    {
	warn_delta_count = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_simcontext::delta_count() is deprecated, use sc_delta_count()" );
    }
    return m_delta_count;
}

bool
sc_simcontext::is_running() const
{
    static bool warn_is_running=true;
    if ( warn_is_running )
    {
	warn_is_running = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_simcontext::is_running() is deprecated, use sc_is_running()" );
    }
    return m_ready_to_simulate;
}

// +----------------------------------------------------------------------------
// |"sc_simcontext::next_time"
// | 
// | This method returns the time of the next event. If there are no events
// | it returns false.
// | 
// | Arguments:
// |     result = where to place time of the next event, if no event is 
// |              found this value will not be changed.
// | Result is true if an event is found, false if not.
// +----------------------------------------------------------------------------
bool
sc_simcontext::next_time( sc_time& result ) const
{
    while( m_timed_events->size() ) {
	sc_event_timed* et = m_timed_events->top();
	if( et->event() != 0 ) {
	    result = et->notify_time();
	    return true;
	}
	delete m_timed_events->extract_top();
    }
    return false;
}

void
sc_simcontext::remove_delta_event( sc_event* e )
{
    int i = e->m_delta_event_index;
    int j = m_delta_events.size() - 1;
    assert( i >= 0 && i <= j );
    if( i != j ) {
	sc_event** l_delta_events = &m_delta_events[0];
	l_delta_events[i] = l_delta_events[j];
	l_delta_events[i]->m_delta_event_index = i;
    }
    m_delta_events.resize(m_delta_events.size()-1);
    e->m_delta_event_index = -1;
}

// +----------------------------------------------------------------------------
// |"sc_simcontext::preempt_with"
// | 
// | This method executes the supplied method immediately, suspending the
// | caller. After executing the supplied method the caller's execution will
// | be restored. It is used to allow a method to immediately throw an 
// | exception, e.g., when the method's kill_process() method was called.
// | There are three cases to consider:
// |   (1) The caller is a method, e.g., murder by method.
// |   (2) The caller is a thread instance, e.g., murder by thread.
// |   (3) The caller is this method instance, e.g., suicide.
// |
// | Arguments:
// |     method_h -> method to be executed.
// +----------------------------------------------------------------------------
void 
sc_simcontext::preempt_with( sc_method_handle method_h )
{
    sc_curr_proc_info caller_info;     // process info for caller.
    sc_method_handle  active_method_h; // active method or null.
    sc_thread_handle  active_thread_h; // active thread or null.

    // Determine the active process and take the thread to be run off the
    // run queue, if its there, since we will be explicitly causing its 
    // execution.

    active_method_h = DCAST<sc_method_handle>(sc_get_current_process_b());
    active_thread_h = DCAST<sc_thread_handle>(sc_get_current_process_b());
    if ( method_h->next_runnable() != NULL )
	remove_runnable_method( method_h );

    // CALLER IS THE METHOD TO BE RUN:
    //
    // Should never get here, ignore it unless we are debugging.

    if ( method_h == active_method_h )
    {
        DEBUG_MSG(DEBUG_NAME,method_h,"self preemption of active method");
    }

    // THE CALLER IS A METHOD:
    //
    //   (a) Set the current process information to our method.
    //   (b) Invoke our method directly by-passing the run queue.
    //   (c) Restore the process info to the caller.
    //   (d) Check to see if the calling method should throw an exception
    //       because of activity that occurred during the preemption.

    else if ( active_method_h != NULL )
    {
	caller_info = m_curr_proc_info;
        DEBUG_MSG( DEBUG_NAME, method_h,
	           "preempting active method with method" );
	sc_get_curr_simcontext()->set_curr_proc( (sc_process_b*)method_h );
	method_h->run_process();
	sc_get_curr_simcontext()->set_curr_proc((sc_process_b*)active_method_h);
	active_method_h->check_for_throws();
    }

    // CALLER IS A THREAD:
    //
    //   (a) Use an invocation thread to execute the method.

    else if ( active_thread_h != NULL )
    {
        DEBUG_MSG( DEBUG_NAME, method_h,
	           "preempting active thread with method" );
	m_method_invoker_p->invoke_method(method_h);
    }

    // CALLER IS THE SIMULATOR:
    //
    // That is not allowed.

    else
    {
	caller_info = m_curr_proc_info;
        DEBUG_MSG( DEBUG_NAME, method_h,
	           "preempting no active process with method" );
	sc_get_curr_simcontext()->set_curr_proc( (sc_process_b*)method_h );
	method_h->run_process();
	m_curr_proc_info = caller_info;
    }
}

//------------------------------------------------------------------------------
//"sc_simcontext::requeue_current_process"
//
// This method requeues the current process at the beginning of the run queue
// if it is a thread. This is called by sc_process_handle::throw_it() to assure
// that a thread that is issuing a throw will execute immediately after the
// processes it notifies via the throw.
//------------------------------------------------------------------------------
void sc_simcontext::requeue_current_process()
{
    sc_thread_handle thread_p;
    thread_p = DCAST<sc_thread_handle>(get_curr_proc_info()->process_handle);
    if ( thread_p )
    {
	execute_thread_next( thread_p );
    }
}

//------------------------------------------------------------------------------
//"sc_simcontext::suspend_current_process"
//
// This method suspends the current process if it is a thread. This is called 
// by sc_process_handle::throw_it() to allow the processes that have received
// a throw to execute.
//------------------------------------------------------------------------------
void sc_simcontext::suspend_current_process()
{
    sc_thread_handle thread_p;
    thread_p = DCAST<sc_thread_handle>(get_curr_proc_info()->process_handle);
    if ( thread_p )
    {
	thread_p->suspend_me(); 
    }
}

void
sc_simcontext::trace_cycle( bool delta_cycle )
{
    int size;
    if( ( size = m_trace_files.size() ) != 0 ) {
	sc_trace_file** l_trace_files = &m_trace_files[0];
	int i = size - 1;
	do {
	    l_trace_files[i]->cycle( delta_cycle );
	} while( -- i >= 0 );
    }
}

// ----------------------------------------------------------------------------

#if 1
#ifdef PURIFY
	static sc_simcontext sc_default_global_context;
	sc_simcontext* sc_curr_simcontext = &sc_default_global_context;
#else
	sc_simcontext* sc_curr_simcontext = 0;
	sc_simcontext* sc_default_global_context = 0;
#endif
#else
// Not MT-safe!
static sc_simcontext* sc_curr_simcontext = 0;


sc_simcontext*
sc_get_curr_simcontext()
{
    if( sc_curr_simcontext == 0 ) {
#ifdef PURIFY
        static sc_simcontext sc_default_global_context;
        sc_curr_simcontext = &sc_default_global_context;
#else
        static sc_simcontext* sc_default_global_context = new sc_simcontext;
        sc_curr_simcontext = sc_default_global_context;
#endif
    }
    return sc_curr_simcontext;
}
#endif // 0

// Generates unique names within each module.

const char*
sc_gen_unique_name( const char* basename_, bool preserve_first )
{
    sc_simcontext* simc = sc_get_curr_simcontext();
    sc_module* curr_module = simc->hierarchy_curr();
    if( curr_module != 0 ) {
	return curr_module->gen_unique_name( basename_, preserve_first );
    } else {
        sc_process_b* curr_proc_p = sc_get_current_process_b();
	if ( curr_proc_p )
	{
	    return curr_proc_p->gen_unique_name( basename_, preserve_first );
	}
	else
	{
	    return simc->gen_unique_name( basename_, preserve_first );
	}
    }
}

// Get a handle for the current process
//
// Note that this method should not be called if the current process is
// in the act of being deleted, it will mess up the reference count management
// of sc_process_b instance the handle represents. Instead, use the a 
// pointer to the raw sc_process_b instance, which may be acquired via
// sc_get_current_process_b().

sc_process_handle
sc_get_current_process_handle()
{
    return ( sc_is_running() ) ?
	sc_process_handle(sc_get_current_process_b()) : 
	sc_get_last_created_process_handle();
}

// THE FOLLOWING FUNCTION IS DEPRECATED IN 2.1
sc_process_b*
sc_get_curr_process_handle()
{
    static bool warn=true;
    if ( warn )
    {
        warn = false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
       "sc_get_curr_process_handle deprecated use sc_get_current_process_handle"
       );
    }

    return sc_get_curr_simcontext()->get_curr_proc_info()->process_handle;
}

// Return indication if there are more processes to execute in this delta phase

bool
sc_simcontext::pending_activity_at_current_time() const
{
    return ( m_delta_events.size() != 0) ||
           ( m_runnable->is_initialized() && !m_runnable->is_empty() ) ||
           m_prim_channel_registry->pending_updates();
}

// Return time of next activity.

sc_time sc_time_to_pending_activity( const sc_simcontext* simc_p ) 
{
    // If there is an activity pending at the current time
    // return a delta of zero.
    
    sc_time result=SC_ZERO_TIME; // time of pending activity.

    if ( simc_p->pending_activity_at_current_time() )
    {
        return result;
    }

    // Any activity will take place in the future pick up the next event's time.

    else
    {
        result = simc_p->max_time();
        simc_p->next_time(result);
        result -= sc_time_stamp();
    }
    return result;
}

// Set the random seed for controlled randomization -- not yet implemented

void
sc_set_random_seed( unsigned int )
{
    SC_REPORT_WARNING( SC_ID_NOT_IMPLEMENTED_,
		       "void sc_set_random_seed( unsigned int )" );
}


// +----------------------------------------------------------------------------
// |"sc_start"
// | 
// | This function starts, or restarts, the execution of the simulator.
// |
// | Arguments:
// |     duration = the amount of time the simulator should execute.
// |     p        = event starvation policy.
// +----------------------------------------------------------------------------
void
sc_start( const sc_time& duration, sc_starvation_policy p )
{
    sc_simcontext* context_p;      // current simulation context.
    sc_time        entry_time;     // simulation time upon entry.
    sc_time        exit_time;      // simulation time to set upon exit.
    sc_dt::uint64  starting_delta; // delta count upon entry.
    int            status;         // current simulation status.

    // Set up based on the arguments passed to us:

    context_p = sc_get_curr_simcontext();
    starting_delta = sc_delta_count();
    entry_time = context_p->m_curr_time;
    if ( p == SC_RUN_TO_TIME )
        exit_time = context_p->m_curr_time + duration;

    // called with duration = SC_ZERO_TIME for the first time
    static bool init_delta_or_pending_updates =
         ( starting_delta == 0 && exit_time == SC_ZERO_TIME );

    // If the simulation status is bad issue the appropriate message:

    status = context_p->sim_status();
    if( status != SC_SIM_OK ) 
    {
        if ( status == SC_SIM_USER_STOP )
            SC_REPORT_ERROR(SC_ID_SIMULATION_START_AFTER_STOP_, "");
        if ( status == SC_SIM_ERROR )
            SC_REPORT_ERROR(SC_ID_SIMULATION_START_AFTER_ERROR_, "");
        return;
    }

    if ( context_p->m_prim_channel_registry->pending_updates() )
        init_delta_or_pending_updates = true;

    // If the simulation status is good perform the simulation:

    context_p->simulate( duration );

    // Re-check the status:

    status = context_p->sim_status();

    // Update the current time to the exit time if that is the starvation
    // policy:

    if ( p == SC_RUN_TO_TIME && !context_p->m_paused && status == SC_SIM_OK )
    {
        context_p->m_curr_time = exit_time;
    }

    // If there was no activity and the simulation clock did not move warn
    // the user, except if we're in a first sc_start(SC_ZERO_TIME) for
    // initialisation (only) or there have been pending updates:

    if ( !init_delta_or_pending_updates &&
         starting_delta == sc_delta_count() &&
         context_p->m_curr_time == entry_time &&
         status == SC_SIM_OK )
    {
        SC_REPORT_WARNING(SC_ID_NO_SC_START_ACTIVITY_, "");
    }

    // reset init/update flag for subsequent calls
    init_delta_or_pending_updates = false;
}

void
sc_start()  
{
    sc_start( sc_max_time() - sc_time_stamp(),
              SC_EXIT_ON_STARVATION );
}

// for backward compatibility with 1.0
#if 0
void
sc_start( double duration )  // in default time units
{
    static bool warn_sc_start=true;
    if ( warn_sc_start )
    {
	warn_sc_start = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_start(double) deprecated, use sc_start(sc_time) or sc_start()");
    }

    if( duration == -1 )  // simulate forever
    {
        sc_start( 
            sc_time(~sc_dt::UINT64_ZERO, false) - sc_time_stamp() );
    }
    else
    {
        sc_start( sc_time( duration, true ) );
    }
}
#endif // 

void
sc_stop()
{
    sc_get_curr_simcontext()->stop();
}


// The following function is deprecated in favor of sc_start(SC_ZERO_TIME):

void
sc_initialize()
{
    static bool warning_initialize = true;

    if ( warning_initialize )
    {
        warning_initialize = false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_initialize() is deprecated: use sc_start(SC_ZERO_TIME)" );
    }
    sc_get_curr_simcontext()->initialize();
}

// The following function has been deprecated in favor of sc_start(duration):

void
sc_cycle( const sc_time& duration )
{
    static bool warning_cycle = true;

    if ( warning_cycle )
    {
        warning_cycle = false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_cycle is deprecated: use sc_start(sc_time)" );
    }
    sc_get_curr_simcontext()->cycle( duration );
}

sc_event* sc_find_event( const char* name )
{
    return sc_get_curr_simcontext()->get_object_manager()->find_event( name );
}

sc_object* sc_find_object( const char* name )
{
    return sc_get_curr_simcontext()->get_object_manager()->find_object( name );
}


const sc_time&
sc_max_time()
{
    return sc_get_curr_simcontext()->max_time();
}

const sc_time&
sc_time_stamp()
{
    return sc_get_curr_simcontext()->time_stamp();
}

double
sc_simulation_time()
{
    static bool warn_simulation_time=true;
    if ( warn_simulation_time )
    {
        warn_simulation_time=false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_simulation_time() is deprecated use sc_time_stamp()" );
    }
    return sc_get_curr_simcontext()->time_stamp().to_default_time_units();
}

void
sc_defunct_process_function( sc_module* )
{
    // This function is pointed to by defunct sc_thread_process'es and
    // sc_cthread_process'es. In a correctly constructed world, this
    // function should never be called; hence the assert.
    assert( false );
}

//------------------------------------------------------------------------------
//"sc_set_stop_mode"
//
// This function sets the mode of operation when sc_stop() is called.
//     mode = SC_STOP_IMMEDIATE or SC_STOP_FINISH_DELTA.
//------------------------------------------------------------------------------
void sc_set_stop_mode(sc_stop_mode mode)
{
    if ( sc_is_running() )
    {
        SC_REPORT_ERROR(SC_ID_STOP_MODE_AFTER_START_,"");
    }
    else
    {
        switch( mode )
        {
          case SC_STOP_IMMEDIATE:
          case SC_STOP_FINISH_DELTA:
              stop_mode = mode;
              break;
          default:
              break;
        }
    }
}

sc_stop_mode
sc_get_stop_mode()
{
    return stop_mode;
}

bool sc_is_unwinding()
{ 
    return sc_get_current_process_handle().is_unwinding();
}

// The IEEE 1666 Standard for 2011 designates that the treatment of
// certain process control interactions as being "implementation dependent".
// These interactions are:
//   (1) What happens when a resume() call is performed on a disabled, 
//       suspended process.
//   (2) What happens when sync_reset_on() or sync_reset_off() is called
//       on a suspended process.
//   (3) What happens when the value specified in a reset_signal_is()
//       call changes value while a process is suspended.
//
// By default this Proof of Concept implementation reports an error
// for these interactions. However, the implementation also provides
// a non-error treatment. The non-error treatment for the interactions is:
//   (1) A resume() call performed on a disabled, suspended process will
//       mark the process as no longer suspended, and if it is capable
//       of execution (not waiting on any events) it will be placed on
//       the queue of runnable processes. See the state diagram below.
//   (2) A call to sync_reset_on() or sync_reset_off() will set or clear
//       the synchronous reset flag. Whether the process is in reset or
//       not will be determined when the process actually executes by
//       looking at the flag's value at that time.
//   (3) If a suspended process has a reset_signal_is() specification
//       the value of the reset variable at the time of its next execution 
//       will determine whether it is in reset or not.
//      
// TO GET THE NON-ERROR BEHAVIOR SET THE VARIABLE BELOW TO TRUE.
//
// This can be done in this source before you build the library, or you
// can use an assignment as the first statement in your sc_main() function:
//    sc_core::sc_allow_process_control_corners = true;

bool sc_allow_process_control_corners = false;

// The state transition diagram for the interaction of disable and suspend
// when sc_allow_process_control_corners is true is shown below:
//
// ......................................................................
// .         ENABLED                    .           DISABLED            .
// .                                    .                               .
// .                 +----------+    disable      +----------+          .
// .   +------------>|          |-------.-------->|          |          .
// .   |             | runnable |       .         | runnable |          .
// .   |     +-------|          |<------.---------|          |------+   .
// .   |     |       +----------+     enable      +----------+      |   .
// .   |     |          |    ^          .            |    ^         |   .
// .   |     |  suspend |    | resume   .    suspend |    | resume  |   .
// .   |     |          V    |          .            V    |         |   .
// .   |     |       +----------+    disable      +----------+      |   .
// .   |     |       | suspend  |-------.-------->| suspend  |      |   .
// . t |   r |       |          |       .         |          |      | r .
// . r |   u |       |  ready   |<------.---------|  ready   |      | u .
// . i |   n |       +----------+     enable      +----------+      | n .
// . g |   / |         ^                .                           | / .
// . g |   w |  trigger|                .                           | w .
// . e |   a |         |                .                           | a .
// . r |   i |       +----------+    disable      +----------+      | i .
// .   |   t |       | suspend  |-------.-------->| suspend  |      | t .
// .   |     |       |          |       .         |          |      |   .
// .   |     |       | waiting  |<------.---------| waiting  |      |   .
// .   |     |       +----------+     enable      +----------+      |   .
// .   |     |          |    ^          .            |    ^         |   .
// .   |     |  suspend |    | resume   .    suspend |    | resume  |   .
// .   |     |          V    |          .            V    |         |   .
// .   |     |       +----------+    disable      +----------+      |   .
// .   |     +------>|          |-------.-------->|          |      |   .
// .   |             | waiting  |       .         | waiting  |      |   .
// .   +-------------|          |<------.---------|          |<-----+   .
// .                 +----------+     enable      +----------+          .
// .                                    .                               .
// ......................................................................

// ----------------------------------------------------------------------------

static std::ostream&
print_status_expression( std::ostream& os, sc_status s );

// utility helper to print a simulation status
std::ostream& operator << ( std::ostream& os, sc_status s )
{
    // print primitive values
    switch(s)
    {
#   define PRINT_STATUS( Status ) \
      case Status: { os << #Status; } break

      PRINT_STATUS( SC_UNITIALIZED );
      PRINT_STATUS( SC_ELABORATION );
      PRINT_STATUS( SC_BEFORE_END_OF_ELABORATION );
      PRINT_STATUS( SC_END_OF_ELABORATION );
      PRINT_STATUS( SC_START_OF_SIMULATION );

      PRINT_STATUS( SC_RUNNING );
      PRINT_STATUS( SC_PAUSED );
      PRINT_STATUS( SC_STOPPED );
      PRINT_STATUS( SC_END_OF_SIMULATION );

      PRINT_STATUS( SC_END_OF_INITIALIZATION );
//      PRINT_STATUS( SC_END_OF_EVALUATION );
      PRINT_STATUS( SC_END_OF_UPDATE );
      PRINT_STATUS( SC_BEFORE_TIMESTEP );

      PRINT_STATUS( SC_STATUS_ANY );

#   undef PRINT_STATUS
    default:

      if( s & SC_STATUS_ANY ) // combination of status bits
        print_status_expression( os, s );
      else                    // invalid number, print hex value
        os << "0x" << std::hex << +s;
    }

    return os;
}

// pretty-print a combination of sc_status bits (i.e. a callback mask)
static std::ostream&
print_status_expression( std::ostream& os, sc_status s )
{
    std::vector<sc_status> bits;
    unsigned               is_set = SC_ELABORATION;

    // collect bits
    while( is_set <= SC_STATUS_LAST )
    {
        if( s & is_set )
            bits.push_back( (sc_status)is_set );
        is_set <<= 1;
    }
    if( s & ~SC_STATUS_ANY ) // remaining bits
        bits.push_back( (sc_status)( s & ~SC_STATUS_ANY ) );

    // print expression
    std::vector<sc_status>::size_type i=0, n=bits.size();
    if ( n>1 )
        os << "(";
    for( ; i<n-1; ++i )
        os << bits[i] << "|";
    os << bits[i];
    if ( n>1 )
        os << ")";
    return os;
}

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Ali Dasdan, Synopsys, Inc.
  Description of Modification: - Added sc_stop() detection into initial_crunch
                                 and crunch. This makes it possible to exit out
                                 of a combinational loop using sc_stop().

      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems 20 May 2003
  Description of Modification: - sc_stop mode
                               - phase callbacks

      Name, Affiliation, Date: Bishnupriya Bhattacharya, Cadence Design Systems,
                               25 August 2003
  Description of Modification: - support for dynamic process
                               - support for sc export registry
                               - new member methods elaborate(), 
				 prepare_to_simulate(), and initial_crunch()
				 that are invoked by initialize() in that order
                               - implement sc_get_last_created_process_handle() for use
                                 before simulation starts
                               - remove "set_curr_proc(handle)" from 
                                 register_method_process and 
                                 register_thread_process - led to bugs
                               
      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems 04 Sep 2003
  Description of Modification: - changed process existence structures to
				 linked lists to eliminate exponential 
				 execution problem with using sc_pvector.
 *****************************************************************************/
// $Log: sc_simcontext.cpp,v $
// Revision 1.37  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.36  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.35  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.34  2011/08/04 17:15:28  acg
//  Andy Goodrich: added documentation to crunch() routine.
//
// Revision 1.32  2011/07/24 11:16:36  acg
//  Philipp A. Hartmann: fix reference counting on deferred deletions of
//  processes.
//
// Revision 1.31  2011/07/01 18:49:07  acg
//  Andy Goodrich: moved pln() from sc_simcontext.cpp to sc_ver.cpp.
//
// Revision 1.30  2011/05/09 04:07:49  acg
//  Philipp A. Hartmann:
//    (1) Restore hierarchy in all phase callbacks.
//    (2) Ensure calls to before_end_of_elaboration.
//
// Revision 1.29  2011/04/08 22:39:09  acg
//  Andy Goodrich: moved method invocation code to sc_method.h so that the
//  details are hidden from sc_simcontext.
//
// Revision 1.28  2011/04/05 20:50:57  acg
//  Andy Goodrich:
//    (1) changes to make sure that event(), posedge() and negedge() only
//        return true if the clock has not moved.
//    (2) fixes for method self-resumes.
//    (3) added SC_PRERELEASE_VERSION
//    (4) removed kernel events from the object hierarchy, added
//        sc_hierarchy_name_exists().
//
// Revision 1.27  2011/04/05 06:14:15  acg
//  Andy Goodrich: fix typo.
//
// Revision 1.26  2011/04/05 06:03:32  acg
//  Philipp A. Hartmann: added code to set ready to run bit for a suspended
//  process that does not have dont_initialize specified at simulation
//  start up.
//
// Revision 1.25  2011/04/01 21:31:55  acg
//  Andy Goodrich: make sure processes suspended before the start of execution
//  don't get scheduled for initial execution.
//
// Revision 1.24  2011/03/28 13:02:52  acg
//  Andy Goodrich: Changes for disable() interactions.
//
// Revision 1.23  2011/03/12 21:07:51  acg
//  Andy Goodrich: changes to kernel generated event support.
//
// Revision 1.22  2011/03/07 17:38:43  acg
//  Andy Goodrich: tightening up of checks for undefined interaction between
//  synchronous reset and suspend.
//
// Revision 1.21  2011/03/06 19:57:11  acg
//  Andy Goodrich: refinements for the illegal suspend - synchronous reset
//  interaction.
//
// Revision 1.20  2011/03/06 15:58:50  acg
//  Andy Goodrich: added escape to turn off process control corner case
//  checks.
//
// Revision 1.19  2011/03/05 04:45:16  acg
//  Andy Goodrich: moved active process calculation to the sc_simcontext class.
//
// Revision 1.18  2011/03/05 01:39:21  acg
//  Andy Goodrich: changes for named events.
//
// Revision 1.17  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.16  2011/02/17 19:53:28  acg
//  Andy Goodrich: eliminated use of ready_to_run() as part of process control
//  simplification.
//
// Revision 1.15  2011/02/13 21:47:38  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.14  2011/02/11 13:25:24  acg
//  Andy Goodrich: Philipp A. Hartmann's changes:
//    (1) Removal of SC_CTHREAD method overloads.
//    (2) New exception processing code.
//
// Revision 1.13  2011/02/08 08:42:50  acg
//  Andy Goodrich: fix ordering of check for stopped versus paused.
//
// Revision 1.12  2011/02/07 19:17:20  acg
//  Andy Goodrich: changes for IEEE 1666 compatibility.
//
// Revision 1.11  2011/02/02 07:18:11  acg
//  Andy Goodrich: removed toggle() calls for the new crunch() toggle usage.
//
// Revision 1.10  2011/02/01 23:01:53  acg
//  Andy Goodrich: removed dead code.
//
// Revision 1.9  2011/02/01 21:11:59  acg
//  Andy Goodrich:
//  (1) Use of new toggle_methods() and toggle_threads() run queue methods
//      to make sure the thread run queue does not execute when allow preempt_me()
//      is called from an SC_METHOD.
//  (2) Use of execute_thread_next() to allow thread execution in the current
//      delta cycle() rather than push_runnable_thread_front which executed
//      in the following cycle.
//
// Revision 1.8  2011/01/25 20:50:37  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.7  2011/01/19 23:21:50  acg
//  Andy Goodrich: changes for IEEE 1666 2011
//
// Revision 1.6  2011/01/18 20:10:45  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.5  2010/11/20 17:10:57  acg
//  Andy Goodrich: reset processing changes for new IEEE 1666 standard.
//
// Revision 1.4  2010/07/22 20:02:33  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.3  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.2  2007/09/20 20:32:35  acg
//  Andy Goodrich: changes to the semantics of throw_it() to match the
//  specification. A call to throw_it() will immediately suspend the calling
//  thread until all the throwees have executed. At that point the calling
//  thread will be restarted before the execution of any other threads.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.21  2006/08/29 23:37:13  acg
//  Andy Goodrich: Added check for negative time.
//
// Revision 1.20  2006/05/26 20:33:16  acg
//   Andy Goodrich: changes required by additional platform compilers (i.e.,
//   Microsoft VC++, Sun Forte, HP aCC).
//
// Revision 1.19  2006/05/08 17:59:52  acg
//  Andy Goodrich: added a check before m_curr_time is set to make sure it
//  is not set to a time before its current value. This will treat
//  sc_event.notify( ) calls with negative times as calls with a zero time.
//
// Revision 1.18  2006/04/20 17:08:17  acg
//  Andy Goodrich: 3.0 style process changes.
//
// Revision 1.17  2006/04/11 23:13:21  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.16  2006/03/21 00:00:34  acg
//   Andy Goodrich: changed name of sc_get_current_process_base() to be
//   sc_get_current_process_b() since its returning an sc_process_b instance.
//
// Revision 1.15  2006/03/13 20:26:50  acg
//  Andy Goodrich: Addition of forward class declarations, e.g.,
//  sc_reset, to keep gcc 4.x happy.
//
// Revision 1.14  2006/02/02 23:42:41  acg
//  Andy Goodrich: implemented a much better fix to the sc_event_finder
//  proliferation problem. This new version allocates only a single event
//  finder for each port for each type of event, e.g., pos(), neg(), and
//  value_change(). The event finder persists as long as the port does,
//  which is what the LRM dictates. Because only a single instance is
//  allocated for each event type per port there is not a potential
//  explosion of storage as was true in the 2.0.1/2.1 versions.
//
// Revision 1.13  2006/02/02 21:29:10  acg
//  Andy Goodrich: removed the call to sc_event_finder::free_instances() that
//  was in end_of_elaboration(), leaving only the call in clean(). This is
//  because the LRM states that sc_event_finder instances are persistent as
//  long as the sc_module hierarchy is valid.
//
// Revision 1.12  2006/02/02 21:09:50  acg
//  Andy Goodrich: added call to sc_event_finder::free_instances in the clean()
//  method.
//
// Revision 1.11  2006/02/02 20:43:14  acg
//  Andy Goodrich: Added an existence linked list to sc_event_finder so that
//  the dynamically allocated instances can be freed after port binding
//  completes. This replaces the individual deletions in ~sc_bind_ef, as these
//  caused an exception if an sc_event_finder instance was used more than
//  once, due to a double freeing of the instance.
//
// Revision 1.10  2006/01/31 21:43:26  acg
//  Andy Goodrich: added comments in constructor to highlight environmental
//  overrides section.
//
// Revision 1.9  2006/01/26 21:04:54  acg
//  Andy Goodrich: deprecation message changes and additional messages.
//
// Revision 1.8  2006/01/25 00:31:19  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.7  2006/01/24 20:49:05  acg
// Andy Goodrich: changes to remove the use of deprecated features within the
// simulator, and to issue warning messages when deprecated features are used.
//
// Revision 1.6  2006/01/19 00:29:52  acg
// Andy Goodrich: Yet another implementation for signal write checking. This
// one uses an environment variable SC_SIGNAL_WRITE_CHECK, that when set to
// DISABLE will disable write checking on signals.
//
// Revision 1.5  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//
// Revision 1.4  2006/01/03 23:18:44  acg
// Changed copyright to include 2006.
//
// Revision 1.3  2005/12/20 22:11:10  acg
// Fixed $Log lines.
//
// Revision 1.2  2005/12/20 22:02:30  acg
// Changed where delta cycles are incremented to match IEEE 1666. Added the
// event_occurred() method to hide how delta cycle comparisions are done within
// sc_simcontext. Changed the boolean update_phase to an enum that shows all
// the phases.
// Taf!

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

  sc_cor_pthread.cpp -- Coroutine implementation with pthreads.

  Original Author: Andy Goodrich, Forte Design Systems, 2002-11-10

 CHANGE LOG APPEARS AT THE END OF THE FILE
 *****************************************************************************/

#if !defined(_WIN32) && !defined(WIN32) && defined(SC_USE_PTHREADS)

// ORDER OF THE INCLUDES AND namespace sc_core IS IMPORTANT!!!

#include "sysc/kernel/sc_cor_pthread.h"
#include "sysc/kernel/sc_simcontext.h"

using namespace std;

namespace sc_core {

// MAKE SURE WE HAVE A NULL THAT WILL WORK:

#if defined(__hpux)
#   define PTHREAD_NULL cma_c_null
#else  // !defined(__hpux)
#   define PTHREAD_NULL NULL
#endif // !defined(__hpux)

#define DEBUGF \
    if (0) std::cout << "sc_cor_pthread.cpp(" << __LINE__ << ") "

// ----------------------------------------------------------------------------
//  File static variables.
//
// (1) The thread creation mutex and the creation condition are used to
//     suspend the thread creating another one until the created thread
//     reaches its invoke_module_method. This allows us to get control of
//     thread scheduling away from the pthread package.
// ----------------------------------------------------------------------------

static sc_cor_pthread* active_cor_p=0;   // Active co-routine.
static pthread_cond_t  create_condition; // See note 1 above.
static pthread_mutex_t create_mutex;     // See note 1 above.
static sc_cor_pthread  main_cor;         // Main coroutine.


// ----------------------------------------------------------------------------
//  CLASS : sc_cor_pthread
//
//  Coroutine class implemented with Posix Threads.
// ----------------------------------------------------------------------------

// constructor

sc_cor_pthread::sc_cor_pthread()
    : m_cor_fn_arg( 0 ), m_pkg_p( 0 )
{
    DEBUGF << this << ": sc_cor_pthread::sc_cor_pthread()" << std::endl;
    pthread_cond_init( &m_pt_condition, PTHREAD_NULL );
    pthread_mutex_init( &m_mutex, PTHREAD_NULL );
}


// destructor

sc_cor_pthread::~sc_cor_pthread()
{
    DEBUGF << this << ": sc_cor_pthread::~sc_cor_pthread()" << std::endl;
	pthread_cond_destroy( &m_pt_condition);
	pthread_mutex_destroy( &m_mutex );
}


// This static method is a Posix Threads helper callback and invokes a thread
// for the first time. It performs some synchronization and then invokes the
// actual sc_cor helper function.
//     context_p -> thread to invoke module method of.
// Result is 0 and ignored.

void* sc_cor_pthread::invoke_module_method(void* context_p)
{
    sc_cor_pthread* p = (sc_cor_pthread*)context_p;
    DEBUGF << p << ": sc_cor_pthread::invoke_module_method()" << std::endl;


    // SUSPEND THE THREAD SO WE CAN GAIN CONTROL FROM THE PTHREAD PACKAGE:
    //
    // Since pthread_create schedules each thread behind our back for its
    // initial execution we immediately suspend a newly created thread
    // here so we can control when its execution will occur. We also wake
    // up the main thread which is waiting for this thread to execute to this
    // wait point.

    pthread_mutex_lock( &create_mutex );
	DEBUGF << p << ": child signalling main thread " << endl;
    pthread_cond_signal( &create_condition );
    pthread_mutex_lock( &p->m_mutex );
    pthread_mutex_unlock( &create_mutex );
    pthread_cond_wait( &p->m_pt_condition, &p->m_mutex );
    pthread_mutex_unlock( &p->m_mutex );


    // CALL THE SYSTEMC CODE THAT WILL ACTUALLY START THE THREAD OFF:

    active_cor_p = p;
    DEBUGF << p << ": about to invoke real method " 
	   << active_cor_p << std::endl;
    (p->m_cor_fn)(p->m_cor_fn_arg);

    return 0;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_cor_pkg_pthread
//
//  Coroutine package class implemented with Posix Threads.
// ----------------------------------------------------------------------------

int sc_cor_pkg_pthread::instance_count = 0;


// constructor

sc_cor_pkg_pthread::sc_cor_pkg_pthread( sc_simcontext* simc )
: sc_cor_pkg( simc )
{
    // initialize the current coroutine
    if( ++ instance_count == 1 )
    {
        pthread_cond_init( &create_condition, PTHREAD_NULL );
        pthread_mutex_init( &create_mutex, PTHREAD_NULL );
        assert( active_cor_p == 0 );
        main_cor.m_pkg_p = this;
		DEBUGF << &main_cor << ": is main co-routine" << std::endl;
        active_cor_p = &main_cor;
    }
}


// destructor

sc_cor_pkg_pthread::~sc_cor_pkg_pthread()
{
    if( -- instance_count == 0 ) {
        // cleanup the main coroutine
    }
}


// create a new coroutine

sc_cor*
sc_cor_pkg_pthread::create( std::size_t stack_size, sc_cor_fn* fn, void* arg )
{
    sc_cor_pthread* cor_p = new sc_cor_pthread;
    DEBUGF << &main_cor << ": sc_cor_pkg_pthread::create(" 
	       << cor_p << ")" << std::endl;


    // INITIALIZE OBJECT'S FIELDS FROM ARGUMENT LIST:

    cor_p->m_pkg_p = this;
    cor_p->m_cor_fn = fn;
    cor_p->m_cor_fn_arg = arg;


	// SET UP THREAD CREATION ATTRIBUTES:
	//
	// Use default values except for stack size. If stack size is non-zero
	// set it.

    pthread_attr_t attr;
	pthread_attr_init( &attr ); 
	if ( stack_size != 0 )
	{
		pthread_attr_setstacksize( &attr, stack_size );
	}


    // ALLOCATE THE POSIX THREAD TO USE AND FORCE SEQUENTIAL EXECUTION:
    //
    // Because pthread_create causes the created thread to be executed,
    // we need to let it run until we can block in the invoke_module_method.
    // So we:
    //   (1) Lock the creation mutex before creating the new thread.
    //   (2) Sleep on the creation condition, which will be signalled by
    //       the newly created thread just before it goes to sleep in
    //       invoke_module_method.
    // This scheme results in the newly created thread being dormant before
    // the main thread continues execution.

    pthread_mutex_lock( &create_mutex );
    DEBUGF << &main_cor << ": about to create actual thread " 
	       << cor_p << std::endl;
    if ( pthread_create( &cor_p->m_thread, &attr,
             &sc_cor_pthread::invoke_module_method, (void*)cor_p ) )
    {
        std::fprintf(stderr, "ERROR - could not create thread\n");
    }

    DEBUGF << &main_cor << ": main thread waiting for signal from " 
	       << cor_p << std::endl;
    pthread_cond_wait( &create_condition, &create_mutex );
	DEBUGF << &main_cor << ": main thread signaled by " 
	       << cor_p << endl;
	pthread_attr_destroy( &attr ); 
    pthread_mutex_unlock( &create_mutex );
    DEBUGF << &main_cor << ": exiting sc_cor_pkg_pthread::create(" 
	       << cor_p << ")" << std::endl;

    return cor_p;
}


// yield to the next coroutine
//
// We don't do anything after the p_thread_cond_wait since it won't
// happen until the thread wakes up again!

void
sc_cor_pkg_pthread::yield( sc_cor* next_cor_p )
{
    sc_cor_pthread* from_p = active_cor_p;
    sc_cor_pthread* to_p = (sc_cor_pthread*)next_cor_p;

    DEBUGF << from_p << ": switch to " << to_p << std::endl;
    if ( to_p != from_p )
    {
        pthread_mutex_lock( &to_p->m_mutex );
        pthread_cond_signal( &to_p->m_pt_condition );
        pthread_mutex_lock( &from_p->m_mutex );
        pthread_mutex_unlock( &to_p->m_mutex );
        pthread_cond_wait( &from_p->m_pt_condition, &from_p->m_mutex );
        pthread_mutex_unlock( &from_p->m_mutex );
    }

    active_cor_p = from_p; // When we come out of wait make ourselves active.
	DEBUGF << from_p << " restarting after yield to " << to_p << std::endl;
}


// abort the current coroutine (and resume the next coroutine)

void
sc_cor_pkg_pthread::abort( sc_cor* next_cor_p )
{
    sc_cor_pthread* n_p = (sc_cor_pthread*)next_cor_p;

    DEBUGF << active_cor_p << ": aborting, switching to " << n_p << std::endl;
    pthread_mutex_lock( &n_p->m_mutex );
    pthread_cond_signal( &n_p->m_pt_condition );
    pthread_mutex_unlock( &n_p->m_mutex );
}


// get the main coroutine

sc_cor*
sc_cor_pkg_pthread::get_main()
{
    return &main_cor;
}

} // namespace sc_core

#endif // !defined(_WIN32) && !defined(WIN32) && defined(SC_USE_PTHREADS)


// $Log: sc_cor_pthread.cpp,v $
// Revision 1.6  2011/08/30 21:51:04  acg
//  Jerome Cornet: auto processing of pthread configurations.
//
// Revision 1.5  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.3  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.2  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.
//

// Taf!

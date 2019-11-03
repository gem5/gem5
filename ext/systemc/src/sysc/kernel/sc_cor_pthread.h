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

  sc_cor_pthread.h -- Coroutine implementation with pthreads.

  Original Author: Andy Goodrich, Forte Design Systems, 2002-11-10

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_COR_PTHREAD_H
#define SC_COR_PTHREAD_H


#if defined(SC_USE_PTHREADS)

#include "sysc/kernel/sc_cor.h"
#include "sysc/kernel/sc_cmnhdr.h"
#include <pthread.h>

namespace sc_core {

class sc_cor_pkg_pthread;
typedef sc_cor_pkg_pthread sc_cor_pkg_t;

// ----------------------------------------------------------------------------
//  CLASS : sc_cor_pthread
//
//  Coroutine class implemented with Posix Threads.
//
// Notes:
//   (1) The thread creation mutex and the creation condition are used to
//       suspend the thread creating another one until the created thread
//       reaches its invoke_module_method. This allows us to get control of
//       thread scheduling away from the pthread package.
// ----------------------------------------------------------------------------

class sc_cor_pthread : public sc_cor
{
  public:

    // constructor
    sc_cor_pthread();

    // destructor
    virtual ~sc_cor_pthread();

	// module method invocator (starts thread execution)
	static void* invoke_module_method( void* context_p );

  public:
	static sc_cor_pthread* m_active_cor_p;	   // Active coroutine.

  public:
	sc_cor_fn*          m_cor_fn;		// Core function.
	void*               m_cor_fn_arg;	// Core function argument.
	pthread_mutex_t     m_mutex;        // Mutex to suspend thread on.
    sc_cor_pkg_pthread* m_pkg_p;        // the creating coroutine package
	pthread_cond_t      m_pt_condition; // Condition waiting for.
	pthread_t           m_thread;       // Our pthread storage.

private:

    // disabled
    sc_cor_pthread( const sc_cor_pthread& );
    sc_cor_pthread& operator = ( const sc_cor_pthread& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_cor_pkg_pthread
//
//  Coroutine package class implemented with Posix Threads.
// ----------------------------------------------------------------------------

class sc_cor_pkg_pthread
: public sc_cor_pkg
{
public:

    // constructor
    sc_cor_pkg_pthread( sc_simcontext* simc );

    // destructor
    virtual ~sc_cor_pkg_pthread();

    // create a new coroutine
    virtual sc_cor* create( std::size_t stack_size, sc_cor_fn* fn, void* arg );

    // yield to the next coroutine
    virtual void yield( sc_cor* next_cor );

    // abort the current coroutine (and resume the next coroutine)
    virtual void abort( sc_cor* next_cor );

    // get the main coroutine
    virtual sc_cor* get_main();

private:

    static int instance_count;

private:

    // disabled
    sc_cor_pkg_pthread();
    sc_cor_pkg_pthread( const sc_cor_pkg_pthread& );
    sc_cor_pkg_pthread& operator = ( const sc_cor_pkg_pthread& );
};

} // namespace sc_core

#endif

// $Log: sc_cor_pthread.h,v $
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

#endif // defined(SC_USE_PTHREADS)

// Taf!

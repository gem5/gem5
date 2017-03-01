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

  sc_cor_fiber.cpp -- Coroutine implementation with fibers.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-12-18

 CHANGE LOG APPEARS AT THE END OF THE FILE
 *****************************************************************************/

#if defined(_WIN32) || defined(WIN32) || defined(WIN64)

#ifndef SC_INCLUDE_WINDOWS_H
#  define SC_INCLUDE_WINDOWS_H // include Windows.h, if needed
#endif

#include "sysc/kernel/sc_cor_fiber.h"
#include "sysc/kernel/sc_simcontext.h"
#if defined(__GNUC__) && __USING_SJLJ_EXCEPTIONS__
#   if (__GNUC__ > 3) || ((__GNUC__ == 3) && (__GNUC_MINOR__ > 2))
#      include <unwind.h>
#   else
       extern "C" void _Unwind_SjLj_Register (struct SjLj_Function_Context *);
       extern "C" void _Unwind_SjLj_Unregister (struct SjLj_Function_Context *);
#   endif
#endif

namespace sc_core {

// ----------------------------------------------------------------------------
//  File static variables.
// ----------------------------------------------------------------------------

// main coroutine

static sc_cor_fiber main_cor;
#if defined(__GNUC__) && __USING_SJLJ_EXCEPTIONS__
// current coroutine
static sc_cor_fiber* curr_cor;
#endif


// ----------------------------------------------------------------------------
//  CLASS : sc_cor_fiber
//
//  Coroutine class implemented with Windows fibers.
// ----------------------------------------------------------------------------

// destructor

sc_cor_fiber::~sc_cor_fiber()
{
    if( m_fiber != 0 ) {
      PVOID cur_fiber = GetCurrentFiber();
      if (m_fiber != cur_fiber)
         DeleteFiber( m_fiber );
    }
}


// ----------------------------------------------------------------------------
//  CLASS : sc_cor_pkg_fiber
//
//  Coroutine package class implemented with QuickThreads.
// ----------------------------------------------------------------------------

int sc_cor_pkg_fiber::instance_count = 0;


// constructor

sc_cor_pkg_fiber::sc_cor_pkg_fiber( sc_simcontext* simc )
: sc_cor_pkg( simc )
{
    if( ++ instance_count == 1 ) {
        // initialize the main coroutine
        assert( main_cor.m_fiber == 0 );
        main_cor.m_fiber = ConvertThreadToFiber( 0 );

        if( !main_cor.m_fiber && GetLastError() == ERROR_ALREADY_FIBER ) {
            // conversion of current thread to fiber has failed, because
            // someone else already converted the main thread to a fiber
            // -> store current fiber
            main_cor.m_fiber = GetCurrentFiber();
        }
        assert( main_cor.m_fiber != 0 );

#       if defined(__GNUC__) && __USING_SJLJ_EXCEPTIONS__
            // initialize the current coroutine
            assert( curr_cor == 0 );
            curr_cor = &main_cor;
#       endif
    }
}


// destructor

sc_cor_pkg_fiber::~sc_cor_pkg_fiber()
{
    if( -- instance_count == 0 ) {
	// cleanup the main coroutine
	main_cor.m_fiber = 0;
#       if defined(__GNUC__) && __USING_SJLJ_EXCEPTIONS__
            // cleanup the current coroutine
            curr_cor = 0;
#       endif
    }
}


// create a new coroutine

sc_cor*
sc_cor_pkg_fiber::create( std::size_t stack_size, sc_cor_fn* fn, void* arg )
{
    sc_cor_fiber* cor = new sc_cor_fiber;
    cor->m_pkg = this;
    cor->m_stack_size = stack_size;
    cor->m_fiber = CreateFiberEx( cor->m_stack_size / 2, cor->m_stack_size, 0,
			        (LPFIBER_START_ROUTINE) fn, arg );
    return cor;
}


// yield to the next coroutine

void
sc_cor_pkg_fiber::yield( sc_cor* next_cor )
{
    sc_cor_fiber* new_cor = SCAST<sc_cor_fiber*>( next_cor );
#   if defined(__GNUC__) && __USING_SJLJ_EXCEPTIONS__
        // Switch SJLJ exception handling function contexts
        _Unwind_SjLj_Register(&curr_cor->m_eh);
        _Unwind_SjLj_Unregister(&new_cor->m_eh);
        curr_cor = new_cor;
#   endif
    SwitchToFiber( new_cor->m_fiber );
}


// abort the current coroutine (and resume the next coroutine)

void
sc_cor_pkg_fiber::abort( sc_cor* next_cor )
{
    sc_cor_fiber* new_cor = SCAST<sc_cor_fiber*>( next_cor );
#   if defined(__GNUC__) && __USING_SJLJ_EXCEPTIONS__
        // Switch SJLJ exception handling function contexts
        _Unwind_SjLj_Register(&curr_cor->m_eh);
        _Unwind_SjLj_Unregister(&new_cor->m_eh);
        curr_cor = new_cor;
#   endif
    SwitchToFiber( new_cor->m_fiber );
}


// get the main coroutine

sc_cor*
sc_cor_pkg_fiber::get_main()
{
    return &main_cor;
}

} // namespace sc_core


// $Log: sc_cor_fiber.cpp,v $
// Revision 1.9  2011/09/08 16:12:45  acg
//  Philipp A. Hartmann: make sure we don't try to make a thread a fiber if
//  its already a fiber.
//
// Revision 1.8  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.7  2011/06/25 17:08:39  acg
//  Andy Goodrich: Jerome Cornet's changes to use libtool to build the
//  library.
//
// Revision 1.6  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.5  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.4  2011/01/19 23:21:50  acg
//  Andy Goodrich: changes for IEEE 1666 2011
//
// Revision 1.3  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
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

#endif

// Taf!

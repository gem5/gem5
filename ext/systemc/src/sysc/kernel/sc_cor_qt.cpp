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

  sc_cor_qt.cpp -- Coroutine implementation with QuickThreads.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-12-18

 CHANGE LOG APPEARS AT THE END OF THE FILE
 *****************************************************************************/

#if !defined(_WIN32) && !defined(WIN32) && !defined(SC_USE_PTHREADS)

#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>

#include "sysc/kernel/sc_cor_qt.h"
#include "sysc/kernel/sc_simcontext.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  File static variables.
// ----------------------------------------------------------------------------

// main coroutine

static sc_cor_qt main_cor;

// current coroutine

static sc_cor_qt* curr_cor = 0;


// ----------------------------------------------------------------------------
//  CLASS : sc_cor_qt
//
//  Coroutine class implemented with QuickThreads.
// ----------------------------------------------------------------------------

// switch stack protection on/off

void
sc_cor_qt::stack_protect( bool enable )
{
    // Code needs to be tested on HP-UX and disabled if it doesn't work there
    // Code still needs to be ported to WIN32

    static std::size_t pagesize;
    
    if( pagesize == 0 ) {
#       if defined(__ppc__)
	    pagesize = getpagesize();
#       else
	    pagesize = sysconf( _SC_PAGESIZE );
#       endif 
    }

    assert( pagesize != 0 );
    assert( m_stack_size > ( 2 * pagesize ) );

#ifdef QUICKTHREADS_GROW_DOWN
    // Stacks grow from high address down to low address
    caddr_t redzone = caddr_t( ( ( std::size_t( m_stack ) + pagesize - 1 ) /
				 pagesize ) * pagesize );
#else
    // Stacks grow from low address up to high address
    caddr_t redzone = caddr_t( ( ( std::size_t( m_stack ) +
				   m_stack_size - pagesize ) /
				 pagesize ) * pagesize );
#endif

    int ret;

    // Enable the red zone at the end of the stack so that references within
    // it will cause an interrupt.

    if( enable ) {
        ret = mprotect( redzone, pagesize - 1, PROT_NONE );
    } 
    
    // Revert the red zone to normal memory usage. Try to make it read - write -
    // execute. If that does not work then settle for read - write

    else {
        ret = mprotect( redzone, pagesize - 1, PROT_READ|PROT_WRITE|PROT_EXEC);
        if ( ret != 0 )
            ret = mprotect( redzone, pagesize - 1, PROT_READ | PROT_WRITE );
    }

    assert( ret == 0 );
}


// ----------------------------------------------------------------------------
//  CLASS : sc_cor_pkg_qt
//
//  Coroutine package class implemented with QuickThreads.
// ----------------------------------------------------------------------------

int sc_cor_pkg_qt::instance_count = 0;


// support function

inline
void*
stack_align( void* sp, int alignment, std::size_t* stack_size )
{
    int round_up_mask = alignment - 1;
    *stack_size = (*stack_size + round_up_mask) & ~round_up_mask;
    return ( (void*)(((qt_word_t) sp + round_up_mask) & ~round_up_mask) );
}


// constructor

sc_cor_pkg_qt::sc_cor_pkg_qt( sc_simcontext* simc )
: sc_cor_pkg( simc )
{
    if( ++ instance_count == 1 ) {
	// initialize the current coroutine
	assert( curr_cor == 0 );
	curr_cor = &main_cor;
    }
}


// destructor

sc_cor_pkg_qt::~sc_cor_pkg_qt()
{
    if( -- instance_count == 0 ) {
	// cleanup the current coroutine
	curr_cor = 0;
    }
}


// create a new coroutine

extern "C"
void
sc_cor_qt_wrapper( void* arg, void* cor, qt_userf_t* fn )
{
    curr_cor = RCAST<sc_cor_qt*>( cor );
    // invoke the user function
    (*(sc_cor_fn*) fn)( arg );
    // not reached
}

sc_cor*
sc_cor_pkg_qt::create( std::size_t stack_size, sc_cor_fn* fn, void* arg )
{
    sc_cor_qt* cor = new sc_cor_qt();
    cor->m_pkg = this;
    cor->m_stack_size = stack_size;
    cor->m_stack = new char[cor->m_stack_size];
    void* sto = stack_align( cor->m_stack, QUICKTHREADS_STKALIGN, 
                             &cor->m_stack_size );
    cor->m_sp = QUICKTHREADS_SP(sto, cor->m_stack_size - QUICKTHREADS_STKALIGN);
    cor->m_sp = QUICKTHREADS_ARGS( cor->m_sp, arg, cor, (qt_userf_t*) fn,
			           sc_cor_qt_wrapper );
    return cor;
}


// yield to the next coroutine

extern "C"
void*
sc_cor_qt_yieldhelp( qt_t* sp, void* old_cor, void* )
{
    RCAST<sc_cor_qt*>( old_cor )->m_sp = sp;
    return 0;
}

void
sc_cor_pkg_qt::yield( sc_cor* next_cor )
{
    sc_cor_qt* new_cor = SCAST<sc_cor_qt*>( next_cor );
    sc_cor_qt* old_cor = curr_cor;
    curr_cor = new_cor;
    QUICKTHREADS_BLOCK( sc_cor_qt_yieldhelp, old_cor, 0, new_cor->m_sp );
}


// abort the current coroutine (and resume the next coroutine)

extern "C"
void*
sc_cor_qt_aborthelp( qt_t*, void*, void* )
{
    return 0;
}

void
sc_cor_pkg_qt::abort( sc_cor* next_cor )
{
    sc_cor_qt* new_cor = SCAST<sc_cor_qt*>( next_cor );
    sc_cor_qt* old_cor = curr_cor;
    curr_cor = new_cor;
    QUICKTHREADS_ABORT( sc_cor_qt_aborthelp, old_cor, 0, new_cor->m_sp );
}


// get the main coroutine

sc_cor*
sc_cor_pkg_qt::get_main()
{
    return &main_cor;
}

} // namespace sc_core

#endif

// $Log: sc_cor_qt.cpp,v $
// Revision 1.9  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.8  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.7  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.6  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.5  2010/08/03 16:52:14  acg
//  Andy Goodrich: line formatting.
//
// Revision 1.4  2008/11/11 14:03:07  acg
//  Andy Goodrich: added execute access to the release of red zone storage
//  per Ulli's suggestion.
//
// Revision 1.3  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.2  2008/03/24 18:32:36  acg
//  Andy Goodrich: added include of sys/types.h to pick up the declaration
//  of caddr_t.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.

// Taf!

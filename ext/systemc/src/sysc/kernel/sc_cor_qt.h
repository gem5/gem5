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

  sc_cor_qt.h -- Coroutine implementation with QuickThreads.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-12-18

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_COR_QT_H
#define SC_COR_QT_H


#if !defined(_WIN32) && !defined(WIN32) && !defined(WIN64)  && !defined(SC_USE_PTHREADS)

#include "sysc/kernel/sc_cor.h"
#include "sysc/qt/qt.h"

namespace sc_core {

class sc_cor_pkg_qt;
typedef sc_cor_pkg_qt sc_cor_pkg_t;

// ----------------------------------------------------------------------------
//  CLASS : sc_cor_qt
//
//  Coroutine class implemented with QuickThreads.
// ----------------------------------------------------------------------------

class sc_cor_qt
: public sc_cor
{
public:

    // constructor
    sc_cor_qt()
	: m_stack_size( 0 ), m_stack( 0 ), m_sp( 0 ), m_pkg( 0 )
	{}

    // destructor
    virtual ~sc_cor_qt()
        { delete[] (char*) m_stack; }

    // switch stack protection on/off
    virtual void stack_protect( bool enable );

public:

    std::size_t    m_stack_size;  // stack size
    void*          m_stack;       // stack
    qt_t*          m_sp;          // stack pointer

    sc_cor_pkg_qt* m_pkg;         // the creating coroutine package

private:

    // disabled
    sc_cor_qt( const sc_cor_qt& );
    sc_cor_qt& operator = ( const sc_cor_qt& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_cor_pkg_qt
//
//  Coroutine package class implemented with QuickThreads.
// ----------------------------------------------------------------------------

class sc_cor_pkg_qt
: public sc_cor_pkg
{
public:

    // constructor
    sc_cor_pkg_qt( sc_simcontext* simc );

    // destructor
    virtual ~sc_cor_pkg_qt();

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
    sc_cor_pkg_qt();
    sc_cor_pkg_qt( const sc_cor_pkg_qt& );
    sc_cor_pkg_qt& operator = ( const sc_cor_pkg_qt& );
};

} // namespace sc_core

#endif

// $Log: sc_cor_qt.h,v $
// Revision 1.6  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
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

#endif

// Taf!

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

  sc_event_queue.cpp -- Event Queue Support

  Original Author: Stuart Swan, Cadence Inc.

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/communication/sc_event_queue.h"
#include "sysc/kernel/sc_method_process.h"

namespace sc_core {

static int 
sc_time_compare( const void* p1, const void* p2 )
{
    const sc_time* t1 = static_cast<const sc_time*>( p1 );
    const sc_time* t2 = static_cast<const sc_time*>( p2 );

    if( *t1 < *t2 ) {
	return 1;
    } else if( *t1 > *t2 ) {
	return -1;
    } else {
	return 0;
    }  
}

sc_event_queue::sc_event_queue( sc_module_name name_ )
    : sc_module( name_ ),
      m_ppq( 128, sc_time_compare ),
      m_e( (std::string(SC_KERNEL_EVENT_PREFIX)+"_event").c_str() ),
      m_change_stamp(0),
      m_pending_delta(0)
{
    SC_METHOD( fire_event );
    sensitive << m_e;
    dont_initialize();
}

sc_event_queue::~sc_event_queue()
{
  while (m_ppq.size() > 0) {
    delete m_ppq.extract_top();
  }
}

void sc_event_queue::cancel_all()
{
    m_pending_delta = 0;
    while( m_ppq.size() > 0 )
	delete m_ppq.extract_top();
    m_e.cancel();
}

void sc_event_queue::notify (const sc_time& when)
{
    m_change_stamp = simcontext()->change_stamp();
    sc_time* t = new sc_time( when+sc_time_stamp() );
    if ( m_ppq.size()==0 || *t < *m_ppq.top() ) {
	m_e.notify( when );
    }
    m_ppq.insert( t );
}
    
void sc_event_queue::fire_event()
{
    if ( m_ppq.empty() ) { // event has been cancelled
        return;
    }
    sc_time* t = m_ppq.extract_top();
    assert( *t==sc_time_stamp() );
    delete t;

    if ( m_ppq.size() > 0 ) {
	m_e.notify( *m_ppq.top() - sc_time_stamp() );
    }
}

} // namespace sc_core

// $Log: sc_event_queue.cpp,v $
// Revision 1.9  2011/08/26 22:45:53  acg
//  Torsten Maehne: remove redundant initialization assignment.
//
// Revision 1.8  2011/08/26 21:44:58  acg
//  Andy Goodrich: fix internal event naming.
//
// Revision 1.7  2011/08/26 20:45:39  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.6  2011/08/24 22:05:35  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.5  2011/04/08 18:22:46  acg
//  Philipp A. Hartmann: use the context of the primitive channel to get
//  the change stamp value.
//
// Revision 1.4  2011/04/05 20:48:09  acg
//  Andy Goodrich: changes to make sure that event(), posedge() and negedge()
//  only return true if the clock has not moved.
//
// Revision 1.3  2011/02/18 20:23:45  acg
//  Andy Goodrich: Copyright update.
//
// Revision 1.2  2010/07/22 20:02:30  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.5  2006/11/28 20:30:48  acg
//  Andy Goodrich: updated from 2.2 source. sc_event_queue constructors
//  collapsed into a single constructor with an optional argument to get
//  the sc_module_name stack done correctly. Class name prefixing added
//  to sc_semaphore calls to wait() to keep gcc 4.x happy.
//
// Revision 1.4  2006/01/26 21:00:50  acg
//  Andy Goodrich: conversion to use sc_event::notify(SC_ZERO_TIME) instead of
//  sc_event::notify_delayed()
//
// Revision 1.3  2006/01/13 18:47:42  acg
// Added $Log command so that CVS comments are reproduced in the source.

// taf

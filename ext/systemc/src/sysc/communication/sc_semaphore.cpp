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

  sc_semaphore.cpp -- The sc_semaphore primitive channel class.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/communication/sc_communication_ids.h"
#include "sysc/communication/sc_semaphore.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_wait.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_semaphore
//
//  The sc_semaphore primitive channel class.
// ----------------------------------------------------------------------------

// error reporting

void
sc_semaphore::report_error( const char* id, const char* add_msg ) const
{
    char msg[BUFSIZ];
    if( add_msg != 0 ) {
	std::sprintf( msg, "%s: semaphore '%s'", add_msg, name() );
    } else {
	std::sprintf( msg, "semaphore '%s'", name() );
    }
    SC_REPORT_ERROR( id, msg );
}


// constructors

sc_semaphore::sc_semaphore( int init_value_ )
: sc_object( sc_gen_unique_name( "semaphore" ) ),
  m_free( (std::string(SC_KERNEL_EVENT_PREFIX)+"_free_event").c_str() ),
  m_value( init_value_ )
{
    if( m_value < 0 ) {
	report_error( SC_ID_INVALID_SEMAPHORE_VALUE_ );
    }
}

sc_semaphore::sc_semaphore( const char* name_, int init_value_ )
: sc_object( name_ ), 
  m_free( (std::string(SC_KERNEL_EVENT_PREFIX)+"_free_event").c_str() ),
  m_value( init_value_ )
{
    if( m_value < 0 ) {
	report_error( SC_ID_INVALID_SEMAPHORE_VALUE_ );
    }
}


// interface methods

// lock (take) the semaphore, block if not available

int
sc_semaphore::wait()
{
    while( in_use() ) {
	sc_core::wait( m_free, sc_get_curr_simcontext() );
    }
    -- m_value;
    return 0;
}


// lock (take) the semaphore, return -1 if not available

int
sc_semaphore::trywait()
{
    if( in_use() ) {
	return -1;
    }
    -- m_value;
    return 0;
}


// unlock (give) the semaphore

int
sc_semaphore::post()
{
    ++m_value;
    m_free.notify();
    return 0;
}

} // namespace sc_core

// $Log: sc_semaphore.cpp,v $
// Revision 1.5  2011/08/26 20:45:42  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/03/23 16:17:22  acg
//  Andy Goodrich: hide the sc_events that are kernel related.
//
// Revision 1.3  2011/02/18 20:23:45  acg
//  Andy Goodrich: Copyright update.
//
// Revision 1.2  2010/11/02 16:31:01  acg
//  Andy Goodrich: changed object derivation to use sc_object rather than
//  sc_prim_channel as the parent class.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.6  2006/11/28 20:30:49  acg
//  Andy Goodrich: updated from 2.2 source. sc_event_queue constructors
//  collapsed into a single constructor with an optional argument to get
//  the sc_module_name stack done correctly. Class name prefixing added
//  to sc_semaphore calls to wait() to keep gcc 4.x happy.
//
// Revision 1.3  2006/01/13 18:47:42  acg
// Added $Log command so that CVS comments are reproduced in the source.
//

// Taf!

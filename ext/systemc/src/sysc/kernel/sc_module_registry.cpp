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

  sc_module_registry.cpp -- Registry for all modules.
                            FOR INTERNAL USE ONLY.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_module.h"
#include "sysc/kernel/sc_module_registry.h"
#include "sysc/kernel/sc_simcontext.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_module_registry
//
//  Registry for all modules.
//  FOR INTERNAL USE ONLY!
// ----------------------------------------------------------------------------

void
sc_module_registry::insert( sc_module& module_ )
{
    if( sc_is_running() ) {
	SC_REPORT_ERROR( SC_ID_INSERT_MODULE_, "simulation running" );
    }

    if( m_simc->elaboration_done() ) {
       SC_REPORT_ERROR( SC_ID_INSERT_MODULE_, "elaboration done" );
    }

#ifdef DEBUG_SYSTEMC
    // check if module_ is already inserted
    for( int i = size() - 1; i >= 0; -- i ) {
	if( &module_ == m_module_vec[i] ) {
	    SC_REPORT_ERROR( SC_ID_INSERT_MODULE_, "already inserted" );
	}
    }
#endif

    // insert
    m_module_vec.push_back( &module_ );
}

void
sc_module_registry::remove( sc_module& module_ )
{
    int i;
    for( i = 0; i < size(); ++ i ) {
	if( &module_ == m_module_vec[i] ) {
	    break;
	}
    }
    if( i == size() ) {
	SC_REPORT_ERROR( SC_ID_REMOVE_MODULE_, 0 );
    }

    // remove
    m_module_vec[i] = m_module_vec[size() - 1];
    m_module_vec.resize(m_module_vec.size()-1);
}


// constructor

sc_module_registry::sc_module_registry( sc_simcontext& simc_ )
 : m_construction_done(0), m_module_vec(), m_simc( &simc_ )
{}


// destructor

sc_module_registry::~sc_module_registry()
{}

// called when construction is done

bool
sc_module_registry::construction_done()
{
    if( size() == m_construction_done )
        // nothing has been updated
        return true;

    for( ; m_construction_done < size(); ++m_construction_done ) {
        m_module_vec[m_construction_done]->construction_done();
    }
    return false;
}

// called when elaboration is done

void
sc_module_registry::elaboration_done()
{
    bool error = false;
    for( int i = 0; i < size(); ++ i ) {
	m_module_vec[i]->elaboration_done( error );
    }
}

// called before simulation begins

void
sc_module_registry::start_simulation()
{
    for( int i = 0; i < size(); ++ i ) {
	m_module_vec[i]->start_simulation();
    }
}

// called after simulation ends

void
sc_module_registry::simulation_done()
{
    for( int i = 0; i < size(); ++ i ) {
	m_module_vec[i]->simulation_done();
    }
}

} // namespace sc_core

// $Log: sc_module_registry.cpp,v $
// Revision 1.8  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.7  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.6  2011/05/09 04:07:49  acg
//  Philipp A. Hartmann:
//    (1) Restore hierarchy in all phase callbacks.
//    (2) Ensure calls to before_end_of_elaboration.
//
// Revision 1.5  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.4  2011/02/14 17:51:40  acg
//  Andy Goodrich: proper pushing an poppping of the module hierarchy for
//  start_of_simulation() and end_of_simulation.
//
// Revision 1.3  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.4  2006/01/26 21:04:54  acg
//  Andy Goodrich: deprecation message changes and additional messages.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//

// Taf!

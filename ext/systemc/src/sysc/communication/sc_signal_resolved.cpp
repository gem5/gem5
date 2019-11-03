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

  sc_signal_resolved.cpp -- The resolved signal class.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/communication/sc_signal_resolved.h"

namespace sc_core {

// Note that we assume that two drivers driving the resolved signal to a 1 or
// 0 is O.K. This might not be true for all technologies, but is certainly
// true for CMOS, the predominant technology in use today.

const sc_dt::sc_logic_value_t
sc_logic_resolution_tbl[4][4] =
{   //    0      1      Z      X
    { sc_dt::Log_0, sc_dt::Log_X, sc_dt::Log_0, sc_dt::Log_X }, // 0
    { sc_dt::Log_X, sc_dt::Log_1, sc_dt::Log_1, sc_dt::Log_X }, // 1
    { sc_dt::Log_0, sc_dt::Log_1, sc_dt::Log_Z, sc_dt::Log_X }, // Z
    { sc_dt::Log_X, sc_dt::Log_X, sc_dt::Log_X, sc_dt::Log_X }  // X
};


// ----------------------------------------------------------------------------
//  FUNCTION : sc_logic_resolve
//
//  Resolution function for sc_dt::sc_logic.
// ----------------------------------------------------------------------------

// resolves sc_dt::sc_logic values and returns the resolved value

static void
sc_logic_resolve( sc_dt::sc_logic& result_,
                  const std::vector<sc_dt::sc_logic>& values_ )
{
    int sz = values_.size();

    assert( sz != 0 );

    if( sz == 1 ) {
	result_ = values_[0];
	return;
    }

    sc_dt::sc_logic_value_t res = values_[0].value();
    for( int i = sz - 1; i > 0 && res != sc_dt::Log_X; -- i ) {
       res = sc_logic_resolution_tbl[res][values_[i].value()];
    }
    result_ = res;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_resolved
//
//  The resolved signal class.
// ----------------------------------------------------------------------------

// write the new value

void
sc_signal_resolved::write( const data_type& value_ )
{
    sc_process_b* cur_proc = sc_get_current_process_b();

    bool value_changed = false;
    bool found = false;
    
    for( int i = m_proc_vec.size() - 1; i >= 0; -- i ) {
	if( cur_proc == m_proc_vec[i] ) {
	    if( value_ != m_val_vec[i] ) {
		m_val_vec[i] = value_;
		value_changed = true;
	    }
	    found = true;
	    break;
	}
    }
    
    if( ! found ) {
	m_proc_vec.push_back( cur_proc );
	m_val_vec.push_back( value_ );
	value_changed = true;
    }
    
    if( value_changed ) {
	request_update();
    }
}


void
sc_signal_resolved::update()
{
    sc_logic_resolve( m_new_val, m_val_vec );
    base_type::update();
}


} // namespace sc_core

// $Log: sc_signal_resolved.cpp,v $
// Revision 1.5  2011/08/26 20:45:44  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/02/18 20:23:45  acg
//  Andy Goodrich: Copyright update.
//
// Revision 1.3  2011/02/07 19:16:50  acg
//  Andy Goodrich: changes for handling multiple writers.
//
// Revision 1.2  2011/01/20 16:52:15  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.4  2006/03/21 00:00:27  acg
//   Andy Goodrich: changed name of sc_get_current_process_base() to be
//   sc_get_current_process_b() since its returning an sc_process_b instance.
//
// Revision 1.3  2006/01/13 18:47:42  acg
// Added $Log command so that CVS comments are reproduced in the source.
//

// Taf!

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

  sc_signal_ports.cpp -- The sc_signal<T> port classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-08-20

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/communication/sc_signal_ports.h"
#include "sysc/datatypes/int/sc_signed.h"
#include "sysc/datatypes/int/sc_unsigned.h"
#include "sysc/datatypes/bit/sc_lv_base.h"
#include "sysc/utils/sc_utils_ids.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_in<bool>
//
//  Specialization of sc_in<T> for type bool.
// ----------------------------------------------------------------------------

// called when elaboration is done

void
sc_in<bool>::end_of_elaboration()
{
    if( m_traces != 0 ) {
	for( int i = 0; i < (int)m_traces->size(); ++ i ) {
	    sc_trace_params* p = (*m_traces)[i];
	    in_if_type* iface = DCAST<in_if_type*>( get_interface() );
	    sc_trace( p->tf, iface->read(), p->name );
	}
	remove_traces();
    }
}

// called by sc_trace

void
sc_in<bool>::add_trace_internal(sc_trace_file* tf_, 
	const std::string& name_) const
{
    if( tf_ != 0 ) {
	if( m_traces == 0 ) {
	    m_traces = new sc_trace_params_vec;
	}
	m_traces->push_back( new sc_trace_params( tf_, name_ ) );
    }
}

void
sc_in<bool>::add_trace(sc_trace_file* tf_, 
	const std::string& name_) const
{
    sc_deprecated_add_trace();
	add_trace_internal(tf_, name_);
}

void
sc_in<bool>::remove_traces() const
{
    if( m_traces != 0 ) {
	for( int i = m_traces->size() - 1; i >= 0; -- i ) {
	    delete (*m_traces)[i];
	}
	delete m_traces;
	m_traces = 0;
    }
}


// called by pbind (for internal use only)

int
sc_in<bool>::vbind( sc_interface& interface_ )
{
    return sc_port_b<if_type>::vbind( interface_ );
}

int
sc_in<bool>::vbind( sc_port_base& parent_ )
{
    in_port_type* in_parent = DCAST<in_port_type*>( &parent_ );
    if( in_parent != 0 ) {
	sc_port_base::bind( *in_parent );
	return 0;
    }
    inout_port_type* inout_parent = DCAST<inout_port_type*>( &parent_ );
    if( inout_parent != 0 ) {
	sc_port_base::bind( *inout_parent );
	return 0;
    }
    // type mismatch
    return 2;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_in<sc_logic>
//
//  Specialization of sc_in<T> for type sc_logic.
// ----------------------------------------------------------------------------

// called when elaboration is done

void
sc_in<sc_dt::sc_logic>::end_of_elaboration()
{
    if( m_traces != 0 ) {
	for( int i = 0; i < (int)m_traces->size(); ++ i ) {
	    sc_trace_params* p = (*m_traces)[i];
	    in_if_type* iface = DCAST<in_if_type*>( get_interface() );
	    sc_trace( p->tf, iface->read(), p->name );
	}
	remove_traces();
    }
}


// called by sc_trace

void
sc_in<sc_dt::sc_logic>::add_trace_internal( sc_trace_file* tf_, 
    const std::string& name_ ) const
{
    if( tf_ != 0 ) {
	if( m_traces == 0 ) {
	    m_traces = new sc_trace_params_vec;
	}
	m_traces->push_back( new sc_trace_params( tf_, name_ ) );
    }
}

void
sc_in<sc_dt::sc_logic>::add_trace( sc_trace_file* tf_, 
    const std::string& name_ ) const
{
    sc_deprecated_add_trace();
    add_trace_internal(tf_, name_);
}

void
sc_in<sc_dt::sc_logic>::remove_traces() const
{
    if( m_traces != 0 ) {
	for( int i = m_traces->size() - 1; i >= 0; -- i ) {
	    delete (*m_traces)[i];
	}
	delete m_traces;
	m_traces = 0;
    }
}


// called by pbind (for internal use only)

int
sc_in<sc_dt::sc_logic>::vbind( sc_interface& interface_ )
{
    return sc_port_b<if_type>::vbind( interface_ );
}

int
sc_in<sc_dt::sc_logic>::vbind( sc_port_base& parent_ )
{
    in_port_type* in_parent = DCAST<in_port_type*>( &parent_ );
    if( in_parent != 0 ) {
	sc_port_base::bind( *in_parent );
	return 0;
    }
    inout_port_type* inout_parent = DCAST<inout_port_type*>( &parent_ );
    if( inout_parent != 0 ) {
	sc_port_base::bind( *inout_parent );
	return 0;
    }
    // type mismatch
    return 2;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_inout<bool>
//
//  Specialization of sc_inout<T> for type bool.
// ----------------------------------------------------------------------------

// destructor

sc_inout<bool>::~sc_inout()
{
    delete m_change_finder_p;
    delete m_neg_finder_p;
    delete m_pos_finder_p;
    delete m_init_val;
    remove_traces();
}


// set initial value (can also be called when port is not bound yet)

void
sc_inout<bool>::initialize( const data_type& value_ )
{
    inout_if_type* iface = DCAST<inout_if_type*>( get_interface() );
    if( iface != 0 ) {
	iface->write( value_ );
    } else {
	if( m_init_val == 0 ) {
	    m_init_val = new data_type;
	}
	*m_init_val = value_;
    }
}


// called when elaboration is done

void
sc_inout<bool>::end_of_elaboration()
{
    if( m_init_val != 0 ) {
	write( *m_init_val );
	delete m_init_val;
	m_init_val = 0;
    }
    if( m_traces != 0 ) {
	for( int i = 0; i < (int)m_traces->size(); ++ i ) {
	    sc_trace_params* p = (*m_traces)[i];
	    in_if_type* iface = DCAST<in_if_type*>( get_interface() );
	    sc_trace( p->tf, iface->read(), p->name );
	}
	remove_traces();
    }
}


// called by sc_trace

void
sc_inout<bool>::add_trace_internal( sc_trace_file* tf_, 
    const std::string& name_ ) const
{
    if( tf_ != 0 ) {
	if( m_traces == 0 ) {
	    m_traces = new sc_trace_params_vec;
	}
	m_traces->push_back( new sc_trace_params( tf_, name_ ) );
    }
}

void
sc_inout<bool>::add_trace( sc_trace_file* tf_, 
    const std::string& name_ ) const
{
    sc_deprecated_add_trace();
    add_trace_internal(tf_, name_);
}

void
sc_inout<bool>::remove_traces() const
{
    if( m_traces != 0 ) {
	for( int i = m_traces->size() - 1; i >= 0; -- i ) {
	    delete (*m_traces)[i];
	}
	delete m_traces;
	m_traces = 0;
    }
}


// ----------------------------------------------------------------------------
//  CLASS : sc_inout<sc_dt::sc_logic>
//
//  Specialization of sc_inout<T> for type sc_dt::sc_logic.
// ----------------------------------------------------------------------------

// destructor

sc_inout<sc_dt::sc_logic>::~sc_inout()
{
    delete m_change_finder_p;
    delete m_neg_finder_p;
    delete m_pos_finder_p;
    delete m_init_val;
    remove_traces();
}


// set initial value (can also be called when port is not bound yet)

void
sc_inout<sc_dt::sc_logic>::initialize( const data_type& value_ )
{
    inout_if_type* iface = DCAST<inout_if_type*>( get_interface() );
    if( iface != 0 ) {
	iface->write( value_ );
    } else {
	if( m_init_val == 0 ) {
	    m_init_val = new data_type;
	}
	*m_init_val = value_;
    }
}


// called when elaboration is done

void
sc_inout<sc_dt::sc_logic>::end_of_elaboration()
{
    if( m_init_val != 0 ) {
	write( *m_init_val );
	delete m_init_val;
	m_init_val = 0;
    }
    if( m_traces != 0 ) {
	for( int i = 0; i < (int)m_traces->size(); ++ i ) {
	    sc_trace_params* p = (*m_traces)[i];
	    in_if_type* iface = DCAST<in_if_type*>( get_interface() );
	    sc_trace( p->tf, iface->read(), p->name );
	}
	remove_traces();
    }
}


// called by sc_trace

void
sc_inout<sc_dt::sc_logic>::add_trace_internal( sc_trace_file* tf_,
			       const std::string& name_ ) const
{
    if( tf_ != 0 ) {
	if( m_traces == 0 ) {
	    m_traces = new sc_trace_params_vec;
	}
	m_traces->push_back( new sc_trace_params( tf_, name_ ) );
    }
}


void
sc_inout<sc_dt::sc_logic>::add_trace( sc_trace_file* tf_,
			       const std::string& name_ ) const
{
    sc_deprecated_add_trace();
    add_trace_internal(tf_, name_);
}

void
sc_inout<sc_dt::sc_logic>::remove_traces() const
{
    if( m_traces != 0 ) {
	for( int i = m_traces->size() - 1; i >= 0; -- i ) {
	    delete (*m_traces)[i];
	}
	delete m_traces;
	m_traces = 0;
    }
}

void sc_deprecated_add_trace()
{
    static bool warn_add_trace_deprecated=true;
    if ( warn_add_trace_deprecated )
    {
        warn_add_trace_deprecated=false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "sc_signal<T>::addtrace() is deprecated");
    }
}
} // namespace sc_core

// $Log: sc_signal_ports.cpp,v $
// Revision 1.3  2011/08/26 20:45:43  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.2  2011/02/18 20:23:45  acg
//  Andy Goodrich: Copyright update.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.8  2006/05/08 17:52:47  acg
//  Andy Goodrich:
//    (1) added David Long's forward declarations for friend functions,
//        methods, and operators to keep the Microsoft compiler happy.
//    (2) Added delta_count() method to sc_prim_channel for use by
//        sc_signal so that the friend declaration in sc_simcontext.h
// 	   can be for a non-templated class (i.e., sc_prim_channel.)
//
// Revision 1.7  2006/04/18 18:01:26  acg
//  Andy Goodrich: added an add_trace_internal() method to the various port
//  classes so that sc_trace has something to call that won't emit an
//  IEEE 1666 deprecation message.
//
// Revision 1.6  2006/02/02 23:42:37  acg
//  Andy Goodrich: implemented a much better fix to the sc_event_finder
//  proliferation problem. This new version allocates only a single event
//  finder for each port for each type of event, e.g., pos(), neg(), and
//  value_change(). The event finder persists as long as the port does,
//  which is what the LRM dictates. Because only a single instance is
//  allocated for each event type per port there is not a potential
//  explosion of storage as was true in the 2.0.1/2.1 versions.
//
// Revision 1.5  2006/01/25 00:31:11  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.4  2006/01/24 20:46:32  acg
// Andy Goodrich: changes to eliminate use of deprecated features. For instance,
// using notify(SC_ZERO_TIME) in place of notify_delayed().
//
// Revision 1.3  2006/01/13 18:47:42  acg
// Added $Log command so that CVS comments are reproduced in the source.
//

// Taf!

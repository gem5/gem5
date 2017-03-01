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

  sc_signal_rv.h -- The resolved vector signal class.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_SIGNAL_RV_H
#define SC_SIGNAL_RV_H

#include "sysc/communication/sc_signal.h"
#include "sysc/datatypes/bit/sc_lv.h"

namespace sc_core {

class sc_process_b;


// ----------------------------------------------------------------------------
//  CLASS sc_lv_resolve<W>
//
//  Resolution function for sc_dt::sc_lv<W>.
// ----------------------------------------------------------------------------

extern const sc_dt::sc_logic_value_t sc_logic_resolution_tbl[4][4];


template <int W>
class sc_lv_resolve
{
public:

    // resolves sc_dt::sc_lv<W> values and returns the resolved value
    static void resolve(sc_dt::sc_lv<W>&, const std::vector<sc_dt::sc_lv<W>*>&);
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// resolves sc_dt::sc_lv<W> values and returns the resolved value

template <int W>
inline
void
sc_lv_resolve<W>::resolve( sc_dt::sc_lv<W>& result_,
			   const std::vector<sc_dt::sc_lv<W>*>& values_ )
{
    int sz = values_.size();

    assert( sz != 0 );

    if( sz == 1 ) {
	result_ = *values_[0];
	return;
    }

    for( int j = result_.length() - 1; j >= 0; -- j ) {
	sc_dt::sc_logic_value_t res = (*values_[0])[j].value();
	for( int i = sz - 1; i > 0 && res != 3; -- i ) {
	    res = sc_logic_resolution_tbl[res][(*values_[i])[j].value()];
	}
	result_[j] = res;
    }
}


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_rv<W>
//
//  The resolved vector signal class.
// ----------------------------------------------------------------------------

template <int W>
class sc_signal_rv
: public sc_signal<sc_dt::sc_lv<W>, SC_MANY_WRITERS>
{
public:

    // typedefs

    typedef sc_signal_rv<W>                             this_type;
    typedef sc_signal<sc_dt::sc_lv<W>, SC_MANY_WRITERS> base_type;
    typedef sc_dt::sc_lv<W>                             data_type;

public:

    // constructors

    sc_signal_rv()
        : base_type( sc_gen_unique_name( "signal_rv" ) )
	{}

    explicit sc_signal_rv( const char* name_ )
        : base_type( name_ )
	{}


    // destructor
    virtual ~sc_signal_rv();


    // interface methods

    virtual void register_port( sc_port_base&, const char* )
	{}


    // write the new value
    virtual void write( const data_type& );


    // other methods

    this_type& operator = ( const data_type& a )
        { write( a ); return *this; }

    this_type& operator = ( const this_type& a )
        { write( a.read() ); return *this; }

    virtual const char* kind() const
        { return "sc_signal_rv"; }

protected:

    virtual void update();

protected:

    std::vector<sc_process_b*> m_proc_vec; // processes writing this signal
    std::vector<data_type*>       m_val_vec;  // new values written this signal

private:

    // disabled
    sc_signal_rv( const this_type& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII


// destructor

template <int W>
inline
sc_signal_rv<W>::~sc_signal_rv()
{
    for( int i = m_val_vec.size() - 1; i >= 0; -- i ) {
	delete m_val_vec[i];
    }
}


// write the new value

template <int W>
inline
void
sc_signal_rv<W>::write( const data_type& value_ )
{
    sc_process_b* cur_proc = sc_get_current_process_b();

    bool value_changed = false;
    bool found = false;
    
    for( int i = m_proc_vec.size() - 1; i >= 0; -- i ) {
	if( cur_proc == m_proc_vec[i] ) {
	    if( value_ != *m_val_vec[i] ) {
		*m_val_vec[i] = value_;
		value_changed = true;
	    }
	    found = true;
	    break;
	}
    }
    
    if( ! found ) {
	m_proc_vec.push_back( cur_proc );
	m_val_vec.push_back( new data_type( value_ ) );
	value_changed = true;
    }
    
    if( value_changed ) {
	this->request_update();
    }
}


template <int W>
inline
void
sc_signal_rv<W>::update()
{
    sc_lv_resolve<W>::resolve( this->m_new_val, m_val_vec );
    base_type::update();
}

} // namespace sc_core

//$Log: sc_signal_rv.h,v $
//Revision 1.4  2011/08/26 20:45:44  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.3  2011/04/19 02:36:26  acg
// Philipp A. Hartmann: new aysnc_update and mutex support.
//
//Revision 1.2  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
//
//Revision 1.1.1.1  2006/12/15 20:20:04  acg
//SystemC 2.3
//
//Revision 1.3  2006/03/21 00:00:27  acg
//  Andy Goodrich: changed name of sc_get_current_process_base() to be
//  sc_get_current_process_b() since its returning an sc_process_b instance.
//
//Revision 1.2  2006/01/03 23:18:26  acg
//Changed copyright to include 2006.
//
//Revision 1.1.1.1  2005/12/19 23:16:43  acg
//First check in of SystemC 2.1 into its own archive.
//
//Revision 1.10  2005/09/15 23:01:52  acg
//Added std:: prefix to appropriate methods and types to get around
//issues with the Edison Front End.
//
//Revision 1.9  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

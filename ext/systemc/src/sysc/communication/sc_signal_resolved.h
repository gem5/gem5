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

  sc_signal_resolved.h -- The resolved signal class.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_SIGNAL_RESOLVED_H
#define SC_SIGNAL_RESOLVED_H


#include "sysc/communication/sc_signal.h"

namespace sc_core {

class sc_process_b;

extern const sc_dt::sc_logic_value_t sc_logic_resolution_tbl[4][4];


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_resolved
//
//  The resolved signal class.
// ----------------------------------------------------------------------------

class sc_signal_resolved
: public sc_signal<sc_dt::sc_logic,SC_MANY_WRITERS>
{
public:

    // typedefs

    typedef sc_signal_resolved                         this_type;
    typedef sc_signal<sc_dt::sc_logic,SC_MANY_WRITERS> base_type;
    typedef sc_dt::sc_logic                            data_type;

public:

    // constructors

    sc_signal_resolved() :
        base_type( sc_gen_unique_name( "signal_resolved" ) ), m_proc_vec(), 
	m_val_vec()
        {}

    explicit sc_signal_resolved( const char* name_ ): 
        base_type( name_ ), m_proc_vec(), m_val_vec()
	{}

    sc_signal_resolved( const char* name_, const data_type & initial_value_ )
      : base_type( name_, initial_value_ )
      , m_proc_vec()
      , m_val_vec()
    {}

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
        { return "sc_signal_resolved"; }

protected:

    virtual void update();

protected:

    std::vector<sc_process_b*> m_proc_vec; // processes writing this signal
    std::vector<data_type>     m_val_vec;  // new values written this signal

private:

    // disabled
    sc_signal_resolved( const this_type& );
};

} // namespace sc_core

//$Log: sc_signal_resolved.h,v $
//Revision 1.6  2011/08/26 20:45:44  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.5  2011/08/24 22:05:36  acg
// Torsten Maehne: initialization changes to remove warnings.
//
//Revision 1.4  2011/04/19 02:36:26  acg
// Philipp A. Hartmann: new aysnc_update and mutex support.
//
//Revision 1.3  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
//
//Revision 1.2  2011/01/20 16:52:15  acg
// Andy Goodrich: changes for IEEE 1666 2011.
//
//Revision 1.1.1.1  2006/12/15 20:20:04  acg
//SystemC 2.3
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

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

  sc_signal_resolved_ports.h -- The sc_signal_resolved port classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-08-20

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_SIGNAL_RESOLVED_PORTS_H
#define SC_SIGNAL_RESOLVED_PORTS_H


#include "sysc/communication/sc_signal_ports.h"
#include "sysc/datatypes/bit/sc_logic.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_in_resolved
//
//  The sc_signal_resolved input port class.
// ----------------------------------------------------------------------------

class sc_in_resolved
    : public sc_in<sc_dt::sc_logic>
{
public:

    // typedefs

    typedef sc_dt::sc_logic            data_type;

    typedef sc_in_resolved             this_type;
    typedef sc_in<data_type>           base_type;

    typedef base_type::in_if_type      in_if_type;
    typedef base_type::in_port_type    in_port_type;
    typedef base_type::inout_port_type inout_port_type;

public:

    // constructors

    sc_in_resolved()
	: base_type()
	{}

    explicit sc_in_resolved( const char* name_ )
	: base_type( name_ )
	{}

    explicit sc_in_resolved( const in_if_type& interface_ )
	: base_type( interface_ )
	{}

    sc_in_resolved( const char* name_, const in_if_type& interface_ )
	: base_type( name_, interface_ )
	{}

    explicit sc_in_resolved( in_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_in_resolved( const char* name_, in_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    explicit sc_in_resolved( inout_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_in_resolved( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    sc_in_resolved( this_type& parent_ )
	: base_type( parent_ )
	{}

    sc_in_resolved( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ )
	{}


    // destructor (does nothing)

    virtual ~sc_in_resolved()
	{}


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();

    virtual const char* kind() const
        { return "sc_in_resolved"; }

private:

    // disabled
    sc_in_resolved( const this_type& );
    this_type& operator = ( const this_type& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_inout_resolved
//
//  The sc_signal_resolved input/output port class.
// ----------------------------------------------------------------------------

class sc_inout_resolved
    : public sc_inout<sc_dt::sc_logic>
{
public:

    // typedefs

    typedef sc_dt::sc_logic            data_type;

    typedef sc_inout_resolved          this_type;
    typedef sc_inout<data_type>        base_type;

    typedef base_type::in_if_type      in_if_type;
    typedef base_type::in_port_type    in_port_type;
    typedef base_type::inout_if_type   inout_if_type;
    typedef base_type::inout_port_type inout_port_type;

public:

    // constructors

    sc_inout_resolved()
	: base_type()
	{}

    explicit sc_inout_resolved( const char* name_ )
	: base_type( name_ )
	{}

    explicit sc_inout_resolved( inout_if_type& interface_ )
	: base_type( interface_ )
	{}

    sc_inout_resolved( const char* name_, inout_if_type& interface_ )
	: base_type( name_, interface_ )
	{}

    explicit sc_inout_resolved( inout_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_inout_resolved( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    sc_inout_resolved( this_type& parent_ )
	: base_type( parent_ )
	{}

    sc_inout_resolved( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ )
	{}


    // destructor (does nothing)

    virtual ~sc_inout_resolved()
	{}


    // write the new value

    this_type& operator = ( const data_type& value_ )
	{ (*this)->write( value_ ); return *this; }

    this_type& operator = ( const in_if_type& interface_ )
	{ (*this)->write( interface_.read() ); return *this; }

    this_type& operator = ( const in_port_type& port_ )
	{ (*this)->write( port_->read() ); return *this; }

    this_type& operator = ( const inout_port_type& port_ )
	{ (*this)->write( port_->read() ); return *this; }

    this_type& operator = ( const this_type& port_ )
	{ (*this)->write( port_->read() ); return *this; }


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();

    virtual const char* kind() const
        { return "sc_inout_resolved"; }

private:

    // disabled
    sc_inout_resolved( const this_type& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_out_resolved
//
//  The sc_signal_resolved output port class.
// ----------------------------------------------------------------------------

// sc_out_resolved can also read from its port, hence no difference with
// sc_inout_resolved. For debugging reasons, a class is provided instead
// of a typedef.

class sc_out_resolved
    : public sc_inout_resolved
{
public:

    // typedefs

    typedef sc_out_resolved            this_type;
    typedef sc_inout_resolved          base_type;

    typedef base_type::data_type       data_type;

    typedef base_type::in_if_type      in_if_type;
    typedef base_type::in_port_type    in_port_type;
    typedef base_type::inout_if_type   inout_if_type;
    typedef base_type::inout_port_type inout_port_type;

public:

    // constructors

    sc_out_resolved()
	: base_type()
	{}

    explicit sc_out_resolved( const char* name_ )
	: base_type( name_ )
	{}

    explicit sc_out_resolved( inout_if_type& interface_ )
	: base_type( interface_ )
	{}

    sc_out_resolved( const char* name_, inout_if_type& interface_ )
	: base_type( name_, interface_ )
	{}

    explicit sc_out_resolved( inout_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_out_resolved( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    sc_out_resolved( this_type& parent_ )
	: base_type( parent_ )
	{}

    sc_out_resolved( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ )
	{}


    // destructor (does nothing)

    virtual ~sc_out_resolved()
	{}


    // write the new value

    this_type& operator = ( const data_type& value_ )
	{ (*this)->write( value_ ); return *this; }

    this_type& operator = ( const in_if_type& interface_ )
	{ (*this)->write( interface_.read() ); return *this; }

    this_type& operator = ( const in_port_type& port_ )
	{ (*this)->write( port_->read() ); return *this; }

    this_type& operator = ( const inout_port_type& port_ )
	{ (*this)->write( port_->read() ); return *this; }

    this_type& operator = ( const this_type& port_ )
	{ (*this)->write( port_->read() ); return *this; }

    virtual const char* kind() const
        { return "sc_out_resolved"; }

private:

    // disabled
    sc_out_resolved( const this_type& );
};

} // namespace sc_core

//$Log: sc_signal_resolved_ports.h,v $
//Revision 1.3  2011/08/26 20:45:44  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.2  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
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
//Revision 1.9  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

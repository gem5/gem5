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

  sc_signal_rv_ports.h -- The resolved vector signal ports.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_SIGNAL_RV_PORTS_H
#define SC_SIGNAL_RV_PORTS_H


#include <cstdio>

#include "sysc/communication/sc_communication_ids.h"
#include "sysc/communication/sc_signal_ports.h"
#include "sysc/communication/sc_signal_rv.h"
#include "sysc/datatypes/bit/sc_lv.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_in_rv<W>
//
//  The sc_signal_rv<W> input port class.
// ----------------------------------------------------------------------------

template <int W>
class sc_in_rv
    : public sc_in<sc_dt::sc_lv<W> >
{
public:

    // typedefs

    typedef sc_dt::sc_lv<W>                     data_type;

    typedef sc_in_rv<W>                         this_type;
    typedef sc_in<data_type>                    base_type;

    typedef typename base_type::in_if_type      in_if_type;
    typedef typename base_type::in_port_type    in_port_type;
    typedef typename base_type::inout_port_type inout_port_type;

public:

    // constructors

    sc_in_rv()
	: base_type()
	{}

    explicit sc_in_rv( const char* name_ )
	: base_type( name_ )
	{}

    explicit sc_in_rv( const in_if_type& interface_ )
	: base_type( interface_ )
	{}

    sc_in_rv( const char* name_, const in_if_type& interface_ )
	: base_type( name_, interface_ )
	{}

    explicit sc_in_rv( in_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_in_rv( const char* name_, in_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    explicit sc_in_rv( inout_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_in_rv( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    sc_in_rv( this_type& parent_ )
	: base_type( parent_ )
	{}

    sc_in_rv( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ )
	{}


    // destructor (does nothing)

    virtual ~sc_in_rv()
	{}


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();

    virtual const char* kind() const
        { return "sc_in_rv"; }

private:

    // disabled
    sc_in_rv( const this_type& );
    this_type& operator = ( const this_type& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII


// called when elaboration is done

template <int W>
void
sc_in_rv<W>::end_of_elaboration()
{
    base_type::end_of_elaboration();
    // check if bound channel is a resolved signal
    if( DCAST<sc_signal_rv<W>*>( this->get_interface() ) == 0 ) {
	char msg[BUFSIZ];
	std::sprintf( msg, "%s (%s)", this->name(), kind() );
	SC_REPORT_ERROR( SC_ID_RESOLVED_PORT_NOT_BOUND_, msg );
    }
}


// ----------------------------------------------------------------------------
//  CLASS : sc_inout_rv<W>
//
//  The sc_signal_rv<W> input/output port class.
// ----------------------------------------------------------------------------

template <int W>
class sc_inout_rv
    : public sc_inout<sc_dt::sc_lv<W> >
{
public:

    // typedefs

    typedef sc_dt::sc_lv<W>                     data_type;

    typedef sc_inout_rv<W>                      this_type;
    typedef sc_inout<data_type>                 base_type;

    typedef typename base_type::in_if_type      in_if_type;
    typedef typename base_type::in_port_type    in_port_type;
    typedef typename base_type::inout_if_type   inout_if_type;
    typedef typename base_type::inout_port_type inout_port_type;

public:

    // constructors

    sc_inout_rv()
	: base_type()
	{}

    explicit sc_inout_rv( const char* name_ )
	: base_type( name_ )
	{}

    explicit sc_inout_rv( inout_if_type& interface_ )
	: base_type( interface_ )
	{}

    sc_inout_rv( const char* name_, inout_if_type& interface_ )
	: base_type( name_, interface_ )
	{}

    explicit sc_inout_rv( inout_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_inout_rv( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    sc_inout_rv( this_type& parent_ )
	: base_type( parent_ )
	{}

    sc_inout_rv( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ )
	{}


    // destructor (does nothing)

    virtual ~sc_inout_rv()
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
        { return "sc_inout_rv"; }

private:

    // disabled
    sc_inout_rv( const this_type& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII


// called when elaboration is done

template <int W>
void
sc_inout_rv<W>::end_of_elaboration()
{
    base_type::end_of_elaboration();
    // check if bound channel is a resolved signal
    if( DCAST<sc_signal_rv<W>*>( this->get_interface() ) == 0 ) {
	char msg[BUFSIZ];
	std::sprintf( msg, "%s (%s)", this->name(), kind() );
	SC_REPORT_ERROR( SC_ID_RESOLVED_PORT_NOT_BOUND_, msg );
    }
}


// ----------------------------------------------------------------------------
//  CLASS : sc_out_rv<W>
//
//  The sc_signal_rv<W> output port class.
// ----------------------------------------------------------------------------

// sc_out_rv can also read from its port, hence no difference with
// sc_inout_rv. For debugging reasons, a class is provided instead
// of a define.

template <int W>
class sc_out_rv
    : public sc_inout_rv<W>
{
public:

    // typedefs

    typedef sc_out_rv<W>                        this_type;
    typedef sc_inout_rv<W>                      base_type;

    typedef typename base_type::data_type       data_type;

    typedef typename base_type::in_if_type      in_if_type;
    typedef typename base_type::in_port_type    in_port_type;
    typedef typename base_type::inout_if_type   inout_if_type;
    typedef typename base_type::inout_port_type inout_port_type;

public:

    // constructors

    sc_out_rv()
	: base_type()
	{}

    explicit sc_out_rv( const char* name_ )
	: base_type( name_ )
	{}

    explicit sc_out_rv( inout_if_type& interface_ )
	: base_type( interface_ )
	{}

    sc_out_rv( const char* name_, inout_if_type& interface_ )
	: base_type( name_, interface_ )
	{}

    explicit sc_out_rv( inout_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_out_rv( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    sc_out_rv( this_type& parent_ )
	: base_type( parent_ )
	{}

    sc_out_rv( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ )
	{}


    // destructor (does nothing)

    virtual ~sc_out_rv()
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
        { return "sc_out_rv"; }

private:

    // disabled
    sc_out_rv( const this_type& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

} // namespace sc_core

//$Log: sc_signal_rv_ports.h,v $
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
//Revision 1.2  2006/01/03 23:18:27  acg
//Changed copyright to include 2006.
//
//Revision 1.1.1.1  2005/12/19 23:16:43  acg
//First check in of SystemC 2.1 into its own archive.
//
//Revision 1.11  2005/09/15 23:01:52  acg
//Added std:: prefix to appropriate methods and types to get around
//issues with the Edison Front End.
//
//Revision 1.10  2005/06/10 22:43:56  acg
//Added CVS change log annotation.
//

#endif

// Taf!

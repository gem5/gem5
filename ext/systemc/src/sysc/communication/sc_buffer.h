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

  sc_buffer.h -- The sc_buffer<T> primitive channel class.
                 Like sc_signal<T>, but *every* write causes an event.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_BUFFER_H
#define SC_BUFFER_H


#include "sysc/communication/sc_signal.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_buffer<T>
//
//  The sc_buffer<T> primitive channel class.
// ----------------------------------------------------------------------------

template< typename T, sc_writer_policy POL = SC_DEFAULT_WRITER_POLICY >
class sc_buffer
: public sc_signal<T,POL>
{
public:

    // typedefs

    typedef sc_buffer<T,POL> this_type;
    typedef sc_signal<T,POL> base_type;

public:

    // constructors

    sc_buffer()
	: base_type( sc_gen_unique_name( "buffer" ) )
	{}

    explicit sc_buffer( const char* name_ )
	: base_type( name_ )
	{}

    sc_buffer( const char* name_, const T& initial_value_ )
      : base_type( name_, initial_value_ )
    {}

    // interface methods

    // write the new value
    virtual void write( const T& );


    // other methods

    this_type& operator = ( const T& a )
	{ write( a ); return *this; }

    this_type& operator = ( const sc_signal_in_if<T>& a )
	{ write( a.read() ); return *this; }

    this_type& operator = ( const this_type& a )
	{ write( a.read() ); return *this; }

    virtual const char* kind() const
        { return "sc_buffer"; }

protected:

    virtual void update();

private:

    // disabled
    sc_buffer( const this_type& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// write the new value

template< typename T, sc_writer_policy POL >
inline
void
sc_buffer<T,POL>::write( const T& value_ )
{
    if( !base_type::policy_type::check_write(this,true) )
      return;

    this->m_new_val = value_;
    this->request_update();
}


template< typename T, sc_writer_policy POL >
inline
void
sc_buffer<T,POL>::update()
{
    base_type::policy_type::update();
    base_type::do_update();
}

} // namespace sc_core

#endif

//$Log: sc_buffer.h,v $
//Revision 1.7  2011/08/26 20:45:39  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.6  2011/04/08 18:22:45  acg
// Philipp A. Hartmann: use the context of the primitive channel to get
// the change stamp value.
//
//Revision 1.5  2011/04/05 20:48:09  acg
// Andy Goodrich: changes to make sure that event(), posedge() and negedge()
// only return true if the clock has not moved.
//
//Revision 1.4  2011/04/05 06:15:18  acg
// Philipp A. Hartmann: sc_writer_policy: ignore no-ops in delta check.
//
//Revision 1.3  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
//
//Revision 1.2  2010/12/07 19:50:36  acg
// Andy Goodrich: addition of writer policies, courtesy of Philipp Hartmann.
//
//Revision 1.1.1.1  2006/12/15 20:20:04  acg
//SystemC 2.3
//
//Revision 1.8  2006/03/13 20:19:43  acg
// Andy Goodrich: changed sc_event instances into pointers to sc_event instances
// that are allocated as needed. This saves considerable storage for large
// numbers of signals, etc.
//
//Revision 1.7  2006/01/26 21:00:49  acg
// Andy Goodrich: conversion to use sc_event::notify(SC_ZERO_TIME) instead of
// sc_event::notify_delayed()
//
//Revision 1.6  2006/01/24 20:46:31  acg
//Andy Goodrich: changes to eliminate use of deprecated features. For instance,
//using notify(SC_ZERO_TIME) in place of notify_delayed().
//
//Revision 1.5  2006/01/19 19:18:25  acg
//Andy Goodrich: eliminated check_writer in favor of inline code within the
//write() method since we always execute the check_writer code even when
//check writing is turned off.
//
//Revision 1.4  2006/01/19 00:30:57  acg
//Andy Goodrich: Yet another implementation for disabling write checks on
//signals. This version uses an environment variable, SC_SIGNAL_WRITE_CHECK,
//that when set to DISABLE will turn off write checking.
//
//Revision 1.3  2006/01/13 18:47:20  acg
//Reversed sense of multiwriter signal check. It now defaults to ON unless the
//user defines SC_NO_WRITE_CHEK before inclusion of the file.
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

// Taf!

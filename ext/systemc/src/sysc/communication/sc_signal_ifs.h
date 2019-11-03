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

  sc_signal_ifs.h -- The sc_signal<T> interface classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_SIGNAL_IFS_H
#define SC_SIGNAL_IFS_H


#include "sysc/communication/sc_interface.h"
#include "sysc/communication/sc_writer_policy.h"


namespace sc_dt
{
    class sc_logic;
}

namespace sc_core {

class sc_signal_bool_deval;
class sc_signal_logic_deval;


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_in_if<T>
//
//  The sc_signal<T> input interface class.
// ----------------------------------------------------------------------------

template <class T>
class sc_signal_in_if
: virtual public sc_interface
{
public:

    // get the value changed event
    virtual const sc_event& value_changed_event() const = 0;


    // read the current value
    virtual const T& read() const = 0;

    // get a reference to the current value (for tracing)
    virtual const T& get_data_ref() const = 0;


    // was there a value changed event?
    virtual bool event() const = 0;

protected:

    // constructor

    sc_signal_in_if()
	{}

private:

    // disabled
    sc_signal_in_if( const sc_signal_in_if<T>& );
    sc_signal_in_if<T>& operator = ( const sc_signal_in_if<T>& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_in_if<bool>
//
//  Specialization of sc_signal_in_if<T> for type bool.
// ----------------------------------------------------------------------------

class sc_reset;

template <>
class sc_signal_in_if<bool>
: virtual public sc_interface
{
    friend class sc_reset;
public:

    // get the value changed event
    virtual const sc_event& value_changed_event() const = 0;

    // get the positive edge event
    virtual const sc_event& posedge_event() const = 0;

    // get the negative edge event
    virtual const sc_event& negedge_event() const = 0;


    // read the current value
    virtual const bool& read() const = 0;

    // get a reference to the current value (for tracing)
    virtual const bool& get_data_ref() const = 0;


    // was there a value changed event?
    virtual bool event() const = 0;

    // was there a positive edge event?
    virtual bool posedge() const = 0;

    // was there a negative edge event?
    virtual bool negedge() const = 0;

protected:

    // constructor

    sc_signal_in_if()
	{}

private:

    // disabled
    sc_signal_in_if( const sc_signal_in_if<bool>& );
    sc_signal_in_if<bool>& operator = ( const sc_signal_in_if<bool>& );

    // designate this object as a reset signal
    virtual sc_reset* is_reset() const
      { return NULL; }
};


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_in_if<sc_dt::sc_logic>
//
//  Specialization of sc_signal_in_if<T> for type sc_dt::sc_logic.
// ----------------------------------------------------------------------------

template <>
class sc_signal_in_if<sc_dt::sc_logic>
: virtual public sc_interface
{
public:

    // get the value changed event
    virtual const sc_event& value_changed_event() const = 0;

    // get the positive edge event
    virtual const sc_event& posedge_event() const = 0;

    // get the negative edge event
    virtual const sc_event& negedge_event() const = 0;


    // read the current value
    virtual const sc_dt::sc_logic& read() const = 0;

    // get a reference to the current value (for tracing)
    virtual const sc_dt::sc_logic& get_data_ref() const = 0;


    // was there a value changed event?
    virtual bool event() const = 0;

    // was there a positive edge event?
    virtual bool posedge() const = 0;

    // was there a negative edge event?
    virtual bool negedge() const = 0;


protected:

    // constructor

    sc_signal_in_if()
	{}

private:

    // disabled
    sc_signal_in_if( const sc_signal_in_if<sc_dt::sc_logic>& );
    sc_signal_in_if<sc_dt::sc_logic>& operator = (
	const sc_signal_in_if<sc_dt::sc_logic>& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_write_if<T>
//
//  The standard output interface class.
// ----------------------------------------------------------------------------
template< typename T >
class sc_signal_write_if : public virtual sc_interface
{
public:
    sc_signal_write_if() {}
    // write the new value
    virtual void write( const T& ) = 0;
    virtual sc_writer_policy get_writer_policy() const
        { return SC_DEFAULT_WRITER_POLICY; }
private:
    // disabled
    sc_signal_write_if( const sc_signal_write_if<T>& );
    sc_signal_write_if<T>& operator = ( const sc_signal_write_if<T>& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_inout_if<T>
//
//  The sc_signal<T> input/output interface class.
// ----------------------------------------------------------------------------

template <class T>
class sc_signal_inout_if
: public sc_signal_in_if<T>, public sc_signal_write_if<T>
{

protected:

    // constructor

    sc_signal_inout_if()
	{}

private:

    // disabled
    sc_signal_inout_if( const sc_signal_inout_if<T>& );
    sc_signal_inout_if<T>& operator = ( const sc_signal_inout_if<T>& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_signal_out_if<T>
//
//  The sc_signal<T> output interface class.
// ----------------------------------------------------------------------------

// sc_signal_out_if can also be read from, hence no difference with
// sc_signal_inout_if.

#define sc_signal_out_if sc_signal_inout_if

} // namespace sc_core

//$Log: sc_signal_ifs.h,v $
//Revision 1.4  2011/08/26 20:45:43  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.3  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
//
//Revision 1.2  2010/12/07 19:50:37  acg
// Andy Goodrich: addition of writer policies, courtesy of Philipp Hartmann.
//
//Revision 1.1.1.1  2006/12/15 20:20:04  acg
//SystemC 2.3
//
//Revision 1.4  2006/08/29 23:35:00  acg
// Andy Goodrich: added bind_count() method to allow users to determine which
// ports are connected in before_end_of_elaboration().
//
//Revision 1.3  2006/04/11 23:11:57  acg
//  Andy Goodrich: Changes for reset support that only includes
//  sc_cthread_process instances.
//
//Revision 1.2  2006/01/03 23:18:26  acg
//Changed copyright to include 2006.
//
//Revision 1.1.1.1  2005/12/19 23:16:43  acg
//First check in of SystemC 2.1 into its own archive.
//
//Revision 1.10  2005/09/15 23:01:51  acg
//Added std:: prefix to appropriate methods and types to get around
//issues with the Edison Front End.
//
//Revision 1.9  2005/06/29 18:12:12  acg
//Added $log.
//
//Revision 1.8  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

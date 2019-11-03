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

  sc_signal_ports.h -- The sc_signal<T> port classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG APPEARS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_SIGNAL_PORTS_H
#define SC_SIGNAL_PORTS_H


#include "sysc/communication/sc_event_finder.h"
#include "sysc/communication/sc_port.h"
#include "sysc/communication/sc_signal_ifs.h"
#include "sysc/datatypes/bit/sc_logic.h"
#include "sysc/tracing/sc_trace.h"

#if ! defined( SC_DISABLE_VIRTUAL_BIND )
#  define SC_VIRTUAL_ virtual
#else
#  define SC_VIRTUAL_ /* non-virtual */
#endif

namespace sc_core {

// ----------------------------------------------------------------------------
//  STRUCT : sc_trace_params
//
//  Struct for storing the trace file and object name of an sc_trace call.
//  FOR INTERNAL USE ONLY!
// ----------------------------------------------------------------------------

extern void sc_deprecated_add_trace();

struct sc_trace_params
{
    sc_trace_file*        tf;
    std::string      name;

    sc_trace_params( sc_trace_file* tf_, const std::string& name_ )
	: tf( tf_ ), name( name_ )
	{}
};


typedef std::vector<sc_trace_params*> sc_trace_params_vec;


// ----------------------------------------------------------------------------
//  CLASS : sc_in<T>
//
//  The sc_signal<T> input port class.
// ----------------------------------------------------------------------------

template <class T>
class sc_in
: public sc_port<sc_signal_in_if<T>,1,SC_ONE_OR_MORE_BOUND>
{
public:

    // typedefs

    typedef T                                             data_type;

    typedef sc_signal_in_if<data_type>                    if_type;
    typedef sc_port<if_type,1,SC_ONE_OR_MORE_BOUND>       base_type;
    typedef sc_in<data_type>                              this_type;
    typedef typename base_type::port_type                 base_port_type;

    typedef if_type                                       in_if_type;
    typedef base_type                                     in_port_type;
    typedef sc_signal_inout_if<data_type>                 inout_if_type;
    typedef sc_port<inout_if_type,1,SC_ONE_OR_MORE_BOUND> inout_port_type;

public:

    // constructors

    sc_in()
	: base_type(), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    explicit sc_in( const char* name_ )
	: base_type( name_ ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    explicit sc_in( const in_if_type& interface_ )
        : base_type( CCAST<in_if_type&>( interface_ ) ), m_traces( 0 ),
	  m_change_finder_p(0)
        {}

    sc_in( const char* name_, const in_if_type& interface_ )
	: base_type( name_, CCAST<in_if_type&>( interface_ ) ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    explicit sc_in( in_port_type& parent_ )
	: base_type( parent_ ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    sc_in( const char* name_, in_port_type& parent_ )
	: base_type( name_, parent_ ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    explicit sc_in( inout_port_type& parent_ )
	: base_type(), m_traces( 0 ),
	  m_change_finder_p(0)
	{ sc_port_base::bind( parent_ ); }

    sc_in( const char* name_, inout_port_type& parent_ )
	: base_type( name_ ), m_traces( 0 ),
	  m_change_finder_p(0)
	{ sc_port_base::bind( parent_ ); }

    sc_in( this_type& parent_ )
	: base_type( parent_ ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    sc_in( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}


    // destructor

    virtual ~sc_in()
	{
	    remove_traces();
	    delete m_change_finder_p;
	}


    // bind to in interface

    SC_VIRTUAL_ void bind( const in_if_type& interface_ )
	{ sc_port_base::bind( CCAST<in_if_type&>( interface_ ) ); }

    SC_VIRTUAL_ void bind( in_if_type& interface_ )
	{ this->bind( CCAST<const in_if_type&>( interface_ ) ); }

    void operator () ( const in_if_type& interface_ )
	{ this->bind( interface_ ); }


    // bind to parent in port

    SC_VIRTUAL_ void bind( in_port_type& parent_ )
        { sc_port_base::bind( parent_ ); }

    void operator () ( in_port_type& parent_ )
        { this->bind( parent_ ); }


    // bind to parent inout port

    SC_VIRTUAL_ void bind( inout_port_type& parent_ )
	{ sc_port_base::bind( parent_ ); }

    void operator () ( inout_port_type& parent_ )
	{ this->bind( parent_ ); }


    // interface access shortcut methods

    // get the default event

    const sc_event& default_event() const
	{ return (*this)->default_event(); }


    // get the value changed event

    const sc_event& value_changed_event() const
	{ return (*this)->value_changed_event(); }


    // read the current value

    const data_type& read() const
	{ return (*this)->read(); }

    operator const data_type& () const
	{ return (*this)->read(); }


    // was there a value changed event?

    bool event() const
	{ return (*this)->event(); }


    // (other) event finder method(s)

    sc_event_finder& value_changed() const
    {
        if ( !m_change_finder_p )
	{
	    m_change_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::value_changed_event );
	}
	return *m_change_finder_p;
    }


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();

    virtual const char* kind() const
        { return "sc_in"; }


    void add_trace( sc_trace_file*, const std::string& ) const;

    // called by sc_trace
    void add_trace_internal( sc_trace_file*, const std::string& ) const;

protected:

    void remove_traces() const;

    mutable sc_trace_params_vec* m_traces;

protected:

    // called by pbind (for internal use only)
    virtual int vbind( sc_interface& );
    virtual int vbind( sc_port_base& );

    // implement virtual base_type port-binding function
    //  - avoids warnings on some compilers
    //  - should only be called, when using sc_port_b explicitly
    //  - errors are detected during elaboration

    SC_VIRTUAL_ void bind( base_port_type& parent_ )
        { sc_port_base::bind( parent_ ); }


private:
  mutable sc_event_finder* m_change_finder_p;

private:

    // disabled
    sc_in( const this_type& );
    this_type& operator = ( const this_type& );

#ifdef __GNUC__
    // Needed to circumvent a problem in the g++-2.95.2 compiler:
    // This unused variable forces the compiler to instantiate
    // an object of T template so an implicit conversion from
    // read() to a C++ intrinsic data type will work.
    static data_type dummy;
#endif
};

template<typename T>
::std::ostream& operator << ( ::std::ostream& os, const sc_in<T>& a )
{
    return os << a->read();
}

// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII


// called when elaboration is done

template <class T>
inline
void
sc_in<T>::end_of_elaboration()
{
    if( m_traces != 0 ) {
	for( int i = 0; i < (int)m_traces->size(); ++ i ) {
	    sc_trace_params* p = (*m_traces)[i];
	    in_if_type* iface = DCAST<in_if_type*>( this->get_interface() );
	    sc_trace( p->tf, iface->read(), p->name );
	}
	remove_traces();
    }
}


// called by sc_trace

template <class T>
inline
void
sc_in<T>::add_trace_internal( sc_trace_file* tf_, const std::string& name_ ) 
const
{
    if( tf_ != 0 ) {
	if( m_traces == 0 ) {
	    m_traces = new sc_trace_params_vec;
	}
	m_traces->push_back( new sc_trace_params( tf_, name_ ) );
    }
}

template <class T>
inline
void
sc_in<T>::add_trace( sc_trace_file* tf_, const std::string& name_ ) 
const
{
    sc_deprecated_add_trace();
    add_trace_internal(tf_, name_);
}

template <class T>
inline
void
sc_in<T>::remove_traces() const
{
    if( m_traces != 0 ) {
	for( int i = (int)m_traces->size() - 1; i >= 0; -- i ) {
	    delete (*m_traces)[i];
	}
	delete m_traces;
	m_traces = 0;
    }
}


// called by pbind (for internal use only)

template <class T>
inline
int
sc_in<T>::vbind( sc_interface& interface_ )
{
    return sc_port_b<if_type>::vbind( interface_ );
}

template <class T>
inline
int
sc_in<T>::vbind( sc_port_base& parent_ )
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
//  CLASS : sc_in<bool>
//
//  Specialization of sc_in<T> for type bool.
// ----------------------------------------------------------------------------

template <>
class sc_in<bool> : 
    public sc_port<sc_signal_in_if<bool>,1,SC_ONE_OR_MORE_BOUND>
{
public:

    // typedefs

    typedef bool                                           data_type;

    typedef sc_signal_in_if<data_type>                     if_type;
    typedef sc_port<if_type,1,SC_ONE_OR_MORE_BOUND>        base_type;
    typedef sc_in<data_type>                               this_type;
    typedef /* typename */ base_type::port_type            base_port_type;

    typedef if_type                                        in_if_type;
    typedef base_type                                      in_port_type;
    typedef sc_signal_inout_if<data_type>                  inout_if_type;
    typedef sc_port<inout_if_type,1,SC_ONE_OR_MORE_BOUND>  inout_port_type;

public:

    // constructors

    sc_in()
	: base_type(), m_traces( 0 ), m_change_finder_p(0), 
	  m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_in( const char* name_ )
	: base_type( name_ ), m_traces( 0 ), m_change_finder_p(0), 
	  m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_in( const in_if_type& interface_ )
	: base_type( CCAST<in_if_type&>( interface_ ) ), m_traces( 0 ), 
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_in( const char* name_, const in_if_type& interface_ )
	: base_type( name_, CCAST<in_if_type&>( interface_ ) ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_in( in_port_type& parent_ )
	: base_type( parent_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_in( const char* name_, in_port_type& parent_ )
	: base_type( name_, parent_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_in( inout_port_type& parent_ )
	: base_type(), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{ sc_port_base::bind( parent_ ); }

    sc_in( const char* name_, inout_port_type& parent_ )
	: base_type( name_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{ sc_port_base::bind( parent_ ); }

    sc_in( this_type& parent_ )
	: base_type( parent_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

#if defined(TESTING)
    sc_in( const this_type& parent_ )
	: base_type( *(in_if_type*)parent_.get_interface() ) , m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}
#endif 

    sc_in( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}


    // destructor

    virtual ~sc_in()
	{
	    remove_traces();
	    delete m_change_finder_p;
	    delete m_neg_finder_p;
	    delete m_pos_finder_p;
	}


    // bind to in interface

    SC_VIRTUAL_ void bind( const in_if_type& interface_ )
	{ sc_port_base::bind( CCAST<in_if_type&>( interface_ ) ); }

    SC_VIRTUAL_ void bind( in_if_type& interface_ )
	{ this->bind( CCAST<const in_if_type&>( interface_ ) ); }

    void operator () ( const in_if_type& interface_ )
	{ this->bind( interface_ ); }


    // bind to parent in port

    SC_VIRTUAL_ void bind( in_port_type& parent_ )
        { sc_port_base::bind( parent_ ); }

    void operator () ( in_port_type& parent_ )
        { this->bind( parent_ ); }


    // bind to parent inout port

    SC_VIRTUAL_ void bind( inout_port_type& parent_ )
	{ sc_port_base::bind( parent_ ); }

    void operator () ( inout_port_type& parent_ )
	{ this->bind( parent_ ); }


    // interface access shortcut methods

    // get the default event

    const sc_event& default_event() const
	{ return (*this)->default_event(); }


    // get the value changed event

    const sc_event& value_changed_event() const
	{ return (*this)->value_changed_event(); }

    // get the positive edge event

    const sc_event& posedge_event() const
	{ return (*this)->posedge_event(); }

    // get the negative edge event

    const sc_event& negedge_event() const
	{ return (*this)->negedge_event(); }


    // read the current value

    const data_type& read() const
	{ return (*this)->read(); }

    operator const data_type& () const
	{ return (*this)->read(); }


    // use for positive edge sensitivity

    sc_event_finder& pos() const
    {
        if ( !m_pos_finder_p )
	{
	    m_pos_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::posedge_event );
	} 
	return *m_pos_finder_p;
    }

    // use for negative edge sensitivity

    sc_event_finder& neg() const
    {
        if ( !m_neg_finder_p )
	{
	    m_neg_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::negedge_event );
	} 
	return *m_neg_finder_p;
    }


    // was there a value changed event?

    bool event() const
	{ return (*this)->event(); }

    // was there a positive edge event?

    bool posedge() const
        { return (*this)->posedge(); }

    // was there a negative edge event?

    bool negedge() const
        { return (*this)->negedge(); }

    // (other) event finder method(s)

    sc_event_finder& value_changed() const
    {
        if ( !m_change_finder_p )
	{
	    m_change_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::value_changed_event );
	}
	return *m_change_finder_p;
    }


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();

    virtual const char* kind() const
        { return "sc_in"; }


    void add_trace( sc_trace_file*, const std::string& ) const;

    // called by sc_trace
    void add_trace_internal( sc_trace_file*, const std::string& ) const;

protected:

    void remove_traces() const;

    mutable sc_trace_params_vec* m_traces;

protected:

    // called by pbind (for internal use only)
    virtual int vbind( sc_interface& );
    virtual int vbind( sc_port_base& );

    // implement virtual base_type port-binding function
    //  - avoids warnings on some compilers
    //  - should only be called, when using sc_port_b explicitly
    //  - errors are detected during elaboration

    SC_VIRTUAL_ void bind( base_port_type& parent_ )
        { sc_port_base::bind( parent_ ); }

private:
  mutable sc_event_finder* m_change_finder_p;
  mutable sc_event_finder* m_neg_finder_p;
  mutable sc_event_finder* m_pos_finder_p;

private:

    // disabled
#if defined(TESTING)
#else
    sc_in( const this_type& );
#endif 
    this_type& operator = ( const this_type& );

#ifdef __GNUC__
    // Needed to circumvent a problem in the g++-2.95.2 compiler:
    // This unused variable forces the compiler to instantiate
    // an object of T template so an implicit conversion from
    // read() to a C++ intrinsic data type will work.
    static data_type dummy;
#endif
};


// ----------------------------------------------------------------------------
//  CLASS : sc_in<sc_dt::sc_logic>
//
//  Specialization of sc_in<T> for type sc_dt::sc_logic.
// ----------------------------------------------------------------------------

template <>
class sc_in<sc_dt::sc_logic>
: public sc_port<sc_signal_in_if<sc_dt::sc_logic>,1,SC_ONE_OR_MORE_BOUND>
{
public:

    // typedefs

    typedef sc_dt::sc_logic                               data_type;

    typedef sc_signal_in_if<data_type>                    if_type;
    typedef sc_port<if_type,1,SC_ONE_OR_MORE_BOUND>       base_type;
    typedef sc_in<data_type>                              this_type;
    typedef /* typename */ base_type::port_type           base_port_type;

    typedef if_type                                       in_if_type;
    typedef base_type                                     in_port_type;
    typedef sc_signal_inout_if<data_type>                 inout_if_type;
    typedef sc_port<inout_if_type,1,SC_ONE_OR_MORE_BOUND> inout_port_type;

public:

    // constructors

    sc_in()
	: base_type(), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_in( const char* name_ )
	: base_type( name_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_in( const in_if_type& interface_ )
	: base_type( CCAST<in_if_type&>( interface_ ) ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_in( const char* name_, const in_if_type& interface_ )
	: base_type( name_, CCAST<in_if_type&>( interface_ ) ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_in( in_port_type& parent_ )
	: base_type( parent_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_in( const char* name_, in_port_type& parent_ )
	: base_type( name_, parent_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_in( inout_port_type& parent_ )
	: base_type(), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{ sc_port_base::bind( parent_ ); }

    sc_in( const char* name_, inout_port_type& parent_ )
	: base_type( name_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{ sc_port_base::bind( parent_ ); }

    sc_in( this_type& parent_ )
	: base_type( parent_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_in( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}


    // destructor

    virtual ~sc_in()
	{
	    remove_traces();
	    delete m_change_finder_p;
	    delete m_neg_finder_p;
	    delete m_pos_finder_p;
	}


    // bind to in interface

    SC_VIRTUAL_ void bind( const in_if_type& interface_ )
	{ sc_port_base::bind( CCAST<in_if_type&>( interface_ ) ); }

    SC_VIRTUAL_ void bind( in_if_type& interface_ )
	{ this->bind( CCAST<const in_if_type&>( interface_ ) ); }

    void operator () ( const in_if_type& interface_ )
	{ this->bind( interface_ ); }


    // bind to parent in port

    SC_VIRTUAL_ void bind( in_port_type& parent_ )
        { sc_port_base::bind( parent_ ); }

    void operator () ( in_port_type& parent_ )
        { this->bind( parent_ ); }


    // bind to parent inout port

    SC_VIRTUAL_ void bind( inout_port_type& parent_ )
	{ sc_port_base::bind( parent_ ); }

    void operator () ( inout_port_type& parent_ )
	{ this->bind( parent_ ); }


    // interface access shortcut methods

    // get the default event

    const sc_event& default_event() const
	{ return (*this)->default_event(); }


    // get the value changed event

    const sc_event& value_changed_event() const
	{ return (*this)->value_changed_event(); }

    // get the positive edge event

    const sc_event& posedge_event() const
	{ return (*this)->posedge_event(); }

    // get the negative edge event

    const sc_event& negedge_event() const
	{ return (*this)->negedge_event(); }


    // read the current value

    const data_type& read() const
	{ return (*this)->read(); }

    operator const data_type& () const
	{ return (*this)->read(); }


    // use for positive edge sensitivity

    sc_event_finder& pos() const
    {
        if ( !m_pos_finder_p )
	{
	    m_pos_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::posedge_event );
	} 
	return *m_pos_finder_p;
    }

    // use for negative edge sensitivity

    sc_event_finder& neg() const
    {
        if ( !m_neg_finder_p )
	{
	    m_neg_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::negedge_event );
	} 
	return *m_neg_finder_p;
    }


    // was there a value changed event?

    bool event() const
	{ return (*this)->event(); }

    // was there a positive edge event?

    bool posedge() const
        { return (*this)->posedge(); }

    // was there a negative edge event?

    bool negedge() const
        { return (*this)->negedge(); }

    // (other) event finder method(s)

    sc_event_finder& value_changed() const
    {
        if ( !m_change_finder_p )
	{
	    m_change_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::value_changed_event );
	}
	return *m_change_finder_p;
    }


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();

    virtual const char* kind() const
        { return "sc_in"; }


    void add_trace( sc_trace_file*, const std::string& ) const;

    // called by sc_trace
    void add_trace_internal( sc_trace_file*, const std::string& ) const;

protected:

    void remove_traces() const;

    mutable sc_trace_params_vec* m_traces;

protected:

    // called by pbind (for internal use only)
    virtual int vbind( sc_interface& );
    virtual int vbind( sc_port_base& );

    // implement virtual base_type port-binding function
    //  - avoids warnings on some compilers
    //  - should only be called, when using sc_port_b explicitly
    //  - errors are detected during elaboration

    SC_VIRTUAL_ void bind( base_port_type& parent_ )
        { sc_port_base::bind( parent_ ); }

private:
  mutable sc_event_finder* m_change_finder_p;
  mutable sc_event_finder* m_neg_finder_p;
  mutable sc_event_finder* m_pos_finder_p;

private:

    // disabled
    sc_in( const this_type& );
    this_type& operator = ( const this_type& );

#ifdef __GNUC__
    // Needed to circumvent a problem in the g++-2.95.2 compiler:
    // This unused variable forces the compiler to instantiate
    // an object of T template so an implicit conversion from
    // read() to a C++ intrinsic data type will work.
    static data_type dummy;
#endif
};


// ----------------------------------------------------------------------------
//  CLASS : sc_inout<T>
//
//  The sc_signal<T> input/output port class.
// ----------------------------------------------------------------------------

template <class T>
class sc_inout
: public sc_port<sc_signal_inout_if<T>,1,SC_ONE_OR_MORE_BOUND>
{
public:

    // typedefs

    typedef T                                          data_type;

    typedef sc_signal_inout_if<data_type>              if_type;
    typedef sc_port<if_type,1,SC_ONE_OR_MORE_BOUND>    base_type;
    typedef sc_inout<data_type>                        this_type;

    typedef sc_signal_in_if<data_type>                 in_if_type;
    typedef sc_port<in_if_type,1,SC_ONE_OR_MORE_BOUND> in_port_type;
    typedef if_type                                    inout_if_type;
    typedef base_type                                  inout_port_type;

public:

    // constructors

    sc_inout()
	: base_type(), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    explicit sc_inout( const char* name_ )
	: base_type( name_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    explicit sc_inout( inout_if_type& interface_ )
	: base_type( interface_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    sc_inout( const char* name_, inout_if_type& interface_ )
	: base_type( name_, interface_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    explicit sc_inout( inout_port_type& parent_ )
	: base_type( parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    sc_inout( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    sc_inout( this_type& parent_ )
	: base_type( parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}

    sc_inout( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0)
	{}


    // destructor

    virtual ~sc_inout();


    // interface access shortcut methods

    // get the default event

    const sc_event& default_event() const
	{ return (*this)->default_event(); }


    // get the value changed event

    const sc_event& value_changed_event() const
	{ return (*this)->value_changed_event(); }


    // read the current value

    const data_type& read() const
	{ return (*this)->read(); }

    operator const data_type& () const
	{ return (*this)->read(); }


    // was there a value changed event?

    bool event() const
	{ return (*this)->event(); }


    // write the new value

    void write( const data_type& value_ )
	{ (*this)->write( value_ ); }

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


    // set initial value (can also be called when port is not bound yet)

    void initialize( const data_type& value_ );

    void initialize( const in_if_type& interface_ )
	{ initialize( interface_.read() ); }


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();


    // (other) event finder method(s)

    sc_event_finder& value_changed() const
    {
        if ( !m_change_finder_p )
	{
	    m_change_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::value_changed_event );
	}
	return *m_change_finder_p;
    }

    virtual const char* kind() const
        { return "sc_inout"; }

protected:

    data_type* m_init_val;

public:

    // called by sc_trace
    void add_trace_internal( sc_trace_file*, const std::string& ) const;

    void add_trace( sc_trace_file*, const std::string& ) const;

protected:

    void remove_traces() const;

    mutable sc_trace_params_vec* m_traces;

private:
  mutable sc_event_finder* m_change_finder_p;

private:

    // disabled
    sc_inout( const this_type& );

#ifdef __GNUC__
    // Needed to circumvent a problem in the g++-2.95.2 compiler:
    // This unused variable forces the compiler to instantiate
    // an object of T template so an implicit conversion from
    // read() to a C++ intrinsic data type will work.
    static data_type dummy;
#endif
};

template<typename T>
::std::ostream& operator << ( ::std::ostream& os, const sc_inout<T>& a )
{
    return os << a->read();
}

// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII


// destructor

template <class T>
inline
sc_inout<T>::~sc_inout()
{
    delete m_change_finder_p;
    delete m_init_val;
    remove_traces();
}


// set initial value (can also be called when port is not bound yet)

template <class T>
inline
void
sc_inout<T>::initialize( const data_type& value_ )
{
    inout_if_type* iface = DCAST<inout_if_type*>( this->get_interface() );
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

template <class T>
inline
void
sc_inout<T>::end_of_elaboration()
{
    if( m_init_val != 0 ) {
	write( *m_init_val );
	delete m_init_val;
	m_init_val = 0;
    }
    if( m_traces != 0 ) {
	for( int i = 0; i < (int)m_traces->size(); ++ i ) {
	    sc_trace_params* p = (*m_traces)[i];
	    in_if_type* iface = DCAST<in_if_type*>( this->get_interface() );
	    sc_trace( p->tf, iface->read(), p->name );
	}
	remove_traces();
    }
}


// called by sc_trace

template <class T>
inline
void
sc_inout<T>::add_trace_internal( sc_trace_file* tf_, const std::string& name_) 
const
{
    if( tf_ != 0 ) {
	    if( m_traces == 0 ) {
	        m_traces = new sc_trace_params_vec;
	    }
	    m_traces->push_back( new sc_trace_params( tf_, name_ ) );
    }
}

template <class T>
inline
void
sc_inout<T>::add_trace( sc_trace_file* tf_, const std::string& name_) const
{
    sc_deprecated_add_trace();
    add_trace_internal(tf_, name_);
}

template <class T>
inline
void
sc_inout<T>::remove_traces() const
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
//  CLASS : sc_inout<bool>
//
//  Specialization of sc_inout<T> for type bool.
// ----------------------------------------------------------------------------

template <>
class sc_inout<bool> : 
    public sc_port<sc_signal_inout_if<bool>,1,SC_ONE_OR_MORE_BOUND>
{
public:

    // typedefs

    typedef bool                                       data_type;

    typedef sc_signal_inout_if<data_type>              if_type;
    typedef sc_port<if_type,1,SC_ONE_OR_MORE_BOUND>    base_type;
    typedef sc_inout<data_type>                        this_type;

    typedef sc_signal_in_if<data_type>                 in_if_type;
    typedef sc_port<in_if_type,1,SC_ONE_OR_MORE_BOUND> in_port_type;
    typedef if_type                                    inout_if_type;
    typedef base_type                                  inout_port_type;

public:

    // constructors

    sc_inout()
	: base_type(), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_inout( const char* name_ )
	: base_type( name_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_inout( inout_if_type& interface_ )
	: base_type( interface_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_inout( const char* name_, inout_if_type& interface_ )
	: base_type( name_, interface_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_inout( inout_port_type& parent_ )
	: base_type( parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_inout( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_inout( this_type& parent_ )
	: base_type( parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_inout( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}


    // destructor

    virtual ~sc_inout();


    // interface access shortcut methods

    // get the default event

    const sc_event& default_event() const
	{ return (*this)->default_event(); }


    // get the value changed event

    const sc_event& value_changed_event() const
	{ return (*this)->value_changed_event(); }

    // get the positive edge event

    const sc_event& posedge_event() const
	{ return (*this)->posedge_event(); }

    // get the negative edge event

    const sc_event& negedge_event() const
	{ return (*this)->negedge_event(); }


    // read the current value

    const data_type& read() const
	{ return (*this)->read(); }

    operator const data_type& () const
	{ return (*this)->read(); }


    // use for positive edge sensitivity

    sc_event_finder& pos() const
    {
        if ( !m_pos_finder_p )
	{
	    m_pos_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::posedge_event );
	} 
	return *m_pos_finder_p;
    }

    // use for negative edge sensitivity

    sc_event_finder& neg() const
    {
        if ( !m_neg_finder_p )
	{
	    m_neg_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::negedge_event );
	} 
	return *m_neg_finder_p;
    }


    // was there a value changed event?

    bool event() const
	{ return (*this)->event(); }

    // was there a positive edge event?

    bool posedge() const
        { return (*this)->posedge(); }

    // was there a negative edge event?

    bool negedge() const
        { return (*this)->negedge(); }

    // write the new value

    void write( const data_type& value_ )
	{ (*this)->write( value_ ); }

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


    // set initial value (can also be called when port is not bound yet)

    void initialize( const data_type& value_ );

    void initialize( const in_if_type& interface_ )
	{ initialize( interface_.read() ); }


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();


    // (other) event finder method(s)

    sc_event_finder& value_changed() const
    {
        if ( !m_change_finder_p )
	{
	    m_change_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::value_changed_event );
	}
	return *m_change_finder_p;
    }

    virtual const char* kind() const
        { return "sc_inout"; }

protected:

    data_type* m_init_val;

public:

    // called by sc_trace
    void add_trace_internal( sc_trace_file*, const std::string& ) const;

    void add_trace( sc_trace_file*, const std::string& ) const;

protected:

    void remove_traces() const;

    mutable sc_trace_params_vec* m_traces;

private:
  mutable sc_event_finder* m_change_finder_p;
  mutable sc_event_finder* m_neg_finder_p;
  mutable sc_event_finder* m_pos_finder_p;

private:

    // disabled
    sc_inout( const this_type& );

#ifdef __GNUC__
    // Needed to circumvent a problem in the g++-2.95.2 compiler:
    // This unused variable forces the compiler to instantiate
    // an object of T template so an implicit conversion from
    // read() to a C++ intrinsic data type will work.
    static data_type dummy;
#endif
};


// ----------------------------------------------------------------------------
//  CLASS : sc_inout<sc_dt::sc_logic>
//
//  Specialization of sc_inout<T> for type sc_dt::sc_logic.
// ----------------------------------------------------------------------------

template <>
class sc_inout<sc_dt::sc_logic>
: public sc_port<sc_signal_inout_if<sc_dt::sc_logic>,1,SC_ONE_OR_MORE_BOUND>
{
public:

    // typedefs

    typedef sc_dt::sc_logic                            data_type;

    typedef sc_signal_inout_if<data_type>              if_type;
    typedef sc_port<if_type,1,SC_ONE_OR_MORE_BOUND>    base_type;
    typedef sc_inout<data_type>                        this_type;

    typedef sc_signal_in_if<data_type>                 in_if_type;
    typedef sc_port<in_if_type,1,SC_ONE_OR_MORE_BOUND> in_port_type;
    typedef if_type                                    inout_if_type;
    typedef base_type                                  inout_port_type;

public:

    // constructors

    sc_inout()
	: base_type(), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_inout( const char* name_ )
	: base_type( name_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_inout( inout_if_type& interface_ )
	: base_type( interface_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_inout( const char* name_, inout_if_type& interface_ )
	: base_type( name_, interface_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    explicit sc_inout( inout_port_type& parent_ )
	: base_type( parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_inout( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_inout( this_type& parent_ )
	: base_type( parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}

    sc_inout( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ ), m_init_val( 0 ), m_traces( 0 ),
	  m_change_finder_p(0), m_neg_finder_p(0), m_pos_finder_p(0)
	{}


    // destructor

    virtual ~sc_inout();


    // interface access shortcut methods

    // get the default event

    const sc_event& default_event() const
	{ return (*this)->default_event(); }


    // get the value changed event

    const sc_event& value_changed_event() const
	{ return (*this)->value_changed_event(); }

    // get the positive edge event

    const sc_event& posedge_event() const
	{ return (*this)->posedge_event(); }

    // get the negative edge event

    const sc_event& negedge_event() const
	{ return (*this)->negedge_event(); }


    // read the current value

    const data_type& read() const
	{ return (*this)->read(); }

    operator const data_type& () const
	{ return (*this)->read(); }


    // use for positive edge sensitivity

    sc_event_finder& pos() const
    {
        if ( !m_pos_finder_p )
	{
	    m_pos_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::posedge_event );
	} 
	return *m_pos_finder_p;
    }

    // use for negative edge sensitivity

    sc_event_finder& neg() const
    {
        if ( !m_neg_finder_p )
	{
	    m_neg_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::negedge_event );
	} 
	return *m_neg_finder_p;
    }


    // was there a value changed event?

    bool event() const
	{ return (*this)->event(); }

    // was there a positive edge event?

    bool posedge() const
        { return (*this)->posedge(); }

    // was there a negative edge event?

    bool negedge() const
        { return (*this)->negedge(); }

    // write the new value

    void write( const data_type& value_ )
	{ (*this)->write( value_ ); }

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


    // set initial value (can also be called when port is not bound yet)

    void initialize( const data_type& value_ );

    void initialize( const in_if_type& interface_ )
	{ initialize( interface_.read() ); }


    // called when elaboration is done
    /*  WHEN DEFINING THIS METHOD IN A DERIVED CLASS, */
    /*  MAKE SURE THAT THIS METHOD IS CALLED AS WELL. */

    virtual void end_of_elaboration();


    // (other) event finder method(s)

    sc_event_finder& value_changed() const
    {
        if ( !m_change_finder_p )
	{
	    m_change_finder_p = new sc_event_finder_t<in_if_type>(
	        *this, &in_if_type::value_changed_event );
	}
        return *m_change_finder_p;
    }

    virtual const char* kind() const
        { return "sc_inout"; }

protected:

    data_type* m_init_val;

public:

    // called by sc_trace
    void add_trace_internal( sc_trace_file*, const std::string& ) const;

    void add_trace( sc_trace_file*, const std::string& ) const;

protected:

    void remove_traces() const;

    mutable sc_trace_params_vec* m_traces;

private:
  mutable sc_event_finder* m_change_finder_p;
  mutable sc_event_finder* m_neg_finder_p;
  mutable sc_event_finder* m_pos_finder_p;

private:

    // disabled
    sc_inout( const this_type& );

#ifdef __GNUC__
    // Needed to circumvent a problem in the g++-2.95.2 compiler:
    // This unused variable forces the compiler to instantiate
    // an object of T template so an implicit conversion from
    // read() to a C++ intrinsic data type will work.
    static data_type dummy;
#endif
};


// ----------------------------------------------------------------------------
//  CLASS : sc_out<T>
//
//  The sc_signal<T> output port class.
// ----------------------------------------------------------------------------

// sc_out can also read from its port, hence no difference with sc_inout.
// For debugging reasons, a class is provided instead of a define.

template <class T>
class sc_out
: public sc_inout<T>
{
public:

    // typedefs

    typedef T                                   data_type;

    typedef sc_out<data_type>                   this_type;
    typedef sc_inout<data_type>                 base_type;

    typedef typename base_type::in_if_type      in_if_type;
    typedef typename base_type::in_port_type    in_port_type;
    typedef typename base_type::inout_if_type   inout_if_type;
    typedef typename base_type::inout_port_type inout_port_type;

public:

    // constructors

    sc_out()
	: base_type()
	{}

    explicit sc_out( const char* name_ )
	: base_type( name_ )
	{}

    explicit sc_out( inout_if_type& interface_ )
	: base_type( interface_ )
	{}

    sc_out( const char* name_, inout_if_type& interface_ )
	: base_type( name_, interface_ )
	{}

    explicit sc_out( inout_port_type& parent_ )
	: base_type( parent_ )
	{}

    sc_out( const char* name_, inout_port_type& parent_ )
	: base_type( name_, parent_ )
	{}

    sc_out( this_type& parent_ )
	: base_type( parent_ )
	{}

    sc_out( const char* name_, this_type& parent_ )
	: base_type( name_, parent_ )
	{}


    // destructor (does nothing)

    virtual ~sc_out()
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
        { return "sc_out"; }

private:

    // disabled
    sc_out( const this_type& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII


// ----------------------------------------------------------------------------
//  FUNCTION : sc_trace
// ----------------------------------------------------------------------------

template <class T>
inline
void
sc_trace(sc_trace_file* tf, const sc_in<T>& port, const std::string& name)
{
    const sc_signal_in_if<T>* iface = 0;
    if (sc_get_curr_simcontext()->elaboration_done() )
    {
	iface = DCAST<const sc_signal_in_if<T>*>( port.get_interface() );
    }

    if ( iface )
	sc_trace( tf, iface->read(), name );
    else
	port.add_trace_internal( tf, name );
}

template <class T>
inline
void
sc_trace( sc_trace_file* tf, const sc_inout<T>& port, 
    const std::string& name )
{
    const sc_signal_in_if<T>* iface = 0;
    if (sc_get_curr_simcontext()->elaboration_done() )
    {
	iface =DCAST<const sc_signal_in_if<T>*>( port.get_interface() );
    }

    if ( iface )
	sc_trace( tf, iface->read(), name );
    else
	port.add_trace_internal( tf, name );
}

} // namespace sc_core

#undef SC_VIRTUAL_

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:  Jason Elbaum, Motorola, Inc., 2001-11-12
  Description of Modification:  Added a static, private, otherwise
                                unused data member to the sc_in
                                and sc_inout classes to address
                                a bug in the GNU compiler *only*.
                                This works around a bug in g++ 2.95.2
                                regarding implicit casting from a
                                templated class to a C++ intrinsic type.

 *****************************************************************************/
//$Log: sc_signal_ports.h,v $
//Revision 1.10  2011/08/29 18:04:32  acg
// Philipp A. Hartmann: miscellaneous clean ups.
//
//Revision 1.9  2011/08/26 20:45:43  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.8  2011/08/07 19:08:01  acg
// Andy Goodrich: moved logs to end of file so line number synching works
// better between versions.
//
//Revision 1.7  2011/08/07 18:53:09  acg
// Philipp A. Hartmann: add virtual instances of the bind function for
// base classes to eliminate warning messages for clang platforms.
//
//Revision 1.6  2011/04/02 00:03:23  acg
// Andy Goodrich: catch the other bind()'s that I missed in Philipp's update.
//
//Revision 1.5  2011/04/01 22:33:31  acg
// Philipp A. Harmann: Use const interface signature to implement non-const
// interface signature for virtual bind(...).
//
//Revision 1.4  2011/03/30 16:46:10  acg
// Andy Goodrich: added a signature and removed a virtual specification
// to eliminate warnings with certain compilers.
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
//Revision 1.11  2006/04/18 23:36:50  acg
// Andy Goodrich: made add_trace_internal public until I can figure out
// how to do a friend specification for sc_trace in an environment where
// there are partial template and full template specifications for its
// arguments.
//
//Revision 1.10  2006/04/18 18:01:26  acg
// Andy Goodrich: added an add_trace_internal() method to the various port
// classes so that sc_trace has something to call that won't emit an
// IEEE 1666 deprecation message.
//
//Revision 1.9  2006/03/13 20:19:44  acg
// Andy Goodrich: changed sc_event instances into pointers to sc_event instances
// that are allocated as needed. This saves considerable storage for large
// numbers of signals, etc.
//
//Revision 1.8  2006/02/02 23:42:37  acg
// Andy Goodrich: implemented a much better fix to the sc_event_finder
// proliferation problem. This new version allocates only a single event
// finder for each port for each type of event, e.g., pos(), neg(), and
// value_change(). The event finder persists as long as the port does,
// which is what the LRM dictates. Because only a single instance is
// allocated for each event type per port there is not a potential
// explosion of storage as was true in the 2.0.1/2.1 versions.
//
//Revision 1.7  2006/02/02 21:38:12  acg
// Andy Goodrich: fix to the comment log.
//
//Revision 1.4  2006/01/24 20:46:32  acg
//Andy Goodrich: changes to eliminate use of deprecated features. For instance,
//using notify(SC_ZERO_TIME) in place of notify_delayed().
//
//Revision 1.3  2006/01/13 18:47:42  acg
//Added $Log command so that CVS comments are reproduced in the source.
//
//Revision 1.2  2006/01/03 23:18:26  acg
//Changed copyright to include 2006.
//
//Revision 1.1.1.1  2005/12/19 23:16:43  acg
//First check in of SystemC 2.1 into its own archive.
//
//Revision 1.18  2005/09/15 23:01:52  acg
//Added std:: prefix to appropriate methods and types to get around
//issues with the Edison Front End.
//
//Revision 1.17  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

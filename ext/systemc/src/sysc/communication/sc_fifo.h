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

  sc_fifo.h -- The sc_fifo<T> primitive channel class.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_FIFO_H
#define SC_FIFO_H


#include "sysc/communication/sc_communication_ids.h"
#include "sysc/communication/sc_prim_channel.h"
#include "sysc/communication/sc_fifo_ifs.h"
#include "sysc/kernel/sc_event.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/tracing/sc_trace.h"
#include <typeinfo>

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_fifo<T>
//
//  The sc_fifo<T> primitive channel class.
// ----------------------------------------------------------------------------

template <class T>
class sc_fifo
: public sc_fifo_in_if<T>,
  public sc_fifo_out_if<T>,
  public sc_prim_channel
{
public:

    // constructors

    explicit sc_fifo( int size_ = 16 )
	: sc_prim_channel( sc_gen_unique_name( "fifo" ) ),
	  m_data_read_event(
	      (std::string(SC_KERNEL_EVENT_PREFIX)+"_read_event").c_str()),
	  m_data_written_event(
	      (std::string(SC_KERNEL_EVENT_PREFIX)+"_write_event").c_str())
	{ init( size_ ); }

    explicit sc_fifo( const char* name_, int size_ = 16 )
	: sc_prim_channel( name_ ),
	  m_data_read_event(
	      (std::string(SC_KERNEL_EVENT_PREFIX)+"_read_event").c_str()),
	  m_data_written_event(
	      (std::string(SC_KERNEL_EVENT_PREFIX)+"_write_event").c_str())
	{ init( size_ ); }


    // destructor

    virtual ~sc_fifo()
	{ delete [] m_buf; }


    // interface methods

    virtual void register_port( sc_port_base&, const char* );


    // blocking read
    virtual void read( T& );
    virtual T read();

    // non-blocking read
    virtual bool nb_read( T& );


    // get the number of available samples

    virtual int num_available() const
	{ return ( m_num_readable - m_num_read ); }


    // get the data written event

    virtual const sc_event& data_written_event() const
	{ return m_data_written_event; }


    // blocking write
    virtual void write( const T& );

    // non-blocking write
    virtual bool nb_write( const T& );


    // get the number of free spaces

    virtual int num_free() const
	{ return ( m_size - m_num_readable - m_num_written ); }


    // get the data read event

    virtual const sc_event& data_read_event() const
	{ return m_data_read_event; }


    // other methods

    operator T ()
	{ return read(); }


    sc_fifo<T>& operator = ( const T& a )
        { write( a ); return *this; }


    void trace( sc_trace_file* tf ) const;


    virtual void print( ::std::ostream& = ::std::cout ) const;
    virtual void dump( ::std::ostream& = ::std::cout ) const;

    virtual const char* kind() const
        { return "sc_fifo"; }

protected:

    virtual void update();

    // support methods

    void init( int );

    void buf_init( int );
    bool buf_write( const T& );
    bool buf_read( T& );

protected:

    int m_size;			// size of the buffer
    T*  m_buf;			// the buffer
    int m_free;			// number of free spaces
    int m_ri;			// index of next read
    int m_wi;			// index of next write

    sc_port_base* m_reader;	// used for static design rule checking
    sc_port_base* m_writer;	// used for static design rule checking

    int m_num_readable;		// #samples readable
    int m_num_read;		// #samples read during this delta cycle
    int m_num_written;		// #samples written during this delta cycle

    sc_event m_data_read_event;
    sc_event m_data_written_event;

private:

    // disabled
    sc_fifo( const sc_fifo<T>& );
    sc_fifo& operator = ( const sc_fifo<T>& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

template <class T>
inline
void
sc_fifo<T>::register_port( sc_port_base& port_,
			    const char* if_typename_ )
{
    std::string nm( if_typename_ );
    if( nm == typeid( sc_fifo_in_if<T> ).name() ||
        nm == typeid( sc_fifo_blocking_in_if<T> ).name() 
    ) {
	// only one reader can be connected
	if( m_reader != 0 ) {
	    SC_REPORT_ERROR( SC_ID_MORE_THAN_ONE_FIFO_READER_, 0 );
	}
	m_reader = &port_;
    } else if( nm == typeid( sc_fifo_out_if<T> ).name() ||
               nm == typeid( sc_fifo_blocking_out_if<T> ).name()
    ) {
	// only one writer can be connected
	if( m_writer != 0 ) {
	    SC_REPORT_ERROR( SC_ID_MORE_THAN_ONE_FIFO_WRITER_, 0 );
	}
	m_writer = &port_;
    }
    else
    {
        SC_REPORT_ERROR( SC_ID_BIND_IF_TO_PORT_, 
	                 "sc_fifo<T> port not recognized" );
    }
}


// blocking read

template <class T>
inline
void
sc_fifo<T>::read( T& val_ )
{
    while( num_available() == 0 ) {
	sc_core::wait( m_data_written_event );
    }
    m_num_read ++;
    buf_read( val_ );
    request_update();
}

template <class T>
inline
T
sc_fifo<T>::read()
{
    T tmp;
    read( tmp );
    return tmp;
}

// non-blocking read

template <class T>
inline
bool
sc_fifo<T>::nb_read( T& val_ )
{
    if( num_available() == 0 ) {
	return false;
    }
    m_num_read ++;
    buf_read( val_ );
    request_update();
    return true;
}


// blocking write

template <class T>
inline
void
sc_fifo<T>::write( const T& val_ )
{
    while( num_free() == 0 ) {
	sc_core::wait( m_data_read_event );
    }
    m_num_written ++;
    buf_write( val_ );
    request_update();
}

// non-blocking write

template <class T>
inline
bool
sc_fifo<T>::nb_write( const T& val_ )
{
    if( num_free() == 0 ) {
	return false;
    }
    m_num_written ++;
    buf_write( val_ );
    request_update();
    return true;
}


template <class T>
inline
void
sc_fifo<T>::trace( sc_trace_file* tf ) const
{
#if defined(DEBUG_SYSTEMC)
    char buf[32];
    std::string nm = name();
    for( int i = 0; i < m_size; ++ i ) {
	std::sprintf( buf, "_%d", i );
	sc_trace( tf, m_buf[i], nm + buf );
    }
#endif
}


template <class T>
inline
void
sc_fifo<T>::print( ::std::ostream& os ) const
{
    if( m_free != m_size ) {
        int i = m_ri;
        do {
            os << m_buf[i] << ::std::endl;
            i = ( i + 1 ) % m_size;
        } while( i != m_wi );
    }
}

template <class T>
inline
void
sc_fifo<T>::dump( ::std::ostream& os ) const
{
    os << "name = " << name() << ::std::endl;
    if( m_free != m_size ) {
        int i = m_ri;
        int j = 0;
        do {
	    os << "value[" << i << "] = " << m_buf[i] << ::std::endl;
	    i = ( i + 1 ) % m_size;
	    j ++;
        } while( i != m_wi );
    }
}


template <class T>
inline
void
sc_fifo<T>::update()
{
    if( m_num_read > 0 ) {
	m_data_read_event.notify(SC_ZERO_TIME);
    }

    if( m_num_written > 0 ) {
	m_data_written_event.notify(SC_ZERO_TIME);
    }

    m_num_readable = m_size - m_free;
    m_num_read = 0;
    m_num_written = 0;
}


// support methods

template <class T>
inline
void
sc_fifo<T>::init( int size_ )
{
    buf_init( size_ );

    m_reader = 0;
    m_writer = 0;

    m_num_readable = 0;
    m_num_read = 0;
    m_num_written = 0;
}


template <class T>
inline
void
sc_fifo<T>::buf_init( int size_ )
{
    if( size_ <= 0 ) {
	SC_REPORT_ERROR( SC_ID_INVALID_FIFO_SIZE_, 0 );
    }
    m_size = size_;
    m_buf = new T[m_size];
    m_free = m_size;
    m_ri = 0;
    m_wi = 0;
}

template <class T>
inline
bool
sc_fifo<T>::buf_write( const T& val_ )
{
    if( m_free == 0 ) {
	return false;
    }
    m_buf[m_wi] = val_;
    m_wi = ( m_wi + 1 ) % m_size;
    m_free --;
    return true;
}

template <class T>
inline
bool
sc_fifo<T>::buf_read( T& val_ )
{
    if( m_free == m_size ) {
	return false;
    }
    val_ = m_buf[m_ri];
    m_buf[m_ri] = T(); // clear entry for boost::shared_ptr, et al.
    m_ri = ( m_ri + 1 ) % m_size;
    m_free ++;
    return true;
}


// ----------------------------------------------------------------------------

template <class T>
inline
::std::ostream&
operator << ( ::std::ostream& os, const sc_fifo<T>& a )
{
    a.print( os );
    return os;
}

} // namespace sc_core

//$Log: sc_fifo.h,v $
//Revision 1.6  2011/08/26 20:45:40  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.5  2011/03/23 16:17:22  acg
// Andy Goodrich: hide the sc_events that are kernel related.
//
//Revision 1.4  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
//
//Revision 1.3  2009/10/14 19:05:40  acg
// Andy Goodrich: added check for blocking interfaces in addition to the
// combined blocking/nonblocking interface.
//
//Revision 1.2  2009/05/22 16:06:24  acg
// Andy Goodrich: process control updates.
//
//Revision 1.1.1.1  2006/12/15 20:20:04  acg
//SystemC 2.3
//
//Revision 1.4  2006/01/24 20:46:31  acg
//Andy Goodrich: changes to eliminate use of deprecated features. For instance,
//using notify(SC_ZERO_TIME) in place of notify_delayed().
//
//Revision 1.3  2006/01/13 20:41:59  acg
//Andy Goodrich: Changes to add port registration to the things that are
//checked when SC_NO_WRITE_CHECK is not defined.
//
//Revision 1.2  2006/01/03 23:18:26  acg
//Changed copyright to include 2006.
//
//Revision 1.1.1.1  2005/12/19 23:16:43  acg
//First check in of SystemC 2.1 into its own archive.
//
//Revision 1.12  2005/09/15 23:01:51  acg
//Added std:: prefix to appropriate methods and types to get around
//issues with the Edison Front End.
//
//Revision 1.11  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

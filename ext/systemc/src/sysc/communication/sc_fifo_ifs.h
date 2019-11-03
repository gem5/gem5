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

  sc_fifo_ifs.h -- The sc_fifo<T> interface classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_FIFO_IFS_H
#define SC_FIFO_IFS_H


#include "sysc/communication/sc_interface.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_fifo_nonblocking_in_if<T>
//
//  The sc_fifo<T> input nonblocking interface class.
// ----------------------------------------------------------------------------

template <class T> 
class sc_fifo_nonblocking_in_if 
: virtual public sc_interface 
{ 
public: 

    // non-blocking read 
    virtual bool nb_read( T& ) = 0; 

    // get the data written event 
    virtual const sc_event& data_written_event() const = 0; 
}; 

// ----------------------------------------------------------------------------
//  CLASS : sc_fifo_blocking_in_if<T>
//
//  The sc_fifo<T> input blocking interface class.
// ----------------------------------------------------------------------------

template <class T> 
class sc_fifo_blocking_in_if 
: virtual public sc_interface 
{ 
public: 

    // blocking read 
    virtual void read( T& ) = 0; 
    virtual T read() = 0; 
}; 

// ----------------------------------------------------------------------------
//  CLASS : sc_fifo_in_if<T>
//
//  The sc_fifo<T> input interface class.
// ----------------------------------------------------------------------------

template <class T> 
class sc_fifo_in_if 
: public sc_fifo_nonblocking_in_if<T>, 
  public sc_fifo_blocking_in_if<T> 
{ 
public: 

    // get the number of available samples 
    virtual int num_available() const = 0; 

protected: 

    // constructor 

    sc_fifo_in_if() 
        {} 

private: 

    // disabled 
    sc_fifo_in_if( const sc_fifo_in_if<T>& ); 
    sc_fifo_in_if<T>& operator = ( const sc_fifo_in_if<T>& ); 
}; 


// ----------------------------------------------------------------------------
//  CLASS : sc_fifo_nonblocking_out_if<T>
//
//  The sc_fifo<T> nonblocking output interface class.
// ----------------------------------------------------------------------------

template <class T> 
class sc_fifo_nonblocking_out_if 
: virtual public sc_interface 
{ 
public: 

    // non-blocking write 
    virtual bool nb_write( const T& ) = 0; 

    // get the data read event 
    virtual const sc_event& data_read_event() const = 0; 
}; 

// ----------------------------------------------------------------------------
//  CLASS : sc_fifo_blocking_out_if<T>
//
//  The sc_fifo<T> blocking output interface class.
// ----------------------------------------------------------------------------

template <class T> 
class sc_fifo_blocking_out_if 
: virtual public sc_interface 
{ 
public: 

    // blocking write 
    virtual void write( const T& ) = 0; 

}; 

// ----------------------------------------------------------------------------
//  CLASS : sc_fifo_out_if<T>
//
//  The sc_fifo<T> output interface class.
// ----------------------------------------------------------------------------

template <class T> 
class sc_fifo_out_if 
: public sc_fifo_nonblocking_out_if<T>, 
  public sc_fifo_blocking_out_if<T> 
{ 
public: 

    // get the number of free spaces 
    virtual int num_free() const = 0; 

protected: 

    // constructor 

    sc_fifo_out_if() 
        {} 

private: 

    // disabled 
    sc_fifo_out_if( const sc_fifo_out_if<T>& ); 
    sc_fifo_out_if<T>& operator = ( const sc_fifo_out_if<T>& ); 
}; 

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Bishnupriya Bhattacharye, Cadence Design Systems,
                               30 Jan, 2004
  Description of Modification: Split up the interfaces into blocking and 
                               non blocking parts
    
      Name, Affiliation, Date: 
  Description of Modification: 

 *****************************************************************************/
//$Log: sc_fifo_ifs.h,v $
//Revision 1.3  2011/08/26 20:45:40  acg
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
//Revision 1.10  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

} // namespace sc_core

#endif

// Taf!

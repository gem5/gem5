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

  sc_mutex_if.h -- The sc_mutex_if interface class.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_MUTEX_IF_H
#define SC_MUTEX_IF_H

#include "sysc/communication/sc_interface.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_mutex_if
//
//  The sc_mutex_if interface class.
// ----------------------------------------------------------------------------

class sc_mutex_if
: virtual public sc_interface
{
public:

    // the classical operations: lock(), trylock(), and unlock()

    // blocks until mutex could be locked
    virtual int lock() = 0;

    // returns -1 if mutex could not be locked
    virtual int trylock() = 0;

    // returns -1 if mutex was not locked by caller
    virtual int unlock() = 0;

protected:

    // constructor

    sc_mutex_if()
	{}

private:

    // disabled
    sc_mutex_if( const sc_mutex_if& );
    sc_mutex_if& operator = ( const sc_mutex_if& );
};

// ----------------------------------------------------------------------------
//  CLASS : sc_scoped_lock
//
//  The sc_scoped_lock class to lock (and automatically release) a mutex.
// ----------------------------------------------------------------------------

//template< typename Lockable = sc_mutex_if >
class sc_scoped_lock
{
public:
    //typedef Lockable lockable_type;
    typedef sc_mutex_if lockable_type;

    explicit
    sc_scoped_lock( lockable_type& mtx )
      : m_ref(mtx)
      , m_active(true)
    {
        m_ref.lock();
    }

    bool release()
    {
        if( m_active )
        {
            m_ref.unlock();
            m_active = false;
            return true;
        }
        return false;
    }

    ~sc_scoped_lock()
    {
        release();
    }

private:
    // disabled
    sc_scoped_lock( const sc_scoped_lock& );
    sc_scoped_lock& operator=( const sc_scoped_lock& );

    lockable_type& m_ref;
    bool           m_active;
};

} // namespace sc_core

//$Log: sc_mutex_if.h,v $
//Revision 1.4  2011/08/26 20:45:41  acg
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
//Revision 1.2  2006/01/03 23:18:26  acg
//Changed copyright to include 2006.
//
//Revision 1.1.1.1  2005/12/19 23:16:43  acg
//First check in of SystemC 2.1 into its own archive.
//
//Revision 1.8  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

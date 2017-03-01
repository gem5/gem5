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

  sc_mutex.h -- The sc_mutex primitive channel class.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_MUTEX_H
#define SC_MUTEX_H

#include "sysc/kernel/sc_event.h"
#include "sysc/kernel/sc_object.h"
#include "sysc/kernel/sc_wait.h"
#include "sysc/communication/sc_mutex_if.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_mutex
//
//  The sc_mutex primitive channel class.
// ----------------------------------------------------------------------------

class sc_mutex
: public sc_mutex_if,
  public sc_object
{
public:

    // constructors and destructor

    sc_mutex();
    explicit sc_mutex( const char* name_ );
	virtual ~sc_mutex();


    // interface methods

    // blocks until mutex could be locked
    virtual int lock();

    // returns -1 if mutex could not be locked
    virtual int trylock();

    // returns -1 if mutex was not locked by caller
    virtual int unlock();

    virtual const char* kind() const
        { return "sc_mutex"; }

protected:

    // support methods

    bool in_use() const
	{ return ( m_owner != 0 ); }

protected:

    sc_process_b* m_owner;
    sc_event      m_free;

private:

    // disabled
    sc_mutex( const sc_mutex& );
    sc_mutex& operator = ( const sc_mutex& );
};

} // namespace sc_core

//$Log: sc_mutex.h,v $
//Revision 1.4  2011/08/26 20:45:41  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.3  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
//
//Revision 1.2  2010/11/02 16:31:01  acg
// Andy Goodrich: changed object derivation to use sc_object rather than
// sc_prim_channel as the parent class.
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
//Revision 1.10  2005/09/15 23:01:51  acg
//Added std:: prefix to appropriate methods and types to get around
//issues with the Edison Front End.
//
//Revision 1.9  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

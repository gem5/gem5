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

  sc_semaphore_if.h -- The sc_semaphore_if interface class.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_SEMAPHORE_IF_H
#define SC_SEMAPHORE_IF_H

#include "sysc/communication/sc_interface.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_semaphore_if
//
//  The sc_semaphore_if interface class.
// ----------------------------------------------------------------------------

class sc_semaphore_if
: virtual public sc_interface
{
public:

    // the classical operations: wait(), trywait(), and post()

    // lock (take) the semaphore, block if not available
    virtual int wait() = 0;

    // lock (take) the semaphore, return -1 if not available
    virtual int trywait() = 0;

    // unlock (give) the semaphore
    virtual int post() = 0;

    // get the value of the semphore
    virtual int get_value() const = 0;

protected:

    // constructor

    sc_semaphore_if()
	{}

private:

    // disabled
    sc_semaphore_if( const sc_semaphore_if& );
    sc_semaphore_if& operator = ( const sc_semaphore_if& );
};

} // namespace sc_core

//$Log: sc_semaphore_if.h,v $
//Revision 1.3  2011/08/26 20:45:42  acg
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
//Revision 1.8  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

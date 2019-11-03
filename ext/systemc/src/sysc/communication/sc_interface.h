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

  sc_interface.h -- Abstract base class of all interface classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_INTERFACE_H
#define SC_INTERFACE_H

namespace sc_core {

class sc_event;
class sc_port_base;


// ----------------------------------------------------------------------------
//  CLASS : sc_interface
//
//  Abstract base class of all interface classes.
//  BEWARE: Direct inheritance from this class must be done virtual.
// ----------------------------------------------------------------------------

class sc_interface
{
public:

    // register a port with this interface (does nothing by default)
    virtual void register_port( sc_port_base& port_,
				const char*    if_typename_ );

    // get the default event
    virtual const sc_event& default_event() const;

    // destructor (does nothing)
    virtual ~sc_interface();

protected:

    // constructor (does nothing)
    sc_interface();

private:

    // disabled
    sc_interface( const sc_interface& );
    sc_interface& operator = ( const sc_interface& );

private:

    static sc_event m_never_notified;

#if defined(__SUNPRO_CC) && (__SUNPRO_CC == 0x520)
    // Workaround for a bug in the Sun WorkShop 6 update 2 compiler.
    // An empty virtual base class can cause the optimizer to
    // generate wrong code.
    char dummy;
#endif
};

} // namespace sc_core

//$Log: sc_interface.h,v $
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
//Revision 1.7  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!

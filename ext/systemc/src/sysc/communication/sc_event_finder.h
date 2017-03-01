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

  sc_event_finder.h --

  Original Author: Martin Janssen, Synopsys, Inc.
                   Stan Y. Liao, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_EVENT_FINDER
#define SC_EVENT_FINDER


#include "sysc/communication/sc_port.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_event_finder
//
//  Event finder base class.
// ----------------------------------------------------------------------------

class sc_event_finder
{
  friend class sc_simcontext;

public:

    const sc_port_base& port() const
        { return m_port; }

    // destructor (does nothing)
    virtual ~sc_event_finder();

    virtual const sc_event& find_event( sc_interface* if_p = 0 ) const = 0;

protected:
    
    // constructor
    sc_event_finder( const sc_port_base& );

    // error reporting
    void report_error( const char* id, const char* add_msg = 0 ) const;


private:
    const  sc_port_base&    m_port;    // port providing the event.

private:

    // disabled
    sc_event_finder();
    sc_event_finder( const sc_event_finder& );
    sc_event_finder& operator = ( const sc_event_finder& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_event_finder_t<IF>
//
//  Interface specific event finder class.
// ----------------------------------------------------------------------------

template <class IF>
class sc_event_finder_t
: public sc_event_finder
{
public:

    // constructor

    sc_event_finder_t( const sc_port_base& port_,
		       const sc_event& (IF::*event_method_) () const )
        : sc_event_finder( port_ ), m_event_method( event_method_ )
        {}

    // destructor (does nothing)

    virtual ~sc_event_finder_t()
        {}

    virtual const sc_event& find_event( sc_interface* if_p = 0 ) const;

private:

    const sc_event& (IF::*m_event_method) () const;

private:

    // disabled
    sc_event_finder_t();
    sc_event_finder_t( const sc_event_finder_t<IF>& );
    sc_event_finder_t<IF>& operator = ( const sc_event_finder_t<IF>& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

template <class IF>
inline
const sc_event&
sc_event_finder_t<IF>::find_event( sc_interface* if_p ) const
{
    const IF* iface = ( if_p ) ? DCAST<const IF*>( if_p ) :
		                 DCAST<const IF*>( port().get_interface() );
    if( iface == 0 ) {
		report_error( SC_ID_FIND_EVENT_, "port is not bound" );
    }
    return (CCAST<IF*>( iface )->*m_event_method) ();
}

} // namespace sc_core

//$Log: sc_event_finder.h,v $
//Revision 1.3  2011/08/26 20:45:39  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.2  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
//
//Revision 1.1.1.1  2006/12/15 20:20:04  acg
//SystemC 2.3
//
//Revision 1.4  2006/02/02 23:42:37  acg
// Andy Goodrich: implemented a much better fix to the sc_event_finder
// proliferation problem. This new version allocates only a single event
// finder for each port for each type of event, e.g., pos(), neg(), and
// value_change(). The event finder persists as long as the port does,
// which is what the LRM dictates. Because only a single instance is
// allocated for each event type per port there is not a potential
// explosion of storage as was true in the 2.0.1/2.1 versions.
//
//Revision 1.3  2006/02/02 20:43:09  acg
// Andy Goodrich: Added an existence linked list to sc_event_finder so that
// the dynamically allocated instances can be freed after port binding
// completes. This replaces the individual deletions in ~sc_bind_ef, as these
// caused an exception if an sc_event_finder instance was used more than
// once, due to a double freeing of the instance.
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

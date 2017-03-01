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

#ifndef __TLM_EVENT_FINDER_H__
#define __TLM_EVENT_FINDER_H__

//#include <systemc>

#include "tlm_core/tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_tag.h"

namespace tlm {

template <class IF , class T>
class tlm_event_finder_t
: public sc_core::sc_event_finder
{
public:

    // constructor

    tlm_event_finder_t( const sc_core::sc_port_base& port_,
                        const sc_core::sc_event& (IF::*event_method_) ( tlm_tag<T> * ) const )
        : sc_core::sc_event_finder( port_ ), m_event_method( event_method_ )
        {}

    // destructor (does nothing)

    virtual ~tlm_event_finder_t()
        {}
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
    virtual const sc_core::sc_event& find_event( sc_core::sc_interface* if_p = 0 ) const;
#else
    virtual const sc_core::sc_event& find_event() const;
#endif

private:

    const sc_core::sc_event& (IF::*m_event_method) ( tlm_tag<T> * ) const;

private:

    // disabled
    tlm_event_finder_t();
    tlm_event_finder_t( const tlm_event_finder_t<IF,T>& );
    tlm_event_finder_t<IF,T>& operator = ( const tlm_event_finder_t<IF,T>& );
};


#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
template <class IF , class T>
inline
const sc_core::sc_event&
tlm_event_finder_t<IF,T>::find_event( sc_core::sc_interface* if_p ) const
{
    const IF* iface = ( if_p ) ? dynamic_cast<const IF*>( if_p ) :
                                 dynamic_cast<const IF*>( port().get_interface() );
    if( iface == 0 ) {
  report_error( sc_core::SC_ID_FIND_EVENT_, "port is not bound" );
    }
    return (const_cast<IF*>( iface )->*m_event_method) ( 0 );
}
#else
template <class IF , class T>
inline
const sc_core::sc_event&
tlm_event_finder_t<IF,T>::find_event() const
{
    const IF* iface = dynamic_cast<const IF*>( port().get_interface() );
    if( iface == 0 ) {
  report_error( sc_core::SC_ID_FIND_EVENT_, "port is not bound" );
    }
    return (const_cast<IF*>( iface )->*m_event_method) ( 0 );
}
#endif

} // namespace tlm

#endif

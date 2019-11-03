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

#ifndef __TLM_NONBLOCKING_PORT_H__
#define __TLM_NONBLOCKING_PORT_H__

#include "tlm_core/tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_core_ifs.h"
#include "tlm_core/tlm_1/tlm_req_rsp/tlm_ports/tlm_event_finder.h"

namespace tlm {

template < typename T >
class tlm_nonblocking_get_port :
public sc_core::sc_port< tlm_nonblocking_get_if< T > , 1 >
{
public:
  typedef tlm_nonblocking_get_if<T> get_if_type;

  tlm_nonblocking_get_port( const char *port_name ) :
    sc_core::sc_port< tlm_nonblocking_get_if< T > , 1 >( port_name ) {}

  sc_core::sc_event_finder& ok_to_get() const {

    return *new tlm_event_finder_t< get_if_type , T >(
       *this,
       &get_if_type::ok_to_get );

  }

};

template < typename T >
class tlm_nonblocking_peek_port :
public sc_core::sc_port< tlm_nonblocking_peek_if< T > , 1 >
{
public:
  typedef tlm_nonblocking_peek_if<T> peek_if_type;

  tlm_nonblocking_peek_port( const char *port_name ) :
    sc_core::sc_port< tlm_nonblocking_peek_if< T > , 1 >( port_name ) {}

  sc_core::sc_event_finder& ok_to_peek() const {

    return *new tlm_event_finder_t< peek_if_type , T >(
       *this,
       &peek_if_type::ok_to_peek );

  }

};


template < typename T >
class tlm_nonblocking_put_port :
public sc_core::sc_port< tlm_nonblocking_put_if< T > , 1 >
{
public:
  typedef tlm_nonblocking_put_if<T> put_if_type;

  tlm_nonblocking_put_port( const char *port_name ) :
    sc_core::sc_port< tlm_nonblocking_put_if< T > , 1 >( port_name ) {}

  sc_core::sc_event_finder& ok_to_put() const {

    return *new tlm_event_finder_t< put_if_type , T >(
             *this,
       &put_if_type::ok_to_put );

  }

};

} // namespace tlm

#endif

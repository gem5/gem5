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

#ifndef __TLM_ADAPTERS_H__
#define __TLM_ADAPTERS_H__

#include "tlm_core/tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_master_slave_ifs.h"

namespace tlm {

template< typename REQ , typename RSP >
class tlm_transport_to_master :
  public sc_core::sc_module ,
  public virtual tlm_transport_if< REQ , RSP >
{
public:
  sc_core::sc_export< tlm_transport_if< REQ , RSP > > target_export;
  sc_core::sc_port< tlm_master_if< REQ , RSP > > master_port;

  tlm_transport_to_master( sc_core::sc_module_name nm ) :
    sc_core::sc_module( nm ) {

    target_export( *this );

  }

  tlm_transport_to_master() :
    sc_core::sc_module( sc_core::sc_module_name( sc_core::sc_gen_unique_name( "transport_to_master" ) ) ){

    target_export( *this );

  }

  RSP transport( const REQ &req ) {

    mutex.lock();

    master_port->put( req );
    rsp = master_port->get();

    mutex.unlock();
    return rsp;

  }

private:
  sc_core::sc_mutex mutex;
  RSP rsp;

};

template< typename REQ , typename RSP >
class tlm_slave_to_transport : public sc_core::sc_module
{
public:

  SC_HAS_PROCESS( tlm_slave_to_transport );

  sc_core::sc_port< tlm_slave_if< REQ , RSP > > slave_port;
  sc_core::sc_port< tlm_transport_if< REQ , RSP > > initiator_port;

  tlm_slave_to_transport( sc_core::sc_module_name nm ) : sc_core::sc_module( nm )
  {}

  tlm_slave_to_transport() :
    sc_core::sc_module( sc_core::sc_module_name( sc_core::sc_gen_unique_name("slave_to_transport") ) )
  {}

private:
  void run() {

    REQ req;
    RSP rsp;

    while( true ) {

     slave_port->get( req );
     rsp = initiator_port->transport( req );
     slave_port->put( rsp );

    }

  }

};

} // namespace tlm

#endif

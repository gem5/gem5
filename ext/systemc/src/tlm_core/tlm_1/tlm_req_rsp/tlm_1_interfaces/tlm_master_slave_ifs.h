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

#ifndef __TLM_MASTER_SLAVE_IFS_H__
#define __TLM_MASTER_SLAVE_IFS_H__

#include "tlm_core/tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_core_ifs.h"

namespace tlm {

//
// req/rsp combined interfaces
//

// blocking

template < typename REQ , typename RSP>
class tlm_blocking_master_if :
  public virtual tlm_blocking_put_if< REQ > ,
  public virtual tlm_blocking_get_peek_if< RSP > {};

template < typename REQ , typename RSP>
class tlm_blocking_slave_if :
  public virtual tlm_blocking_put_if< RSP > ,
  public virtual tlm_blocking_get_peek_if< REQ > {};

// nonblocking

template < typename REQ , typename RSP >
class tlm_nonblocking_master_if :
  public virtual tlm_nonblocking_put_if< REQ > ,
  public virtual tlm_nonblocking_get_peek_if< RSP > {};

template < typename REQ , typename RSP >
class tlm_nonblocking_slave_if :
  public virtual tlm_nonblocking_put_if< RSP > ,
  public virtual tlm_nonblocking_get_peek_if< REQ > {};

// combined

template < typename REQ , typename RSP >
class tlm_master_if :
  public virtual tlm_put_if< REQ > ,
  public virtual tlm_get_peek_if< RSP > ,
  public virtual tlm_blocking_master_if< REQ , RSP > ,
  public virtual tlm_nonblocking_master_if< REQ , RSP > {};

template < typename REQ , typename RSP >
class tlm_slave_if :
  public virtual tlm_put_if< RSP > ,
  public virtual tlm_get_peek_if< REQ > ,
  public virtual tlm_blocking_slave_if< REQ , RSP > ,
  public virtual tlm_nonblocking_slave_if< REQ , RSP > {};

} // namespace tlm

#endif

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

//
// Note to the LRM writer : These interfaces are channel specific interfaces
// useful in the context of tlm_fifo.
//

#ifndef __TLM_FIFO_IFS_H__
#define __TLM_FIFO_IFS_H__

#include "tlm_core/tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_core_ifs.h"

namespace tlm {

//
// Fifo specific interfaces
//

// Fifo Debug Interface

template< typename T >
class tlm_fifo_debug_if : public virtual sc_core::sc_interface
{
public:
  virtual int used() const = 0;
  virtual int size() const = 0;
  virtual void debug() const = 0;

  //
  // non blocking peek and poke - no notification
  //
  // n is index of data :
  // 0 <= n < size(), where 0 is most recently written, and size() - 1
  // is oldest ie the one about to be read.
  //

  virtual bool nb_peek( T & , int n ) const = 0;
  virtual bool nb_poke( const T & , int n = 0 ) = 0;

};

// fifo interfaces = extended + debug

template < typename T >
class tlm_fifo_put_if :
  public virtual tlm_put_if<T> ,
  public virtual tlm_fifo_debug_if<T> {};

template < typename T >
class tlm_fifo_get_if :
  public virtual tlm_get_peek_if<T> ,
  public virtual tlm_fifo_debug_if<T> {};

class tlm_fifo_config_size_if : public virtual sc_core::sc_interface
{
public:
  virtual void nb_expand( unsigned int n = 1 ) = 0;
  virtual void nb_unbound( unsigned int n = 16 ) = 0;

  virtual bool nb_reduce( unsigned int n = 1 ) = 0;
  virtual bool nb_bound( unsigned int n ) = 0;

};

} // namespace tlm

#endif


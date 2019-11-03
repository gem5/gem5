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

#ifndef __SYSTEMC_EXT_TLM_CORE_2_SOCKETS_BASE_SOCKET_IF_H__
#define __SYSTEMC_EXT_TLM_CORE_2_SOCKETS_BASE_SOCKET_IF_H__

#include <typeindex>

#include "../../../core/sc_export.hh"
#include "../../../core/sc_port.hh"

namespace tlm
{

enum tlm_socket_category
{
    TLM_UNKNOWN_SOCKET = 0,
    TLM_INITIATOR_SOCKET = 0x1,
    TLM_TARGET_SOCKET = 0x2,

    TLM_MULTI_SOCKET = 0x10,

    TLM_MULTI_INITIATOR_SOCKET = TLM_INITIATOR_SOCKET | TLM_MULTI_SOCKET,
    TLM_MULTI_TARGET_SOCKET = TLM_TARGET_SOCKET | TLM_MULTI_SOCKET
};

class tlm_base_socket_if
{
  public:
    virtual sc_core::sc_port_base &get_port_base() = 0;
    virtual sc_core::sc_port_base const &get_port_base() const = 0;
    virtual sc_core::sc_export_base &get_export_base() = 0;
    virtual sc_core::sc_export_base const &get_export_base() const = 0;
    virtual unsigned int get_bus_width() const = 0;
    virtual std::type_index get_protocol_types() const = 0;
    virtual tlm_socket_category get_socket_category() const = 0;

  protected:
    virtual ~tlm_base_socket_if() {}
};

} // namespace tlm

#endif // __SYSTEMC_EXT_TLM_CORE_2_SOCKETS_BASE_SOCKET_IF_H__

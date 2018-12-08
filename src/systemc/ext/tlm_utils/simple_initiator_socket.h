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

#ifndef TLM_UTILS_SIMPLE_INITIATOR_SOCKET_H_INCLUDED_
#define TLM_UTILS_SIMPLE_INITIATOR_SOCKET_H_INCLUDED_

#include <tlm>
#include "tlm_utils/convenience_socket_bases.h"

namespace tlm_utils {

template< typename MODULE, unsigned int BUSWIDTH, typename TYPES
        , sc_core::sc_port_policy POL = sc_core::SC_ONE_OR_MORE_BOUND >
class simple_initiator_socket_b
  : public tlm::tlm_initiator_socket<BUSWIDTH, TYPES, 1, POL>
  , protected simple_socket_base
{
public:
  typedef typename TYPES::tlm_payload_type                transaction_type;
  typedef typename TYPES::tlm_phase_type                  phase_type;
  typedef tlm::tlm_sync_enum                              sync_enum_type;
  typedef tlm::tlm_fw_transport_if<TYPES>                 fw_interface_type;
  typedef tlm::tlm_bw_transport_if<TYPES>                 bw_interface_type;
  typedef tlm::tlm_initiator_socket<BUSWIDTH,TYPES,1,POL> base_type;

public:
  static const char* default_name()
    { return sc_core::sc_gen_unique_name("simple_initiator_socket"); }

  explicit simple_initiator_socket_b(const char* n = default_name())
    : base_type(n)
    , m_process(this)
  {
    this->m_export.bind(m_process);
  }

  void register_nb_transport_bw(MODULE* mod,
                                sync_enum_type (MODULE::*cb)(transaction_type&,
                                                             phase_type&,
                                                             sc_core::sc_time&))
  {
    m_process.set_transport_ptr(mod, cb);
  }

  void register_invalidate_direct_mem_ptr(MODULE* mod,
                                          void (MODULE::*cb)(sc_dt::uint64, sc_dt::uint64))
  {
    m_process.set_invalidate_direct_mem_ptr(mod, cb);
  }

private:
  class process
    : public tlm::tlm_bw_transport_if<TYPES>
    , protected convenience_socket_cb_holder
  {
  public:
    typedef sync_enum_type (MODULE::*TransportPtr)(transaction_type&,
                                                   phase_type&,
                                                   sc_core::sc_time&);
    typedef void (MODULE::*InvalidateDirectMemPtr)(sc_dt::uint64,
                                                   sc_dt::uint64);

    explicit process(simple_socket_base* owner)
      : convenience_socket_cb_holder(owner), m_mod(0)
      , m_transport_ptr(0)
      , m_invalidate_direct_mem_ptr(0)
    {
    }

    void set_transport_ptr(MODULE* mod, TransportPtr p)
    {
      if (m_transport_ptr) {
        display_warning("non-blocking callback already registered");
        return;
      }
      sc_assert(!m_mod || m_mod == mod);
      m_mod = mod;
      m_transport_ptr = p;
    }

    void set_invalidate_direct_mem_ptr(MODULE* mod, InvalidateDirectMemPtr p)
    {
      if (m_invalidate_direct_mem_ptr) {
        display_warning("invalidate DMI callback already registered");
        return;
      }
      sc_assert(!m_mod || m_mod == mod);
      m_mod = mod;
      m_invalidate_direct_mem_ptr = p;
    }

    sync_enum_type nb_transport_bw(transaction_type& trans, phase_type& phase, sc_core::sc_time& t)
    {
      if (m_transport_ptr) {
        // forward call
        sc_assert(m_mod);
        return (m_mod->*m_transport_ptr)(trans, phase, t);
      }
      display_error("no transport callback registered");
      return tlm::TLM_COMPLETED;
    }

    void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                   sc_dt::uint64 end_range)
    {
      if (m_invalidate_direct_mem_ptr) {
        // forward call
        sc_assert(m_mod);
        (m_mod->*m_invalidate_direct_mem_ptr)(start_range, end_range);
      }
    }

  private:
    MODULE* m_mod;
    TransportPtr m_transport_ptr;
    InvalidateDirectMemPtr m_invalidate_direct_mem_ptr;
  };

private:
  const sc_core::sc_object* get_socket() const { return this; }
private:
  process m_process;
};

template< typename MODULE, unsigned int BUSWIDTH = 32
        , typename TYPES = tlm::tlm_base_protocol_types >
class simple_initiator_socket
  : public simple_initiator_socket_b<MODULE,BUSWIDTH,TYPES>
{
  typedef simple_initiator_socket_b<MODULE,BUSWIDTH,TYPES> socket_b;
public:
  simple_initiator_socket() : socket_b() {}
  explicit simple_initiator_socket(const char* name) : socket_b(name) {}
};

template< typename MODULE, unsigned int BUSWIDTH = 32
        , typename TYPES = tlm::tlm_base_protocol_types >
class simple_initiator_socket_optional
  : public simple_initiator_socket_b<MODULE,BUSWIDTH,TYPES,sc_core::SC_ZERO_OR_MORE_BOUND>
{
  typedef simple_initiator_socket_b<MODULE,BUSWIDTH,TYPES,sc_core::SC_ZERO_OR_MORE_BOUND> socket_b;
public:
  simple_initiator_socket_optional() : socket_b() {}
  explicit simple_initiator_socket_optional(const char* name) : socket_b(name) {}
};


// Tagged version

template< typename MODULE, unsigned int BUSWIDTH, typename TYPES
        , sc_core::sc_port_policy POL = sc_core::SC_ONE_OR_MORE_BOUND >
class simple_initiator_socket_tagged_b
  : public tlm::tlm_initiator_socket<BUSWIDTH, TYPES, 1, POL>
  , protected simple_socket_base
{
public:
  typedef typename TYPES::tlm_payload_type                transaction_type;
  typedef typename TYPES::tlm_phase_type                  phase_type;
  typedef tlm::tlm_sync_enum                              sync_enum_type;
  typedef tlm::tlm_fw_transport_if<TYPES>                 fw_interface_type;
  typedef tlm::tlm_bw_transport_if<TYPES>                 bw_interface_type;
  typedef tlm::tlm_initiator_socket<BUSWIDTH,TYPES,1,POL> base_type;

public:
  static const char* default_name()
    { return sc_core::sc_gen_unique_name("simple_initiator_socket_tagged"); }

  explicit simple_initiator_socket_tagged_b(const char* n = default_name())
    : base_type(n)
    , m_process(this)
  {
    this->m_export.bind(m_process);
  }

  void register_nb_transport_bw(MODULE* mod,
                                sync_enum_type (MODULE::*cb)(int,
                                                             transaction_type&,
                                                             phase_type&,
                                                             sc_core::sc_time&),
                                int id)
  {
    m_process.set_transport_ptr(mod, cb);
    m_process.set_transport_user_id(id);
  }

  void register_invalidate_direct_mem_ptr(MODULE* mod,
                                          void (MODULE::*cb)(int, sc_dt::uint64, sc_dt::uint64),
                                           int id)
  {
    m_process.set_invalidate_direct_mem_ptr(mod, cb);
    m_process.set_invalidate_dmi_user_id(id);
  }

private:
  class process
    : public tlm::tlm_bw_transport_if<TYPES>
    , protected convenience_socket_cb_holder
  {
  public:
    typedef sync_enum_type (MODULE::*TransportPtr)(int,
                                                   transaction_type&,
                                                   phase_type&,
                                                   sc_core::sc_time&);
    typedef void (MODULE::*InvalidateDirectMemPtr)(int,
                                                   sc_dt::uint64,
                                                   sc_dt::uint64);

    explicit process(simple_socket_base* owner)
      : convenience_socket_cb_holder(owner), m_mod(0)
      , m_transport_ptr(0)
      , m_invalidate_direct_mem_ptr(0)
      , m_transport_user_id(0)
      , m_invalidate_direct_mem_user_id(0)
    {
    }

    void set_transport_user_id(int id) { m_transport_user_id = id; }
    void set_invalidate_dmi_user_id(int id) { m_invalidate_direct_mem_user_id = id; }

    void set_transport_ptr(MODULE* mod, TransportPtr p)
    {
      if (m_transport_ptr) {
        display_warning("non-blocking callback already registered");
        return;
      }
      sc_assert(!m_mod || m_mod == mod);
      m_mod = mod;
      m_transport_ptr = p;
    }

    void set_invalidate_direct_mem_ptr(MODULE* mod, InvalidateDirectMemPtr p)
    {
      if (m_invalidate_direct_mem_ptr) {
        display_warning("invalidate DMI callback already registered");
        return;
      }
      sc_assert(!m_mod || m_mod == mod);
      m_mod = mod;
      m_invalidate_direct_mem_ptr = p;
    }

    sync_enum_type nb_transport_bw(transaction_type& trans, phase_type& phase, sc_core::sc_time& t)
    {
      if (m_transport_ptr) {
        // forward call
        sc_assert(m_mod);
        return (m_mod->*m_transport_ptr)(m_transport_user_id, trans, phase, t);
      }
      display_error("no transport callback registered");
      return tlm::TLM_COMPLETED;
    }

    void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                   sc_dt::uint64 end_range)
    {
      if (m_invalidate_direct_mem_ptr) {
        // forward call
        sc_assert(m_mod);
        (m_mod->*m_invalidate_direct_mem_ptr)(m_invalidate_direct_mem_user_id, start_range, end_range);
      }
    }

  private:
    MODULE* m_mod;
    TransportPtr m_transport_ptr;
    InvalidateDirectMemPtr m_invalidate_direct_mem_ptr;
    int m_transport_user_id;
    int m_invalidate_direct_mem_user_id;
  };

private:
  const sc_core::sc_object* get_socket() const { return this; }
private:
  process m_process;
};

template< typename MODULE, unsigned int BUSWIDTH = 32
        , typename TYPES = tlm::tlm_base_protocol_types >
class simple_initiator_socket_tagged
  : public simple_initiator_socket_tagged_b<MODULE,BUSWIDTH,TYPES>
{
  typedef simple_initiator_socket_tagged_b<MODULE,BUSWIDTH,TYPES> socket_b;
public:
  simple_initiator_socket_tagged() : socket_b() {}
  explicit simple_initiator_socket_tagged(const char* name) : socket_b(name) {}
};

template< typename MODULE, unsigned int BUSWIDTH = 32
        , typename TYPES = tlm::tlm_base_protocol_types >
class simple_initiator_socket_tagged_optional
  : public simple_initiator_socket_tagged_b<MODULE,BUSWIDTH,TYPES,sc_core::SC_ZERO_OR_MORE_BOUND>
{
  typedef simple_initiator_socket_tagged_b<MODULE,BUSWIDTH,TYPES,sc_core::SC_ZERO_OR_MORE_BOUND> socket_b;
public:
  simple_initiator_socket_tagged_optional() : socket_b() {}
  explicit simple_initiator_socket_tagged_optional(const char* name) : socket_b(name) {}
};

} // namespace tlm_utils
#endif // TLM_UTILS_SIMPLE_INITIATOR_SOCKET_H_INCLUDED_

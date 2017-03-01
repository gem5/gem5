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

#ifndef __TLM_INITIATOR_SOCKET_H__
#define __TLM_INITIATOR_SOCKET_H__

//#include <systemc>
#include "tlm_core/tlm_2/tlm_2_interfaces/tlm_fw_bw_ifs.h"

namespace tlm {


template <unsigned int BUSWIDTH = 32,
          typename FW_IF = tlm_fw_transport_if<>,
          typename BW_IF = tlm_bw_transport_if<> >
class tlm_base_initiator_socket_b
{
public:
  virtual ~tlm_base_initiator_socket_b() {}

  virtual sc_core::sc_port_b<FW_IF> &       get_base_port() = 0;
  virtual sc_core::sc_port_b<FW_IF> const & get_base_port() const = 0;
  virtual                    BW_IF  &       get_base_interface() = 0;
  virtual                    BW_IF  const & get_base_interface() const = 0;
  virtual sc_core::sc_export<BW_IF> &       get_base_export() = 0;
  virtual sc_core::sc_export<BW_IF> const & get_base_export() const = 0;
};


template <unsigned int BUSWIDTH,
          typename FW_IF,
          typename BW_IF> class tlm_base_target_socket_b;

template <unsigned int BUSWIDTH,
          typename FW_IF,
          typename BW_IF,
          int N
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
          ,sc_core::sc_port_policy POL
#endif
          > class tlm_base_target_socket;

template <unsigned int BUSWIDTH = 32,
          typename FW_IF = tlm_fw_transport_if<>,
          typename BW_IF = tlm_bw_transport_if<>,
          int N = 1
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
          ,sc_core::sc_port_policy POL = sc_core::SC_ONE_OR_MORE_BOUND
#endif
          >
class tlm_base_initiator_socket : public tlm_base_initiator_socket_b<BUSWIDTH, FW_IF, BW_IF>,
                                  public sc_core::sc_port<FW_IF, N
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
                                                        , POL
#endif
                                                    >

{
public:
  typedef FW_IF                                 fw_interface_type;
  typedef BW_IF                                 bw_interface_type;
  typedef sc_core::sc_port<fw_interface_type, N
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
                                            , POL
#endif
                                            >   port_type;

  typedef sc_core::sc_export<bw_interface_type> export_type;

  typedef tlm_base_target_socket_b<BUSWIDTH,
                                   fw_interface_type,
                                   bw_interface_type> base_target_socket_type;
  typedef tlm_base_initiator_socket_b<BUSWIDTH,
                                      fw_interface_type,
                                      bw_interface_type> base_type;

  template <unsigned int, typename, typename, int
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
           ,sc_core::sc_port_policy
#endif
           >
  friend class tlm_base_target_socket;

public:
  tlm_base_initiator_socket()
  : port_type(sc_core::sc_gen_unique_name("tlm_base_initiator_socket"))
  , m_export(sc_core::sc_gen_unique_name("tlm_base_initiator_socket_export"))
  {
  }

  explicit tlm_base_initiator_socket(const char* name)
  : port_type(name)
  , m_export(sc_core::sc_gen_unique_name((std::string(name) + "_export").c_str()))
  {
  }

  virtual const char* kind() const
  {
    return "tlm_base_initiator_socket";
  }

  unsigned int get_bus_width() const
  {
    return BUSWIDTH;
  }

  //
  // Bind initiator socket to target socket
  // - Binds the port of the initiator socket to the export of the target
  //   socket
  // - Binds the port of the target socket to the export of the initiator
  //   socket
  //
  virtual void bind(base_target_socket_type& s)
  {
    // initiator.port -> target.export
    (get_base_port())(s.get_base_interface());
    // target.port -> initiator.export
    (s.get_base_port())(get_base_interface());
  }

  void operator() (base_target_socket_type& s)
  {
    bind(s);
  }

  //
  // Bind initiator socket to initiator socket (hierarchical bind)
  // - Binds both the export and the port
  //
  virtual void bind(base_type& s)
  {
    // port
    (get_base_port())(s.get_base_port());
    // export
    (s.get_base_export())(get_base_export());
  }

  void operator() (base_type& s)
  {
    bind(s);
  }

  //
  // Bind interface to socket
  // - Binds the interface to the export of this socket
  //
  virtual void bind(bw_interface_type& ifs)
  {
    (get_base_export())(ifs);
  }

  void operator() (bw_interface_type& s)
  {
    bind(s);
  }

  // Implementation of pure virtual functions of base class
  virtual sc_core::sc_port_b<FW_IF> &       get_base_port()
    { return *this; }
  virtual sc_core::sc_port_b<FW_IF> const & get_base_port() const
    { return *this; }

  virtual                    BW_IF  &       get_base_interface()
    { return m_export; }
  virtual                    BW_IF  const & get_base_interface() const
#if !( defined(IEEE_1666_SYSTEMC) && IEEE_1666_SYSTEMC >= 201101L )
    { return const_cast<export_type &>(m_export); }
#else
    { return m_export; }
#endif

  virtual sc_core::sc_export<BW_IF> &       get_base_export()
    { return m_export; }
  virtual sc_core::sc_export<BW_IF> const & get_base_export() const
    { return m_export; }

protected:
  export_type m_export;
};

//
// Convenience socket classes
//

template <unsigned int BUSWIDTH = 32,
          typename TYPES = tlm_base_protocol_types,
          int N = 1
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
          ,sc_core::sc_port_policy POL = sc_core::SC_ONE_OR_MORE_BOUND
#endif
          >
class tlm_initiator_socket :
  public tlm_base_initiator_socket <BUSWIDTH,
                               tlm_fw_transport_if<TYPES>,
                               tlm_bw_transport_if<TYPES>,
                               N
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
                               ,POL
#endif
                              >
{
public:
  tlm_initiator_socket() :
    tlm_base_initiator_socket<BUSWIDTH,
                         tlm_fw_transport_if<TYPES>,
                         tlm_bw_transport_if<TYPES>,
                         N
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
                         ,POL
#endif
                         >()
  {
  }

  explicit tlm_initiator_socket(const char* name) :
    tlm_base_initiator_socket<BUSWIDTH,
                         tlm_fw_transport_if<TYPES>,
                         tlm_bw_transport_if<TYPES>,
                         N
#if !(defined SYSTEMC_VERSION & SYSTEMC_VERSION <= 20050714)
                         ,POL
#endif
                         >(name)
  {
  }

  virtual const char* kind() const
  {
    return "tlm_initiator_socket";
  }
};

} // namespace tlm

#endif

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

#ifndef __SYSTEMC_EXT_TLM_CORE_2_SOCKETS_TARGET_SOCKET_HH__
#define __SYSTEMC_EXT_TLM_CORE_2_SOCKETS_TARGET_SOCKET_HH__

#include <typeindex>

#include "../interfaces/fw_bw_ifs.hh"
#include "base_socket_if.hh"

namespace tlm
{

template <unsigned int BUSWIDTH=32, typename FW_IF=tlm_fw_transport_if<>,
          typename BW_IF=tlm_bw_transport_if<>>
class tlm_base_target_socket_b
{
  public:
    virtual ~tlm_base_target_socket_b() {}

    virtual sc_core::sc_port_b<BW_IF> &get_base_port() = 0;
    virtual sc_core::sc_export<FW_IF> &get_base_export() = 0;
    virtual FW_IF &get_base_interface() = 0;
};

template <unsigned int BUSWIDTH, typename FW_IF, typename BW_IF>
class tlm_base_initiator_socket_b;

template <unsigned int BUSWIDTH, typename FW_IF, typename BW_IF, int N,
          sc_core::sc_port_policy POL>
class tlm_base_initiator_socket;

template <unsigned int BUSWIDTH=32, typename FW_IF=tlm_fw_transport_if<>,
          typename BW_IF=tlm_bw_transport_if<>, int N=1,
          sc_core::sc_port_policy POL=sc_core::SC_ONE_OR_MORE_BOUND>
class tlm_base_target_socket :
    public tlm_base_socket_if,
    public tlm_base_target_socket_b<BUSWIDTH, FW_IF, BW_IF>,
    public sc_core::sc_export<FW_IF>
{
  public:
    typedef FW_IF fw_interface_type;
    typedef BW_IF bw_interface_type;
    typedef sc_core::sc_port<bw_interface_type, N, POL> port_type;

    typedef sc_core::sc_export<fw_interface_type> export_type;
    typedef tlm_base_initiator_socket_b<
        BUSWIDTH, fw_interface_type, bw_interface_type>
        base_initiator_socket_type;

    typedef tlm_base_target_socket_b<
        BUSWIDTH, fw_interface_type, bw_interface_type> base_type;

    template <unsigned int, typename, typename, int, sc_core::sc_port_policy>
    friend class tlm_base_initiator_socket;

  public:
    tlm_base_target_socket() :
        export_type(sc_core::sc_gen_unique_name("tlm_base_target_socket")),
        m_port(sc_core::sc_gen_unique_name("tlm_base_target_socket_port"))
    {}

    explicit tlm_base_target_socket(const char *name) :
        export_type(name), m_port(sc_core::sc_gen_unique_name(
                    (std::string(name) + "_port").c_str()))
    {}

    virtual const char *kind() const { return "tlm_base_target_socket"; }

    //
    // Bind target socket to initiator socket
    // - Binds the port of the initiator socket to the export of the target
    //   socket
    // - Binds the port of the target socket to the export of the initiator
    //   socket
    //

#pragma GCC diagnostic push
/**
 * The following warning is disabled because the bind methods are overloaded
 * in the derived class and the base class. In GCC v13+ this
 * 'overloaded-virtual' warning is strict enough to trigger here (though the
 * code is correct).
 */
#if defined(__GNUC__) && (__GNUC__ >= 13)
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#endif
    virtual void
    bind(base_initiator_socket_type &s)
    {
        // initiator.port -> target.export
        (s.get_base_port())(get_base_interface());
        // target.port -> initiator.export
        get_base_port()(s.get_base_interface());
    }

    void operator () (base_initiator_socket_type &s) { bind(s); }

    //
    // Bind target socket to target socket (hierarchical bind)
    // - Binds both the export and the port
    //
    virtual void
    bind(base_type &s)
    {
        // export
        (get_base_export())(s.get_base_export());
        // port
        (s.get_base_port())(get_base_port());
    }

    void operator () (base_type &s) { bind(s); }

    //
    // Bind interface to socket
    // - Binds the interface to the export
    //
    virtual void
    bind(fw_interface_type &ifs)
    {
        export_type *exp = &get_base_export();
        if (this == exp) {
            export_type::bind(ifs);
        } else {
            exp->bind( ifs );
        }
    }

    void operator () (fw_interface_type &s) { bind(s); }
#pragma GCC diagnostic pop

    //
    // Forward to 'size()' of port class.
    //
    int size() const { return m_port.size(); }

    //
    // Forward to 'operator->()' of port class.
    //
    bw_interface_type *operator->() { return m_port.operator->(); }

    //
    // Forward to 'operator[]()' of port class.
    //
    bw_interface_type *operator[](int i) { return m_port.operator[](i); }

    // Implementation of tlm_base_socket_if functions.
    virtual sc_core::sc_port_base &get_port_base() { return m_port; }
    virtual sc_core::sc_port_base const &
    get_port_base() const
    {
        return m_port;
    }
    virtual sc_core::sc_export_base &get_export_base() { return *this; }
    virtual sc_core::sc_export_base const &
    get_export_base() const
    {
        return *this;
    }
    virtual unsigned int get_bus_width() const { return BUSWIDTH; }
    virtual tlm_socket_category
    get_socket_category() const
    {
        return TLM_TARGET_SOCKET;
    }

    // Implementation of tlm_base_target_socket_b functions
    virtual sc_core::sc_port_b<BW_IF> &get_base_port() { return m_port; }
    virtual sc_core::sc_port_b<BW_IF> const &
    get_base_port() const
    {
        return m_port;
    }

    virtual FW_IF &get_base_interface() { return *this; }
    virtual FW_IF const &get_base_interface() const { return *this; }

    virtual sc_core::sc_export<FW_IF> &get_base_export() { return *this; }
    virtual sc_core::sc_export<FW_IF> const &
    get_base_export() const
    {
        return *this;
    }

  protected:
    port_type m_port;
};

template <unsigned int BUSWIDTH=32, typename TYPES=tlm_base_protocol_types,
          int N=1, sc_core::sc_port_policy POL=sc_core::SC_ONE_OR_MORE_BOUND>
class tlm_target_socket :
    public tlm_base_target_socket<
        BUSWIDTH, tlm_fw_transport_if<TYPES>,
        tlm_bw_transport_if<TYPES>, N, POL>
{
  public:
    tlm_target_socket() :
        tlm_base_target_socket<
            BUSWIDTH, tlm_fw_transport_if<TYPES>,
            tlm_bw_transport_if<TYPES>, N, POL>()
    {}

    explicit tlm_target_socket(const char *name) :
        tlm_base_target_socket<
            BUSWIDTH, tlm_fw_transport_if<TYPES>,
            tlm_bw_transport_if<TYPES>, N, POL>(name)
    {}

    virtual const char* kind() const { return "tlm_target_socket"; }

    virtual std::type_index
    get_protocol_types() const
    {
        return typeid(TYPES);
    }
};

} // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_2_SOCKETS_TARGET_SOCKET_HH__ */

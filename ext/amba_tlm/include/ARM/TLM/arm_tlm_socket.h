/*
 * The Clear BSD License
 *
 * Copyright (c) 2015-2021 Arm Limited.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARM_TLM_SOCKET_H
#define ARM_TLM_SOCKET_H

#include <tlm.h>
#include <sstream>

#include "arm_tlm_protocol.h"

namespace ARM
{
namespace TLM
{

template <typename Types>
class BaseInitiatorSocket;

/** Base target socket implementing protocol/width checking. */
template <typename Types>
class BaseTargetSocket : public tlm::tlm_target_socket<0, Types>
{
public:
    /* Protocol to test against other sockets when binding. */
    const Protocol protocol;

    /** Port data width to test against other sockets when binding. */
    const unsigned port_width;

public:
    BaseTargetSocket(const char* name_,
        Protocol protocol_, unsigned port_width_) :
        tlm::tlm_target_socket<0, Types>(name_),
        protocol(protocol_),
        port_width(port_width_)
    {}

    /*
     * Base types for sockets used in SystemC to define bind. These are needed
     * to match the type signature of bind correctly for the overrides below.
     */
    typedef tlm::tlm_base_initiator_socket_b<0, tlm::tlm_fw_transport_if<Types>,
        tlm::tlm_bw_transport_if<Types> > base_initiator_socket;

    typedef tlm::tlm_base_target_socket_b<0, tlm::tlm_fw_transport_if<Types>,
        tlm::tlm_bw_transport_if<Types> > base_target_socket;

    /* Explicitly protocol checking version of bind. */
    void protocol_check_bind(BaseTargetSocket<Types>& socket);
    void protocol_check_bind(BaseInitiatorSocket<Types>& socket);

    /*
     * Protocol checking overloadings of in TLM 2.0.2 (SystemC 2.3.0). In TLM
     * 2.0.1 (and earlier), bind isn't virtual and so protocol checking will be
     * hit and miss depending on how bind is called.
     */
    void bind(base_target_socket& socket);
    void bind(base_initiator_socket& socket);

    /* Use all other forms of bind from the parent. */
    using tlm::tlm_target_socket<0, Types>::bind;
};

/** Base initiator socket implementing protocol/width checking. */
template <typename Types>
class BaseInitiatorSocket : public tlm::tlm_initiator_socket<0, Types>
{
public:
    /* Protocol to test against other sockets when binding. */
    const Protocol protocol;

    /** Port data width to test against other sockets when binding. */
    const unsigned port_width;

public:
    BaseInitiatorSocket(const char* name_,
        Protocol protocol_, unsigned port_width_) :
        tlm::tlm_initiator_socket<0, Types>(name_),
        protocol(protocol_),
        port_width(port_width_)
    {}

    /*
     * Base types for sockets used in SystemC to define bind. These are needed
     * to match the type signature of bind correctly for the overrides below.
     */
    typedef tlm::tlm_base_initiator_socket_b<0, tlm::tlm_fw_transport_if<Types>,
        tlm::tlm_bw_transport_if<Types> > base_initiator_socket;

    typedef tlm::tlm_base_target_socket_b<0, tlm::tlm_fw_transport_if<Types>,
        tlm::tlm_bw_transport_if<Types> > base_target_socket;

    /* Explicitly protocol checking version of bind. */
    void protocol_check_bind(BaseTargetSocket<Types>& socket);
    void protocol_check_bind(BaseInitiatorSocket<Types>& socket);

    /*
     * Protocol checking overloadings of in TLM 2.0.2 (SystemC 2.3.0). In TLM
     * 2.0.1 (and earlier), bind isn't virtual and so protocol checking will be
     * hit and miss depending on how bind is called.
     */
    void bind(base_target_socket& socket);
    void bind(base_initiator_socket& socket);

    /* Use all other forms of bind from the parent. */
    using tlm::tlm_initiator_socket<0, Types>::bind;
};

template <typename Types>
void BaseTargetSocket<Types>::protocol_check_bind(
    BaseTargetSocket<Types>& socket)
{
    if (socket.protocol != protocol || socket.port_width != port_width)
    {
        std::ostringstream message;
        message << this->name() << ": Protocol/width mismatch on socket: "
            << "(target) "
            << protocol << '/' << port_width
            << " <-> (parent target) "
            << socket.protocol << '/' << socket.port_width;
        SC_REPORT_ERROR("/ARM/TLM/BaseTargetSocket", message.str().c_str());
    }
    tlm::tlm_target_socket<0, Types>::bind(socket);
}

template <typename Types>
void BaseTargetSocket<Types>::protocol_check_bind(
    BaseInitiatorSocket<Types>& socket)
{
    if (socket.protocol != protocol || socket.port_width != port_width)
    {
        std::ostringstream message;
        message << this->name() << ": Protocol/width mismatch on socket: "
            << "(target) "
            << protocol << '/' << port_width
            << " <-> (initiator) "
            << socket.protocol << '/' << socket.port_width;
        SC_REPORT_ERROR("/ARM/TLM/BaseTargetSocket", message.str().c_str());
    }
    tlm::tlm_target_socket<0, Types>::bind(socket);
}

template <typename Types>
void BaseTargetSocket<Types>::bind(base_target_socket& socket)
{
    BaseTargetSocket<Types>* arm_base =
        dynamic_cast<BaseTargetSocket<Types>*>(&socket);

    if (arm_base)
        protocol_check_bind(*arm_base);
    else
        tlm::tlm_target_socket<0, Types>::bind(socket);
}

template <typename Types>
void BaseTargetSocket<Types>::bind(base_initiator_socket& socket)
{
    BaseInitiatorSocket<Types>* arm_base =
        dynamic_cast<BaseInitiatorSocket<Types>*>(&socket);

    if (arm_base)
        protocol_check_bind(*arm_base);
    else
        tlm::tlm_target_socket<0, Types>::bind(socket);
}

template <typename Types>
void BaseInitiatorSocket<Types>::protocol_check_bind(
    BaseTargetSocket<Types>& socket)
{
    if (socket.protocol != protocol || socket.port_width != port_width)
    {
        std::ostringstream message;
        message << this->name() << ": Protocol/width mismatch on socket: "
            << "(initiator) "
            << protocol << '/' << port_width
            << " <-> (target) "
            << socket.protocol << '/' << socket.port_width;
        SC_REPORT_ERROR("/ARM/TLM/BaseInitiatorSocket", message.str().c_str());
    }
    tlm::tlm_initiator_socket<0, Types>::bind(socket);
}

template <typename Types>
void BaseInitiatorSocket<Types>::protocol_check_bind(
    BaseInitiatorSocket<Types>& socket)
{
    if (socket.protocol != protocol || socket.port_width != port_width)
    {
        std::ostringstream message;
        message << this->name() << ": Protocol/width mismatch on socket: "
            << "(initiator) "
            << protocol << '/' << port_width
            << " <-> (parent initiator) "
            << socket.protocol << '/' << socket.port_width;
        SC_REPORT_ERROR("/ARM/TLM/BaseInitiatorSocket", message.str().c_str());
    }
    tlm::tlm_initiator_socket<0, Types>::bind(socket);
}

template <typename Types>
void BaseInitiatorSocket<Types>::bind(base_target_socket& socket)
{
    BaseTargetSocket<Types>* arm_base =
        dynamic_cast<BaseTargetSocket<Types>*>(&socket);

    if (arm_base)
        protocol_check_bind(*arm_base);
    else
        tlm::tlm_initiator_socket<0, Types>::bind(socket);
}

template <typename Types>
void BaseInitiatorSocket<Types>::bind(base_initiator_socket& socket)
{
    BaseInitiatorSocket<Types>* arm_base =
        dynamic_cast<BaseInitiatorSocket<Types>*>(&socket);

    if (arm_base)
        protocol_check_bind(*arm_base);
    else
        tlm::tlm_initiator_socket<0, Types>::bind(socket);
}

/**
 * Simple target socket allowing a Module class to implement communication
 * functions as member functions and register them with the socket in the
 * same way at tlm_utils::simple_target_socket.
 */
template <typename Module, typename Types>
class SimpleTargetSocket : public BaseTargetSocket<Types>
{
protected:
    typedef typename Types::tlm_payload_type PayloadType;
    typedef typename Types::tlm_phase_type PhaseType;

public:
    /** Non blocking transport function pointer. */
    typedef tlm::tlm_sync_enum (Module::* NBFunc)(PayloadType&, PhaseType&);

    /** Debug transport function pointer. */
    typedef unsigned (Module::* DebugFunc)(PayloadType&);

protected:
    /** Proxy object implementing the fw transport interface. */
    class Proxy : public tlm::tlm_fw_transport_if<Types>
    {
    public:
        /** Owner of the proxy. */
        const SimpleTargetSocket<Module, Types>& owner;

        /** Object and functions to which to defer interface calls. */
        Module& t;
        NBFunc fw;
        DebugFunc dbg;

        Proxy(const SimpleTargetSocket<Module, Types>& owner_,
            Module& t_, NBFunc fw_, DebugFunc dbg_) :
            owner(owner_), t(t_), fw(fw_), dbg(dbg_)
        {}

        tlm::tlm_sync_enum nb_transport_fw(PayloadType& trans,
            PhaseType& phase, sc_core::sc_time& /* delay */)
        {
            return (t.*fw)(trans, phase);
        }

        void b_transport(PayloadType&, sc_core::sc_time&)
        {
            std::ostringstream message;
            message << owner.name() << ": b_transport not implemented";
            SC_REPORT_ERROR("/ARM/TLM/SimpleTargetSocket",
                message.str().c_str());
        }

        unsigned transport_dbg(PayloadType& trans)
        {
            if (dbg)
                return (t.*dbg)(trans);
            else
                return 0;
        }

        bool get_direct_mem_ptr(PayloadType&, tlm::tlm_dmi&)
        {
            return false;
        }
    };

    Proxy proxy;

public:
    SimpleTargetSocket(const char* name_, Module& t, NBFunc fw,
        Protocol protocol_, unsigned port_width_, DebugFunc dbg = nullptr) :
        BaseTargetSocket<Types>(name_, protocol_, port_width_),
        proxy(*this, t, fw, dbg)
    {
        this->bind(proxy);
    }

    /** Convenience function for bw without time. */
    tlm::tlm_sync_enum nb_transport_bw(PayloadType& trans, PhaseType& phase)
    {
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
        return (*this)->nb_transport_bw(trans, phase, delay);
    }
};

/**
 * Simple target socket allowing a Module class to implement communication
 * functions as member functions and register them with the socket in the
 * same way at tlm_utils::simple_initiator_socket.
 */
template <typename Module, typename Types>
class SimpleInitiatorSocket : public BaseInitiatorSocket<Types>
{
protected:
    typedef typename Types::tlm_payload_type PayloadType;
    typedef typename Types::tlm_phase_type PhaseType;

public:
    /** Non blocking transport function pointer. */
    typedef tlm::tlm_sync_enum (Module::* NBFunc)(PayloadType&, PhaseType&);

protected:
    /** Proxy object implementing the bw transport interface. */
    class Proxy : public tlm::tlm_bw_transport_if<Types>
    {
    public:
        /** Owner of the proxy. */
        const SimpleInitiatorSocket<Module, Types>& owner;

        /** Object and function(s) to which to defer interface calls. */
        Module& t;
        NBFunc bw;

        Proxy(const SimpleInitiatorSocket<Module, Types>& owner_,
            Module& t_, NBFunc bw_) :
            owner(owner_), t(t_), bw(bw_)
        {}

        tlm::tlm_sync_enum nb_transport_bw(PayloadType& trans,
            PhaseType& phase, sc_core::sc_time& /* delay */)
        {
            return (t.*bw)(trans, phase);
        }

        void invalidate_direct_mem_ptr(sc_dt::uint64, sc_dt::uint64)
        {
            std::ostringstream message;
            message << owner.name() << ": DMI not implemented";
            SC_REPORT_ERROR("/ARM/TLM/SimpleInitiatorSocket",
                message.str().c_str());
        }
    };

    Proxy proxy;

public:
    SimpleInitiatorSocket(const char* name_, Module& t, NBFunc bw,
        Protocol protocol_, unsigned port_width_) :
        BaseInitiatorSocket<Types>(name_, protocol_, port_width_),
        proxy(*this, t, bw)
    {
        tlm::tlm_initiator_socket<0, Types>::bind(proxy);
    }

    /** Convenience function for fw without time. */
    tlm::tlm_sync_enum nb_transport_fw(PayloadType& trans, PhaseType& phase)
    {
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
        return (*this)->nb_transport_fw(trans, phase, delay);
    }
};

}
}

#endif /* ARM_TLM_SOCKET_H */

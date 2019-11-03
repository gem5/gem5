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

#ifndef __EXTENSIONS_ADAPTORS_H__
#define __EXTENSIONS_ADAPTORS_H__

#include "tlm.h"
#include "my_extension.h"

#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"

template <unsigned int BUSWIDTH = 32>
class adapt_ext2gp : public sc_core::sc_module
{
    public:
    typedef tlm::tlm_generic_payload                   initiator_payload_type;
    typedef tlm::tlm_generic_payload                   target_payload_type;
    typedef tlm_utils::simple_initiator_socket<adapt_ext2gp, BUSWIDTH,
                                  tlm::tlm_base_protocol_types> initiator_socket_type;
    typedef tlm_utils::simple_target_socket<adapt_ext2gp, BUSWIDTH,
                               my_extended_payload_types>  target_socket_type;
    
    target_socket_type  target_socket;
    initiator_socket_type initiator_socket;

    SC_HAS_PROCESS(adapt_ext2gp);
    adapt_ext2gp(sc_core::sc_module_name name_)
        : sc_core::sc_module(name_)
    {
      target_socket.register_nb_transport_fw(this, &adapt_ext2gp::forward_nb_transport);
      target_socket.register_transport_dbg(this, &adapt_ext2gp::transport_debug);
      target_socket.register_get_direct_mem_ptr(this, &adapt_ext2gp::get_dmi_pointer);
        
      initiator_socket.register_nb_transport_bw(this, &adapt_ext2gp::backward_nb_transport);
      initiator_socket.register_invalidate_direct_mem_ptr(this, &adapt_ext2gp::invalidate_dmi_pointers);
    }

    ///////////////
    // NB transport
    ///////////////

    // Forward direction: The initiator calls this method with an extended
    // payload. We leave the extension class in the vector, and it will be
    // ignored by the GP target.
    tlm::tlm_sync_enum forward_nb_transport(initiator_payload_type& trans,
                                            tlm::tlm_phase& phase,
                                            sc_core::sc_time& t)
    {
        return initiator_socket->nb_transport_fw(trans, phase, t);
    }
    // Backward direction: we can  assume here that the payload we get
    // as parameter is the same one that the initiator sent out. Thus, the
    // extension vector is known to be present.
    tlm::tlm_sync_enum backward_nb_transport(target_payload_type& trans,
                                             tlm::tlm_phase& phase,
                                             sc_core::sc_time& t)
    {
        return target_socket->nb_transport_bw(trans, phase, t);
    }
    
    bool get_dmi_pointer(target_payload_type& trans,
                         tlm::tlm_dmi& dmi_data)
    {
        bool tmp_ret = initiator_socket->get_direct_mem_ptr(trans,
                                                            dmi_data);
        return tmp_ret;
    }

    //////////////////////////
    // simple call forwarders:
    //////////////////////////
    unsigned int transport_debug(target_payload_type& trans)
    {
        return initiator_socket->transport_dbg(trans);
    }
    void invalidate_dmi_pointers(sc_dt::uint64 start_range,
                                 sc_dt::uint64 end_range)
    {
        target_socket->invalidate_direct_mem_ptr(start_range,
                                                 end_range);
    }
};

template <unsigned int BUSWIDTH = 32>
class adapt_gp2ext : public sc_core::sc_module
{
    public:
    typedef tlm::tlm_generic_payload                   initiator_payload_type;
    typedef tlm::tlm_generic_payload                   target_payload_type;
    typedef tlm_utils::simple_initiator_socket<adapt_gp2ext, BUSWIDTH,
                                  my_extended_payload_types> initiator_socket_type;
    typedef tlm_utils::simple_target_socket<adapt_gp2ext, BUSWIDTH,
                               tlm::tlm_base_protocol_types> target_socket_type;
    
    target_socket_type  target_socket;
    initiator_socket_type initiator_socket;

    SC_HAS_PROCESS(adapt_gp2ext);
    adapt_gp2ext(sc_core::sc_module_name name_)
        : sc_core::sc_module(name_)
    {
        // Optionally, we can initialize our private extension class
        // here, if required.

      target_socket.register_nb_transport_fw(this, &adapt_gp2ext::forward_nb_transport);
      target_socket.register_transport_dbg(this, &adapt_gp2ext::transport_debug);
      target_socket.register_get_direct_mem_ptr(this, &adapt_gp2ext::get_dmi_pointer);
        
      initiator_socket.register_nb_transport_bw(this, &adapt_gp2ext::backward_nb_transport);
      initiator_socket.register_invalidate_direct_mem_ptr(this, &adapt_gp2ext::invalidate_dmi_pointers);

        m_ext.m_data = 13;
    }

    ///////////////
    // NB transport
    ///////////////

    // Forward direction: We extend the payload on the fly (if needed).
    tlm::tlm_sync_enum forward_nb_transport(initiator_payload_type& trans,
                                            tlm::tlm_phase& phase,
                                            sc_core::sc_time& t)
    {
        // If the mandatory extension is not there, we need to add it and
        // store it so that we can re-construc the original state.
        // Otherwise we don't touch the extension, so that we don't overwrite
        // it in e.g. a nonGP->GP->nonGP (initiator->interconnect->target)
        // setup.
        // Note, however, that there might be situations where we might need to
        // re-initialize the extension, e.g. for mutable data fields in
        // different system setups.
        trans.get_extension(m_initiator_ext);
        if (!m_initiator_ext)
        {
            m_initiator_ext = trans.set_extension(&m_ext);
        }
        tlm::tlm_sync_enum tmp =
        initiator_socket->nb_transport_fw(trans, phase, t);
        if (tmp == tlm::TLM_COMPLETED)
        {
            m_initiator_ext = trans.set_extension(m_initiator_ext);            
        }
        return tmp;
    }
    // Backward direction: only restore of original extension and static_cast.
    tlm::tlm_sync_enum backward_nb_transport(target_payload_type& trans,
                                             tlm::tlm_phase& phase,
                                             sc_core::sc_time& t)
    {
        m_initiator_ext = trans.set_extension(m_initiator_ext);
        return target_socket->nb_transport_bw(trans, phase, t);
    }

    bool get_dmi_pointer(target_payload_type& trans,
                         tlm::tlm_dmi& dmi_data)
    {
        // If the mandatory extension is not there, we need to add it and
        // store it so that we can re-construc the original state.
        // Otherwise we don't touch the extension, so that we don't overwrite
        // it in e.g. a nonGP->GP->nonGP (initiator->interconnect->target)
        // setup.
        my_extension* tmp_ext;
        trans.get_extension(tmp_ext);
        if (!tmp_ext)
        {
            trans.set_extension(&m_ext);
        }
        bool tmp_ret = initiator_socket->get_direct_mem_ptr(trans,
                                                            dmi_data);
        if(!tmp_ext)
        {
            trans.clear_extension(tmp_ext);
        }
        return tmp_ret;
    }
    //////////////////////////
    // simple call forwarders:
    //////////////////////////
    unsigned int transport_debug(target_payload_type& trans)
    {
        return initiator_socket->transport_dbg(trans);
    }
    void invalidate_dmi_pointers(sc_dt::uint64 start_range,
                                 sc_dt::uint64 end_range)
    {
        target_socket->invalidate_direct_mem_ptr(start_range,
                                                 end_range);
    }

private:
    my_extension  m_ext;
    my_extension* m_initiator_ext;
};

#endif

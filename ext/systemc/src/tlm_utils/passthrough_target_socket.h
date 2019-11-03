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

#ifndef __PASSTHROUGH_TARGET_SOCKET_H__
#define __PASSTHROUGH_TARGET_SOCKET_H__

#include <tlm>
#include <sstream>

namespace tlm_utils {

template <typename MODULE,
          unsigned int BUSWIDTH = 32,
          typename TYPES = tlm::tlm_base_protocol_types>
class passthrough_target_socket :
  public tlm::tlm_target_socket<BUSWIDTH, TYPES>
{
public:
  typedef typename TYPES::tlm_payload_type              transaction_type;
  typedef typename TYPES::tlm_phase_type                phase_type;
  typedef tlm::tlm_sync_enum                            sync_enum_type;
  typedef tlm::tlm_fw_transport_if<TYPES>               fw_interface_type;
  typedef tlm::tlm_bw_transport_if<TYPES>               bw_interface_type;
  typedef tlm::tlm_target_socket<BUSWIDTH, TYPES>       base_type;

public:
  passthrough_target_socket() :
    base_type(sc_core::sc_gen_unique_name("passthrough_target_socket")),
    m_process(this->name())
  {
    bind(m_process);
  }

  explicit passthrough_target_socket(const char* n) :
    base_type(n),
    m_process(this->name())
  {
    bind(m_process);
  }

  using tlm::tlm_target_socket<BUSWIDTH, TYPES>::bind;

  // REGISTER_XXX
  void register_nb_transport_fw(MODULE* mod,
                                sync_enum_type (MODULE::*cb)(transaction_type&,
                                                             phase_type&,
                                                             sc_core::sc_time&))
  {
    m_process.set_nb_transport_ptr(mod, cb);
  }

  void register_b_transport(MODULE* mod,
                            void (MODULE::*cb)(transaction_type&,
                                               sc_core::sc_time&))
  {
    m_process.set_b_transport_ptr(mod, cb);
  }

  void register_transport_dbg(MODULE* mod,
                              unsigned int (MODULE::*cb)(transaction_type&))
  {
    m_process.set_transport_dbg_ptr(mod, cb);
  }

  void register_get_direct_mem_ptr(MODULE* mod,
                                   bool (MODULE::*cb)(transaction_type&,
                                                      tlm::tlm_dmi&))
  {
    m_process.set_get_direct_mem_ptr(mod, cb);
  }

private:
  class process : public tlm::tlm_fw_transport_if<TYPES>
  {
  public:
    typedef sync_enum_type (MODULE::*NBTransportPtr)(transaction_type&,
                                                     phase_type&,
                                                     sc_core::sc_time&);
    typedef void (MODULE::*BTransportPtr)(transaction_type&,
                                            sc_core::sc_time&);
    typedef unsigned int (MODULE::*TransportDbgPtr)(transaction_type&);
    typedef bool (MODULE::*GetDirectMem_ptr)(transaction_type&,
                                               tlm::tlm_dmi&);
      
    process(const std::string& name) :
      m_name(name),
      m_mod(0),
      m_nb_transport_ptr(0),
      m_b_transport_ptr(0),
      m_transport_dbg_ptr(0),
      m_get_direct_mem_ptr(0)
    {
    }
  
    void set_nb_transport_ptr(MODULE* mod, NBTransportPtr p)
    {
      if (m_nb_transport_ptr) {
        std::stringstream s;
        s << m_name << ": non-blocking callback allready registered";
        SC_REPORT_WARNING("/OSCI_TLM-2/passthrough_socket",s.str().c_str());

      } else {
        assert(!m_mod || m_mod == mod);
        m_mod = mod;
        m_nb_transport_ptr = p;
      }
    }

    void set_b_transport_ptr(MODULE* mod, BTransportPtr p)
    {
      if (m_b_transport_ptr) {
        std::stringstream s;
        s << m_name << ": blocking callback allready registered";
        SC_REPORT_WARNING("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      } else {
        assert(!m_mod || m_mod == mod);
        m_mod = mod;
        m_b_transport_ptr = p;
      }
    }

    void set_transport_dbg_ptr(MODULE* mod, TransportDbgPtr p)
    {
      if (m_transport_dbg_ptr) {
        std::stringstream s;
        s << m_name << ": debug callback allready registered";
        SC_REPORT_WARNING("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      } else {
        assert(!m_mod || m_mod == mod);
        m_mod = mod;
        m_transport_dbg_ptr = p;
      }
    }

    void set_get_direct_mem_ptr(MODULE* mod, GetDirectMem_ptr p)
    {
      if (m_get_direct_mem_ptr) {
        std::stringstream s;
        s << m_name << ": get DMI pointer callback allready registered";
        SC_REPORT_WARNING("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      } else {
        assert(!m_mod || m_mod == mod);
        m_mod = mod;
        m_get_direct_mem_ptr = p;
      }
    }

    sync_enum_type nb_transport_fw(transaction_type& trans,
                                   phase_type& phase,
                                   sc_core::sc_time& t)
    {
      if (m_nb_transport_ptr) {
        // forward call
        assert(m_mod);
        return (m_mod->*m_nb_transport_ptr)(trans, phase, t);

      } else {
        std::stringstream s;
        s << m_name << ": no non-blocking callback registered";
        SC_REPORT_ERROR("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      }
      return tlm::TLM_ACCEPTED;   ///< unreachable code
    }

    void b_transport(transaction_type& trans, sc_core::sc_time& t)
    {
      if (m_b_transport_ptr) {
        // forward call
        assert(m_mod);
        return (m_mod->*m_b_transport_ptr)(trans, t);

      } else {
        std::stringstream s;
        s << m_name << ": no blocking callback registered";
        SC_REPORT_ERROR("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      }
    }

    unsigned int transport_dbg(transaction_type& trans)
    {
      if (m_transport_dbg_ptr) {
        // forward call
        assert(m_mod);
        return (m_mod->*m_transport_dbg_ptr)(trans);

      } else {
        // No debug support
        return 0;
      }
    }

    bool get_direct_mem_ptr(transaction_type& trans,
                            tlm::tlm_dmi&  dmi_data)
    {
      if (m_get_direct_mem_ptr) {
        // forward call
        assert(m_mod);
        return (m_mod->*m_get_direct_mem_ptr)(trans, dmi_data);

      } else {
        // No DMI support
        dmi_data.allow_read_write();
        dmi_data.set_start_address(0x0);
        dmi_data.set_end_address((sc_dt::uint64)-1);
        return false;
      }
    }

  private:
    const std::string m_name;
    MODULE* m_mod;
    NBTransportPtr m_nb_transport_ptr;
    BTransportPtr m_b_transport_ptr;
    TransportDbgPtr m_transport_dbg_ptr;
    GetDirectMem_ptr m_get_direct_mem_ptr;
  };

private:
  process m_process;
};

//ID Tagged version
template <typename MODULE,
          unsigned int BUSWIDTH = 32,
          typename TYPES = tlm::tlm_base_protocol_types>
class passthrough_target_socket_tagged :
  public tlm::tlm_target_socket<BUSWIDTH, TYPES>
{
public:
  typedef typename TYPES::tlm_payload_type              transaction_type;
  typedef typename TYPES::tlm_phase_type                phase_type;
  typedef tlm::tlm_sync_enum                            sync_enum_type;
  typedef tlm::tlm_fw_transport_if<TYPES>               fw_interface_type;
  typedef tlm::tlm_bw_transport_if<TYPES>               bw_interface_type;
  typedef tlm::tlm_target_socket<BUSWIDTH, TYPES>       base_type;

public:
  passthrough_target_socket_tagged() :
    base_type(sc_core::sc_gen_unique_name("passthrough_target_socket_tagged")),
    m_process(this->name())
  {
    bind(m_process);
  }

  explicit passthrough_target_socket_tagged(const char* n) :
    base_type(n),
    m_process(this->name())
  {
    bind(m_process);
  }

  using tlm::tlm_target_socket<BUSWIDTH, TYPES>::bind;

  // REGISTER_XXX
  void register_nb_transport_fw(MODULE* mod,
                                sync_enum_type (MODULE::*cb)(int id,
                                                             transaction_type&,
                                                             phase_type&,
                                                             sc_core::sc_time&),
                                int id)
  {
    m_process.set_nb_transport_ptr(mod, cb);
    m_process.set_nb_transport_user_id(id);
  }

  void register_b_transport(MODULE* mod,
                            void (MODULE::*cb)(int id,
                                               transaction_type&,
                                               sc_core::sc_time&),
                            int id)
  {
    m_process.set_b_transport_ptr(mod, cb);
    m_process.set_b_transport_user_id(id);
  }

  void register_transport_dbg(MODULE* mod,
                              unsigned int (MODULE::*cb)(int id,
                                                         transaction_type&),
                              int id)
  {
    m_process.set_transport_dbg_ptr(mod, cb);
    m_process.set_transport_dbg_user_id(id);
  }

  void register_get_direct_mem_ptr(MODULE* mod,
                                   bool (MODULE::*cb)(int id,
                                                      transaction_type&,
                                                      tlm::tlm_dmi&),
                                   int id)
  {
    m_process.set_get_direct_mem_ptr(mod, cb);
    m_process.set_get_dmi_user_id(id);
  }

private:
  class process : public tlm::tlm_fw_transport_if<TYPES>
  {
  public:
    typedef sync_enum_type (MODULE::*NBTransportPtr)(int id,
                                                     transaction_type&,
                                                     phase_type&,
                                                     sc_core::sc_time&);
    typedef void (MODULE::*BTransportPtr)(int id,
                                          transaction_type&,
                                          sc_core::sc_time&);
    typedef unsigned int (MODULE::*TransportDbgPtr)(int id,
                                                    transaction_type&);
    typedef bool (MODULE::*GetDirectMem_ptr)(int id,
                                             transaction_type&,
                                             tlm::tlm_dmi&);
      
    process(const std::string& name) :
      m_name(name),
      m_mod(0),
      m_nb_transport_ptr(0),
      m_b_transport_ptr(0),
      m_transport_dbg_ptr(0),
      m_get_direct_mem_ptr(0),
      m_nb_transport_user_id(0),
      m_b_transport_user_id(0),
      m_transport_dbg_user_id(0),
      m_get_dmi_user_id(0)
    {
    }

    void set_nb_transport_user_id(int id) { m_nb_transport_user_id = id; }
    void set_b_transport_user_id(int id) { m_b_transport_user_id = id; }
    void set_transport_dbg_user_id(int id) { m_transport_dbg_user_id = id; }
    void set_get_dmi_user_id(int id) { m_get_dmi_user_id = id; }

    void set_nb_transport_ptr(MODULE* mod, NBTransportPtr p)
    {
      if (m_nb_transport_ptr) {
        std::stringstream s;
        s << m_name << ": non-blocking callback allready registered";
        SC_REPORT_WARNING("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      } else {
        assert(!m_mod || m_mod == mod);
        m_mod = mod;
        m_nb_transport_ptr = p;
      }
    }

    void set_b_transport_ptr(MODULE* mod, BTransportPtr p)
    {
      if (m_b_transport_ptr) {
        std::stringstream s;
        s << m_name << ": blocking callback allready registered";
        SC_REPORT_WARNING("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      } else {
        assert(!m_mod || m_mod == mod);
        m_mod = mod;
        m_b_transport_ptr = p;
      }
    }

    void set_transport_dbg_ptr(MODULE* mod, TransportDbgPtr p)
    {
      if (m_transport_dbg_ptr) {
        std::stringstream s;
        s << m_name << ": debug callback allready registered";
        SC_REPORT_WARNING("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      } else {
        assert(!m_mod || m_mod == mod);
        m_mod = mod;
        m_transport_dbg_ptr = p;
      }
    }

    void set_get_direct_mem_ptr(MODULE* mod, GetDirectMem_ptr p)
    {
      if (m_get_direct_mem_ptr) {
        std::stringstream s;
        s << m_name << ": get DMI pointer callback allready registered";
        SC_REPORT_WARNING("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      } else {
        assert(!m_mod || m_mod == mod);
        m_mod = mod;
        m_get_direct_mem_ptr = p;
      }
    }

    sync_enum_type nb_transport_fw(transaction_type& trans,
                                   phase_type& phase,
                                   sc_core::sc_time& t)
    {
      if (m_nb_transport_ptr) {
        // forward call
        assert(m_mod);
        return (m_mod->*m_nb_transport_ptr)(m_nb_transport_user_id, trans, phase, t);

      } else {
        std::stringstream s;
        s << m_name << ": no non-blocking callback registered";
        SC_REPORT_ERROR("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      }
      return tlm::TLM_ACCEPTED;   ///< unreachable code
    }

    void b_transport(transaction_type& trans, sc_core::sc_time& t)
    {
      if (m_b_transport_ptr) {
        // forward call
        assert(m_mod);
        return (m_mod->*m_b_transport_ptr)(m_b_transport_user_id, trans, t);

      } else {
        std::stringstream s;
        s << m_name << ": no blocking callback registered";
        SC_REPORT_ERROR("/OSCI_TLM-2/passthrough_socket",s.str().c_str());
      }
    }

    unsigned int transport_dbg(transaction_type& trans)
    {
      if (m_transport_dbg_ptr) {
        // forward call
        assert(m_mod);
        return (m_mod->*m_transport_dbg_ptr)(m_transport_dbg_user_id, trans);

      } else {
        // No debug support
        return 0;
      }
    }

    bool get_direct_mem_ptr(transaction_type& trans,
                            tlm::tlm_dmi&  dmi_data)
    {
      if (m_get_direct_mem_ptr) {
        // forward call
        assert(m_mod);
        return (m_mod->*m_get_direct_mem_ptr)(m_get_dmi_user_id, trans, dmi_data);

      } else {
        // No DMI support
        dmi_data.allow_read_write();
        dmi_data.set_start_address(0x0);
        dmi_data.set_end_address((sc_dt::uint64)-1);
        return false;
      }
    }

  private:
    const std::string m_name;
    MODULE* m_mod;
    NBTransportPtr m_nb_transport_ptr;
    BTransportPtr m_b_transport_ptr;
    TransportDbgPtr m_transport_dbg_ptr;
    GetDirectMem_ptr m_get_direct_mem_ptr;
    int m_nb_transport_user_id;
    int m_b_transport_user_id;
    int m_transport_dbg_user_id;
    int m_get_dmi_user_id;
  };

private:
  process m_process;
};

}

#endif

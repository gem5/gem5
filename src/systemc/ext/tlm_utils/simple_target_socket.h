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

#ifndef __SYSTEMC_EXT_TLM_UTILS_SIMPLE_TARGET_SOCKET_H__
#define __SYSTEMC_EXT_TLM_UTILS_SIMPLE_TARGET_SOCKET_H__

#include "../core/sc_event.hh"
#include "../core/sc_module.hh"
#include "../core/sc_port.hh"
#include "../core/sc_spawn.hh"
#include "../tlm_core/2/generic_payload/gp.hh"
#include "../tlm_core/2/interfaces/fw_bw_ifs.hh"
#include "../tlm_core/2/sockets/initiator_socket.hh"
#include "../tlm_core/2/sockets/target_socket.hh"
#include "../utils/sc_report_handler.hh"
#include "convenience_socket_bases.h"
#include "peq_with_get.h"

namespace tlm_utils
{

template <typename MODULE, unsigned int BUSWIDTH, typename TYPES,
          sc_core::sc_port_policy POL=sc_core::SC_ONE_OR_MORE_BOUND>
class simple_target_socket_b :
    public tlm::tlm_target_socket<BUSWIDTH, TYPES, 1, POL>,
    protected simple_socket_base
{
    friend class fw_process;
    friend class bw_process;
  public:
    typedef typename TYPES::tlm_payload_type transaction_type;
    typedef typename TYPES::tlm_phase_type phase_type;
    typedef tlm::tlm_sync_enum sync_enum_type;
    typedef tlm::tlm_fw_transport_if<TYPES> fw_interface_type;
    typedef tlm::tlm_bw_transport_if<TYPES> bw_interface_type;
    typedef tlm::tlm_target_socket<BUSWIDTH, TYPES, 1, POL>  base_type;

  public:
    static const char *
    default_name()
    {
        return sc_core::sc_gen_unique_name("simple_target_socket");
    }

    explicit simple_target_socket_b(const char *n=default_name()) :
        base_type(n), m_fw_process(this), m_bw_process(this)
    {
        bind(m_fw_process);
    }

    using base_type::bind;

    // bw transport must come through us.
    tlm::tlm_bw_transport_if<TYPES> *operator -> () { return &m_bw_process; }

    // REGISTER_XXX
    void
    register_nb_transport_fw(MODULE *mod,
            sync_enum_type (MODULE::*cb)(
                transaction_type &, phase_type &, sc_core::sc_time &))
    {
        elaboration_check("register_nb_transport_fw");
        m_fw_process.set_nb_transport_ptr(mod, cb);
    }

    void
    register_b_transport(MODULE *mod,
            void (MODULE::*cb)(transaction_type &, sc_core::sc_time &))
    {
        elaboration_check("register_b_transport");
        m_fw_process.set_b_transport_ptr(mod, cb);
    }

    void
    register_transport_dbg(MODULE *mod,
            unsigned int (MODULE::*cb)(transaction_type &))
    {
        elaboration_check("register_transport_dbg");
        m_fw_process.set_transport_dbg_ptr(mod, cb);
    }

    void
    register_get_direct_mem_ptr(MODULE *mod,
            bool (MODULE::*cb)(transaction_type &, tlm::tlm_dmi &))
    {
        elaboration_check("register_get_direct_mem_ptr");
        m_fw_process.set_get_direct_mem_ptr(mod, cb);
    }

  protected:
    void
    start_of_simulation()
    {
        base_type::start_of_simulation();
        m_fw_process.start_of_simulation();
    }

  private:
    // Make call on bw path.
    sync_enum_type
    bw_nb_transport(transaction_type &trans, phase_type &phase,
                    sc_core::sc_time &t)
    {
        return base_type::operator -> ()->nb_transport_bw(trans, phase, t);
    }

    void
    bw_invalidate_direct_mem_ptr(sc_dt::uint64 s, sc_dt::uint64 e)
    {
        base_type::operator -> ()->invalidate_direct_mem_ptr(s, e);
    }

    // Helper class to handle bw path calls Needed to detect transaction end
    // when called from b_transport.
    class bw_process : public tlm::tlm_bw_transport_if<TYPES>
    {
      public:
        bw_process(simple_target_socket_b *p_own) : m_owner(p_own) {}

        sync_enum_type
        nb_transport_bw(transaction_type &trans, phase_type &phase,
                        sc_core::sc_time &t)
        {
            typename std::map<transaction_type *,
                              sc_core::sc_event *>::iterator it =
                                  m_owner->m_pending_trans.find(&trans);

            if (it == m_owner->m_pending_trans.end()) {
                // Not a blocking call, forward.
                return m_owner->bw_nb_transport(trans, phase, t);

            }

            if (phase == tlm::END_REQ) {
                m_owner->m_end_request.notify(sc_core::SC_ZERO_TIME);
                return tlm::TLM_ACCEPTED;
            }
            if (phase == tlm::BEGIN_RESP) {
                if (m_owner->m_current_transaction == &trans) {
                    m_owner->m_end_request.notify(sc_core::SC_ZERO_TIME);
                }
                it->second->notify(t);
                m_owner->m_pending_trans.erase(it);
                return tlm::TLM_COMPLETED;
            }
            m_owner->display_error("invalid phase received");
            return tlm::TLM_COMPLETED;
        }

        void
        invalidate_direct_mem_ptr(sc_dt::uint64 s, sc_dt::uint64 e)
        {
            return m_owner->bw_invalidate_direct_mem_ptr(s, e);
        }

      private:
        simple_target_socket_b *m_owner;
    };

    class fw_process : public tlm::tlm_fw_transport_if<TYPES>,
                      public tlm::tlm_mm_interface
    {
      public:
        typedef sync_enum_type (MODULE::*NBTransportPtr)(
                transaction_type &, phase_type &, sc_core::sc_time &);
        typedef void (MODULE::*BTransportPtr)(
                transaction_type &, sc_core::sc_time &);
        typedef unsigned int (MODULE::*TransportDbgPtr)(transaction_type &);
        typedef bool (MODULE::*GetDirectMemPtr)(
                transaction_type &, tlm::tlm_dmi &);

        fw_process(simple_target_socket_b *p_own) :
            m_owner(p_own), m_mod(0), m_nb_transport_ptr(0),
            m_b_transport_ptr(0), m_transport_dbg_ptr(0),
            m_get_direct_mem_ptr(0),
            m_peq(sc_core::sc_gen_unique_name("m_peq")),
            m_response_in_progress(false)
        {}

        void
        start_of_simulation()
        {
            // Only spawn b2nb_thread, if needed.
            if (!m_b_transport_ptr && m_nb_transport_ptr) {
                sc_core::sc_spawn_options opts;
                opts.set_sensitivity(&m_peq.get_event());
                opts.dont_initialize();
                sc_core::sc_spawn(sc_bind(&fw_process::b2nb_thread, this),
                        sc_core::sc_gen_unique_name("b2nb_thread"), &opts);
            }
        }

        void
        set_nb_transport_ptr(MODULE *mod, NBTransportPtr p)
        {
            if (m_nb_transport_ptr) {
                m_owner->display_warning(
                        "non-blocking callback already registered");
                return;
            }
            sc_assert(!m_mod || m_mod == mod);
            m_mod = mod;
            m_nb_transport_ptr = p;
        }

        void
        set_b_transport_ptr(MODULE *mod, BTransportPtr p)
        {
            if (m_b_transport_ptr) {
                m_owner->display_warning(
                        "blocking callback already registered");
                return;
            }
            sc_assert(!m_mod || m_mod == mod);
            m_mod = mod;
            m_b_transport_ptr = p;
        }

        void
        set_transport_dbg_ptr(MODULE *mod, TransportDbgPtr p)
        {
            if (m_transport_dbg_ptr) {
                m_owner->display_warning("debug callback already registered");
                return;
            }
            sc_assert(!m_mod || m_mod == mod);
            m_mod = mod;
            m_transport_dbg_ptr = p;
        }

        void
        set_get_direct_mem_ptr(MODULE *mod, GetDirectMemPtr p)
        {
            if (m_get_direct_mem_ptr) {
                m_owner->display_warning(
                        "get DMI pointer callback already registered");
                return;
            }
            sc_assert(!m_mod || m_mod == mod);
            m_mod = mod;
            m_get_direct_mem_ptr = p;
        }

        // Interface implementation.
        sync_enum_type
        nb_transport_fw(transaction_type &trans, phase_type &phase,
                        sc_core::sc_time & t)
        {
            if (m_nb_transport_ptr) {
                // Forward call.
                sc_assert(m_mod);
                return (m_mod->*m_nb_transport_ptr)(trans, phase, t);
            }

            // nb->b conversion
            if (m_b_transport_ptr) {
                if (phase == tlm::BEGIN_REQ) {
                    // Prepare thread to do blocking call.
                    process_handle_class *ph =
                        m_process_handle.get_handle(&trans);

                    if (!ph) { // Create new dynamic process.
                        ph = new process_handle_class(&trans);
                        m_process_handle.put_handle(ph);

                        sc_core::sc_spawn_options opts;
                        opts.dont_initialize();
                        opts.set_sensitivity(&ph->m_e);

                        sc_core::sc_spawn(
                                sc_bind(&fw_process::nb2b_thread, this, ph),
                                sc_core::sc_gen_unique_name("nb2b_thread"),
                                &opts);
                    }

                    ph->m_e.notify(t);
                    return tlm::TLM_ACCEPTED;
                }
                if (phase == tlm::END_RESP) {
                    m_response_in_progress = false;
                    m_end_response.notify(t);
                    return tlm::TLM_COMPLETED;
                }
                m_owner->display_error("invalid phase received");
                return tlm::TLM_COMPLETED;
            }
             m_owner->display_error(
                     "no non-blocking transport callback registered");
            return tlm::TLM_COMPLETED;
        }

        void
        b_transport(transaction_type &trans, sc_core::sc_time &t)
        {
            if (m_b_transport_ptr) {
                // Forward call.
                sc_assert(m_mod);
                (m_mod->*m_b_transport_ptr)(trans, t);
                return;
            }

            // b->nb conversion
            if (m_nb_transport_ptr) {
                m_peq.notify(trans, t);
                t = sc_core::SC_ZERO_TIME;

                mm_end_event_ext mm_ext;
                const bool mm_added = !trans.has_mm();

                if (mm_added) {
                    trans.set_mm(this);
                    trans.set_auto_extension(&mm_ext);
                    trans.acquire();
                }

                // Wait until transaction is finished.
                sc_core::sc_event end_event;
                m_owner->m_pending_trans[&trans] = &end_event;
                sc_core::wait(end_event);

                if (mm_added) {
                    // Release will not delete the transaction, it will
                    // notify mm_ext.done.
                    trans.release();
                    if (trans.get_ref_count()) {
                        sc_core::wait(mm_ext.done);
                    }
                    trans.set_mm(0);
                }
                return;
            }

            // Should not be reached.
            m_owner->display_error(
                    "no blocking transport callback registered");
        }

        unsigned int
        transport_dbg(transaction_type &trans)
        {
            if (m_transport_dbg_ptr) {
                // Forward call.
                sc_assert(m_mod);
                return (m_mod->*m_transport_dbg_ptr)(trans);
            }
            // No debug support.
            return 0;
        }

        bool
        get_direct_mem_ptr(transaction_type &trans, tlm::tlm_dmi &dmi_data)
        {
            if (m_get_direct_mem_ptr) {
                // Forward call.
                sc_assert(m_mod);
                return (m_mod->*m_get_direct_mem_ptr)(trans, dmi_data);
            }
            // No DMI support.
            dmi_data.allow_read_write();
            dmi_data.set_start_address(0x0);
            dmi_data.set_end_address((sc_dt::uint64)-1);
            return false;
        }

      private:

        // Dynamic process handler for nb2b conversion.

        class process_handle_class
        {
          public:
            explicit process_handle_class(transaction_type *trans) :
                m_trans(trans), m_suspend(false)
            {}

            transaction_type *m_trans;
            sc_core::sc_event m_e;
            bool m_suspend;
        };

        class process_handle_list
        {
          public:
            process_handle_list() {}

            ~process_handle_list()
            {
                for (typename std::vector<
                        process_handle_class *>::iterator it = v.begin(),
                        end = v.end(); it != end; ++it) {
                    delete *it;
                }
            }

            process_handle_class *
            get_handle(transaction_type *trans)
            {
                typename std::vector<process_handle_class *>::iterator it;

                for (it = v.begin(); it != v.end(); it++) {
                    if ((*it)->m_suspend) {
                        // Found suspended dynamic process, re-use it.
                        (*it)->m_trans = trans; // Replace to new one.
                        (*it)->m_suspend = false;
                        return *it;
                    }
                }
                return NULL; // No suspended process.
            }

            void
            put_handle(process_handle_class *ph)
            {
                v.push_back(ph);
            }

          private:
            std::vector<process_handle_class*> v;
        };

        process_handle_list m_process_handle;

        void
        nb2b_thread(process_handle_class *h)
        {
            while (1) {
                transaction_type *trans = h->m_trans;
                sc_core::sc_time t = sc_core::SC_ZERO_TIME;

                // Forward call.
                sc_assert(m_mod);
                (m_mod->*m_b_transport_ptr)(*trans, t);

                sc_core::wait(t);

                // Return path.
                while (m_response_in_progress) {
                    sc_core::wait(m_end_response);
                }
                t = sc_core::SC_ZERO_TIME;
                phase_type phase = tlm::BEGIN_RESP;
                sync_enum_type sync =
                    m_owner->bw_nb_transport(*trans, phase, t);
                if (!(sync == tlm::TLM_COMPLETED ||
                            (sync == tlm::TLM_UPDATED &&
                             phase == tlm::END_RESP))) {
                    m_response_in_progress = true;
                }

                // Suspend until next transaction.
                h->m_suspend = true;
                sc_core::wait();
            }
        }

        void
        b2nb_thread()
        {
            while (true) {
                transaction_type *trans;
                while ((trans = m_peq.get_next_transaction()) != 0) {
                    sc_assert(m_mod);
                    sc_assert(m_nb_transport_ptr);
                    phase_type phase = tlm::BEGIN_REQ;
                    sc_core::sc_time t = sc_core::SC_ZERO_TIME;

                    switch ((m_mod->*m_nb_transport_ptr)(*trans, phase, t)) {
                      case tlm::TLM_COMPLETED:
                        {
                            // Notify transaction is finished.
                            typename std::map<transaction_type *,
                                     sc_core::sc_event *>::iterator it =
                                         m_owner->m_pending_trans.find(trans);
                            sc_assert(it != m_owner->m_pending_trans.end());
                            it->second->notify(t);
                            m_owner->m_pending_trans.erase(it);
                            break;
                        }

                      case tlm::TLM_ACCEPTED:
                      case tlm::TLM_UPDATED:
                        switch (phase) {
                          case tlm::BEGIN_REQ:
                            m_owner->m_current_transaction = trans;
                            sc_core::wait(m_owner->m_end_request);
                            m_owner->m_current_transaction = 0;
                            break;

                          case tlm::END_REQ:
                            sc_core::wait(t);
                            break;

                          case tlm::BEGIN_RESP:
                            {
                                phase = tlm::END_RESP;
                                // This line is a bug fix added in TLM-2.0.2
                                sc_core::wait(t);
                                t = sc_core::SC_ZERO_TIME;
                                (m_mod->*m_nb_transport_ptr)(
                                        *trans, phase, t);

                                // Notify transaction is finished.
                                typename std::map<transaction_type *,
                                         sc_core::sc_event *>::iterator it =
                                             m_owner->m_pending_trans.find(
                                                     trans);
                                sc_assert(it !=
                                        m_owner->m_pending_trans.end());
                                it->second->notify(t);
                                m_owner->m_pending_trans.erase(it);
                                break;
                            }

                          default:
                            m_owner->display_error("invalid phase received");
                        }
                        break;

                      default:
                        m_owner->display_error("invalid sync value received");
                    }
                }
                sc_core::wait();
            }
        }

        void
        free(tlm::tlm_generic_payload *trans)
        {
            mm_end_event_ext *ext =
                trans->template get_extension<mm_end_event_ext>();
            sc_assert(ext);
            // Notify event first before freeing extensions (reset).
            ext->done.notify();
            trans->reset();
        }

      private:
        struct mm_end_event_ext : public tlm::tlm_extension<mm_end_event_ext>
        {
            tlm::tlm_extension_base *clone() const { return NULL; }
            void free() {}
            void copy_from(tlm::tlm_extension_base const &) {}
            sc_core::sc_event done;
        };

      private:
        simple_target_socket_b *m_owner;
        MODULE *m_mod;
        NBTransportPtr m_nb_transport_ptr;
        BTransportPtr m_b_transport_ptr;
        TransportDbgPtr m_transport_dbg_ptr;
        GetDirectMemPtr m_get_direct_mem_ptr;
        peq_with_get<transaction_type> m_peq;
        bool m_response_in_progress;
        sc_core::sc_event m_end_response;
    };

  private:
    const sc_core::sc_object *get_socket() const { return this; }

  private:
    fw_process m_fw_process;
    bw_process m_bw_process;
    std::map<transaction_type *, sc_core::sc_event *> m_pending_trans;
    sc_core::sc_event m_end_request;
    transaction_type* m_current_transaction;
};

template <typename MODULE, unsigned int BUSWIDTH=32,
          typename TYPES=tlm::tlm_base_protocol_types>
class simple_target_socket :
    public simple_target_socket_b<MODULE, BUSWIDTH, TYPES>
{
    typedef simple_target_socket_b<MODULE, BUSWIDTH, TYPES> socket_b;
  public:
    simple_target_socket() : socket_b() {}
    explicit simple_target_socket(const char *name) : socket_b(name) {}
};

template <typename MODULE, unsigned int BUSWIDTH=32,
          typename TYPES=tlm::tlm_base_protocol_types>
class simple_target_socket_optional :
    public simple_target_socket_b<MODULE, BUSWIDTH, TYPES,
                                  sc_core::SC_ZERO_OR_MORE_BOUND>
{
    typedef simple_target_socket_b<MODULE, BUSWIDTH, TYPES,
                                   sc_core::SC_ZERO_OR_MORE_BOUND> socket_b;
  public:
    simple_target_socket_optional() : socket_b() {}
    explicit simple_target_socket_optional(const char *name) :
        socket_b(name)
    {}
};

// ID Tagged version.
template <typename MODULE, unsigned int BUSWIDTH, typename TYPES,
          sc_core::sc_port_policy POL=sc_core::SC_ONE_OR_MORE_BOUND>
class simple_target_socket_tagged_b :
    public tlm::tlm_target_socket<BUSWIDTH, TYPES, 1, POL>,
    protected simple_socket_base
{
    friend class fw_process;
    friend class bw_process;
  public:
    typedef typename TYPES::tlm_payload_type transaction_type;
    typedef typename TYPES::tlm_phase_type phase_type;
    typedef tlm::tlm_sync_enum sync_enum_type;
    typedef tlm::tlm_fw_transport_if<TYPES> fw_interface_type;
    typedef tlm::tlm_bw_transport_if<TYPES> bw_interface_type;
    typedef tlm::tlm_target_socket<BUSWIDTH, TYPES, 1, POL> base_type;

  public:
    static const char *
    default_name()
    {
        return sc_core::sc_gen_unique_name("simple_target_socket_tagged");
    }

    explicit simple_target_socket_tagged_b(const char *n=default_name()) :
        base_type(n), m_fw_process(this), m_bw_process(this)
    {
        bind(m_fw_process);
    }

    using base_type::bind;

    // bw transport must come through us.
    tlm::tlm_bw_transport_if<TYPES> *operator -> () { return &m_bw_process; }

    // REGISTER_XXX
    void
    register_nb_transport_fw(MODULE *mod,
            sync_enum_type (MODULE::*cb)(int id, transaction_type &,
                                         phase_type &, sc_core::sc_time &),
            int id)
    {
        elaboration_check("register_nb_transport_fw");
        m_fw_process.set_nb_transport_ptr(mod, cb);
        m_fw_process.set_nb_transport_user_id(id);
    }

    void
    register_b_transport(MODULE *mod,
            void (MODULE::*cb)(int id, transaction_type &,
                               sc_core::sc_time &),
            int id)
    {
        elaboration_check("register_b_transport");
        m_fw_process.set_b_transport_ptr(mod, cb);
        m_fw_process.set_b_transport_user_id(id);
    }

    void
    register_transport_dbg(MODULE *mod,
            unsigned int (MODULE::*cb)(int id, transaction_type &), int id)
    {
        elaboration_check("register_transport_dbg");
        m_fw_process.set_transport_dbg_ptr(mod, cb);
        m_fw_process.set_transport_dbg_user_id(id);
    }

    void
    register_get_direct_mem_ptr(MODULE *mod,
            bool (MODULE::*cb)(int id, transaction_type &, tlm::tlm_dmi &),
            int id)
    {
        elaboration_check("register_get_direct_mem_ptr");
        m_fw_process.set_get_direct_mem_ptr(mod, cb);
        m_fw_process.set_get_dmi_user_id(id);
    }

  protected:
    void
    start_of_simulation()
    {
        base_type::start_of_simulation();
        m_fw_process.start_of_simulation();
    }

  private:
    // Make call on bw path.
    sync_enum_type
    bw_nb_transport(transaction_type &trans, phase_type &phase,
                    sc_core::sc_time &t)
    {
        return base_type::operator -> ()->nb_transport_bw(trans, phase, t);
    }

    void
    bw_invalidate_direct_mem_ptr(sc_dt::uint64 s, sc_dt::uint64 e)
    {
        base_type::operator -> ()->invalidate_direct_mem_ptr(s, e);
    }

    // Helper class to handle bw path calls Needed to detect transaction
    // end when called from b_transport.
    class bw_process : public tlm::tlm_bw_transport_if<TYPES>
    {
      public:
        bw_process(simple_target_socket_tagged_b *p_own) : m_owner(p_own) {}

        sync_enum_type
        nb_transport_bw(transaction_type &trans, phase_type &phase,
                        sc_core::sc_time &t)
        {
            typename std::map<transaction_type *,
                              sc_core::sc_event *>::iterator it =
                                  m_owner->m_pending_trans.find(&trans);

            if (it == m_owner->m_pending_trans.end()) {
                // Not a blocking call, forward.
                return m_owner->bw_nb_transport(trans, phase, t);
            }
            if (phase == tlm::END_REQ) {
                m_owner->m_end_request.notify(sc_core::SC_ZERO_TIME);
                return tlm::TLM_ACCEPTED;
            }
            if (phase == tlm::BEGIN_RESP) {
                if (m_owner->m_current_transaction == &trans) {
                    m_owner->m_end_request.notify(sc_core::SC_ZERO_TIME);
                }
                it->second->notify(t);
                m_owner->m_pending_trans.erase(it);
                return tlm::TLM_COMPLETED;
            }
            m_owner->display_error("invalid phase received");
            return tlm::TLM_COMPLETED;
        }

        void
        invalidate_direct_mem_ptr(sc_dt::uint64 s, sc_dt::uint64 e)
        {
            return m_owner->bw_invalidate_direct_mem_ptr(s, e);
        }

      private:
        simple_target_socket_tagged_b *m_owner;
    };

    class fw_process : public tlm::tlm_fw_transport_if<TYPES>,
                       public tlm::tlm_mm_interface
    {
      public:
        typedef sync_enum_type (MODULE::*NBTransportPtr)(
                int id, transaction_type &, phase_type &,
                sc_core::sc_time &);
        typedef void (MODULE::*BTransportPtr)(
                int id, transaction_type &, sc_core::sc_time &);
        typedef unsigned int (MODULE::*TransportDbgPtr)(
                int id, transaction_type &);
        typedef bool (MODULE::*GetDirectMemPtr)(
                int id, transaction_type &, tlm::tlm_dmi &);

        fw_process(simple_target_socket_tagged_b *p_own) :
            m_owner(p_own), m_mod(0), m_nb_transport_ptr(0),
            m_b_transport_ptr(0), m_transport_dbg_ptr(0),
            m_get_direct_mem_ptr(0), m_nb_transport_user_id(0),
            m_b_transport_user_id(0), m_transport_dbg_user_id(0),
            m_get_dmi_user_id(0),
            m_peq(sc_core::sc_gen_unique_name("m_peq")),
            m_response_in_progress(false)
        {}

        void
        start_of_simulation()
        {
            if (!m_b_transport_ptr && m_nb_transport_ptr) {
                // Only spawn b2nb_thread if needed.
                sc_core::sc_spawn_options opts;
                opts.set_sensitivity(&m_peq.get_event());
                opts.dont_initialize();
                sc_core::sc_spawn(sc_bind(&fw_process::b2nb_thread, this),
                        sc_core::sc_gen_unique_name("b2nb_thread"), &opts);
            }
        }

        void set_nb_transport_user_id(int id) { m_nb_transport_user_id = id; }
        void set_b_transport_user_id(int id) { m_b_transport_user_id = id; }
        void
        set_transport_dbg_user_id(int id)
        {
            m_transport_dbg_user_id = id;
        }
        void set_get_dmi_user_id(int id) { m_get_dmi_user_id = id; }

        void
        set_nb_transport_ptr(MODULE *mod, NBTransportPtr p)
        {
            if (m_nb_transport_ptr) {
                m_owner->display_warning(
                        "non-blocking callback already registered");
                return;
            }
            sc_assert(!m_mod || m_mod == mod);
            m_mod = mod;
            m_nb_transport_ptr = p;
        }

        void
        set_b_transport_ptr(MODULE* mod, BTransportPtr p)
        {
            if (m_b_transport_ptr) {
                m_owner->display_warning(
                        "blocking callback already registered");
                return;
            }
            sc_assert(!m_mod || m_mod == mod);
            m_mod = mod;
            m_b_transport_ptr = p;
        }

        void
        set_transport_dbg_ptr(MODULE *mod, TransportDbgPtr p)
        {
            if (m_transport_dbg_ptr) {
                m_owner->display_warning(
                        "debug callback already registered");
                return;
            }
            sc_assert(!m_mod || m_mod == mod);
            m_mod = mod;
            m_transport_dbg_ptr = p;
        }

        void
        set_get_direct_mem_ptr(MODULE *mod, GetDirectMemPtr p)
        {
            if (m_get_direct_mem_ptr) {
                m_owner->display_warning(
                        "get DMI pointer callback already registered");
            }
            sc_assert(!m_mod || m_mod == mod);
            m_mod = mod;
            m_get_direct_mem_ptr = p;
        }

        // Interface implementation.
        sync_enum_type
        nb_transport_fw(transaction_type &trans, phase_type &phase,
                        sc_core::sc_time &t)
        {
            if (m_nb_transport_ptr) {
                // Forward call.
                sc_assert(m_mod);
                return (m_mod->*m_nb_transport_ptr)(
                        m_nb_transport_user_id, trans, phase, t);
            }

            // nb->b conversion
            if (m_b_transport_ptr) {
                if (phase == tlm::BEGIN_REQ) {

                    // Prepare thread to do blocking call.
                    process_handle_class *ph =
                        m_process_handle.get_handle(&trans);

                    if (!ph) { // Create new dynamic process.
                        ph = new process_handle_class(&trans);
                        m_process_handle.put_handle(ph);

                        sc_core::sc_spawn_options opts;
                        opts.dont_initialize();
                        opts.set_sensitivity(&ph->m_e);

                        sc_core::sc_spawn(
                                sc_bind(&fw_process::nb2b_thread, this, ph),
                                sc_core::sc_gen_unique_name("nb2b_thread"),
                                &opts);
                    }

                    ph->m_e.notify(t);
                    return tlm::TLM_ACCEPTED;
                }
                if (phase == tlm::END_RESP) {
                    m_response_in_progress = false;
                    m_end_response.notify(t);
                    return tlm::TLM_COMPLETED;
                }
                m_owner->display_error("invalid phase");
                return tlm::TLM_COMPLETED;
            }

            m_owner->display_error(
                    "no non-blocking transport callback registered");
            return tlm::TLM_COMPLETED;
        }

        void
        b_transport(transaction_type &trans, sc_core::sc_time &t)
        {
            if (m_b_transport_ptr) {
                // Forward call.
                sc_assert(m_mod);
                (m_mod->*m_b_transport_ptr)(m_b_transport_user_id, trans, t);
                return;
            }

            // b->nb conversion
            if (m_nb_transport_ptr) {
                m_peq.notify(trans, t);
                t = sc_core::SC_ZERO_TIME;

                mm_end_event_ext mm_ext;
                const bool mm_added = !trans.has_mm();

                if (mm_added) {
                    trans.set_mm(this);
                    trans.set_auto_extension(&mm_ext);
                    trans.acquire();
                }

                // Wait until transaction is finished.
                sc_core::sc_event end_event;
                m_owner->m_pending_trans[&trans] = &end_event;
                sc_core::wait(end_event);

                if (mm_added) {
                    // Release will not delete the transaction, it will
                    // notify mm_ext.done.
                    trans.release();
                    if (trans.get_ref_count()) {
                        sc_core::wait(mm_ext.done);
                    }
                    trans.set_mm(0);
                }
                return;
            }

            m_owner->display_error("no transport callback registered");
        }

        unsigned int
        transport_dbg(transaction_type &trans)
        {
            if (m_transport_dbg_ptr) {
                // Forward call.
                sc_assert(m_mod);
                return (m_mod->*m_transport_dbg_ptr)(
                        m_transport_dbg_user_id, trans);
            }
            // No debug support.
            return 0;
        }

        bool
        get_direct_mem_ptr(transaction_type &trans, tlm::tlm_dmi &dmi_data)
        {
            if (m_get_direct_mem_ptr) {
                // Forward call.
                sc_assert(m_mod);
                return (m_mod->*m_get_direct_mem_ptr)(
                        m_get_dmi_user_id, trans, dmi_data);
            }
            // No DMI support.
            dmi_data.allow_read_write();
            dmi_data.set_start_address(0x0);
            dmi_data.set_end_address((sc_dt::uint64)-1);
            return false;
        }

      private:

        // Dynamic process handler for nb2b conversion.
        class process_handle_class
        {
          public:
            explicit process_handle_class(transaction_type *trans) :
                m_trans(trans), m_suspend(false)
            {}

            transaction_type *m_trans;
            sc_core::sc_event  m_e;
            bool m_suspend;
        };

        class process_handle_list
        {
          public:
            process_handle_list() {}

            ~process_handle_list()
            {
                for (typename std::vector<
                        process_handle_class *>::iterator it = v.begin(),
                        end = v.end(); it != end; ++it) {
                    delete *it;
                }
            }

            process_handle_class *
            get_handle(transaction_type *trans)
            {
                typename std::vector<process_handle_class *>::iterator it;

                for (it = v.begin(); it != v.end(); it++) {
                    if ((*it)->m_suspend) {
                        // Found suspended dynamic process, re-use it.
                        (*it)->m_trans = trans; // Replace to new one.
                        (*it)->m_suspend = false;
                        return *it;
                    }
                }
                return NULL; // No suspended process.
            }

            void put_handle(process_handle_class *ph) { v.push_back(ph); }

          private:
            std::vector<process_handle_class *> v;
        };

        process_handle_list m_process_handle;

        void
        nb2b_thread(process_handle_class *h)
        {

            while (1) {
                transaction_type *trans = h->m_trans;
                sc_core::sc_time t = sc_core::SC_ZERO_TIME;

                // Forward call.
                sc_assert(m_mod);
                (m_mod->*m_b_transport_ptr)(
                        m_b_transport_user_id, *trans, t);

                sc_core::wait(t);

                // Return path.
                while (m_response_in_progress) {
                    sc_core::wait(m_end_response);
                }
                t = sc_core::SC_ZERO_TIME;
                phase_type phase = tlm::BEGIN_RESP;
                sync_enum_type sync =
                    m_owner->bw_nb_transport(*trans, phase, t);
                if (!(sync == tlm::TLM_COMPLETED ||
                            (sync == tlm::TLM_UPDATED &&
                             phase == tlm::END_RESP))) {
                    m_response_in_progress = true;
                }

                // Suspend until next transaction.
                h->m_suspend = true;
                sc_core::wait();
            }
        }

        void
        b2nb_thread()
        {
            while (true) {
                transaction_type *trans;
                while ((trans = m_peq.get_next_transaction()) != 0) {
                    sc_assert(m_mod);
                    sc_assert(m_nb_transport_ptr);
                    phase_type phase = tlm::BEGIN_REQ;
                    sc_core::sc_time t = sc_core::SC_ZERO_TIME;

                    switch ((m_mod->*m_nb_transport_ptr)(
                                m_nb_transport_user_id, *trans, phase, t)) {
                      case tlm::TLM_COMPLETED:
                        {
                            // Notify transaction is finished.
                            typename std::map<transaction_type *,
                                     sc_core::sc_event *>::iterator it =
                                         m_owner->m_pending_trans.find(trans);
                            sc_assert(it != m_owner->m_pending_trans.end());
                            it->second->notify(t);
                            m_owner->m_pending_trans.erase(it);
                            break;
                        }

                      case tlm::TLM_ACCEPTED:
                      case tlm::TLM_UPDATED:
                        switch (phase) {
                          case tlm::BEGIN_REQ:
                            m_owner->m_current_transaction = trans;
                            sc_core::wait(m_owner->m_end_request);
                            m_owner->m_current_transaction = 0;
                            break;

                          case tlm::END_REQ:
                            sc_core::wait(t);
                            break;

                          case tlm::BEGIN_RESP:
                            {
                                phase = tlm::END_RESP;
                                // This line is a bug fix added in TLM-2.0.2.
                                sc_core::wait(t);
                                t = sc_core::SC_ZERO_TIME;
                                (m_mod->*m_nb_transport_ptr)(
                                        m_nb_transport_user_id,
                                        *trans, phase, t);

                                // Notify transaction is finished.
                                typename std::map<transaction_type *,
                                         sc_core::sc_event *>::iterator it =
                                             m_owner->m_pending_trans.find(
                                                     trans);
                                sc_assert(it !=
                                        m_owner->m_pending_trans.end());
                                it->second->notify(t);
                                m_owner->m_pending_trans.erase(it);
                                break;
                            }

                          default:
                            m_owner->display_error("invalid phase received");
                        };
                        break;

                      default:
                        m_owner->display_error("invalid sync value received");
                    }
                }
                sc_core::wait();
            }
        }

        void
        free(tlm::tlm_generic_payload *trans)
        {
            mm_end_event_ext *ext =
                trans->template get_extension<mm_end_event_ext>();
            sc_assert(ext);
            // Notify event first before freeing extensions (reset).
            ext->done.notify();
            trans->reset();
        }

      private:
        struct mm_end_event_ext : public tlm::tlm_extension<mm_end_event_ext>
        {
            tlm::tlm_extension_base *clone() const { return NULL; }
            void free() {}
            void copy_from(tlm::tlm_extension_base const &) {}
            sc_core::sc_event done;
        };

      private:
        simple_target_socket_tagged_b *m_owner;
        MODULE *m_mod;
        NBTransportPtr m_nb_transport_ptr;
        BTransportPtr m_b_transport_ptr;
        TransportDbgPtr m_transport_dbg_ptr;
        GetDirectMemPtr m_get_direct_mem_ptr;
        int m_nb_transport_user_id;
        int m_b_transport_user_id;
        int m_transport_dbg_user_id;
        int m_get_dmi_user_id;
        peq_with_get<transaction_type> m_peq;
        bool m_response_in_progress;
        sc_core::sc_event m_end_response;
    };

  private:
    const sc_core::sc_object *get_socket() const { return this; }

  private:
    fw_process m_fw_process;
    bw_process m_bw_process;
    std::map<transaction_type *, sc_core::sc_event *> m_pending_trans;
    sc_core::sc_event m_end_request;
    transaction_type* m_current_transaction;
};

template <typename MODULE, unsigned int BUSWIDTH=32,
          typename TYPES=tlm::tlm_base_protocol_types>
class simple_target_socket_tagged :
    public simple_target_socket_tagged_b<MODULE, BUSWIDTH, TYPES>
{
    typedef simple_target_socket_tagged_b<MODULE, BUSWIDTH, TYPES> socket_b;
  public:
    simple_target_socket_tagged() : socket_b() {}
    explicit simple_target_socket_tagged(const char *name) : socket_b(name) {}
};

template <typename MODULE, unsigned int BUSWIDTH=32,
          typename TYPES=tlm::tlm_base_protocol_types>
class simple_target_socket_tagged_optional :
    public simple_target_socket_tagged_b<MODULE, BUSWIDTH, TYPES,
                                         sc_core::SC_ZERO_OR_MORE_BOUND>
{
    typedef simple_target_socket_tagged_b<
        MODULE, BUSWIDTH, TYPES, sc_core::SC_ZERO_OR_MORE_BOUND> socket_b;
  public:
    simple_target_socket_tagged_optional() : socket_b() {}
    explicit simple_target_socket_tagged_optional(const char *name) :
        socket_b(name)
    {}
};

} // namespace tlm_utils

#endif /* __SYSTEMC_EXT_TLM_UTILS_SIMPLE_TARGET_SOCKET_H__ */

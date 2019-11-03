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

#ifndef __SYSTEMC_EXT_TLM_UTILS_PEQ_WITH_CB_AND_PHASE_H__
#define __SYSTEMC_EXT_TLM_UTILS_PEQ_WITH_CB_AND_PHASE_H__

#include <vector>

#include "../core/sc_main.hh"
#include "../core/sc_object.hh"
#include "../core/sc_spawn.hh"
#include "../core/sc_time.hh"
#include "../dt/int/sc_nbdefs.hh"
#include "../tlm_core/2/interfaces/fw_bw_ifs.hh"

namespace tlm_utils
{

template <typename PAYLOAD>
class time_ordered_list
{
  public:
    struct element
    {
        struct element *next;
        PAYLOAD p;
        sc_core::sc_time t;
        sc_dt::uint64 d;
        element(PAYLOAD &p, sc_core::sc_time t, sc_dt::uint64 d) :
            p(p), t(t), d(d)
        {}
        element() {}
    };

    element *nill;
    element *empties;
    element *list;
    unsigned int size;

    time_ordered_list() : nill(new element()), empties(NULL),
                          list(nill), size(0)
    {}

    ~time_ordered_list()
    {
        reset();
        while (empties) {
            struct element *e = empties->next;
            delete empties;
            empties = e;
        }
        delete nill;
    }

    void
    reset()
    {
        while (size) {
            delete_top();
        }
    }

    void
    insert(const PAYLOAD &p, sc_core::sc_time t)
    {
        if (!empties) {
            empties = new struct element();
            empties->next=NULL;
        }

        struct element *e = empties;
        empties = empties->next;
        e->p = p;
        e->t = t;
        e->d = sc_core::sc_delta_count();

        struct element *ancestor = nill;
        struct element *iterator = list;
        while (iterator != nill && iterator->t <= t) {
            ancestor = iterator;
            iterator = iterator->next;
        }
        if (ancestor == nill) {
            e->next = list;
            list = e;
        } else {
            e->next = iterator;
            ancestor->next = e;
        }
        size++;
    }

    void
    delete_top()
    {
        if (list != nill) {
            struct element *e = list;
            list = list->next;
            e->next = empties;
            empties = e;
            size--;
        }
    }

    unsigned int get_size() { return size; }
    PAYLOAD &top() { return list->p; }
    sc_core::sc_time top_time() { return list->t; }
    sc_dt::uint64 &top_delta() { return list->d; }
    sc_core::sc_time next_time() { return list->next->t; }
};

//---------------------------------------------------------------------------
/**
 * An event queue that can contain any number of pending
 * notifications. Each notification have an associate payload.
 */
//---------------------------------------------------------------------------
template<typename OWNER, typename TYPES=tlm::tlm_base_protocol_types>
class peq_with_cb_and_phase : public sc_core::sc_object
{
    typedef typename TYPES::tlm_payload_type tlm_payload_type;
    typedef typename TYPES::tlm_phase_type tlm_phase_type;
    typedef std::pair<tlm_payload_type *, tlm_phase_type> PAYLOAD;
    typedef void (OWNER::*cb)(tlm_payload_type &, const tlm_phase_type &);

    class delta_list
    {
      public:
        delta_list()
        {
            reset();
            entries.resize(100);
        }

        inline void
        insert(const PAYLOAD &p)
        {
            if (size==entries.size()) {
                entries.resize(entries.size() * 2);
            }
            entries[size++] = p;
        }

        inline PAYLOAD &get() { return entries[out++]; }
        inline bool next() { return out < size; }
        inline void
        reset()
        {
            size=0;
            out=0;
        }

      public:
        unsigned int size;

      private:
        std::vector<PAYLOAD> entries;
        unsigned int out;
    };

  public:
    peq_with_cb_and_phase(OWNER *_owner, cb _cb) :
        sc_core::sc_object(sc_core::sc_gen_unique_name(
                    "peq_with_cb_and_phase")),
        m_owner(_owner), m_cb(_cb)
    {
        sc_core::sc_spawn_options opts;
        opts.spawn_method();
        opts.set_sensitivity(&m_e);
        opts.dont_initialize();
        sc_core::sc_spawn(sc_bind(&peq_with_cb_and_phase::fec, this),
                          sc_core::sc_gen_unique_name("fec"), &opts);
    }

    peq_with_cb_and_phase(const char *_name, OWNER *_owner, cb _cb) :
        sc_core::sc_object(_name), m_owner(_owner), m_cb(_cb)
    {
        sc_core::sc_spawn_options opts;
        opts.spawn_method();
        opts.set_sensitivity(&m_e);
        opts.dont_initialize();
        sc_core::sc_spawn(sc_bind(&peq_with_cb_and_phase::fec, this),
                          sc_core::sc_gen_unique_name("fec"), &opts);
    }

    ~peq_with_cb_and_phase() {}

    void
    notify(tlm_payload_type &t, const tlm_phase_type &p,
            const sc_core::sc_time &when)
    {
        if (when == sc_core::SC_ZERO_TIME) {
            if (sc_core::sc_delta_count() & (sc_dt::uint64)0x1) {
                // Uneven delta cycle so delta delay is for even cycle.
                m_even_delta.insert(PAYLOAD(&t,p));
            } else {
                // Even delta cycle so delta delay is for uneven delta.
                m_uneven_delta.insert(PAYLOAD(&t, p));
            }
            m_e.notify(sc_core::SC_ZERO_TIME);
        } else {
            m_ppq.insert(PAYLOAD(&t, p), when + sc_core::sc_time_stamp());
            // Note, this will only overwrite the "newest" event.
            m_e.notify(when);
        }
    }

    void
    notify(tlm_payload_type &t, const tlm_phase_type &p)
    {
        m_immediate_yield.insert(PAYLOAD(&t, p));
        m_e.notify(); // Immediate notification.
    }

    // Cancel all events from the event queue.
    void
    cancel_all()
    {
        m_ppq.reset();
        m_uneven_delta.reset();
        m_even_delta.reset();
        m_immediate_yield.reset();
        m_e.cancel();
    }

  private:
    void
    fec()
    {
        // Immediate yield notifications.
        while (m_immediate_yield.next()) {
            PAYLOAD &tmp = m_immediate_yield.get();
            (m_owner->*m_cb)(*tmp.first, tmp.second);
        }
        m_immediate_yield.reset();

        // Delta notifications.
        if (sc_core::sc_delta_count() & (sc_dt::uint64)0x1) {
            // Uneven delta so put out all payloads for uneven delta.
            while (m_uneven_delta.next()) {
                PAYLOAD &tmp = m_uneven_delta.get();
                (m_owner->*m_cb)(*tmp.first, tmp.second);
            }
            m_uneven_delta.reset();
            if (m_even_delta.size)
                m_e.notify(sc_core::SC_ZERO_TIME);
        } else {
            while (m_even_delta.next()) {
                PAYLOAD &tmp = m_even_delta.get();
                (m_owner->*m_cb)(*tmp.first, tmp.second);
            }
            m_even_delta.reset();
            if (m_uneven_delta.size)
                m_e.notify(sc_core::SC_ZERO_TIME);
        }
        if (!m_ppq.get_size())
            return; // There were only delta notification.

        // Timed notifications.
        const sc_core::sc_time now = sc_core::sc_time_stamp();
        sc_core::sc_time top = m_ppq.top_time();

        while (m_ppq.get_size() && top == now) {
            // Push all active ones into target.
            PAYLOAD &tmp = m_ppq.top();
            (m_owner->*m_cb)(*tmp.first, tmp.second);
            m_ppq.delete_top();
            top = m_ppq.top_time();
        }
        if (m_ppq.get_size()) {
            m_e.notify(top - now);
        }
    }

    OWNER *m_owner;
    cb m_cb;

    time_ordered_list<PAYLOAD> m_ppq;
    delta_list m_uneven_delta;
    delta_list m_even_delta;
    delta_list m_immediate_yield;

    sc_core::sc_event m_e; // Default event.
};

} // namespace tlm_utils

#endif /* __SYSTEMC_EXT_TLM_UTILS_PEQ_WITH_CB_AND_PHASE_H__ */

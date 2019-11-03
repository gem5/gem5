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

// 12-Jan-2009  John Aynsley  Bug fix. sc_time argument to notify should be const
// 20-Mar-2009  John Aynsley  Add cancel_all() method


#ifndef __SYSTEMC_EXT_TLM_UTILS_PEQ_WITH_GET_H__
#define __SYSTEMC_EXT_TLM_UTILS_PEQ_WITH_GET_H__

#include <map>

#include "../core/sc_event.hh"
#include "../core/sc_main.hh"
#include "../core/sc_object.hh"
#include "../core/sc_time.hh"

namespace tlm_utils
{

template <class PAYLOAD>
class peq_with_get : public sc_core::sc_object
{
  public:
    typedef PAYLOAD transaction_type;
    typedef std::pair<const sc_core::sc_time, transaction_type *> pair_type;

  public:
    peq_with_get(const char *name) : sc_core::sc_object(name) {}

    void
    notify(transaction_type &trans, const sc_core::sc_time &t)
    {
        m_scheduled_events.insert(pair_type(t + sc_core::sc_time_stamp(),
                    &trans));
        m_event.notify(t);
    }

    void
    notify(transaction_type &trans)
    {
        m_scheduled_events.insert(pair_type(sc_core::sc_time_stamp(), &trans));
        m_event.notify(); // Immediate notification.
    }

    // Needs to be called until it returns NULL
    transaction_type *
    get_next_transaction()
    {
        if (m_scheduled_events.empty()) {
            return nullptr;
        }

        sc_core::sc_time now = sc_core::sc_time_stamp();
        if (m_scheduled_events.begin()->first <= now) {
            transaction_type *trans = m_scheduled_events.begin()->second;
            m_scheduled_events.erase(m_scheduled_events.begin());
            return trans;
        }

        m_event.notify(m_scheduled_events.begin()->first - now);

        return nullptr;
    }

    sc_core::sc_event &get_event() { return m_event; }

    // Cancel all events from the event queue.
    void
    cancel_all()
    {
        m_scheduled_events.clear();
        m_event.cancel();
    }

  private:
    std::multimap<const sc_core::sc_time, transaction_type *>
        m_scheduled_events;
    sc_core::sc_event m_event;
};

} // namespace tlm_utils

#endif /* __SYSTEMC_EXT_TLM_UTILS_PEQ_WITH_GET_H__ */

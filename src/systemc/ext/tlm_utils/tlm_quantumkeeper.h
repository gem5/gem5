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

#ifndef __SYSTEMC_EXT_TLM_UTILS_TLM_QUANTUMKEEPER_H__
#define __SYSTEMC_EXT_TLM_UTILS_TLM_QUANTUMKEEPER_H__

#include "../core/sc_time.hh"

namespace tlm_utils
{

// tlm_quantumkeeper class
//
// The tlm_quantumkeeper class is used to keep track of the local time in
// an initiator (how much it has run ahead of the SystemC time), to
// synchronize with SystemC time etc.
class tlm_quantumkeeper
{
  public:
    //
    // Static setters/getters for the global quantum value.
    //
    // The global quantum is the maximum time an initiator can run ahead of
    // SystemC time. All initiators will synchronize on timing points that are
    // multiples of the global quantum value.
    //
    static void
    set_global_quantum(const sc_core::sc_time &t)
    {
        tlm::tlm_global_quantum::instance().set(t);
    }

    static const sc_core::sc_time &
    get_global_quantum()
    {
        return tlm::tlm_global_quantum::instance().get();
    }

  public:
    tlm_quantumkeeper() : m_next_sync_point(sc_core::SC_ZERO_TIME),
                          m_local_time(sc_core::SC_ZERO_TIME)
    {}

    virtual ~tlm_quantumkeeper() {}

    // Increment the local time (the time the initiator is ahead of the
    // systemC time) After incrementing the local time an initiator should
    // check (with the need_sync method) if a sync is required.
    virtual void inc(const sc_core::sc_time &t) { m_local_time += t; }

    // Sets the local time (the time the initiator is ahead of the
    // systemC time) After changing the local time an initiator should
    // check (with the need_sync method) if a sync is required.
    virtual void set(const sc_core::sc_time &t) { m_local_time = t; }

    // Checks if a sync to systemC is required for this initiator. This will
    // be the case if the local time becomes greater than the local (current)
    // quantum value for this initiator.
    virtual bool
    need_sync() const
    {
        return sc_core::sc_time_stamp() + m_local_time >= m_next_sync_point;
    }

    // Synchronize to systemC. This call will do a wait for the time the
    // initiator was running ahead of systemC time and reset the
    // tlm_quantumkeeper.
    virtual void
    sync()
    {
        sc_core::wait(m_local_time);
        reset();
    }

    // Non-virtual convenience method to set the local time and sync only if
    // needed
    void
    set_and_sync(const sc_core::sc_time &t)
    {
        set(t);
        if (need_sync())
            sync();
    }

    // Resets the local time to SC_ZERO_TIME and computes the value of the
    // next local quantum. This method should be called by an initiator after
    // a wait because of a synchronization request by a target (TLM_ACCEPTED,
    // or TLM_UPDATED).
    virtual void
    reset()
    {
        m_local_time = sc_core::SC_ZERO_TIME;
        m_next_sync_point = sc_core::sc_time_stamp() + compute_local_quantum();
    }

    // Helper function to get the current systemC time, taken the local time
    // into account. The current systemC time is calculated as the time
    // returned by sc_time_stamp incremeneted with the time the initiator is
    // running ahead.
    virtual sc_core::sc_time
    get_current_time() const
    {
        return sc_core::sc_time_stamp() + m_local_time;
    }

    // Helper functions to get the time the initiator is running ahead of
    // systenC (local time). This time should be passed to a target in the
    // nb_transport call
    virtual sc_core::sc_time
    get_local_time() const
    {
        return m_local_time;
    }

  protected:
    // Calculate the next local quantum for this initiator.
    //
    // The method can be overloaded in a derived object if an initiator wants
    // to use another local quantum. This derived object should also take the
    // global quantum into account. It's local quantum should not be set to a
    // value that is larger than the quantum returned by the
    // compute_local_quantum of the tlm_global_quantum singleton.
    virtual sc_core::sc_time
    compute_local_quantum()
    {
        return tlm::tlm_global_quantum::instance().compute_local_quantum();
    }

  protected:
    sc_core::sc_time m_next_sync_point;
    sc_core::sc_time m_local_time;
};

} // namespace tlm_utils

#endif /* __SYSTEMC_EXT_TLM_UTILS_TLM_QUANTUMKEEPER_H__ */

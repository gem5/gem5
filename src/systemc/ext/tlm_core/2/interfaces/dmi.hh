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

#ifndef __SYSTEMC_EXT_TLM_CORE_2_INTERFACES_DMI_HH__
#define __SYSTEMC_EXT_TLM_CORE_2_INTERFACES_DMI_HH__

#include "../../../core/sc_time.hh"
#include "../../../dt/int/sc_nbdefs.hh"

namespace tlm
{

class tlm_dmi
{
  public:
    // Enum for indicating the access granted to the initiator.
    // The initiator uses gp.m_command to indicate it intention (read/write)
    //  The target is allowed to promote DMI_ACCESS_READ or DMI_ACCESS_WRITE
    //  requests to dmi_access_read_write.

    enum dmi_access_e {
        DMI_ACCESS_NONE = 0x00, // no access
        DMI_ACCESS_READ = 0x01, // read access
        DMI_ACCESS_WRITE = 0x02, // write access
        DMI_ACCESS_READ_WRITE = DMI_ACCESS_READ | DMI_ACCESS_WRITE
            // read/write access
    };

    tlm_dmi() { init(); }

    void
    init()
    {
        m_dmi_ptr = nullptr;
        m_dmi_start_address = 0x0;
        m_dmi_end_address = (sc_dt::uint64)(-1);
        m_dmi_access = DMI_ACCESS_NONE;
        m_dmi_read_latency = sc_core::SC_ZERO_TIME;
        m_dmi_write_latency = sc_core::SC_ZERO_TIME;
    }

    unsigned char *get_dmi_ptr() const { return m_dmi_ptr; }
    sc_dt::uint64 get_start_address() const { return m_dmi_start_address; }
    sc_dt::uint64 get_end_address() const { return m_dmi_end_address; }
    sc_core::sc_time get_read_latency() const { return m_dmi_read_latency; }
    sc_core::sc_time get_write_latency() const { return m_dmi_write_latency; }
    dmi_access_e get_granted_access() const { return m_dmi_access; }
    bool is_none_allowed() const { return m_dmi_access == DMI_ACCESS_NONE; }
    bool
    is_read_allowed() const
    {
        return (m_dmi_access & DMI_ACCESS_READ) == DMI_ACCESS_READ;
    }
    bool
    is_write_allowed() const
    {
        return (m_dmi_access & DMI_ACCESS_WRITE) == DMI_ACCESS_WRITE;
    }
    bool
    is_read_write_allowed() const
    {
        return (m_dmi_access & DMI_ACCESS_READ_WRITE) == DMI_ACCESS_READ_WRITE;
    }

    void set_dmi_ptr(unsigned char *p) { m_dmi_ptr = p; }
    void set_start_address(sc_dt::uint64 addr) { m_dmi_start_address = addr; }
    void set_end_address(sc_dt::uint64 addr) { m_dmi_end_address = addr; }
    void set_read_latency(sc_core::sc_time t) { m_dmi_read_latency = t; }
    void set_write_latency(sc_core::sc_time t) { m_dmi_write_latency = t; }
    void set_granted_access(dmi_access_e a) { m_dmi_access = a; }
    void allow_none() { m_dmi_access = DMI_ACCESS_NONE; }
    void allow_read() { m_dmi_access = DMI_ACCESS_READ; }
    void allow_write() { m_dmi_access = DMI_ACCESS_WRITE; }
    void allow_read_write() { m_dmi_access = DMI_ACCESS_READ_WRITE; }

  private:
    // If the forward call is successful, the target returns the dmi_ptr,
    // which must point to the data element corresponding to the
    // dmi_start_address. The data is organized as a byte array with the
    // endianness of the target (endianness member of the tlm_dmi struct).

    unsigned char *m_dmi_ptr;

    // The absolute start and end addresses of the DMI region. If the decoder
    // logic in the interconnect changes the address field e.g. by masking, the
    // interconnect is responsible to transform the relative address back to an
    // absolute address again.

    sc_dt::uint64 m_dmi_start_address;
    sc_dt::uint64 m_dmi_end_address;

    // Granted access

    dmi_access_e m_dmi_access;

    // These members define the latency of read/write transactions. The
    // initiator must initialize these members to zero before requesting a
    // dmi pointer, because both the interconnect as well as the target can
    // add to the total transaction latency.
    // Depending on the 'type' attribute only one, or both of these attributes
    // will be valid.

    sc_core::sc_time m_dmi_read_latency;
    sc_core::sc_time m_dmi_write_latency;
};

} // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_2_INTERFACES_DMI_HH__ */

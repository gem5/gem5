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

#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_time.hh"
#include "systemc/ext/tlm_core/2/quantum/global_quantum.hh"

namespace tlm
{

tlm_global_quantum::tlm_global_quantum()
    : m_global_quantum(sc_core::SC_ZERO_TIME)
{}

tlm_global_quantum &
tlm_global_quantum::instance()
{
    static tlm_global_quantum instance_;
    return instance_;
}

sc_core::sc_time
tlm_global_quantum::compute_local_quantum()
{
    if (m_global_quantum != sc_core::SC_ZERO_TIME) {
        const sc_core::sc_time current = sc_core::sc_time_stamp();
        const sc_core::sc_time g_quant = m_global_quantum;
        return sc_core::sc_time::from_value(
            g_quant.value() - (current.value() % g_quant.value()));
    } else {
        return sc_core::SC_ZERO_TIME;
    }
}

} // namespace tlm

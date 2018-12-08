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

#ifndef TLM_CORE_TLM2_TLM_GLOBAL_QUANTUM_H_INCLUDED_
#define TLM_CORE_TLM2_TLM_GLOBAL_QUANTUM_H_INCLUDED_

#include "sysc/kernel/sc_time.h"

namespace tlm {

//
// tlm_global_quantum class
//
// The global quantum is the maximum time an initiator can run ahead of
// SystemC time. All initiators should synchronize on timingpoints that
// are multiples of the global quantum value.
//
// sc_set_time_resolution can only be called before the first
// sc_time object is created. This means that after setting the
// global quantum it will not be possible to call sc_set_time_resolution.
// If sc_set_time_resolution must be called this must be done before
// the global quantum is set.
//

class SC_API tlm_global_quantum
{
public:
  //
  // Returns a reference to the tlm_global_quantum singleton
  //
  static tlm_global_quantum& instance();

public:

  //
  // Setter/getter for the global quantum
  //
  void set(const sc_core::sc_time& t)
  {
    m_global_quantum = t;
  }

  const sc_core::sc_time& get() const
  {
    return m_global_quantum;
  }

  //
  // This function will calculate the maximum value for the next local
  // quantum for an initiator. All initiators should synchronize on
  // integer multiples of the global quantum value. The value for the
  // local quantum of an initiator can be smaller, but should never be
  // greater than the value returned by this method.
  //
  sc_core::sc_time compute_local_quantum();

protected:
  tlm_global_quantum();

protected:
  sc_core::sc_time m_global_quantum;
};

} // namespace tlm

#endif // TLM_CORE_TLM2_TLM_GLOBAL_QUANTUM_H_INCLUDED_

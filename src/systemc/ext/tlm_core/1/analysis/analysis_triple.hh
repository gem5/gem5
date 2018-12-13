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

#ifndef __SYSTEMC_EXT_TLM_CORE_1_ANALYSIS_ANALYSIS_TRIPLE_HH__
#define __SYSTEMC_EXT_TLM_CORE_1_ANALYSIS_ANALYSIS_TRIPLE_HH__

namespace tlm
{

template <typename T>
struct tlm_analysis_triple
{
    sc_core::sc_time start_time;
    T transaction;
    sc_core::sc_time end_time;

    tlm_analysis_triple() {}

    tlm_analysis_triple(const tlm_analysis_triple &triple)
    {
        start_time = triple.start_time;
        transaction = triple.transaction;
        end_time = triple.end_time;
    }

    tlm_analysis_triple(const T &t)
    {
        transaction = t;
    }

    operator T() { return transaction; }
    operator const T &() const { return transaction; }
};

} // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_1_ANALYSIS_ANALYSIS_TRIPLE_HH__ */

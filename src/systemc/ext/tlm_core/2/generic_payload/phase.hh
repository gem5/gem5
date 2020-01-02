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

#ifndef __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_PHASE_HH__
#define __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_PHASE_HH__

#include <iostream>
#include <typeinfo>
#include <vector>

#define SC_CONCAT_HELPER_(a, b) SC_CONCAT_HELPER_DEFERRED_(a, b)
#define SC_CONCAT_HELPER_DEFERRED_(a, b) SC_CONCAT_HELPER_MORE_DEFERRED_(a, b)
#define SC_CONCAT_HELPER_MORE_DEFERRED_(a, b) a ## b

#define SC_STRINGIFY_HELPER_(a) SC_STRINGIFY_HELPER_DEFERRED_(a)
#define SC_STRINGIFY_HELPER_DEFERRED_(a) SC_STRINGIFY_HELPER_MORE_DEFERRED_(a)
#define SC_STRINGIFY_HELPER_MORE_DEFERRED_(a) #a

namespace tlm
{

enum tlm_phase_enum
{
    UNINITIALIZED_PHASE = 0,
    BEGIN_REQ = 1,
    END_REQ,
    BEGIN_RESP,
    END_RESP
};

class tlm_phase
{
  public:
    tlm_phase();
    tlm_phase(unsigned int id);

    tlm_phase(tlm_phase_enum standard);
    tlm_phase &operator = (tlm_phase_enum standard);

    operator unsigned int() const { return m_id; }
    const char *get_name() const;

  protected:
    // Register extended phase.
    tlm_phase(const std::type_info &type, const char *name);

  private:
    unsigned int m_id;
};

inline tlm_phase::tlm_phase() : m_id(UNINITIALIZED_PHASE) {}

inline tlm_phase::tlm_phase(tlm_phase_enum standard) : m_id(standard) {}

inline tlm_phase &
tlm_phase::operator = (tlm_phase_enum standard)
{
    m_id = standard;
    return *this;
}

inline std::ostream &
operator << (std::ostream &s, const tlm_phase &p)
{
    s << p.get_name();
    return s;
}

#define TLM_DECLARE_EXTENDED_PHASE(name_arg) \
static class SC_CONCAT_HELPER_(tlm_phase_, name_arg) : \
    public ::tlm::tlm_phase \
{ \
    typedef SC_CONCAT_HELPER_(tlm_phase_, name_arg) this_type; \
  public: \
    SC_CONCAT_HELPER_(tlm_phase_, name_arg)() : \
        /* register extended phase */ \
        ::tlm::tlm_phase(typeid(*this), SC_STRINGIFY_HELPER_(name_arg)) \
    {} \
    \
    static const this_type &get_phase() \
        /* needed only for IEEE 1666-2011 */ \
    { \
        static this_type this_; \
        return this_; \
    } \
} const name_arg

// for backwards-compatibility
#define DECLARE_EXTENDED_PHASE(NameArg) TLM_DECLARE_EXTENDED_PHASE(NameArg)

} // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_PHASE_HH__ */

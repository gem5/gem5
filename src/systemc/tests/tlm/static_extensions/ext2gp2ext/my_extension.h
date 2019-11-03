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

#ifndef __MY_EXTENSION_H__
#define __MY_EXTENSION_H__

#include "tlm.h"
#include <cassert>

class my_extension :
   public tlm::tlm_extension<my_extension>
{
public:
    my_extension()
        : m_data(0)
    {}
    tlm_extension_base* clone() const
    {
        return new my_extension(*this);
    }
    void free()
    {
        delete this;
    }
    void copy_from(tlm_extension_base const & e)
    {
        sc_assert(typeid(this) == typeid(e));
        m_data = static_cast<my_extension const &>(e).m_data;
    }
    
    int m_data;
};

struct my_extended_payload_types
{
  typedef tlm::tlm_base_protocol_types::tlm_payload_type tlm_payload_type;
  typedef tlm::tlm_base_protocol_types::tlm_phase_type tlm_phase_type;
};

#endif

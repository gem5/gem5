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

#ifndef __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOADS_ARRAY_HH__
#define __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOADS_ARRAY_HH__

#include <vector>

namespace tlm
{

// This implements a lean and fast array class that supports array expansion on
// request. The class is primarily used in the tlm_generic_payload class for
// storing the pointers to the extensions.
//
// Individual array elements can be accessed through the [] operators, and the
// array length is returned by the size() method.
//
// The size can be dynamically expanded using the expand(uint) method. There
// is no shrinking mechanism implemented, because the extension mechanism
// does not require this feature. Bear in mind that calling the expand method
// may invalidate all direct pointers into the array.


// The tlm_array shall always be used with T=tlm_extension_base*.
template <typename T>
class tlm_array : private std::vector<T>
{
  private:
    typedef std::vector<T> base_type;
    typedef typename base_type::size_type size_type;

  public:
    tlm_array(size_type size=0) : base_type(size), m_entries() {}

    // Operators for dereferencing:
    using base_type::operator [];

    // Array size:
    using base_type::size;

    // Expand the array if needed:
    void
    expand(size_type new_size)
    {
        if (new_size > size())
            base_type::resize(new_size);
    }

    static const char *const kind_string;
    const char *kind() const { return kind_string; }

    // This function shall get a pointer to an array slot
    // it stores this slot in a cache of active slots
    void insert_in_cache(T *p) { m_entries.push_back(p - &(*this)[0]); }

    // This functions clears all active slots of the array.
    void
    free_entire_cache()
    {
        while (m_entries.size()) {
            // We make sure no one cleared the slot manually.
            if ((*this)[m_entries.back()]) {
                // ...and then we call free on the content of the slot
                (*this)[m_entries.back()]->free();
            }
            // Afterwards we set the slot to NULL
            (*this)[m_entries.back()] = nullptr;
            m_entries.pop_back();
        }
    }

  protected:
    std::vector<size_type> m_entries;
};

template <typename T>
const char *const tlm_array<T>::kind_string = "tlm_array";

} // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOADS_ARRAY_HH__ */

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

#include <cstring>  // std::memcpy et.al.
#include <map>
#include <typeindex>

#include "systemc/ext/tlm_core/2/generic_payload/gp.hh"

namespace tlm
{

template class tlm_array<tlm_extension_base *>;

//---------------------------------------------------------------------------
// Classes for the extension mechanism
//---------------------------------------------------------------------------

namespace
{

class tlm_extension_registry
{
    typedef unsigned int key_type;
    typedef std::map<std::type_index, key_type> type_map;
  public:
    static tlm_extension_registry &
    instance()
    {
        if (!instance_) {
            // Don't cleanup registry.
            instance_ = new tlm_extension_registry();
        }
        return *instance_;
    }

    unsigned int
    register_extension(std::type_index type)
    {
        type_map::const_iterator it = ids_.find(type);

        if (it == ids_.end()) {
            // New extension - generate/store ID.
            type_map::value_type v(type, static_cast<key_type>(ids_.size()));
            ids_.insert(v);
            return v.second;
        }
        return it->second;
    }

    static unsigned int
    max_num_extensions()
    {
        return (instance_) ? instance().ids_.size() : 0;
    }

  private:
    static tlm_extension_registry *instance_;
    type_map ids_;
    tlm_extension_registry() {}

};

tlm_extension_registry *tlm_extension_registry::instance_ = NULL;

} // anonymous namespace

unsigned int
max_num_extensions()
{
    return tlm_extension_registry::max_num_extensions();
}

unsigned int
tlm_extension_base::register_extension(const std::type_info &type)
{
    return tlm_extension_registry::instance().register_extension(type);
}

//---------------------------------------------------------------------------
// The generic payload class:
//---------------------------------------------------------------------------

tlm_generic_payload::tlm_generic_payload() : m_address(0),
    m_command(TLM_IGNORE_COMMAND), m_data(0), m_length(0),
    m_response_status(TLM_INCOMPLETE_RESPONSE), m_dmi(false), m_byte_enable(0),
    m_byte_enable_length(0), m_streaming_width(0),
    m_gp_option(TLM_MIN_PAYLOAD), m_extensions(max_num_extensions()), m_mm(0),
    m_ref_count(0)
{}

tlm_generic_payload::tlm_generic_payload(tlm_mm_interface *mm): m_address(0),
    m_command(TLM_IGNORE_COMMAND), m_data(0), m_length(0),
    m_response_status(TLM_INCOMPLETE_RESPONSE), m_dmi(false), m_byte_enable(0),
    m_byte_enable_length(0), m_streaming_width(0),
    m_gp_option(TLM_MIN_PAYLOAD), m_extensions(max_num_extensions()), m_mm(mm),
    m_ref_count(0)
{}

void
tlm_generic_payload::reset()
{
    // Should the other members be reset too?
    m_gp_option = TLM_MIN_PAYLOAD;
    m_extensions.free_entire_cache();
};

// Non-virtual deep-copying of the object.
void
tlm_generic_payload::deep_copy_from(const tlm_generic_payload &other)
{
    m_command = other.get_command();
    m_address = other.get_address();
    m_length = other.get_data_length();
    m_response_status = other.get_response_status();
    m_byte_enable_length = other.get_byte_enable_length();
    m_streaming_width = other.get_streaming_width();
    m_gp_option = other.get_gp_option();
    m_dmi = other.is_dmi_allowed();

    // Deep copy data.
    // There must be enough space in the target transaction!
    if (m_data && other.m_data) {
        std::memcpy(m_data, other.m_data, m_length);
    }
    // Deep copy byte enables.
    // There must be enough space in the target transaction!
    if (m_byte_enable && other.m_byte_enable) {
        std::memcpy(m_byte_enable, other.m_byte_enable, m_byte_enable_length);
    }
    // Deep copy extensions (sticky and non-sticky).
    if (m_extensions.size() < other.m_extensions.size()) {
        m_extensions.expand(other.m_extensions.size());
    }
    for (unsigned int i = 0; i < other.m_extensions.size(); i++) {
        if (other.m_extensions[i]) {
            // Original has extension i.
            if (!m_extensions[i]) {
                // We don't: clone.
                tlm_extension_base *ext = other.m_extensions[i]->clone();
                if (ext) { // Extension may not be clonable.
                    if (has_mm()) {
                        // mm can take care of removing cloned extensions.
                        set_auto_extension(i, ext);
                    } else {
                        // no mm, user will call free_all_extensions().
                        set_extension(i, ext);
                    }
                }
            } else {
                // We already have such extension. Copy original over it.
                m_extensions[i]->copy_from(*other.m_extensions[i]);
            }
        }
    }
}

// To update the state of the original generic payload from a deep copy.
// Assumes that "other" was created from the original by calling
// deep_copy_from. Argument use_byte_enable_on_read determines whether to use
// or ignores byte enables when copying back the data array on a read command.

void
tlm_generic_payload::update_original_from(
        const tlm_generic_payload &other, bool use_byte_enable_on_read)
{
    // Copy back extensions that are present on the original.
    update_extensions_from(other);

    // Copy back the response status and DMI hint attributes.
    m_response_status = other.get_response_status();
    m_dmi = other.is_dmi_allowed();

    // Copy back the data array for a read command only deep_copy_from allowed
    // null pointers, and so will we.
    // We assume the arrays are the same size.
    // We test for equal pointers in case the original and the copy share the
    // same array.

    if (is_read() && m_data && other.m_data && m_data != other.m_data) {
        if (m_byte_enable && use_byte_enable_on_read) {
            if (m_byte_enable_length == 8 && m_length % 8 == 0) {
                // Optimized implementation copies 64-bit words by masking.
                for (unsigned int i = 0; i < m_length; i += 8) {
                    typedef sc_dt::uint64 *u;
                    *reinterpret_cast<u>(&m_data[i]) &=
                        ~*reinterpret_cast<u>(m_byte_enable);
                    *reinterpret_cast<u>(&m_data[i]) |=
                        *reinterpret_cast<u>(&other.m_data[i]) &
                        *reinterpret_cast<u>(m_byte_enable);
                }
            } else if (m_byte_enable_length == 4 && m_length % 4 == 0) {
                // Optimized implementation copies 32-bit words by masking.
                for (unsigned int i = 0; i < m_length; i += 4) {
                    typedef unsigned int *u;
                    *reinterpret_cast<u>(&m_data[i]) &=
                        ~*reinterpret_cast<u>(m_byte_enable);
                    *reinterpret_cast<u>(&m_data[i]) |=
                        *reinterpret_cast<u>(&other.m_data[i]) &
                        *reinterpret_cast<u>(m_byte_enable);
                }
            } else {
                // Unoptimized implementation.
                for (unsigned int i = 0; i < m_length; i++) {
                    if (m_byte_enable[i % m_byte_enable_length])
                        m_data[i] = other.m_data[i];
                }
            }
        } else {
            std::memcpy(m_data, other.m_data, m_length);
        }
    }
}

void
tlm_generic_payload::update_extensions_from(const tlm_generic_payload &other)
{
    // Deep copy extensions that are already present.
    sc_assert(m_extensions.size() <= other.m_extensions.size());
    for (unsigned int i = 0; i < m_extensions.size(); i++) {
        if (other.m_extensions[i]) {
            // Original has extension i.
            if (m_extensions[i]) {
                // We have it too. Copy.
                m_extensions[i]->copy_from(*other.m_extensions[i]);
            }
        }
    }
}

// Free all extensions. Useful when reusing a cloned transaction that doesn't
// have memory manager. Normal and sticky extensions are freed and extension
// array cleared.
void
tlm_generic_payload::free_all_extensions()
{
    m_extensions.free_entire_cache();
    for (unsigned int i = 0; i < m_extensions.size(); i++) {
        if (m_extensions[i]) {
            m_extensions[i]->free();
            m_extensions[i] = 0;
        }
    }
}

tlm_generic_payload::~tlm_generic_payload()
{
    for (unsigned int i = 0; i < m_extensions.size(); i++) {
        if (m_extensions[i])
            m_extensions[i]->free();
    }
}

//----------------
// API (including setters & getters)
//---------------

std::string
tlm_generic_payload::get_response_string() const
{
    switch (m_response_status) {
      case TLM_OK_RESPONSE:
        return "TLM_OK_RESPONSE";
      case TLM_INCOMPLETE_RESPONSE:
        return "TLM_INCOMPLETE_RESPONSE";
      case TLM_GENERIC_ERROR_RESPONSE:
        return "TLM_GENERIC_ERROR_RESPONSE";
      case TLM_ADDRESS_ERROR_RESPONSE:
        return "TLM_ADDRESS_ERROR_RESPONSE";
      case TLM_COMMAND_ERROR_RESPONSE:
        return "TLM_COMMAND_ERROR_RESPONSE";
      case TLM_BURST_ERROR_RESPONSE:
        return "TLM_BURST_ERROR_RESPONSE";
      case TLM_BYTE_ENABLE_ERROR_RESPONSE:
        return "TLM_BYTE_ENABLE_ERROR_RESPONSE";
    }
    return "TLM_UNKNOWN_RESPONSE";
}

/* --------------------------------------------------------------------- */
/* Dynamic extension mechanism:                                          */
/* --------------------------------------------------------------------- */

tlm_extension_base *
tlm_generic_payload::set_extension(unsigned int index, tlm_extension_base *ext)
{
    sc_assert(index < m_extensions.size());
    tlm_extension_base *tmp = m_extensions[index];
    m_extensions[index] = ext;
    return tmp;
}

tlm_extension_base *
tlm_generic_payload::set_auto_extension(
        unsigned int index, tlm_extension_base *ext)
{
    sc_assert(index < m_extensions.size());
    tlm_extension_base *tmp = m_extensions[index];
    m_extensions[index] = ext;
    if (!tmp)
        m_extensions.insert_in_cache(&m_extensions[index]);
    sc_assert(m_mm != 0);
    return tmp;
}

tlm_extension_base *
tlm_generic_payload::get_extension(unsigned int index) const
{
    sc_assert(index < m_extensions.size());
    return m_extensions[index];
}

void
tlm_generic_payload::clear_extension(unsigned int index)
{
    sc_assert(index < m_extensions.size());
    m_extensions[index] = static_cast<tlm_extension_base *>(0);
}

void
tlm_generic_payload::release_extension(unsigned int index)
{
    sc_assert(index < m_extensions.size());
    if (m_mm) {
        m_extensions.insert_in_cache(&m_extensions[index]);
    } else {
        m_extensions[index]->free();
        m_extensions[index] = static_cast<tlm_extension_base *>(nullptr);
    }
}

void
tlm_generic_payload::resize_extensions()
{
    m_extensions.expand(max_num_extensions());
}

} // namespace tlm

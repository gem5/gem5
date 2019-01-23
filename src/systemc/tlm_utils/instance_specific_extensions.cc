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

#include <tlm_utils/instance_specific_extensions_int.h>

#include <iostream>
#include <map>
#include <typeindex>

namespace tlm
{

template class tlm_array<tlm_utils::ispex_base *>;

} // namespace tlm

namespace tlm_utils
{

namespace
{

class ispex_registry // Copied from tlm_gp.cpp.
{
    typedef unsigned int key_type;
    typedef std::map<std::type_index, key_type> type_map;

  public:
    static ispex_registry &
    instance()
    {
        if (!instance_) {
            // Don't cleanup registry.
            instance_ = new ispex_registry();
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
    static ispex_registry *instance_;
    type_map ids_;
    ispex_registry() {}
};

ispex_registry *ispex_registry::instance_ = nullptr;

} //  anonymous namespace

unsigned int
ispex_base::register_private_extension(const std::type_info &type)
{
    return ispex_registry::instance().register_extension(type);
}

// Helper to do the numbering of private extension accessors.
static unsigned int
max_num_ispex_accessors(bool increment=false)
{
    static unsigned int max_num = 0;
    if (increment)
        ++max_num;
    return max_num;
}

// ----------------------------------------------------------------------------

// The pool for the container, plain as can be.
class instance_specific_extension_container_pool
{
    instance_specific_extension_container_pool() : unused(nullptr) {}
    ~instance_specific_extension_container_pool();

  public:
    static instance_specific_extension_container_pool &
    instance()
    {
        static instance_specific_extension_container_pool inst;
        return inst;
    }

    instance_specific_extension_container *create();
    void free(instance_specific_extension_container *);

  private:
    instance_specific_extension_container *unused;
};

instance_specific_extension_container *
instance_specific_extension_container_pool::create()
{
    if (!unused) {
        unused = new instance_specific_extension_container();
    }
    instance_specific_extension_container *tmp = unused;
    unused = unused->next;
    return tmp;
}

void
instance_specific_extension_container_pool::free(
        instance_specific_extension_container *cont)
{
    cont->next = unused;
    unused = cont;
}

instance_specific_extension_container_pool::
    ~instance_specific_extension_container_pool()
{
    while (unused) {
        instance_specific_extension_container *tmp = unused;
        unused = unused->next;
        delete tmp;
    }
}

// ----------------------------------------------------------------------------

instance_specific_extension_container *
instance_specific_extension_container::create()
{
    return instance_specific_extension_container_pool::instance().create();
}

instance_specific_extension_container::
    instance_specific_extension_container() :
    use_count(0), m_txn(NULL), m_release_fn(NULL), m_carrier(NULL), next(NULL)
{
    resize();
}

void
instance_specific_extension_container::
    attach_carrier(instance_specific_extension_carrier *carrier,
            void *txn, release_fn *rel_fn)
{
    m_txn = txn;
    m_release_fn = rel_fn;
    m_carrier = carrier;
}

void
instance_specific_extension_container::resize()
{
    m_ispex_per_accessor.resize(max_num_ispex_accessors());

    for (unsigned int i = 0; i < m_ispex_per_accessor.size(); ++i) {
        m_ispex_per_accessor[i] =
            new instance_specific_extensions_per_accessor(this);
        m_ispex_per_accessor[i]->resize_extensions();
    }
}

instance_specific_extension_container::
    ~instance_specific_extension_container()
{
    for (unsigned int i = 0; i < m_ispex_per_accessor.size(); ++i)
        delete m_ispex_per_accessor[i];
}

void
instance_specific_extension_container::inc_use_count()
{
    use_count++;
}

void
instance_specific_extension_container::dec_use_count()
{
    if ((--use_count) == 0) {
        // If this container isn't used any more we release the carrier
        // extension.
        m_release_fn(m_carrier, m_txn);
        // We send it back to our pool.
        instance_specific_extension_container_pool::instance().free(this);
    }
}

instance_specific_extensions_per_accessor *
instance_specific_extension_container::get_accessor(unsigned int idx)
{
    return m_ispex_per_accessor[idx];
}

// ----------------------------------------------------------------------------

// non-templatized version with manual index:
ispex_base *
instance_specific_extensions_per_accessor::set_extension(
        unsigned int index, ispex_base *ext)
{
    resize_extensions();
    ispex_base *tmp = m_extensions[index];
    m_extensions[index] = ext;
    if (!tmp && ext)
        m_container->inc_use_count();
    return tmp;
}

ispex_base *
instance_specific_extensions_per_accessor::get_extension(
        unsigned int index) const
{
    return (index < m_extensions.size()) ? m_extensions[index] : nullptr;
}

void
instance_specific_extensions_per_accessor::clear_extension(unsigned int index)
{
    if (index < m_extensions.size()) {
        if (m_extensions[index])
            m_container->dec_use_count();
        m_extensions[index] = static_cast<ispex_base *>(nullptr);
    }
}

void
instance_specific_extensions_per_accessor::resize_extensions()
{
    m_extensions.expand(ispex_registry::max_num_extensions());
}

// ----------------------------------------------------------------------------

instance_specific_extension_accessor::instance_specific_extension_accessor() :
    m_index(max_num_ispex_accessors(true) - 1)
{}

} // namespace tlm_utils

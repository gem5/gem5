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
#ifndef __SYSTEMC_EXT_TLM_UTILS_INSTANCE_SPECIFIC_EXTENSIONS_INT_H__
#define __SYSTEMC_EXT_TLM_UTILS_INSTANCE_SPECIFIC_EXTENSIONS_INT_H__

#include <typeinfo>
#include <vector>

#include "../tlm_core/2/generic_payload/array.hh"

namespace tlm_utils
{

class ispex_base;
class instance_specific_extension_accessor;
class instance_specific_extension_container;
class instance_specific_extension_carrier;
class instance_specific_extension_container_pool;

} // namespace tlm_utils

namespace tlm
{

extern template class tlm_array<tlm_utils::ispex_base *>;

} // namespace tlm

namespace tlm_utils
{

// The private extension base. Similar to normal extension base, but without
// clone and free.
class ispex_base
{
    friend class tlm::tlm_array<ispex_base*>;
    void free() {} // Needed for explicit tlm_array instantiation.

  public:
    virtual ~ispex_base() {}

  protected:
    static unsigned int register_private_extension(const std::type_info &);
};

// This thing is basically a snippet of the generic_payload.
// It contains all the extension specific code (the extension API so to speak)
// the differences are:
// - it calls back to its owner whenever a real (==non-NULL) extension gets
// set for the first time.
// - it calls back to its owner whenever a living (==non-NULL) extension gets
// cleared.
class instance_specific_extensions_per_accessor
{
  public:
    typedef instance_specific_extension_container container_type;

    explicit
    instance_specific_extensions_per_accessor(container_type *container) :
        m_container(container)
    {}

    template <typename T>
    T *
    set_extension(T *ext)
    {
        return static_cast<T *>(set_extension(T::priv_id, ext));
    }

    // Non-templatized version with manual index:
    ispex_base *set_extension(unsigned int index, ispex_base *ext);

    // Check for an extension, ext will be nullptr if not present.
    template <typename T>
    void get_extension(T *& ext) const
    {
        ext = static_cast<T *>(get_extension(T::priv_id));
    }
    // Non-templatized version:
    ispex_base *get_extension(unsigned int index) const;

    // Clear extension, the argument is needed to find the right index:
    template <typename T>
    void clear_extension(const T *)
    {
        clear_extension(T::priv_id);
    }

    // Non-templatized version with manual index
    void clear_extension(unsigned int index);

    // Make sure the extension array is large enough. Can be called once by
    // an initiator module (before issuing the first transaction) to make
    // sure that the extension array is of correct size. This is only needed
    // if the initiator cannot guarantee that the generic payload object is
    // allocated after C++ static construction time.
    void resize_extensions();

  private:
    tlm::tlm_array<ispex_base *> m_extensions;
    container_type* m_container;
};

// This thing contains the vector of extensions per accessor
// which can be really large so this one should be pool allocated.
// Therefore it keeps a use_count of itself to automatically free itself.
// - to this end it provides callbacks to the extensions per accessor
//   to increment and decrement the use_count.
class instance_specific_extension_container
{
    friend class instance_specific_extension_accessor;
    friend class instance_specific_extension_carrier;
    friend class instance_specific_extension_container_pool;
    friend class instance_specific_extensions_per_accessor;

    typedef void release_fn(instance_specific_extension_carrier *, void *);

    instance_specific_extension_container();
    ~instance_specific_extension_container();

    void resize();

    void inc_use_count();
    void dec_use_count();

    static instance_specific_extension_container *create();
    void attach_carrier(
            instance_specific_extension_carrier *, void *txn, release_fn *);

    instance_specific_extensions_per_accessor *
        get_accessor(unsigned int index);

    std::vector<instance_specific_extensions_per_accessor *>
        m_ispex_per_accessor;
    unsigned int use_count;
    void *m_txn;
    release_fn *m_release_fn;
    instance_specific_extension_carrier *m_carrier;
    instance_specific_extension_container *next; // For pooling.
};

// ----------------------------------------------------------------------------

// This class 'hides' all the instance specific extension stuff from the user.
// They instantiates one of those (e.g. instance_specific_extension_accessor
// extAcc;) and can then access the private extensions.
//    extAcc(txn).extensionAPIFnCall()
// where extensionAPIFnCall is set_extension, get_extension,
//      clear_extension,...
class instance_specific_extension_accessor
{
  public:
    instance_specific_extension_accessor();

    // Implementation in instance_specific_extensions.h
    template <typename T>
    inline instance_specific_extensions_per_accessor &operator () (T &txn);

  protected:
    template<typename T>
    static void release_carrier(
            instance_specific_extension_carrier *, void * txn);

    unsigned int m_index;
};

} // namespace tlm_utils

#endif /* __SYSTEMC_EXT_TLM_UTILS_INSTANCE_SPECIFIC_EXTENSIONS_INT_H__ */

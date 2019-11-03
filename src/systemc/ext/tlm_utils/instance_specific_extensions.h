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

/*
Instance specific extensions, are extension that only a single instance of a
module may access. They are invisible to all other modules; they are private
to this instance so to speak.

As they are only of value to a certain instance, this instance knows very
well when it needs them and when it does not need them any longer (usually
when a transaction passes through a module for the last time). It does not
have to care if anyone else in the system may still have a reference to the
transaction as this one is not able to access the extension anyway.
Therefore the instance is obliged to call set_extension when it wants to add a
private extension and clear_extension when it does not need it any more.

To get access to an instance specifc extension the module must own a so called
instance_specific_extension_accessor that provides the exclusive access rights.
Assuming the instance_specific_extension_accessor of a given module is called
m_accessor and the transaction of which the private extension is about to be
accessed is called txn, then the calls have to be

m_accessor(txn).set_extension(...);
or
m_accessor(txn).clear_extension(...);

The owner of the private extension is responsible to allocate/deallocate
the extension before/after setting/clearing the extension.
*/

#ifndef __SYSTEMC_EXT_TLM_UTILS_INSTANCE_SPECIFIC_EXTENSIONS_H__
#define __SYSTEMC_EXT_TLM_UTILS_INSTANCE_SPECIFIC_EXTENSIONS_H__

#include "instance_specific_extensions_int.h"

namespace tlm_utils
{

// The templated private extension. Similar to normal extension.
template <typename T>
class instance_specific_extension : public ispex_base
{
  public:
    virtual ~instance_specific_extension() {}
    const static unsigned int priv_id;
};

template <typename T>
const unsigned int instance_specific_extension<T>::priv_id =
    ispex_base::register_private_extension(typeid(T));

// ----------------------------------------------------------------------------

// This is the class that actually sits in the extension array
// - We keep this small since that one gets allocated and deallocated all
//   the times.
// - We keep the implementation in the header to avoid registration
//   of the extension itself unless used in the model.
class instance_specific_extension_carrier :
    public tlm::tlm_extension<instance_specific_extension_carrier>
{
    friend class instance_specific_extension_accessor;
  public:
    instance_specific_extension_carrier() : m_container() {}

    virtual tlm::tlm_extension_base *
    clone() const
    {
        // We don't clone since private info is instance specific and
        // associated to a given txn (the original) so the deep copied txn
        // will be virgin in terms of private info.
        return NULL;
    }

    void copy_from(tlm::tlm_extension_base const &) { return; }
    void free() { return; }

  private:
    instance_specific_extension_container *m_container;
};

// ----------------------------------------------------------------------------

template <typename T>
instance_specific_extensions_per_accessor &
instance_specific_extension_accessor::operator () (T &txn)
{
    instance_specific_extension_carrier *carrier = NULL;
    txn.get_extension(carrier);
    if (!carrier) {
        carrier = new instance_specific_extension_carrier();
        carrier->m_container = instance_specific_extension_container::create();
        carrier->m_container->attach_carrier(
                carrier, &txn, &release_carrier<T>);
        txn.set_extension(carrier);
    }
    return *carrier->m_container->get_accessor(m_index);
}

template <typename T>
void
instance_specific_extension_accessor::release_carrier(
        instance_specific_extension_carrier *carrier, void *txn)
{
    T *typed_txn = static_cast<T *>(txn);
    typed_txn->clear_extension(carrier);
    delete carrier;
}

} // namespace tlm_utils

#endif /* __SYSTEMC_EXT_TLM_UTILS_INSTANCE_SPECIFIC_EXTENSIONS_H__ */

/*
 * Copyright 2018 Google, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#ifndef __SYSTEMC_EXT_UTIL_SC_VECTOR_HH__
#define __SYSTEMC_EXT_UTIL_SC_VECTOR_HH__

#include <stdint.h>

#include <exception>
#include <vector>

#include "../core/sc_object.hh"
#include "warn_unimpl.hh"

namespace sc_core
{

template <typename T, typename MT>
class sc_vector_assembly;

template <typename T>
class sc_vector;

template <typename T, typename MT>
sc_vector_assembly<T, MT> sc_assemble_vector(
        sc_vector<T> &, MT(T::* member_ptr));

class sc_vector_base : public sc_object
{
  public:
    typedef size_t size_type;

    virtual const char *kind() const { return "sc_vector"; }
    size_type size() const;
    const std::vector<sc_object *> &get_elements() const;
};

template <typename T>
class sc_vector_iter :
        public std::iterator<std::random_access_iterator_tag, T>
{
    // Conforms to Random Access Iterator category.
    // See ISO/IEC 14882:2003(E), 24.1 [lib.iterator.requirements]

    // Implementation-defined
};

template <typename T>
class sc_vector : public sc_vector_base
{
  public:
    using sc_vector_base::size_type;
    typedef sc_vector_iter<T> iterator;
    typedef sc_vector_iter<const T> const_iterator;

    sc_vector() : sc_vector_base()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }
    explicit sc_vector(const char *) : sc_vector_base()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }
    sc_vector(const char *, size_type) : sc_vector_base()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }
    template <typename Creator>
    sc_vector(const char *, size_type, Creator) : sc_vector_base()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual ~sc_vector() {}

    void
    init(size_type)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }
    static T *
    create_element(const char *, size_type)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return nullptr;
    }

    template <typename Creator>
    void
    init(size_type, Creator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }

    T &
    operator [] (size_type)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return *(T *)nullptr;
    }
    const T &
    operator [] (size_type) const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const T *)nullptr;
    }

    T &
    at(size_type)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return *(T *)nullptr;
    }
    const T &
    at(size_type) const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const T *)nullptr;
    }

    iterator
    begin()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }
    iterator
    end()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    const_iterator
    begin() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return const_iterator();
    }
    const_iterator
    end() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return const_iterator();
    }

    const_iterator
    cbegin() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return const_iterator();
    }
    const_iterator
    cend() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return const_iterator();
    }

    template <typename ContainerType, typename ArgumentType>
    iterator
    bind(sc_vector_assembly<ContainerType, ArgumentType>)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename BindableContainer>
    iterator
    bind(BindableContainer &)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename BindableIterator>
    iterator
    bind(BindableIterator, BindableIterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename BindableIterator>
    iterator
    bind(BindableIterator, BindableIterator, iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ContainerType, typename ArgumentType>
    iterator
    operator () (sc_vector_assembly<ContainerType, ArgumentType> c)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ArgumentContainer>
    iterator
    operator () (ArgumentContainer &)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ArgumentIterator>
    iterator
    operator () (ArgumentIterator, ArgumentIterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ArgumentIterator>
    iterator
    operator () (ArgumentIterator, ArgumentIterator, iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

  private:
    // Disabled
    sc_vector(const sc_vector &) : sc_vector_base() {}
    sc_vector &operator = (const sc_vector &) { return *this; }
};

template <typename T, typename MT>
class sc_vector_assembly
{
  public:
    friend sc_vector_assembly<T, MT> sc_assemble_vector<>(
            sc_vector<T> &, MT(T::* member_ptr));

    typedef size_t size_type;
    // These next two types are supposed to be implementation defined. We'll
    // just stick in a substitute for now, but these should probably not just
    // be STL vector iterators.
    typedef typename std::vector<T>::iterator iterator;
    typedef typename std::vector<T>::const_iterator const_iterator;
    typedef MT (T::* member_type);

    sc_vector_assembly(const sc_vector_assembly &)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }

    iterator
    begin()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }
    iterator
    end()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    const_iterator
    cbegin() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return const_iterator();
    }
    const_iterator
    cend() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return const_iterator();
    }

    size_type
    size() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return 0;
    }
    std::vector<sc_object *>
    get_elements() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return *(std::vector<sc_object *> *)nullptr;
    }

    typename iterator::reference
    operator [] (size_type)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return typename iterator::reference();
    }
    typename const_iterator::reference
    operator [] (size_type) const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return typename iterator::reference();
    }

    typename iterator::reference
    at(size_type)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return typename iterator::reference();
    }
    typename const_iterator::reference
    at(size_type) const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return typename iterator::reference();
    }

    template <typename ContainerType, typename ArgumentType>
    iterator
    bind(sc_vector_assembly<ContainerType, ArgumentType>)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename BindableContainer>
    iterator
    bind(BindableContainer &)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename BindableIterator>
    iterator
    bind(BindableIterator, BindableIterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename BindableIterator>
    iterator
    bind(BindableIterator, BindableIterator, iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename BindableIterator>
    iterator
    bind(BindableIterator, BindableIterator, typename sc_vector<T>::iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ContainerType, typename ArgumentType>
    iterator
    operator () (sc_vector_assembly<ContainerType, ArgumentType>)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ArgumentContainer>
    iterator
    operator () (ArgumentContainer &)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ArgumentIterator>
    iterator
    operator () (ArgumentIterator, ArgumentIterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ArgumentIterator>
    iterator
    operator () (ArgumentIterator, ArgumentIterator, iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

    template <typename ArgumentIterator>
    iterator
    operator () (ArgumentIterator, ArgumentIterator,
                 typename sc_vector<T>::iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return iterator();
    }

  private:
    // Temporary constructor which will (eventually) actually bind an
    // sc_vector_assembly instance to an sc_vector.
    sc_vector_assembly<T, MT>()
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }
};

template <typename T, typename MT>
sc_vector_assembly<T, MT>
sc_assemble_vector(sc_vector<T> &, MT(T::* member_ptr))
{
    sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    return sc_vector_assembly<T, MT>();
}

} // namespace sc_core

#endif  //__SYSTEMC_EXT_UTIL_SC_VECTOR_HH__

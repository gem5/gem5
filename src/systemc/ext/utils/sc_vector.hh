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
#include <iterator>
#include <vector>

#include "../core/sc_object.hh"
#include "warn_unimpl.hh"

namespace sc_gem5
{

// Goop for supporting sc_vector_iter, simplified from the Accellera version.

#if __cplusplus >= 201103L

using std::enable_if;
using std::remove_const;
using std::is_same;
using std::is_const;

#else

template<bool Cond, typename T=void>
struct enable_if
{};

template<typename T>
struct enable_if<true, T>
{
    typedef T type;
};

template <typename T>
struct remove_const
{
    typedef T type;
};
template <typename T>
struct remove_const<const T>
{
    typedef T type;
};

template <typename T, typename U>
struct is_same
{
    static const bool value = false;
};
template <typename T>
struct is_same<T, T>
{
    static const bool value = true;
};

template <typename T>
struct is_const
{
    static const bool value = false;
};
template <typename T>
struct is_const<const T>
{
    static const bool value = true;
};

#endif

template <typename CT, typename T>
struct is_more_const
{
    static const bool value =
        is_same<typename remove_const<CT>::type,
                typename remove_const<T>::type>::value &&
        is_const<CT>::value >= is_const<T>::value;
};

struct special_result
{};

template <typename T>
struct remove_special_fptr
{};

template <typename T>
struct remove_special_fptr<special_result & (*)(T)>
{
    typedef T type;
};

#define SC_RPTYPE_(Type) \
    ::sc_gem5::remove_special_fptr< \
        ::sc_gem5::special_result & (*) Type>::type::value

#define SC_ENABLE_IF_(Cond) \
    typename ::sc_gem5::enable_if<SC_RPTYPE_(Cond)>::type * = NULL

} // namespace sc_gem5

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


/*
 * Non-standard iterator access adapters. Without using these, the classes as
 * defined in the standard won't compile because of redundant bind() overloads.
 */

template <typename Element>
class sc_direct_access
{
  public:
    typedef Element ElementType;
    typedef ElementType Type;
    typedef typename sc_gem5::remove_const<ElementType>::type PlainType;

    typedef sc_direct_access<ElementType> Policy;
    typedef sc_direct_access<PlainType> NonConstPolicy;
    typedef sc_direct_access<const PlainType> ConstPolicy;

    sc_direct_access() {}
    sc_direct_access(const NonConstPolicy &) {}

    template <typename U>
    sc_direct_access(const U &,
        SC_ENABLE_IF_((
            sc_gem5::is_more_const<
                    ElementType, typename U::Policy::ElementType>
        ))
    )
    {}

    ElementType *
    get(ElementType *this_) const
    {
        return this_;
    }
};

template <typename Element, typename Access>
class sc_member_access
{
  public:
    template <typename, typename>
    friend class sc_member_access;

    typedef Element ElementType;
    typedef Access AccessType;
    typedef AccessType (ElementType::*MemberType);
    typedef AccessType Type;
    typedef typename sc_gem5::remove_const<AccessType>::type PlainType;
    typedef typename sc_gem5::remove_const<ElementType>::type PlainElemType;

    typedef sc_member_access<ElementType, AccessType> Policy;
    typedef sc_member_access<PlainElemType, PlainType> NonConstPolicy;
    typedef sc_member_access<const PlainElemType, const PlainType> ConstPolicy;

    sc_member_access(MemberType ptr) : ptr_(ptr) {}
    sc_member_access(const NonConstPolicy &other) : ptr_(other.ptr_) {}

    AccessType *get(ElementType *this_) const { return &(this_->*ptr_); }

  private:
    MemberType ptr_;
};

template <typename Element,
          typename AccessPolicy=sc_direct_access<Element> >
class sc_vector_iter :
        public std::iterator<std::random_access_iterator_tag,
                             typename AccessPolicy::Type>,
        private AccessPolicy
{
  private:
    typedef Element ElementType;
    typedef typename AccessPolicy::Policy Policy;
    typedef typename AccessPolicy::NonConstPolicy NonConstPolicy;
    typedef typename AccessPolicy::ConstPolicy ConstPolicy;
    typedef typename Policy::Type AccessType;

    typedef typename sc_gem5::remove_const<ElementType>::type PlainType;
    typedef const PlainType ConstPlainType;
    typedef typename sc_direct_access<PlainType>::ConstPolicy
        ConstDirectPolicy;

    friend class sc_vector<PlainType>;
    template <typename, typename>
    friend class sc_vector_assembly;
    template <typename, typename>
    friend class sc_vector_iter;

    typedef std::iterator<std::random_access_iterator_tag, AccessType>
        BaseType;
    typedef sc_vector_iter ThisType;
    typedef sc_vector<PlainType> VectorType;
    typedef std::vector<void *> StorageType;

    template <typename U>
    struct SelectIter
    {
        typedef typename std::vector<void *>::iterator type;
    };
    template <typename U>
    struct SelectIter<const U>
    {
        typedef typename std::vector<void *>::const_iterator type;
    };
    typedef typename SelectIter<ElementType>::type RawIterator;
    typedef sc_vector_iter<ConstPlainType, ConstPolicy> ConstIterator;
    typedef sc_vector_iter<ConstPlainType, ConstDirectPolicy>
        ConstDirectIterator;

    RawIterator it_;

    sc_vector_iter(RawIterator it, Policy acc=Policy()) :
        Policy(acc), it_(it)
    {}

    Policy const &get_policy() const { return *this; }

  public:
    // Conforms to Random Access Iterator category.
    // See ISO/IEC 14882:2003(E), 24.1 [lib.iterator.requirements]

    typedef typename BaseType::difference_type difference_type;
    typedef typename BaseType::reference reference;
    typedef typename BaseType::pointer pointer;

    sc_vector_iter() : Policy(), it_() {}

    template <typename It>
    sc_vector_iter(const It &it,
        SC_ENABLE_IF_((
            sc_gem5::is_more_const<
                ElementType, typename It::Policy::ElementType>
        ))
    ) : Policy(it.get_policy()), it_(it.it_)
    {}

    ThisType &
    operator ++ ()
    {
        ++it_;
        return *this;
    }
    ThisType &
    operator -- ()
    {
        --it_;
        return *this;
    }
    ThisType
    operator ++ (int)
    {
        ThisType old(*this);
        ++it_;
        return old;
    }
    ThisType
    operator -- (int)
    {
        ThisType old(*this);
        --it_;
        return old;
    }

    ThisType
    operator + (difference_type n) const
    {
        return ThisType(it_ + n, get_policy());
    }
    ThisType
    operator - (difference_type n) const
    {
        return ThisType(it_ - n, get_policy());
    }

    ThisType &
    operator += (difference_type n)
    {
        it_ += n;
        return *this;
    }

    ThisType &
    operator -= (difference_type n)
    {
        it_ -= n;
        return *this;
    }

    bool
    operator == (const ConstDirectIterator &other) const
    {
        return it_ == other.it_;
    }
    bool
    operator != (const ConstDirectIterator &other) const
    {
        return it_ != other.it_;
    }
    bool
    operator <= (const ConstDirectIterator &other) const
    {
        return it_ <= other.it_;
    }
    bool
    operator >= (const ConstDirectIterator &other) const
    {
        return it_ >= other.it_;
    }
    bool
    operator < (const ConstDirectIterator &other) const
    {
        return it_ < other.it_;
    }
    bool
    operator > (const ConstDirectIterator &other) const
    {
        return it_ > other.it_;
    }

    reference
    operator * () const
    {
        return *Policy::get(static_cast<ElementType *>((void *)*it_));
    }
    pointer
    operator -> () const
    {
        return Policy::get(static_cast<ElementType *>((void *)*it_));
    }
    reference
    operator [] (difference_type n) const
    {
        return *Policy::get(static_cast<ElementType *>((void *)it_[n]));
    }

    difference_type
    operator - (ConstDirectIterator const &other) const
    {
        return it_ - other.it_;
    }
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
            sc_vector<T> &, MT (T::*));

    typedef size_t size_type;
    typedef sc_vector_iter<T, sc_member_access<T, MT> > iterator;
    typedef sc_vector_iter<
        const T, sc_member_access<const T, const MT> > const_iterator;
    typedef MT (T::*MemberType);

    sc_vector_assembly(const sc_vector_assembly &)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
    }

    iterator begin() { return iterator(vec_->begin().it_, ptr_); }
    iterator end() { return iterator(vec_->end().it_, ptr_); }

    const_iterator
    cbegin() const
    {
        return const_iterator(vec_->begin().it_, ptr_);
    }
    const_iterator
    cend() const
    {
        return const_iterator(vec_->end().it_, ptr_);
    }

    const_iterator
    begin() const
    {
        return const_iterator(vec_->begin().it_, ptr_);
    }
    const_iterator
    end() const
    {
        return const_iterator(vec_->end().it_, ptr_);
    }

    size_type size() const { return vec_->size(); }

    std::vector<sc_object *>
    get_elements() const
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return *(std::vector<sc_object *> *)nullptr;
    }

    typename iterator::reference
    operator [] (size_type i)
    {
        return (*vec_)[i].*ptr_;
    }
    typename const_iterator::reference
    operator [] (size_type i) const
    {
        return (*vec_)[i].*ptr_;
    }

    typename iterator::reference
    at(size_type i)
    {
        return vec_->at(i).*ptr_;
    }
    typename const_iterator::reference
    at(size_type i) const
    {
        return vec_->at(i).*ptr_;
    }

    template <typename ContainerType, typename ArgumentType>
    iterator
    bind(sc_vector_assembly<ContainerType, ArgumentType>)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename BindableContainer>
    iterator
    bind(BindableContainer &)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename BindableIterator>
    iterator
    bind(BindableIterator, BindableIterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename BindableIterator>
    iterator
    bind(BindableIterator, BindableIterator, iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename BindableIterator>
    iterator
    bind(BindableIterator, BindableIterator, typename sc_vector<T>::iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename ContainerType, typename ArgumentType>
    iterator
    operator () (sc_vector_assembly<ContainerType, ArgumentType>)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename ArgumentContainer>
    iterator
    operator () (ArgumentContainer &)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename ArgumentIterator>
    iterator
    operator () (ArgumentIterator, ArgumentIterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename ArgumentIterator>
    iterator
    operator () (ArgumentIterator, ArgumentIterator, iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

    template <typename ArgumentIterator>
    iterator
    operator () (ArgumentIterator, ArgumentIterator,
                 typename sc_vector<T>::iterator)
    {
        sc_utils_warn_unimpl(__PRETTY_FUNCTION__);
        return begin();
    }

  private:
    sc_vector_assembly(sc_vector<T> &v, MemberType ptr) :
        vec_(&v), ptr_(ptr)
    {}

    sc_vector<T> *vec_;
    MemberType ptr_;
};

template <typename T, typename MT>
sc_vector_assembly<T, MT>
sc_assemble_vector(sc_vector<T> &v, MT (T::*ptr))
{
    return sc_vector_assembly<T, MT>(v, ptr);
}

} // namespace sc_core

#endif  //__SYSTEMC_EXT_UTIL_SC_VECTOR_HH__

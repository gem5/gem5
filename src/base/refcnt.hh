/*
 * Copyright (c) 2017-2018 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
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
 */

#ifndef __BASE_REFCNT_HH__
#define __BASE_REFCNT_HH__

#include <type_traits>

/**
 * @file base/refcnt.hh
 *
 * Classes for managing reference counted objects.
 */

namespace gem5
{

/**
 * Derive from RefCounted if you want to enable reference counting of
 * this class.  If you want to use automatic reference counting, you
 * should use RefCountingPtr<T> instead of regular pointers.
 */
class RefCounted
{
  private:
    // The reference count is mutable because one may want to
    // reference count a const pointer.  This really is OK because
    // const is about logical constness of the object not really about
    // strictly disallowing an object to change.
    mutable int count;

  private:
    // Don't allow a default copy constructor or copy operator on
    // these objects because the default operation will copy the
    // reference count as well and we certainly don't want that.
    RefCounted(const RefCounted &);
    RefCounted &operator=(const RefCounted &);

  public:
    /**
     * We initialize the reference count to zero and the first object
     * to take ownership of it must increment it to one.
     *
     * @attention A memory leak will occur if you never assign a newly
     * constructed object to a reference counting pointer.
     */
    RefCounted() : count(0) {}

    /**
     * We make the destructor virtual because we're likely to have
     * virtual functions on reference counted objects.
     *
     * @todo Even if this were true, does it matter?  Shouldn't the
     * derived class indicate this?  This only matters if we would
     * ever choose to delete a "RefCounted *" which I doubt we'd ever
     * do.  We don't ever delete a "void *".
     */
    virtual ~RefCounted() {}

    /// Increment the reference count
    void
    incref() const
    {
        ++count;
    }

    /// Decrement the reference count and destroy the object if all
    /// references are gone.
    void
    decref() const
    {
        if (--count <= 0)
            delete this;
    }
};

/**
 * If you want a reference counting pointer to a mutable object,
 * create it like this:
 * @code
 * typedef RefCountingPtr<Foo> FooPtr;
 * @endcode
 *
 * @attention Do not use "const FooPtr"
 * To create a reference counting pointer to a const object, use this:
 * @code
 * typedef RefCountingPtr<const Foo> ConstFooPtr;
 * @endcode
 *
 * These two usages are analogous to iterator and const_iterator in the stl.
 */
template <class T>
class RefCountingPtr
{
  public:
    using PtrType = T *;

  protected:
    /** Convenience aliases for const/non-const versions of T w/ friendship. */
    /** @{ */
    static constexpr auto TisConst = std::is_const_v<T>;
    using ConstT = typename std::conditional_t<
        TisConst, RefCountingPtr<T>,
        RefCountingPtr<typename std::add_const<T>::type>>;
    friend ConstT;
    using NonConstT = typename std::conditional_t<
        TisConst, RefCountingPtr<typename std::remove_const<T>::type>,
        RefCountingPtr<T>>;
    friend NonConstT;
    /** @} */
    /// The stored pointer.
    /// Arguably this should be private.
    T *data;

    /**
     * Copy a new pointer value and increment the reference count if
     * it is a valid pointer.  Note, this does not delete the
     * reference any existing object.
     * @param d Pointer to store.
     */
    void
    copy(T *d)
    {
        data = d;
        if (data)
            data->incref();
    }

    /**
     * Delete the reference to any existing object if it is non NULL.
     * @attention this doesn't clear the pointer value, so a double
     * decref could happen if not careful.
     */
    void
    del()
    {
        if (data)
            data->decref();
    }

    /**
     * Drop the old reference and change it to something new.
     */
    void
    set(T *d)
    {
        // Need to check if we're actually changing because otherwise
        // we could delete the last reference before adding the new
        // reference.
        if (data != d) {
            del();
            copy(d);
        }
    }

  public:
    /// Create an empty reference counting pointer.
    RefCountingPtr() : data(0) {}

    /// Create a new reference counting pointer to some object
    /// (probably something newly created).  Adds a reference.
    RefCountingPtr(T *data) { copy(data); }

    /// Create a new reference counting pointer by copying another
    /// one.  Adds a reference.
    RefCountingPtr(const RefCountingPtr &r) { copy(r.data); }

    /** Move-constructor.
     * Does not add a reference.
     */
    RefCountingPtr(RefCountingPtr &&r)
    {
        data = r.data;
        r.data = nullptr;
    }

    template <bool B = TisConst>
    RefCountingPtr(const NonConstT &r)
    {
        copy(r.data);
    }

    /// Destroy the pointer and any reference it may hold.
    ~RefCountingPtr() { del(); }

    // The following pointer access functions are const because they
    // don't actually change the pointer, though the user could change
    // what is pointed to.  This is analagous to a "Foo * const".

    /// Access a member variable.
    T *
    operator->() const
    {
        return data;
    }

    /// Dereference the pointer.
    T &
    operator*() const
    {
        return *data;
    }

    /// Directly access the pointer itself without taking a reference.
    T *
    get() const
    {
        return data;
    }

    template <bool B = TisConst>
    operator RefCountingPtr<typename std::enable_if_t<!B, ConstT>>()
    {
        return RefCountingPtr<const T>(*this);
    }

    /// Assign a new value to the pointer
    const RefCountingPtr &
    operator=(T *p)
    {
        set(p);
        return *this;
    }

    /// Copy the pointer from another RefCountingPtr
    const RefCountingPtr &
    operator=(const RefCountingPtr &r)
    {
        return operator=(r.data);
    }

    /// Move-assign the pointer from another RefCountingPtr
    const RefCountingPtr &
    operator=(RefCountingPtr &&r)
    {
        /* This happens regardless of whether the pointer is the same or not,
         * because of the move semantics, the rvalue needs to be 'destroyed'.
         */
        del();
        data = r.data;
        r.data = nullptr;
        return *this;
    }

    /// Check if the pointer is empty
    bool
    operator!() const
    {
        return data == 0;
    }

    /// Check if the pointer is non-empty
    operator bool() const { return data != 0; }
};

/// Check for equality of two reference counting pointers.
template <class T>
inline bool
operator==(const RefCountingPtr<T> &l, const RefCountingPtr<T> &r)
{
    return l.get() == r.get();
}

/// Check for equality of of a reference counting pointers and a
/// regular pointer
template <class T>
inline bool
operator==(const RefCountingPtr<T> &l, const T *r)
{
    return l.get() == r;
}

/// Check for equality of of a reference counting pointers and a
/// regular pointer
template <class T>
inline bool
operator==(const T *l, const RefCountingPtr<T> &r)
{
    return l == r.get();
}

/// Check for inequality of two reference counting pointers.
template <class T>
inline bool
operator!=(const RefCountingPtr<T> &l, const RefCountingPtr<T> &r)
{
    return l.get() != r.get();
}

/// Check for inequality of of a reference counting pointers and a
/// regular pointer
template <class T>
inline bool
operator!=(const RefCountingPtr<T> &l, const T *r)
{
    return l.get() != r;
}

/// Check for inequality of of a reference counting pointers and a
/// regular pointer
template <class T>
inline bool
operator!=(const T *l, const RefCountingPtr<T> &r)
{
    return l != r.get();
}

} // namespace gem5

#endif // __BASE_REFCNT_HH__

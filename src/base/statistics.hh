/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2017, Centre National de la Recherche Scientifique
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
 *
 * Authors: Nathan Binkert
 *          Pierre-Yves Peneau
 */

/** @file
 * Declaration of Statistics objects.
 */

/**
* @todo
*
* Generalized N-dimensinal vector
* documentation
* key stats
* interval stats
*   -- these both can use the same function that prints out a
*   specific set of stats
* VectorStandardDeviation totals
* Document Namespaces
*/
#ifndef __BASE_STATISTICS_HH__
#define __BASE_STATISTICS_HH__

#include <algorithm>
#include <cassert>
#ifdef __SUNPRO_CC
#include <math.h>
#endif
#include <cmath>
#include <functional>
#include <iosfwd>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "base/stats/info.hh"
#include "base/stats/output.hh"
#include "base/stats/types.hh"
#include "base/cast.hh"
#include "base/cprintf.hh"
#include "base/intmath.hh"
#include "base/str.hh"
#include "base/types.hh"

class Callback;

/** The current simulated tick. */
extern Tick curTick();

/* A namespace for all of the Statistics */
namespace Stats {

template <class Stat, class Base>
class InfoProxy : public Base
{
  protected:
    Stat &s;

  public:
    InfoProxy(Stat &stat) : s(stat) {}

    bool check() const { return s.check(); }
    void prepare() { s.prepare(); }
    void reset() { s.reset(); }
    void
    visit(Output &visitor)
    {
        visitor.visit(*static_cast<Base *>(this));
    }
    bool zero() const { return s.zero(); }
};

template <class Stat>
class ScalarInfoProxy : public InfoProxy<Stat, ScalarInfo>
{
  public:
    ScalarInfoProxy(Stat &stat) : InfoProxy<Stat, ScalarInfo>(stat) {}

    Counter value() const { return this->s.value(); }
    Result result() const { return this->s.result(); }
    Result total() const { return this->s.total(); }
};

template <class Stat>
class VectorInfoProxy : public InfoProxy<Stat, VectorInfo>
{
  protected:
    mutable VCounter cvec;
    mutable VResult rvec;

  public:
    VectorInfoProxy(Stat &stat) : InfoProxy<Stat, VectorInfo>(stat) {}

    size_type size() const { return this->s.size(); }

    VCounter &
    value() const
    {
        this->s.value(cvec);
        return cvec;
    }

    const VResult &
    result() const
    {
        this->s.result(rvec);
        return rvec;
    }

    Result total() const { return this->s.total(); }
};

template <class Stat>
class DistInfoProxy : public InfoProxy<Stat, DistInfo>
{
  public:
    DistInfoProxy(Stat &stat) : InfoProxy<Stat, DistInfo>(stat) {}
};

template <class Stat>
class VectorDistInfoProxy : public InfoProxy<Stat, VectorDistInfo>
{
  public:
    VectorDistInfoProxy(Stat &stat) : InfoProxy<Stat, VectorDistInfo>(stat) {}

    size_type size() const { return this->s.size(); }
};

template <class Stat>
class Vector2dInfoProxy : public InfoProxy<Stat, Vector2dInfo>
{
  public:
    Vector2dInfoProxy(Stat &stat) : InfoProxy<Stat, Vector2dInfo>(stat) {}

    Result total() const { return this->s.total(); }
};

struct StorageParams
{
    virtual ~StorageParams();
};

class InfoAccess
{
  protected:
    /** Set up an info class for this statistic */
    void setInfo(Info *info);
    /** Save Storage class parameters if any */
    void setParams(const StorageParams *params);
    /** Save Storage class parameters if any */
    void setInit();

    /** Grab the information class for this statistic */
    Info *info();
    /** Grab the information class for this statistic */
    const Info *info() const;

  public:
    /**
     * Reset the stat to the default state.
     */
    void reset() { }

    /**
     * @return true if this stat has a value and satisfies its
     * requirement as a prereq
     */
    bool zero() const { return true; }

    /**
     * Check that this stat has been set up properly and is ready for
     * use
     * @return true for success
     */
    bool check() const { return true; }
};

template <class Derived, template <class> class InfoProxyType>
class DataWrap : public InfoAccess
{
  public:
    typedef InfoProxyType<Derived> Info;

  protected:
    Derived &self() { return *static_cast<Derived *>(this); }

  protected:
    Info *
    info()
    {
        return safe_cast<Info *>(InfoAccess::info());
    }

  public:
    const Info *
    info() const
    {
        return safe_cast<const Info *>(InfoAccess::info());
    }

  protected:
    /**
     * Copy constructor, copies are not allowed.
     */
    DataWrap(const DataWrap &stat) = delete;

    /**
     * Can't copy stats.
     */
    void operator=(const DataWrap &) {}

  public:
    DataWrap()
    {
        this->setInfo(new Info(self()));
    }

    /**
     * Set the name and marks this stat to print at the end of simulation.
     * @param name The new name.
     * @return A reference to this stat.
     */
    Derived &
    name(const std::string &name)
    {
        Info *info = this->info();
        info->setName(name);
        info->flags.set(display);
        return this->self();
    }
    const std::string &name() const { return this->info()->name; }

    /**
     * Set the character(s) used between the name and vector number
     * on vectors, dist, etc.
     * @param _sep The new separator string
     * @return A reference to this stat.
     */
    Derived &
    setSeparator(const std::string &_sep)
    {
      this->info()->setSeparator(_sep);
      return this->self();
    }
    const std::string &setSeparator() const
    {
      return this->info()->separatorString;
    }

    /**
     * Set the description and marks this stat to print at the end of
     * simulation.
     * @param desc The new description.
     * @return A reference to this stat.
     */
    Derived &
    desc(const std::string &_desc)
    {
        this->info()->desc = _desc;
        return this->self();
    }

    /**
     * Set the precision and marks this stat to print at the end of simulation.
     * @param _precision The new precision
     * @return A reference to this stat.
     */
    Derived &
    precision(int _precision)
    {
        this->info()->precision = _precision;
        return this->self();
    }

    /**
     * Set the flags and marks this stat to print at the end of simulation.
     * @param f The new flags.
     * @return A reference to this stat.
     */
    Derived &
    flags(Flags _flags)
    {
        this->info()->flags.set(_flags);
        return this->self();
    }

    /**
     * Set the prerequisite stat and marks this stat to print at the end of
     * simulation.
     * @param prereq The prerequisite stat.
     * @return A reference to this stat.
     */
    template <class Stat>
    Derived &
    prereq(const Stat &prereq)
    {
        this->info()->prereq = prereq.info();
        return this->self();
    }
};

template <class Derived, template <class> class InfoProxyType>
class DataWrapVec : public DataWrap<Derived, InfoProxyType>
{
  public:
    typedef InfoProxyType<Derived> Info;

    DataWrapVec()
    {}

    DataWrapVec(const DataWrapVec &ref)
    {}

    void operator=(const DataWrapVec &)
    {}

    // The following functions are specific to vectors.  If you use them
    // in a non vector context, you will get a nice compiler error!

    /**
     * Set the subfield name for the given index, and marks this stat to print
     * at the end of simulation.
     * @param index The subfield index.
     * @param name The new name of the subfield.
     * @return A reference to this stat.
     */
    Derived &
    subname(off_type index, const std::string &name)
    {
        Derived &self = this->self();
        Info *info = self.info();

        std::vector<std::string> &subn = info->subnames;
        if (subn.size() <= index)
            subn.resize(index + 1);
        subn[index] = name;
        return self;
    }

    // The following functions are specific to 2d vectors.  If you use
    // them in a non vector context, you will get a nice compiler
    // error because info doesn't have the right variables.

    /**
     * Set the subfield description for the given index and marks this stat to
     * print at the end of simulation.
     * @param index The subfield index.
     * @param desc The new description of the subfield
     * @return A reference to this stat.
     */
    Derived &
    subdesc(off_type index, const std::string &desc)
    {
        Info *info = this->info();

        std::vector<std::string> &subd = info->subdescs;
        if (subd.size() <= index)
            subd.resize(index + 1);
        subd[index] = desc;

        return this->self();
    }

    void
    prepare()
    {
        Derived &self = this->self();
        Info *info = this->info();

        size_t size = self.size();
        for (off_type i = 0; i < size; ++i)
            self.data(i)->prepare(info);
    }

    void
    reset()
    {
        Derived &self = this->self();
        Info *info = this->info();

        size_t size = self.size();
        for (off_type i = 0; i < size; ++i)
            self.data(i)->reset(info);
    }
};

template <class Derived, template <class> class InfoProxyType>
class DataWrapVec2d : public DataWrapVec<Derived, InfoProxyType>
{
  public:
    typedef InfoProxyType<Derived> Info;

    /**
     * @warning This makes the assumption that if you're gonna subnames a 2d
     * vector, you're subnaming across all y
     */
    Derived &
    ysubnames(const char **names)
    {
        Derived &self = this->self();
        Info *info = this->info();

        info->y_subnames.resize(self.y);
        for (off_type i = 0; i < self.y; ++i)
            info->y_subnames[i] = names[i];
        return self;
    }

    Derived &
    ysubname(off_type index, const std::string &subname)
    {
        Derived &self = this->self();
        Info *info = this->info();

        assert(index < self.y);
        info->y_subnames.resize(self.y);
        info->y_subnames[index] = subname.c_str();
        return self;
    }

    std::string
    ysubname(off_type i) const
    {
        return this->info()->y_subnames[i];
    }

};

//////////////////////////////////////////////////////////////////////
//
// Simple Statistics
//
//////////////////////////////////////////////////////////////////////

/**
 * Templatized storage and interface for a simple scalar stat.
 */
class StatStor
{
  private:
    /** The statistic value. */
    Counter data;

  public:
    struct Params : public StorageParams {};

  public:
    /**
     * Builds this storage element and calls the base constructor of the
     * datatype.
     */
    StatStor(Info *info)
        : data(Counter())
    { }

    /**
     * The the stat to the given value.
     * @param val The new value.
     */
    void set(Counter val) { data = val; }
    /**
     * Increment the stat by the given value.
     * @param val The new value.
     */
    void inc(Counter val) { data += val; }
    /**
     * Decrement the stat by the given value.
     * @param val The new value.
     */
    void dec(Counter val) { data -= val; }
    /**
     * Return the value of this stat as its base type.
     * @return The value of this stat.
     */
    Counter value() const { return data; }
    /**
     * Return the value of this stat as a result type.
     * @return The value of this stat.
     */
    Result result() const { return (Result)data; }
    /**
     * Prepare stat data for dumping or serialization
     */
    void prepare(Info *info) { }
    /**
     * Reset stat value to default
     */
    void reset(Info *info) { data = Counter(); }

    /**
     * @return true if zero value
     */
    bool zero() const { return data == Counter(); }
};

/**
 * Templatized storage and interface to a per-tick average stat. This keeps
 * a current count and updates a total (count * ticks) when this count
 * changes. This allows the quick calculation of a per tick count of the item
 * being watched. This is good for keeping track of residencies in structures
 * among other things.
 */
class AvgStor
{
  private:
    /** The current count. */
    Counter current;
    /** The tick of the last reset */
    Tick lastReset;
    /** The total count for all tick. */
    mutable Result total;
    /** The tick that current last changed. */
    mutable Tick last;

  public:
    struct Params : public StorageParams {};

  public:
    /**
     * Build and initializes this stat storage.
     */
    AvgStor(Info *info)
        : current(0), lastReset(0), total(0), last(0)
    { }

    /**
     * Set the current count to the one provided, update the total and last
     * set values.
     * @param val The new count.
     */
    void
    set(Counter val)
    {
        total += current * (curTick() - last);
        last = curTick();
        current = val;
    }

    /**
     * Increment the current count by the provided value, calls set.
     * @param val The amount to increment.
     */
    void inc(Counter val) { set(current + val); }

    /**
     * Deccrement the current count by the provided value, calls set.
     * @param val The amount to decrement.
     */
    void dec(Counter val) { set(current - val); }

    /**
     * Return the current count.
     * @return The current count.
     */
    Counter value() const { return current; }

    /**
     * Return the current average.
     * @return The current average.
     */
    Result
    result() const
    {
        assert(last == curTick());
        return (Result)(total + current) / (Result)(curTick() - lastReset + 1);
    }

    /**
     * @return true if zero value
     */
    bool zero() const { return total == 0.0; }

    /**
     * Prepare stat data for dumping or serialization
     */
    void
    prepare(Info *info)
    {
        total += current * (curTick() - last);
        last = curTick();
    }

    /**
     * Reset stat value to default
     */
    void
    reset(Info *info)
    {
        total = 0.0;
        last = curTick();
        lastReset = curTick();
    }

};

/**
 * Implementation of a scalar stat. The type of stat is determined by the
 * Storage template.
 */
template <class Derived, class Stor>
class ScalarBase : public DataWrap<Derived, ScalarInfoProxy>
{
  public:
    typedef Stor Storage;
    typedef typename Stor::Params Params;

  protected:
    /** The storage of this stat. */
    char storage[sizeof(Storage)] __attribute__ ((aligned (8)));

  protected:
    /**
     * Retrieve the storage.
     * @param index The vector index to access.
     * @return The storage object at the given index.
     */
    Storage *
    data()
    {
        return reinterpret_cast<Storage *>(storage);
    }

    /**
     * Retrieve a const pointer to the storage.
     * for the given index.
     * @param index The vector index to access.
     * @return A const pointer to the storage object at the given index.
     */
    const Storage *
    data() const
    {
        return reinterpret_cast<const Storage *>(storage);
    }

    void
    doInit()
    {
        new (storage) Storage(this->info());
        this->setInit();
    }

  public:
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    Counter value() const { return data()->value(); }

  public:
    ScalarBase()
    {
        this->doInit();
    }

  public:
    // Common operators for stats
    /**
     * Increment the stat by 1. This calls the associated storage object inc
     * function.
     */
    void operator++() { data()->inc(1); }
    /**
     * Decrement the stat by 1. This calls the associated storage object dec
     * function.
     */
    void operator--() { data()->dec(1); }

    /** Increment the stat by 1. */
    void operator++(int) { ++*this; }
    /** Decrement the stat by 1. */
    void operator--(int) { --*this; }

    /**
     * Set the data value to the given value. This calls the associated storage
     * object set function.
     * @param v The new value.
     */
    template <typename U>
    void operator=(const U &v) { data()->set(v); }

    /**
     * Increment the stat by the given value. This calls the associated
     * storage object inc function.
     * @param v The value to add.
     */
    template <typename U>
    void operator+=(const U &v) { data()->inc(v); }

    /**
     * Decrement the stat by the given value. This calls the associated
     * storage object dec function.
     * @param v The value to substract.
     */
    template <typename U>
    void operator-=(const U &v) { data()->dec(v); }

    /**
     * Return the number of elements, always 1 for a scalar.
     * @return 1.
     */
    size_type size() const { return 1; }

    Counter value() { return data()->value(); }

    Result result() { return data()->result(); }

    Result total() { return result(); }

    bool zero() { return result() == 0.0; }

    void reset() { data()->reset(this->info()); }
    void prepare() { data()->prepare(this->info()); }
};

class ProxyInfo : public ScalarInfo
{
  public:
    std::string str() const { return std::to_string(value()); }
    size_type size() const { return 1; }
    bool check() const { return true; }
    void prepare() { }
    void reset() { }
    bool zero() const { return value() == 0; }

    void visit(Output &visitor) { visitor.visit(*this); }
};

template <class T>
class ValueProxy : public ProxyInfo
{
  private:
    T *scalar;

  public:
    ValueProxy(T &val) : scalar(&val) {}
    Counter value() const { return *scalar; }
    Result result() const { return *scalar; }
    Result total() const { return *scalar; }
};

template <class T>
class FunctorProxy : public ProxyInfo
{
  private:
    T *functor;

  public:
    FunctorProxy(T &func) : functor(&func) {}
    Counter value() const { return (*functor)(); }
    Result result() const { return (*functor)(); }
    Result total() const { return (*functor)(); }
};

/**
 * A proxy similar to the FunctorProxy, but allows calling a method of a bound
 * object, instead of a global free-standing function.
 */
template <class T, class V>
class MethodProxy : public ProxyInfo
{
  private:
    T *object;
    typedef V (T::*MethodPointer) () const;
    MethodPointer method;

  public:
    MethodProxy(T *obj, MethodPointer meth) : object(obj), method(meth) {}
    Counter value() const { return (object->*method)(); }
    Result result() const { return (object->*method)(); }
    Result total() const { return (object->*method)(); }
};

template <class Derived>
class ValueBase : public DataWrap<Derived, ScalarInfoProxy>
{
  private:
    ProxyInfo *proxy;

  public:
    ValueBase() : proxy(NULL) { }
    ~ValueBase() { if (proxy) delete proxy; }

    template <class T>
    Derived &
    scalar(T &value)
    {
        proxy = new ValueProxy<T>(value);
        this->setInit();
        return this->self();
    }

    template <class T>
    Derived &
    functor(T &func)
    {
        proxy = new FunctorProxy<T>(func);
        this->setInit();
        return this->self();
    }

    /**
     * Extended functor that calls the specified method of the provided object.
     *
     * @param obj Pointer to the object whose method should be called.
     * @param method Pointer of the function / method of the object.
     * @return Updated stats item.
     */
    template <class T, class V>
    Derived &
    method(T *obj,  V (T::*method)() const)
    {
        proxy = new MethodProxy<T,V>(obj, method);
        this->setInit();
        return this->self();
    }

    Counter value() { return proxy->value(); }
    Result result() const { return proxy->result(); }
    Result total() const { return proxy->total(); };
    size_type size() const { return proxy->size(); }

    std::string str() const { return proxy->str(); }
    bool zero() const { return proxy->zero(); }
    bool check() const { return proxy != NULL; }
    void prepare() { }
    void reset() { }
};

//////////////////////////////////////////////////////////////////////
//
// Vector Statistics
//
//////////////////////////////////////////////////////////////////////

/**
 * A proxy class to access the stat at a given index in a VectorBase stat.
 * Behaves like a ScalarBase.
 */
template <class Stat>
class ScalarProxy
{
  private:
    /** Pointer to the parent Vector. */
    Stat &stat;

    /** The index to access in the parent VectorBase. */
    off_type index;

  public:
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    Counter value() const { return stat.data(index)->value(); }

    /**
     * Return the current value of this statas a result type.
     * @return The current value.
     */
    Result result() const { return stat.data(index)->result(); }

  public:
    /**
     * Create and initialize this proxy, do not register it with the database.
     * @param i The index to access.
     */
    ScalarProxy(Stat &s, off_type i)
        : stat(s), index(i)
    {
    }

    /**
     * Create a copy of the provided ScalarProxy.
     * @param sp The proxy to copy.
     */
    ScalarProxy(const ScalarProxy &sp)
        : stat(sp.stat), index(sp.index)
    {}

    /**
     * Set this proxy equal to the provided one.
     * @param sp The proxy to copy.
     * @return A reference to this proxy.
     */
    const ScalarProxy &
    operator=(const ScalarProxy &sp)
    {
        stat = sp.stat;
        index = sp.index;
        return *this;
    }

  public:
    // Common operators for stats
    /**
     * Increment the stat by 1. This calls the associated storage object inc
     * function.
     */
    void operator++() { stat.data(index)->inc(1); }
    /**
     * Decrement the stat by 1. This calls the associated storage object dec
     * function.
     */
    void operator--() { stat.data(index)->dec(1); }

    /** Increment the stat by 1. */
    void operator++(int) { ++*this; }
    /** Decrement the stat by 1. */
    void operator--(int) { --*this; }

    /**
     * Set the data value to the given value. This calls the associated storage
     * object set function.
     * @param v The new value.
     */
    template <typename U>
    void
    operator=(const U &v)
    {
        stat.data(index)->set(v);
    }

    /**
     * Increment the stat by the given value. This calls the associated
     * storage object inc function.
     * @param v The value to add.
     */
    template <typename U>
    void
    operator+=(const U &v)
    {
        stat.data(index)->inc(v);
    }

    /**
     * Decrement the stat by the given value. This calls the associated
     * storage object dec function.
     * @param v The value to substract.
     */
    template <typename U>
    void
    operator-=(const U &v)
    {
        stat.data(index)->dec(v);
    }

    /**
     * Return the number of elements, always 1 for a scalar.
     * @return 1.
     */
    size_type size() const { return 1; }

  public:
    std::string
    str() const
    {
        return csprintf("%s[%d]", stat.info()->name, index);
    }
};

/**
 * Implementation of a vector of stats. The type of stat is determined by the
 * Storage class. @sa ScalarBase
 */
template <class Derived, class Stor>
class VectorBase : public DataWrapVec<Derived, VectorInfoProxy>
{
  public:
    typedef Stor Storage;
    typedef typename Stor::Params Params;

    /** Proxy type */
    typedef ScalarProxy<Derived> Proxy;
    friend class ScalarProxy<Derived>;
    friend class DataWrapVec<Derived, VectorInfoProxy>;

  protected:
    /** The storage of this stat. */
    Storage *storage;
    size_type _size;

  protected:
    /**
     * Retrieve the storage.
     * @param index The vector index to access.
     * @return The storage object at the given index.
     */
    Storage *data(off_type index) { return &storage[index]; }

    /**
     * Retrieve a const pointer to the storage.
     * @param index The vector index to access.
     * @return A const pointer to the storage object at the given index.
     */
    const Storage *data(off_type index) const { return &storage[index]; }

    void
    doInit(size_type s)
    {
        assert(s > 0 && "size must be positive!");
        assert(!storage && "already initialized");
        _size = s;

        char *ptr = new char[_size * sizeof(Storage)];
        storage = reinterpret_cast<Storage *>(ptr);

        for (off_type i = 0; i < _size; ++i)
            new (&storage[i]) Storage(this->info());

        this->setInit();
    }

  public:
    void
    value(VCounter &vec) const
    {
        vec.resize(size());
        for (off_type i = 0; i < size(); ++i)
            vec[i] = data(i)->value();
    }

    /**
     * Copy the values to a local vector and return a reference to it.
     * @return A reference to a vector of the stat values.
     */
    void
    result(VResult &vec) const
    {
        vec.resize(size());
        for (off_type i = 0; i < size(); ++i)
            vec[i] = data(i)->result();
    }

    /**
     * Return a total of all entries in this vector.
     * @return The total of all vector entries.
     */
    Result
    total() const
    {
        Result total = 0.0;
        for (off_type i = 0; i < size(); ++i)
            total += data(i)->result();
        return total;
    }

    /**
     * @return the number of elements in this vector.
     */
    size_type size() const { return _size; }

    bool
    zero() const
    {
        for (off_type i = 0; i < size(); ++i)
            if (data(i)->zero())
                return false;
        return true;
    }

    bool
    check() const
    {
        return storage != NULL;
    }

  public:
    VectorBase()
        : storage(nullptr), _size(0)
    {}

    ~VectorBase()
    {
        if (!storage)
            return;

        for (off_type i = 0; i < _size; ++i)
            data(i)->~Storage();
        delete [] reinterpret_cast<char *>(storage);
    }

    /**
     * Set this vector to have the given size.
     * @param size The new size.
     * @return A reference to this stat.
     */
    Derived &
    init(size_type size)
    {
        Derived &self = this->self();
        self.doInit(size);
        return self;
    }

    /**
     * Return a reference (ScalarProxy) to the stat at the given index.
     * @param index The vector index to access.
     * @return A reference of the stat.
     */
    Proxy
    operator[](off_type index)
    {
        assert (index >= 0 && index < size());
        return Proxy(this->self(), index);
    }
};

template <class Stat>
class VectorProxy
{
  private:
    Stat &stat;
    off_type offset;
    size_type len;

  private:
    mutable VResult vec;

    typename Stat::Storage *
    data(off_type index)
    {
        assert(index < len);
        return stat.data(offset + index);
    }

    const typename Stat::Storage *
    data(off_type index) const
    {
        assert(index < len);
        return stat.data(offset + index);
    }

  public:
    const VResult &
    result() const
    {
        vec.resize(size());

        for (off_type i = 0; i < size(); ++i)
            vec[i] = data(i)->result();

        return vec;
    }

    Result
    total() const
    {
        Result total = 0.0;
        for (off_type i = 0; i < size(); ++i)
            total += data(i)->result();
        return total;
    }

  public:
    VectorProxy(Stat &s, off_type o, size_type l)
        : stat(s), offset(o), len(l)
    {
    }

    VectorProxy(const VectorProxy &sp)
        : stat(sp.stat), offset(sp.offset), len(sp.len)
    {
    }

    const VectorProxy &
    operator=(const VectorProxy &sp)
    {
        stat = sp.stat;
        offset = sp.offset;
        len = sp.len;
        return *this;
    }

    ScalarProxy<Stat>
    operator[](off_type index)
    {
        assert (index >= 0 && index < size());
        return ScalarProxy<Stat>(stat, offset + index);
    }

    size_type size() const { return len; }
};

template <class Derived, class Stor>
class Vector2dBase : public DataWrapVec2d<Derived, Vector2dInfoProxy>
{
  public:
    typedef Vector2dInfoProxy<Derived> Info;
    typedef Stor Storage;
    typedef typename Stor::Params Params;
    typedef VectorProxy<Derived> Proxy;
    friend class ScalarProxy<Derived>;
    friend class VectorProxy<Derived>;
    friend class DataWrapVec<Derived, Vector2dInfoProxy>;
    friend class DataWrapVec2d<Derived, Vector2dInfoProxy>;

  protected:
    size_type x;
    size_type y;
    size_type _size;
    Storage *storage;

  protected:
    Storage *data(off_type index) { return &storage[index]; }
    const Storage *data(off_type index) const { return &storage[index]; }

  public:
    Vector2dBase()
        : x(0), y(0), _size(0), storage(nullptr)
    {}

    ~Vector2dBase()
    {
        if (!storage)
            return;

        for (off_type i = 0; i < _size; ++i)
            data(i)->~Storage();
        delete [] reinterpret_cast<char *>(storage);
    }

    Derived &
    init(size_type _x, size_type _y)
    {
        assert(_x > 0 && _y > 0 && "sizes must be positive!");
        assert(!storage && "already initialized");

        Derived &self = this->self();
        Info *info = this->info();

        x = _x;
        y = _y;
        info->x = _x;
        info->y = _y;
        _size = x * y;

        char *ptr = new char[_size * sizeof(Storage)];
        storage = reinterpret_cast<Storage *>(ptr);

        for (off_type i = 0; i < _size; ++i)
            new (&storage[i]) Storage(info);

        this->setInit();

        return self;
    }

    Proxy
    operator[](off_type index)
    {
        off_type offset = index * y;
        assert (index >= 0 && offset + y <= size());
        return Proxy(this->self(), offset, y);
    }


    size_type
    size() const
    {
        return _size;
    }

    bool
    zero() const
    {
        return data(0)->zero();
#if 0
        for (off_type i = 0; i < size(); ++i)
            if (!data(i)->zero())
                return false;
        return true;
#endif
    }

    /**
     * Return a total of all entries in this vector.
     * @return The total of all vector entries.
     */
    Result
    total() const
    {
        Result total = 0.0;
        for (off_type i = 0; i < size(); ++i)
            total += data(i)->result();
        return total;
    }

    void
    prepare()
    {
        Info *info = this->info();
        size_type size = this->size();

        for (off_type i = 0; i < size; ++i)
            data(i)->prepare(info);

        info->cvec.resize(size);
        for (off_type i = 0; i < size; ++i)
            info->cvec[i] = data(i)->value();
    }

    /**
     * Reset stat value to default
     */
    void
    reset()
    {
        Info *info = this->info();
        size_type size = this->size();
        for (off_type i = 0; i < size; ++i)
            data(i)->reset(info);
    }

    bool
    check() const
    {
        return storage != NULL;
    }
};

//////////////////////////////////////////////////////////////////////
//
// Non formula statistics
//
//////////////////////////////////////////////////////////////////////
/** The parameters for a distribution stat. */
struct DistParams : public StorageParams
{
    const DistType type;
    DistParams(DistType t) : type(t) {}
};

/**
 * Templatized storage and interface for a distribution stat.
 */
class DistStor
{
  public:
    /** The parameters for a distribution stat. */
    struct Params : public DistParams
    {
        /** The minimum value to track. */
        Counter min;
        /** The maximum value to track. */
        Counter max;
        /** The number of entries in each bucket. */
        Counter bucket_size;
        /** The number of buckets. Equal to (max-min)/bucket_size. */
        size_type buckets;

        Params() : DistParams(Dist), min(0), max(0), bucket_size(0),
                   buckets(0) {}
    };

  private:
    /** The minimum value to track. */
    Counter min_track;
    /** The maximum value to track. */
    Counter max_track;
    /** The number of entries in each bucket. */
    Counter bucket_size;

    /** The smallest value sampled. */
    Counter min_val;
    /** The largest value sampled. */
    Counter max_val;
    /** The number of values sampled less than min. */
    Counter underflow;
    /** The number of values sampled more than max. */
    Counter overflow;
    /** The current sum. */
    Counter sum;
    /** The sum of squares. */
    Counter squares;
    /** The number of samples. */
    Counter samples;
    /** Counter for each bucket. */
    VCounter cvec;

  public:
    DistStor(Info *info)
        : cvec(safe_cast<const Params *>(info->storageParams)->buckets)
    {
        reset(info);
    }

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void
    sample(Counter val, int number)
    {
        if (val < min_track)
            underflow += number;
        else if (val > max_track)
            overflow += number;
        else {
            size_type index =
                (size_type)std::floor((val - min_track) / bucket_size);
            assert(index < size());
            cvec[index] += number;
        }

        if (val < min_val)
            min_val = val;

        if (val > max_val)
            max_val = val;

        sum += val * number;
        squares += val * val * number;
        samples += number;
    }

    /**
     * Return the number of buckets in this distribution.
     * @return the number of buckets.
     */
    size_type size() const { return cvec.size(); }

    /**
     * Returns true if any calls to sample have been made.
     * @return True if any values have been sampled.
     */
    bool
    zero() const
    {
        return samples == Counter();
    }

    void
    prepare(Info *info, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(info->storageParams);

        assert(params->type == Dist);
        data.type = params->type;
        data.min = params->min;
        data.max = params->max;
        data.bucket_size = params->bucket_size;

        data.min_val = (min_val == CounterLimits::max()) ? 0 : min_val;
        data.max_val = (max_val == CounterLimits::min()) ? 0 : max_val;
        data.underflow = underflow;
        data.overflow = overflow;

        data.cvec.resize(params->buckets);
        for (off_type i = 0; i < params->buckets; ++i)
            data.cvec[i] = cvec[i];

        data.sum = sum;
        data.squares = squares;
        data.samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void
    reset(Info *info)
    {
        const Params *params = safe_cast<const Params *>(info->storageParams);
        min_track = params->min;
        max_track = params->max;
        bucket_size = params->bucket_size;

        min_val = CounterLimits::max();
        max_val = CounterLimits::min();
        underflow = Counter();
        overflow = Counter();

        size_type size = cvec.size();
        for (off_type i = 0; i < size; ++i)
            cvec[i] = Counter();

        sum = Counter();
        squares = Counter();
        samples = Counter();
    }
};

/**
 * Templatized storage and interface for a histogram stat.
 */
class HistStor
{
  public:
    /** The parameters for a distribution stat. */
    struct Params : public DistParams
    {
        /** The number of buckets.. */
        size_type buckets;

        Params() : DistParams(Hist), buckets(0) {}
    };

  private:
    /** The minimum value to track. */
    Counter min_bucket;
    /** The maximum value to track. */
    Counter max_bucket;
    /** The number of entries in each bucket. */
    Counter bucket_size;

    /** The current sum. */
    Counter sum;
    /** The sum of logarithm of each sample, used to compute geometric mean. */
    Counter logs;
    /** The sum of squares. */
    Counter squares;
    /** The number of samples. */
    Counter samples;
    /** Counter for each bucket. */
    VCounter cvec;

  public:
    HistStor(Info *info)
        : cvec(safe_cast<const Params *>(info->storageParams)->buckets)
    {
        reset(info);
    }

    void grow_up();
    void grow_out();
    void grow_convert();
    void add(HistStor *);

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void
    sample(Counter val, int number)
    {
        assert(min_bucket < max_bucket);
        if (val < min_bucket) {
            if (min_bucket == 0)
                grow_convert();

            while (val < min_bucket)
                grow_out();
        } else if (val >= max_bucket + bucket_size) {
            if (min_bucket == 0) {
                while (val >= max_bucket + bucket_size)
                    grow_up();
            } else {
                while (val >= max_bucket + bucket_size)
                    grow_out();
            }
        }

        size_type index =
            (int64_t)std::floor((val - min_bucket) / bucket_size);

        assert(index < size());
        cvec[index] += number;

        sum += val * number;
        squares += val * val * number;
        logs += log(val) * number;
        samples += number;
    }

    /**
     * Return the number of buckets in this distribution.
     * @return the number of buckets.
     */
    size_type size() const { return cvec.size(); }

    /**
     * Returns true if any calls to sample have been made.
     * @return True if any values have been sampled.
     */
    bool
    zero() const
    {
        return samples == Counter();
    }

    void
    prepare(Info *info, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(info->storageParams);

        assert(params->type == Hist);
        data.type = params->type;
        data.min = min_bucket;
        data.max = max_bucket + bucket_size - 1;
        data.bucket_size = bucket_size;

        data.min_val = min_bucket;
        data.max_val = max_bucket;

        int buckets = params->buckets;
        data.cvec.resize(buckets);
        for (off_type i = 0; i < buckets; ++i)
            data.cvec[i] = cvec[i];

        data.sum = sum;
        data.logs = logs;
        data.squares = squares;
        data.samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void
    reset(Info *info)
    {
        const Params *params = safe_cast<const Params *>(info->storageParams);
        min_bucket = 0;
        max_bucket = params->buckets - 1;
        bucket_size = 1;

        size_type size = cvec.size();
        for (off_type i = 0; i < size; ++i)
            cvec[i] = Counter();

        sum = Counter();
        squares = Counter();
        samples = Counter();
        logs = Counter();
    }
};

/**
 * Templatized storage and interface for a distribution that calculates mean
 * and variance.
 */
class SampleStor
{
  public:
    struct Params : public DistParams
    {
        Params() : DistParams(Deviation) {}
    };

  private:
    /** The current sum. */
    Counter sum;
    /** The sum of squares. */
    Counter squares;
    /** The number of samples. */
    Counter samples;

  public:
    /**
     * Create and initialize this storage.
     */
    SampleStor(Info *info)
        : sum(Counter()), squares(Counter()), samples(Counter())
    { }

    /**
     * Add a value the given number of times to this running average.
     * Update the running sum and sum of squares, increment the number of
     * values seen by the given number.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void
    sample(Counter val, int number)
    {
        Counter value = val * number;
        sum += value;
        squares += value * value;
        samples += number;
    }

    /**
     * Return the number of entries in this stat, 1
     * @return 1.
     */
    size_type size() const { return 1; }

    /**
     * Return true if no samples have been added.
     * @return True if no samples have been added.
     */
    bool zero() const { return samples == Counter(); }

    void
    prepare(Info *info, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(info->storageParams);

        assert(params->type == Deviation);
        data.type = params->type;
        data.sum = sum;
        data.squares = squares;
        data.samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void
    reset(Info *info)
    {
        sum = Counter();
        squares = Counter();
        samples = Counter();
    }
};

/**
 * Templatized storage for distribution that calculates per tick mean and
 * variance.
 */
class AvgSampleStor
{
  public:
    struct Params : public DistParams
    {
        Params() : DistParams(Deviation) {}
    };

  private:
    /** Current total. */
    Counter sum;
    /** Current sum of squares. */
    Counter squares;

  public:
    /**
     * Create and initialize this storage.
     */
    AvgSampleStor(Info *info)
        : sum(Counter()), squares(Counter())
    {}

    /**
     * Add a value to the distribution for the given number of times.
     * Update the running sum and sum of squares.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void
    sample(Counter val, int number)
    {
        Counter value = val * number;
        sum += value;
        squares += value * value;
    }

    /**
     * Return the number of entries, in this case 1.
     * @return 1.
     */
    size_type size() const { return 1; }

    /**
     * Return true if no samples have been added.
     * @return True if the sum is zero.
     */
    bool zero() const { return sum == Counter(); }

    void
    prepare(Info *info, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(info->storageParams);

        assert(params->type == Deviation);
        data.type = params->type;
        data.sum = sum;
        data.squares = squares;
        data.samples = curTick();
    }

    /**
     * Reset stat value to default
     */
    void
    reset(Info *info)
    {
        sum = Counter();
        squares = Counter();
    }
};

/**
 * Implementation of a distribution stat. The type of distribution is
 * determined by the Storage template. @sa ScalarBase
 */
template <class Derived, class Stor>
class DistBase : public DataWrap<Derived, DistInfoProxy>
{
  public:
    typedef DistInfoProxy<Derived> Info;
    typedef Stor Storage;
    typedef typename Stor::Params Params;

  protected:
    /** The storage for this stat. */
    char storage[sizeof(Storage)] __attribute__ ((aligned (8)));

  protected:
    /**
     * Retrieve the storage.
     * @return The storage object for this stat.
     */
    Storage *
    data()
    {
        return reinterpret_cast<Storage *>(storage);
    }

    /**
     * Retrieve a const pointer to the storage.
     * @return A const pointer to the storage object for this stat.
     */
    const Storage *
    data() const
    {
        return reinterpret_cast<const Storage *>(storage);
    }

    void
    doInit()
    {
        new (storage) Storage(this->info());
        this->setInit();
    }

  public:
    DistBase() { }

    /**
     * Add a value to the distribtion n times. Calls sample on the storage
     * class.
     * @param v The value to add.
     * @param n The number of times to add it, defaults to 1.
     */
    template <typename U>
    void sample(const U &v, int n = 1) { data()->sample(v, n); }

    /**
     * Return the number of entries in this stat.
     * @return The number of entries.
     */
    size_type size() const { return data()->size(); }
    /**
     * Return true if no samples have been added.
     * @return True if there haven't been any samples.
     */
    bool zero() const { return data()->zero(); }

    void
    prepare()
    {
        Info *info = this->info();
        data()->prepare(info, info->data);
    }

    /**
     * Reset stat value to default
     */
    void
    reset()
    {
        data()->reset(this->info());
    }

    /**
     *  Add the argument distribution to the this distribution.
     */
    void add(DistBase &d) { data()->add(d.data()); }

};

template <class Stat>
class DistProxy;

template <class Derived, class Stor>
class VectorDistBase : public DataWrapVec<Derived, VectorDistInfoProxy>
{
  public:
    typedef VectorDistInfoProxy<Derived> Info;
    typedef Stor Storage;
    typedef typename Stor::Params Params;
    typedef DistProxy<Derived> Proxy;
    friend class DistProxy<Derived>;
    friend class DataWrapVec<Derived, VectorDistInfoProxy>;

  protected:
    Storage *storage;
    size_type _size;

  protected:
    Storage *
    data(off_type index)
    {
        return &storage[index];
    }

    const Storage *
    data(off_type index) const
    {
        return &storage[index];
    }

    void
    doInit(size_type s)
    {
        assert(s > 0 && "size must be positive!");
        assert(!storage && "already initialized");
        _size = s;

        char *ptr = new char[_size * sizeof(Storage)];
        storage = reinterpret_cast<Storage *>(ptr);

        Info *info = this->info();
        for (off_type i = 0; i < _size; ++i)
            new (&storage[i]) Storage(info);

        this->setInit();
    }

  public:
    VectorDistBase()
        : storage(NULL)
    {}

    ~VectorDistBase()
    {
        if (!storage)
            return ;

        for (off_type i = 0; i < _size; ++i)
            data(i)->~Storage();
        delete [] reinterpret_cast<char *>(storage);
    }

    Proxy operator[](off_type index)
    {
        assert(index >= 0 && index < size());
        return Proxy(this->self(), index);
    }

    size_type
    size() const
    {
        return _size;
    }

    bool
    zero() const
    {
        for (off_type i = 0; i < size(); ++i)
            if (!data(i)->zero())
                return false;
        return true;
    }

    void
    prepare()
    {
        Info *info = this->info();
        size_type size = this->size();
        info->data.resize(size);
        for (off_type i = 0; i < size; ++i)
            data(i)->prepare(info, info->data[i]);
    }

    bool
    check() const
    {
        return storage != NULL;
    }
};

template <class Stat>
class DistProxy
{
  private:
    Stat &stat;
    off_type index;

  protected:
    typename Stat::Storage *data() { return stat.data(index); }
    const typename Stat::Storage *data() const { return stat.data(index); }

  public:
    DistProxy(Stat &s, off_type i)
        : stat(s), index(i)
    {}

    DistProxy(const DistProxy &sp)
        : stat(sp.stat), index(sp.index)
    {}

    const DistProxy &
    operator=(const DistProxy &sp)
    {
        stat = sp.stat;
        index = sp.index;
        return *this;
    }

  public:
    template <typename U>
    void
    sample(const U &v, int n = 1)
    {
        data()->sample(v, n);
    }

    size_type
    size() const
    {
        return 1;
    }

    bool
    zero() const
    {
        return data()->zero();
    }

    /**
     * Proxy has no state.  Nothing to reset.
     */
    void reset() { }
};

//////////////////////////////////////////////////////////////////////
//
//  Formula Details
//
//////////////////////////////////////////////////////////////////////

/**
 * Base class for formula statistic node. These nodes are used to build a tree
 * that represents the formula.
 */
class Node
{
  public:
    /**
     * Return the number of nodes in the subtree starting at this node.
     * @return the number of nodes in this subtree.
     */
    virtual size_type size() const = 0;
    /**
     * Return the result vector of this subtree.
     * @return The result vector of this subtree.
     */
    virtual const VResult &result() const = 0;
    /**
     * Return the total of the result vector.
     * @return The total of the result vector.
     */
    virtual Result total() const = 0;

    /**
     *
     */
    virtual std::string str() const = 0;

    virtual ~Node() {};
};

/** Shared pointer to a function Node. */
typedef std::shared_ptr<Node> NodePtr;

class ScalarStatNode : public Node
{
  private:
    const ScalarInfo *data;
    mutable VResult vresult;

  public:
    ScalarStatNode(const ScalarInfo *d) : data(d), vresult(1) {}

    const VResult &
    result() const
    {
        vresult[0] = data->result();
        return vresult;
    }

    Result total() const { return data->result(); };

    size_type size() const { return 1; }

    /**
     *
     */
    std::string str() const { return data->name; }
};

template <class Stat>
class ScalarProxyNode : public Node
{
  private:
    const ScalarProxy<Stat> proxy;
    mutable VResult vresult;

  public:
    ScalarProxyNode(const ScalarProxy<Stat> &p)
        : proxy(p), vresult(1)
    { }

    const VResult &
    result() const
    {
        vresult[0] = proxy.result();
        return vresult;
    }

    Result
    total() const
    {
        return proxy.result();
    }

    size_type
    size() const
    {
        return 1;
    }

    /**
     *
     */
    std::string
    str() const
    {
        return proxy.str();
    }
};

class VectorStatNode : public Node
{
  private:
    const VectorInfo *data;

  public:
    VectorStatNode(const VectorInfo *d) : data(d) { }
    const VResult &result() const { return data->result(); }
    Result total() const { return data->total(); };

    size_type size() const { return data->size(); }

    std::string str() const { return data->name; }
};

template <class T>
class ConstNode : public Node
{
  private:
    VResult vresult;

  public:
    ConstNode(T s) : vresult(1, (Result)s) {}
    const VResult &result() const { return vresult; }
    Result total() const { return vresult[0]; };
    size_type size() const { return 1; }
    std::string str() const { return std::to_string(vresult[0]); }
};

template <class T>
class ConstVectorNode : public Node
{
  private:
    VResult vresult;

  public:
    ConstVectorNode(const T &s) : vresult(s.begin(), s.end()) {}
    const VResult &result() const { return vresult; }

    Result
    total() const
    {
        size_type size = this->size();
        Result tmp = 0;
        for (off_type i = 0; i < size; i++)
            tmp += vresult[i];
        return tmp;
    }

    size_type size() const { return vresult.size(); }
    std::string
    str() const
    {
        size_type size = this->size();
        std::string tmp = "(";
        for (off_type i = 0; i < size; i++)
            tmp += csprintf("%s ", std::to_string(vresult[i]));
        tmp += ")";
        return tmp;
    }
};

template <class Op>
struct OpString;

template<>
struct OpString<std::plus<Result> >
{
    static std::string str() { return "+"; }
};

template<>
struct OpString<std::minus<Result> >
{
    static std::string str() { return "-"; }
};

template<>
struct OpString<std::multiplies<Result> >
{
    static std::string str() { return "*"; }
};

template<>
struct OpString<std::divides<Result> >
{
    static std::string str() { return "/"; }
};

template<>
struct OpString<std::modulus<Result> >
{
    static std::string str() { return "%"; }
};

template<>
struct OpString<std::negate<Result> >
{
    static std::string str() { return "-"; }
};

template <class Op>
class UnaryNode : public Node
{
  public:
    NodePtr l;
    mutable VResult vresult;

  public:
    UnaryNode(NodePtr &p) : l(p) {}

    const VResult &
    result() const
    {
        const VResult &lvec = l->result();
        size_type size = lvec.size();

        assert(size > 0);

        vresult.resize(size);
        Op op;
        for (off_type i = 0; i < size; ++i)
            vresult[i] = op(lvec[i]);

        return vresult;
    }

    Result
    total() const
    {
        const VResult &vec = this->result();
        Result total = 0.0;
        for (off_type i = 0; i < size(); i++)
            total += vec[i];
        return total;
    }

    size_type size() const { return l->size(); }

    std::string
    str() const
    {
        return OpString<Op>::str() + l->str();
    }
};

template <class Op>
class BinaryNode : public Node
{
  public:
    NodePtr l;
    NodePtr r;
    mutable VResult vresult;

  public:
    BinaryNode(NodePtr &a, NodePtr &b) : l(a), r(b) {}

    const VResult &
    result() const override
    {
        Op op;
        const VResult &lvec = l->result();
        const VResult &rvec = r->result();

        assert(lvec.size() > 0 && rvec.size() > 0);

        if (lvec.size() == 1 && rvec.size() == 1) {
            vresult.resize(1);
            vresult[0] = op(lvec[0], rvec[0]);
        } else if (lvec.size() == 1) {
            size_type size = rvec.size();
            vresult.resize(size);
            for (off_type i = 0; i < size; ++i)
                vresult[i] = op(lvec[0], rvec[i]);
        } else if (rvec.size() == 1) {
            size_type size = lvec.size();
            vresult.resize(size);
            for (off_type i = 0; i < size; ++i)
                vresult[i] = op(lvec[i], rvec[0]);
        } else if (rvec.size() == lvec.size()) {
            size_type size = rvec.size();
            vresult.resize(size);
            for (off_type i = 0; i < size; ++i)
                vresult[i] = op(lvec[i], rvec[i]);
        }

        return vresult;
    }

    Result
    total() const override
    {
        const VResult &vec = this->result();
        const VResult &lvec = l->result();
        const VResult &rvec = r->result();
        Result total = 0.0;
        Result lsum = 0.0;
        Result rsum = 0.0;
        Op op;

        assert(lvec.size() > 0 && rvec.size() > 0);
        assert(lvec.size() == rvec.size() ||
               lvec.size() == 1 || rvec.size() == 1);

        /** If vectors are the same divide their sums (x0+x1)/(y0+y1) */
        if (lvec.size() == rvec.size() && lvec.size() > 1) {
            for (off_type i = 0; i < size(); ++i) {
                lsum += lvec[i];
                rsum += rvec[i];
            }
            return op(lsum, rsum);
        }

        /** Otherwise divide each item by the divisor */
        for (off_type i = 0; i < size(); ++i) {
            total += vec[i];
        }

        return total;
    }

    size_type
    size() const override
    {
        size_type ls = l->size();
        size_type rs = r->size();
        if (ls == 1) {
            return rs;
        } else if (rs == 1) {
            return ls;
        } else {
            assert(ls == rs && "Node vector sizes are not equal");
            return ls;
        }
    }

    std::string
    str() const override
    {
        return csprintf("(%s %s %s)", l->str(), OpString<Op>::str(), r->str());
    }
};

template <class Op>
class SumNode : public Node
{
  public:
    NodePtr l;
    mutable VResult vresult;

  public:
    SumNode(NodePtr &p) : l(p), vresult(1) {}

    const VResult &
    result() const
    {
        const VResult &lvec = l->result();
        size_type size = lvec.size();
        assert(size > 0);

        vresult[0] = 0.0;

        Op op;
        for (off_type i = 0; i < size; ++i)
            vresult[0] = op(vresult[0], lvec[i]);

        return vresult;
    }

    Result
    total() const
    {
        const VResult &lvec = l->result();
        size_type size = lvec.size();
        assert(size > 0);

        Result result = 0.0;

        Op op;
        for (off_type i = 0; i < size; ++i)
            result = op(result, lvec[i]);

        return result;
    }

    size_type size() const { return 1; }

    std::string
    str() const
    {
        return csprintf("total(%s)", l->str());
    }
};


//////////////////////////////////////////////////////////////////////
//
// Visible Statistics Types
//
//////////////////////////////////////////////////////////////////////
/**
 * @defgroup VisibleStats "Statistic Types"
 * These are the statistics that are used in the simulator.
 * @{
 */

/**
 * This is a simple scalar statistic, like a counter.
 * @sa Stat, ScalarBase, StatStor
 */
class Scalar : public ScalarBase<Scalar, StatStor>
{
  public:
    using ScalarBase<Scalar, StatStor>::operator=;
};

/**
 * A stat that calculates the per tick average of a value.
 * @sa Stat, ScalarBase, AvgStor
 */
class Average : public ScalarBase<Average, AvgStor>
{
  public:
    using ScalarBase<Average, AvgStor>::operator=;
};

class Value : public ValueBase<Value>
{
};

/**
 * A vector of scalar stats.
 * @sa Stat, VectorBase, StatStor
 */
class Vector : public VectorBase<Vector, StatStor>
{
};

/**
 * A vector of Average stats.
 * @sa Stat, VectorBase, AvgStor
 */
class AverageVector : public VectorBase<AverageVector, AvgStor>
{
};

/**
 * A 2-Dimensional vecto of scalar stats.
 * @sa Stat, Vector2dBase, StatStor
 */
class Vector2d : public Vector2dBase<Vector2d, StatStor>
{
};

/**
 * A simple distribution stat.
 * @sa Stat, DistBase, DistStor
 */
class Distribution : public DistBase<Distribution, DistStor>
{
  public:
    /**
     * Set the parameters of this distribution. @sa DistStor::Params
     * @param min The minimum value of the distribution.
     * @param max The maximum value of the distribution.
     * @param bkt The number of values in each bucket.
     * @return A reference to this distribution.
     */
    Distribution &
    init(Counter min, Counter max, Counter bkt)
    {
        DistStor::Params *params = new DistStor::Params;
        params->min = min;
        params->max = max;
        params->bucket_size = bkt;
        params->buckets = (size_type)ceil((max - min + 1.0) / bkt);
        this->setParams(params);
        this->doInit();
        return this->self();
    }
};

/**
 * A simple histogram stat.
 * @sa Stat, DistBase, HistStor
 */
class Histogram : public DistBase<Histogram, HistStor>
{
  public:
    /**
     * Set the parameters of this histogram. @sa HistStor::Params
     * @param size The number of buckets in the histogram
     * @return A reference to this histogram.
     */
    Histogram &
    init(size_type size)
    {
        HistStor::Params *params = new HistStor::Params;
        params->buckets = size;
        this->setParams(params);
        this->doInit();
        return this->self();
    }
};

/**
 * Calculates the mean and variance of all the samples.
 * @sa DistBase, SampleStor
 */
class StandardDeviation : public DistBase<StandardDeviation, SampleStor>
{
  public:
    /**
     * Construct and initialize this distribution.
     */
    StandardDeviation()
    {
        SampleStor::Params *params = new SampleStor::Params;
        this->doInit();
        this->setParams(params);
    }
};

/**
 * Calculates the per tick mean and variance of the samples.
 * @sa DistBase, AvgSampleStor
 */
class AverageDeviation : public DistBase<AverageDeviation, AvgSampleStor>
{
  public:
    /**
     * Construct and initialize this distribution.
     */
    AverageDeviation()
    {
        AvgSampleStor::Params *params = new AvgSampleStor::Params;
        this->doInit();
        this->setParams(params);
    }
};

/**
 * A vector of distributions.
 * @sa VectorDistBase, DistStor
 */
class VectorDistribution : public VectorDistBase<VectorDistribution, DistStor>
{
  public:
    /**
     * Initialize storage and parameters for this distribution.
     * @param size The size of the vector (the number of distributions).
     * @param min The minimum value of the distribution.
     * @param max The maximum value of the distribution.
     * @param bkt The number of values in each bucket.
     * @return A reference to this distribution.
     */
    VectorDistribution &
    init(size_type size, Counter min, Counter max, Counter bkt)
    {
        DistStor::Params *params = new DistStor::Params;
        params->min = min;
        params->max = max;
        params->bucket_size = bkt;
        params->buckets = (size_type)ceil((max - min + 1.0) / bkt);
        this->setParams(params);
        this->doInit(size);
        return this->self();
    }
};

/**
 * This is a vector of StandardDeviation stats.
 * @sa VectorDistBase, SampleStor
 */
class VectorStandardDeviation
    : public VectorDistBase<VectorStandardDeviation, SampleStor>
{
  public:
    /**
     * Initialize storage for this distribution.
     * @param size The size of the vector.
     * @return A reference to this distribution.
     */
    VectorStandardDeviation &
    init(size_type size)
    {
        SampleStor::Params *params = new SampleStor::Params;
        this->doInit(size);
        this->setParams(params);
        return this->self();
    }
};

/**
 * This is a vector of AverageDeviation stats.
 * @sa VectorDistBase, AvgSampleStor
 */
class VectorAverageDeviation
    : public VectorDistBase<VectorAverageDeviation, AvgSampleStor>
{
  public:
    /**
     * Initialize storage for this distribution.
     * @param size The size of the vector.
     * @return A reference to this distribution.
     */
    VectorAverageDeviation &
    init(size_type size)
    {
        AvgSampleStor::Params *params = new AvgSampleStor::Params;
        this->doInit(size);
        this->setParams(params);
        return this->self();
    }
};

template <class Stat>
class FormulaInfoProxy : public InfoProxy<Stat, FormulaInfo>
{
  protected:
    mutable VResult vec;
    mutable VCounter cvec;

  public:
    FormulaInfoProxy(Stat &stat) : InfoProxy<Stat, FormulaInfo>(stat) {}

    size_type size() const { return this->s.size(); }

    const VResult &
    result() const
    {
        this->s.result(vec);
        return vec;
    }
    Result total() const { return this->s.total(); }
    VCounter &value() const { return cvec; }

    std::string str() const { return this->s.str(); }
};

template <class Stat>
class SparseHistInfoProxy : public InfoProxy<Stat, SparseHistInfo>
{
  public:
    SparseHistInfoProxy(Stat &stat) : InfoProxy<Stat, SparseHistInfo>(stat) {}
};

/**
 * Implementation of a sparse histogram stat. The storage class is
 * determined by the Storage template.
 */
template <class Derived, class Stor>
class SparseHistBase : public DataWrap<Derived, SparseHistInfoProxy>
{
  public:
    typedef SparseHistInfoProxy<Derived> Info;
    typedef Stor Storage;
    typedef typename Stor::Params Params;

  protected:
    /** The storage for this stat. */
    char storage[sizeof(Storage)];

  protected:
    /**
     * Retrieve the storage.
     * @return The storage object for this stat.
     */
    Storage *
    data()
    {
        return reinterpret_cast<Storage *>(storage);
    }

    /**
     * Retrieve a const pointer to the storage.
     * @return A const pointer to the storage object for this stat.
     */
    const Storage *
    data() const
    {
        return reinterpret_cast<const Storage *>(storage);
    }

    void
    doInit()
    {
        new (storage) Storage(this->info());
        this->setInit();
    }

  public:
    SparseHistBase() { }

    /**
     * Add a value to the distribtion n times. Calls sample on the storage
     * class.
     * @param v The value to add.
     * @param n The number of times to add it, defaults to 1.
     */
    template <typename U>
    void sample(const U &v, int n = 1) { data()->sample(v, n); }

    /**
     * Return the number of entries in this stat.
     * @return The number of entries.
     */
    size_type size() const { return data()->size(); }
    /**
     * Return true if no samples have been added.
     * @return True if there haven't been any samples.
     */
    bool zero() const { return data()->zero(); }

    void
    prepare()
    {
        Info *info = this->info();
        data()->prepare(info, info->data);
    }

    /**
     * Reset stat value to default
     */
    void
    reset()
    {
        data()->reset(this->info());
    }
};

/**
 * Templatized storage and interface for a sparse histogram stat.
 */
class SparseHistStor
{
  public:
    /** The parameters for a sparse histogram stat. */
    struct Params : public DistParams
    {
        Params() : DistParams(Hist) {}
    };

  private:
    /** Counter for number of samples */
    Counter samples;
    /** Counter for each bucket. */
    MCounter cmap;

  public:
    SparseHistStor(Info *info)
    {
        reset(info);
    }

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void
    sample(Counter val, int number)
    {
        cmap[val] += number;
        samples += number;
    }

    /**
     * Return the number of buckets in this distribution.
     * @return the number of buckets.
     */
    size_type size() const { return cmap.size(); }

    /**
     * Returns true if any calls to sample have been made.
     * @return True if any values have been sampled.
     */
    bool
    zero() const
    {
        return samples == Counter();
    }

    void
    prepare(Info *info, SparseHistData &data)
    {
        MCounter::iterator it;
        data.cmap.clear();
        for (it = cmap.begin(); it != cmap.end(); it++) {
            data.cmap[(*it).first] = (*it).second;
        }

        data.samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void
    reset(Info *info)
    {
        cmap.clear();
        samples = 0;
    }
};

class SparseHistogram : public SparseHistBase<SparseHistogram, SparseHistStor>
{
  public:
    /**
     * Set the parameters of this histogram. @sa HistStor::Params
     * @param size The number of buckets in the histogram
     * @return A reference to this histogram.
     */
    SparseHistogram &
    init(size_type size)
    {
        SparseHistStor::Params *params = new SparseHistStor::Params;
        this->setParams(params);
        this->doInit();
        return this->self();
    }
};

class Temp;
/**
 * A formula for statistics that is calculated when printed. A formula is
 * stored as a tree of Nodes that represent the equation to calculate.
 * @sa Stat, ScalarStat, VectorStat, Node, Temp
 */
class Formula : public DataWrapVec<Formula, FormulaInfoProxy>
{
  protected:
    /** The root of the tree which represents the Formula */
    NodePtr root;
    friend class Temp;

  public:
    /**
     * Create and initialize thie formula, and register it with the database.
     */
    Formula();

    /**
     * Create a formula with the given root node, register it with the
     * database.
     * @param r The root of the expression tree.
     */
    Formula(Temp r);

    /**
     * Set an unitialized Formula to the given root.
     * @param r The root of the expression tree.
     * @return a reference to this formula.
     */
    const Formula &operator=(Temp r);

    /**
     * Add the given tree to the existing one.
     * @param r The root of the expression tree.
     * @return a reference to this formula.
     */
    const Formula &operator+=(Temp r);

    /**
     * Divide the existing tree by the given one.
     * @param r The root of the expression tree.
     * @return a reference to this formula.
     */
    const Formula &operator/=(Temp r);

    /**
     * Return the result of the Fomula in a vector.  If there were no Vector
     * components to the Formula, then the vector is size 1.  If there were,
     * like x/y with x being a vector of size 3, then the result returned will
     * be x[0]/y, x[1]/y, x[2]/y, respectively.
     * @return The result vector.
     */
    void result(VResult &vec) const;

    /**
     * Return the total Formula result.  If there is a Vector
     * component to this Formula, then this is the result of the
     * Formula if the formula is applied after summing all the
     * components of the Vector.  For example, if Formula is x/y where
     * x is size 3, then total() will return (x[1]+x[2]+x[3])/y.  If
     * there is no Vector component, total() returns the same value as
     * the first entry in the VResult val() returns.
     * @return The total of the result vector.
     */
    Result total() const;

    /**
     * Return the number of elements in the tree.
     */
    size_type size() const;

    void prepare() { }

    /**
     * Formulas don't need to be reset
     */
    void reset();

    /**
     *
     */
    bool zero() const;

    std::string str() const;
};

class FormulaNode : public Node
{
  private:
    const Formula &formula;
    mutable VResult vec;

  public:
    FormulaNode(const Formula &f) : formula(f) {}

    size_type size() const { return formula.size(); }
    const VResult &result() const { formula.result(vec); return vec; }
    Result total() const { return formula.total(); }

    std::string str() const { return formula.str(); }
};

/**
 * Helper class to construct formula node trees.
 */
class Temp
{
  protected:
    /**
     * Pointer to a Node object.
     */
    NodePtr node;

  public:
    /**
     * Copy the given pointer to this class.
     * @param n A pointer to a Node object to copy.
     */
    Temp(const NodePtr &n) : node(n) { }

    Temp(NodePtr &&n) : node(std::move(n)) { }

    /**
     * Return the node pointer.
     * @return the node pointer.
     */
    operator NodePtr&() { return node; }

    /**
     * Makde gcc < 4.6.3 happy and explicitly get the underlying node.
     */
    NodePtr getNodePtr() const { return node; }

  public:
    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    Temp(const Scalar &s)
        : node(new ScalarStatNode(s.info()))
    { }

    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    Temp(const Value &s)
        : node(new ScalarStatNode(s.info()))
    { }

    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    Temp(const Average &s)
        : node(new ScalarStatNode(s.info()))
    { }

    /**
     * Create a new VectorStatNode.
     * @param s The VectorStat to place in a node.
     */
    Temp(const Vector &s)
        : node(new VectorStatNode(s.info()))
    { }

    Temp(const AverageVector &s)
        : node(new VectorStatNode(s.info()))
    { }

    /**
     *
     */
    Temp(const Formula &f)
        : node(new FormulaNode(f))
    { }

    /**
     * Create a new ScalarProxyNode.
     * @param p The ScalarProxy to place in a node.
     */
    template <class Stat>
    Temp(const ScalarProxy<Stat> &p)
        : node(new ScalarProxyNode<Stat>(p))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed char value)
        : node(new ConstNode<signed char>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned char value)
        : node(new ConstNode<unsigned char>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed short value)
        : node(new ConstNode<signed short>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned short value)
        : node(new ConstNode<unsigned short>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed int value)
        : node(new ConstNode<signed int>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned int value)
        : node(new ConstNode<unsigned int>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed long value)
        : node(new ConstNode<signed long>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned long value)
        : node(new ConstNode<unsigned long>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed long long value)
        : node(new ConstNode<signed long long>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned long long value)
        : node(new ConstNode<unsigned long long>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(float value)
        : node(new ConstNode<float>(value))
    { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(double value)
        : node(new ConstNode<double>(value))
    { }
};


/**
 * @}
 */

inline Temp
operator+(Temp l, Temp r)
{
    return Temp(std::make_shared<BinaryNode<std::plus<Result> > >(l, r));
}

inline Temp
operator-(Temp l, Temp r)
{
    return Temp(std::make_shared<BinaryNode<std::minus<Result> > >(l, r));
}

inline Temp
operator*(Temp l, Temp r)
{
    return Temp(std::make_shared<BinaryNode<std::multiplies<Result> > >(l, r));
}

inline Temp
operator/(Temp l, Temp r)
{
    return Temp(std::make_shared<BinaryNode<std::divides<Result> > >(l, r));
}

inline Temp
operator-(Temp l)
{
    return Temp(std::make_shared<UnaryNode<std::negate<Result> > >(l));
}

template <typename T>
inline Temp
constant(T val)
{
    return Temp(std::make_shared<ConstNode<T> >(val));
}

template <typename T>
inline Temp
constantVector(T val)
{
    return Temp(std::make_shared<ConstVectorNode<T> >(val));
}

inline Temp
sum(Temp val)
{
    return Temp(std::make_shared<SumNode<std::plus<Result> > >(val));
}

/** Dump all statistics data to the registered outputs */
void dump();
void reset();
void enable();
bool enabled();

/**
 * Register reset and dump handlers.  These are the functions which
 * will actually perform the whole statistics reset/dump actions
 * including processing the reset/dump callbacks
 */
typedef void (*Handler)();

void registerHandlers(Handler reset_handler, Handler dump_handler);

/**
 * Register a callback that should be called whenever statistics are
 * reset
 */
void registerResetCallback(Callback *cb);

/**
 * Register a callback that should be called whenever statistics are
 * about to be dumped
 */
void registerDumpCallback(Callback *cb);

/**
 * Process all the callbacks in the reset callbacks queue
 */
void processResetQueue();

/**
 * Process all the callbacks in the dump callbacks queue
 */
void processDumpQueue();

std::list<Info *> &statsList();

typedef std::map<const void *, Info *> MapType;
MapType &statsMap();

typedef std::map<std::string, Info *> NameMapType;
NameMapType &nameMap();

bool validateStatName(const std::string &name);

} // namespace Stats

void debugDumpStats();

#endif // __BASE_STATISTICS_HH__

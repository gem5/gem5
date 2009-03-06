/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
#include <string>
#include <vector>

#include "base/cast.hh"
#include "base/cprintf.hh"
#include "base/intmath.hh"
#include "base/refcnt.hh"
#include "base/str.hh"
#include "base/stats/flags.hh"
#include "base/stats/visit.hh"
#include "base/stats/types.hh"
#include "sim/host.hh"

class Callback;

/** The current simulated tick. */
extern Tick curTick;

/* A namespace for all of the Statistics */
namespace Stats {

struct StorageParams
{
    virtual ~StorageParams();
};

//////////////////////////////////////////////////////////////////////
//
// Statistics Framework Base classes
//
//////////////////////////////////////////////////////////////////////
class Info
{
  public:
    /** The name of the stat. */
    std::string name;
    /** The description of the stat. */
    std::string desc;
    /** The formatting flags. */
    StatFlags flags;
    /** The display precision. */
    int precision;
    /** A pointer to a prerequisite Stat. */
    const Info *prereq;
    /**
     * A unique stat ID for each stat in the simulator.
     * Can be used externally for lookups as well as for debugging.
     */
    static int id_count;
    int id;

  public:
    const StorageParams *storageParams;

  public:
    Info();
    virtual ~Info();

    /**
     * Reset the corresponding stat to the default state.
     */
    virtual void reset() = 0;

    /**
     * @return true if this stat has a value and satisfies its
     * requirement as a prereq
     */
    virtual bool zero() const = 0;

    /**
     * Check that this stat has been set up properly and is ready for
     * use
     * @return true for success
     */
    virtual bool check() const = 0;
    bool baseCheck() const;

    /**
     * Visitor entry for outputing statistics data
     */
    virtual void visit(Visit &visitor) = 0;

    /**
     * Checks if the first stat's name is alphabetically less than the second.
     * This function breaks names up at periods and considers each subname
     * separately.
     * @param stat1 The first stat.
     * @param stat2 The second stat.
     * @return stat1's name is alphabetically before stat2's
     */
    static bool less(Info *stat1, Info *stat2);
};

class ScalarInfoBase : public Info
{
  public:
    virtual Counter value() const = 0;
    virtual Result result() const = 0;
    virtual Result total() const = 0;
    virtual void visit(Visit &visitor) { visitor.visit(*this); }
};

template <class Stat>
class ScalarInfo : public ScalarInfoBase
{
  protected:
    Stat &s;

  public:
    ScalarInfo(Stat &stat) : s(stat) {}

    virtual bool check() const { return s.check(); }
    virtual Counter value() const { return s.value(); }
    virtual Result result() const { return s.result(); }
    virtual Result total() const { return s.total(); }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }
};

class VectorInfoBase : public Info
{
  public:
    /** Names and descriptions of subfields. */
    mutable std::vector<std::string> subnames;
    mutable std::vector<std::string> subdescs;

  public:
    virtual size_type size() const  = 0;
    virtual const VCounter &value() const = 0;
    virtual const VResult &result() const = 0;
    virtual Result total() const  = 0;

    void
    update()
    {
        if (!subnames.empty()) {
            size_type s = size();
            if (subnames.size() < s)
                subnames.resize(s);

            if (subdescs.size() < s)
                subdescs.resize(s);
        }
    }
};

template <class Stat>
class VectorInfo : public VectorInfoBase
{
  protected:
    Stat &s;
    mutable VCounter cvec;
    mutable VResult rvec;

  public:
    VectorInfo(Stat &stat) : s(stat) {}

    virtual bool check() const { return s.check(); }
    virtual bool zero() const { return s.zero(); }
    virtual void reset() { s.reset(); }

    virtual size_type size() const { return s.size(); }

    virtual VCounter &
    value() const
    {
        s.value(cvec);
        return cvec;
    }

    virtual const VResult &
    result() const
    {
        s.result(rvec);
        return rvec;
    }

    virtual Result total() const { return s.total(); }

    virtual void
    visit(Visit &visitor)
    {
        update();
        s.update(this);
        visitor.visit(*this);
    }
};

struct DistData
{
    Counter min_val;
    Counter max_val;
    Counter underflow;
    Counter overflow;
    VCounter cvec;
    Counter sum;
    Counter squares;
    Counter samples;

    bool fancy;
};

class DistInfoBase : public Info
{
  public:
    /** Local storage for the entry values, used for printing. */
    DistData data;
};

template <class Stat>
class DistInfo : public DistInfoBase
{
  protected:
    Stat &s;

  public:
    DistInfo(Stat &stat) : s(stat) {}

    virtual bool check() const { return s.check(); }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }

    virtual void
    visit(Visit &visitor)
    {
        s.update(this);
        visitor.visit(*this);
    }
};

class VectorDistInfoBase : public Info
{
  public:
    std::vector<DistData> data;

   /** Names and descriptions of subfields. */
    mutable std::vector<std::string> subnames;
    mutable std::vector<std::string> subdescs;

  protected:
    /** Local storage for the entry values, used for printing. */
    mutable VResult rvec;

  public:
    virtual size_type size() const = 0;

    void
    update()
    {
        size_type s = size();
        if (subnames.size() < s)
            subnames.resize(s);

        if (subdescs.size() < s)
            subdescs.resize(s);
    }
};

template <class Stat>
class VectorDistInfo : public VectorDistInfoBase
{
  protected:
    Stat &s;

  public:
    VectorDistInfo(Stat &stat) : s(stat) {}

    virtual bool check() const { return s.check(); }
    virtual void reset() { s.reset(); }
    virtual size_type size() const { return s.size(); }
    virtual bool zero() const { return s.zero(); }

    virtual void
    visit(Visit &visitor)
    {
        update();
        s.update(this);
        visitor.visit(*this);
    }
};

class Vector2dInfoBase : public Info
{
  public:
    /** Names and descriptions of subfields. */
    std::vector<std::string> subnames;
    std::vector<std::string> subdescs;
    std::vector<std::string> y_subnames;

    /** Local storage for the entry values, used for printing. */
    mutable VCounter cvec;
    mutable size_type x;
    mutable size_type y;

  public:
    void
    update()
    {
        if (subnames.size() < x)
            subnames.resize(x);
    }
};

template <class Stat>
class Vector2dInfo : public Vector2dInfoBase
{
  protected:
    Stat &s;

  public:
    Vector2dInfo(Stat &stat) : s(stat) {}

    virtual bool check() const { return s.check(); }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }

    virtual void
    visit(Visit &visitor)
    {
        update();
        s.update(this);
        visitor.visit(*this);
    }
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
};

template <class Parent, class Child, template <class> class Info>
class Wrap : public Child
{
  public:
    typedef Parent ParentType;
    typedef Child ChildType;
    typedef Info<Child> InfoType;

  protected:
    Parent &self() { return *reinterpret_cast<Parent *>(this); }

  protected:
    InfoType *
    info()
    {
        return safe_cast<InfoType *>(InfoAccess::info());
    }

  public:
    const InfoType *
    info() const
    {
        return safe_cast<const InfoType *>(InfoAccess::info());
    }

  protected:
    /**
     * Copy constructor, copies are not allowed.
     */
    Wrap(const Wrap &stat);

    /**
     * Can't copy stats.
     */
    void operator=(const Wrap &);

  public:
    Wrap()
    {
        this->setInfo(new InfoType(*this));
    }

    /**
     * Set the name and marks this stat to print at the end of simulation.
     * @param name The new name.
     * @return A reference to this stat.
     */
    Parent &
    name(const std::string &_name)
    {
        InfoType *info = this->info();
        info->name = _name;
        info->flags |= print;
        return this->self();
    }

    /**
     * Set the description and marks this stat to print at the end of
     * simulation.
     * @param desc The new description.
     * @return A reference to this stat.
     */
    Parent &
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
    Parent &
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
    Parent &
    flags(StatFlags _flags)
    {
        this->info()->flags |= _flags;
        return this->self();
    }

    /**
     * Set the prerequisite stat and marks this stat to print at the end of
     * simulation.
     * @param prereq The prerequisite stat.
     * @return A reference to this stat.
     */
    template <class Stat>
    Parent &
    prereq(const Stat &prereq)
    {
        this->info()->prereq = prereq.info();
        return this->self();
    }
};

template <class Parent, class Child, template <class Child> class Info>
class WrapVec : public Wrap<Parent, Child, Info>
{
  public:
    typedef Parent ParentType;
    typedef Child ChildType;
    typedef Info<Child> InfoType;

  public:
    // The following functions are specific to vectors.  If you use them
    // in a non vector context, you will get a nice compiler error!

    /**
     * Set the subfield name for the given index, and marks this stat to print
     * at the end of simulation.
     * @param index The subfield index.
     * @param name The new name of the subfield.
     * @return A reference to this stat.
     */
    Parent &
    subname(off_type index, const std::string &name)
    {
        std::vector<std::string> &subn = this->info()->subnames;
        if (subn.size() <= index)
            subn.resize(index + 1);
        subn[index] = name;
        return this->self();
    }

    /**
     * Set the subfield description for the given index and marks this stat to
     * print at the end of simulation.
     * @param index The subfield index.
     * @param desc The new description of the subfield
     * @return A reference to this stat.
     */
    Parent &
    subdesc(off_type index, const std::string &desc)
    {
        std::vector<std::string> &subd = this->info()->subdescs;
        if (subd.size() <= index)
            subd.resize(index + 1);
        subd[index] = desc;

        return this->self();
    }

};

template <class Parent, class Child, template <class Child> class Info>
class WrapVec2d : public WrapVec<Parent, Child, Info>
{
  public:
    typedef Parent ParentType;
    typedef Child ChildType;
    typedef Info<Child> InfoType;

  public:
    /**
     * @warning This makes the assumption that if you're gonna subnames a 2d
     * vector, you're subnaming across all y
     */
    Parent &
    ysubnames(const char **names)
    {
        InfoType *info = this->info();
        info->y_subnames.resize(this->y);
        for (off_type i = 0; i < this->y; ++i)
            info->y_subnames[i] = names[i];
        return this->self();
    }

    Parent &
    ysubname(off_type index, const std::string subname)
    {
        InfoType *info = this->info();
        assert(index < this->y);
        info->y_subnames.resize(this->y);
        info->y_subnames[index] = subname.c_str();
        return this->self();
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
        : current(0), total(0), last(0)
    { }

    /**
     * Set the current count to the one provided, update the total and last
     * set values.
     * @param val The new count.
     */
    void
    set(Counter val)
    {
        total += current * (curTick - last);
        last = curTick;
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
        total += current * (curTick - last);
        last = curTick;
        return (Result)(total + current) / (Result)(curTick + 1);
    }

    /**
     * Reset stat value to default
     */
    void
    reset(Info *info)
    {
        total = 0.0;
        last = curTick;
    }

    /**
     * @return true if zero value
     */
    bool zero() const { return total == 0.0; }
};

/**
 * Implementation of a scalar stat. The type of stat is determined by the
 * Storage template.
 */
template <class Stor>
class ScalarBase : public InfoAccess
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
        new (storage) Storage(info());
        setInit();
    }

  public:
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    Counter value() const { return data()->value(); }

  public:
    ScalarBase() { }

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

    bool check() const { return true; }

    /**
     * Reset stat value to default
     */
    void reset() { data()->reset(info()); }

    Counter value() { return data()->value(); }

    Result result() { return data()->result(); }

    Result total() { return result(); }

    bool zero() { return result() == 0.0; }

};

class ProxyInfo : public ScalarInfoBase
{
  public:
    virtual void visit(Visit &visitor) { visitor.visit(*this); }
    virtual std::string str() const { return to_string(value()); }
    virtual size_type size() const { return 1; }
    virtual bool zero() const { return value() == 0; }
    virtual bool check() const { return true; }
    virtual void reset() { }
};

template <class T>
class ValueProxy : public ProxyInfo
{
  private:
    T *scalar;

  public:
    ValueProxy(T &val) : scalar(&val) {}
    virtual Counter value() const { return *scalar; }
    virtual Result result() const { return *scalar; }
    virtual Result total() const { return *scalar; }
};

template <class T>
class FunctorProxy : public ProxyInfo
{
  private:
    T *functor;

  public:
    FunctorProxy(T &func) : functor(&func) {}
    virtual Counter value() const { return (*functor)(); }
    virtual Result result() const { return (*functor)(); }
    virtual Result total() const { return (*functor)(); }
};

class ValueBase : public InfoAccess
{
  private:
    ProxyInfo *proxy;

  public:
    ValueBase() : proxy(NULL) { }
    ~ValueBase() { if (proxy) delete proxy; }

    template <class T>
    void
    scalar(T &value)
    {
        proxy = new ValueProxy<T>(value);
        setInit();
    }

    template <class T>
    void
    functor(T &func)
    {
        proxy = new FunctorProxy<T>(func);
        setInit();
    }

    Counter value() { return proxy->value(); }
    Result result() const { return proxy->result(); }
    Result total() const { return proxy->total(); };
    size_type size() const { return proxy->size(); }

    std::string str() const { return proxy->str(); }
    bool zero() const { return proxy->zero(); }
    bool check() const { return proxy != NULL; }
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
    Stat *stat;

    /** The index to access in the parent VectorBase. */
    off_type index;

  public:
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    Counter value() const { return stat->data(index)->value(); }

    /**
     * Return the current value of this statas a result type.
     * @return The current value.
     */
    Result result() const { return stat->data(index)->result(); }

  public:
    /**
     * Create and initialize this proxy, do not register it with the database.
     * @param i The index to access.
     */
    ScalarProxy(Stat *s, off_type i)
        : stat(s), index(i)
    {
        assert(stat);
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
    void operator++() { stat->data(index)->inc(1); }
    /**
     * Decrement the stat by 1. This calls the associated storage object dec
     * function.
     */
    void operator--() { stat->data(index)->dec(1); }

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
        stat->data(index)->set(v);
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
        stat->data(index)->inc(v);
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
        stat->data(index)->dec(v);
    }

    /**
     * Return the number of elements, always 1 for a scalar.
     * @return 1.
     */
    size_type size() const { return 1; }

    /**
     * This stat has no state.  Nothing to reset
     */
    void reset() {  }

  public:
    std::string
    str() const
    {
        return csprintf("%s[%d]", stat->info()->name, index);
    }
};

/**
 * Implementation of a vector of stats. The type of stat is determined by the
 * Storage class. @sa ScalarBase
 */
template <class Stor>
class VectorBase : public InfoAccess
{
  public:
    typedef Stor Storage;
    typedef typename Stor::Params Params;

    /** Proxy type */
    typedef ScalarProxy<VectorBase<Storage> > Proxy;

    friend class ScalarProxy<VectorBase<Storage> >;

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
            new (&storage[i]) Storage(info());

        setInit();
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

    void
    reset()
    {
        for (off_type i = 0; i < size(); ++i)
            data(i)->reset(info());
    }

  public:
    VectorBase()
        : storage(NULL)
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
     * Return a reference (ScalarProxy) to the stat at the given index.
     * @param index The vector index to access.
     * @return A reference of the stat.
     */
    Proxy
    operator[](off_type index)
    {
        assert (index >= 0 && index < size());
        return Proxy(this, index);
    }

    void update(Info *data) {}
};

template <class Stat>
class VectorProxy
{
  private:
    Stat *stat;
    off_type offset;
    size_type len;

  private:
    mutable VResult vec;

    typename Stat::Storage *
    data(off_type index)
    {
        assert(index < len);
        return stat->data(offset + index);
    }

    const typename Stat::Storage *
    data(off_type index) const
    {
        assert(index < len);
        return const_cast<Stat *>(stat)->data(offset + index);
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
    VectorProxy(Stat *s, off_type o, size_type l)
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

    /**
     * This stat has no state.  Nothing to reset.
     */
    void reset() { }
};

template <class Stor>
class Vector2dBase : public InfoAccess
{
  public:
    typedef Stor Storage;
    typedef typename Stor::Params Params;
    typedef VectorProxy<Vector2dBase<Storage> > Proxy;
    friend class ScalarProxy<Vector2dBase<Storage> >;
    friend class VectorProxy<Vector2dBase<Storage> >;

  protected:
    size_type x;
    size_type y;
    size_type _size;
    Storage *storage;

  protected:
    Storage *data(off_type index) { return &storage[index]; }
    const Storage *data(off_type index) const { return &storage[index]; }

    void
    doInit(size_type _x, size_type _y)
    {
        assert(_x > 0 && _y > 0 && "sizes must be positive!");
        assert(!storage && "already initialized");

        Vector2dInfoBase *info = safe_cast<Vector2dInfoBase *>(this->info());

        x = _x;
        y = _y;
        info->x = _x;
        info->y = _y;
        _size = x * y;

        char *ptr = new char[_size * sizeof(Storage)];
        storage = reinterpret_cast<Storage *>(ptr);

        for (off_type i = 0; i < _size; ++i)
            new (&storage[i]) Storage(info);

        setInit();
    }

  public:
    Vector2dBase()
        : storage(NULL)
    {}

    ~Vector2dBase()
    {
        if (!storage)
            return;

        for (off_type i = 0; i < _size; ++i)
            data(i)->~Storage();
        delete [] reinterpret_cast<char *>(storage);
    }

    void
    update(Vector2dInfoBase *newinfo)
    {
        size_type size = this->size();
        newinfo->cvec.resize(size);
        for (off_type i = 0; i < size; ++i)
            newinfo->cvec[i] = data(i)->value();
    }

    std::string ysubname(off_type i) const { return (*this->y_subnames)[i]; }

    Proxy
    operator[](off_type index)
    {
        off_type offset = index * y;
        assert (index >= 0 && offset + index < size());
        return Proxy(this, offset, y);
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
     * Reset stat value to default
     */
    void
    reset()
    {
        for (off_type i = 0; i < size(); ++i)
            data(i)->reset(info());
    }

    bool
    check()
    {
        return storage != NULL;
    }
};

//////////////////////////////////////////////////////////////////////
//
// Non formula statistics
//
//////////////////////////////////////////////////////////////////////

/**
 * Templatized storage and interface for a distrbution stat.
 */
class DistStor
{
  public:
    /** The parameters for a distribution stat. */
    struct Params : public StorageParams
    {
        /** The minimum value to track. */
        Counter min;
        /** The maximum value to track. */
        Counter max;
        /** The number of entries in each bucket. */
        Counter bucket_size;
        /** The number of buckets. Equal to (max-min)/bucket_size. */
        size_type buckets;
    };
    enum { fancy = false };

  private:
    /** The minimum value to track. */
    Counter min_track;
    /** The maximum value to track. */
    Counter max_track;
    /** The number of entries in each bucket. */
    Counter bucket_size;
    /** The number of buckets. Equal to (max-min)/bucket_size. */
    size_type buckets;

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

        Counter sample = val * number;
        sum += sample;
        squares += sample * sample;
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
    update(Info *info, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(info->storageParams);

        data.min_val = (min_val == CounterLimits::max()) ? 0 : min_val;
        data.max_val = (max_val == CounterLimits::min()) ? 0 : max_val;
        data.underflow = underflow;
        data.overflow = overflow;

        int buckets = params->buckets;
        data.cvec.resize(buckets);
        for (off_type i = 0; i < buckets; ++i)
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
        underflow = 0;
        overflow = 0;

        size_type size = cvec.size();
        for (off_type i = 0; i < size; ++i)
            cvec[i] = Counter();

        sum = Counter();
        squares = Counter();
        samples = Counter();
    }
};

/**
 * Templatized storage and interface for a distribution that calculates mean
 * and variance.
 */
class FancyStor
{
  public:
    struct Params : public StorageParams {};

  public:
    enum { fancy = true };

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
    FancyStor(Info *info)
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

    void
    update(Info *info, DistData &data)
    {
        data.sum = sum;
        data.squares = squares;
        data.samples = samples;
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
class AvgFancy
{
  public:
    struct Params : public StorageParams {};

  public:
    enum { fancy = true };

  private:
    /** Current total. */
    Counter sum;
    /** Current sum of squares. */
    Counter squares;

  public:
    /**
     * Create and initialize this storage.
     */
    AvgFancy(Info *info)
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

    void
    update(Info *info, DistData &data)
    {
        data.sum = sum;
        data.squares = squares;
        data.samples = curTick;
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
template <class Stor>
class DistBase : public InfoAccess
{
  public:
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
        new (storage) Storage(info());
        setInit();
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
    update(DistInfoBase *base)
    {
        base->data.fancy = Storage::fancy;
        data()->update(info(), base->data);
    }

    /**
     * Reset stat value to default
     */
    void
    reset()
    {
        data()->reset(info());
    }

    bool
    check()
    {
        return true;
    }
};

template <class Stat>
class DistProxy;

template <class Stor>
class VectorDistBase : public InfoAccess
{
  public:
    typedef Stor Storage;
    typedef typename Stor::Params Params;
    typedef DistProxy<VectorDistBase<Storage> > Proxy;
    friend class DistProxy<VectorDistBase<Storage> >;

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

        for (off_type i = 0; i < _size; ++i)
            new (&storage[i]) Storage(info());

        setInit();
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

    Proxy operator[](off_type index);

    size_type
    size() const
    {
        return _size;
    }

    bool
    zero() const
    {
        return false;
#if 0
        for (off_type i = 0; i < size(); ++i)
            if (!data(i)->zero())
                return false;
        return true;
#endif
    }

    /**
     * Reset stat value to default
     */
    void
    reset()
    {
        for (off_type i = 0; i < size(); ++i)
            data(i)->reset(info());
    }

    bool
    check()
    {
        return storage != NULL;
    }

    void
    update(VectorDistInfoBase *base)
    {
        size_type size = this->size();
        base->data.resize(size);
        for (off_type i = 0; i < size; ++i) {
            base->data[i].fancy = Storage::fancy;
            data(i)->update(info(), base->data[i]);
        }
    }
};

template <class Stat>
class DistProxy
{
  private:
    Stat *stat;
    off_type index;

  protected:
    typename Stat::Storage *data() { return stat->data(index); }
    const typename Stat::Storage *data() const { return stat->data(index); }

  public:
    DistProxy(Stat *s, off_type i)
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

template <class Storage>
inline typename VectorDistBase<Storage>::Proxy
VectorDistBase<Storage>::operator[](off_type index)
{
    assert (index >= 0 && index < size());
    return typename VectorDistBase<Storage>::Proxy(this, index);
}

#if 0
template <class Storage>
Result
VectorDistBase<Storage>::total(off_type index) const
{
    Result total = 0.0;
    for (off_type i = 0; i < x_size(); ++i)
        total += data(i)->result();
}
#endif

//////////////////////////////////////////////////////////////////////
//
//  Formula Details
//
//////////////////////////////////////////////////////////////////////

/**
 * Base class for formula statistic node. These nodes are used to build a tree
 * that represents the formula.
 */
class Node : public RefCounted
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
};

/** Reference counting pointer to a function Node. */
typedef RefCountingPtr<Node> NodePtr;

class ScalarStatNode : public Node
{
  private:
    const ScalarInfoBase *data;
    mutable VResult vresult;

  public:
    ScalarStatNode(const ScalarInfoBase *d) : data(d), vresult(1) {}

    virtual const VResult &
    result() const
    {
        vresult[0] = data->result();
        return vresult;
    }

    virtual Result total() const { return data->result(); };

    virtual size_type size() const { return 1; }

    /**
     *
     */
    virtual std::string str() const { return data->name; }
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

    virtual const VResult &
    result() const
    {
        vresult[0] = proxy.result();
        return vresult;
    }

    virtual Result
    total() const
    {
        return proxy.result();
    }

    virtual size_type
    size() const
    {
        return 1;
    }

    /**
     *
     */
    virtual std::string
    str() const
    {
        return proxy.str();
    }
};

class VectorStatNode : public Node
{
  private:
    const VectorInfoBase *data;

  public:
    VectorStatNode(const VectorInfoBase *d) : data(d) { }
    virtual const VResult &result() const { return data->result(); }
    virtual Result total() const { return data->total(); };

    virtual size_type size() const { return data->size(); }

    virtual std::string str() const { return data->name; }
};

template <class T>
class ConstNode : public Node
{
  private:
    VResult vresult;

  public:
    ConstNode(T s) : vresult(1, (Result)s) {}
    const VResult &result() const { return vresult; }
    virtual Result total() const { return vresult[0]; };
    virtual size_type size() const { return 1; }
    virtual std::string str() const { return to_string(vresult[0]); }
};

template <class T>
class ConstVectorNode : public Node
{
  private:
    VResult vresult;

  public:
    ConstVectorNode(const T &s) : vresult(s.begin(), s.end()) {}
    const VResult &result() const { return vresult; }

    virtual Result
    total() const
    {
        size_type size = this->size();
        Result tmp = 0;
        for (off_type i = 0; i < size; i++)
            tmp += vresult[i];
        return tmp;
    }

    virtual size_type size() const { return vresult.size(); }
    virtual std::string
    str() const
    {
        size_type size = this->size();
        std::string tmp = "(";
        for (off_type i = 0; i < size; i++)
            tmp += csprintf("%s ",to_string(vresult[i]));
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

    virtual size_type size() const { return l->size(); }

    virtual std::string
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
    result() const
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
    total() const
    {
        const VResult &vec = this->result();
        Result total = 0.0;
        for (off_type i = 0; i < size(); i++)
            total += vec[i];
        return total;
    }

    virtual size_type
    size() const
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

    virtual std::string
    str() const
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

        Result vresult = 0.0;

        Op op;
        for (off_type i = 0; i < size; ++i)
            vresult = op(vresult, lvec[i]);

        return vresult;
    }

    virtual size_type size() const { return 1; }

    virtual std::string
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
template<int N = 0>
class Scalar : public Wrap<Scalar<N>, ScalarBase<StatStor>, ScalarInfo>
{
  public:
    /** The base implementation. */
    typedef ScalarBase<StatStor> Base;

    Scalar()
    {
        this->doInit();
    }

    /**
     * Sets the stat equal to the given value. Calls the base implementation
     * of operator=
     * @param v The new value.
     */
    template <typename U>
    void operator=(const U &v) { Base::operator=(v); }
};

class Value : public Wrap<Value, ValueBase, ScalarInfo>
{
  public:
    /** The base implementation. */
    typedef ValueBase Base;

    template <class T>
    Value &
    scalar(T &value)
    {
        Base::scalar(value);
        return *this;
    }

    template <class T>
    Value &
    functor(T &func)
    {
        Base::functor(func);
        return *this;
    }
};

/**
 * A stat that calculates the per tick average of a value.
 * @sa Stat, ScalarBase, AvgStor
 */
template<int N = 0>
class Average : public Wrap<Average<N>, ScalarBase<AvgStor>, ScalarInfo>
{
  public:
    /** The base implementation. */
    typedef ScalarBase<AvgStor> Base;

    Average()
    {
        this->doInit();
    }

    /**
     * Sets the stat equal to the given value. Calls the base implementation
     * of operator=
     * @param v The new value.
     */
    template <typename U>
    void
    operator=(const U &v)
    {
        Base::operator=(v);
    }
};

/**
 * A vector of scalar stats.
 * @sa Stat, VectorBase, StatStor
 */
template<int N = 0>
class Vector : public WrapVec<Vector<N>, VectorBase<StatStor>, VectorInfo>
{
  public:
    /** The base implementation. */
    typedef ScalarBase<StatStor> Base;

    /**
     * Set this vector to have the given size.
     * @param size The new size.
     * @return A reference to this stat.
     */
    Vector &
    init(size_type size)
    {
        this->doInit(size);
        return *this;
    }
};

/**
 * A vector of Average stats.
 * @sa Stat, VectorBase, AvgStor
 */
template<int N = 0>
class AverageVector
    : public WrapVec<AverageVector<N>, VectorBase<AvgStor>, VectorInfo>
{
  public:
    /**
     * Set this vector to have the given size.
     * @param size The new size.
     * @return A reference to this stat.
     */
    AverageVector &
    init(size_type size)
    {
        this->doInit(size);
        return *this;
    }
};

/**
 * A 2-Dimensional vecto of scalar stats.
 * @sa Stat, Vector2dBase, StatStor
 */
template<int N = 0>
class Vector2d
    : public WrapVec2d<Vector2d<N>, Vector2dBase<StatStor>, Vector2dInfo>
{
  public:
    Vector2d &
    init(size_type x, size_type y)
    {
        this->doInit(x, y);
        return *this;
    }
};

/**
 * A simple distribution stat.
 * @sa Stat, DistBase, DistStor
 */
template<int N = 0>
class Distribution
    : public Wrap<Distribution<N>, DistBase<DistStor>, DistInfo>
{
  public:
    /** Base implementation. */
    typedef DistBase<DistStor> Base;

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
        params->buckets = (size_type)rint((max - min) / bkt + 1.0);
        this->setParams(params);
        this->doInit();
        return *this;
    }
};

/**
 * Calculates the mean and variance of all the samples.
 * @sa Stat, DistBase, FancyStor
 */
template<int N = 0>
class StandardDeviation
    : public Wrap<StandardDeviation<N>, DistBase<FancyStor>, DistInfo>
{
  public:
    /** The base implementation */
    typedef DistBase<DistStor> Base;

  public:
    /**
     * Construct and initialize this distribution.
     */
    StandardDeviation()
    {
        this->doInit();
    }
};

/**
 * Calculates the per tick mean and variance of the samples.
 * @sa Stat, DistBase, AvgFancy
 */
template<int N = 0>
class AverageDeviation
    : public Wrap<AverageDeviation<N>, DistBase<AvgFancy>, DistInfo>
{
  public:
    /** The base implementation */
    typedef DistBase<DistStor> Base;

  public:
    /**
     * Construct and initialize this distribution.
     */
    AverageDeviation()
    {
        this->doInit();
    }
};

/**
 * A vector of distributions.
 * @sa Stat, VectorDistBase, DistStor
 */
template<int N = 0>
class VectorDistribution
    : public WrapVec<VectorDistribution<N>,
                     VectorDistBase<DistStor>,
                     VectorDistInfo>
{
  public:
    /** The base implementation */
    typedef VectorDistBase<DistStor> Base;

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
        params->buckets = rint((max - min) / bkt + 1.0);
        this->setParams(params);
        this->doInit(size);
        return *this;
    }
};

/**
 * This is a vector of StandardDeviation stats.
 * @sa Stat, VectorDistBase, FancyStor
 */
template<int N = 0>
class VectorStandardDeviation
    : public WrapVec<VectorStandardDeviation<N>,
                     VectorDistBase<FancyStor>,
                     VectorDistInfo>
{
  public:
    /** The base implementation */
    typedef VectorDistBase<FancyStor> Base;

  public:
    /**
     * Initialize storage for this distribution.
     * @param size The size of the vector.
     * @return A reference to this distribution.
     */
    VectorStandardDeviation &
    init(size_type size)
    {
        this->doInit(size);
        return *this;
    }
};

/**
 * This is a vector of AverageDeviation stats.
 * @sa Stat, VectorDistBase, AvgFancy
 */
template<int N = 0>
class VectorAverageDeviation
    : public WrapVec<VectorAverageDeviation<N>,
                     VectorDistBase<AvgFancy>,
                     VectorDistInfo>
{
  public:
    /** The base implementation */
    typedef VectorDistBase<AvgFancy> Base;

  public:
    /**
     * Initialize storage for this distribution.
     * @param size The size of the vector.
     * @return A reference to this distribution.
     */
    VectorAverageDeviation &
    init(size_type size)
    {
        this->doInit(size);
        return *this;
    }
};

/**
 * A formula for statistics that is calculated when printed. A formula is
 * stored as a tree of Nodes that represent the equation to calculate.
 * @sa Stat, ScalarStat, VectorStat, Node, Temp
 */
class FormulaBase : public InfoAccess
{
  protected:
    /** The root of the tree which represents the Formula */
    NodePtr root;
    friend class Temp;

  public:
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

    bool check() const { return true; }

    /**
     * Formulas don't need to be reset
     */
    void reset();

    /**
     *
     */
    bool zero() const;

    /**
     *
     */
    void update(Info *);

    std::string str() const;
};

class FormulaInfoBase : public VectorInfoBase
{
  public:
    virtual std::string str() const = 0;
    virtual bool check() const { return true; }
};

template <class Stat>
class FormulaInfo : public FormulaInfoBase
{
  protected:
    Stat &s;
    mutable VResult vec;
    mutable VCounter cvec;

  public:
    FormulaInfo(Stat &stat) : s(stat) {}

    virtual bool zero() const { return s.zero(); }
    virtual void reset() { s.reset(); }

    virtual size_type size() const { return s.size(); }

    virtual const VResult &
    result() const
    {
        s.result(vec);
        return vec;
    }
    virtual Result total() const { return s.total(); }
    virtual VCounter &value() const { return cvec; }

    virtual void
    visit(Visit &visitor)
    {
        update();
        s.update(this);
        visitor.visit(*this);
    }

    virtual std::string str() const { return s.str(); }
};

class Temp;
class Formula
    : public WrapVec<Formula,
                     FormulaBase,
                     FormulaInfo>
{
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
};

class FormulaNode : public Node
{
  private:
    const Formula &formula;
    mutable VResult vec;

  public:
    FormulaNode(const Formula &f) : formula(f) {}

    virtual size_type size() const { return formula.size(); }
    virtual const VResult &result() const { formula.result(vec); return vec; }
    virtual Result total() const { return formula.total(); }

    virtual std::string str() const { return formula.str(); }
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
    Temp(NodePtr n) : node(n) { }

    /**
     * Return the node pointer.
     * @return the node pointer.
     */
    operator NodePtr&() { return node; }

  public:
    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    template <int N>
    Temp(const Scalar<N> &s)
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
    template <int N>
    Temp(const Average<N> &s)
        : node(new ScalarStatNode(s.info()))
    { }

    /**
     * Create a new VectorStatNode.
     * @param s The VectorStat to place in a node.
     */
    template <int N>
    Temp(const Vector<N> &s)
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

void check();
void dump();
void reset();
void registerResetCallback(Callback *cb);

inline Temp
operator+(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::plus<Result> >(l, r));
}

inline Temp
operator-(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::minus<Result> >(l, r));
}

inline Temp
operator*(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::multiplies<Result> >(l, r));
}

inline Temp
operator/(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::divides<Result> >(l, r));
}

inline Temp
operator-(Temp l)
{
    return NodePtr(new UnaryNode<std::negate<Result> >(l));
}

template <typename T>
inline Temp
constant(T val)
{
    return NodePtr(new ConstNode<T>(val));
}

template <typename T>
inline Temp
constantVector(T val)
{
    return NodePtr(new ConstVectorNode<T>(val));
}

inline Temp
sum(Temp val)
{
    return NodePtr(new SumNode<std::plus<Result> >(val));
}

std::list<Info *> &statsList();

/* namespace Stats */ }

#endif // __BASE_STATISTICS_HH__

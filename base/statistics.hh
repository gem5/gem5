/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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
#include <cmath>
#include <functional>
#include <iosfwd>
#include <sstream>
#include <string>
#include <vector>

#include "base/cprintf.hh"
#include "base/intmath.hh"
#include "base/refcnt.hh"
#include "base/str.hh"
#include "base/stats/bin.hh"
#include "base/stats/flags.hh"
#include "base/stats/visit.hh"
#include "base/stats/types.hh"
#include "sim/host.hh"

class Callback;

/** The current simulated cycle. */
extern Tick curTick;

/* A namespace for all of the Statistics */
namespace Stats {

/* Contains the statistic implementation details */
//////////////////////////////////////////////////////////////////////
//
// Statistics Framework Base classes
//
//////////////////////////////////////////////////////////////////////
struct StatData
{
    /** The name of the stat. */
    std::string name;
    /** The description of the stat. */
    std::string desc;
    /** The formatting flags. */
    StatFlags flags;
    /** The display precision. */
    int precision;
    /** A pointer to a prerequisite Stat. */
    const StatData *prereq;
    /**
     * A unique stat ID for each stat in the simulator.
     * Can be used externally for lookups as well as for debugging.
     */
    int id;

    StatData();
    virtual ~StatData();

    /**
     * @return true if the stat is binned.
     */
    virtual bool binned() const = 0;

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
    static bool less(StatData *stat1, StatData *stat2);
};

class ScalarData : public StatData
{
  public:
    virtual Counter value() const = 0;
    virtual Result result() const = 0;
    virtual Result total() const = 0;
    virtual void visit(Visit &visitor) { visitor.visit(*this); }
};

template <class Stat>
class ScalarStatData : public ScalarData
{
  protected:
    Stat &s;

  public:
    ScalarStatData(Stat &stat) : s(stat) {}

    virtual bool binned() const { return s.binned(); }
    virtual bool check() const { return s.check(); }
    virtual Counter value() const { return s.value(); }
    virtual Result result() const { return s.result(); }
    virtual Result total() const { return s.total(); }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }
};

struct VectorData : public StatData
{
    /** Names and descriptions of subfields. */
    mutable std::vector<std::string> subnames;
    mutable std::vector<std::string> subdescs;

    virtual size_t size() const  = 0;
    virtual const VCounter &value() const = 0;
    virtual const VResult &result() const = 0;
    virtual Result total() const  = 0;
    void update()
    {
        if (!subnames.empty()) {
            int s = size();
            if (subnames.size() < s)
                subnames.resize(s);

            if (subdescs.size() < s)
                subdescs.resize(s);
        }
    }
};

template <class Stat>
class VectorStatData : public VectorData
{
  protected:
    Stat &s;
    mutable VCounter cvec;
    mutable VResult rvec;

  public:
    VectorStatData(Stat &stat) : s(stat) {}

    virtual bool binned() const { return s.binned(); }
    virtual bool check() const { return s.check(); }
    virtual bool zero() const { return s.zero(); }
    virtual void reset() { s.reset(); }

    virtual size_t size() const { return s.size(); }
    virtual VCounter &value() const
    {
        s.value(cvec);
        return cvec;
    }
    virtual const VResult &result() const
    {
        s.result(rvec);
        return rvec;
    }
    virtual Result total() const { return s.total(); }
    virtual void visit(Visit &visitor)
    {
        update();
        s.update(this);
        visitor.visit(*this);
    }
};

struct DistDataData
{
    Counter min_val;
    Counter max_val;
    Counter underflow;
    Counter overflow;
    VCounter cvec;
    Counter sum;
    Counter squares;
    Counter samples;

    Counter min;
    Counter max;
    Counter bucket_size;
    int size;
    bool fancy;
};

struct DistData : public StatData
{
    /** Local storage for the entry values, used for printing. */
    DistDataData data;
};

template <class Stat>
class DistStatData : public DistData
{
  protected:
    Stat &s;

  public:
    DistStatData(Stat &stat) : s(stat) {}

    virtual bool binned() const { return s.binned(); }
    virtual bool check() const { return s.check(); }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }
    virtual void visit(Visit &visitor)
    {
        s.update(this);
        visitor.visit(*this);
    }
};

struct VectorDistData : public StatData
{
    std::vector<DistDataData> data;

   /** Names and descriptions of subfields. */
    mutable std::vector<std::string> subnames;
    mutable std::vector<std::string> subdescs;

    /** Local storage for the entry values, used for printing. */
    mutable VResult rvec;

    virtual size_t size() const = 0;
    void update()
    {
        int s = size();
        if (subnames.size() < s)
            subnames.resize(s);

        if (subdescs.size() < s)
            subdescs.resize(s);
    }
};

template <class Stat>
class VectorDistStatData : public VectorDistData
{
  protected:
    Stat &s;
    typedef typename Stat::bin_t bin_t;

  public:
    VectorDistStatData(Stat &stat) : s(stat) {}

    virtual bool binned() const { return bin_t::binned; }
    virtual bool check() const { return s.check(); }
    virtual void reset() { s.reset(); }
    virtual size_t size() const { return s.size(); }
    virtual bool zero() const { return s.zero(); }
    virtual void visit(Visit &visitor)
    {
        update();
        s.update(this);
        visitor.visit(*this);
    }
};

struct Vector2dData : public StatData
{
    /** Names and descriptions of subfields. */
    std::vector<std::string> subnames;
    std::vector<std::string> subdescs;
    std::vector<std::string> y_subnames;

    /** Local storage for the entry values, used for printing. */
    mutable VCounter cvec;
    mutable int x;
    mutable int y;

    void update()
    {
        if (subnames.size() < x)
            subnames.resize(x);
    }
};

template <class Stat>
class Vector2dStatData : public Vector2dData
{
  protected:
    Stat &s;
    typedef typename Stat::bin_t bin_t;

  public:
    Vector2dStatData(Stat &stat) : s(stat) {}

    virtual bool binned() const { return bin_t::binned; }
    virtual bool check() const { return s.check(); }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }
    virtual void visit(Visit &visitor)
    {
        update();
        s.update(this);
        visitor.visit(*this);
    }
};


class DataAccess
{
  protected:
    StatData *find() const;
    void map(StatData *data);

    StatData *statData();
    const StatData *statData() const;

    void setInit();
    void setPrint();
};

template <class Parent, class Child, template <class> class Data>
class Wrap : public Child
{
  protected:
    Parent &self() { return *reinterpret_cast<Parent *>(this); }

  protected:
    Data<Child> *statData()
    {
        StatData *__data = DataAccess::statData();
        Data<Child> *ptr = dynamic_cast<Data<Child> *>(__data);
        assert(ptr);
        return ptr;
    }

  public:
    const Data<Child> *statData() const
    {
        const StatData *__data = DataAccess::statData();
        const Data<Child> *ptr = dynamic_cast<const Data<Child> *>(__data);
        assert(ptr);
        return ptr;
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
        this->map(new Data<Child>(*this));
    }

    /**
     * Set the name and marks this stat to print at the end of simulation.
     * @param name The new name.
     * @return A reference to this stat.
     */
    Parent &name(const std::string &_name)
    {
        Data<Child> *data = statData();
        data->name = _name;
        setPrint();
        return self();
    }

    /**
     * Set the description and marks this stat to print at the end of
     * simulation.
     * @param desc The new description.
     * @return A reference to this stat.
     */
    Parent &desc(const std::string &_desc)
    {
        statData()->desc = _desc;
        return self();
    }

    /**
     * Set the precision and marks this stat to print at the end of simulation.
     * @param p The new precision
     * @return A reference to this stat.
     */
    Parent &precision(int _precision)
    {
        statData()->precision = _precision;
        return self();
    }

    /**
     * Set the flags and marks this stat to print at the end of simulation.
     * @param f The new flags.
     * @return A reference to this stat.
     */
    Parent &flags(StatFlags _flags)
    {
        statData()->flags |= _flags;
        return self();
    }

    /**
     * Set the prerequisite stat and marks this stat to print at the end of
     * simulation.
     * @param prereq The prerequisite stat.
     * @return A reference to this stat.
     */
    template <class Stat>
    Parent &prereq(const Stat &prereq)
    {
        statData()->prereq = prereq.statData();
        return self();
    }
};

template <class Parent, class Child, template <class Child> class Data>
class WrapVec : public Wrap<Parent, Child, Data>
{
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
    Parent &subname(int index, const std::string &name)
    {
        std::vector<std::string> &subn = statData()->subnames;
        if (subn.size() <= index)
            subn.resize(index + 1);
        subn[index] = name;
        return self();
    }

    /**
     * Set the subfield description for the given index and marks this stat to
     * print at the end of simulation.
     * @param index The subfield index.
     * @param desc The new description of the subfield
     * @return A reference to this stat.
     */
    Parent &subdesc(int index, const std::string &desc)
    {
        std::vector<std::string> &subd = statData()->subdescs;
        if (subd.size() <= index)
            subd.resize(index + 1);
        subd[index] = desc;

        return self();
    }

};

template <class Parent, class Child, template <class Child> class Data>
class WrapVec2d : public WrapVec<Parent, Child, Data>
{
  public:
    /**
     * @warning This makes the assumption that if you're gonna subnames a 2d
     * vector, you're subnaming across all y
     */
    Parent &ysubnames(const char **names)
    {
        Data<Child> *data = statData();
        data->y_subnames.resize(y);
        for (int i = 0; i < y; ++i)
            data->y_subnames[i] = names[i];
        return self();
    }
    Parent &ysubname(int index, const std::string subname)
    {
        Data<Child> *data = statData();
        assert(index < y);
        data->y_subnames.resize(y);
        data->y_subnames[index] = subname.c_str();
        return self();
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
struct StatStor
{
  public:
    /** The paramaters for this storage type, none for a scalar. */
    struct Params { };

  private:
    /** The statistic value. */
    Counter data;

  public:
    /**
     * Builds this storage element and calls the base constructor of the
     * datatype.
     */
    StatStor(const Params &) : data(Counter()) {}

    /**
     * The the stat to the given value.
     * @param val The new value.
     * @param p The paramters of this storage type.
     */
    void set(Counter val, const Params &p) { data = val; }
    /**
     * Increment the stat by the given value.
     * @param val The new value.
     * @param p The paramters of this storage type.
     */
    void inc(Counter val, const Params &p) { data += val; }
    /**
     * Decrement the stat by the given value.
     * @param val The new value.
     * @param p The paramters of this storage type.
     */
    void dec(Counter val, const Params &p) { data -= val; }
    /**
     * Return the value of this stat as its base type.
     * @param p The params of this storage type.
     * @return The value of this stat.
     */
    Counter value(const Params &p) const { return data; }
    /**
     * Return the value of this stat as a result type.
     * @param p The parameters of this storage type.
     * @return The value of this stat.
     */
    Result result(const Params &p) const { return (Result)data; }
    /**
     * Reset stat value to default
     */
    void reset() { data = Counter(); }

    /**
     * @return true if zero value
     */
    bool zero() const { return data == Counter(); }
};

/**
 * Templatized storage and interface to a per-cycle average stat. This keeps
 * a current count and updates a total (count * cycles) when this count
 * changes. This allows the quick calculation of a per cycle count of the item
 * being watched. This is good for keeping track of residencies in structures
 * among other things.
 * @todo add lateny to the stat and fix binning.
 */
struct AvgStor
{
  public:
    /** The paramaters for this storage type */
    struct Params
    {
        /**
         * The current count.  We stash this here because the current
         * value is not a binned value.
         */
        Counter current;
    };

  private:
    /** The total count for all cycles. */
    mutable Result total;
    /** The cycle that current last changed. */
    mutable Tick last;

  public:
    /**
     * Build and initializes this stat storage.
     */
    AvgStor(Params &p) : total(0), last(0) { p.current = Counter(); }

    /**
     * Set the current count to the one provided, update the total and last
     * set values.
     * @param val The new count.
     * @param p The parameters for this storage.
     */
    void set(Counter val, Params &p) {
        total += p.current * (curTick - last);
        last = curTick;
        p.current = val;
    }

    /**
     * Increment the current count by the provided value, calls set.
     * @param val The amount to increment.
     * @param p The parameters for this storage.
     */
    void inc(Counter val, Params &p) { set(p.current + val, p); }

    /**
     * Deccrement the current count by the provided value, calls set.
     * @param val The amount to decrement.
     * @param p The parameters for this storage.
     */
    void dec(Counter val, Params &p) { set(p.current - val, p); }

    /**
     * Return the current count.
     * @param p The parameters for this storage.
     * @return The current count.
     */
    Counter value(const Params &p) const { return p.current; }

    /**
     * Return the current average.
     * @param p The parameters for this storage.
     * @return The current average.
     */
    Result result(const Params &p) const
    {
        total += p.current * (curTick - last);
        last = curTick;
        return (Result)(total + p.current) / (Result)(curTick + 1);
    }

    /**
     * Reset stat value to default
     */
    void reset()
    {
        total = 0;
        last = curTick;
    }

    /**
     * @return true if zero value
     */
    bool zero() const { return total == 0.0; }
};

/**
 * Implementation of a scalar stat. The type of stat is determined by the
 * Storage template. The storage for this stat is held within the Bin class.
 * This allows for breaking down statistics across multiple bins easily.
 */
template <class Storage, class Bin>
class ScalarBase : public DataAccess
{
  public:
    /** Define the params of the storage class. */
    typedef typename Storage::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::Bin<Storage> bin_t;

  protected:
    /** The bin of this stat. */
    bin_t bin;
    /** The parameters for this stat. */
    params_t params;

  protected:
    /**
     * Retrieve the storage from the bin.
     * @return The storage object for this stat.
     */
    Storage *data() { return bin.data(params); }
    /**
     * Retrieve a const pointer to the storage from the bin.
     * @return A const pointer to the storage object for this stat.
     */
    const Storage *data() const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(*_params);
    }

  public:
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    Counter value() const { return data()->value(params); }

  public:
    /**
     * Create and initialize this stat, register it with the database.
     */
    ScalarBase()
    {
        bin.init(params);
    }

  public:
    // Common operators for stats
    /**
     * Increment the stat by 1. This calls the associated storage object inc
     * function.
     */
    void operator++() { data()->inc(1, params); }
    /**
     * Decrement the stat by 1. This calls the associated storage object dec
     * function.
     */
    void operator--() { data()->dec(1, params); }

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
    void operator=(const U &v) { data()->set(v, params); }

    /**
     * Increment the stat by the given value. This calls the associated
     * storage object inc function.
     * @param v The value to add.
     */
    template <typename U>
    void operator+=(const U &v) { data()->inc(v, params); }

    /**
     * Decrement the stat by the given value. This calls the associated
     * storage object dec function.
     * @param v The value to substract.
     */
    template <typename U>
    void operator-=(const U &v) { data()->dec(v, params); }

    /**
     * Return the number of elements, always 1 for a scalar.
     * @return 1.
     */
    size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    bool binned() const { return bin_t::binned; }

    bool check() const { return bin.initialized(); }

    /**
     * Reset stat value to default
     */
    void reset() { bin.reset(); }

    Counter value() { return data()->value(params); }

    Result result() { return data()->result(params); }

    Result total() { return result(); }

    bool zero() { return result() == 0.0; }

};

class ProxyData : public ScalarData
{
  public:
    virtual void visit(Visit &visitor) { visitor.visit(*this); }
    virtual bool binned() const { return false; }
    virtual std::string str() const { return to_string(value()); }
    virtual size_t size() const { return 1; }
    virtual bool zero() const { return value() == 0; }
    virtual bool check() const { return true; }
    virtual void reset() { }
};

template <class T>
class ValueProxy : public ProxyData
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
class FunctorProxy : public ProxyData
{
  private:
    T *functor;

  public:
    FunctorProxy(T &func) : functor(&func) {}
    virtual Counter value() const { return (*functor)(); }
    virtual Result result() const { return (*functor)(); }
    virtual Result total() const { return (*functor)(); }
};

class ValueBase : public DataAccess
{
  private:
    ProxyData *proxy;

  public:
    ValueBase() : proxy(NULL) { }
    ~ValueBase() { if (proxy) delete proxy; }

    template <class T>
    void scalar(T &value)
    {
        proxy = new ValueProxy<T>(value);
        setInit();
    }

    template <class T>
    void functor(T &func)
    {
        proxy = new FunctorProxy<T>(func);
        setInit();
    }

    Counter value() { return proxy->value(); }
    Result result() const { return proxy->result(); }
    Result total() const { return proxy->total(); };
    size_t size() const { return proxy->size(); }

    bool binned() const { return proxy->binned(); }
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
template <class Storage, class Bin>
class ScalarProxy;

/**
 * Implementation of a vector of stats. The type of stat is determined by the
 * Storage class. @sa ScalarBase
 */
template <class Storage, class Bin>
class VectorBase : public DataAccess
{
  public:
    /** Define the params of the storage class. */
    typedef typename Storage::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::VectorBin<Storage> bin_t;

  protected:
    /** The bin of this stat. */
    bin_t bin;
    /** The parameters for this stat. */
    params_t params;

  protected:
    /**
     * Retrieve the storage from the bin  for the given index.
     * @param index The vector index to access.
     * @return The storage object at the given index.
     */
    Storage *data(int index) { return bin.data(index, params); }
    /**
     * Retrieve a const pointer to the storage from the bin
     * for the given index.
     * @param index The vector index to access.
     * @return A const pointer to the storage object at the given index.
     */
    const Storage *data(int index) const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(index, *_params);
    }

  public:
    void value(VCounter &vec) const
    {
        vec.resize(size());
        for (int i = 0; i < size(); ++i)
            vec[i] = data(i)->value(params);
    }

    /**
     * Copy the values to a local vector and return a reference to it.
     * @return A reference to a vector of the stat values.
     */
    void result(VResult &vec) const
    {
        vec.resize(size());
        for (int i = 0; i < size(); ++i)
            vec[i] = data(i)->result(params);
    }

    /**
     * @return True is stat is binned.
     */
    bool binned() const { return bin_t::binned; }

    /**
     * Return a total of all entries in this vector.
     * @return The total of all vector entries.
     */
    Result total() const {
        Result total = 0.0;
        for (int i = 0; i < size(); ++i)
            total += data(i)->result(params);
        return total;
    }

    /**
     * @return the number of elements in this vector.
     */
    size_t size() const { return bin.size(); }

    bool zero() const
    {
        for (int i = 0; i < size(); ++i)
            if (data(i)->zero())
                return true;
        return false;
    }

    bool check() const { return bin.initialized(); }
    void reset() { bin.reset(); }

  public:
    VectorBase() {}

    /** Friend this class with the associated scalar proxy. */
    friend class ScalarProxy<Storage, Bin>;

    /**
     * Return a reference (ScalarProxy) to the stat at the given index.
     * @param index The vector index to access.
     * @return A reference of the stat.
     */
    ScalarProxy<Storage, Bin> operator[](int index);

    void update(StatData *data) {}
};

const StatData * getStatData(const void *stat);

/**
 * A proxy class to access the stat at a given index in a VectorBase stat.
 * Behaves like a ScalarBase.
 */
template <class Storage, class Bin>
class ScalarProxy
{
  public:
    /** Define the params of the storage class. */
    typedef typename Storage::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::VectorBin<Storage> bin_t;

  private:
    /** Pointer to the bin in the parent VectorBase. */
    bin_t *bin;
    /** Pointer to the params in the parent VectorBase. */
    params_t *params;
    /** The index to access in the parent VectorBase. */
    int index;
    /** Keep a pointer to the original stat so was can get data */
    void *stat;

  protected:
    /**
     * Retrieve the storage from the bin.
     * @return The storage from the bin for this stat.
     */
    Storage *data() { return bin->data(index, *params); }
    /**
     * Retrieve a const pointer to the storage from the bin.
     * @return A const pointer to the storage for this stat.
     */
    const Storage *data() const
    {
        bin_t *_bin = const_cast<bin_t *>(bin);
        params_t *_params = const_cast<params_t *>(params);
        return _bin->data(index, *_params);
    }

  public:
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    Counter value() const { return data()->value(*params); }

    /**
     * Return the current value of this statas a result type.
     * @return The current value.
     */
    Result result() const { return data()->result(*params); }

  public:
    /**
     * Create and initialize this proxy, do not register it with the database.
     * @param b The bin to use.
     * @param p The params to use.
     * @param i The index to access.
     */
    ScalarProxy(bin_t &b, params_t &p, int i, void *s)
        : bin(&b), params(&p), index(i), stat(s)  {}
    /**
     * Create a copy of the provided ScalarProxy.
     * @param sp The proxy to copy.
     */
    ScalarProxy(const ScalarProxy &sp)
        : bin(sp.bin), params(sp.params), index(sp.index), stat(sp.stat) {}
    /**
     * Set this proxy equal to the provided one.
     * @param sp The proxy to copy.
     * @return A reference to this proxy.
     */
    const ScalarProxy &operator=(const ScalarProxy &sp) {
        bin = sp.bin;
        params = sp.params;
        index = sp.index;
        stat = sp.stat;
        return *this;
    }

  public:
    // Common operators for stats
    /**
     * Increment the stat by 1. This calls the associated storage object inc
     * function.
     */
    void operator++() { data()->inc(1, *params); }
    /**
     * Decrement the stat by 1. This calls the associated storage object dec
     * function.
     */
    void operator--() { data()->dec(1, *params); }

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
    void operator=(const U &v) { data()->set(v, *params); }

    /**
     * Increment the stat by the given value. This calls the associated
     * storage object inc function.
     * @param v The value to add.
     */
    template <typename U>
    void operator+=(const U &v) { data()->inc(v, *params); }

    /**
     * Decrement the stat by the given value. This calls the associated
     * storage object dec function.
     * @param v The value to substract.
     */
    template <typename U>
    void operator-=(const U &v) { data()->dec(v, *params); }

    /**
     * Return the number of elements, always 1 for a scalar.
     * @return 1.
     */
    size_t size() const { return 1; }

    /**
     * Return true if stat is binned.
     *@return false since Proxies aren't printed/binned
     */
    bool binned() const { return false; }

    /**
     * This stat has no state.  Nothing to reset
     */
    void reset() {  }

  public:
    const StatData *statData() const { return getStatData(stat); }
    std::string str() const
    {
        return csprintf("%s[%d]", statData()->name, index);

    }
};

template <class Storage, class Bin>
inline ScalarProxy<Storage, Bin>
VectorBase<Storage, Bin>::operator[](int index)
{
    assert (index >= 0 && index < size());
    return ScalarProxy<Storage, Bin>(bin, params, index, this);
}

template <class Storage, class Bin>
class VectorProxy;

template <class Storage, class Bin>
class Vector2dBase : public DataAccess
{
  public:
    typedef typename Storage::Params params_t;
    typedef typename Bin::VectorBin<Storage> bin_t;

  protected:
    size_t x;
    size_t y;
    bin_t bin;
    params_t params;

  protected:
    Storage *data(int index) { return bin.data(index, params); }
    const Storage *data(int index) const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(index, *_params);
    }

  public:
    Vector2dBase() {}

    void update(Vector2dData *data)
    {
        int size = this->size();
        data->cvec.resize(size);
        for (int i = 0; i < size; ++i)
            data->cvec[i] = this->data(i)->value(params);
    }

    std::string ysubname(int i) const { return (*y_subnames)[i]; }

    friend class VectorProxy<Storage, Bin>;
    VectorProxy<Storage, Bin> operator[](int index);

    size_t size() const { return bin.size(); }
    bool zero() const { return data(0)->value(params) == 0.0; }

    /**
     * Reset stat value to default
     */
    void reset() { bin.reset(); }

    bool check() { return bin.initialized(); }
};

template <class Storage, class Bin>
class VectorProxy
{
  public:
    typedef typename Storage::Params params_t;
    typedef typename Bin::VectorBin<Storage> bin_t;

  private:
    bin_t *bin;
    params_t *params;
    int offset;
    int len;
    void *stat;

  private:
    mutable VResult *vec;

    Storage *data(int index) {
        assert(index < len);
        return bin->data(offset + index, *params);
    }

    const Storage *data(int index) const {
        bin_t *_bin = const_cast<bin_t *>(bin);
        params_t *_params = const_cast<params_t *>(params);
        return _bin->data(offset + index, *_params);
    }

  public:
    const VResult &result() const {
        if (vec)
            vec->resize(size());
        else
            vec = new VResult(size());

        for (int i = 0; i < size(); ++i)
            (*vec)[i] = data(i)->result(*params);

        return *vec;
    }

    Result total() const {
        Result total = 0.0;
        for (int i = 0; i < size(); ++i)
            total += data(i)->result(*params);
        return total;
    }

  public:
    VectorProxy(bin_t &b, params_t &p, int o, int l, void *s)
        : bin(&b), params(&p), offset(o), len(l), stat(s), vec(NULL)
    {
    }

    VectorProxy(const VectorProxy &sp)
        : bin(sp.bin), params(sp.params), offset(sp.offset), len(sp.len),
          stat(sp.stat), vec(NULL)
    {
    }

    ~VectorProxy()
    {
        if (vec)
            delete vec;
    }

    const VectorProxy &operator=(const VectorProxy &sp)
    {
        bin = sp.bin;
        params = sp.params;
        offset = sp.offset;
        len = sp.len;
        stat = sp.stat;
        if (vec)
            delete vec;
        vec = NULL;
        return *this;
    }

    ScalarProxy<Storage, Bin> operator[](int index)
    {
        assert (index >= 0 && index < size());
        return ScalarProxy<Storage, Bin>(*bin, *params, offset + index, stat);
    }

    size_t size() const { return len; }

    /**
     * Return true if stat is binned.
     *@return false since Proxies aren't printed/binned
     */
    bool binned() const { return false; }

    /**
     * This stat has no state.  Nothing to reset.
     */
    void reset() { }
};

template <class Storage, class Bin>
inline VectorProxy<Storage, Bin>
Vector2dBase<Storage, Bin>::operator[](int index)
{
    int offset = index * y;
    assert (index >= 0 && offset < size());
    return VectorProxy<Storage, Bin>(bin, params, offset, y, this);
}

//////////////////////////////////////////////////////////////////////
//
// Non formula statistics
//
//////////////////////////////////////////////////////////////////////

/**
 * Templatized storage and interface for a distrbution stat.
 */
struct DistStor
{
  public:
    /** The parameters for a distribution stat. */
    struct Params
    {
        /** The minimum value to track. */
        Counter min;
        /** The maximum value to track. */
        Counter max;
        /** The number of entries in each bucket. */
        Counter bucket_size;
        /** The number of buckets. Equal to (max-min)/bucket_size. */
        int size;
    };
    enum { fancy = false };

  private:
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
    /**
     * Construct this storage with the supplied params.
     * @param params The parameters.
     */
    DistStor(const Params &params)
        : min_val(INT_MAX), max_val(INT_MIN), underflow(Counter()),
          overflow(Counter()), sum(Counter()), squares(Counter()),
          samples(Counter()), cvec(params.size)
    {
        reset();
    }

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     * @param params The paramters of the distribution.
     */
    void sample(Counter val, int number, const Params &params)
    {
        if (val < params.min)
            underflow += number;
        else if (val > params.max)
            overflow += number;
        else {
            int index = (int)floor((val - params.min) / params.bucket_size);
            assert(index < size(params));
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
     * @todo Is it faster to return the size from the parameters?
     */
    size_t size(const Params &) const { return cvec.size(); }

    /**
     * Returns true if any calls to sample have been made.
     * @param params The paramters of the distribution.
     * @return True if any values have been sampled.
     */
    bool zero(const Params &params) const
    {
        return samples == Counter();
    }

    void update(DistDataData *data, const Params &params)
    {
        data->min = params.min;
        data->max = params.max;
        data->bucket_size = params.bucket_size;
        data->size = params.size;

        data->min_val = (min_val == INT_MAX) ? 0 : min_val;
        data->max_val = (max_val == INT_MIN) ? 0 : max_val;
        data->underflow = underflow;
        data->overflow = overflow;
        data->cvec.resize(params.size);
        for (int i = 0; i < params.size; ++i)
            data->cvec[i] = cvec[i];

        data->sum = sum;
        data->squares = squares;
        data->samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void reset()
    {
        min_val = INT_MAX;
        max_val = INT_MIN;
        underflow = 0;
        overflow = 0;

        int size = cvec.size();
        for (int i = 0; i < size; ++i)
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
struct FancyStor
{
  public:
    /**
     * No paramters for this storage.
     */
    struct Params {};
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
    FancyStor(const Params &)
        : sum(Counter()), squares(Counter()), samples(Counter())
    { }

    /**
     * Add a value the given number of times to this running average.
     * Update the running sum and sum of squares, increment the number of
     * values seen by the given number.
     * @param val The value to add.
     * @param number The number of times to add the value.
     * @param p The parameters of this stat.
     */
    void sample(Counter val, int number, const Params &p)
    {
        Counter value = val * number;
        sum += value;
        squares += value * value;
        samples += number;
    }

    void update(DistDataData *data, const Params &params)
    {
        data->sum = sum;
        data->squares = squares;
        data->samples = samples;
    }

    /**
     * Return the number of entries in this stat, 1
     * @return 1.
     */
    size_t size(const Params &) const { return 1; }

    /**
     * Return true if no samples have been added.
     * @return True if no samples have been added.
     */
    bool zero(const Params &) const { return samples == Counter(); }

    /**
     * Reset stat value to default
     */
    void reset()
    {
        sum = Counter();
        squares = Counter();
        samples = Counter();
    }
};

/**
 * Templatized storage for distribution that calculates per cycle mean and
 * variance.
 */
struct AvgFancy
{
  public:
    /** No parameters for this storage. */
    struct Params {};
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
    AvgFancy(const Params &) : sum(Counter()), squares(Counter()) {}

    /**
     * Add a value to the distribution for the given number of times.
     * Update the running sum and sum of squares.
     * @param val The value to add.
     * @param number The number of times to add the value.
     * @param p The paramters of the distribution.
     */
    void sample(Counter val, int number, const Params &p)
    {
        Counter value = val * number;
        sum += value;
        squares += value * value;
    }

    void update(DistDataData *data, const Params &params)
    {
        data->sum = sum;
        data->squares = squares;
        data->samples = curTick;
    }

    /**
     * Return the number of entries, in this case 1.
     * @return 1.
     */
    size_t size(const Params &params) const { return 1; }
    /**
     * Return true if no samples have been added.
     * @return True if the sum is zero.
     */
    bool zero(const Params &params) const { return sum == Counter(); }
    /**
     * Reset stat value to default
     */
    void reset()
    {
        sum = Counter();
        squares = Counter();
    }
};

/**
 * Implementation of a distribution stat. The type of distribution is
 * determined by the Storage template. @sa ScalarBase
 */
template <class Storage, class Bin>
class DistBase : public DataAccess
{
  public:
    /** Define the params of the storage class. */
    typedef typename Storage::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::Bin<Storage> bin_t;

  protected:
    /** The bin of this stat. */
    bin_t bin;
    /** The parameters for this stat. */
    params_t params;

  protected:
    /**
     * Retrieve the storage from the bin.
     * @return The storage object for this stat.
     */
    Storage *data() { return bin.data(params); }
    /**
     * Retrieve a const pointer to the storage from the bin.
     * @return A const pointer to the storage object for this stat.
     */
    const Storage *data() const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(*_params);
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
    void sample(const U &v, int n = 1) { data()->sample(v, n, params); }

    /**
     * Return the number of entries in this stat.
     * @return The number of entries.
     */
    size_t size() const { return data()->size(params); }
    /**
     * Return true if no samples have been added.
     * @return True if there haven't been any samples.
     */
    bool zero() const { return data()->zero(params); }

    void update(DistData *base)
    {
        base->data.fancy = Storage::fancy;
        data()->update(&(base->data), params);
    }
    /**
     * @return True is stat is binned.
     */
    bool binned() const { return bin_t::binned; }
    /**
     * Reset stat value to default
     */
    void reset()
    {
        bin.reset();
    }

    bool check() { return bin.initialized(); }
};

template <class Storage, class Bin>
class DistProxy;

template <class Storage, class Bin>
class VectorDistBase : public DataAccess
{
  public:
    typedef typename Storage::Params params_t;
    typedef typename Bin::VectorBin<Storage> bin_t;

  protected:
    bin_t bin;
    params_t params;

  protected:
    Storage *data(int index) { return bin.data(index, params); }
    const Storage *data(int index) const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(index, *_params);
    }

  public:
    VectorDistBase() {}

    friend class DistProxy<Storage, Bin>;
    DistProxy<Storage, Bin> operator[](int index);
    const DistProxy<Storage, Bin> operator[](int index) const;

    size_t size() const { return bin.size(); }
    bool zero() const { return false; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    bool binned() const { return bin_t::binned; }
    /**
     * Reset stat value to default
     */
    void reset() { bin.reset(); }

    bool check() { return bin.initialized(); }
    void update(VectorDistData *base)
    {
        int size = this->size();
        base->data.resize(size);
        for (int i = 0; i < size; ++i) {
            base->data[i].fancy = Storage::fancy;
            data(i)->update(&(base->data[i]), params);
        }
    }
};

template <class Storage, class Bin>
class DistProxy
{
  public:
    typedef typename Storage::Params params_t;
    typedef typename Bin::Bin<Storage> bin_t;
    typedef VectorDistBase<Storage, Bin> base_t;

  private:
    union {
        base_t *stat;
        const base_t *cstat;
    };
    int index;

  protected:
    Storage *data() { return stat->data(index); }
    const Storage *data() const { return cstat->data(index); }

  public:
    DistProxy(const VectorDistBase<Storage, Bin> &s, int i)
        : cstat(&s), index(i) {}
    DistProxy(const DistProxy &sp)
        : cstat(sp.cstat), index(sp.index) {}
    const DistProxy &operator=(const DistProxy &sp) {
        cstat = sp.cstat; index = sp.index; return *this;
    }

  public:
    template <typename U>
    void sample(const U &v, int n = 1) { data()->sample(v, n, cstat->params); }

    size_t size() const { return 1; }
    bool zero() const { return data()->zero(cstat->params); }
    /**
     * Return true if stat is binned.
     *@return false since Proxies are not binned/printed.
     */
    bool binned() const { return false; }
    /**
     * Proxy has no state.  Nothing to reset.
     */
    void reset() { }
};

template <class Storage, class Bin>
inline DistProxy<Storage, Bin>
VectorDistBase<Storage, Bin>::operator[](int index)
{
    assert (index >= 0 && index < size());
    return DistProxy<Storage, Bin>(*this, index);
}

template <class Storage, class Bin>
inline const DistProxy<Storage, Bin>
VectorDistBase<Storage, Bin>::operator[](int index) const
{
    assert (index >= 0 && index < size());
    return DistProxy<Storage, Bin>(*this, index);
}

#if 0
template <class Storage, class Bin>
Result
VectorDistBase<Storage, Bin>::total(int index) const
{
    int total = 0;
    for (int i=0; i < x_size(); ++i) {
        total += data(i)->result(*params);
    }
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
    virtual size_t size() const = 0;
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
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const = 0;

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
    const ScalarData *data;
    mutable VResult vresult;

  public:
    ScalarStatNode(const ScalarData *d) : data(d), vresult(1) {}
    virtual const VResult &result() const
    {
        vresult[0] = data->result();
        return vresult;
    }
    virtual Result total() const { return data->result(); };

    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return data->binned(); }

    /**
     *
     */
    virtual std::string str() const { return data->name; }
};

template <class Storage, class Bin>
class ScalarProxyNode : public Node
{
  private:
    const ScalarProxy<Storage, Bin> proxy;
    mutable VResult vresult;

  public:
    ScalarProxyNode(const ScalarProxy<Storage, Bin> &p)
        : proxy(p), vresult(1) { }
    virtual const VResult &result() const
    {
        vresult[0] = proxy.result();
        return vresult;
    }
    virtual Result total() const { return proxy.result(); };

    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return proxy.binned(); }

    /**
     *
     */
    virtual std::string str() const { return proxy.str(); }
};

class VectorStatNode : public Node
{
  private:
    const VectorData *data;

  public:
    VectorStatNode(const VectorData *d) : data(d) { }
    virtual const VResult &result() const { return data->result(); }
    virtual Result total() const { return data->total(); };

    virtual size_t size() const { return data->size(); }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return data->binned(); }

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
    virtual size_t size() const { return 1; }

    /**
     * Return true if stat is binned.
     *@return False since constants aren't binned.
     */
    virtual bool binned() const { return false; }

    virtual std::string str() const { return to_string(vresult[0]); }
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

    const VResult &result() const
    {
        const VResult &lvec = l->result();
        int size = lvec.size();

        assert(size > 0);

        vresult.resize(size);
        Op op;
        for (int i = 0; i < size; ++i)
            vresult[i] = op(lvec[i]);

        return vresult;
    }

    Result total() const {
        Op op;
        return op(l->total());
    }

    virtual size_t size() const { return l->size(); }
    /**
     * Return true if child of node is binned.
     *@return True if child of node is binned.
     */
    virtual bool binned() const { return l->binned(); }

    virtual std::string str() const
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

    const VResult &result() const
    {
        Op op;
        const VResult &lvec = l->result();
        const VResult &rvec = r->result();

        assert(lvec.size() > 0 && rvec.size() > 0);

        if (lvec.size() == 1 && rvec.size() == 1) {
            vresult.resize(1);
            vresult[0] = op(lvec[0], rvec[0]);
        } else if (lvec.size() == 1) {
            int size = rvec.size();
            vresult.resize(size);
            for (int i = 0; i < size; ++i)
                vresult[i] = op(lvec[0], rvec[i]);
        } else if (rvec.size() == 1) {
            int size = lvec.size();
            vresult.resize(size);
            for (int i = 0; i < size; ++i)
                vresult[i] = op(lvec[i], rvec[0]);
        } else if (rvec.size() == lvec.size()) {
            int size = rvec.size();
            vresult.resize(size);
            for (int i = 0; i < size; ++i)
                vresult[i] = op(lvec[i], rvec[i]);
        }

        return vresult;
    }

    Result total() const {
        Op op;
        return op(l->total(), r->total());
    }

    virtual size_t size() const {
        int ls = l->size();
        int rs = r->size();
        if (ls == 1)
            return rs;
        else if (rs == 1)
            return ls;
        else {
            assert(ls == rs && "Node vector sizes are not equal");
            return ls;
        }
    }
    /**
     * Return true if any children of node are binned
     *@return True if either child of node is binned.
     */
    virtual bool binned() const { return (l->binned() || r->binned()); }

    virtual std::string str() const
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

    const VResult &result() const
    {
        const VResult &lvec = l->result();
        int size = lvec.size();
        assert(size > 0);

        vresult[0] = 0.0;

        Op op;
        for (int i = 0; i < size; ++i)
            vresult[0] = op(vresult[0], lvec[i]);

        return vresult;
    }

    Result total() const
    {
        const VResult &lvec = l->result();
        int size = lvec.size();
        assert(size > 0);

        Result vresult = 0.0;

        Op op;
        for (int i = 0; i < size; ++i)
            vresult = op(vresult, lvec[i]);

        return vresult;
    }

    virtual size_t size() const { return 1; }
    /**
     * Return true if child of node is binned.
     *@return True if child of node is binned.
     */
    virtual bool binned() const { return l->binned(); }

    virtual std::string str() const
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
 * These are the statistics that are used in the simulator. By default these
 * store counters and don't use binning, but are templatized to accept any type
 * and any Bin class.
 * @{
 */

/**
 * This is an easy way to assign all your stats to be binned or not
 * binned.  If the typedef is NoBin, nothing is binned.  If it is
 * MainBin, then all stats are binned under that Bin.
 */
#if defined(FS_MEASURE) || defined(STATS_BINNING)
typedef MainBin DefaultBin;
#else
typedef NoBin DefaultBin;
#endif

/**
 * This is a simple scalar statistic, like a counter.
 * @sa Stat, ScalarBase, StatStor
 */
template <class Bin = DefaultBin>
class Scalar
    : public Wrap<Scalar<Bin>,
                  ScalarBase<StatStor, Bin>,
                  ScalarStatData>
{
  public:
    /** The base implementation. */
    typedef ScalarBase<StatStor, Bin> Base;

    Scalar()
    {
        setInit();
    }

    /**
     * Sets the stat equal to the given value. Calls the base implementation
     * of operator=
     * @param v The new value.
     */
    template <typename U>
    void operator=(const U &v) { Base::operator=(v); }
};

class Value
    : public Wrap<Value,
                  ValueBase,
                  ScalarStatData>
{
  public:
    /** The base implementation. */
    typedef ValueBase Base;

    template <class T>
    Value &scalar(T &value)
    {
        Base::scalar(value);
        return *this;
    }

    template <class T>
    Value &functor(T &func)
    {
        Base::functor(func);
        return *this;
    }
};

/**
 * A stat that calculates the per cycle average of a value.
 * @sa Stat, ScalarBase, AvgStor
 */
template <class Bin = DefaultBin>
class Average
    : public Wrap<Average<Bin>,
                  ScalarBase<AvgStor, Bin>,
                  ScalarStatData>
{
  public:
    /** The base implementation. */
    typedef ScalarBase<AvgStor, Bin> Base;

    Average()
    {
        setInit();
    }

    /**
     * Sets the stat equal to the given value. Calls the base implementation
     * of operator=
     * @param v The new value.
     */
    template <typename U>
    void operator=(const U &v) { Base::operator=(v); }
};

/**
 * A vector of scalar stats.
 * @sa Stat, VectorBase, StatStor
 */
template <class Bin = DefaultBin>
class Vector
    : public WrapVec<Vector<Bin>,
                     VectorBase<StatStor, Bin>,
                     VectorStatData>
{
  public:
    /** The base implementation. */
    typedef ScalarBase<StatStor, Bin> Base;

    /**
     * Set this vector to have the given size.
     * @param size The new size.
     * @return A reference to this stat.
     */
    Vector &init(size_t size) {
        bin.init(size, params);
        setInit();

        return *this;
    }
};

/**
 * A vector of Average stats.
 * @sa Stat, VectorBase, AvgStor
 */
template <class Bin = DefaultBin>
class AverageVector
    : public WrapVec<AverageVector<Bin>,
                     VectorBase<AvgStor, Bin>,
                     VectorStatData>
{
  public:
    /**
     * Set this vector to have the given size.
     * @param size The new size.
     * @return A reference to this stat.
     */
    AverageVector &init(size_t size) {
        bin.init(size, params);
        setInit();

        return *this;
    }
};

/**
 * A 2-Dimensional vecto of scalar stats.
 * @sa Stat, Vector2dBase, StatStor
 */
template <class Bin = DefaultBin>
class Vector2d
    : public WrapVec2d<Vector2d<Bin>,
                       Vector2dBase<StatStor, Bin>,
                       Vector2dStatData>
{
  public:
    Vector2d &init(size_t _x, size_t _y) {
        statData()->x = x = _x;
        statData()->y = y = _y;
        bin.init(x * y, params);
        setInit();

        return *this;
    }
};

/**
 * A simple distribution stat.
 * @sa Stat, DistBase, DistStor
 */
template <class Bin = DefaultBin>
class Distribution
    : public Wrap<Distribution<Bin>,
                  DistBase<DistStor, Bin>,
                  DistStatData>
{
  public:
    /** Base implementation. */
    typedef DistBase<DistStor, Bin> Base;
    /** The Parameter type. */
    typedef typename DistStor::Params Params;

  public:
    /**
     * Set the parameters of this distribution. @sa DistStor::Params
     * @param min The minimum value of the distribution.
     * @param max The maximum value of the distribution.
     * @param bkt The number of values in each bucket.
     * @return A reference to this distribution.
     */
    Distribution &init(Counter min, Counter max, Counter bkt) {
        params.min = min;
        params.max = max;
        params.bucket_size = bkt;
        params.size = (int)rint((max - min) / bkt + 1.0);
        bin.init(params);
        setInit();

        return *this;
    }
};

/**
 * Calculates the mean and variance of all the samples.
 * @sa Stat, DistBase, FancyStor
 */
template <class Bin = DefaultBin>
class StandardDeviation
    : public Wrap<StandardDeviation<Bin>,
                  DistBase<FancyStor, Bin>,
                  DistStatData>
{
  public:
    /** The base implementation */
    typedef DistBase<DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor::Params Params;

  public:
    /**
     * Construct and initialize this distribution.
     */
    StandardDeviation() {
        bin.init(params);
        setInit();
    }
};

/**
 * Calculates the per cycle mean and variance of the samples.
 * @sa Stat, DistBase, AvgFancy
 */
template <class Bin = DefaultBin>
class AverageDeviation
    : public Wrap<AverageDeviation<Bin>,
                  DistBase<AvgFancy, Bin>,
                  DistStatData>
{
  public:
    /** The base implementation */
    typedef DistBase<DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor::Params Params;

  public:
    /**
     * Construct and initialize this distribution.
     */
    AverageDeviation()
    {
        bin.init(params);
        setInit();
    }
};

/**
 * A vector of distributions.
 * @sa Stat, VectorDistBase, DistStor
 */
template <class Bin = DefaultBin>
class VectorDistribution
    : public WrapVec<VectorDistribution<Bin>,
                     VectorDistBase<DistStor, Bin>,
                     VectorDistStatData>
{
  public:
    /** The base implementation */
    typedef VectorDistBase<DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor::Params Params;

  public:
    /**
     * Initialize storage and parameters for this distribution.
     * @param size The size of the vector (the number of distributions).
     * @param min The minimum value of the distribution.
     * @param max The maximum value of the distribution.
     * @param bkt The number of values in each bucket.
     * @return A reference to this distribution.
     */
    VectorDistribution &init(int size, Counter min, Counter max, Counter bkt) {
        params.min = min;
        params.max = max;
        params.bucket_size = bkt;
        params.size = (int)rint((max - min) / bkt + 1.0);
        bin.init(size, params);
        setInit();

        return *this;
    }
};

/**
 * This is a vector of StandardDeviation stats.
 * @sa Stat, VectorDistBase, FancyStor
 */
template <class Bin = DefaultBin>
class VectorStandardDeviation
    : public WrapVec<VectorStandardDeviation<Bin>,
                     VectorDistBase<FancyStor, Bin>,
                     VectorDistStatData>
{
  public:
    /** The base implementation */
    typedef VectorDistBase<FancyStor, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor::Params Params;

  public:
    /**
     * Initialize storage for this distribution.
     * @param size The size of the vector.
     * @return A reference to this distribution.
     */
    VectorStandardDeviation &init(int size) {
        bin.init(size, params);
        setInit();

        return *this;
    }
};

/**
 * This is a vector of AverageDeviation stats.
 * @sa Stat, VectorDistBase, AvgFancy
 */
template <class Bin = DefaultBin>
class VectorAverageDeviation
    : public WrapVec<VectorAverageDeviation<Bin>,
                     VectorDistBase<AvgFancy, Bin>,
                     VectorDistStatData>
{
  public:
    /** The base implementation */
    typedef VectorDistBase<AvgFancy, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor::Params Params;

  public:
    /**
     * Initialize storage for this distribution.
     * @param size The size of the vector.
     * @return A reference to this distribution.
     */
    VectorAverageDeviation &init(int size) {
        bin.init(size, params);
        setInit();

        return *this;
    }
};

/**
 * A formula for statistics that is calculated when printed. A formula is
 * stored as a tree of Nodes that represent the equation to calculate.
 * @sa Stat, ScalarStat, VectorStat, Node, Temp
 */
class FormulaBase : public DataAccess
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
    size_t size() const;

    /**
     * Return true if Formula is binned. i.e. any of its children
     * nodes are binned
     * @return True if Formula is binned.
     */
    bool binned() const;

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
    void update(StatData *);

    std::string str() const;
};

class FormulaData : public VectorData
{
  public:
    virtual std::string str() const = 0;
    virtual bool check() const { return true; }
};

template <class Stat>
class FormulaStatData : public FormulaData
{
  protected:
    Stat &s;
    mutable VResult vec;
    mutable VCounter cvec;

  public:
    FormulaStatData(Stat &stat) : s(stat) {}

    virtual bool binned() const { return s.binned(); }
    virtual bool zero() const { return s.zero(); }
    virtual void reset() { s.reset(); }

    virtual size_t size() const { return s.size(); }
    virtual const VResult &result() const
    {
        s.result(vec);
        return vec;
    }
    virtual Result total() const { return s.total(); }
    virtual VCounter &value() const { return cvec; }
    virtual void visit(Visit &visitor)
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
                     FormulaStatData>
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

    virtual size_t size() const { return formula.size(); }
    virtual const VResult &result() const { formula.result(vec); return vec; }
    virtual Result total() const { return formula.total(); }
    virtual bool binned() const { return formula.binned(); }

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
    operator NodePtr&() { return node;}

  public:
    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    template <class Bin>
    Temp(const Scalar<Bin> &s)
        : node(new ScalarStatNode(s.statData())) { }

    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    Temp(const Value &s)
        : node(new ScalarStatNode(s.statData())) { }

    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    template <class Bin>
    Temp(const Average<Bin> &s)
        : node(new ScalarStatNode(s.statData())) { }

    /**
     * Create a new VectorStatNode.
     * @param s The VectorStat to place in a node.
     */
    template <class Bin>
    Temp(const Vector<Bin> &s)
        : node(new VectorStatNode(s.statData())) { }

    /**
     *
     */
    Temp(const Formula &f)
        : node(new FormulaNode(f)) { }

    /**
     * Create a new ScalarProxyNode.
     * @param p The ScalarProxy to place in a node.
     */
    template <class Storage, class Bin>
    Temp(const ScalarProxy<Storage, Bin> &p)
        : node(new ScalarProxyNode<Storage, Bin>(p)) { }

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed char value)
        : node(new ConstNode<signed char>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned char value)
        : node(new ConstNode<unsigned char>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed short value)
        : node(new ConstNode<signed short>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned short value)
        : node(new ConstNode<unsigned short>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed int value)
        : node(new ConstNode<signed int>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned int value)
        : node(new ConstNode<unsigned int>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed long value)
        : node(new ConstNode<signed long>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned long value)
        : node(new ConstNode<unsigned long>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed long long value)
        : node(new ConstNode<signed long long>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned long long value)
        : node(new ConstNode<unsigned long long>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(float value)
        : node(new ConstNode<float>(value)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(double value)
        : node(new ConstNode<double>(value)) {}
};


/**
 * @}
 */

void check();
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
operator%(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::modulus<Result> >(l, r));
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

inline Temp
sum(Temp val)
{
    return NodePtr(new SumNode<std::plus<Result> >(val));
}

/* namespace Stats */ }

#endif // __BASE_STATISTICS_HH__

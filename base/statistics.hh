/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
#ifndef __STATISTICS_HH__
#define __STATISTICS_HH__

#include <algorithm>
#include <functional>
#include <iosfwd>
#include <sstream>
#include <string>
#include <vector>

#include <assert.h>

#include "base/refcnt.hh"
#include "base/str.hh"
#include "base/intmath.hh"
#include <math.h>
#include "sim/host.hh"

#ifdef FS_MEASURE
#include "base/trace.hh"
#endif
//
//  Un-comment this to enable weirdo-stat debugging
//
// #define STAT_DEBUG


#ifndef NAN
float __nan();
/** Define Not a number. */
#define NAN (__nan())
/** Need to define __nan() */
#define __M5_NAN
#endif

class Callback;

/** The current simulated cycle. */
extern Tick curTick;

/* A namespace for all of the Statistics */
namespace Statistics {
/** All results are doubles. */
typedef double result_t;
/** A vector to hold results. */
typedef std::vector<result_t> rvec_t;

/**
 * Define the storage for format flags.
 * @todo Can probably shrink this.
 */
typedef u_int32_t FormatFlags;
/** Nothing extra to print. */
const FormatFlags none =	0x0000;
/** Print the total. */
const FormatFlags total =	0x0001;
/** Print the percent of the total that this entry represents. */
const FormatFlags pdf =		0x0002;
/** Print the cumulative percentage of total upto this entry. */
const FormatFlags cdf =		0x0004;
/** Don't print if this is zero. */
const FormatFlags nozero =	0x0010;
/** Don't print if this is NAN */
const FormatFlags nonan =	0x0020;
/** Print the distribution. */
const FormatFlags dist = 	0x0100;
/** Used for SS compatability. */
const FormatFlags __substat = 	0x8000;
/** Mask of flags that can't be set directly */
const FormatFlags __reserved =  __substat;

enum DisplayMode
{
    mode_m5,
    mode_simplescalar,
    mode_python
};

extern DisplayMode default_mode;

/* Contains the statistic implementation details */
//////////////////////////////////////////////////////////////////////
//
// Statistics Framework Base classes
//
//////////////////////////////////////////////////////////////////////
struct StatData
{
    /** True if the stat has been initialized. */
    bool init;
    /** True if the stat should be printed. */
    bool print;
    /** The name of the stat. */
    std::string name;
    /** The description of the stat. */
    std::string desc;
    /** The display precision. */
    int precision;
    /** Display Mode */
    DisplayMode mode;
    /** The formatting flags. */
    FormatFlags flags;


    /** A pointer to a prerequisite Stat. */
    const StatData *prereq;

    StatData()
        : init(false), print(false), precision(-1), mode(default_mode),
          flags(0), prereq(0)
    {}

    virtual ~StatData();

    /**
     * @return true if the stat is binned.
     */
    virtual bool binned() const = 0;

    /**
     * Print this stat to the given ostream.
     * @param stream The stream to print to.
     */
    virtual void display(std::ostream &stream) const = 0;
    bool dodisplay() const { return !prereq || !prereq->zero(); }

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
    virtual bool check() const;

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

struct ScalarDataBase : public StatData
{
    virtual result_t val() const = 0;
    virtual result_t total() const = 0;

    virtual void display(std::ostream &stream) const;
};

template <class T>
class ScalarData : public ScalarDataBase
{
  protected:
    T &s;

  public:
    ScalarData(T &stat) : s(stat) {}

    virtual bool binned() const { return s.binned(); }
    virtual result_t val() const { return s.val(); }
    virtual result_t total() const { return s.total(); }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }
};

struct VectorDataBase : public StatData
{
    /** Names and descriptions of subfields. */
    mutable std::vector<std::string> subnames;
    mutable std::vector<std::string> subdescs;

    virtual void display(std::ostream &stream) const;

    virtual size_t size() const  = 0;
    virtual const rvec_t &val() const  = 0;
    virtual result_t total() const  = 0;
    virtual void update()
    {
        int s = size();
        if (subnames.size() < s)
            subnames.resize(s);

        if (subdescs.size() < s)
            subdescs.resize(s);
    }
};

template <class T>
class VectorData : public VectorDataBase
{
  protected:
    T &s;
    mutable rvec_t vec;

  public:
    VectorData(T &stat) : s(stat) {}

    virtual bool binned() const { return s.binned(); }
    virtual bool zero() const { return s.zero(); }
    virtual void reset() { s.reset(); }

    virtual size_t size() const { return s.size(); }
    virtual const rvec_t &val() const
    {
        s.val(vec);
        return vec;
    }
    virtual result_t total() const { return s.total(); }
    virtual void update()
    {
        VectorDataBase::update();
        s.update(this);
    }
};


struct DistDataData
{
    result_t min_val;
    result_t max_val;
    result_t underflow;
    result_t overflow;
    rvec_t vec;
    result_t sum;
    result_t squares;
    result_t samples;

    int min;
    int max;
    int bucket_size;
    int size;
    bool fancy;
};

struct DistDataBase : public StatData
{
    /** Local storage for the entry values, used for printing. */
    DistDataData data;

    virtual void display(std::ostream &stream) const;
    virtual void update() = 0;
};

template <class T>
class DistData : public DistDataBase
{
  protected:
    T &s;

  public:
    DistData(T &stat) : s(stat) {}

    virtual bool binned() const { return s.binned(); }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }
    virtual void update() { return s.update(this); }
};

struct VectorDistDataBase : public StatData
{
    std::vector<DistDataData> data;

   /** Names and descriptions of subfields. */
    mutable std::vector<std::string> subnames;
    mutable std::vector<std::string> subdescs;

    /** Local storage for the entry values, used for printing. */
    mutable rvec_t vec;

    virtual size_t size() const = 0;
    virtual void display(std::ostream &stream) const;
    virtual void update()
    {
        int s = size();
        if (subnames.size() < s)
            subnames.resize(s);

        if (subdescs.size() < s)
            subdescs.resize(s);
    }
};

template <class T>
class VectorDistData : public VectorDistDataBase
{
  protected:
    T &s;

  public:
    VectorDistData(T &stat) : s(stat) {}

    virtual bool binned() const { return T::bin_t::binned; }
    virtual void reset() { s.reset(); }
    virtual size_t size() const { return s.size(); }
    virtual bool zero() const { return s.zero(); }
    virtual void update()
    {
        VectorDistDataBase::update();
        return s.update(this);
    }
};

struct Vector2dDataBase : public StatData
{
    /** Names and descriptions of subfields. */
    std::vector<std::string> subnames;
    std::vector<std::string> subdescs;
    std::vector<std::string> y_subnames;

    /** Local storage for the entry values, used for printing. */
    mutable rvec_t vec;
    mutable int x;
    mutable int y;

    virtual void display(std::ostream &stream) const;
    virtual void update()
    {
        if (subnames.size() < x)
            subnames.resize(x);
    }
};

template <class T>
class Vector2dData : public Vector2dDataBase
{
  protected:
    T &s;

  public:
    Vector2dData(T &stat) : s(stat) {}

    virtual bool binned() const { return T::bin_t::binned; }
    virtual void reset() { s.reset(); }
    virtual bool zero() const { return s.zero(); }
    virtual void update()
    {
        Vector2dDataBase::update();
        return s.update(this);
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

template <class Parent, class Child, template <class Child> class Data>
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

  public:
    Wrap()
    {
        map(new Data<Child>(*this));
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
    Parent &flags(FormatFlags _flags)
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
    template <class T>
    Parent &prereq(const T &prereq)
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
        assert(i < y);
        data->y_subnames.resize(y);
        data->y_subnames[i] = subname.c_str();
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
template <typename T>
struct StatStor
{
  public:
    /** The paramaters for this storage type, none for a scalar. */
    struct Params { };

  private:
    /** The statistic value. */
    T data;
    static T &Null()
    {
        static T __T = T();
        return __T;
    }

  public:
    /**
     * Builds this storage element and calls the base constructor of the
     * datatype.
     */
    StatStor(const Params &) : data(Null()) {}

    /**
     * The the stat to the given value.
     * @param val The new value.
     * @param p The paramters of this storage type.
     */
    void set(T val, const Params &p) { data = val; }
    /**
     * Increment the stat by the given value.
     * @param val The new value.
     * @param p The paramters of this storage type.
     */
    void inc(T val, const Params &p) { data += val; }
    /**
     * Decrement the stat by the given value.
     * @param val The new value.
     * @param p The paramters of this storage type.
     */
    void dec(T val, const Params &p) { data -= val; }
    /**
     * Return the value of this stat as a result type.
     * @param p The parameters of this storage type.
     * @return The value of this stat.
     */
    result_t val(const Params &p) const { return (result_t)data; }
    /**
     * Return the value of this stat as its base type.
     * @param p The params of this storage type.
     * @return The value of this stat.
     */
    T value(const Params &p) const { return data; }
    /**
     * Reset stat value to default
     */
    void reset() { data = Null(); }

    /**
     * @return true if zero value
     */
    bool zero() const { return data == Null(); }
};

/**
 * Templatized storage and interface to a per-cycle average stat. This keeps
 * a current count and updates a total (count * cycles) when this count
 * changes. This allows the quick calculation of a per cycle count of the item
 * being watched. This is good for keeping track of residencies in structures
 * among other things.
 * @todo add lateny to the stat and fix binning.
 */
template <typename T>
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
        T current;
    };

  private:
    /** The total count for all cycles. */
    mutable result_t total;
    /** The cycle that current last changed. */
    mutable Tick last;

  public:
    /**
     * Build and initializes this stat storage.
     */
    AvgStor(Params &p) : total(0), last(0) { p.current = T(); }

    /**
     * Set the current count to the one provided, update the total and last
     * set values.
     * @param val The new count.
     * @param p The parameters for this storage.
     */
    void set(T val, Params &p) {
        total += p.current * (curTick - last);
        last = curTick;
        p.current = val;
    }

    /**
     * Increment the current count by the provided value, calls set.
     * @param val The amount to increment.
     * @param p The parameters for this storage.
     */
    void inc(T val, Params &p) { set(p.current + val, p); }

    /**
     * Deccrement the current count by the provided value, calls set.
     * @param val The amount to decrement.
     * @param p The parameters for this storage.
     */
    void dec(T val, Params &p) { set(p.current - val, p); }

    /**
     * Return the current average.
     * @param p The parameters for this storage.
     * @return The current average.
     */
    result_t val(const Params &p) const {
        total += p.current * (curTick - last);
        last = curTick;
        return (result_t)(total + p.current) / (result_t)(curTick + 1);
    }

    /**
     * Return the current count.
     * @param p The parameters for this storage.
     * @return The current count.
     */
    T value(const Params &p) const { return p.current; }

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
template <typename T, template <typename T> class Storage, class Bin>
class ScalarBase : public DataAccess
{
  protected:
    /** Define the type of the storage class. */
    typedef Storage<T> storage_t;
    /** Define the params of the storage class. */
    typedef typename storage_t::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::Bin<storage_t> bin_t;

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
    storage_t *data() { return bin.data(params); }
    /**
     * Retrieve a const pointer to the storage from the bin.
     * @return A const pointer to the storage object for this stat.
     */
    const storage_t *data() const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(*_params);
    }

  protected:
    /**
     * Copy constructor, copies are not allowed.
     */
    ScalarBase(const ScalarBase &stat);
    /**
     * Can't copy stats.
     */
    const ScalarBase &operator=(const ScalarBase &);

  public:
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    T value() const { return data()->value(params); }

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
    void operator=(const U& v) { data()->set(v, params); }

    /**
     * Increment the stat by the given value. This calls the associated
     * storage object inc function.
     * @param v The value to add.
     */
    template <typename U>
    void operator+=(const U& v) { data()->inc(v, params); }

    /**
     * Decrement the stat by the given value. This calls the associated
     * storage object dec function.
     * @param v The value to substract.
     */
    template <typename U>
    void operator-=(const U& v) { data()->dec(v, params); }

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

    /**
     * Reset stat value to default
     */
    void reset() { bin.reset(); }

    result_t val() { return data()->val(params); }

    result_t total() { return val(); }

    bool zero() { return val() == 0.0; }
};

//////////////////////////////////////////////////////////////////////
//
// Vector Statistics
//
//////////////////////////////////////////////////////////////////////
template <typename T, template <typename T> class Storage, class Bin>
class ScalarProxy;

/**
 * Implementation of a vector of stats. The type of stat is determined by the
 * Storage class. @sa ScalarBase
 */
template <typename T, template <typename T> class Storage, class Bin>
class VectorBase : public DataAccess
{
  protected:
    /** Define the type of the storage class. */
    typedef Storage<T> storage_t;
    /** Define the params of the storage class. */
    typedef typename storage_t::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::VectorBin<storage_t> bin_t;

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
    storage_t *data(int index) { return bin.data(index, params); }
    /**
     * Retrieve a const pointer to the storage from the bin
     * for the given index.
     * @param index The vector index to access.
     * @return A const pointer to the storage object at the given index.
     */
    const storage_t *data(int index) const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(index, *_params);
    }

  protected:
    // Copying stats is not allowed
    /** Copying stats isn't allowed. */
    VectorBase(const VectorBase &stat);
    /** Copying stats isn't allowed. */
    const VectorBase &operator=(const VectorBase &);

  public:
    /**
     * Copy the values to a local vector and return a reference to it.
     * @return A reference to a vector of the stat values.
     */
    void val(rvec_t &vec) const
    {
        vec.resize(size());
        for (int i = 0; i < size(); ++i)
            vec[i] = data(i)->val(params);
    }

    /**
     * @return True is stat is binned.
     */
    bool binned() const { return bin_t::binned; }

    /**
     * Return a total of all entries in this vector.
     * @return The total of all vector entries.
     */
    result_t total() const {
        result_t total = 0.0;
        for (int i = 0; i < size(); ++i)
            total += data(i)->val(params);
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

    bool check() const { return true; }
    void reset() { bin.reset(); }

  public:
    VectorBase() {}

    /** Friend this class with the associated scalar proxy. */
    friend class ScalarProxy<T, Storage, Bin>;

    /**
     * Return a reference (ScalarProxy) to the stat at the given index.
     * @param index The vector index to access.
     * @return A reference of the stat.
     */
    ScalarProxy<T, Storage, Bin> operator[](int index);

    void update(StatData *data) {}
};

/**
 * A proxy class to access the stat at a given index in a VectorBase stat.
 * Behaves like a ScalarBase.
 */
template <typename T, template <typename T> class Storage, class Bin>
class ScalarProxy
{
  protected:
    /** Define the type of the storage class. */
    typedef Storage<T> storage_t;
    /** Define the params of the storage class. */
    typedef typename storage_t::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::VectorBin<storage_t> bin_t;

  private:
    /** Pointer to the bin in the parent VectorBase. */
    bin_t *bin;
    /** Pointer to the params in the parent VectorBase. */
    params_t *params;
    /** The index to access in the parent VectorBase. */
    int index;

  protected:
    /**
     * Retrieve the storage from the bin.
     * @return The storage from the bin for this stat.
     */
    storage_t *data() { return bin->data(index, *params); }
    /**
     * Retrieve a const pointer to the storage from the bin.
     * @return A const pointer to the storage for this stat.
     */
    const storage_t *data() const
    {
        bin_t *_bin = const_cast<bin_t *>(bin);
        params_t *_params = const_cast<params_t *>(params);
        return _bin->data(index, *_params);
    }

  public:
    /**
     * Return the current value of this statas a result type.
     * @return The current value.
     */
    result_t val() const { return data()->val(*params); }
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    T value() const { return data()->value(*params); }

  public:
    /**
     * Create and initialize this proxy, do not register it with the database.
     * @param b The bin to use.
     * @param p The params to use.
     * @param i The index to access.
     */
    ScalarProxy(bin_t &b, params_t &p, int i)
        : bin(&b), params(&p), index(i)  {}
    /**
     * Create a copy of the provided ScalarProxy.
     * @param sp The proxy to copy.
     */
    ScalarProxy(const ScalarProxy &sp)
        : bin(sp.bin), params(sp.params), index(sp.index) {}
    /**
     * Set this proxy equal to the provided one.
     * @param sp The proxy to copy.
     * @return A reference to this proxy.
     */
    const ScalarProxy &operator=(const ScalarProxy &sp) {
        bin = sp.bin;
        params = sp.params;
        index = sp.index;
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
    void operator=(const U& v) { data()->set(v, *params); }

    /**
     * Increment the stat by the given value. This calls the associated
     * storage object inc function.
     * @param v The value to add.
     */
    template <typename U>
    void operator+=(const U& v) { data()->inc(v, *params); }

    /**
     * Decrement the stat by the given value. This calls the associated
     * storage object dec function.
     * @param v The value to substract.
     */
    template <typename U>
    void operator-=(const U& v) { data()->dec(v, *params); }

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
};

template <typename T, template <typename T> class Storage, class Bin>
inline ScalarProxy<T, Storage, Bin>
VectorBase<T, Storage, Bin>::operator[](int index)
{
    assert (index >= 0 && index < size());
    return ScalarProxy<T, Storage, Bin>(bin, params, index);
}

template <typename T, template <typename T> class Storage, class Bin>
class VectorProxy;

template <typename T, template <typename T> class Storage, class Bin>
class Vector2dBase : public DataAccess
{
  protected:
    typedef Storage<T> storage_t;
    typedef typename storage_t::Params params_t;
    typedef typename Bin::VectorBin<storage_t> bin_t;

  protected:
    size_t x;
    size_t y;
    bin_t bin;
    params_t params;

  protected:
    storage_t *data(int index) { return bin.data(index, params); }
    const storage_t *data(int index) const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(index, *_params);
    }

  protected:
    // Copying stats is not allowed
    Vector2dBase(const Vector2dBase &stat);
    const Vector2dBase &operator=(const Vector2dBase &);

  public:
    Vector2dBase() {}

    void update(Vector2dDataBase *data)
    {
        data->x = x;
        data->y = y;
        int size = this->size();
        data->vec.resize(size);
        for (int i = 0; i < size; ++i)
            data->vec[i] = this->data(i)->val(params);
    }

    std::string ysubname(int i) const { return (*y_subnames)[i]; }

    friend class VectorProxy<T, Storage, Bin>;
    VectorProxy<T, Storage, Bin> operator[](int index);

    size_t size() const { return bin.size(); }
    bool zero() const { return data(0)->value(params) == 0.0; }

    /**
     * Reset stat value to default
     */
    void reset() { bin.reset(); }

    bool check() { return true; }
};

template <typename T, template <typename T> class Storage, class Bin>
class VectorProxy
{
  protected:
    typedef Storage<T> storage_t;
    typedef typename storage_t::Params params_t;
    typedef typename Bin::VectorBin<storage_t> bin_t;

  private:
    bin_t *bin;
    params_t *params;
    int offset;
    int len;

  private:
    mutable rvec_t *vec;

    storage_t *data(int index) {
        assert(index < len);
        return bin->data(offset + index, *params);
    }

    const storage_t *data(int index) const {
        bin_t *_bin = const_cast<bin_t *>(bin);
        params_t *_params = const_cast<params_t *>(params);
        return _bin->data(offset + index, *_params);
    }

  public:
    const rvec_t &val() const {
        if (vec)
            vec->resize(size());
        else
            vec = new rvec_t(size());

        for (int i = 0; i < size(); ++i)
            (*vec)[i] = data(i)->val(*params);

        return *vec;
    }

    result_t total() const {
        result_t total = 0.0;
        for (int i = 0; i < size(); ++i)
            total += data(i)->val(*params);
        return total;
    }

  public:
    VectorProxy(bin_t &b, params_t &p, int o, int l)
        : bin(&b), params(&p), offset(o), len(l), vec(NULL)
        { }
    VectorProxy(const VectorProxy &sp)
        : bin(sp.bin), params(sp.params), offset(sp.offset), len(sp.len),
          vec(NULL)
        { }
    ~VectorProxy() {
        if (vec)
            delete vec;
    }

    const VectorProxy &operator=(const VectorProxy &sp)
    {
        bin = sp.bin;
        params = sp.params;
        offset = sp.offset;
        len = sp.len;
        if (vec)
            delete vec;
        vec = NULL;
        return *this;
    }

    ScalarProxy<T, Storage, Bin> operator[](int index)
    {
        assert (index >= 0 && index < size());
        return ScalarProxy<T, Storage, Bin>(*bin, *params, offset + index);
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

template <typename T, template <typename T> class Storage, class Bin>
inline VectorProxy<T, Storage, Bin>
Vector2dBase<T, Storage, Bin>::operator[](int index)
{
    int offset = index * y;
    assert (index >= 0 && offset < size());
    return VectorProxy<T, Storage, Bin>(bin, params, offset, y);
}

//////////////////////////////////////////////////////////////////////
//
// Non formula statistics
//
//////////////////////////////////////////////////////////////////////

/**
 * Templatized storage and interface for a distrbution stat.
 */
template <typename T>
struct DistStor
{
  public:
    /** The parameters for a distribution stat. */
    struct Params
    {
        /** The minimum value to track. */
        int min;
        /** The maximum value to track. */
        int max;
        /** The number of entries in each bucket. */
        int bucket_size;
        /** The number of buckets. Equal to (max-min)/bucket_size. */
        int size;
    };
    enum { fancy = false };

  private:
    /** The smallest value sampled. */
    T min_val;
    /** The largest value sampled. */
    T max_val;
    /** The number of values sampled less than min. */
    T underflow;
    /** The number of values sampled more than max. */
    T overflow;
    /** The current sum. */
    T sum;
    /** The sum of squares. */
    T squares;
    /** The number of samples. */
    int samples;
    /** Counter for each bucket. */
    std::vector<T> vec;

  public:
    /**
     * Construct this storage with the supplied params.
     * @param params The parameters.
     */
    DistStor(const Params &params)
        : min_val(INT_MAX), max_val(INT_MIN), underflow(0), overflow(0),
          sum(T()), squares(T()), samples(0), vec(params.size)
    {
        reset();
    }

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     * @param params The paramters of the distribution.
     */
    void sample(T val, int number, const Params &params)
    {
        if (val < params.min)
            underflow += number;
        else if (val > params.max)
            overflow += number;
        else {
            int index = (val - params.min) / params.bucket_size;
            assert(index < size(params));
            vec[index] += number;
        }

        if (val < min_val)
            min_val = val;

        if (val > max_val)
            max_val = val;

        T sample = val * number;
        sum += sample;
        squares += sample * sample;
        samples += number;
    }

    /**
     * Return the number of buckets in this distribution.
     * @return the number of buckets.
     * @todo Is it faster to return the size from the parameters?
     */
    size_t size(const Params &) const { return vec.size(); }

    /**
     * Returns true if any calls to sample have been made.
     * @param params The paramters of the distribution.
     * @return True if any values have been sampled.
     */
    bool zero(const Params &params) const
    {
        return samples == 0;
    }

    void update(DistDataData *data, DisplayMode mode, const Params &params)
    {
        data->min = params.min;
        data->max = params.max;
        data->bucket_size = params.bucket_size;
        data->size = params.size;

        if (mode == mode_m5)
            data->min_val = (min_val == INT_MAX) ? params.min : min_val;
        else
            data->min_val = params.min;

        data->max_val = (max_val == INT_MIN) ? 0 : max_val;
        data->underflow = underflow;
        data->overflow = overflow;
        data->vec.resize(params.size);
        for (int i = 0; i < params.size; ++i)
            data->vec[i] = vec[i];

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

        int size = vec.size();
        for (int i = 0; i < size; ++i)
            vec[i] = T();

        sum = T();
        squares = T();
        samples = T();
    }
};

/**
 * Templatized storage and interface for a distribution that calculates mean
 * and variance.
 */
template <typename T>
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
    T sum;
    /** The sum of squares. */
    T squares;
    /** The number of samples. */
    int samples;

  public:
    /**
     * Create and initialize this storage.
     */
    FancyStor(const Params &) : sum(T()), squares(T()), samples(0) {}

    /**
     * Add a value the given number of times to this running average.
     * Update the running sum and sum of squares, increment the number of
     * values seen by the given number.
     * @param val The value to add.
     * @param number The number of times to add the value.
     * @param p The parameters of this stat.
     */
    void sample(T val, int number, const Params &p)
    {
        T value = val * number;
        sum += value;
        squares += value * value;
        samples += number;
    }

    void update(DistDataData *data, DisplayMode mode, const Params &params)
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
    bool zero(const Params &) const { return samples == 0; }

    /**
     * Reset stat value to default
     */
    void reset()
    {
        sum = T();
        squares = T();
        samples = 0;
    }
};

/**
 * Templatized storage for distribution that calculates per cycle mean and
 * variance.
 */
template <typename T>
struct AvgFancy
{
  public:
    /** No parameters for this storage. */
    struct Params {};
    enum { fancy = true };

  private:
    /** Current total. */
    T sum;
    /** Current sum of squares. */
    T squares;

  public:
    /**
     * Create and initialize this storage.
     */
    AvgFancy(const Params &) : sum(T()), squares(T()) {}

    /**
     * Add a value to the distribution for the given number of times.
     * Update the running sum and sum of squares.
     * @param val The value to add.
     * @param number The number of times to add the value.
     * @param p The paramters of the distribution.
     */
    void sample(T val, int number, const Params& p)
    {
        T value = val * number;
        sum += value;
        squares += value * value;
    }

    void update(DistDataData *data, DisplayMode mode, const Params &params)
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
    bool zero(const Params &params) const { return sum == 0; }
    /**
     * Reset stat value to default
     */
    void reset()
    {
        sum = T();
        squares = T();
    }
};

/**
 * Implementation of a distribution stat. The type of distribution is
 * determined by the Storage template. @sa ScalarBase
 */
template <typename T, template <typename T> class Storage, class Bin>
class DistBase : public DataAccess
{
  protected:
    /** Define the type of the storage class. */
    typedef Storage<T> storage_t;
    /** Define the params of the storage class. */
    typedef typename storage_t::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::Bin<storage_t> bin_t;

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
    storage_t *data() { return bin.data(params); }
    /**
     * Retrieve a const pointer to the storage from the bin.
     * @return A const pointer to the storage object for this stat.
     */
    const storage_t *data() const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(*_params);
    }

  protected:
    // Copying stats is not allowed
    /** Copies are not allowed. */
    DistBase(const DistBase &stat);
    /** Copies are not allowed. */
    const DistBase &operator=(const DistBase &);

  public:
    DistBase() { }

    /**
     * Add a value to the distribtion n times. Calls sample on the storage
     * class.
     * @param v The value to add.
     * @param n The number of times to add it, defaults to 1.
     */
    template <typename U>
    void sample(const U& v, int n = 1) { data()->sample(v, n, params); }

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

    void update(DistDataBase *base)
    {
        base->data.fancy = storage_t::fancy;
        data()->update(&(base->data), base->mode, params);
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

    bool check() { return true; }
};

template <typename T, template <typename T> class Storage, class Bin>
class DistProxy;

template <typename T, template <typename T> class Storage, class Bin>
class VectorDistBase : public DataAccess
{
  protected:
    typedef Storage<T> storage_t;
    typedef typename storage_t::Params params_t;
    typedef typename Bin::VectorBin<storage_t> bin_t;

  protected:
    bin_t bin;
    params_t params;

  protected:
    storage_t *data(int index) { return bin.data(index, params); }
    const storage_t *data(int index) const
    {
        bin_t *_bin = const_cast<bin_t *>(&bin);
        params_t *_params = const_cast<params_t *>(&params);
        return _bin->data(index, *_params);
    }

  protected:
    // Copying stats is not allowed
    VectorDistBase(const VectorDistBase &stat);
    const VectorDistBase &operator=(const VectorDistBase &);

  public:
    VectorDistBase() {}

    friend class DistProxy<T, Storage, Bin>;
    DistProxy<T, Storage, Bin> operator[](int index);
    const DistProxy<T, Storage, Bin> operator[](int index) const;

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

    bool check() { return true; }
    void update(VectorDistDataBase *base)
    {
        int size = this->size();
        base->data.resize(size);
        for (int i = 0; i < size; ++i) {
            base->data[i].fancy = storage_t::fancy;
            data(i)->update(&(base->data[i]), base->mode, params);
        }
    }
};

template <typename T, template <typename T> class Storage, class Bin>
class DistProxy
{
  protected:
    typedef Storage<T> storage_t;
    typedef typename storage_t::Params params_t;
    typedef typename Bin::Bin<storage_t> bin_t;
    typedef VectorDistBase<T, Storage, Bin> base_t;

  private:
    union {
        base_t *stat;
        const base_t *cstat;
    };
    int index;

  protected:
    storage_t *data() { return stat->data(index); }
    const storage_t *data() const { return cstat->data(index); }

  public:
    DistProxy(const VectorDistBase<T, Storage, Bin> &s, int i)
        : cstat(&s), index(i) {}
    DistProxy(const DistProxy &sp)
        : cstat(sp.cstat), index(sp.index) {}
    const DistProxy &operator=(const DistProxy &sp) {
        cstat = sp.cstat; index = sp.index; return *this;
    }

  public:
    template <typename U>
    void sample(const U& v, int n = 1) { data()->sample(v, n, cstat->params); }

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

template <typename T, template <typename T> class Storage, class Bin>
inline DistProxy<T, Storage, Bin>
VectorDistBase<T, Storage, Bin>::operator[](int index)
{
    assert (index >= 0 && index < size());
    return DistProxy<T, Storage, Bin>(*this, index);
}

template <typename T, template <typename T> class Storage, class Bin>
inline const DistProxy<T, Storage, Bin>
VectorDistBase<T, Storage, Bin>::operator[](int index) const
{
    assert (index >= 0 && index < size());
    return DistProxy<T, Storage, Bin>(*this, index);
}

#if 0
template <typename T, template <typename T> class Storage, class Bin>
result_t
VectorDistBase<T, Storage, Bin>::total(int index) const
{
    int total = 0;
    for (int i=0; i < x_size(); ++i) {
        total += data(i)->val(*params);
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
    virtual const rvec_t &val() const = 0;
    /**
     * Return the total of the result vector.
     * @return The total of the result vector.
     */
    virtual result_t total() const = 0;
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const = 0;
};

/** Reference counting pointer to a function Node. */
typedef RefCountingPtr<Node> NodePtr;

class ScalarStatNode : public Node
{
  private:
    const ScalarDataBase *data;
    mutable rvec_t result;

  public:
    ScalarStatNode(const ScalarDataBase *d) : data(d), result(1) {}
    virtual const rvec_t &val() const
    {
        result[0] = data->val();
        return result;
    }
    virtual result_t total() const { return data->val(); };

    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return data->binned(); }
};

template <typename T, template <typename T> class Storage, class Bin>
class ScalarProxyNode : public Node
{
  private:
    const ScalarProxy<T, Storage, Bin> proxy;
    mutable rvec_t result;

  public:
    ScalarProxyNode(const ScalarProxy<T, Storage, Bin> &p)
        : proxy(p), result(1) { }
    virtual const rvec_t &val() const
    {
        result[0] = proxy.val();
        return result;
    }
    virtual result_t total() const { return proxy.val(); };

    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return proxy.binned(); }
};

class VectorStatNode : public Node
{
  private:
    const VectorDataBase *data;

  public:
    VectorStatNode(const VectorDataBase *d) : data(d) { }
    virtual const rvec_t &val() const { return data->val(); }
    virtual result_t total() const { return data->total(); };

    virtual size_t size() const { return data->size(); }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return data->binned(); }
};

template <typename T>
class ConstNode : public Node
{
  private:
    rvec_t data;

  public:
    ConstNode(T s) : data(1, (result_t)s) {}
    const rvec_t &val() const { return data; }
    virtual result_t total() const { return data[0]; };

    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return False since constants aren't binned.
     */
    virtual bool binned() const { return false; }
};

template <typename T>
class FunctorNode : public Node
{
  private:
    T &functor;
    mutable rvec_t result;

  public:
    FunctorNode(T &f) : functor(f) { result.resize(1); }
    const rvec_t &val() const {
        result[0] = (result_t)functor();
        return result;
    }
    virtual result_t total() const { return (result_t)functor(); };

    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return False since Functors aren't binned
     */
    virtual bool binned() const { return false; }
};

template <typename T>
class ScalarNode : public Node
{
  private:
    T &scalar;
    mutable rvec_t result;

  public:
    ScalarNode(T &s) : scalar(s) { result.resize(1); }
    const rvec_t &val() const {
        result[0] = (result_t)scalar;
        return result;
    }
    virtual result_t total() const { return (result_t)scalar; };

    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return False since Scalar's aren't binned
     */
    virtual bool binned() const { return false; }
};

template <class Op>
class UnaryNode : public Node
{
  public:
    NodePtr l;
    mutable rvec_t result;

  public:
    UnaryNode(NodePtr p) : l(p) {}

    const rvec_t &val() const {
        const rvec_t &lvec = l->val();
        int size = lvec.size();

        assert(size > 0);

        result.resize(size);
        Op op;
        for (int i = 0; i < size; ++i)
            result[i] = op(lvec[i]);

        return result;
    }

    result_t total() const {
        Op op;
        return op(l->total());
    }

    virtual size_t size() const { return l->size(); }
    /**
     * Return true if child of node is binned.
     *@return True if child of node is binned.
     */
    virtual bool binned() const { return l->binned(); }
};

template <class Op>
class BinaryNode : public Node
{
  public:
    NodePtr l;
    NodePtr r;
    mutable rvec_t result;

  public:
    BinaryNode(NodePtr a, NodePtr b) : l(a), r(b) {}

    const rvec_t &val() const {
        Op op;
        const rvec_t &lvec = l->val();
        const rvec_t &rvec = r->val();

        assert(lvec.size() > 0 && rvec.size() > 0);

        if (lvec.size() == 1 && rvec.size() == 1) {
            result.resize(1);
            result[0] = op(lvec[0], rvec[0]);
        } else if (lvec.size() == 1) {
            int size = rvec.size();
            result.resize(size);
            for (int i = 0; i < size; ++i)
                result[i] = op(lvec[0], rvec[i]);
        } else if (rvec.size() == 1) {
            int size = lvec.size();
            result.resize(size);
            for (int i = 0; i < size; ++i)
                result[i] = op(lvec[i], rvec[0]);
        } else if (rvec.size() == lvec.size()) {
            int size = rvec.size();
            result.resize(size);
            for (int i = 0; i < size; ++i)
                result[i] = op(lvec[i], rvec[i]);
        }

        return result;
    }

    result_t total() const {
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
};

template <class Op>
class SumNode : public Node
{
  public:
    NodePtr l;
    mutable rvec_t result;

  public:
    SumNode(NodePtr p) : l(p), result(1) {}

    const rvec_t &val() const {
        const rvec_t &lvec = l->val();
        int size = lvec.size();
        assert(size > 0);

        result[0] = 0.0;

        Op op;
        for (int i = 0; i < size; ++i)
            result[0] = op(result[0], lvec[i]);

        return result;
    }

    result_t total() const {
        const rvec_t &lvec = l->val();
        int size = lvec.size();
        assert(size > 0);

        result_t result = 0.0;

        Op op;
        for (int i = 0; i < size; ++i)
            result = op(result, lvec[i]);

        return result;
    }

    virtual size_t size() const { return 1; }
    /**
     * Return true if child of node is binned.
     *@return True if child of node is binned.
     */
    virtual bool binned() const { return l->binned(); }
};

//////////////////////////////////////////////////////////////////////
//
// Binning Interface
//
//////////////////////////////////////////////////////////////////////
struct MainBin
{
  private:
    std::string _name;
    char *mem;

  protected:
    off_t memsize;
    off_t size() const { return memsize; }
    char *memory(off_t off);

  public:
    static MainBin *&curBin()
    {
        static MainBin *current = NULL;
        return current;
    }

    static void setCurBin(MainBin *bin) { curBin() = bin; }
    static MainBin *current() { assert(curBin()); return curBin(); }

    static off_t &offset()
    {
        static off_t offset = 0;
        return offset;
    }

    static off_t new_offset(size_t size)
    {
        size_t mask = sizeof(u_int64_t) - 1;
        off_t off = offset();

        // That one is for the last trailing flags byte.
        offset() += (size + 1 + mask) & ~mask;
        return off;
    }

  public:
    MainBin(const std::string &name);
    ~MainBin();

    const std::string &
    name() const
    {
        return _name;
    }

    void
    activate()
    {
        setCurBin(this);
#ifdef FS_MEASURE
        DPRINTF(TCPIP, "activating %s Bin\n", name());
#endif
    }

    class BinBase
    {
      private:
        int offset;

      public:
        BinBase() : offset(-1) {}
        void allocate(size_t size)
        {
            offset = new_offset(size);
        }
        char *access()
        {
            assert(offset != -1);
            return current()->memory(offset);
        }
    };

    template <class Storage>
    class Bin : public BinBase
    {
      public:
        typedef typename Storage::Params Params;

      public:
        enum { binned = true };
        Bin() { allocate(sizeof(Storage)); }
        bool initialized() const { return true; }
        void init(Params &params) { }

        int size() const { return 1; }

        Storage *
        data(Params &params)
        {
            assert(initialized());
            char *ptr = access();
            char *flags = ptr + sizeof(Storage);
            if (!(*flags & 0x1)) {
                *flags |= 0x1;
                new (ptr) Storage(params);
            }
            return reinterpret_cast<Storage *>(ptr);
        }

        void
        reset()
        {
            char *ptr = access();
            char *flags = ptr + size() * sizeof(Storage);
            if (!(*flags & 0x1))
                return;

            Storage *s = reinterpret_cast<Storage *>(ptr);
            s->reset();
        }
    };

    template <class Storage>
    class VectorBin : public BinBase
    {
      public:
        typedef typename Storage::Params Params;

      private:
        int _size;

      public:
        enum { binned = true };
        VectorBin() : _size(0) {}

        bool initialized() const { return _size > 0; }
        void init(int s, Params &params)
        {
            assert(!initialized());
            assert(s > 0);
            _size = s;
            allocate(_size * sizeof(Storage));
        }

        int size() const { return _size; }

        Storage *data(int index, Params &params)
        {
            assert(initialized());
            assert(index >= 0 && index < size());
            char *ptr = access();
            char *flags = ptr + size() * sizeof(Storage);
            if (!(*flags & 0x1)) {
                *flags |= 0x1;
                for (int i = 0; i < size(); ++i)
                    new (ptr + i * sizeof(Storage)) Storage(params);
            }
            return reinterpret_cast<Storage *>(ptr + index * sizeof(Storage));
        }
        void reset()
        {
            char *ptr = access();
            char *flags = ptr + size() * sizeof(Storage);
            if (!(*flags & 0x1))
                return;

            for (int i = 0; i < _size; ++i) {
                char *p = ptr + i * sizeof(Storage);
                Storage *s = reinterpret_cast<Storage *>(p);
                s->reset();
            }
        }
    };
};

struct NoBin
{
    template <class Storage>
    struct Bin
    {
      public:
        typedef typename Storage::Params Params;
        enum { binned = false };

      private:
        char ptr[sizeof(Storage)];

      public:
        ~Bin()
        {
            reinterpret_cast<Storage *>(ptr)->~Storage();
        }

        bool initialized() const { return true; }
        void init(Params &params)
        {
            new (ptr) Storage(params);
        }
        int size() const{ return 1; }
        Storage *data(Params &params)
        {
            assert(initialized());
            return reinterpret_cast<Storage *>(ptr);
        }
        void reset()
        {
            Storage *s = reinterpret_cast<Storage *>(ptr);
            s->reset();
        }
    };

    template <class Storage>
    struct VectorBin
    {
      public:
        typedef typename Storage::Params Params;
        enum { binned = false };

      private:
        char *ptr;
        int _size;

      public:
        VectorBin() : ptr(NULL) { }
        ~VectorBin()
        {
            if (!initialized())
                return;

            for (int i = 0; i < _size; ++i) {
                char *p = ptr + i * sizeof(Storage);
                reinterpret_cast<Storage *>(p)->~Storage();
            }
            delete [] ptr;
        }

        bool initialized() const { return ptr != NULL; }
        void init(int s, Params &params)
        {
            assert(s > 0 && "size must be positive!");
            assert(!initialized());
            _size = s;
            ptr = new char[_size * sizeof(Storage)];
            for (int i = 0; i < _size; ++i)
                new (ptr + i * sizeof(Storage)) Storage(params);
        }

        int size() const { return _size; }

        Storage *data(int index, Params &params)
        {
            assert(initialized());
            assert(index >= 0 && index < size());
            return reinterpret_cast<Storage *>(ptr + index * sizeof(Storage));
        }
        void reset()
        {
            for (int i = 0; i < _size; ++i) {
                char *p = ptr + i * sizeof(Storage);
                Storage *s = reinterpret_cast<Storage *>(p);
                s->reset();
            }
        }
    };
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
#ifdef FS_MEASURE
typedef MainBin DefaultBin;
#else
typedef NoBin DefaultBin;
#endif

/**
 * This is a simple scalar statistic, like a counter.
 * @sa Stat, ScalarBase, StatStor
 */
template <typename T = Counter, class Bin = DefaultBin>
class Scalar : public Wrap<Scalar<T, Bin>, ScalarBase<T, StatStor, Bin>, ScalarData>
{
  public:
    /** The base implementation. */
    typedef ScalarBase<T, StatStor, Bin> Base;

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
    void operator=(const U& v) { Base::operator=(v); }
};

/**
 * A stat that calculates the per cycle average of a value.
 * @sa Stat, ScalarBase, AvgStor
 */
template <typename T = Counter, class Bin = DefaultBin>
class Average : public Wrap<Average<T, Bin>, ScalarBase<T, AvgStor, Bin>, ScalarData>
{
  public:
    /** The base implementation. */
    typedef ScalarBase<T, AvgStor, Bin> Base;

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
    void operator=(const U& v) { Base::operator=(v); }
};

/**
 * A vector of scalar stats.
 * @sa Stat, VectorBase, StatStor
 */
template <typename T = Counter, class Bin = DefaultBin>
class Vector : public WrapVec<Vector<T, Bin>, VectorBase<T, StatStor, Bin>, VectorData>
{
  public:
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
template <typename T = Counter, class Bin = DefaultBin>
class AverageVector : public WrapVec<AverageVector<T, Bin>, VectorBase<T, AvgStor, Bin>, VectorData>
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
template <typename T = Counter, class Bin = DefaultBin>
class Vector2d : public WrapVec2d<Vector2d<T, Bin>, Vector2dBase<T, StatStor, Bin>, Vector2dData>
{
  public:
    Vector2d &init(size_t _x, size_t _y) {
        x = _x;
        y = _y;
        bin.init(x * y, params);
        setInit();

        return *this;
    }
};

/**
 * A simple distribution stat.
 * @sa Stat, DistBase, DistStor
 */
template <typename T = Counter, class Bin = DefaultBin>
class Distribution : public Wrap<Distribution<T, Bin>, DistBase<T, DistStor, Bin>, DistData>
{
  private:
    /** Base implementation. */
    typedef DistBase<T, DistStor, Bin> Base;
    /** The Parameter type. */
    typedef typename DistStor<T>::Params Params;

  public:
    /**
     * Set the parameters of this distribution. @sa DistStor::Params
     * @param min The minimum value of the distribution.
     * @param max The maximum value of the distribution.
     * @param bkt The number of values in each bucket.
     * @return A reference to this distribution.
     */
    Distribution &init(T min, T max, int bkt) {
        params.min = min;
        params.max = max;
        params.bucket_size = bkt;
        params.size = (max - min) / bkt + 1;
        bin.init(params);
        setInit();

        return *this;
    }
};

/**
 * Calculates the mean and variance of all the samples.
 * @sa Stat, DistBase, FancyStor
 */
template <typename T = Counter, class Bin = DefaultBin>
class StandardDeviation : public Wrap<StandardDeviation<T, Bin>, DistBase<T, FancyStor, Bin>, DistData>
{
  private:
    /** The base implementation */
    typedef DistBase<T, DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor<T>::Params Params;

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
template <typename T = Counter, class Bin = DefaultBin>
class AverageDeviation : public Wrap<AverageDeviation<T, Bin>, DistBase<T, AvgFancy, Bin>, DistData>
{
  private:
    /** The base implementation */
    typedef DistBase<T, DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor<T>::Params Params;

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
template <typename T = Counter, class Bin = DefaultBin>
class VectorDistribution : public WrapVec<VectorDistribution<T, Bin>, VectorDistBase<T, DistStor, Bin>, VectorDistData>
{
  private:
    /** The base implementation */
    typedef VectorDistBase<T, DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor<T>::Params Params;

  public:
    /**
     * Initialize storage and parameters for this distribution.
     * @param size The size of the vector (the number of distributions).
     * @param min The minimum value of the distribution.
     * @param max The maximum value of the distribution.
     * @param bkt The number of values in each bucket.
     * @return A reference to this distribution.
     */
    VectorDistribution &init(int size, T min, T max, int bkt) {
        params.min = min;
        params.max = max;
        params.bucket_size = bkt;
        params.size = (max - min) / bkt + 1;
        bin.init(size, params);
        setInit();

        return *this;
    }
};

/**
 * This is a vector of StandardDeviation stats.
 * @sa Stat, VectorDistBase, FancyStor
 */
template <typename T = Counter, class Bin = DefaultBin>
class VectorStandardDeviation : public WrapVec<VectorStandardDeviation<T, Bin>, VectorDistBase<T, FancyStor, Bin>, VectorDistData>
{
  private:
    /** The base implementation */
    typedef VectorDistBase<T, FancyStor, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor<T>::Params Params;

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
template <typename T = Counter, class Bin = DefaultBin>
class VectorAverageDeviation : public WrapVec<VectorAverageDeviation<T, Bin>, VectorDistBase<T, AvgFancy, Bin>, VectorDistData>
{
  private:
    /** The base implementation */
    typedef VectorDistBase<T, AvgFancy, Bin> Base;
    /** The parameter type. */
    typedef typename DistStor<T>::Params Params;

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
    void val(rvec_t &vec) const;

    /**
     * Return the total Formula result.  If there is a Vector
     * component to this Formula, then this is the result of the
     * Formula if the formula is applied after summing all the
     * components of the Vector.  For example, if Formula is x/y where
     * x is size 3, then total() will return (x[1]+x[2]+x[3])/y.  If
     * there is no Vector component, total() returns the same value as
     * the first entry in the rvec_t val() returns.
     * @return The total of the result vector.
     */
    result_t total() const;

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
};

class Temp;
class Formula : public WrapVec<Formula, FormulaBase, VectorData>
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
    mutable rvec_t vec;

  public:
    FormulaNode(const Formula &f) : formula(f) {}

    virtual size_t size() const { return formula.size(); }
    virtual const rvec_t &val() const { formula.val(vec); return vec; }
    virtual result_t total() const { return formula.total(); }
    virtual bool binned() const { return formula.binned(); }
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
    operator NodePtr() { return node;}

  public:
    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    template <typename T, class Bin>
    Temp(const Scalar<T, Bin> &s)
        : node(new ScalarStatNode(s.statData())) { }

    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    template <typename T, class Bin>
    Temp(const Average<T, Bin> &s)
        : node(new ScalarStatNode(s.statData())) { }

    /**
     * Create a new VectorStatNode.
     * @param s The VectorStat to place in a node.
     */
    template <typename T, class Bin>
    Temp(const Vector<T, Bin> &s)
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
    template <typename T, template <typename T> class Storage, class Bin>
    Temp(const ScalarProxy<T, Storage, Bin> &p)
        : node(new ScalarProxyNode<T, Storage, Bin>(p)) { }

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
void dump(std::ostream &stream);
void reset();
void RegResetCallback(Callback *cb);

inline Temp
operator+(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::plus<result_t> >(l, r));
}

inline Temp
operator-(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::minus<result_t> >(l, r));
}

inline Temp
operator*(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::multiplies<result_t> >(l, r));
}

inline Temp
operator/(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::divides<result_t> >(l, r));
}

inline Temp
operator%(Temp l, Temp r)
{
    return NodePtr(new BinaryNode<std::modulus<result_t> >(l, r));
}

inline Temp
operator-(Temp l)
{
    return NodePtr(new UnaryNode<std::negate<result_t> >(l));
}

template <typename T>
inline Temp
constant(T val)
{
    return NodePtr(new ConstNode<T>(val));
}

template <typename T>
inline Temp
functor(T &val)
{
    return NodePtr(new FunctorNode<T>(val));
}

template <typename T>
inline Temp
scalar(T &val)
{
    return NodePtr(new ScalarNode<T>(val));
}

inline Temp
sum(Temp val)
{
    return NodePtr(new SumNode<std::plus<result_t> >(val));
}
extern bool PrintDescriptions;

} // namespace statistics

#endif // __STATISTICS_HH__

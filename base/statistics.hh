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

#include "sim/host.hh"

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

/** Print stats out in SS format. */
#define STAT_DISPLAY_COMPAT

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
/** Don't print if this is zero. */
const FormatFlags nozero =	0x0004;
/** Don't print if this is NAN */
const FormatFlags nonan =	0x0008;
/** Print the cumulative percentage of total upto this entry. */
const FormatFlags cdf =		0x0010;
/** Print the distribution. */
const FormatFlags dist = 	0x0020;
/** Used for SS compatability. */
const FormatFlags __substat = 	0x8000;
/** Mask of flags that can't be set directly */
const FormatFlags __reserved =  __substat;

/* Contains the statistic implementation details */
namespace Detail {
//////////////////////////////////////////////////////////////////////
//
// Statistics Framework Base classes
//
//////////////////////////////////////////////////////////////////////
struct StatData;
struct SubData;

/**
 * Common base class for all statistics, used to maintain a list and print.
 * This class holds no data itself but is used to find the associated
 * StatData in the stat database @sa Statistics::Database.
 */
class Stat
{
  protected:
    /** Mark this statistics as initialized. */
    void setInit();
    /**
     * Finds and returns the associated StatData from the database.
     * @return The formatting and output data of this statistic.
     */
    StatData *mydata();
    /**
     * Finds and returns a const pointer to the associated StatData.
     * @return The formatting and output data of this statistic.
     */
    const StatData *mydata() const;
    /**
     * Mark this stat for output at the end of simulation.
     * @return The formatting and output data of this statistic.
     */
    StatData *print();
    /**
     * Finds and returns the SubData at the given index.
     * @param index The index of the SubData to find.
     * @return The name and description of the given index.
     */
    const SubData *mysubdata(int index) const;
    /**
     * Create and return a new SubData field for the given index.
     * @param index The index to create a SubData for.
     * @return A pointer to the created SubData.
     */
    SubData *mysubdata_create(int index);

  public:
    /**
     * Return the name of this stat.
     * @return the name of the stat.
     */
    virtual std::string myname() const;
    /**
     * Return the name of the sub field at the given index.
     * @param index the subfield index.
     * @return the name of the subfield.
     */
    virtual std::string mysubname(int index) const;
    /**
     * Return the description of this stat.
     * @return the description of this stat.
     */
    virtual std::string mydesc() const;
    /**
     * Return the description of the subfield at the given index.
     * @param index The subfield index.
     * @return the description of the subfield.
     */
    virtual std::string mysubdesc(int index) const;
    /**
     * Return the format flags of this stat.
     * @return the format flags.
     */
    virtual FormatFlags myflags() const;
    /**
     * Return true if this stat's prereqs have been satisfied (they are non
     * zero).
     * @return true if the prerequisite stats aren't zero.
     */
    virtual bool dodisplay() const;
    /**
     * Return the display percision.
     * @return The display precision.
     */
    virtual int myprecision() const;

  public:
    /**
     * Create this stat and perhaps register it with the stat database. To be
     * printed a stat must be registered with the database.
     * @param reg If true, register this stat in the database.
     */
    Stat(bool reg);
    /**
     * Destructor
     */
    virtual ~Stat() {}

    /**
     * Print this stat to the given ostream.
     * @param stream The stream to print to.
     */
    virtual void display(std::ostream &stream) const = 0;
    /**
     * Reset this stat to the default state.
     */
    virtual void reset() = 0;
    /**
     * Return the number of entries in this stat.
     * @return The number of entries.
     */
    virtual size_t size() const = 0;
    /**
     * Return true if the stat has value zero.
     * @return True if the stat is zero.
     */
    virtual bool zero() const = 0;

    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const = 0;

    /**
     * Set the name and marks this stat to print at the end of simulation.
     * @param name The new name.
     * @return A reference to this stat.
     */
    Stat &name(const std::string &name);
    /**
     * Set the description and marks this stat to print at the end of
     * simulation.
     * @param desc The new description.
     * @return A reference to this stat.
     */
    Stat &desc(const std::string &desc);
    /**
     * Set the precision and marks this stat to print at the end of simulation.
     * @param p The new precision
     * @return A reference to this stat.
     */
    Stat &precision(int p);
    /**
     * Set the flags and marks this stat to print at the end of simulation.
     * @param f The new flags.
     * @return A reference to this stat.
     */
    Stat &flags(FormatFlags f);
    /**
     * Set the prerequisite stat and marks this stat to print at the end of
     * simulation.
     * @param prereq The prerequisite stat.
     * @return A reference to this stat.
     */
    Stat &prereq(const Stat &prereq);
    /**
     * Set the subfield name for the given index, and marks this stat to print
     * at the end of simulation.
     * @param index The subfield index.
     * @param name The new name of the subfield.
     * @return A reference to this stat.
     */
    Stat &subname(int index, const std::string &name);
    /**
     * Set the subfield description for the given index and marks this stat to
     * print at the end of simulation.
     * @param index The subfield index.
     * @param desc The new description of the subfield
     * @return A reference to this stat.
     */
    Stat &subdesc(int index, const std::string &desc);

  public:
    /**
     * Checks if the first stat's name is alphabetically less than the second.
     * This function breaks names up at periods and considers each subname
     * separately.
     * @param stat1 The first stat.
     * @param stat2 The second stat.
     * @return stat1's name is alphabetically before stat2's
     */
    static bool less(Stat *stat1, Stat *stat2);

#ifdef STAT_DEBUG
    /** A unique ID used for debugging. */
    int number;
#endif
};

/**
 * Base class for all scalar stats. The class provides an interface to access
 * the current value of the stat. This class can be used in formulas.
 */
class ScalarStat : public Stat
{
  public:
    /**
     * Create and perhaps register this stat with the database.
     * @param reg If true, register this stat with the database.
     */
    ScalarStat(bool reg) : Stat(reg) {}
    /**
     * Return the current value of this statistic as a result type.
     * @return The current value of this statistic.
     */
    virtual result_t val() const = 0;
    /**
     * Return true if this stat has value zero.
     * @return True if this stat is zero.
     */
    virtual bool zero() const;
    /**
     * Print this stat to the provided ostream.
     * @param stream The output stream.
     */
    virtual void display(std::ostream &stream) const;

    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const = 0;
};

void
VectorDisplay(std::ostream &stream, const std::string &myname,
              const std::vector<std::string> *mysubnames,
              const std::string &mydesc,
              const std::vector<std::string> *mysubdescs,
              int myprecision, FormatFlags myflags, const rvec_t &vec,
              result_t mytotal);

/**
 * Base class for all vector stats. This class provides interfaces to access
 * the current values of the stats as well as the totals. This class can be
 * used in formulas.
 */
class VectorStat : public Stat
{
  public:
    /**
     * Create and perhaps register this stat with the database.
     * @param reg If true, register this stat with the database.
     */
    VectorStat(bool reg) : Stat(reg) {}
    /**
     * Return a vector of result typesd of all the values in the vector.
     * @return The values of the vector.
     */
    virtual const rvec_t &val() const = 0;
    /**
     * Return the total of all the entries in the vector.
     * @return The total of the vector.
     */
    virtual result_t total() const = 0;
    /**
     * Return true if this stat has value zero.
     * @return True if this stat is zero.
     */
    virtual bool zero() const;
    /**
     * Print this stat to the provided ostream.
     * @param stream The output stream.
     */
    virtual void display(std::ostream &stream) const;

    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const = 0;
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

  public:
    /**
     * Builds this storage element and calls the base constructor of the
     * datatype.
     */
    StatStor(const Params &) : data(T()) {}

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
    void reset() { data = T(); }
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
    /** The paramaters for this storage type, none for this average. */
    struct Params { };

  private:
    /** The current count. */
    T current;
    /** The total count for all cycles. */
    mutable result_t total;
    /** The cycle that current last changed. */
    mutable Tick last;

  public:
    /**
     * Build and initializes this stat storage.
     */
    AvgStor(const Params &) : current(T()), total(0), last(0) { }

    /**
     * Set the current count to the one provided, update the total and last
     * set values.
     * @param val The new count.
     * @param p The parameters for this storage.
     */
    void set(T val, const Params &p) {
        total += current * (curTick - last);
        last = curTick;
        current = val;
    }
    /**
     * Increment the current count by the provided value, calls set.
     * @param val The amount to increment.
     * @param p The parameters for this storage.
     */
    void inc(T val, const Params &p) { set(current + val, p); }
    /**
     * Deccrement the current count by the provided value, calls set.
     * @param val The amount to decrement.
     * @param p The parameters for this storage.
     */
    void dec(T val, const Params &p) { set(current - val, p); }
    /**
     * Return the current average.
     * @param p The parameters for this storage.
     * @return The current average.
     */
    result_t val(const Params &p) const {
        total += current * (curTick - last);
        last = curTick;
        return (result_t)(total + current) / (result_t)(curTick + 1);
    }
    /**
     * Return the current count.
     * @param p The parameters for this storage.
     * @return The current count.
     */
    T value(const Params &p) const { return current; }
    /**
     * Reset stat value to default
     */
    void reset()
    {
        current = T();
        total = 0;
        last = curTick;
    }
};

/**
 * Implementation of a scalar stat. The type of stat is determined by the
 * Storage template. The storage for this stat is held within the Bin class.
 * This allows for breaking down statistics across multiple bins easily.
 */
template <typename T, template <typename T> class Storage, class Bin>
class ScalarBase : public ScalarStat
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
    const storage_t *data() const {
        return (const_cast<bin_t *>(&bin))->data(params);
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
     * Return the current value of this stat as a result type.
     * @return The current value.
     */
    result_t val() const { return data()->val(params); }
    /**
     * Return the current value of this stat as its base type.
     * @return The current value.
     */
    T value() const { return data()->value(params); }

  public:
    /**
     * Create and initialize this stat, register it with the database.
     */
    ScalarBase() : ScalarStat(true) {
        bin.init(params);
        setInit();
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
    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return bin_t::binned; }

    /**
     * Reset stat value to default
     */
    void reset() { bin.reset(); }
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
class VectorBase : public VectorStat
{
  protected:
    /** Define the type of the storage class. */
    typedef Storage<T> storage_t;
    /** Define the params of the storage class. */
    typedef typename storage_t::Params params_t;
    /** Define the bin type. */
    typedef typename Bin::VectorBin<storage_t> bin_t;

  private:
    /** Local storage for the entry values, used for printing. */
    mutable rvec_t *vec;

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
    const storage_t *data(int index) const {
        return (const_cast<bin_t *>(&bin))->data(index, params);
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
    const rvec_t &val() const {
        if (vec)
            vec->resize(size());
        else
            vec = new rvec_t(size());

        for (int i = 0; i < size(); ++i)
            (*vec)[i] = data(i)->val(params);

        return *vec;
    }

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

  public:
    /**
     * Create this vector and register it with the database.
     */
    VectorBase() : VectorStat(true), vec(NULL) {}
    /**
     * Destructor.
     */
    ~VectorBase() { if (vec) delete vec; }

    /**
     * Set this vector to have the given size.
     * @param size The new size.
     * @return A reference to this stat.
     */
    VectorBase &init(size_t size) {
        bin.init(size, params);
        setInit();

        return *this;
    }

    /** Friend this class with the associated scalar proxy. */
    friend class ScalarProxy<T, Storage, Bin>;

    /**
     * Return a reference (ScalarProxy) to the stat at the given index.
     * @param index The vector index to access.
     * @return A reference of the stat.
     */
    ScalarProxy<T, Storage, Bin> operator[](int index);

    /**
     * Return the number of elements in this vector.
     * @return The size of the vector.
     */
    virtual size_t size() const { return bin.size(); }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return bin_t::binned; }
    /**
     * Reset stat value to default
     */
    virtual void reset() { bin.reset(); }
};

/**
 * A proxy class to access the stat at a given index in a VectorBase stat.
 * Behaves like a ScalarBase.
 */
template <typename T, template <typename T> class Storage, class Bin>
class ScalarProxy : public ScalarStat
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
    const storage_t *data() const { return bin->data(index, *params); }

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
        : ScalarStat(false), bin(&b), params(&p), index(i)  {}
    /**
     * Create a copy of the provided ScalarProxy.
     * @param sp The proxy to copy.
     */
    ScalarProxy(const ScalarProxy &sp)
        : ScalarStat(false), bin(sp.bin), params(sp.params), index(sp.index) {}
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
    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return false since Proxies aren't printed/binned
     */
    virtual bool binned() const { return false; }
    /**
     * This stat has no state.  Nothing to reset
     */
    virtual void reset() {  }
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
class Vector2dBase : public Stat
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
    std::vector<std::string> *y_subnames;

  protected:
    storage_t *data(int index) { return bin.data(index, params); }
    const storage_t *data(int index) const {
        return (const_cast<bin_t *>(&bin))->data(index, params);
    }

  protected:
    // Copying stats is not allowed
    Vector2dBase(const Vector2dBase &stat);
    const Vector2dBase &operator=(const Vector2dBase &);

  public:
    Vector2dBase() : Stat(true) {}
    ~Vector2dBase() { }

    Vector2dBase &init(size_t _x, size_t _y) {
        x = _x;
        y = _y;
        bin.init(x * y, params);
        setInit();
        y_subnames = new std::vector<std::string>(y);

        return *this;
    }

    /**
     * @warning This makes the assumption that if you're gonna subnames a 2d
     * vector, you're subnaming across all y
     */
    Vector2dBase &ysubnames(const char **names)
    {
        for (int i=0; i < y; ++i) {
            (*y_subnames)[i] = names[i];
        }
        return *this;
    }
    Vector2dBase &ysubname(int index, const std::string subname)
    {
        (*y_subnames)[i] = subname.c_str();
        return *this;
    }
    std::string ysubname(int i) const { return (*y_subnames)[i]; }

    friend class VectorProxy<T, Storage, Bin>;
    VectorProxy<T, Storage, Bin> operator[](int index);

    virtual size_t size() const { return bin.size(); }
    virtual bool zero() const { return data(0)->value(params) == 0.0; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return bin_t::binned; }

    virtual void
    display(std::ostream &out) const
    {
        bool have_subname = false;
        for (int i = 0; i < x; ++i) {
            if (!mysubname(i).empty())
                have_subname = true;
        }

        rvec_t tot_vec(y);
        result_t super_total = 0.0;
        for (int i = 0; i < x; ++i) {
            std::string subname;
            if (have_subname) {
                subname = mysubname(i);
                if (subname.empty())
                    continue;
            } else
                subname = to_string(i);

            int iy = i * y;
            rvec_t vec(y);

            result_t total = 0.0;
            for (int j = 0; j < y; ++j) {
                vec[j] = data(iy + j)->val(params);
                tot_vec[j] += vec[j];
                total += vec[j];
                super_total += vec[j];
            }

            std::string desc;
            if (mysubdesc(i).empty()) {
                desc = mydesc();
            } else {
                desc = mysubdesc(i);
            }

            VectorDisplay(out, myname() + "_" + subname, y_subnames, desc, 0,
                          myprecision(), myflags(), vec, total);

        }
        if ((myflags() & ::Statistics::total) && (x > 1)) {
            VectorDisplay(out, myname(), y_subnames, mydesc(), 0,
                          myprecision(), myflags(), tot_vec, super_total);

        }
    }
    /**
     * Reset stat value to default
     */
    virtual void reset() { bin.reset(); }
};

template <typename T, template <typename T> class Storage, class Bin>
class VectorProxy : public VectorStat
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
        return (const_cast<bin_t *>(bin))->data(offset + index, *params);
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
        : VectorStat(false), bin(&b), params(&p), offset(o), len(l), vec(NULL)
        { }
    VectorProxy(const VectorProxy &sp)
        : VectorStat(false), bin(sp.bin), params(sp.params), offset(sp.offset),
          len(sp.len), vec(NULL)
        { }
    ~VectorProxy() {
        if (vec)
            delete vec;
    }

    const VectorProxy &operator=(const VectorProxy &sp) {
        bin = sp.bin;
        params = sp.params;
        offset = sp.offset;
        len = sp.len;
        if (vec)
            delete vec;
        vec = NULL;
        return *this;
    }

    virtual size_t size() const { return len; }

    ScalarProxy<T, Storage, Bin> operator[](int index) {
        assert (index >= 0 && index < size());
        return ScalarProxy<T, Storage, Bin>(*bin, *params, offset + index);
    }
    /**
     * Return true if stat is binned.
     *@return false since Proxies aren't printed/binned
     */
    virtual bool binned() const { return false; }

    /**
     * This stat has no state.  Nothing to reset.
     */
    virtual void reset() { }
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

void DistDisplay(std::ostream &stream, const std::string &name,
                 const std::string &desc, int precision, FormatFlags flags,
                 result_t min_val, result_t max_val,
                 result_t underflow, result_t overflow,
                 const rvec_t &vec, int min, int max, int bucket_size,
                 int size);
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

  private:
    /** The smallest value sampled. */
    T min_val;
    /** The largest value sampled. */
    T max_val;
    /** The number of values sampled less than min. */
    T underflow;
    /** The number of values sampled more than max. */
    T overflow;
    /** Counter for each bucket. */
    std::vector<T> vec;

  public:
    /**
     * Construct this storage with the supplied params.
     * @param params The parameters.
     */
    DistStor(const Params &params)
        : min_val(INT_MAX), max_val(INT_MIN), underflow(0), overflow(0),
          vec(params.size)
    {
        reset();
    }

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     * @param params The paramters of the distribution.
     */
    void sample(T val, int number, const Params &params) {
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
    bool zero(const Params &params) const {
        if (underflow != 0 || overflow != 0)
            return false;

        int s = size(params);
        for (int i = 0; i < s; i++)
            if (vec[i] != 0)
                return false;

        return true;
    }

    /**
     * Print this distribution and the given print data to the given ostream.
     * @param stream The output stream.
     * @param name The name of this stat (from StatData).
     * @param desc The description of this stat (from StatData).
     * @param precision The print precision (from StatData).
     * @param flags The format flags (from StatData).
     * @param params The paramters of this distribution.
     */
    void display(std::ostream &stream, const std::string &name,
                 const std::string &desc, int precision, FormatFlags flags,
                 const Params &params) const {

#ifdef STAT_DISPLAY_COMPAT
        result_t min = params.min;
#else
        result_t min = (min_val == INT_MAX) ? params.min : min_val;
#endif
        result_t max = (max_val == INT_MIN) ? 0 : max_val;

        rvec_t rvec(params.size);
        for (int i = 0; i < params.size; ++i)
            rvec[i] = vec[i];

        DistDisplay(stream, name, desc, precision, flags,
                    (result_t)min, (result_t)max,
                    (result_t)underflow, (result_t)overflow,
                    rvec, params.min, params.max, params.bucket_size,
                    params.size);
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
    }

};

void FancyDisplay(std::ostream &stream, const std::string &name,
                  const std::string &desc, int precision, FormatFlags flags,
                  result_t mean, result_t variance);

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

  private:
    /** The current sum. */
    T sum;
    /** The sum of squares. */
    T squares;
    /** The total number of samples. */
    int total;

  public:
    /**
     * Create and initialize this storage.
     */
    FancyStor(const Params &) : sum(T()), squares(T()), total(0) {}

    /**
     * Add a value the given number of times to this running average.
     * Update the running sum and sum of squares, increment the number of
     * values seen by the given number.
     * @param val The value to add.
     * @param number The number of times to add the value.
     * @param p The parameters of this stat.
     */
    void sample(T val, int number, const Params &p) {
        T value = val * number;
        sum += value;
        squares += value * value;
        total += number;
    }

    /**
     * Print this distribution and the given print data to the given ostream.
     * @param stream The output stream.
     * @param name The name of this stat (from StatData).
     * @param desc The description of this stat (from StatData).
     * @param precision The print precision (from StatData).
     * @param flags The format flags (from StatData).
     * @param params The paramters of this distribution.
     */
    void display(std::ostream &stream, const std::string &name,
                 const std::string &desc, int precision, FormatFlags flags,
                 const Params &params) const {

        result_t mean = NAN;
        result_t variance = NAN;

        if (total != 0) {
            result_t fsum = sum;
            result_t fsq = squares;
            result_t ftot = total;

            mean = fsum / ftot;
            variance = (ftot * fsq - (fsum * fsum)) / (ftot * (ftot - 1.0));
        }

        FancyDisplay(stream, name, desc, precision, flags, mean, variance);
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
    bool zero(const Params &) const { return total == 0; }
    /**
     * Reset stat value to default
     */
    virtual void reset()
    {
        sum = T();
        squares = T();
        total = 0;
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
    void sample(T val, int number, const Params& p) {
        T value = val * number;
        sum += value;
        squares += value * value;
    }

    /**
     * Print this distribution and the given print data to the given ostream.
     * @param stream The output stream.
     * @param name The name of this stat (from StatData).
     * @param desc The description of this stat (from StatData).
     * @param precision The print precision (from StatData).
     * @param flags The format flags (from StatData).
     * @param params The paramters of this distribution.
     */
    void display(std::ostream &stream, const std::string &name,
                 const std::string &desc, int precision, FormatFlags flags,
                 const Params &params) const {
        result_t mean = sum / curTick;
        result_t variance = (squares - sum * sum) / curTick;

        FancyDisplay(stream, name, desc, precision, flags, mean, variance);
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
    virtual void reset()
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
class DistBase : public Stat
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
    const storage_t *data() const {
        return (const_cast<bin_t *>(&bin))->data(params);
    }

  protected:
    // Copying stats is not allowed
    /** Copies are not allowed. */
    DistBase(const DistBase &stat);
    /** Copies are not allowed. */
    const DistBase &operator=(const DistBase &);

  public:
    /**
     * Create this distrubition and register it with the database.
     */
    DistBase() : Stat(true) { }
    /**
     * Destructor.
     */
    ~DistBase() { }

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
    virtual size_t size() const { return data()->size(params); }
    /**
     * Return true if no samples have been added.
     * @return True if there haven't been any samples.
     */
    virtual bool zero() const { return data()->zero(params); }
    /**
     * Print this distribution to the given ostream.
     * @param stream The output stream.
     */
    virtual void display(std::ostream &stream) const {
        data()->display(stream, myname(), mydesc(), myprecision(), myflags(),
                        params);
    }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return bin_t::binned; }
    /**
     * Reset stat value to default
     */
    virtual void reset()
    {
        bin.reset();
    }
};

template <typename T, template <typename T> class Storage, class Bin>
class DistProxy;

template <typename T, template <typename T> class Storage, class Bin>
class VectorDistBase : public Stat
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
    const storage_t *data(int index) const {
        return (const_cast<bin_t *>(&bin))->data(index, params);
    }

  protected:
    // Copying stats is not allowed
    VectorDistBase(const VectorDistBase &stat);
    const VectorDistBase &operator=(const VectorDistBase &);

  public:
    VectorDistBase() : Stat(true) { }
    ~VectorDistBase() { }

    friend class DistProxy<T, Storage, Bin>;
    DistProxy<T, Storage, Bin> operator[](int index);
    const DistProxy<T, Storage, Bin> operator[](int index) const;

    virtual size_t size() const { return bin.size(); }
    virtual bool zero() const { return false; }
    virtual void display(std::ostream &stream) const;
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return bin_t::binned; }
    /**
     * Reset stat value to default
     */
    virtual void reset()
    {
        bin.reset();
    }
};

template <typename T, template <typename T> class Storage, class Bin>
class DistProxy : public Stat
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
        : Stat(false), cstat(&s), index(i) {}
    DistProxy(const DistProxy &sp)
        : Stat(false), cstat(sp.cstat), index(sp.index) {}
    const DistProxy &operator=(const DistProxy &sp) {
        cstat = sp.cstat; index = sp.index; return *this;
    }

  public:
    template <typename U>
    void sample(const U& v, int n = 1) { data()->sample(v, n, cstat->params); }

    virtual size_t size() const { return 1; }
    virtual bool zero() const {
        return data()->zero(cstat->params);
    }
    virtual void display(std::ostream &stream) const {
        std::stringstream name, desc;

        if (!(cstat->mysubname(index).empty())) {
            name << cstat->myname() << cstat->mysubname(index);
        } else {
            name << cstat->myname() << "_" << index;
        }
        if (!(cstat->mysubdesc(index).empty())) {
            desc << cstat->mysubdesc(index);
        } else {
            desc << cstat->mydesc();
        }

        data()->display(stream, name.str(), desc.str(),
                        cstat->myprecision(), cstat->myflags(), cstat->params);
    }
    /**
     * Return true if stat is binned.
     *@return false since Proxies are not binned/printed.
     */
    virtual bool binned() const { return false; }
    /**
     * Proxy has no state.  Nothing to reset.
     */
    virtual void reset() { }
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

/**
 * @todo Need a way to print Distribution totals across the Vector
 */
template <typename T, template <typename T> class Storage, class Bin>
void
VectorDistBase<T, Storage, Bin>::display(std::ostream &stream) const
{
    for (int i = 0; i < size(); ++i) {
        DistProxy<T, Storage, Bin> proxy(*this, i);
        proxy.display(stream);
    }
}

#if 0
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
    const ScalarStat &stat;
    mutable rvec_t result;

  public:
    ScalarStatNode(const ScalarStat &s) : stat(s), result(1) {}
    const rvec_t &val() const { result[0] = stat.val(); return result; }
    virtual result_t total() const { return stat.val(); };

    virtual size_t size() const { return 1; }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return stat.binned(); }
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
    const rvec_t &val() const { result[0] = proxy.val(); return result; }
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
    const VectorStat &stat;

  public:
    VectorStatNode(const VectorStat &s) : stat(s) {}
    const rvec_t &val() const { return stat.val(); }
    virtual result_t total() const { return stat.total(); };

    virtual size_t size() const { return stat.size(); }
    /**
     * Return true if stat is binned.
     *@return True is stat is binned.
     */
    virtual bool binned() const { return stat.binned(); }
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

/**
 * Helper class to construct formula node trees.
 */
class Temp
{
  private:
    /**
     * Pointer to a Node object.
     */
    NodePtr node;

  public:
    /**
     * Copy the given pointer to this class.
     * @param n A pointer to a Node object to copy.
     */
    Temp(NodePtr n) : node(n) {}
    /**
     * Create a new ScalarStatNode.
     * @param s The ScalarStat to place in a node.
     */
    Temp(const ScalarStat &s) : node(new ScalarStatNode(s)) {}
    /**
     * Create a new ScalarProxyNode.
     * @param p The ScalarProxy to place in a node.
     */
    template <typename T, template <typename T> class Storage, class Bin>
    Temp(const ScalarProxy<T, Storage, Bin> &p)
        : node(new ScalarProxyNode<T, Storage, Bin>(p)) {}
    /**
     * Create a new VectorStatNode.
     * @param s The VectorStat to place in a node.
     */
    Temp(const VectorStat &s) : node(new VectorStatNode(s)) {}

    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed char value) : node(new ConstNode<signed char>(value)) {}
    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned char value) : node(new ConstNode<unsigned char>(value)) {}
    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed short value) : node(new ConstNode<signed short>(value)) {}
    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned short value) : node(new ConstNode<unsigned short>(value)) {}
    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed int value) : node(new ConstNode<signed int>(value)) {}
    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned int value) : node(new ConstNode<unsigned int>(value)) {}
    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(signed long value) : node(new ConstNode<signed long>(value)) {}
    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(unsigned long value) : node(new ConstNode<unsigned long>(value)) {}
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
    Temp(float value) : node(new ConstNode<float>(value)) {}
    /**
     * Create a ConstNode
     * @param value The value of the const node.
     */
    Temp(double value) : node(new ConstNode<double>(value)) {}

    /**
     * Return the node pointer.
     * @return the node pointer.
     */
    operator NodePtr() { return node;}
};


//////////////////////////////////////////////////////////////////////
//
// Binning Interface
//
//////////////////////////////////////////////////////////////////////

class BinBase
{
  private:
    off_t memsize;
    char *mem;

  protected:
    off_t size() const { return memsize; }
    char *memory();

  public:
    BinBase(size_t size);
    virtual ~BinBase();
    virtual void activate() = 0;
    void regBin(BinBase *bin, std::string name);
};

} // namespace Detail

template <class BinType>
struct StatBin : public Detail::BinBase
{
  private:
    std::string _name;

  public:
    std::string name() const { return _name;}

    static StatBin *&curBin() {
        static StatBin *current = NULL;
        return current;
    }

    static void setCurBin(StatBin *bin) { curBin() = bin; }
    static StatBin *current() { assert(curBin()); return curBin(); }

    static off_t &offset() {
        static off_t offset = 0;
        return offset;
    }

    static off_t new_offset(size_t size) {
        size_t mask = sizeof(u_int64_t) - 1;
        off_t off = offset();

        // That one is for the last trailing flags byte.
        offset() += (size + 1 + mask) & ~mask;

        return off;
    }

    explicit StatBin(std::string name, size_t size = 1024) : Detail::BinBase(size) {  _name = name; this->regBin(this, name); }

    char *memory(off_t off) {
        assert(offset() <= size());
        return Detail::BinBase::memory() + off;
    }

    virtual void activate()  { setCurBin(this); }
    static void activate(StatBin &bin) { setCurBin(&bin); }

    class BinBase
    {
      private:
        int offset;

      public:
        BinBase() : offset(-1) {}
        void allocate(size_t size) {
            offset = new_offset(size);
        }
        char *access() {
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
        void init(const Params &params) { }

        int size() const { return 1; }

        Storage *data(const Params &params) {
            assert(initialized());
            char *ptr = access();
            char *flags = ptr + sizeof(Storage);
            if (!(*flags & 0x1)) {
                *flags |= 0x1;
                new (ptr) Storage(params);
            }
            return reinterpret_cast<Storage *>(ptr);
        }
        void reset()
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
        VectorBin() : _size(0) {}

        bool initialized() const { return _size > 0; }
        void init(int s, const Params &params) {
            assert(!initialized());
            assert(s > 0);
            _size = s;
            allocate(_size * sizeof(Storage));
        }

        int size() const { return _size; }

        Storage *data(int index, const Params &params) {
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

class MainBinType {};
typedef StatBin<MainBinType> MainBin;

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
        void init(const Params &params) {
            new (ptr) Storage(params);
        }
        int size() const{ return 1; }
        Storage *data(const Params &params) {
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
        void init(int s, const Params &params) {
            assert(s > 0 && "size must be positive!");
            assert(!initialized());
            _size = s;
            ptr = new char[_size * sizeof(Storage)];
            for (int i = 0; i < _size; ++i)
                new (ptr + i * sizeof(Storage)) Storage(params);
        }

        int size() const { return _size; }

        Storage *data(int index, const Params &params) {
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
 * This is a simple scalar statistic, like a counter.
 * @sa Stat, ScalarBase, StatStor
 */
template <typename T = Counter, class Bin = NoBin>
class Scalar : public Detail::ScalarBase<T, Detail::StatStor, Bin>
{
  public:
    /** The base implementation. */
    typedef Detail::ScalarBase<T, Detail::StatStor, Bin> Base;

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
template <typename T = Counter, class Bin = NoBin>
class Average : public Detail::ScalarBase<T, Detail::AvgStor, Bin>
{
  public:
    /** The base implementation. */
    typedef Detail::ScalarBase<T, Detail::AvgStor, Bin> Base;

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
template <typename T = Counter, class Bin = NoBin>
class Vector : public Detail::VectorBase<T, Detail::StatStor, Bin>
{ };

/**
 * A vector of Average stats.
 * @sa Stat, VectorBase, AvgStor
 */
template <typename T = Counter, class Bin = NoBin>
class AverageVector : public Detail::VectorBase<T, Detail::AvgStor, Bin>
{ };

/**
 * A 2-Dimensional vecto of scalar stats.
 * @sa Stat, Vector2dBase, StatStor
 */
template <typename T = Counter, class Bin = NoBin>
class Vector2d : public Detail::Vector2dBase<T, Detail::StatStor, Bin>
{ };

/**
 * A simple distribution stat.
 * @sa Stat, DistBase, DistStor
 */
template <typename T = Counter, class Bin = NoBin>
class Distribution : public Detail::DistBase<T, Detail::DistStor, Bin>
{
  private:
    /** Base implementation. */
    typedef Detail::DistBase<T, Detail::DistStor, Bin> Base;
    /** The Parameter type. */
    typedef typename Detail::DistStor<T>::Params Params;

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
template <typename T = Counter, class Bin = NoBin>
class StandardDeviation : public Detail::DistBase<T, Detail::FancyStor, Bin>
{
  private:
    /** The base implementation */
    typedef Detail::DistBase<T, Detail::DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename Detail::DistStor<T>::Params Params;

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
template <typename T = Counter, class Bin = NoBin>
class AverageDeviation : public Detail::DistBase<T, Detail::AvgFancy, Bin>
{
  private:
    /** The base implementation */
    typedef Detail::DistBase<T, Detail::DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename Detail::DistStor<T>::Params Params;

  public:
    /**
     * Construct and initialize this distribution.
     */
    AverageDeviation() {
        bin.init(params);
        setInit();
    }
};

/**
 * A vector of distributions.
 * @sa Stat, VectorDistBase, DistStor
 */
template <typename T = Counter, class Bin = NoBin>
class VectorDistribution
    : public Detail::VectorDistBase<T, Detail::DistStor, Bin>
{
  private:
    /** The base implementation */
    typedef Detail::VectorDistBase<T, Detail::DistStor, Bin> Base;
    /** The parameter type. */
    typedef typename Detail::DistStor<T>::Params Params;

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
template <typename T = Counter, class Bin = NoBin>
class VectorStandardDeviation
    : public Detail::VectorDistBase<T, Detail::FancyStor, Bin>
{
  private:
    /** The base implementation */
    typedef Detail::VectorDistBase<T, Detail::FancyStor, Bin> Base;
    /** The parameter type. */
    typedef typename Detail::DistStor<T>::Params Params;

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
template <typename T = Counter, class Bin = NoBin>
class VectorAverageDeviation
    : public Detail::VectorDistBase<T, Detail::AvgFancy, Bin>
{
  private:
    /** The base implementation */
    typedef Detail::VectorDistBase<T, Detail::AvgFancy, Bin> Base;
    /** The parameter type. */
    typedef typename Detail::DistStor<T>::Params Params;

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
 * @sa Stat, ScalarStat, VectorStat, Node, Detail::Temp
 */
class Formula : public Detail::VectorStat
{
  private:
    /** The root of the tree which represents the Formula */
    Detail::NodePtr root;
    friend class Statistics::Detail::Temp;

  public:
    /**
     * Create and initialize thie formula, and register it with the database.
     */
    Formula() : VectorStat(true) { setInit(); }
    /**
     * Create a formula with the given root node, register it with the
     * database.
     * @param r The root of the expression tree.
     */
    Formula(Detail::Temp r) : VectorStat(true) {
        root = r;
        assert(size());
    }

    /**
     * Set an unitialized Formula to the given root.
     * @param r The root of the expression tree.
     * @return a reference to this formula.
     */
    const Formula &operator=(Detail::Temp r) {
        assert(!root && "Can't change formulas");
        root = r;
        assert(size());
        return *this;
    }

    /**
     * Add the given tree to the existing one.
     * @param r The root of the expression tree.
     * @return a reference to this formula.
     */
    const Formula &operator+=(Detail::Temp r) {
        using namespace Detail;
        if (root)
            root = NodePtr(new BinaryNode<std::plus<result_t> >(root, r));
        else
            root = r;
        assert(size());
        return *this;
    }

    /**
     * Return the result of the Fomula in a vector.  If there were no Vector
     * components to the Formula, then the vector is size 1.  If there were,
     * like x/y with x being a vector of size 3, then the result returned will
     * be x[0]/y, x[1]/y, x[2]/y, respectively.
     * @return The result vector.
     */
    const rvec_t &val() const { return root->val(); }
    /**
     * Return the total Formula result.  If there is a Vector component to this
     * Formula, then this is the result of the Formula if the formula is applied
     * after summing all the components of the Vector.  For example, if Formula
     * is x/y where x is size 3, then total() will return (x[1]+x[2]+x[3])/y.  If there is no
     * Vector component, total() returns the same value as the first entry in the rvec_t
     * val() returns.
     * @return The total of the result vector.
     */
    result_t total() const { return root->total(); }

    /**
     * Return the number of elements in the tree.
     */
    size_t size() const {
        if (!root)
            return 0;
        else
            return root->size();
    }
    /**
     * Return true if Formula is binned. i.e. any of its children nodes are binned
     *@return True if Formula is binned.
     */
    virtual bool binned() const { return root->binned(); }

    /**
     * Formulas don't need to be reset
     */
    virtual void reset() {}
};

/**
 * @}
 */

void check();
void dump(std::ostream &stream);
void reset();
void regReset(Callback *cb);

inline Detail::Temp
operator+(Detail::Temp l, Detail::Temp r)
{
    using namespace Detail;
    return NodePtr(new BinaryNode<std::plus<result_t> >(l, r));
}

inline Detail::Temp
operator-(Detail::Temp l, Detail::Temp r)
{
    using namespace Detail;
    return NodePtr(new BinaryNode<std::minus<result_t> >(l, r));
}

inline Detail::Temp
operator*(Detail::Temp l, Detail::Temp r)
{
    using namespace Detail;
    return NodePtr(new BinaryNode<std::multiplies<result_t> >(l, r));
}

inline Detail::Temp
operator/(Detail::Temp l, Detail::Temp r)
{
    using namespace Detail;
    return NodePtr(new BinaryNode<std::divides<result_t> >(l, r));
}

inline Detail::Temp
operator%(Detail::Temp l, Detail::Temp r)
{
    using namespace Detail;
    return NodePtr(new BinaryNode<std::modulus<result_t> >(l, r));
}

inline Detail::Temp
operator-(Detail::Temp l)
{
    using namespace Detail;
    return NodePtr(new UnaryNode<std::negate<result_t> >(l));
}

template <typename T>
inline Detail::Temp
constant(T val)
{
    using namespace Detail;
    return NodePtr(new ConstNode<T>(val));
}

template <typename T>
inline Detail::Temp
functor(T &val)
{
    using namespace Detail;
    return NodePtr(new FunctorNode<T>(val));
}

template <typename T>
inline Detail::Temp
scalar(T &val)
{
    using namespace Detail;
    return NodePtr(new ScalarNode<T>(val));
}

inline Detail::Temp
sum(Detail::Temp val)
{
    using namespace Detail;
    return NodePtr(new SumNode<std::plus<result_t> >(val));
}

extern bool PrintDescriptions;

} // namespace statistics

#endif // __STATISTICS_HH__

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

/** @file  */

/**
* @todo
*
* Generalized N-dimensinal vector
* documentation
* fix AvgStor
* key stats
* interval stats
*   -- these both can use the same function that prints out a
*   specific set of stats
* VectorStandardDeviation totals
*
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

#ifndef NAN
float __nan();
#define NAN (__nan())
#define __M5_NAN
#endif

#define STAT_DISPLAY_COMPAT

extern Tick curTick;

namespace Statistics {
typedef double result_t;
typedef std::vector<result_t> rvec_t;

typedef u_int32_t FormatFlags;
const FormatFlags none =	0x0000;
const FormatFlags total =	0x0001;
const FormatFlags pdf =		0x0002;
const FormatFlags nozero =	0x0004;
const FormatFlags nonan =	0x0008;
const FormatFlags cdf =		0x0010;
const FormatFlags dist = 	0x0020;
const FormatFlags __substat = 	0x8000;
const FormatFlags __reserved =  __substat;

namespace Detail {
//////////////////////////////////////////////////////////////////////
//
// Statistics Framework Base classes
//
//////////////////////////////////////////////////////////////////////
struct StatData;
struct SubData;

/**
 *The base class of all Stats.  This does NOT actually hold all the data, but
 *it does provide the means for accessing all the Stats data.
 */
class Stat
{
  protected:
    void setInit();
    StatData *mydata();
    const StatData *mydata() const;
    StatData *print();
    const SubData *mysubdata(int index) const;
    SubData *mysubdata_create(int index);

  public:
    virtual std::string myname() const;
    virtual std::string mysubname(int index) const;
    virtual std::string mydesc() const;
    virtual std::string mysubdesc(int index) const;
    virtual FormatFlags myflags() const;
    virtual bool dodisplay() const;
    virtual int myprecision() const;

  public:
    Stat(bool reg);
    virtual ~Stat() {}

    virtual void display(std::ostream &stream) const = 0;
    virtual size_t size() const = 0;
    virtual bool zero() const = 0;

    Stat &name(const std::string &name);
    Stat &desc(const std::string &desc);
    Stat &precision(int p);
    Stat &flags(FormatFlags f);
    Stat &prereq(const Stat &prereq);
    Stat &subname(int index, const std::string &name);
    Stat &subdesc(int index, const std::string &name);

  public:
    static bool less(Stat *stat1, Stat *stat2);

#ifdef STAT_DEBUG
    int number;
#endif
};

// Scalar stats involved in formulas
class ScalarStat : public Stat
{
  public:
    ScalarStat(bool reg) : Stat(reg) {}
    virtual result_t val() const = 0;
    virtual bool zero() const;
    virtual void display(std::ostream &stream) const;
};

void
VectorDisplay(std::ostream &stream, const std::string &myname,
              const std::vector<std::string> *mysubnames,
              const std::string &mydesc,
              const std::vector<std::string> *mysubdescs,
              int myprecision, FormatFlags myflags, const rvec_t &vec,
              result_t mytotal);

// Vector stats involved in formulas
class VectorStat : public Stat
{
  public:
    VectorStat(bool reg) : Stat(reg) {}
    virtual const rvec_t &val() const = 0;
    virtual result_t total() const = 0;
    virtual bool zero() const;
    virtual void display(std::ostream &stream) const;
};

//////////////////////////////////////////////////////////////////////
//
// Simple Statistics
//
//////////////////////////////////////////////////////////////////////
template <typename T>
struct StatStor
{
  public:
    struct Params { };

  private:
    T data;

  public:
    StatStor(const Params &) : data(T()) {}

    void set(T val, const Params &p) { data = val; }
    void inc(T val, const Params &p) { data += val; }
    void dec(T val, const Params &p) { data -= val; }
    result_t val(const Params &p) const { return (result_t)data; }
    T value(const Params &p) const { return data; }
};

template <typename T>
struct AvgStor
{
  public:
    struct Params { };

  private:
    T current;
    mutable result_t total;
    mutable Tick last;

  public:
    AvgStor(const Params &) : current(T()), total(0), last(0) { }

    void set(T val, const Params &p) {
        total += current * (curTick - last);
        last = curTick;
        current = val;
    }
    void inc(T val, const Params &p) { set(current + val, p); }
    void dec(T val, const Params &p) { set(current - val, p); }
    result_t val(const Params &p) const {
        total += current * (curTick - last);
        last = curTick;
        return (result_t)(total + current) / (result_t)(curTick + 1);
    }
    T value(const Params &p) const { return current; }
};

template <typename T, template <typename T> class Storage, class Bin>
class ScalarBase : public ScalarStat
{
  protected:
    typedef Storage<T> storage_t;
    typedef typename storage_t::Params params_t;
    typedef typename Bin::Bin<storage_t> bin_t;

  protected:
    bin_t bin;
    params_t params;

  protected:
    storage_t *data() { return bin.data(params); }
    const storage_t *data() const {
        return (const_cast<bin_t *>(&bin))->data(params);
    }

  protected:
    // Copying stats is not allowed
    ScalarBase(const ScalarBase &stat);
    const ScalarBase &operator=(const ScalarBase &);

  public:
    result_t val() const { return data()->val(params); }
    T value() const { return data()->value(params); }

  public:
    ScalarBase() : ScalarStat(true) {
        bin.init(params);
        setInit();
    }

  public:
    // Common operators for stats
    void operator++() { data()->inc(1, params); }
    void operator--() { data()->dec(1, params); }

    void operator++(int) { ++*this; }
    void operator--(int) { --*this; }

    template <typename U>
    void operator=(const U& v) { data()->set(v, params); }

    template <typename U>
    void operator+=(const U& v) { data()->inc(v, params); }

    template <typename U>
    void operator-=(const U& v) { data()->dec(v, params); }

    virtual size_t size() const { return 1; }
};

//////////////////////////////////////////////////////////////////////
//
// Vector Statistics
//
//////////////////////////////////////////////////////////////////////
template <typename T, template <typename T> class Storage, class Bin>
class ScalarProxy;

template <typename T, template <typename T> class Storage, class Bin>
class VectorBase : public VectorStat
{
  protected:
    typedef Storage<T> storage_t;
    typedef typename storage_t::Params params_t;
    typedef typename Bin::VectorBin<storage_t> bin_t;

  private:
    mutable rvec_t *vec;

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
    VectorBase(const VectorBase &stat);
    const VectorBase &operator=(const VectorBase &);

  public:
    const rvec_t &val() const {
        if (vec)
            vec->resize(size());
        else
            vec = new rvec_t(size());

        for (int i = 0; i < size(); ++i)
            (*vec)[i] = data(i)->val(params);

        return *vec;
    }

    result_t total() const {
        result_t total = 0.0;
        for (int i = 0; i < size(); ++i)
            total += data(i)->val(params);
        return total;
    }

  public:
    VectorBase() : VectorStat(true), vec(NULL) {}
    ~VectorBase() { if (vec) delete vec; }

    VectorBase &init(size_t size) {
        bin.init(size, params);
        setInit();

        return *this;
    }

    friend class ScalarProxy<T, Storage, Bin>;
    ScalarProxy<T, Storage, Bin> operator[](int index);

    virtual size_t size() const { return bin.size(); }
};

template <typename T, template <typename T> class Storage, class Bin>
class ScalarProxy : public ScalarStat
{
  protected:
    typedef Storage<T> storage_t;
    typedef typename storage_t::Params params_t;
    typedef typename Bin::VectorBin<storage_t> bin_t;

  private:
    bin_t *bin;
    params_t *params;
    int index;

  protected:
    storage_t *data() { return bin->data(index, *params); }
    const storage_t *data() const { return bin->data(index, *params); }

  public:
    result_t val() const { return data()->val(*params); }
    T value() const { return data()->value(*params); }

  public:
    ScalarProxy(bin_t &b, params_t &p, int i)
        : ScalarStat(false), bin(&b), params(&p), index(i)  {}
    ScalarProxy(const ScalarProxy &sp)
        : ScalarStat(false), bin(sp.bin), params(sp.params), index(sp.index) {}
    const ScalarProxy &operator=(const ScalarProxy &sp) {
        bin = sp.bin;
        params = sp.params;
        index = sp.index;
        return *this;
    }

  public:
    // Common operators for stats
    void operator++() { data()->inc(1, *params); }
    void operator--() { data()->dec(1, *params); }

    void operator++(int) { ++*this; }
    void operator--(int) { --*this; }

    template <typename U>
    void operator=(const U& v) { data()->set(v, *params); }

    template <typename U>
    void operator+=(const U& v) { data()->inc(v, *params); }

    template <typename U>
    void operator-=(const U& v) { data()->dec(v, *params); }

    virtual size_t size() const { return 1; }
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
     * This makes the assumption that if you're gonna subnames a 2d vector,
     * you're subnaming across all y
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

template <typename T>
struct DistStor
{
  public:
    struct Params
    {
        int min;
        int max;
        int bucket_size;
        int size;
    };

  private:
    T min_val;
    T max_val;
    T underflow;
    T overflow;
    std::vector<T> vec;

  public:
    DistStor(const Params &params)
        : min_val(INT_MAX), max_val(INT_MIN), underflow(0), overflow(0),
          vec(params.size) {
    }
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

    size_t size(const Params &) const { return vec.size(); }

    bool zero(const Params &params) const {
        if (underflow != 0 || overflow != 0)
            return true;

        int s = size(params);
        for (int i = 0; i < s; i++)
            if (vec[i] != 0)
                return true;

        return false;
    }

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
};

void FancyDisplay(std::ostream &stream, const std::string &name,
                  const std::string &desc, int precision, FormatFlags flags,
                  result_t mean, result_t variance);
template <typename T>
struct FancyStor
{
  public:
    struct Params {};

  private:
    T sum;
    T squares;
    int total;

  public:
    FancyStor(const Params &) : sum(0), squares(0), total(0) {}

    void sample(T val, int number, const Params &) {
        T value = val * number;
        sum += value;
        squares += value * value;
        total += number;
    }
    void display(std::ostream &stream, const std::string &name,
                 const std::string &desc, int precision, FormatFlags flags,
                 const Params &) const {

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

    size_t size(const Params &) const { return 1; }
    bool zero(const Params &) const { return total == 0; }
};

template <typename T>
struct AvgFancy
{
  public:
    struct Params {};

  private:
    T sum;
    T squares;

  public:
    AvgFancy(const Params &) : sum(0), squares(0) {}

    void sample(T val, int number, const Params& p) {
        T value = val * number;
        sum += value;
        squares += value * value;
    }
    void display(std::ostream &stream, const std::string &name,
                 const std::string &desc, int precision, FormatFlags flags,
                 const Params &params) const {
        result_t mean = sum / curTick;
        result_t variance = (squares - sum * sum) / curTick;

        FancyDisplay(stream, name, desc, precision, flags, mean, variance);
    }

    size_t size(const Params &params) const { return 1; }
    bool zero(const Params &params) const { return sum == 0; }
};

template <typename T, template <typename T> class Storage, class Bin>
class DistBase : public Stat
{
  protected:
    typedef Storage<T> storage_t;
    typedef typename storage_t::Params params_t;
    typedef typename Bin::Bin<storage_t> bin_t;

  protected:
    bin_t bin;
    params_t params;

  protected:
    storage_t *data() { return bin.data(params); }
    const storage_t *data() const {
        return (const_cast<bin_t *>(&bin))->data(params);
    }

  protected:
    // Copying stats is not allowed
    DistBase(const DistBase &stat);
    const DistBase &operator=(const DistBase &);

  public:
    DistBase() : Stat(true) { }
    ~DistBase() { }

    template <typename U>
    void sample(const U& v, int n = 1) { data()->sample(v, n, params); }

    virtual size_t size() const { return data()->size(params); }
    virtual bool zero() const { return data()->zero(params); }
    virtual void display(std::ostream &stream) const {
        data()->display(stream, myname(), mydesc(), myprecision(), myflags(),
                        params);
    }
};

template <typename T, template <typename T> class Storage, class Bin>
class VectorDistProxy;

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

    friend class VectorDistProxy<T, Storage, Bin>;
    VectorDistProxy<T, Storage, Bin> operator[](int index);
    const VectorDistProxy<T, Storage, Bin> operator[](int index) const;

    virtual size_t size() const { return bin.size(); }
    virtual bool zero() const { return false; }
    virtual void display(std::ostream &stream) const;
};

template <typename T, template <typename T> class Storage, class Bin>
class VectorDistProxy : public Stat
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
    VectorDistProxy(const VectorDistBase<T, Storage, Bin> &s, int i)
        : Stat(false), cstat(&s), index(i) {}
    VectorDistProxy(const VectorDistProxy &sp)
        : Stat(false), cstat(sp.cstat), index(sp.index) {}
    const VectorDistProxy &operator=(const VectorDistProxy &sp) {
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
};

template <typename T, template <typename T> class Storage, class Bin>
inline VectorDistProxy<T, Storage, Bin>
VectorDistBase<T, Storage, Bin>::operator[](int index)
{
    assert (index >= 0 && index < size());
    return VectorDistProxy<T, Storage, Bin>(*this, index);
}

template <typename T, template <typename T> class Storage, class Bin>
inline const VectorDistProxy<T, Storage, Bin>
VectorDistBase<T, Storage, Bin>::operator[](int index) const
{
    assert (index >= 0 && index < size());
    return VectorDistProxy<T, Storage, Bin>(*this, index);
}

/**
 * @todo Need a way to print Distribution totals across the Vector
 */
template <typename T, template <typename T> class Storage, class Bin>
void
VectorDistBase<T, Storage, Bin>::display(std::ostream &stream) const
{
    for (int i = 0; i < size(); ++i) {
        VectorDistProxy<T, Storage, Bin> proxy(*this, i);
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
class Node : public RefCounted
{
  public:
    virtual size_t size() const = 0;
    virtual const rvec_t &val() const = 0;
    virtual result_t total() const = 0;
};

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
};

class Temp
{
  private:
    NodePtr node;

  public:
    Temp(NodePtr n) : node(n) {}
    Temp(const ScalarStat &s) : node(new ScalarStatNode(s)) {}
    template <typename T, template <typename T> class Storage, class Bin>
    Temp(const ScalarProxy<T, Storage, Bin> &p)
        : node(new ScalarProxyNode<T, Storage, Bin>(p)) {}
    Temp(const VectorStat &s) : node(new VectorStatNode(s)) {}

#define TempSCALAR(T) \
    Temp(T value) : node(new ConstNode<T>(value)) {}

    TempSCALAR(  signed char);
    TempSCALAR(unsigned char);
    TempSCALAR(  signed short);
    TempSCALAR(unsigned short);
    TempSCALAR(  signed int);
    TempSCALAR(unsigned int);
    TempSCALAR(  signed long);
    TempSCALAR(unsigned long);
    TempSCALAR(  signed long long);
    TempSCALAR(unsigned long long);
    TempSCALAR(float);
    TempSCALAR(double);
#undef TempSCALAR

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
    ~BinBase();
};

} // namespace Detail

template <class BinType>
struct StatBin : public Detail::BinBase
{
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

    explicit StatBin(size_t size = 1024) : Detail::BinBase(size) {}

    char *memory(off_t off) {
        assert(offset() <= size());
        return Detail::BinBase::memory() + off;
    }

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

      private:
        char ptr[sizeof(Storage)];

      public:
        bool initialized() const { return true; }
        void init(const Params &params) {
            new (ptr) Storage(params);
        }
        int size() const{ return 1; }
        Storage *data(const Params &params) {
            assert(initialized());
            return reinterpret_cast<Storage *>(ptr);
        }
    };

    template <class Storage>
    struct VectorBin
    {
      public:
        typedef typename Storage::Params Params;

      private:
        char *ptr;
        int _size;

      public:
        VectorBin() : ptr(NULL) { }
        ~VectorBin() {
            if (initialized())
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
    };
};

//////////////////////////////////////////////////////////////////////
//
// Visible Statistics Types
//
//////////////////////////////////////////////////////////////////////
/**@defgroup VStats  VisibleStatTypes
 */

/** @ingroup VStats
 *This is the simplest counting stat. Default type is Counter, but can be
 *anything (like double, int, etc).  To bin, just designate the name of the bin
 * when declaring.  It can be used like a regular Counter.
 *Example:  Stat<> foo;
 *foo += num_foos;
 */
template <typename T = Counter, class Bin = NoBin>
class Scalar : public Detail::ScalarBase<T, Detail::StatStor, Bin>
{
  public:
    typedef Detail::ScalarBase<T, Detail::StatStor, Bin> Base;

/** sets Stat equal to value of type U */
    template <typename U>
    void operator=(const U& v) { Base::operator=(v); }
};

/** @ingroup VStats
 *This calculates averages over number of cycles.  Additionally, the update per
 *cycle is implicit if there is no change.  In other words, if you want to know
 *the average number of instructions in the IQ per cycle, then you can use this
 * stat and not have to update it on cycles where there is no change.
 */
template <typename T = Counter, class Bin = NoBin>
class Average : public Detail::ScalarBase<T, Detail::AvgStor, Bin>
{
  public:
    typedef Detail::ScalarBase<T, Detail::AvgStor, Bin> Base;

/** sets Average equalt to value of type U*/
    template <typename U>
    void operator=(const U& v) { Base::operator=(v); }
};

/** @ingroup VStats
 *This is a vector of type T, ideally suited to track stats across something like
 * SMT threads.
 */
template <typename T = Counter, class Bin = NoBin>
class Vector : public Detail::VectorBase<T, Detail::StatStor, Bin>
{ };

/** @ingroup VStats
 *This is a vector of Averages of type T
 */
template <typename T = Counter, class Bin = NoBin>
class AverageVector : public Detail::VectorBase<T, Detail::AvgStor, Bin>
{ };

/** @ingroup VStats
 *This is a 2-dimensional vector.  Intended  usage is for something like tracking  a
 * Vector stat across another Vector like  SMT threads.
 */
template <typename T = Counter, class Bin = NoBin>
class Vector2d : public Detail::Vector2dBase<T, Detail::StatStor, Bin>
{ };

/** @ingroup VStats
 * This is essentially a Vector, but with minor differences.  Where a
 * Vector's index maps directly to what it's tracking, a Distribution's index can
 * map to an arbitrary bucket type.  For example, you could map 1-8 to bucket 0
 * of a Distribution, and if ever there are 1-8 instructions within an IQ, increment
 * bucket 0.
 */
template <typename T = Counter, class Bin = NoBin>
class Distribution : public Detail::DistBase<T, Detail::DistStor, Bin>
{
  private:
    typedef Detail::DistBase<T, Detail::DistStor, Bin> Base;
    typedef typename Detail::DistStor<T>::Params Params;

  public:
    /**
     *This must be called to set some data members of the distribution
     *as well as to allocate the appropriate storage size.
     *@param min The minimum value of the Distribution
     *@param max The maximum value of the Distribution (NOT the size!)
     *@param bkt The size of the buckets to indicate mapping.  I.e. if you have
     *min=0, max=15, bkt=8, you will have two buckets, and anything from 0-7
     *will go into bucket 0, and anything from 8-15 be in bucket 1.
     *@return the Distribution itself.
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

/** @ingroup VStats
 *This has the functionality of a standard deviation built into it.  Update it
 *every cycle, and at the end you will have the standard deviation.
 */
template <typename T = Counter, class Bin = NoBin>
class StandardDeviation : public Detail::DistBase<T, Detail::FancyStor, Bin>
{
  private:
    typedef Detail::DistBase<T, Detail::DistStor, Bin> Base;
    typedef typename Detail::DistStor<T>::Params Params;

  public:
    StandardDeviation() {
        bin.init(params);
        setInit();
    }
};

/** @ingroup VStats
 *This also calculates standard deviations, but there is no need to
 *update every cycle if there is no change, the stat will update for you.
 */
template <typename T = Counter, class Bin = NoBin>
class AverageDeviation : public Detail::DistBase<T, Detail::AvgFancy, Bin>
{
  private:
    typedef Detail::DistBase<T, Detail::DistStor, Bin> Base;
    typedef typename Detail::DistStor<T>::Params Params;

  public:
    AverageDeviation() {
        bin.init(params);
        setInit();
    }
};

/** @ingroup VStats
 *This is a vector of Distributions. (The complexity increases!). Intended usage
 * is for something like tracking a distribution across a vector like SMT threads.
 */
template <typename T = Counter, class Bin = NoBin>
class VectorDistribution
    : public Detail::VectorDistBase<T, Detail::DistStor, Bin>
{
  private:
    typedef Detail::VectorDistBase<T, Detail::DistStor, Bin> Base;
    typedef typename Detail::DistStor<T>::Params Params;

  public:
    /**
     *This must be called to set some data members and allocate storage space.
     *@param size The size of the Vector
     *@param min The minumum value of the Distribution
     *@param max The maximum value of the Distribution (NOT the size)
     *@param bkt The range of the bucket.  I.e if min=0, max=15, and bkt=8,
     *then 0-7 will be bucket 0, and 8-15 will be bucket 1.
     *@return return the VectorDistribution itself.
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

/** @ingroup VStats
 *This is a vector of Standard Deviations.  Intended usage is for tracking
 *Standard Deviations across a vector like SMT threads.
 */
template <typename T = Counter, class Bin = NoBin>
class VectorStandardDeviation
    : public Detail::VectorDistBase<T, Detail::FancyStor, Bin>
{
  private:
    typedef Detail::VectorDistBase<T, Detail::FancyStor, Bin> Base;
    typedef typename Detail::DistStor<T>::Params Params;

  public:
    /** This must be called to initialize some data members and allocate
     * approprate storage space for the stat.
     *@param size The size of the Vector
     * @return the VectorStandardDeviation itself.
     */
    VectorStandardDeviation &init(int size) {
        bin.init(size, params);
        setInit();

        return *this;
    }
};

/** @ingroup VStats
 * This is a vector of Average Deviations.  Intended usage is for tracking
 *Average Deviations across a vector like SMT threads.
 */
template <typename T = Counter, class Bin = NoBin>
class VectorAverageDeviation
    : public Detail::VectorDistBase<T, Detail::AvgFancy, Bin>
{
  private:
    typedef Detail::VectorDistBase<T, Detail::AvgFancy, Bin> Base;
    typedef typename Detail::DistStor<T>::Params Params;

  public:
/** This must be called to initialize some data members and allocate
 * approprate storage space for the stat.
 *@param size The size of the Vector
 * @return The VectorAverageDeviation itself.
 */
    VectorAverageDeviation &init(int size) {
        bin.init(size, params);
        setInit();

        return *this;
    }
};

/** @ingroup VStats
 *This is a formula type.  When defining it, you can just say:
 *Formula foo = manchu + 3 / bar;
 *The calculations for Formulas are all done at the end of the simulation, this
 *really is just a definition of how to calculate at the end.
 */
class Formula : public Detail::VectorStat
{
  private:
    /** The root of the tree which represents the Formula */
    Detail::NodePtr root;
    friend class Detail::Temp;

  public:
    Formula() : VectorStat(true) { setInit(); }
    Formula(Detail::Temp r) : VectorStat(true) {
        root = r;
        assert(size());
    }

    const Formula &operator=(Detail::Temp r) {
        assert(!root && "Can't change formulas");
        root = r;
        assert(size());
        return *this;
    }

    const Formula &operator+=(Detail::Temp r) {
        using namespace Detail;
        if (root)
            root = NodePtr(new BinaryNode<std::plus<result_t> >(root, r));
        else
            root = r;
        assert(size());
        return *this;
    }

    const rvec_t &val() const { return root->val(); }
    result_t total() const { return root->total(); }

    size_t size() const {
        if (!root)
            return 0;
        else
            return root->size();
    }
};

void check();
void dump(std::ostream &stream);

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

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

#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <string>
#include <sstream>

#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/misc.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "sim/universe.hh"

#ifdef __M5_NAN
float
__nan()
{
    union {
        uint32_t ui;
        float f;
    } nan;

    nan.ui = 0x7fc00000;
    return nan.f;
}
#endif

#ifdef STAT_DEBUG
static int total_stats = 0;
#endif

using namespace std;

// This is a hack to get this parameter from the old stats package.
namespace Statistics {
bool PrintDescriptions = true;
DisplayMode default_mode = mode_simplescalar;

namespace Database
{
    class Data
    {
      private:
        typedef list<StatData *> list_t;
        typedef map<void *, StatData *> map_t;

        list<MainBin *> bins;
        map<const MainBin *, string > bin_names;
        list_t binnedStats;

        list_t allStats;
        list_t printStats;
        map_t statMap;

      public:
        void dump(ostream &stream);

        StatData *find(void *stat);
        void mapStat(void *stat, StatData *data);

        void check();
        void reset();
        void regBin(MainBin *bin, string name);
        void regPrint(void *stat);
    };


void
Data::dump(ostream &stream)
{
#ifndef FS_MEASURE
    list_t::iterator i = printStats.begin();
    list_t::iterator end = printStats.end();
    while (i != end) {
        StatData *stat = *i;
        if (stat->binned())
            binnedStats.push_back(stat);
        ++i;
    }
#endif //FS_MEASURE

    list<MainBin *>::iterator j = bins.begin();
    list<MainBin *>::iterator bins_end=bins.end();

    if (!bins.empty()) {
        ccprintf(stream, "PRINTING BINNED STATS\n");
        while (j != bins_end) {
            (*j)->activate();
            map<const MainBin  *, string>::const_iterator iter;
            iter = bin_names.find(*j);
            if (iter == bin_names.end())
                panic("a binned stat not found in names map!");
            ccprintf(stream,"---%s Bin------------\n", (*iter).second);

#ifdef FS_MEASURE
            list_t::iterator i = printStats.begin();
            list_t::iterator end = printStats.end();
#else
            list_t::iterator i = binnedStats.begin();
            list_t::iterator end = binnedStats.end();
#endif
            while (i != end) {
                StatData *stat = *i;
                if (stat->dodisplay())
                    stat->display(stream);
                ++i;
            }
            ++j;
            ccprintf(stream, "---------------------------------\n");
        }
#ifndef FS_MEASURE
        ccprintf(stream, "**************ALL STATS************\n");
#endif
    }

/**
 * get bin totals working, then print the stat here (as total), even if
 * its' binned.  (this is only for the case you selectively bin a few stats
 */
#ifndef FS_MEASURE
    list_t::iterator k = printStats.begin();
    list_t::iterator endprint = printStats.end();
    while (k != endprint) {
        StatData *stat = *k;
        if (stat->dodisplay() /*&& !stat->binned()*/)
            stat->display(stream);
        ++k;
    }
#endif
}

StatData *
Data::find(void *stat)
{
    map_t::const_iterator i = statMap.find(stat);

    if (i == statMap.end())
        return NULL;

    return (*i).second;
}

void
Data::check()
{
    list_t::iterator i = allStats.begin();
    list_t::iterator end = allStats.end();

    while (i != end) {
        StatData *stat = *i;
        assert(stat);
        stat->check();
        ++i;
    }
}

void
Data::reset()
{
    list_t::iterator i = allStats.begin();
    list_t::iterator end = allStats.end();
    while (i != end) {
        StatData *stat = *i;
        stat->reset();
        ++i;
    }

    MainBin *orig = MainBin::curBin();

    list<MainBin *>::iterator bi = bins.begin();
    list<MainBin *>::iterator be = bins.end();
    while (bi != be) {
        MainBin *bin = *bi;
        bin->activate();

        i = allStats.begin();
        while (i != end) {
            StatData *stat = *i;
            stat->reset();
            ++i;
        }
        ++bi;
    }

    if (orig)
        orig->activate();
}

void
Data::mapStat(void *stat, StatData *data)
{
    if (statMap.find(stat) != statMap.end())
        panic("shouldn't register stat twice!");

    allStats.push_back(data);

#ifndef NDEBUG
    bool success =
#endif
        (statMap.insert(make_pair(stat, data))).second;
    assert(statMap.find(stat) != statMap.end());
    assert(success && "this should never fail");
}

void
Data::regBin(MainBin *bin, string name)
{
    if (bin_names.find(bin) != bin_names.end())
        panic("shouldn't register bin twice");

    bins.push_back(bin);

#ifndef NDEBUG
    bool success =
#endif
        (bin_names.insert(make_pair(bin,name))).second;
    assert(bin_names.find(bin) != bin_names.end());
    assert(success && "this should not fail");

    cprintf("registering %s\n", name);
}

void
Data::regPrint(void *stat)
{
    StatData *data = find(stat);

    if (!data->print) {
        data->print = true;

        list_t::iterator j = printStats.insert(printStats.end(), data);
        inplace_merge(printStats.begin(), j,
                      printStats.end(), StatData::less);
    }

}

Data &
StatDB()
{
    static Data db;
    return db;
}

}

StatData *
DataAccess::find() const
{
    return Database::StatDB().find(const_cast<void *>((const void *)this));
}

void
DataAccess::map(StatData *data)
{
    Database::StatDB().mapStat(this, data);
}

StatData *
DataAccess::statData()
{
    StatData *ptr = find();
    assert(ptr);
    return ptr;
}

const StatData *
DataAccess::statData() const
{
    const StatData *ptr = find();
    assert(ptr);
    return ptr;
}

void
DataAccess::setInit()
{
    statData()->init = true;
}

void
DataAccess::setPrint()
{
    Database::StatDB().regPrint(this);
}

StatData::~StatData()
{
}

bool
StatData::less(StatData *stat1, StatData *stat2)
{
    const string &name1 = stat1->name;
    const string &name2 = stat2->name;

    vector<string> v1;
    vector<string> v2;

    tokenize(v1, name1, '.');
    tokenize(v2, name2, '.');

    int last = min(v1.size(), v2.size()) - 1;
    for (int i = 0; i < last; ++i)
        if (v1[i] != v2[i])
            return v1[i] < v2[i];

    // Special compare for last element.
    if (v1[last] == v2[last])
        return v1.size() < v2.size();
    else
        return v1[last] < v2[last];

    return false;
}

bool
StatData::check() const
{
    if (!init) {
#ifdef STAT_DEBUG
        cprintf("this is stat number %d\n",(*i)->number);
#endif
        panic("Not all stats have been initialized");
        return false;
    }

    if (print && name.empty()) {
        panic("all printable stats must be named");
        return false;
    }

    return true;
}

string
ValueToString(result_t value, DisplayMode mode, int precision)
{
    stringstream val;

    if (!isnan(value)) {
        if (precision != -1)
            val.precision(precision);
        else if (value == rint(value))
            val.precision(0);

        val.unsetf(ios::showpoint);
        val.setf(ios::fixed);
        val << value;
    } else {
        val << (mode == mode_m5 ? "no value" : "<err: div-0>");
    }

    return val.str();
}

struct ScalarPrint
{
    result_t value;
    string name;
    string desc;
    int precision;
    DisplayMode mode;
    FormatFlags flags;
    result_t pdf;
    result_t cdf;

    ScalarPrint()
        : value(0.0), precision(0), mode(default_mode), flags(0),
          pdf(NAN), cdf(NAN)
    {}

    void operator()(ostream &stream) const;
};

void
ScalarPrint::operator()(ostream &stream) const
{
    if (flags & nozero && value == 0.0 ||
        flags & nonan && isnan(value))
        return;

    stringstream pdfstr, cdfstr;

    if (!isnan(pdf))
        ccprintf(pdfstr, "%.2f%%", pdf * 100.0);

    if (!isnan(cdf))
        ccprintf(cdfstr, "%.2f%%", cdf * 100.0);

    if (mode == mode_simplescalar && flags & __substat) {
        ccprintf(stream, "%32s %12s %10s %10s", name,
                 ValueToString(value, mode, precision),
                 pdfstr, cdfstr);
    } else {
        ccprintf(stream, "%-40s %12s %10s %10s", name,
                 ValueToString(value, mode, precision), pdfstr, cdfstr);
    }

    if (PrintDescriptions) {
        if (!desc.empty())
            ccprintf(stream, " # %s", desc);
    }
    stream << endl;
}

struct VectorPrint
{
    string name;
    string desc;
    vector<string> subnames;
    vector<string> subdescs;
    int precision;
    DisplayMode mode;
    FormatFlags flags;
    rvec_t vec;
    result_t total;

    VectorPrint()
        : subnames(0), subdescs(0), precision(-1), mode(default_mode),
          flags(0), total(NAN)
    {}

    void operator()(ostream &stream) const;
};

void
VectorPrint::operator()(std::ostream &stream) const
{
    int _size = vec.size();
    result_t _total = 0.0;

    if (flags & (pdf | cdf)) {
        for (int i = 0; i < _size; ++i) {
            _total += vec[i];
        }
    }

    string base = name + ((mode == mode_simplescalar) ? "_" : "::");

    ScalarPrint print;
    print.name = name;
    print.desc = desc;
    print.precision = precision;
    print.flags = flags;

    bool havesub = !subnames.empty();

    if (_size == 1) {
        print.value = vec[0];
        print(stream);
    } else if (mode == mode_m5) {
        for (int i = 0; i < _size; ++i) {
            if (havesub && (i >= subnames.size() || subnames[i].empty()))
                continue;

            print.name = base + (havesub ? subnames[i] : to_string(i));
            print.desc = subdescs.empty() ? desc : subdescs[i];
            print.value = vec[i];

            if (_total && (flags & pdf)) {
                print.pdf = vec[i] / _total;
                print.cdf += print.pdf;
            }

            print(stream);
        }

        if (flags & ::Statistics::total) {
            print.name = base + "total";
            print.desc = desc;
            print.value = total;
            print(stream);
        }
    } else {
        if (flags & ::Statistics::total) {
            print.value = total;
            print(stream);
        }

        result_t _pdf = 0.0;
        result_t _cdf = 0.0;
        if (flags & dist) {
            ccprintf(stream, "%s.start_dist\n", name);
            for (int i = 0; i < _size; ++i) {
                print.name = havesub ? subnames[i] : to_string(i);
                print.desc = subdescs.empty() ? desc : subdescs[i];
                print.flags |= __substat;
                print.value = vec[i];

                if (_total) {
                    _pdf = vec[i] / _total;
                    _cdf += _pdf;
                }

                if (flags & pdf)
                    print.pdf = _pdf;
                if (flags & cdf)
                    print.cdf = _cdf;

                print(stream);
            }
            ccprintf(stream, "%s.end_dist\n", name);
        } else {
            for (int i = 0; i < _size; ++i) {
                if (havesub && subnames[i].empty())
                    continue;

                print.name = base;
                print.name += havesub ? subnames[i] : to_string(i);
                print.desc = subdescs.empty() ? desc : subdescs[i];
                print.value = vec[i];

                if (_total) {
                    _pdf = vec[i] / _total;
                    _cdf += _pdf;
                } else {
                    _pdf = _cdf = NAN;
                }

                if (flags & pdf) {
                    print.pdf = _pdf;
                    print.cdf = _cdf;
                }

                print(stream);
            }
        }
    }
}

struct DistPrint
{
    string name;
    string desc;
    int precision;
    DisplayMode mode;
    FormatFlags flags;

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

    void operator()(ostream &stream) const;
};

void
DistPrint::operator()(ostream &stream) const
{
    if (fancy) {
        ScalarPrint print;
        string base = name + ((mode == mode_m5) ? "::" : "_");

        print.precision = precision;
        print.flags = flags;
        print.desc = desc;

        print.name = base + "mean";
        print.value = samples ? sum / samples : NAN;
        print(stream);

        print.name = base + "stdev";
        print.value = samples ? sqrt((samples * squares - sum * sum) /
                                     (samples * (samples - 1.0))) : NAN;
        print(stream);

        print.name = "**Ignore: " + base + "TOT";
        print.value = samples;
        print(stream);
        return;
    }

    assert(size == vec.size());

    result_t total = 0.0;

    total += underflow;
    for (int i = 0; i < size; ++i)
        total += vec[i];
    total += overflow;

    string base = name + (mode == mode_m5 ? "::" : ".");

    ScalarPrint print;
    print.desc = (mode == mode_m5) ? desc : "";
    print.precision = precision;
    print.mode = mode;
    print.flags = flags;

    if (mode == mode_simplescalar) {
        ccprintf(stream, "%-42s", base + "start_dist");
        if (PrintDescriptions && !desc.empty())
            ccprintf(stream, "                     # %s", desc);
        stream << endl;
    }

    print.name = base + "samples";
    print.value = samples;
    print(stream);

    print.name = base + "min_value";
    print.value = min_val;
    print(stream);

    if (mode == mode_m5 || underflow > 0.0) {
        print.name = base + "underflows";
        print.value = underflow;
        if (mode == mode_m5 && total) {
            print.pdf = underflow / total;
            print.cdf += print.pdf;
        }
        print(stream);
    }


    if (mode == mode_m5) {
        for (int i = 0; i < size; ++i) {
            stringstream namestr;
            namestr << name;

            int low = i * bucket_size + min;
            int high = ::min((i + 1) * bucket_size + min - 1, max);
            namestr << low;
            if (low < high)
                namestr << "-" << high;

            print.name = namestr.str();
            print.value = vec[i];
            if (total) {
                print.pdf = vec[i] / total;
                print.cdf += print.pdf;
            }
            print(stream);
        }

    } else {
        int _min;
        result_t _pdf;
        result_t _cdf = 0.0;

        print.flags = flags | __substat;

        for (int i = 0; i < size; ++i) {
            if (flags & nozero && vec[i] == 0.0 ||
                flags & nonan && isnan(vec[i]))
                continue;

            _min = i * bucket_size + min;
            _pdf = vec[i] / total * 100.0;
            _cdf += _pdf;


            print.name = ValueToString(_min, mode, 0);
            print.value = vec[i];
            print.pdf = (flags & pdf) ? _pdf : NAN;
            print.cdf = (flags & cdf) ? _cdf : NAN;
            print(stream);
        }

        print.flags = flags;
        if (flags & (pdf || cdf)) {
            print.pdf = NAN;
            print.cdf = NAN;
        }
    }

    if (mode == mode_m5 || overflow > 0.0) {
        print.name = base + "overflows";
        print.value = overflow;
        if (mode == mode_m5 && total) {
            print.pdf = overflow / total;
            print.cdf += print.pdf;
        }
        print(stream);
    }

    print.pdf = NAN;
    print.cdf = NAN;

    if (mode != mode_simplescalar) {
        print.name = base + "total";
        print.value = total;
        print(stream);
    }

    print.name = base + "max_value";
    print.value = max_val;
    print(stream);

    if (mode != mode_simplescalar && samples != 0) {
        print.name = base + "mean";
        print.value = sum / samples;
        print(stream);

        print.name = base + "stdev";
        print.value = sqrt((samples * squares - sum * sum) /
                           (samples * (samples - 1.0)));
        print(stream);
    }

    if (mode == mode_simplescalar)
        ccprintf(stream, "%send_dist\n\n", base);
}

void
ScalarDataBase::display(ostream &stream) const
{
    ScalarPrint print;
    print.value = val();
    print.name = name;
    print.desc = desc;
    print.precision = precision;
    print.flags = flags;

    print(stream);
}

void
VectorDataBase::display(ostream &stream) const
{
    int size = this->size();
    const_cast<VectorDataBase *>(this)->update();

    VectorPrint print;

    print.name = name;
    print.desc = desc;
    print.mode = mode;
    print.flags = flags;
    print.precision = precision;
    print.vec = val();
    print.total = total();

    for (int i = 0; i < size; ++i) {
        if (!subnames[i].empty()) {
            print.subnames = subnames;
            print.subnames.resize(size);
            for (int i = 0; i < size; ++i) {
                if (!subnames[i].empty() && !subdescs[i].empty()) {
                    print.subdescs = subdescs;
                    print.subdescs.resize(size);
                    break;
                }
            }
            break;
        }
    }


    print(stream);
}

void
Vector2dDataBase::display(ostream &stream) const
{
    const_cast<Vector2dDataBase *>(this)->update();

    bool havesub = false;
    VectorPrint print;

    print.subnames = y_subnames;
    print.mode = mode;
    print.flags = flags;
    print.precision = precision;

    if (!subnames.empty()) {
        for (int i = 0; i < x; ++i)
            if (!subnames[i].empty())
                havesub = true;
    }

    rvec_t tot_vec(y);
    result_t super_total = 0.0;
    for (int i = 0; i < x; ++i) {
        if (havesub && (i >= subnames.size() || subnames[i].empty()))
            continue;

        int iy = i * y;
        rvec_t yvec(y);

        result_t total = 0.0;
        for (int j = 0; j < y; ++j) {
            yvec[j] = vec[iy + j];
            tot_vec[j] += yvec[j];
            total += yvec[j];
            super_total += yvec[j];
        }

        print.name = name + "_" + (havesub ? subnames[i] : to_string(i));
        print.desc = desc;
        print.vec = yvec;
        print.total = total;
        print(stream);
    }

    if ((flags & ::Statistics::total) && (x > 1)) {
        print.name = name;
        print.desc = desc;
        print.vec = tot_vec;
        print.total = super_total;
        print(stream);
    }
}

void
DistDataBase::display(ostream &stream) const
{
    const_cast<DistDataBase *>(this)->update();

    DistPrint print;

    print.name = name;
    print.desc = desc;
    print.precision = precision;
    print.mode = mode;
    print.flags = flags;

    print.min_val = data.min_val;
    print.max_val = data.max_val;
    print.underflow = data.underflow;
    print.overflow = data.overflow;
    print.vec = data.vec;
    print.sum = data.sum;
    print.squares = data.squares;
    print.samples = data.samples;

    print.min = data.min;
    print.max = data.max;
    print.bucket_size = data.bucket_size;
    print.size = data.size;
    print.fancy = data.fancy;

    print(stream);
}

void
VectorDistDataBase::display(ostream &stream) const
{
    const_cast<VectorDistDataBase *>(this)->update();

    for (int i = 0; i < size(); ++i) {
        DistPrint print;

        print.name = name +
            (subnames[i].empty() ? ("_" + to_string(i)) : subnames[i]);
        print.desc = subdescs[i].empty() ? desc : subdescs[i];
        print.precision = precision;
        print.mode = mode;
        print.flags = flags;

        print.min_val = data[i].min_val;
        print.max_val = data[i].max_val;
        print.underflow = data[i].underflow;
        print.overflow = data[i].overflow;
        print.vec = data[i].vec;
        print.sum = data[i].sum;
        print.squares = data[i].squares;
        print.samples = data[i].samples;

        print.min = data[i].min;
        print.max = data[i].max;
        print.bucket_size = data[i].bucket_size;
        print.size = data[i].size;
        print.fancy = data[i].fancy;

        print(stream);
    }
}

void
FormulaBase::val(rvec_t &vec) const
{
    vec = root->val();
}

result_t
FormulaBase::total() const
{
    return root->total();
}

size_t
FormulaBase::size() const
{
    if (!root)
        return 0;
    else
        return root->size();
}

bool
FormulaBase::binned() const
{
    return root->binned();
}

void
FormulaBase::reset()
{
}

bool
FormulaBase::zero() const
{
    rvec_t vec;
    val(vec);
    for (int i = 0; i < vec.size(); ++i)
        if (vec[i] != 0.0)
            return false;
    return true;
}

void
FormulaBase::update(StatData *)
{
}

Formula::Formula()
{
    setInit();
}

Formula::Formula(Temp r)
{
    root = r;
    assert(size());
}

const Formula &
Formula::operator=(Temp r)
{
    assert(!root && "Can't change formulas");
    root = r;
    assert(size());
    return *this;
}

const Formula &
Formula::operator+=(Temp r)
{
    if (root)
        root = NodePtr(new BinaryNode<std::plus<result_t> >(root, r));
    else
        root = r;
    assert(size());
    return *this;
}

MainBin::MainBin(const string &name)
    : _name(name), mem(NULL), memsize(-1)
{
    Database::StatDB().regBin(this, name);
}

MainBin::~MainBin()
{
    if (mem)
        delete [] mem;
}

char *
MainBin::memory(off_t off)
{
    if (memsize == -1)
        memsize = CeilPow2((size_t) offset());

    if (!mem) {
        mem = new char[memsize];
        memset(mem, 0, memsize);
    }

    assert(offset() <= size());
    return mem + off;
}

void
check()
{
    Database::StatDB().check();
}

void
dump(ostream &stream)
{
    Database::StatDB().dump(stream);
}

CallbackQueue resetQueue;

void
registerResetCallback(Callback *cb)
{
    resetQueue.add(cb);
}

void
reset()
{
    Database::StatDB().reset();
    resetQueue.process();
}

} // namespace Statistics

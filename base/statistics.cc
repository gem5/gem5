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

#include <math.h>

#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/intmath.hh"
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

namespace Detail {
/**
 * Struct to contain a name and description of statistic subfield.
 */
struct SubData
{
    /** Subfield name. */
    string name;
    /** Subfield desc. */
    string desc;
};

/**
 * Struct to contain print data of a Stat.
 */
struct StatData
{
    /**
     * Create this struct.
     */
    StatData();
    /**
     * Destructor.
     */
    ~StatData();

    /** True if the stat has been initialized. */
    bool init;
    /** True if the stat should be printed. */
    bool print;
    /** The name of the stat. */
    string name;
    /** Names and descriptions of subfields. */
    vector<SubData> *subdata;
    /** The description of the stat. */
    string desc;
    /** The display precision. */
    int precision;
    /** The formatting flags. */
    FormatFlags flags;
    /** A pointer to a prerequisite Stat. */
    const Stat *prereq;
};

StatData::StatData()
    : init(false), print(false), subdata(NULL), precision(-1), flags(none),
      prereq(NULL)
{
}

StatData::~StatData()
{
    if (subdata)
        delete subdata;
}

class Database
{
  private:
    Database(const Database &) {}

  private:
    typedef list<Stat *> list_t;
    typedef map<const Stat *, StatData *> map_t;

    list<BinBase *> bins;
    map<const BinBase *, std::string > bin_names;
    list_t binnedStats;

    list_t allStats;
    list_t printStats;
    map_t map;

  public:
    Database();
    ~Database();

    void dump(ostream &stream);

    StatData *find(const Stat *stat);
    void check();
    void reset();
    void regStat(Stat *stat);
    StatData *print(Stat *stat);
    void regBin(BinBase *bin, std::string name);
};

Database::Database()
{}

Database::~Database()
{}

void
Database::dump(ostream &stream)
{

    list_t::iterator i = printStats.begin();
    list_t::iterator end = printStats.end();
    while (i != end) {
        Stat *stat = *i;
        if (stat->binned())
            binnedStats.push_back(stat);
        ++i;
    }

    list<BinBase *>::iterator j = bins.begin();
    list<BinBase *>::iterator bins_end=bins.end();

    if (!bins.empty()) {
        ccprintf(stream, "PRINTING BINNED STATS\n");
        while (j != bins_end) {
            (*j)->activate();
           ::map<const BinBase  *, std::string>::const_iterator iter;
            iter = bin_names.find(*j);
            if (iter == bin_names.end())
                panic("a binned stat not found in names map!");
            ccprintf(stream,"---%s Bin------------\n", (*iter).second);

           list_t::iterator i = binnedStats.begin();
           list_t::iterator end = binnedStats.end();
           while (i != end) {
               Stat *stat = *i;
               if (stat->dodisplay())
                   stat->display(stream);
               ++i;
           }
           ++j;
           ccprintf(stream, "---------------------------------\n");
        }
        ccprintf(stream, "**************ALL STATS************\n");
    }

    list_t::iterator k = printStats.begin();
    list_t::iterator endprint = printStats.end();
    while (k != endprint) {
        Stat *stat = *k;
        if (stat->dodisplay() && !stat->binned())
            stat->display(stream);
        ++k;
    }
}

StatData *
Database::find(const Stat *stat)
{
    map_t::const_iterator i = map.find(stat);

    if (i == map.end())
        return NULL;

    return (*i).second;
}

void
Database::check()
{
    list_t::iterator i = allStats.begin();
    list_t::iterator end = allStats.end();

    while (i != end) {
        Stat *stat = *i;
        StatData *data = find(stat);
        if (!data || !data->init) {
#ifdef STAT_DEBUG
            cprintf("this is stat number %d\n",(*i)->number);
#endif
            panic("Not all stats have been initialized");
        }

        if (data->print) {
            if (data->name.empty())
                panic("all printable stats must be named");

            list_t::iterator j = printStats.insert(printStats.end(), *i);
            inplace_merge(printStats.begin(), j,
                          printStats.end(), Stat::less);
        }

        ++i;
    }
}

void
Database::reset()
{
    list_t::iterator i = allStats.begin();
    list_t::iterator end = allStats.end();

    while (i != end) {
        (*i)->reset();
        ++i;
    }
}

void
Database::regStat(Stat *stat)
{
    if (map.find(stat) != map.end())
        panic("shouldn't register stat twice!");

    allStats.push_back(stat);

    StatData *data = new StatData;
    bool success = (map.insert(make_pair(stat, data))).second;
    assert(map.find(stat) != map.end());
    assert(success && "this should never fail");
}

void
Database::regBin(BinBase *bin, std::string name)
{
    if (bin_names.find(bin) != bin_names.end())
        panic("shouldn't register bin twice");

    bins.push_back(bin);

    bool success = (bin_names.insert(make_pair(bin,name))).second;
    assert(bin_names.find(bin) != bin_names.end());
    assert(success && "this should not fail");

    cprintf("registering %s\n", name);
}

bool
Stat::less(Stat *stat1, Stat *stat2)
{
    const string &name1 = stat1->myname();
    const string &name2 = stat2->myname();

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

StatData *
Database::print(Stat *stat)
{
    StatData *data = find(stat);
    assert(data);

    data->print = true;

    return data;
}

Database &
StatDB()
{
    static Database db;
    return db;
}

Stat::Stat(bool reg)
{
#if 0
    // This assert can help you find that pesky stat.
    assert(this != (void *)0xbffff5c0);
#endif

    if (reg)
        StatDB().regStat(this);

#ifdef STAT_DEBUG
    number = ++total_stats;
    cprintf("I'm stat number %d\n",number);
#endif
}

void
Stat::setInit()
{ mydata()->init = true; }

StatData *
Stat::mydata()
{
    StatData *data = StatDB().find(this);
    assert(data);

    return data;
}

const StatData *
Stat::mydata() const
{
    StatData *data = StatDB().find(this);
    assert(data);

    return data;
}

const SubData *
Stat::mysubdata(int index) const
{
    assert(index >= 0);
    if (index >= size())
        return NULL;

    const StatData *data = this->mydata();
    if (!data->subdata || data->subdata->size() <= index)
        return NULL;

    return &(*data->subdata)[index];
}

SubData *
Stat::mysubdata_create(int index)
{
    int size = this->size();
    assert(index >= 0 && (size == 0 || size > 0 && index < size));

    StatData *data = this->mydata();
    if (!data->subdata) {
        if (!data->subdata) {
            if (size == 0)
                size = index + 1;

            data->subdata = new vector<SubData>(size);
        }
    } else if (data->subdata->size() <= index)
            data->subdata->resize(index + 1);

    SubData *sd = &(*data->subdata)[index];
    assert(sd);

    return sd;
}

string
Stat::myname() const
{ return mydata()->name; }

string
Stat::mysubname(int index) const
{
    const SubData *sd = mysubdata(index);
    return sd ? sd->name : "";
}

string
Stat::mydesc() const
{ return mydata()->desc; }

string
Stat::mysubdesc(int index) const
{
    const SubData *sd = mysubdata(index);
    return sd ? sd->desc : "";
}

int
Stat::myprecision() const
{ return mydata()->precision; }

FormatFlags
Stat::myflags() const
{ return mydata()->flags; }

bool
Stat::dodisplay() const
{ return !mydata()->prereq || !mydata()->prereq->zero(); }

StatData *
Stat::print()
{
    StatData *data = StatDB().print(this);
    assert(data && data->init);

    return data;
}

Stat &
Stat::name(const string &name)
{
    print()->name = name;
    return *this;
}

Stat &
Stat::desc(const string &desc)
{
    print()->desc = desc;
    return *this;
}

Stat &
Stat::precision(int precision)
{
    print()->precision = precision;
    return *this;
}

Stat &
Stat::flags(FormatFlags flags)
{
    if (flags & __reserved)
        panic("Cannot set reserved flags!\n");

    print()->flags |= flags;
    return *this;
}

Stat &
Stat::prereq(const Stat &prereq)
{
    print()->prereq = &prereq;
    return *this;
}

Stat &
Stat::subname(int index, const string &name)
{
    print();
    mysubdata_create(index)->name = name;
    return *this;
}
Stat &
Stat::subdesc(int index, const string &desc)
{
    print();
    mysubdata_create(index)->desc = desc;
    return *this;
}

bool
ScalarStat::zero() const
{
    return val() == 0.0;
}

bool
VectorStat::zero() const
{
    return val()[0] == 0.0;
}

string
ValueToString(result_t value, int precision)
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
#ifndef STAT_DISPLAY_COMPAT
        val << "no value";
#else
        val << "<err: div-0>";
#endif
    }

    return val.str();
}

void
PrintOne(ostream &stream, result_t value,
         const string &name, const string &desc, int precision,
         FormatFlags flags, result_t pdf = NAN, result_t cdf = NAN)
{
    if (flags & nozero && value == 0.0 ||
        flags & nonan && isnan(value))
        return;

    stringstream pdfstr, cdfstr;

    if (!isnan(pdf))
        ccprintf(pdfstr, "%.2f%%", pdf * 100.0);

    if (!isnan(cdf))
        ccprintf(cdfstr, "%.2f%%", cdf * 100.0);

#ifdef STAT_DISPLAY_COMPAT
    if (flags & __substat) {
        ccprintf(stream, "%32s%12s%10s%10s", name,
                 ValueToString(value, precision),
                 pdfstr, cdfstr);
    } else
#endif
    {
        ccprintf(stream, "%-40s%12s%10s%10s", name,
                 ValueToString(value, precision), pdfstr, cdfstr);
    }

    if (PrintDescriptions) {
        if (!desc.empty())
            ccprintf(stream, " # %s", desc);
    }
    stream << endl;
}

void
ScalarStat::display(ostream &stream) const
{
    PrintOne(stream, val(), myname(), mydesc(), myprecision(), myflags());
}

void
VectorStat::display(ostream &stream) const
{
    bool have_subname = false;
    bool have_subdesc = false;
    int size = this->size();
    for (int i = 0; i < size; ++i) {
        if (!mysubname(i).empty())
            have_subname = true;
        if (!mysubdesc(i).empty())
            have_subdesc = true;
    }

    vector<string> *subnames = 0;
    vector<string> *subdescs = 0;
    if (have_subname) {
        subnames = new vector<string>(size);
        for (int i = 0; i < size; ++i)
            (*subnames)[i] = mysubname(i);
    }
    if (have_subdesc) {
        subdescs = new vector<string>(size);
        for (int i = 0; i < size; ++i)
            (*subdescs)[i] = mysubdesc(i);
    }

    VectorDisplay(stream, myname(), subnames, mydesc(), subdescs,
                  myprecision(), myflags(), val(), total());
}

#ifndef STAT_DISPLAY_COMPAT
#define NAMESEP "::"
#else
#define NAMESEP "_"
#endif

#ifndef STAT_DISPLAY_COMPAT
void
VectorDisplay(std::ostream &stream,
              const std::string &myname,
              const std::vector<std::string> *mysubnames,
              const std::string &mydesc,
              const std::vector<std::string> *mysubdescs,
              int myprecision, FormatFlags myflags,
              const rvec_t &vec, result_t mytotal)
{
    int _size = vec.size();
    result_t _total = 0.0;
    result_t _pdf, _cdf = 0.0;

    if (myflags & (pdf | cdf)) {
        for (int i = 0; i < _size; ++i) {
            _total += vec[i];
        }
    }

    if (_size == 1) {
        PrintOne(stream, vec[0], myname, mydesc, myprecision, myflags);
    } else {
        for (int i = 0; i < _size; ++i) {
            string subname;
            if (mysubnames) {
                subname = (*mysubnames)[i];
                if (subname.empty())
                    continue;
            } else {
                subname = to_string(i);
            }

            string name = myname + NAMESEP + subname;
            if (!(myflags & pdf))
                PrintOne(stream, vec[i], name, mydesc, myprecision, myflags);
            else {
                _pdf = vec[i] / _total;
                _cdf += _pdf;
                PrintOne(stream, vec[i], name, mydesc, myprecision, myflags,
                         _pdf, _cdf);
            }
        }

        if (myflags & total)
            PrintOne(stream, mytotal, myname + NAMESEP + "total",
                     mydesc, myprecision, myflags);
    }
}
#else
void
VectorDisplay(std::ostream &stream,
              const std::string &myname,
              const std::vector<std::string> *mysubnames,
              const std::string &mydesc,
              const std::vector<std::string> *mysubdescs,
              int myprecision, FormatFlags myflags,
              const rvec_t &vec, result_t mytotal)
{
    int _size = vec.size();
    result_t _total = 0.0;
    result_t _pdf, _cdf = 0.0;

    if (myflags & (pdf | cdf)) {
        for (int i = 0; i < _size; ++i) {
            _total += vec[i];
        }
    }

    if (_size == 1) {
        PrintOne(stream, vec[0], myname, mydesc, myprecision, myflags);
    } else {
        if (myflags & total)
            PrintOne(stream, mytotal, myname, mydesc, myprecision, myflags);

        if (myflags & dist) {
            ccprintf(stream, "%s.start_dist\n", myname);
            for (int i = 0; i < _size; ++i) {
                string subname, subdesc;
                subname = to_string(i);
                if (mysubnames) {
                    if (!subname.empty()) {
                        subname = (*mysubnames)[i];
                    }
                }
                if (mysubdescs) {
                    subdesc = (*mysubdescs)[i];
                }
                if (!(myflags & (pdf | cdf))) {
                    PrintOne(stream, vec[i], subname, subdesc, myprecision,
                             myflags | __substat);
                } else {
                    if (_total) {
                        _pdf = vec[i] / _total;
                        _cdf += _pdf;
                    } else {
                        _pdf = _cdf = 0.0;
                    }
                    if (!(myflags & cdf)) {
                        PrintOne(stream, vec[i], subname, subdesc, myprecision,
                                 myflags | __substat, _pdf);
                    } else {
                        PrintOne(stream, vec[i], subname, subdesc, myprecision,
                                 myflags | __substat, _pdf, _cdf);
                    }
                }
            }
            ccprintf(stream, "%s.end_dist\n", myname);
        } else {
            for (int i = 0; i < _size; ++i) {
                string subname;
                if (mysubnames) {
                    subname = (*mysubnames)[i];
                    if (subname.empty())
                        continue;
                } else {
                    subname = to_string(i);
                }

                string name = myname + NAMESEP + subname;
                if (!(myflags & pdf)) {
                    PrintOne(stream, vec[i], name, mydesc, myprecision,
                             myflags);
                } else {
                    if (_total) {
                        _pdf = vec[i] / _total;
                        _cdf += _pdf;
                    } else {
                        _pdf = _cdf = 0.0;
                    }
                    _pdf = vec[i] / _total;
                    _cdf += _pdf;
                    PrintOne(stream, vec[i], name, mydesc, myprecision,
                             myflags, _pdf, _cdf);
                }
            }
        }
    }
}
#endif

#ifndef STAT_DISPLAY_COMPAT
void
DistDisplay(ostream &stream, const string &name, const string &desc,
            int precision, FormatFlags flags,
            result_t min_val, result_t max_val,
            result_t underflow, result_t overflow,
            const rvec_t &vec, int min, int max, int bucket_size, int size);
{
    assert(size == vec.size());

    result_t total = 0.0;
    result_t pdf, cdf = 0.0;

    total += underflow;
    for (int i = 0; i < size; ++i)
        total += vec[i];
    total += overflow;

    pdf = underflow / total;
    cdf += pdf;

    PrintOne(stream, underflow, name + NAMESEP + "underflow", desc,
             precision, myflags, pdf, cdf);

    for (int i = 0; i < size; ++i) {
        stringstream namestr;
        namestr << name;

        int low = i * bucket_size + min;
        int high = ::std::min((i + 1) * bucket_size + min - 1, max);
        namestr << low;
        if (low < high)
            namestr << "-" << high;

        pdf = vec[i] / total;
        cdf += pdf;
        PrintOne(stream, vec[i], namestr.str(), desc, precision, myflags,
                 pdf, cdf);
    }

    pdf = overflow / total;
    cdf += pdf;
    PrintOne(stream, overflow, name + NAMESEP + "overflow", desc,
             precision, myflags, pdf, cdf);
    PrintOne(stream, total, name + NAMESEP + "total", desc,
             precision, myflags);
}
#else
void
DistDisplay(ostream &stream, const string &name, const string &desc,
            int precision, FormatFlags flags,
            result_t min_val, result_t max_val,
            result_t underflow, result_t overflow,
            const rvec_t &vec, int min, int max, int bucket_size, int size)
{
    assert(size == vec.size());
    string blank;

    result_t total = 0.0;

    total += underflow;
    for (int i = 0; i < size; ++i)
        total += vec[i];
    total += overflow;

    ccprintf(stream, "%-42s", name + ".start_dist");
    if (PrintDescriptions && !desc.empty())
        ccprintf(stream, "                     # %s", desc);
    stream << endl;

    PrintOne(stream, total, name + ".samples", blank, precision, flags);
    PrintOne(stream, min_val, name + ".min_value", blank, precision, flags);

    if (underflow > 0)
        PrintOne(stream, min_val, name + ".underflows", blank, precision,
                 flags);

    int _min;
    result_t _pdf, _cdf, mypdf, mycdf;

    _cdf = 0.0;
    for (int i = 0; i < size; ++i) {
        if (flags & nozero && vec[i] == 0.0 ||
            flags & nonan && isnan(vec[i]))
            return;

        _min = i * bucket_size + min;
        _pdf = vec[i] / total * 100.0;
        _cdf += _pdf;

        mypdf = (flags & pdf) ? _pdf : NAN;
        mycdf = (flags & cdf) ? _cdf : NAN;

        PrintOne(stream, vec[i], ValueToString(_min, 0), blank, precision,
                 flags | __substat, mypdf, mycdf);
    }

    if (overflow > 0)
        PrintOne(stream, overflow, name + ".overflows", blank, precision,
                 flags);
    PrintOne(stream, max_val, name + ".max_value", blank, precision, flags);
    ccprintf(stream, "%s.end_dist\n\n", name);
}
#endif

void
FancyDisplay(ostream &stream, const string &name, const string &desc,
             int precision, FormatFlags flags, result_t mean,
             result_t variance)
{
    result_t stdev = isnan(variance) ? NAN : sqrt(variance);
    PrintOne(stream, mean, name + NAMESEP + "mean", desc, precision, flags);
    PrintOne(stream, stdev, name + NAMESEP + "stdev", desc, precision, flags);
}

BinBase::BinBase(size_t size)
    : memsize(CeilPow2(size)), mem(NULL)
{
}

BinBase::~BinBase()
{
    if (mem)
        delete [] mem;
}

char *
BinBase::memory()
{
    if (!mem) {
        mem = new char[memsize];
        memset(mem, 0, memsize);
    }

    return mem;
}

void
BinBase::regBin(BinBase *bin, std::string name)
{
    StatDB().regBin(bin, name);
}

} // namespace Detail

void
check()
{
    Detail::StatDB().check();
}

void
dump(ostream &stream)
{
    Detail::StatDB().dump(stream);
}

CallbackQueue resetQueue;

void
regReset(Callback *cb)
{
    resetQueue.add(cb);
}

void
reset()
{
    Detail::StatDB().reset();
    resetQueue.process();
}

} // namespace Statistics

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
#include <fstream>
#include <list>
#include <map>
#include <string>
#include <sstream>

#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/hostinfo.hh"
#include "base/misc.hh"
#include "base/python.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "base/time.hh"
#include "base/trace.hh"

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
DisplayMode DefaultMode = mode_simplescalar;

namespace Database
{
    class Data
    {
      private:
        typedef list<StatData *> list_t;
        typedef map<void *, StatData *> map_t;

        list<MainBin *> bins;

        list_t allStats;
        list_t printStats;
        map_t statMap;

        ofstream *stream;
        Python *py;

      public:
        Data();
        ~Data();

        void dump(ostream &stream, DisplayMode mode);
        void display(ostream &stream, DisplayMode mode);
        void python_start(const string &file);
        void python_dump(const string &name, const string &subname);
        void python(const string &name, const string &subname,
                    const string &bin);

        StatData *find(void *stat);
        void mapStat(void *stat, StatData *data);

        void check();
        void reset();
        void regBin(MainBin *bin, string name);
        void regPrint(void *stat);

        static std::string name() { return "Statistics Database"; }
    };

Data::Data()
    : stream(0), py(0)
{
}

Data::~Data()
{
    if (stream) {
        delete py;
        ccprintf(*stream, "\n\nif __name__ == '__main__':\n");
        ccprintf(*stream, "    program_display()\n");
        stream->close();
        delete stream;
    }
}

void
Data::dump(ostream &stream, DisplayMode mode)
{
    MainBin *orig = MainBin::curBin();

    switch (mode) {
      case mode_m5:
      case mode_simplescalar:
        display(stream, mode);
        break;
      default:
        warn("invalid display mode!\n");
        display(stream, mode_m5);
        break;
    }

    if (orig)
        orig->activate();
}

void
Data::display(ostream &stream, DisplayMode mode)
{
    if (!bins.empty()) {
        list<MainBin *>::iterator i = bins.begin();
        list<MainBin *>::iterator bins_end = bins.end();
        ccprintf(stream, "PRINTING BINNED STATS\n");
        while (i != bins_end) {
            (*i)->activate();
            ccprintf(stream,"---%s Bin------------\n", (*i)->name());

            list_t::iterator j = printStats.begin();
            list_t::iterator end = printStats.end();
            while (j != end) {
                StatData *stat = *j;
                if (stat->dodisplay())
                    stat->display(stream, mode);
                ++j;
            }
            ++i;
            ccprintf(stream, "---------------------------------\n");
        }
    } else {
        list_t::iterator i = printStats.begin();
        list_t::iterator end = printStats.end();
        while (i != end) {
            StatData *stat = *i;
            if (stat->dodisplay() && !stat->binned())
                stat->display(stream, mode);
            ++i;
        }
    }
}

void
Data::python_start(const string &file)
{
    if (stream)
        panic("can't start python twice!");

    stream = new ofstream(file.c_str(), ios::trunc);
    py = new Python(*stream);

    ccprintf(*stream, "import sys\n");
    ccprintf(*stream, "sys.path.append('.')\n");
    ccprintf(*stream, "from m5stats import *\n\n");
}

void
Data::python_dump(const string &name, const string &subname)
{
    if (!py)
        panic("Can't dump python without first opening the file");

    if (bins.empty()) {
        python(name, subname, "");
    } else {
        list<MainBin *>::iterator i = bins.begin();
        list<MainBin *>::iterator end = bins.end();

        while (i != end) {
            (*i)->activate();
            python(name, subname, (*i)->name());
            ++i;
        }
    }
//    py->next();
}

void
Data::python(const string &name, const string &subname, const string &bin)
{
    py->name("collections.append");
    py->newline();
    py->name("Collection");
    py->newline();
    py->qarg(name);
    py->newline();
    py->qarg(subname);
    py->newline();
    py->qarg(bin);
    py->newline();
    py->qarg(hostname());
    py->newline();
    py->qarg(Time::start.date());
    py->newline();
    py->list();
    list_t::iterator i = allStats.begin();
    list_t::iterator end = allStats.end();
    while (i != end) {
        StatData *stat = *i;
        py->newline();
        stat->python(*py);
        ++i;
    }
    py->newline();
    py->listEnd();
    py->newline();
    py->nameEnd();
    py->newline();
    py->nameEnd();
    py->newline();
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
        StatData *data = *i;
        assert(data);
        data->check();
        ++i;
    }

    i = allStats.begin();
    int j = 0;
    while (i != end) {
        StatData *data = *i;
        if (!(data->flags & print))
            data->name = "__Stat" + to_string(j++);
        ++i;
    }
}

void
Data::reset()
{
    // reset non-binned stats
    list_t::iterator i = allStats.begin();
    list_t::iterator end = allStats.end();
    while (i != end) {
        StatData *data = *i;
        if (!data->binned())
            data->reset();
        ++i;
    }

    // save the bin so we can go back to where we were
    MainBin *orig = MainBin::curBin();

    // reset binned stats
    list<MainBin *>::iterator bi = bins.begin();
    list<MainBin *>::iterator be = bins.end();
    while (bi != be) {
        MainBin *bin = *bi;
        bin->activate();

        i = allStats.begin();
        while (i != end) {
            StatData *data = *i;
            if (data->binned())
                data->reset();
            ++i;
        }
        ++bi;
    }

    // restore bin
    MainBin::curBin() = orig;
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
Data::regBin(MainBin *bin, string _name)
{
    bins.push_back(bin);
    DPRINTF(Stats, "registering %s\n", _name);
}

void
Data::regPrint(void *stat)
{
    StatData *data = find(stat);

    if (data->flags & print)
        return;

    data->flags |= print;

    list_t::iterator j = printStats.insert(printStats.end(), data);
    inplace_merge(printStats.begin(), j, printStats.end(), StatData::less);
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

const StatData *
getStatData(const void *stat)
{
    return Database::StatDB().find(const_cast<void *>(stat));
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
    statData()->flags |= init;
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
StatData::baseCheck() const
{
    if (!(flags & init)) {
#ifdef STAT_DEBUG
        cprintf("this is stat number %d\n",(*i)->number);
#endif
        panic("Not all stats have been initialized");
        return false;
    }

    if ((flags & print) && name.empty()) {
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
    StatFlags flags;
    DisplayMode mode;
    int precision;
    result_t pdf;
    result_t cdf;

    ScalarPrint()
        : value(0.0), flags(0), mode(DefaultMode), precision(0),
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
                 ValueToString(value, mode, precision), pdfstr, cdfstr);
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
    StatFlags flags;
    DisplayMode mode;
    int precision;
    rvec_t vec;
    result_t total;

    VectorPrint()
        : subnames(0), subdescs(0), flags(0), mode(DefaultMode),
          precision(-1), total(NAN)
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
    StatFlags flags;
    DisplayMode mode;
    int precision;

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
        print.mode = mode;
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
    print.flags = flags;
    print.mode = mode;
    print.precision = precision;

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
    }

    if (mode == mode_m5 || overflow > 0.0) {
        print.name = base + "overflows";
        print.value = overflow;
        if (mode == mode_m5 && total) {
            print.pdf = overflow / total;
            print.cdf += print.pdf;
        } else {
            print.pdf = NAN;
            print.cdf = NAN;
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
ScalarDataBase::display(ostream &stream, DisplayMode mode) const
{
    ScalarPrint print;
    print.value = val();
    print.name = name;
    print.desc = desc;
    print.flags = flags;
    print.mode = mode;
    print.precision = precision;

    print(stream);
}

void
VectorDataBase::display(ostream &stream, DisplayMode mode) const
{
    int size = this->size();
    const_cast<VectorDataBase *>(this)->update();

    VectorPrint print;

    print.name = name;
    print.desc = desc;
    print.flags = flags;
    print.mode = mode;
    print.precision = precision;
    print.vec = val();
    print.total = total();

    if (!subnames.empty()) {
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
    }

    print(stream);
}

void
Vector2dDataBase::display(ostream &stream, DisplayMode mode) const
{
    const_cast<Vector2dDataBase *>(this)->update();

    bool havesub = false;
    VectorPrint print;

    print.subnames = y_subnames;
    print.flags = flags;
    print.mode = mode;
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
DistDataBase::display(ostream &stream, DisplayMode mode) const
{
    const_cast<DistDataBase *>(this)->update();

    DistPrint print;

    print.name = name;
    print.desc = desc;
    print.flags = flags;
    print.mode = mode;
    print.precision = precision;

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
VectorDistDataBase::display(ostream &stream, DisplayMode mode) const
{
    const_cast<VectorDistDataBase *>(this)->update();

    for (int i = 0; i < size(); ++i) {
        DistPrint print;

        print.name = name +
            (subnames[i].empty() ? ("_" + to_string(i)) : subnames[i]);
        print.desc = subdescs[i].empty() ? desc : subdescs[i];
        print.flags = flags;
        print.mode = mode;
        print.precision = precision;

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
ScalarDataBase::python(Python &py) const
{
    py.name("Scalar");
    py.qarg(name);
    py.qqqarg(desc);
    py.kwarg("binned", binned());
    py.kwarg("precision", precision);
    py.kwarg("flags", flags);
    if (prereq)
        py.qkwarg("prereq", prereq->name);
    py.kwarg("value", val());
    py.nameEnd();
}

void
VectorDataBase::python(Python &py) const
{
    const_cast<VectorDataBase *>(this)->update();

    py.name("Vector");
    py.qarg(name);
    py.qqqarg(desc);
    py.kwarg("binned", binned());
    py.kwarg("precision", precision);
    py.kwarg("flags", flags);
    if (prereq)
        py.qkwarg("prereq", prereq->name);
    py.kwarg("value", val());
    if (!subnames.empty())
        py.qkwarg("subnames", subnames);
    if (!subdescs.empty())
        py.qkwarg("subdescs", subdescs);
    py.nameEnd();
}

void
DistDataData::python(Python &py, const string &name) const
{
    string s = name.empty() ? "" : name + "=";

    if (samples == 0 || fancy)
        s += "SimpleDist";
    else
        s += "FullDist";

    py.name(s);
    py.arg(sum);
    py.arg(squares);
    py.arg(samples);
    if (samples && !fancy) {
        py.arg(min_val);
        py.arg(min_val);
        py.arg(underflow);
        py.arg(vec);
        py.arg(overflow);
        py.arg(min);
        py.arg(max);
        py.arg(bucket_size);
        py.arg(size);
    }
    py.nameEnd();
}

void
FormulaDataBase::python(Python &py) const
{
    const_cast<FormulaDataBase *>(this)->update();

    py.name("Formula");
    py.qarg(name);
    py.qqqarg(desc);
    py.kwarg("binned", binned());
    py.kwarg("precision", precision);
    py.kwarg("flags", flags);
    if (prereq)
        py.qkwarg("prereq", prereq->name);
    py.qkwarg("formula", str());
    if (!subnames.empty())
        py.qkwarg("subnames", subnames);
    if (!subdescs.empty())
        py.qkwarg("subdescs", subdescs);
    py.nameEnd();
}

void
DistDataBase::python(Python &py) const
{
    const_cast<DistDataBase *>(this)->update();

    py.name("Dist");
    py.qarg(name);
    py.qqqarg(desc);
    py.kwarg("binned", binned());
    py.kwarg("precision", precision);
    py.kwarg("flags", flags);
    if (prereq)
        py.qkwarg("prereq", prereq->name);
    data.python(py, "dist");
    py.nameEnd();
}

void
VectorDistDataBase::python(Python &py) const
{
    const_cast<VectorDistDataBase *>(this)->update();

    py.name("VectorDist");
    py.qarg(name);
    py.qqqarg(desc);
    py.kwarg("binned", binned());
    py.kwarg("precision", precision);
    py.kwarg("flags", flags);
    if (prereq)
        py.qkwarg("prereq", prereq->name);
    if (!subnames.empty())
        py.qkwarg("subnames", subnames);
    if (!subdescs.empty())
        py.qkwarg("subdescs", subdescs);

    py.tuple("dist");
    typedef std::vector<DistDataData>::const_iterator iter;
    iter i = data.begin();
    iter end = data.end();
    while (i != end) {
        i->python(py, "");
        ++i;
    }
    py.tupleEnd();
    py.nameEnd();
}

void
Vector2dDataBase::python(Python &py) const
{
    const_cast<Vector2dDataBase *>(this)->update();

    py.name("Vector2d");
    py.qarg(name);
    py.qqqarg(desc);
    py.kwarg("binned", binned());
    py.kwarg("precision", precision);
    py.kwarg("flags", flags);
    if (prereq)
        py.qkwarg("prereq", prereq->name);

    py.kwarg("value", vec);
    if (!subnames.empty())
        py.qkwarg("subnames", subnames);
    if (!subdescs.empty())
        py.qkwarg("subdescs", subdescs);
    if (!y_subnames.empty())
        py.qkwarg("ysubnames", y_subnames);

    py.kwarg("x", x);
    py.kwarg("y", y);
    py.nameEnd();
}

void
FormulaBase::val(rvec_t &vec) const
{
    if (root)
        vec = root->val();
}

result_t
FormulaBase::total() const
{
    return root ? root->total() : 0.0;
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
    return root && root->binned();
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

string
FormulaBase::str() const
{
    return root ? root->str() : "";
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
dump(ostream &stream, DisplayMode mode)
{
    Database::StatDB().dump(stream, mode);
}

void
python_start(const string &file)
{
    Database::StatDB().python_start(file);
}

void
python_dump(const string &name, const string &subname)
{
    Database::StatDB().python_dump(name, subname);
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

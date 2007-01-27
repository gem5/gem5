/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#if defined(__APPLE__)
#define _GLIBCPP_USE_C99 1
#endif

#if defined(__sun)
#include <math.h>
#endif

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "base/misc.hh"
#include "base/statistics.hh"
#include "base/stats/statdb.hh"
#include "base/stats/text.hh"
#include "base/stats/visit.hh"

using namespace std;

#ifndef NAN
float __nan();
/** Define Not a number. */
#define NAN (__nan())
/** Need to define __nan() */
#define __M5_NAN
#endif

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

namespace Stats {

Text::Text()
    : mystream(false), stream(NULL), compat(false), descriptions(false)
{
}

Text::Text(std::ostream &stream)
    : mystream(false), stream(NULL), compat(false), descriptions(false)
{
    open(stream);
}

Text::Text(const std::string &file)
    : mystream(false), stream(NULL), compat(false), descriptions(false)
{
    open(file);
}


Text::~Text()
{
    if (mystream) {
        assert(stream);
        delete stream;
    }
}

void
Text::open(std::ostream &_stream)
{
    if (stream)
        panic("stream already set!");

    mystream = false;
    stream = &_stream;
    assert(valid());
}

void
Text::open(const std::string &file)
{
    if (stream)
        panic("stream already set!");

    mystream = true;
    stream = new ofstream(file.c_str(), ios::trunc);
    assert(valid());
}

bool
Text::valid() const
{
    return stream != NULL;
}

void
Text::output()
{
    using namespace Database;

    ccprintf(*stream, "\n---------- Begin Simulation Statistics ----------\n");
    stat_list_t::const_iterator i, end = stats().end();
    for (i = stats().begin(); i != end; ++i)
        (*i)->visit(*this);
    ccprintf(*stream, "\n---------- End Simulation Statistics   ----------\n");
    stream->flush();
}

bool
Text::noOutput(const StatData &data)
{
    if (!(data.flags & print))
        return true;

    if (data.prereq && data.prereq->zero())
        return true;

    return false;
}

string
ValueToString(Result value, int precision, bool compat)
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
        val << (compat ? "<err: div-0>" : "no value");
    }

    return val.str();
}

struct ScalarPrint
{
    Result value;
    string name;
    string desc;
    StatFlags flags;
    bool compat;
    bool descriptions;
    int precision;
    Result pdf;
    Result cdf;

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

    if (compat && flags & __substat) {
        ccprintf(stream, "%32s %12s %10s %10s", name,
                 ValueToString(value, precision, compat), pdfstr, cdfstr);
    } else {
        ccprintf(stream, "%-40s %12s %10s %10s", name,
                 ValueToString(value, precision, compat), pdfstr, cdfstr);
    }

    if (descriptions) {
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
    bool compat;
    bool descriptions;
    int precision;
    VResult vec;
    Result total;

    void operator()(ostream &stream) const;
};

void
VectorPrint::operator()(std::ostream &stream) const
{
    int _size = vec.size();
    Result _total = 0.0;

    if (flags & (pdf | cdf)) {
        for (int i = 0; i < _size; ++i) {
            _total += vec[i];
        }
    }

    string base = name + (compat ? "_" : "::");

    ScalarPrint print;
    print.name = name;
    print.desc = desc;
    print.precision = precision;
    print.descriptions = descriptions;
    print.flags = flags;
    print.pdf = NAN;
    print.cdf = NAN;

    bool havesub = !subnames.empty();

    if (_size == 1) {
        print.value = vec[0];
        print(stream);
    } else if (!compat) {
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

        if (flags & ::Stats::total) {
            print.name = base + "total";
            print.desc = desc;
            print.value = total;
            print(stream);
        }
    } else {
        if (flags & ::Stats::total) {
            print.value = total;
            print(stream);
        }

        Result _pdf = 0.0;
        Result _cdf = 0.0;
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
    bool compat;
    bool descriptions;
    int precision;

    Result min_val;
    Result max_val;
    Result underflow;
    Result overflow;
    VResult vec;
    Result sum;
    Result squares;
    Result samples;

    Counter min;
    Counter max;
    Counter bucket_size;
    int size;
    bool fancy;

    void operator()(ostream &stream) const;
};

void
DistPrint::operator()(ostream &stream) const
{
    if (fancy) {
        ScalarPrint print;
        string base = name + (compat ? "_" : "::");

        print.precision = precision;
        print.flags = flags;
        print.compat = compat;
        print.descriptions = descriptions;
        print.desc = desc;
        print.pdf = NAN;
        print.cdf = NAN;

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

    Result total = 0.0;

    total += underflow;
    for (int i = 0; i < size; ++i)
        total += vec[i];
    total += overflow;

    string base = name + (compat ? "." : "::");

    ScalarPrint print;
    print.desc = compat ? "" : desc;
    print.flags = flags;
    print.compat = compat;
    print.descriptions = descriptions;
    print.precision = precision;
    print.pdf = NAN;
    print.cdf = NAN;

    if (compat) {
        ccprintf(stream, "%-42s", base + "start_dist");
        if (descriptions && !desc.empty())
            ccprintf(stream, "                     # %s", desc);
        stream << endl;
    }

    print.name = base + "samples";
    print.value = samples;
    print(stream);

    print.name = base + "min_value";
    print.value = min_val;
    print(stream);

    if (!compat || underflow > 0.0) {
        print.name = base + "underflows";
        print.value = underflow;
        if (!compat && total) {
            print.pdf = underflow / total;
            print.cdf += print.pdf;
        }
        print(stream);
    }


    if (!compat) {
        for (int i = 0; i < size; ++i) {
            stringstream namestr;
            namestr << name;

            Counter low = i * bucket_size + min;
            Counter high = ::min(low + bucket_size, max);
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
        Counter _min;
        Result _pdf;
        Result _cdf = 0.0;

        print.flags = flags | __substat;

        for (int i = 0; i < size; ++i) {
            if (flags & nozero && vec[i] == 0.0 ||
                flags & nonan && isnan(vec[i]))
                continue;

            _min = i * bucket_size + min;
            _pdf = vec[i] / total * 100.0;
            _cdf += _pdf;


            print.name = ValueToString(_min, 0, compat);
            print.value = vec[i];
            print.pdf = (flags & pdf) ? _pdf : NAN;
            print.cdf = (flags & cdf) ? _cdf : NAN;
            print(stream);
        }

        print.flags = flags;
    }

    if (!compat || overflow > 0.0) {
        print.name = base + "overflows";
        print.value = overflow;
        if (!compat && total) {
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

    if (!compat) {
        print.name = base + "total";
        print.value = total;
        print(stream);
    }

    print.name = base + "max_value";
    print.value = max_val;
    print(stream);

    if (!compat && samples != 0) {
        print.name = base + "mean";
        print.value = sum / samples;
        print(stream);

        print.name = base + "stdev";
        print.value = sqrt((samples * squares - sum * sum) /
                           (samples * (samples - 1.0)));
        print(stream);
    }

    if (compat)
        ccprintf(stream, "%send_dist\n\n", base);
}

void
Text::visit(const ScalarData &data)
{
    if (noOutput(data))
        return;

    ScalarPrint print;
    print.value = data.result();
    print.name = data.name;
    print.desc = data.desc;
    print.flags = data.flags;
    print.compat = compat;
    print.descriptions = descriptions;
    print.precision = data.precision;
    print.pdf = NAN;
    print.cdf = NAN;

    print(*stream);
}

void
Text::visit(const VectorData &data)
{
    if (noOutput(data))
        return;

    int size = data.size();
    VectorPrint print;

    print.name = data.name;
    print.desc = data.desc;
    print.flags = data.flags;
    print.compat = compat;
    print.descriptions = descriptions;
    print.precision = data.precision;
    print.vec = data.result();
    print.total = data.total();

    if (!data.subnames.empty()) {
        for (int i = 0; i < size; ++i) {
            if (!data.subnames[i].empty()) {
                print.subnames = data.subnames;
                print.subnames.resize(size);
                for (int i = 0; i < size; ++i) {
                    if (!data.subnames[i].empty() &&
                        !data.subdescs[i].empty()) {
                        print.subdescs = data.subdescs;
                        print.subdescs.resize(size);
                        break;
                    }
                }
                break;
            }
        }
    }

    print(*stream);
}

void
Text::visit(const Vector2dData &data)
{
    if (noOutput(data))
        return;

    bool havesub = false;
    VectorPrint print;

    print.subnames = data.y_subnames;
    print.flags = data.flags;
    print.compat = compat;
    print.descriptions = descriptions;
    print.precision = data.precision;

    if (!data.subnames.empty()) {
        for (int i = 0; i < data.x; ++i)
            if (!data.subnames[i].empty())
                havesub = true;
    }

    VResult tot_vec(data.y);
    Result super_total = 0.0;
    for (int i = 0; i < data.x; ++i) {
        if (havesub && (i >= data.subnames.size() || data.subnames[i].empty()))
            continue;

        int iy = i * data.y;
        VResult yvec(data.y);

        Result total = 0.0;
        for (int j = 0; j < data.y; ++j) {
            yvec[j] = data.cvec[iy + j];
            tot_vec[j] += yvec[j];
            total += yvec[j];
            super_total += yvec[j];
        }

        print.name = data.name + "_" + (havesub ? data.subnames[i] : to_string(i));
        print.desc = data.desc;
        print.vec = yvec;
        print.total = total;
        print(*stream);
    }

    if ((data.flags & ::Stats::total) && (data.x > 1)) {
        print.name = data.name;
        print.desc = data.desc;
        print.vec = tot_vec;
        print.total = super_total;
        print(*stream);
    }
}

void
Text::visit(const DistData &data)
{
    if (noOutput(data))
        return;

    DistPrint print;

    print.name = data.name;
    print.desc = data.desc;
    print.flags = data.flags;
    print.compat = compat;
    print.descriptions = descriptions;
    print.precision = data.precision;

    print.min_val = data.data.min_val;
    print.max_val = data.data.max_val;
    print.underflow = data.data.underflow;
    print.overflow = data.data.overflow;
    print.vec.resize(data.data.cvec.size());
    for (int i = 0; i < print.vec.size(); ++i)
        print.vec[i] = (Result)data.data.cvec[i];
    print.sum = data.data.sum;
    print.squares = data.data.squares;
    print.samples = data.data.samples;

    print.min = data.data.min;
    print.max = data.data.max;
    print.bucket_size = data.data.bucket_size;
    print.size = data.data.size;
    print.fancy = data.data.fancy;

    print(*stream);
}

void
Text::visit(const VectorDistData &data)
{
    if (noOutput(data))
        return;

    for (int i = 0; i < data.size(); ++i) {
        DistPrint print;

        print.name = data.name +
            (data.subnames[i].empty() ? ("_" + to_string(i)) : data.subnames[i]);
        print.desc = data.subdescs[i].empty() ? data.desc : data.subdescs[i];
        print.flags = data.flags;
        print.compat = compat;
        print.descriptions = descriptions;
        print.precision = data.precision;

        print.min_val = data.data[i].min_val;
        print.max_val = data.data[i].max_val;
        print.underflow = data.data[i].underflow;
        print.overflow = data.data[i].overflow;
        print.vec.resize(data.data[i].cvec.size());
        for (int j = 0; j < print.vec.size(); ++j)
            print.vec[j] = (Result)data.data[i].cvec[j];
        print.sum = data.data[i].sum;
        print.squares = data.data[i].squares;
        print.samples = data.data[i].samples;

        print.min = data.data[i].min;
        print.max = data.data[i].max;
        print.bucket_size = data.data[i].bucket_size;
        print.size = data.data[i].size;
        print.fancy = data.data[i].fancy;

        print(*stream);
    }
}

void
Text::visit(const FormulaData &data)
{
    visit((const VectorData &)data);
}

/* namespace Stats */ }

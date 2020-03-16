/*
 * Copyright (c) 2019-2020 Arm Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 */

#if defined(__APPLE__)
#define _GLIBCPP_USE_C99 1
#endif

#if defined(__sun)
#include <cmath>

#endif

#include <cassert>

#ifdef __SUNPRO_CC
#include <cmath>

#endif
#include "base/stats/text.hh"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "base/cast.hh"
#include "base/logging.hh"
#include "base/stats/info.hh"
#include "base/str.hh"

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

std::list<Info *> &statsList();

Text::Text()
    : mystream(false), stream(NULL), descriptions(false), spaces(false)
{
}

Text::Text(std::ostream &stream) : Text()
{
    open(stream);
}

Text::Text(const std::string &file) : Text()
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
    if (!valid())
        fatal("Unable to open output stream for writing\n");
}

void
Text::open(const std::string &file)
{
    if (stream)
        panic("stream already set!");

    mystream = true;
    stream = new ofstream(file.c_str(), ios::trunc);
    if (!valid())
        fatal("Unable to open statistics file for writing\n");
}

bool
Text::valid() const
{
    return stream != NULL && stream->good();
}

void
Text::begin()
{
    ccprintf(*stream, "\n---------- Begin Simulation Statistics ----------\n");
}

void
Text::end()
{
    ccprintf(*stream, "\n---------- End Simulation Statistics   ----------\n");
    stream->flush();
}

std::string
Text::statName(const std::string &name) const
{
    if (path.empty())
        return name;
    else
        return csprintf("%s.%s", path.top(), name);
}

void
Text::beginGroup(const char *name)
{
    if (path.empty()) {
        path.push(name);
    } else {
        path.push(csprintf("%s.%s", path.top(), name));
    }
}

void
Text::endGroup()
{
    assert(!path.empty());
    path.pop();
}

bool
Text::noOutput(const Info &info)
{
    if (!info.flags.isSet(display))
        return true;

    if (info.prereq && info.prereq->zero())
        return true;

    return false;
}

string
ValueToString(Result value, int precision)
{
    stringstream val;

    if (!std::isnan(value)) {
        if (precision != -1)
            val.precision(precision);
        else if (value == rint(value))
            val.precision(0);

        val.unsetf(ios::showpoint);
        val.setf(ios::fixed);
        val << value;
    } else {
        val << "nan";
    }

    return val.str();
}

struct ScalarPrint
{
    Result value;
    string name;
    string desc;
    Flags flags;
    bool descriptions;
    bool spaces;
    int precision;
    Result pdf;
    Result cdf;
    int nameSpaces;
    int valueSpaces;
    int pdfstrSpaces;
    int cdfstrSpaces;

    ScalarPrint(bool spaces) : spaces(spaces) {
        if (spaces) {
            nameSpaces = 40;
            valueSpaces = 12;
            pdfstrSpaces = 10;
            cdfstrSpaces = 10;
        } else {
            nameSpaces = 0;
            valueSpaces = 0;
            pdfstrSpaces = 0;
            cdfstrSpaces = 0;
        }
    }
    void update(Result val, Result total);
    void operator()(ostream &stream, bool oneLine = false) const;
};

void
ScalarPrint::update(Result val, Result total)
{
    value = val;
    if (total) {
        pdf = val / total;
        cdf += pdf;
    }
}

void
ScalarPrint::operator()(ostream &stream, bool oneLine) const
{
    if ((flags.isSet(nozero) && (!oneLine) && value == 0.0) ||
        (flags.isSet(nonan) && std::isnan(value)))
        return;

    stringstream pdfstr, cdfstr;

    if (!std::isnan(pdf))
        ccprintf(pdfstr, "%.2f%%", pdf * 100.0);

    if (!std::isnan(cdf))
        ccprintf(cdfstr, "%.2f%%", cdf * 100.0);

    if (oneLine) {
        ccprintf(stream, " |");
    } else {
        ccprintf(stream, "%-*s ", nameSpaces, name);
    }
    ccprintf(stream, "%*s", valueSpaces, ValueToString(value, precision));
    if (spaces || pdfstr.rdbuf()->in_avail())
        ccprintf(stream, " %*s", pdfstrSpaces, pdfstr.str());
    if (spaces || cdfstr.rdbuf()->in_avail())
        ccprintf(stream, " %*s", cdfstrSpaces, cdfstr.str());
    if (!oneLine) {
        if (descriptions) {
            if (!desc.empty())
                ccprintf(stream, " # %s", desc);
        }
        stream << endl;
    }
}

struct VectorPrint
{
    string name;
    string separatorString;
    string desc;
    vector<string> subnames;
    vector<string> subdescs;
    Flags flags;
    bool descriptions;
    bool spaces;
    int precision;
    VResult vec;
    Result total;
    bool forceSubnames;
    int nameSpaces;

    VectorPrint() = delete;
    VectorPrint(bool spaces) : spaces(spaces) {
        if (spaces) {
            nameSpaces = 40;
        } else {
            nameSpaces = 0;
        }
    }
    void operator()(ostream &stream) const;
};

void
VectorPrint::operator()(std::ostream &stream) const
{
    size_type _size = vec.size();
    Result _total = 0.0;

    if (flags.isSet(pdf | cdf)) {
        for (off_type i = 0; i < _size; ++i) {
            _total += vec[i];
        }
    }

    string base = name + separatorString;

    ScalarPrint print(spaces);
    print.name = name;
    print.desc = desc;
    print.precision = precision;
    print.descriptions = descriptions;
    print.flags = flags;
    print.pdf = _total ? 0.0 : NAN;
    print.cdf = _total ? 0.0 : NAN;

    bool havesub = !subnames.empty();

    if (_size == 1) {
        // If forceSubnames is set, get the first subname (or index in
        // the case where there are no subnames) and append it to the
        // base name.
        if (forceSubnames)
            print.name = base + (havesub ? subnames[0] : std::to_string(0));
        print.value = vec[0];
        print(stream);
        return;
    }

    if ((!flags.isSet(nozero)) || (total != 0)) {
        if (flags.isSet(oneline)) {
            ccprintf(stream, "%-*s", nameSpaces, name);
            print.flags = print.flags & (~nozero);
        }

        for (off_type i = 0; i < _size; ++i) {
            if (havesub && (i >= subnames.size() || subnames[i].empty()))
                continue;

            print.name = base + (havesub ? subnames[i] : std::to_string(i));
            print.desc = subdescs.empty() ? desc : subdescs[i];

            print.update(vec[i], _total);
            print(stream, flags.isSet(oneline));
        }

        if (flags.isSet(oneline)) {
            if (descriptions) {
                if (!desc.empty())
                    ccprintf(stream, " # %s", desc);
            }
            stream << endl;
        }
    }

    if (flags.isSet(::Stats::total)) {
        print.pdf = NAN;
        print.cdf = NAN;
        print.name = base + "total";
        print.desc = desc;
        print.value = total;
        print(stream);
    }
}

struct DistPrint
{
    string name;
    string separatorString;
    string desc;
    Flags flags;
    bool descriptions;
    bool spaces;
    int precision;
    int nameSpaces;

    const DistData &data;

    DistPrint(const Text *text, const DistInfo &info);
    DistPrint(const Text *text, const VectorDistInfo &info, int i);
    void init(const Text *text, const Info &info);
    void operator()(ostream &stream) const;
};

DistPrint::DistPrint(const Text *text, const DistInfo &info)
    : data(info.data)
{
    init(text, info);
}

DistPrint::DistPrint(const Text *text, const VectorDistInfo &info,
    int i) : data(info.data[i])
{
    init(text, info);

    name = text->statName(
        info.name + "_" +
        (info.subnames[i].empty() ? (std::to_string(i)) : info.subnames[i]));

    if (!info.subdescs[i].empty())
        desc = info.subdescs[i];
}

void
DistPrint::init(const Text *text, const Info &info)
{
    name = text->statName(info.name);
    separatorString = info.separatorString;
    desc = info.desc;
    flags = info.flags;
    precision = info.precision;
    descriptions = text->descriptions;
    spaces = text->spaces;
    if (spaces) {
        nameSpaces = 40;
    } else {
        nameSpaces = 0;
    }
}

void
DistPrint::operator()(ostream &stream) const
{
    if (flags.isSet(nozero) && data.samples == 0) return;
    string base = name + separatorString;

    ScalarPrint print(spaces);
    print.precision = precision;
    print.flags = flags;
    print.descriptions = descriptions;
    print.desc = desc;
    print.pdf = NAN;
    print.cdf = NAN;

    if (flags.isSet(oneline)) {
        print.name = base + "bucket_size";
        print.value = data.bucket_size;
        print(stream);

        print.name = base + "min_bucket";
        print.value = data.min;
        print(stream);

        print.name = base + "max_bucket";
        print.value = data.max;
        print(stream);
    }

    print.name = base + "samples";
    print.value = data.samples;
    print(stream);

    print.name = base + "mean";
    print.value = data.samples ? data.sum / data.samples : NAN;
    print(stream);

    if (data.type == Hist) {
        print.name = base + "gmean";
        print.value = data.samples ? exp(data.logs / data.samples) : NAN;
        print(stream);
    }

    Result stdev = NAN;
    if (data.samples)
        stdev = sqrt((data.samples * data.squares - data.sum * data.sum) /
                     (data.samples * (data.samples - 1.0)));
    print.name = base + "stdev";
    print.value = stdev;
    print(stream);

    if (data.type == Deviation)
        return;

    size_t size = data.cvec.size();

    Result total = 0.0;
    if (data.type == Dist && data.underflow != NAN)
        total += data.underflow;
    for (off_type i = 0; i < size; ++i)
        total += data.cvec[i];
    if (data.type == Dist && data.overflow != NAN)
        total += data.overflow;

    if (total) {
        print.pdf = 0.0;
        print.cdf = 0.0;
    }

    if (data.type == Dist && data.underflow != NAN) {
        print.name = base + "underflows";
        print.update(data.underflow, total);
        print(stream);
    }

    if (flags.isSet(oneline)) {
        ccprintf(stream, "%-*s", nameSpaces, name);
    }

    for (off_type i = 0; i < size; ++i) {
        stringstream namestr;
        namestr << base;

        Counter low = i * data.bucket_size + data.min;
        Counter high = ::min(low + data.bucket_size - 1.0, data.max);
        namestr << low;
        if (low < high)
            namestr << "-" << high;

        print.name = namestr.str();
        print.update(data.cvec[i], total);
        print(stream, flags.isSet(oneline));
    }

    if (flags.isSet(oneline)) {
        if (descriptions) {
            if (!desc.empty())
                ccprintf(stream, " # %s", desc);
        }
        stream << endl;
    }

    if (data.type == Dist && data.overflow != NAN) {
        print.name = base + "overflows";
        print.update(data.overflow, total);
        print(stream);
    }

    print.pdf = NAN;
    print.cdf = NAN;

    if (data.type == Dist && data.min_val != NAN) {
        print.name = base + "min_value";
        print.value = data.min_val;
        print(stream);
    }

    if (data.type == Dist && data.max_val != NAN) {
        print.name = base + "max_value";
        print.value = data.max_val;
        print(stream);
    }

    print.name = base + "total";
    print.value = total;
    print(stream);
}

void
Text::visit(const ScalarInfo &info)
{
    if (noOutput(info))
        return;

    ScalarPrint print(spaces);
    print.value = info.result();
    print.name = statName(info.name);
    print.desc = info.desc;
    print.flags = info.flags;
    print.descriptions = descriptions;
    print.precision = info.precision;
    print.pdf = NAN;
    print.cdf = NAN;

    print(*stream);
}

void
Text::visit(const VectorInfo &info)
{
    if (noOutput(info))
        return;

    size_type size = info.size();
    VectorPrint print(spaces);

    print.name = statName(info.name);
    print.separatorString = info.separatorString;
    print.desc = info.desc;
    print.flags = info.flags;
    print.descriptions = descriptions;
    print.precision = info.precision;
    print.vec = info.result();
    print.total = info.total();
    print.forceSubnames = false;

    if (!info.subnames.empty()) {
        for (off_type i = 0; i < size; ++i) {
            if (!info.subnames[i].empty()) {
                print.subnames = info.subnames;
                print.subnames.resize(size);
                for (off_type i = 0; i < size; ++i) {
                    if (!info.subnames[i].empty() &&
                        !info.subdescs[i].empty()) {
                        print.subdescs = info.subdescs;
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
Text::visit(const Vector2dInfo &info)
{
    if (noOutput(info))
        return;

    bool havesub = false;
    VectorPrint print(spaces);

    if (!info.y_subnames.empty()) {
        for (off_type i = 0; i < info.y; ++i) {
            if (!info.y_subnames[i].empty()) {
                print.subnames = info.y_subnames;
                break;
            }
        }
    }
    print.flags = info.flags;
    print.separatorString = info.separatorString;
    print.descriptions = descriptions;
    print.precision = info.precision;
    print.forceSubnames = true;

    if (!info.subnames.empty()) {
        for (off_type i = 0; i < info.x; ++i)
            if (!info.subnames[i].empty())
                havesub = true;
    }

    VResult tot_vec(info.y);
    for (off_type i = 0; i < info.x; ++i) {
        if (havesub && (i >= info.subnames.size() || info.subnames[i].empty()))
            continue;

        off_type iy = i * info.y;
        VResult yvec(info.y);

        Result total = 0.0;
        for (off_type j = 0; j < info.y; ++j) {
            yvec[j] = info.cvec[iy + j];
            tot_vec[j] += yvec[j];
            total += yvec[j];
        }

        print.name = statName(
            info.name + "_" +
            (havesub ? info.subnames[i] : std::to_string(i)));
        print.desc = info.desc;
        print.vec = yvec;
        print.total = total;
        print(*stream);
    }

    // Create a subname for printing the total
    vector<string> total_subname;
    total_subname.push_back("total");

    if (info.flags.isSet(::Stats::total) && (info.x > 1)) {
        print.name = statName(info.name);
        print.subnames = total_subname;
        print.desc = info.desc;
        print.vec = VResult(1, info.total());
        print.flags = print.flags & ~total;
        print(*stream);
    }
}

void
Text::visit(const DistInfo &info)
{
    if (noOutput(info))
        return;

    DistPrint print(this, info);
    print(*stream);
}

void
Text::visit(const VectorDistInfo &info)
{
    if (noOutput(info))
        return;

    for (off_type i = 0; i < info.size(); ++i) {
        DistPrint print(this, info, i);
        print(*stream);
    }
}

void
Text::visit(const FormulaInfo &info)
{
    visit((const VectorInfo &)info);
}

/*
  This struct implements the output methods for the sparse
  histogram stat
*/
struct SparseHistPrint
{
    string name;
    string separatorString;
    string desc;
    Flags flags;
    bool descriptions;
    bool spaces;
    int precision;

    const SparseHistData &data;

    SparseHistPrint(const Text *text, const SparseHistInfo &info);
    void init(const Text *text, const Info &info);
    void operator()(ostream &stream) const;
};

/* Call initialization function */
SparseHistPrint::SparseHistPrint(const Text *text, const SparseHistInfo &info)
    : data(info.data)
{
    init(text, info);
}

/* Initialization function */
void
SparseHistPrint::init(const Text *text, const Info &info)
{
    name = text->statName(info.name);
    separatorString = info.separatorString;
    desc = info.desc;
    flags = info.flags;
    precision = info.precision;
    descriptions = text->descriptions;
    spaces = text->spaces;
}

/* Grab data from map and write to output stream */
void
SparseHistPrint::operator()(ostream &stream) const
{
    string base = name + separatorString;

    ScalarPrint print(spaces);
    print.precision = precision;
    print.flags = flags;
    print.descriptions = descriptions;
    print.desc = desc;
    print.pdf = NAN;
    print.cdf = NAN;

    print.name = base + "samples";
    print.value = data.samples;
    print(stream);

    MCounter::const_iterator it;
    for (it = data.cmap.begin(); it != data.cmap.end(); it++) {
        stringstream namestr;
        namestr << base;

        namestr <<(*it).first;
        print.name = namestr.str();
        print.value = (*it).second;
        print(stream);
    }
}

void
Text::visit(const SparseHistInfo &info)
{
    if (noOutput(info))
        return;

    SparseHistPrint print(this, info);
    print(*stream);
}

Output *
initText(const string &filename, bool desc, bool spaces)
{
    static Text text;
    static bool connected = false;

    if (!connected) {
        text.open(*simout.findOrCreate(filename)->stream());
        text.descriptions = desc;
        text.spaces = spaces;
        connected = true;
    }

    return &text;
}

} // namespace Stats

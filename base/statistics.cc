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
#include "base/stats/statdb.hh"

using namespace std;

namespace Stats {

StatData *
DataAccess::find() const
{
    return Database::find(const_cast<void *>((const void *)this));
}

const StatData *
getStatData(const void *stat)
{
    return Database::find(const_cast<void *>(stat));
}

void
DataAccess::map(StatData *data)
{
    Database::regStat(this, data);
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
    Database::regPrint(this);
}

StatData::StatData()
    : flags(none), precision(-1), prereq(0)
{
    static int count = 0;
    id = count++;
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
#ifdef DEBUG
        cprintf("this is stat number %d\n", id);
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


void
FormulaBase::result(VResult &vec) const
{
    if (root)
        vec = root->result();
}

Result
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
    VResult vec;
    result(vec);
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
        root = NodePtr(new BinaryNode<std::plus<Result> >(root, r));
    else
        root = r;
    assert(size());
    return *this;
}

MainBin::MainBin(const string &name)
    : _name(name), mem(NULL), memsize(-1)
{
    Database::regBin(this, name);
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
    typedef Database::stat_list_t::iterator iter_t;

    iter_t i, end = Database::stats().end();
    for (i = Database::stats().begin(); i != end; ++i) {
        StatData *data = *i;
        assert(data);
        data->check();
    }

    int j = 0;
    for (i = Database::stats().begin(); i != end; ++i) {
        StatData *data = *i;
        if (!(data->flags & print))
            data->name = "__Stat" + to_string(j++);
    }

    Database::stats().sort(StatData::less);

    if (i == end)
        return;

    iter_t last = i;
    ++i;

    for (i = Database::stats().begin(); i != end; ++i) {
        if ((*i)->name == (*last)->name)
            panic("same name used twice! name=%s\n", (*i)->name);

        last = i;
    }
}

CallbackQueue resetQueue;

void
reset()
{
    // reset non-binned stats
    Database::stat_list_t::iterator i = Database::stats().begin();
    Database::stat_list_t::iterator end = Database::stats().end();
    while (i != end) {
        StatData *data = *i;
        if (!data->binned())
            data->reset();
        ++i;
    }

    // save the bin so we can go back to where we were
    MainBin *orig = MainBin::curBin();

    // reset binned stats
    Database::bin_list_t::iterator bi = Database::bins().begin();
    Database::bin_list_t::iterator be = Database::bins().end();
    while (bi != be) {
        MainBin *bin = *bi;
        bin->activate();

        i = Database::stats().begin();
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

    resetQueue.process();
}

void
registerResetCallback(Callback *cb)
{
    resetQueue.add(cb);
}

/* namespace Stats */ }

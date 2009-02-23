/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include <iomanip>
#include <fstream>
#include <list>
#include <map>
#include <string>

#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/hostinfo.hh"
#include "base/misc.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "base/time.hh"
#include "base/trace.hh"
#include "base/stats/statdb.hh"

using namespace std;

namespace Stats {

Info *
InfoAccess::find() const
{
    return Database::find(const_cast<void *>((const void *)this));
}

const Info *
getInfo(const void *stat)
{
    return Database::find(const_cast<void *>(stat));
}

void
InfoAccess::setInfo(Info *info)
{
    Database::regStat(this, info);
}

void
InfoAccess::setInit()
{
    info()->flags |= init;
}

Info *
InfoAccess::info()
{
    Info *info = find();
    assert(info);
    return info;
}

const Info *
InfoAccess::info() const
{
    const Info *info = find();
    assert(info);
    return info;
}

Info::Info()
    : flags(none), precision(-1), prereq(0)
{
    static int count = 0;
    id = count++;
}

Info::~Info()
{
}

bool
Info::less(Info *stat1, Info *stat2)
{
    const string &name1 = stat1->name;
    const string &name2 = stat2->name;

    vector<string> v1;
    vector<string> v2;

    tokenize(v1, name1, '.');
    tokenize(v2, name2, '.');

    size_type last = min(v1.size(), v2.size()) - 1;
    for (off_type i = 0; i < last; ++i)
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
Info::baseCheck() const
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

size_type
FormulaBase::size() const
{
    if (!root)
        return 0;
    else
        return root->size();
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
    for (off_t i = 0; i < vec.size(); ++i)
        if (vec[i] != 0.0)
            return false;
    return true;
}

void
FormulaBase::update(Info *)
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

void
check()
{
    typedef Database::stat_list_t::iterator iter_t;

    iter_t i, end = Database::stats().end();
    for (i = Database::stats().begin(); i != end; ++i) {
        Info *info = *i;
        assert(info);
        if (!info->check() || !info->baseCheck())
            panic("stat check failed for %s\n", info->name);
    }

    off_t j = 0;
    for (i = Database::stats().begin(); i != end; ++i) {
        Info *info = *i;
        if (!(info->flags & print))
            info->name = "__Stat" + to_string(j++);
    }

    Database::stats().sort(Info::less);

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
    Database::stat_list_t::iterator i = Database::stats().begin();
    Database::stat_list_t::iterator end = Database::stats().end();
    while (i != end) {
        Info *info = *i;
        info->reset();
        ++i;
    }

    resetQueue.process();
}

void
registerResetCallback(Callback *cb)
{
    resetQueue.add(cb);
}

/* namespace Stats */ }

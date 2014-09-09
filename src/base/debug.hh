/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2010 The Hewlett-Packard Development Company
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

#ifndef __BASE_DEBUG_HH__
#define __BASE_DEBUG_HH__

#include <map>
#include <string>
#include <vector>

namespace Debug {

void breakpoint();

class Flag
{
  protected:
    const char *_name;
    const char *_desc;
    std::vector<Flag *> _kids;

  public:
    Flag(const char *name, const char *desc);
    virtual ~Flag();

    std::string name() const { return _name; }
    std::string desc() const { return _desc; }
    std::vector<Flag *> kids() { return _kids; }

    virtual void enable() = 0;
    virtual void disable() = 0;
};

class SimpleFlag : public Flag
{
  protected:
    bool _status;

  public:
    SimpleFlag(const char *name, const char *desc)
        : Flag(name, desc), _status(false)
    { }

    bool status() const { return _status; }
    operator bool() const { return _status; }
    bool operator!() const { return !_status; }

    void enable() { _status = true; }
    void disable() { _status = false; }
};

class CompoundFlag : public SimpleFlag
{
  protected:
    void
    addFlag(Flag &f)
    {
        if (&f != NULL)
            _kids.push_back(&f);
    }

  public:
    CompoundFlag(const char *name, const char *desc,
        Flag &f00 = *(Flag *)0, Flag &f01 = *(Flag *)0,
        Flag &f02 = *(Flag *)0, Flag &f03 = *(Flag *)0,
        Flag &f04 = *(Flag *)0, Flag &f05 = *(Flag *)0,
        Flag &f06 = *(Flag *)0, Flag &f07 = *(Flag *)0,
        Flag &f08 = *(Flag *)0, Flag &f09 = *(Flag *)0,
        Flag &f10 = *(Flag *)0, Flag &f11 = *(Flag *)0,
        Flag &f12 = *(Flag *)0, Flag &f13 = *(Flag *)0,
        Flag &f14 = *(Flag *)0, Flag &f15 = *(Flag *)0,
        Flag &f16 = *(Flag *)0, Flag &f17 = *(Flag *)0,
        Flag &f18 = *(Flag *)0, Flag &f19 = *(Flag *)0)
        : SimpleFlag(name, desc)
    {
        addFlag(f00); addFlag(f01); addFlag(f02); addFlag(f03); addFlag(f04);
        addFlag(f05); addFlag(f06); addFlag(f07); addFlag(f08); addFlag(f09);
        addFlag(f10); addFlag(f11); addFlag(f12); addFlag(f13); addFlag(f14);
        addFlag(f15); addFlag(f16); addFlag(f17); addFlag(f18); addFlag(f19);
    }

    void enable();
    void disable();
};

typedef std::map<std::string, Flag *> FlagsMap;
FlagsMap &allFlags();

Flag *findFlag(const std::string &name);

extern Flag *const All;

bool changeFlag(const char *s, bool value);

} // namespace Debug

void setDebugFlag(const char *string);

void clearDebugFlag(const char *string);

void dumpDebugFlags();

#endif // __BASE_DEBUG_HH__

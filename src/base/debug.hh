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
 */

#ifndef __BASE_DEBUG_HH__
#define __BASE_DEBUG_HH__

#include <initializer_list>
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

  public:
    Flag(const char *name, const char *desc);
    virtual ~Flag();

    std::string name() const { return _name; }
    std::string desc() const { return _desc; }
    virtual std::vector<Flag *> kids() { return std::vector<Flag*>(); }

    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual void sync() {}
};

class SimpleFlag : public Flag
{
    static bool _active; // whether debug tracings are enabled
  protected:
    bool _tracing; // tracing is enabled and flag is on
    bool _status;  // flag status

  public:
    SimpleFlag(const char *name, const char *desc)
        : Flag(name, desc), _status(false)
    { }

    bool status() const { return _tracing; }
    operator bool() const { return _tracing; }
    bool operator!() const { return !_tracing; }

    void enable()  { _status = true;  sync(); }
    void disable() { _status = false; sync(); }

    void sync() { _tracing = _active && _status; }

    static void enableAll();
    static void disableAll();
};

class CompoundFlag : public Flag
{
  protected:
    std::vector<Flag *> _kids;

  public:
    template<typename... Args>
    CompoundFlag(const char *name, const char *desc,
                 std::initializer_list<Flag *> flags)
        : Flag(name, desc),
          _kids(flags)
    {
    }

    std::vector<Flag *> kids() { return _kids; }

    void enable();
    void disable();
};

typedef std::map<std::string, Flag *> FlagsMap;
FlagsMap &allFlags();

Flag *findFlag(const std::string &name);

bool changeFlag(const char *s, bool value);

} // namespace Debug

void setDebugFlag(const char *string);

void clearDebugFlag(const char *string);

void dumpDebugFlags();

#if TRACING_ON
#   define DTRACE(x) (Debug::x)
#else // !TRACING_ON
#   define DTRACE(x) (false)
#endif  // TRACING_ON

#endif // __BASE_DEBUG_HH__

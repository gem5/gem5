/*
 * Copyright 2020 Google Inc.
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

#ifndef __CALL_TYPE_HH__
#define __CALL_TYPE_HH__

#include <iostream>
#include <map>
#include <string>
#include <utility>

class Args;
class DispatchTable;

class CallType
{
  public:
    enum class CheckArgsResult {
        Match,
        NoMatch,
        Usage
    };

  protected:
    const std::string name;

    virtual bool isDefault() const = 0;
    virtual CheckArgsResult checkArgs(Args &args);
    virtual void init() {}

    static std::map<std::string, CallType &> &map();

    virtual void printBrief(std::ostream &os) const { os << "--" << name; }
    virtual void printDesc(std::ostream &os) const = 0;
    std::string formattedUsage() const;

  public:
    CallType(const std::string &_name) : name(_name)
    {
        map().emplace(std::piecewise_construct,
            std::forward_as_tuple(std::string(_name)),
            std::forward_as_tuple(*this));
    }

    ~CallType()
    {
        map().erase(name);
    }

    static CallType *detect(Args &args);
    static std::string usageSummary();

    virtual const DispatchTable &getDispatch() const = 0;
};


#endif // __CALL_TYPE_HH__

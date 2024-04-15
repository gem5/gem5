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
 */

#ifndef __COMMAND_HH__
#define __COMMAND_HH__

#include <map>
#include <string>
#include <utility>

class Args;
class DispatchTable;

class Command
{
  private:
    const std::string name;

    // The minimum number of arguments the command expects.
    const int minArgs;
    // The maximum number of arguments the command can handle.
    const int maxArgs;

    using FuncType = bool (*)(const DispatchTable &dt, Args &args);
    // A function which processes command line arguments and passes them to
    // the underlying function through the dispatch table.
    FuncType func;

    // Help text for this command.
    const std::string usageStr;

    static std::map<std::string, Command &> &map();

  public:
    Command(const std::string &_name, int _min, int _max, FuncType _func,
            const std::string &_usage)
        : name(_name),
          minArgs(_min),
          maxArgs(_max),
          func(_func),
          usageStr(_usage)
    {
        map().emplace(std::piecewise_construct,
                      std::forward_as_tuple(std::string(_name)),
                      std::forward_as_tuple(*this));
    }

    ~Command() { map().erase(name); }

    static bool run(const DispatchTable &dt, Args &args);

    static std::string
    usageSummary()
    {
        std::string summary;
        for (auto &p : Command::map())
            summary += "    " + p.first + " " + p.second.usageStr + "\n";
        return summary;
    }
};

#endif // __COMMAND_HH__

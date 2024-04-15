/*
 * Copyright 2017 Google Inc.
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

#include "base/gtest/logging.hh"

#include <string>

#include "base/logging.hh"

namespace gem5
{

namespace
{

class GTestLogger : public Logger
{
  public:
    using Logger::Logger;

  protected:
    void
    log(const Loc &loc, std::string s) override
    {
        gtestLogOutput << s;
        SUCCEED() << s;
    }
};

class GTestExitLogger : public Logger
{
  public:
    using Logger::Logger;

  protected:
    void
    log(const Loc &loc, std::string s) override
    {
        gtestLogOutput << s;
        std::cerr << loc.file << ":" << loc.line << ": " << s;
    }

    // Throw an exception to escape down to the gtest framework.
    void
    exit() override
    {
        throw GTestException();
    }
};

} // anonymous namespace

// We intentionally put all the loggers on the heap to prevent them from being
// destructed at the end of the program. This make them safe to be used inside
// destructor of other global objects. Also, we make them function static
// veriables to ensure they are initialized ondemand, so it is also safe to use
// them inside constructor of other global objects.

Logger &
Logger::getPanic()
{
    static GTestExitLogger *panic_logger = new GTestExitLogger("panic: ");
    return *panic_logger;
}

Logger &
Logger::getFatal()
{
    static GTestExitLogger *fatal_logger = new GTestExitLogger("fatal: ");
    return *fatal_logger;
}

Logger &
Logger::getWarn()
{
    static GTestLogger *warn_logger = new GTestLogger("warn: ");
    return *warn_logger;
}

Logger &
Logger::getInfo()
{
    static GTestLogger *info_logger = new GTestLogger("info: ");
    return *info_logger;
}

Logger &
Logger::getHack()
{
    static GTestLogger *hack_logger = new GTestLogger("hack: ");
    return *hack_logger;
}

} // namespace gem5

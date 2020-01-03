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

#include <gtest/gtest.h>

#include <array>
#include <string>

#include "base/cprintf.hh"
#include "base/logging.hh"

namespace {

// This custom exception type will help prevent fatal exceptions from being
// caught by other code in gem5 and let them escape to the gtest framework.
// Unfortunately that results in a somewhat confusing message about an unknown
// exception being thrown after the panic/fatal message has been printed, but
// there will at least be some indication what went wrong.
struct GTestException
{};

class GTestLogger : public Logger
{
  public:
    using Logger::Logger;

  protected:
    void log(const Loc &loc, std::string s) override { SUCCEED() << s; }
};

class GTestExitLogger : public Logger
{
  public:
    using Logger::Logger;

  protected:
    void
    log(const Loc &loc, std::string s) override
    {
        std::cerr << loc.file << ":" << loc.line << ": " << s;
    }
    // Throw an exception to escape down to the gtest framework.
    void exit() override { throw GTestException(); }
};

GTestExitLogger panicLogger("panic: ");
GTestExitLogger fatalLogger("fatal: ");
GTestLogger warnLogger("warn: ");
GTestLogger infoLogger("info: ");
GTestLogger hackLogger("hack: ");

} // anonymous namespace

Logger &Logger::getPanic() { return panicLogger; }
Logger &Logger::getFatal() { return fatalLogger; }
Logger &Logger::getWarn() { return warnLogger; }
Logger &Logger::getInfo() { return infoLogger; }
Logger &Logger::getHack() { return hackLogger; }

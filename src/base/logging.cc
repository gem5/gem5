/*
 * Copyright (c) 2014, 2017 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#include "base/logging.hh"

#include <sstream>

#include "base/hostinfo.hh"

namespace gem5
{

namespace {

class ExitLogger : public Logger
{
  public:
    ExitLogger(const char *prefix) : Logger(prefix)
    {
        registerExtraLog([]() {
            return csprintf("Memory Usage: %ld KBytes\n", memUsage());
        });
    }
};

class FatalLogger : public ExitLogger
{
  public:
    using ExitLogger::ExitLogger;

  protected:
    void exit() override { ::exit(1); }
};

} // anonymous namespace

// We intentionally put all the loggers on the heap to prevent them from being
// destructed at the end of the program. This make them safe to be used inside
// destructor of other global objects. Also, we make them function static
// veriables to ensure they are initialized ondemand, so it is also safe to use
// them inside constructor of other global objects.

Logger&
Logger::getPanic() {
    static ExitLogger* panic_logger = new ExitLogger("panic: ");
    return *panic_logger;
}

Logger&
Logger::getFatal() {
    static FatalLogger* fatal_logger = new FatalLogger("fatal: ");
    return *fatal_logger;
}

Logger&
Logger::getWarn() {
    static Logger* warn_logger = new Logger("warn: ");
    return *warn_logger;
}

Logger&
Logger::getInfo() {
    static Logger* info_logger = new Logger("info: ");
    return *info_logger;
}

Logger&
Logger::getHack() {
    static Logger* hack_logger = new Logger("hack: ");
    return *hack_logger;
}

} // namespace gem5

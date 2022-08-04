/*
 * Copyright (c) 2014, 2019 ARM Limited
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

/**
 * @file
 *
 * A logger to allow SystemC to capture DPRINTF messages (and similar things)
 * using sc_report
 */

#include <cstdlib>
#include <cstring>
#include <sstream>

#include "sc_logger.hh"

namespace Gem5SystemC
{

/** Class to act as a streambuf for std::ostream which cuts output strings
 *  into lines and offers them to a logger */
class CuttingStreambuf : public std::streambuf
{
  public:
    /** Accumulate line so far */
    std::ostringstream line;

    /** Logger to send complete lines to */
    gem5::Trace::Logger *logger;

    CuttingStreambuf(gem5::Trace::Logger *logger_) : logger(logger_)
    { }

    /** Accumulate to line up to \n and then emit */
    int overflow(int i);
    int sync();

    /** Push a line out to the logger */
    void outputLine();

    ~CuttingStreambuf();
};

void CuttingStreambuf::outputLine()
{
    logger->logMessage((gem5::Tick)-1, "gem5", "", line.str());
    line.clear();
    line.str("");
}

/** This is pretty much the least efficient way of doing this, but it has the
 *  advantage of having very few corners to get wrong.
 *
 *  A newly allocated streambuf will have no buffer to serve to its
 *  [oi]stream.  It will, therefore, call overflow for every character it
 *  wants to insert into the output stream.  Those characters are captured one
 *  by one here and added to this->line. */
int
CuttingStreambuf::overflow(int chr)
{
    if (chr == '\n')
        outputLine();
    else if (chr != EOF)
        line << (char) chr;

    /* Always succeeds */
    return 0;
}

int
CuttingStreambuf::sync()
{
    if (!line.str().empty())
        outputLine();

    /* Always succeeds */
    return 0;
}

CuttingStreambuf::~CuttingStreambuf()
{
    sync();
}

Logger::Logger() :
    cuttingStreambuf(new CuttingStreambuf(this)),
    stream(cuttingStreambuf)
{
}

Logger::~Logger()
{
    stream.flush();
    delete cuttingStreambuf;
}

/** Log a single message as a single sc_report call */
void
Logger::logMessage(gem5::Tick when, const std::string &name,
    const std::string &flag, const std::string &message)
{
    /* Need to chop the newline off the message */
    std::string message_without_nl = message;
    message_without_nl.erase(
        message_without_nl.find_last_not_of(" \n\r") + 1);

    SC_REPORT_INFO(name.c_str(), message_without_nl.c_str());
}

std::ostream &
Logger::getOstream()
{
    return stream;
}

} // namespace Gem5SystemC

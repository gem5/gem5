/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#include <sys/types.h>
#include <sys/stat.h>

#include <cstring>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "sim/universe.hh"
#include "sim/host.hh"
#include "sim/param.hh"

using namespace std;

Tick curTick = 0;
Tick ticksPerSecond;
double __ticksPerMS;
double __ticksPerUS;
double __ticksPerNS;
double __ticksPerPS;

string outputDirectory;
ostream *outputStream;
ostream *configStream;

class UniverseParamContext : public ParamContext
{
  public:
    UniverseParamContext(const string &is) : ParamContext(is) {}
    void checkParams();
};

UniverseParamContext universe("Universe");

Param<Tick> universe_freq(&universe, "frequency", "tick frequency",
                          200000000);

Param<string> universe_output_dir(&universe, "output_dir",
                                  "directory to output data to",
                                  ".");
Param<string> universe_output_file(&universe, "output_file",
                                   "file to dump simulator output to",
                                   "cout");
Param<string> universe_config_output_file(&universe, "config_output_file",
                                          "file to dump simulator config to",
                                          "m5config.out");

void
UniverseParamContext::checkParams()
{
    ticksPerSecond = universe_freq;
    double freq = double(ticksPerSecond);
    __ticksPerMS = freq / 1.0e3;
    __ticksPerUS = freq / 1.0e6;
    __ticksPerNS = freq / 1.0e9;
    __ticksPerPS = freq / 1.0e12;

    if (universe_output_dir.isValid()) {
        outputDirectory = universe_output_dir;

        // guarantee that directory ends with a '/'
        if (outputDirectory[outputDirectory.size() - 1] != '/')
            outputDirectory += "/";

        if (mkdir(outputDirectory.c_str(), 0777) < 0) {
            if (errno != EEXIST) {
                panic("%s\ncould not make output directory: %s\n",
                      strerror(errno), outputDirectory);
            }
        }
    }

    outputStream = makeOutputStream(universe_output_file);
    configStream = universe_config_output_file.isValid()
        ? makeOutputStream(universe_config_output_file)
        : outputStream;
}


std::ostream *
makeOutputStream(std::string &name)
{
    if (name == "cerr" || name == "stderr")
        return &std::cerr;

    if (name == "cout" || name == "stdout")
        return &std::cout;

    string path = (name[0] != '/') ? outputDirectory + name : name;

    // have to dynamically allocate a stream since we're going to
    // return it... though the caller can't easily free it since it
    // may be cerr or cout.  need GC!
    ofstream *s = new ofstream(path.c_str(), ios::trunc);

    if (!s->is_open())
        fatal("Cannot open file %s", path);

    return s;
}


void
closeOutputStream(std::ostream *os)
{
    // can't close cerr or cout
    if (os == &std::cerr || os == &std::cout)
        return;

    // can only close ofstreams, not generic ostreams, so try to
    // downcast and close only if the downcast succeeds
    std::ofstream *ofs = dynamic_cast<std::ofstream *>(os);
    if (ofs)
        ofs->close();
}




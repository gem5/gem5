// Copyright (c) 2015 ARM Limited
// All rights reserved.
// 
// The license below extends only to copyright in the software and shall
// not be construed as granting a license to any other intellectual
// property including but not limited to intellectual property relating
// to a hardware implementation of the functionality of the software
// licensed hereunder.  You may use the software subject to the license
// terms below provided that you ensure that this notice is replicated
// unmodified and in its entirety in all distributions of the software,
// modified or unmodified, in source code or in binary form.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Copyright 2009-2014 Sandia Coporation.  Under the terms
// of Contract DE-AC04-94AL85000 with Sandia Corporation, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2014, Sandia Corporation
// All rights reserved.
//
// For license information, see the LICENSE file in the current directory.

#ifndef EXT_SST_GEM5_HH
#define EXT_SST_GEM5_HH

#include <string>
#include <vector>

#include <core/sst_config.h>
#include <core/component.h>

#include <sim/simulate.hh>

#include "ExtMaster.hh"
#include "ExtSlave.hh"

namespace SST {
namespace gem5 {

class gem5Component : public SST::Component,
                      public ExternalSlave::Handler,
                      public ExternalMaster::Handler {
private:

    Output dbg;
    Output info;
    uint64_t sim_cycles;
    uint64_t clocks_processed;

    std::vector<ExtMaster*> masters;
    std::vector<ExtSlave*> slaves;

    void splitCommandArgs(std::string &cmd, std::vector<char*> &args);
    void initPython(int argc, char *argv[]);

public:
    gem5Component(ComponentId_t id, Params &params);
    ~gem5Component();
    virtual void init(unsigned);
    virtual void setup();
    virtual void finish();
    bool clockTick(Cycle_t);

    virtual ExternalMaster::Port *getExternalPort(
        const std::string &name, ExternalMaster &owner,
        const std::string &port_data);

    virtual ExternalSlave::Port *getExternalPort(
        const std::string &name, ExternalSlave &owner,
        const std::string &port_data);
};

}
}

#endif

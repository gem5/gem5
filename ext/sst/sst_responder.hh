// Copyright (c) 2023 The Regents of the University of California
// All rights reserved.
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

#ifndef __SST_RESPONDER_HH__
#define __SST_RESPONDER_HH__

#define TRACING_ON 0

#include <string>
#include <vector>

#include <sst/core/sst_config.h>
#include <sst/core/component.h>

#include <sst/core/interfaces/stringEvent.h>
#include <sst/core/interfaces/stdMem.h>

#include <sst/core/eli/elementinfo.h>
#include <sst/core/link.h>

#include <sst/elements/memHierarchy/memEvent.h>
#include <sst/elements/memHierarchy/memTypes.h>
#include <sst/elements/memHierarchy/util.h>

// from gem5
#include <sim/sim_object.hh>
#include <sst/outgoing_request_bridge.hh>
#include <sst/sst_responder_interface.hh>

#include "sst_responder_subcomponent.hh"

class SSTResponderSubComponent;

class SSTResponder: public gem5::SSTResponderInterface
{
  private:
    SSTResponderSubComponent* owner;
    SST::Output* output;
  public:
    SSTResponder(SSTResponderSubComponent* owner_);
    ~SSTResponder() override;

    void setOutputStream(SST::Output* output_);

    bool handleRecvTimingReq(gem5::PacketPtr pkt) override;
    void handleRecvRespRetry() override;
    void handleRecvFunctional(gem5::PacketPtr pkt) override;
};

#endif // __SST_RESPONDER_HH__

/*
 * Copyright (c) 2015 ARM Limited
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

#ifndef __MEM_PROBES_BASE_HH__
#define __MEM_PROBES_BASE_HH__

#include <memory>
#include <vector>

#include "sim/probe/mem.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct BaseMemProbeParams;

/**
 * Base class for memory system probes accepting Packet instances.
 *
 * This is a helper base class for memory system probes that
 * instrument Packet handling. Unlike the ProbeListenerObject base
 * class, this class supports instrumentation of multiple ProbeManager
 * instances. However, it's limited to one probe point name. This
 * enables features like tracing or stack distance analysis of packets
 * from multiple components using the same probe. For example, a stack
 * distance probe could be hooked up to multiple memories in a
 * multi-channel configuration.
 */
class BaseMemProbe : public SimObject
{
  public:
    BaseMemProbe(const BaseMemProbeParams &params);

    void regProbeListeners() override;

  protected:
    /**
     * Callback to analyse intercepted Packets.
     */
    virtual void handleRequest(const probing::PacketInfo &pkt_info) = 0;

  private:
    class PacketListener : public ProbeListenerArgBase<probing::PacketInfo>
    {
      public:
        PacketListener(BaseMemProbe &_parent, std::string name)
            : ProbeListenerArgBase(std::move(name)),
              parent(_parent) {}

        void notify(const probing::PacketInfo &pkt_info) override {
            parent.handleRequest(pkt_info);
        }

      protected:
        BaseMemProbe &parent;
    };

    std::vector<ProbeConnectionPtr> listeners;
};

} // namespace gem5

#endif //  __MEM_PROBES_BASE_HH__

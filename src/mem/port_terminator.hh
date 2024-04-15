/*
 * Copyright (c) 2021 The Regents of the University of California.
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

#ifndef __MEM_PORT_TERMINATOR_HH__
#define __MEM_PORT_TERMINATOR_HH__

/**
 * @file port_terminator.hh
 * Contains the description of the class PortTerminator. It is useful for cases
 * where you do not need to connect all of the ports in your system, but the
 * simulator is complaining about orphan ports. For example if you have
 * configured a cache hierarchy and want to test its performance using
 * PyTrafficGen, you will end up with an icache that is not connected to any
 * other component in the system. In this case you can just connect that port
 * to this object. This object will not issue any request or respond to any
 * request. It is neccessary to make sure the ports that are connected to
 * a PortTerminator are never going to be used in your system.
 */

#include <vector>

#include "mem/port.hh"
#include "params/PortTerminator.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class PortTerminator : public SimObject
{
  private:
    /**
     * @brief definition of the ReqPort class. It is of type RequestPort
     * It will always return true when it receives a timing response. However,
     * it should never become useful if PortTerminator is used correctly in
     * your system (since it is a pure virtual function it should be
     * implemented for any class that inherits from RequestPort). It will never
     * send request retires, nor it will need to keep track of address ranges
     * of its peer port.
     */
    class ReqPort : public RequestPort
    {
      public:
        ReqPort(const std::string &name) : RequestPort(name) {}

      protected:
        bool
        recvTimingResp(PacketPtr pkt) override
        {
            panic("Received an unexpected response. RequestPorts on a "
                  "PortTerminator never issue any requests. Therefore, they "
                  "should "
                  "never receive a response.\n");
        }

        void
        recvReqRetry() override
        {
            return;
        }

        void
        recvRangeChange() override
        {
            return;
        }
    };

    /**
     * @brief definition of the RespPort class. It is of type ResponsePort
     * It is a ReponsePort that should never receive a request. If this is not
     * true in the system you configured, probably you should not use
     * PortTerminator for the port connected to PortTerminator.
     */
    class RespPort : public ResponsePort
    {
      public:
        RespPort(const std::string &name) : ResponsePort(name) {}
    };

    std::vector<ReqPort> reqPorts;

    std::vector<RespPort> respPorts;

  public:
    PortTerminator(const PortTerminatorParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

} // namespace gem5

#endif // __MEM_PORT_TERMINATOR_HH__

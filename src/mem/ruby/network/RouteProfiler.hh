/*
 * Copyright (c) 2026 Arm Limited
 * All rights reserved.
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

#ifndef __MEM_RUBY_NETWORK_ROUTEPROFILER_HH__
#define __MEM_RUBY_NETWORK_ROUTEPROFILER_HH__

#include <iostream>
#include <unordered_map>
#include <vector>

#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "mem/ruby/network/MessageBuffer.hh"

namespace gem5
{

namespace ruby
{

class RouteProfiler
{

  public:
    RouteProfiler() : m_enabled(false) {}

    // Notifies a message is ready at a router input port
    void profFrontEndRdy(Message *msg, BasicRouter *router,
                         MessageBuffer *src_link, int vnet);

    // Notifies ready message was cloned (e.g. on multicast messages)
    void profFrontEndRdyClone(Message *msg, BasicRouter *router,
                              MessageBuffer *src_link, int vnet,
                              Message *clone_from_msg);

    // Notifies a ready message was routed to an output port buffer
    void profFrontEndFwd(Message *msg, BasicRouter *router, int vnet);

    // Notifies a message is ready at an output port buffer
    void profBackEndRdy(Message *msg, BasicRouter *router, int vnet);

    // Notifies a ready message was sent through the output link
    void profBackEndFwd(Message *msg, BasicRouter *router, int vnet);

    // Notifies a ready message was sent through the output link to
    // its final destination
    void profBackEndFwdExt(Message *msg, BasicRouter *router,
                           MessageBuffer *dest_link, int vnet);

    // Dump all routes to file
    void dumpRoutes();

    // enable disable profiling routes
    void
    enable()
    {
        m_enabled = true;
    }
    void
    disable()
    {
        m_enabled = false;
    }

  private:
    bool m_enabled;

    struct OutstandingRouteEntry
    {
        BasicRouter *router;
        Tick frontend_rdy;
        Tick frontend_fwd;
        Tick backend_rdy;
        Tick backend_fwd;

        OutstandingRouteEntry(BasicRouter *_router, Tick _frontend_rdy)
            : router(_router), frontend_rdy(_frontend_rdy)
        {}
    };

    struct OutstandingRoute
    {
        Message *msg;
        MessageBuffer *src_link;
        int vnet;
        std::vector<OutstandingRouteEntry> entries;

        OutstandingRoute(Message *_msg, MessageBuffer *_src_link, int _vnet)
            : msg(_msg), src_link(_src_link), vnet(_vnet)
        {}
    };

    struct AvgVal
    {
        double acc;
        uint64_t cnt;
        AvgVal() : acc(0), cnt(0) {}
        void
        inc(double val)
        {
            acc += val;
            cnt += 1;
        }
        double
        avg() const
        {
            return acc / cnt;
        };
    };

    struct RouteEntry
    {
        BasicRouter *router;
        AvgVal frontend_delay;
        AvgVal backend_delay;
    };

    struct Route
    {
        int vnet;
        MessageBuffer *src_link;
        MessageBuffer *dst_link;
        AvgVal total_delay;
        std::vector<RouteEntry> route;
        Route() : vnet(), src_link(nullptr), dst_link(nullptr) {}
        double
        badness() const
        {
            return total_delay.avg() / route.size();
        }
    };

    std::unordered_map<Message *, OutstandingRoute> outstanding_routes;
    std::unordered_map<std::string, Route> routes;

    std::string genRouteString(OutstandingRoute &route,
                               MessageBuffer *dest_link);
    void dumpRoute(const Route &route, std::ostream &outs) const;
};

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_ROUTEPROFILER_HH__

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

#include "mem/ruby/network/RouteProfiler.hh"

#include <cassert>
#include <numeric>

#include "base/output.hh"
#include "debug/TraceRoutesDebug.hh"

namespace gem5
{

namespace ruby
{

void
RouteProfiler::profFrontEndRdy(Message *msg, BasicRouter *router,
                               MessageBuffer *src_link, int vnet)
{
    if (!m_enabled) {
        return;
    }

    auto tmp = outstanding_routes.emplace(
        std::piecewise_construct, std::forward_as_tuple(msg),
        std::forward_as_tuple(msg, src_link, vnet));
    OutstandingRoute &entry = tmp.first->second;
    assert(msg == entry.msg);
    assert(vnet == entry.vnet);
    if (bool is_new = tmp.second; is_new) {
        assert(entry.entries.empty());
        assert(entry.src_link == src_link);
    }
    if (entry.entries.empty() || (entry.entries.back().router != router)) {
        entry.entries.emplace_back(router, curTick());
        DPRINTFS(TraceRoutesDebug, router, "Msg %#x entering router vnet=%d\n",
                 msg, vnet);
    }
    assert(entry.entries.back().router == router);
}

void
RouteProfiler::profFrontEndRdyClone(Message *msg, BasicRouter *router,
                                    MessageBuffer *src_link, int vnet,
                                    Message *clone_from_msg)
{
    if (!m_enabled) {
        return;
    }

    auto tmp = outstanding_routes.emplace(
        std::piecewise_construct, std::forward_as_tuple(msg),
        std::forward_as_tuple(msg, src_link, vnet));
    OutstandingRoute &entry = tmp.first->second;
    assert(msg == entry.msg);
    assert(vnet == entry.vnet);
    assert(tmp.second);
    assert(entry.entries.empty());

    auto iter = outstanding_routes.find(clone_from_msg);
    assert(iter != outstanding_routes.end());
    OutstandingRoute &other = iter->second;
    assert(entry.vnet == other.vnet);
    assert(!other.entries.empty());
    assert(other.entries.back().router == router);
    entry.src_link = other.src_link;
    entry.entries = other.entries;
    DPRINTFS(TraceRoutesDebug, router, "Msg %#x cloned in router vnet=%d\n",
             msg, vnet);
}

void
RouteProfiler::profFrontEndFwd(Message *msg, BasicRouter *router, int vnet)
{
    if (!m_enabled) {
        return;
    }

    auto iter = outstanding_routes.find(msg);
    assert(iter != outstanding_routes.end());
    assert(!iter->second.entries.empty());
    assert(iter->second.msg == msg);
    assert(vnet == iter->second.vnet);
    OutstandingRouteEntry &entry = iter->second.entries.back();
    assert(entry.router == router);
    entry.frontend_fwd = curTick();
    entry.backend_rdy = 0;
}

void
RouteProfiler::profBackEndRdy(Message *msg, BasicRouter *router, int vnet)
{
    if (!m_enabled) {
        return;
    }

    auto iter = outstanding_routes.find(msg);
    assert(iter != outstanding_routes.end());
    assert(!iter->second.entries.empty());
    assert(iter->second.msg == msg);
    assert(vnet == iter->second.vnet);
    OutstandingRouteEntry &entry = iter->second.entries.back();
    assert(entry.router == router);
    if (entry.backend_rdy == 0) {
        entry.backend_rdy = curTick();
    }
}

void
RouteProfiler::profBackEndFwd(Message *msg, BasicRouter *router, int vnet)
{
    if (!m_enabled) {
        return;
    }

    auto iter = outstanding_routes.find(msg);
    assert(iter != outstanding_routes.end());
    assert(!iter->second.entries.empty());
    assert(iter->second.msg == msg);
    assert(vnet == iter->second.vnet);
    OutstandingRouteEntry &entry = iter->second.entries.back();
    assert(entry.router == router);
    entry.backend_fwd = curTick();
    DPRINTFS(TraceRoutesDebug, router,
             "Msg %#x leaving router vnet=%d delay=%d delay_total=%d\n", msg,
             vnet, entry.backend_fwd - entry.frontend_rdy,
             entry.backend_fwd - iter->second.entries.front().frontend_rdy);
}

void
RouteProfiler::profBackEndFwdExt(Message *msg, BasicRouter *router,
                                 MessageBuffer *dest_link, int vnet)
{
    if (!m_enabled) {
        return;
    }

    profBackEndFwd(msg, router, vnet);
    auto iter = outstanding_routes.find(msg);
    OutstandingRoute &entry = iter->second;

    std::string rs = genRouteString(entry, dest_link);

    Route &route = routes[rs];

    bool was_empty = route.src_link == nullptr;
    if (was_empty) {
        route.src_link = entry.src_link;
        route.dst_link = dest_link;
        route.vnet = vnet;
        for (auto e : entry.entries) {
            route.route.emplace_back();
            auto &r = route.route.back();
            r.router = e.router;
            r.frontend_delay.inc(
                router->ticksToCycles(e.frontend_fwd - e.frontend_rdy));
            r.backend_delay.inc(
                router->ticksToCycles(e.backend_fwd - e.frontend_fwd));
        }
    } else {
        assert(route.src_link == entry.src_link);
        assert(route.dst_link == dest_link);
        assert(route.vnet == vnet);
        assert(route.route.size() == entry.entries.size());

        for (unsigned i = 0; i < entry.entries.size(); ++i) {
            auto &e = entry.entries[i];
            auto &r = route.route[i];
            assert(r.router == e.router);
            r.frontend_delay.inc(
                router->ticksToCycles(e.frontend_fwd - e.frontend_rdy));
            r.backend_delay.inc(
                router->ticksToCycles(e.backend_fwd - e.frontend_fwd));
        }
    }
    route.total_delay.inc(
        router->ticksToCycles(entry.entries.back().backend_fwd -
                              entry.entries.front().frontend_rdy));

    if (debug::TraceRoutesDebug) {
        std::ostringstream ss;
        dumpRoute(route, ss);
        DPRINTF(TraceRoutesDebug, "%s route: %s",
                was_empty ? "New" : "Updated", ss.str());
    }

    outstanding_routes.erase(iter);
}

std::string
RouteProfiler::genRouteString(OutstandingRoute &route,
                              MessageBuffer *dest_link)
{
    assert(!route.entries.empty());
    std::stringstream ss;
    ss << route.vnet << " " << route.src_link->name() << "->";
    for (auto e : route.entries) {
        ss << "R" << e.router->getID() << "->";
    }
    ss << dest_link->name();
    return ss.str();
}

void
RouteProfiler::dumpRoute(const Route &route, std::ostream &outs) const
{
    assert(!route.route.empty());
    ccprintf(outs, "%.1f %d ", route.total_delay.avg(), route.total_delay.cnt);
    ccprintf(outs, "vnet%d %s -> ", route.vnet, route.src_link->name());
    for (auto r : route.route) {
        ccprintf(outs, "R%d(%.1f,%.1f) -> ", r.router->getID(),
                 r.frontend_delay.avg(), r.backend_delay.avg());
    }
    ccprintf(outs, "%s\n", route.dst_link->name());
}

void
RouteProfiler::dumpRoutes()
{
    if (routes.empty()) {
        return;
    }

    // use set to order map value by badness
    typedef std::pair<std::string, Route> settype;
    struct compare
    {
        bool
        operator()(const settype &l, const settype &r) const
        {
            if (l.second.badness() != r.second.badness()) {
                return l.second.badness() > r.second.badness();
            } else {
                return l.first > r.first;
            }
        }
    };

    std::set<settype, compare> ordered(routes.begin(), routes.end());

    OutputStream *out =
        simout.open("interconnect_routes.txt", std::ios_base::out);
    inform("Dumping all routes to %s\n", out->name());
    std::ostream &outs = *out->stream();

    for (auto &val : ordered) {
        dumpRoute(val.second, outs);
    }
}

} // namespace ruby
} // namespace gem5

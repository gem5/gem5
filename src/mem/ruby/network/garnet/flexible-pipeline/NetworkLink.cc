/*
 * Copyright (c) 2008 Princeton University
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
 *
 * Authors: Niket Agarwal
 */

#include "mem/ruby/network/garnet/flexible-pipeline/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/NetworkLink.hh"

NetworkLink::NetworkLink(const Params *p)
    : ClockedObject(p), Consumer(this), m_id(p->link_id),
      m_latency(p->link_latency), m_in_port(0), m_out_port(0),
      linkBuffer(new flitBuffer()), link_consumer(nullptr),
      link_source(nullptr), link_srcQueue(nullptr),  m_link_utilized(0),
      m_vc_load(p->virt_nets * p->vcs_per_vnet)
{
}

NetworkLink::~NetworkLink()
{
    delete linkBuffer;
}

void
NetworkLink::setLinkConsumer(FlexibleConsumer *consumer)
{
    link_consumer = consumer;
}

void
NetworkLink::setSourceQueue(flitBuffer *srcQueue)
{
    link_srcQueue = srcQueue;
}

void
NetworkLink::setSource(FlexibleConsumer *source)
{
    link_source = source;
}

void
NetworkLink::request_vc_link(int vc, NetDest destination, Cycles request_time)
{
    link_consumer->request_vc(vc, m_in_port, destination, request_time);
}

bool
NetworkLink::isBufferNotFull_link(int vc)
{
    return link_consumer->isBufferNotFull(vc, m_in_port);
}

void
NetworkLink::grant_vc_link(int vc, Cycles grant_time)
{
    link_source->grant_vc(m_out_port, vc, grant_time);
}

void
NetworkLink::release_vc_link(int vc, Cycles release_time)
{
    link_source->release_vc(m_out_port, vc, release_time);
}

bool
NetworkLink::isReady()
{
    return linkBuffer->isReady(curCycle());
}

void
NetworkLink::setInPort(int port)
{
    m_in_port = port;
}

void
NetworkLink::setOutPort(int port)
{
    m_out_port = port;
}

void
NetworkLink::wakeup()
{
    if (!link_srcQueue->isReady(curCycle()))
        return;

    flit *t_flit = link_srcQueue->getTopFlit();
    t_flit->set_time(curCycle() + m_latency);
    linkBuffer->insert(t_flit);
    link_consumer->scheduleEventAbsolute(clockEdge(m_latency));

    m_link_utilized++;
    m_vc_load[t_flit->get_vc()]++;
}

flit*
NetworkLink::peekLink()
{
    return linkBuffer->peekTopFlit();
}

flit*
NetworkLink::consumeLink()
{
    return linkBuffer->getTopFlit();
}

bool
NetworkLink::functionalRead(Packet *pkt)
{
    return linkBuffer->functionalRead(pkt);
}

uint32_t
NetworkLink::functionalWrite(Packet *pkt)
{
    return linkBuffer->functionalWrite(pkt);
}

NetworkLink *
NetworkLinkParams::create()
{
    return new NetworkLink(this);
}

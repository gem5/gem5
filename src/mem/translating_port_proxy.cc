/*
 * Copyright (c) 2011,2013 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

/**
 * @file
 * Port object definitions.
 */

#include "mem/translating_port_proxy.hh"

#include "base/chunk_generator.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "sim/system.hh"

TranslatingPortProxy::TranslatingPortProxy(
        ThreadContext *tc, Request::Flags _flags) :
    PortProxy(tc->getCpuPtr()->getSendFunctional(),
              tc->getSystemPtr()->cacheLineSize()), _tc(tc),
              pageBytes(tc->getSystemPtr()->getPageBytes()),
              flags(_flags)
{}

bool
TranslatingPortProxy::tryTLBsOnce(RequestPtr req, BaseTLB::Mode mode) const
{
    BaseTLB *dtb = _tc->getDTBPtr();
    BaseTLB *itb = _tc->getDTBPtr();
    return dtb->translateFunctional(req, _tc, mode) == NoFault ||
           itb->translateFunctional(req, _tc, BaseTLB::Read) == NoFault;
}

bool
TranslatingPortProxy::tryTLBs(RequestPtr req, BaseTLB::Mode mode) const
{
    // If at first this doesn't succeed, try to fixup and translate again. If
    // it still fails, report failure.
    return tryTLBsOnce(req, mode) ||
        (fixupAddr(req->getVaddr(), mode) && tryTLBsOnce(req, mode));
}

bool
TranslatingPortProxy::tryReadBlob(Addr addr, void *p, int size) const
{
    for (ChunkGenerator gen(addr, size, pageBytes); !gen.done();
         gen.next())
    {
        auto req = std::make_shared<Request>(
                gen.addr(), gen.size(), flags, Request::funcRequestorId, 0,
                _tc->contextId());

        if (!tryTLBs(req, BaseTLB::Read))
            return false;

        PortProxy::readBlobPhys(
                req->getPaddr(), req->getFlags(), p, gen.size());

        p = static_cast<uint8_t *>(p) + gen.size();
    }
    return true;
}

bool
TranslatingPortProxy::tryWriteBlob(
        Addr addr, const void *p, int size) const
{
    for (ChunkGenerator gen(addr, size, pageBytes); !gen.done();
         gen.next())
    {
        auto req = std::make_shared<Request>(
                gen.addr(), gen.size(), flags, Request::funcRequestorId, 0,
                _tc->contextId());

        if (!tryTLBs(req, BaseTLB::Write))
            return false;

        PortProxy::writeBlobPhys(
                req->getPaddr(), req->getFlags(), p, gen.size());
        p = static_cast<const uint8_t *>(p) + gen.size();
    }
    return true;
}

bool
TranslatingPortProxy::tryMemsetBlob(Addr address, uint8_t v, int size) const
{
    for (ChunkGenerator gen(address, size, pageBytes); !gen.done();
         gen.next())
    {
        auto req = std::make_shared<Request>(
                gen.addr(), gen.size(), flags, Request::funcRequestorId, 0,
                _tc->contextId());

        if (!tryTLBs(req, BaseTLB::Write))
            return false;

        PortProxy::memsetBlobPhys(
                req->getPaddr(), req->getFlags(), v, gen.size());
    }
    return true;
}

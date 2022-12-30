/*
 * Copyright (c) 2020 ARM Limited
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
 * Copyright 2020 Google Inc.
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

#ifndef __ARCH_GENERIC_ISA_HH__
#define __ARCH_GENERIC_ISA_HH__

#include <vector>

#include "arch/generic/pcstate.hh"
#include "cpu/reg_class.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ThreadContext;
class ExecContext;

class BaseISA : public SimObject
{
  public:
    typedef std::vector<const RegClass *> RegClasses;

  protected:
    using SimObject::SimObject;

    ThreadContext *tc = nullptr;

    RegClasses _regClasses;

  public:
    virtual PCStateBase *newPCState(Addr new_inst_addr=0) const = 0;
    virtual void clear() {}

    virtual RegVal readMiscRegNoEffect(RegIndex idx) const = 0;
    virtual RegVal readMiscReg(RegIndex idx) = 0;

    virtual void setMiscRegNoEffect(RegIndex idx, RegVal val) = 0;
    virtual void setMiscReg(RegIndex idx, RegVal val) = 0;

    virtual void takeOverFrom(ThreadContext *new_tc, ThreadContext *old_tc) {}
    virtual void setThreadContext(ThreadContext *_tc) { tc = _tc; }

    virtual uint64_t getExecutingAsid() const { return 0; }
    virtual bool inUserMode() const = 0;
    virtual void copyRegsFrom(ThreadContext *src) = 0;

    const RegClasses &regClasses() const { return _regClasses; }

    // Locked memory handling functions.
    virtual void handleLockedRead(const RequestPtr &req) {}
    virtual void
    handleLockedRead(ExecContext *xc, const RequestPtr &req)
    {
        handleLockedRead(req);
    }
    virtual bool
    handleLockedWrite(const RequestPtr &req, Addr cacheBlockMask)
    {
        return true;
    }
    virtual bool
    handleLockedWrite(ExecContext *xc, const RequestPtr &req,
            Addr cacheBlockMask)
    {
        return handleLockedWrite(req, cacheBlockMask);
    }

    virtual void handleLockedSnoop(PacketPtr pkt, Addr cacheBlockMask) {}
    virtual void
    handleLockedSnoop(ExecContext *xc, PacketPtr pkt, Addr cacheBlockMask)
    {
        handleLockedSnoop(pkt, cacheBlockMask);
    }
    virtual void handleLockedSnoopHit() {}
    virtual void
    handleLockedSnoopHit(ExecContext *xc)
    {
        handleLockedSnoopHit();
    }

    virtual void globalClearExclusive() {}
    virtual void
    globalClearExclusive(ExecContext *xc)
    {
        globalClearExclusive();
    }
};

} // namespace gem5

#endif // __ARCH_GENERIC_ISA_HH__

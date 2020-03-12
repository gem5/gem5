/*
 * Copyright (c) 2011 ARM Limited
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

#ifndef __MEM_TRANSLATING_PORT_PROXY_HH__
#define __MEM_TRANSLATING_PORT_PROXY_HH__

#include "arch/generic/tlb.hh"
#include "mem/port_proxy.hh"

class ThreadContext;

/**
 * This proxy attempts to translate virtual addresses using the TLBs. If it
 * fails, subclasses can override the fixupAddr virtual method to try to
 * recover, and then attempt the translation again. If it still fails then the
 * access as a whole fails.
 */
class TranslatingPortProxy : public PortProxy
{
  private:
    bool tryTLBsOnce(RequestPtr req, BaseTLB::Mode) const;
    bool tryTLBs(RequestPtr req, BaseTLB::Mode) const;

  protected:
    ThreadContext* _tc;
    const Addr pageBytes;

    Request::Flags flags;

    virtual bool
    fixupAddr(Addr addr, BaseTLB::Mode mode) const
    {
        return false;
    }

  public:

    TranslatingPortProxy(ThreadContext *tc, Request::Flags _flags=0);

    /** Version of tryReadblob that translates virt->phys and deals
      * with page boundries. */
    bool tryReadBlob(Addr addr, void *p, int size) const override;

    /** Version of tryWriteBlob that translates virt->phys and deals
      * with page boundries. */
    bool tryWriteBlob(Addr addr, const void *p, int size) const override;

    /**
     * Fill size bytes starting at addr with byte value val.
     */
    bool tryMemsetBlob(Addr address, uint8_t  v, int size) const override;
};

#endif //__MEM_TRANSLATING_PORT_PROXY_HH__

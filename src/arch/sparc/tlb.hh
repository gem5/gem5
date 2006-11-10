/*
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
 *
 * Authors: Ali Saidi
 */

#ifndef __ARCH_SPARC_TLB_HH__
#define __ARCH_SPARC_TLB_HH__

#include "base/misc.hh"
#include "mem/request.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

class ThreadContext;

namespace SparcISA
{
    class TLB : public SimObject
    {
      public:
        TLB(const std::string &name, int size) : SimObject(name)
        {
        }
    };

    class ITB : public TLB
    {
      public:
        ITB(const std::string &name, int size) : TLB(name, size)
        {
        }

        Fault translate(RequestPtr &req, ThreadContext *tc) const
        {
            //For now, always assume the address is already physical.
            //Also assume that there are 40 bits of physical address space.
            req->setPaddr(req->getVaddr() & ((1ULL << 40) - 1));
            return NoFault;
        }
    };

    class DTB : public TLB
    {
      public:
        DTB(const std::string &name, int size) : TLB(name, size)
        {
        }

        Fault translate(RequestPtr &req, ThreadContext *tc, bool write) const
        {
            //For now, always assume the address is already physical.
            //Also assume that there are 40 bits of physical address space.
            req->setPaddr(req->getVaddr() & ((1ULL << 40) - 1));
            return NoFault;
        }
    };
}

#endif // __ARCH_SPARC_TLB_HH__

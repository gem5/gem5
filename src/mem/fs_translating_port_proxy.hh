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
 *
 * Authors: Ali Saidi
 *          Andreas Hansson
 */

/**
 * @file
 * TranslatingPortProxy Object Declaration for FS.
 *
 * Port proxies are used when non structural entities need access to
 * the memory system. Proxy objects replace the previous
 * FunctionalPort, TranslatingPort and VirtualPort objects, which
 * provided the same functionality as the proxies, but were instances
 * of ports not corresponding to real structural ports of the
 * simulated system. Via the port proxies all the accesses go through
 * an actual port and thus are transparent to a potentially
 * distributed memory and automatically adhere to the memory map of
 * the system.
 */

#ifndef __MEM_FS_PORT_PROXY_HH__
#define __MEM_FS_PORT_PROXY_HH__

#include "mem/port_proxy.hh"

class ThreadContext;

/**
 * A TranslatingPortProxy in FS mode translates a virtual address to a
 * physical address and then calls the read/write functions of the
 * port. If a thread context is provided the address can alway be
 * translated, If not it can only be translated if it is a simple
 * address masking operation (such as alpha super page accesses).
 */
class FSTranslatingPortProxy : public PortProxy
{
  private:
    ThreadContext* _tc;

  public:

    FSTranslatingPortProxy(ThreadContext* tc);

    FSTranslatingPortProxy(MasterPort &port, unsigned int cacheLineSize);

    virtual ~FSTranslatingPortProxy();

    /** Version of readblob that translates virt->phys and deals
      * with page boundries. */
    virtual void readBlob(Addr addr, uint8_t *p, int size) const;

    /** Version of writeBlob that translates virt->phys and deals
      * with page boundries. */
    virtual void writeBlob(Addr addr, const uint8_t *p, int size) const;

    /**
     * Fill size bytes starting at addr with byte value val.
     */
    virtual void memsetBlob(Addr address, uint8_t  v, int size) const;
};

void CopyOut(ThreadContext *tc, void *dest, Addr src, size_t cplen);
void CopyIn(ThreadContext *tc, Addr dest, const void *source, size_t cplen);
void CopyStringOut(ThreadContext *tc, char *dst, Addr vaddr, size_t maxlen);
void CopyStringIn(ThreadContext *tc, const char *src, Addr vaddr);

#endif //__MEM_FS_PORT_PROXY_HH__

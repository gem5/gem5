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

/**
 * @file
 * Virtual Port Object Decleration. These ports incorporate some translation
 * into their access methods. Thus you can use one to read and write data
 * to/from virtual addresses.
 */

#ifndef __MEM_VPORT_HH__
#define __MEM_VPORT_HH__

#include "mem/port.hh"
#include "config/full_system.hh"
#include "arch/vtophys.hh"


/** A class that translates a virtual address to a physical address and then
 * calls the above read/write functions. If an execution context is provided the
 * address can alway be translated, If not it can only be translated if it is a
 * simple address masking operation (such as alpha super page accesses).
 */

class VirtualPort  : public FunctionalPort
{
  private:
    ExecContext *xc;

  public:
    VirtualPort(const std::string &_name, ExecContext *_xc = NULL)
        : FunctionalPort(_name), xc(_xc)
    {}

    /** Return true if we have an exec context. This is used to prevent someone
     * from accidently deleting the cpus statically allocated vport.
     * @return true if an execution context isn't valid
     */
    bool nullExecContext() { return xc != NULL; }

    /** Version of readblob that translates virt->phys and deals
      * with page boundries. */
    virtual void readBlob(Addr addr, uint8_t *p, int size);

    /** Version of writeBlob that translates virt->phys and deals
      * with page boundries. */
    virtual void writeBlob(Addr addr, uint8_t *p, int size);
};

#endif //__MEM_VPORT_HH__


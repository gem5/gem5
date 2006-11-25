/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include "arch/sparc/types.hh"
#include "cpu/o3/thread_context.hh"

template <class Impl>
class SparcTC : public O3ThreadContext<Impl>
{
  public:
#if FULL_SYSTEM
    /** Returns a pointer to the ITB. */
    virtual SparcISA::ITB *getITBPtr() { return this->cpu->itb; }

    /** Returns a pointer to the DTB. */
    virtual SparcISA::DTB *getDTBPtr() { return this->cpu->dtb; }

    /** Returns pointer to the quiesce event. */
    virtual EndQuiesceEvent *getQuiesceEvent()
    {
        return this->thread->quiesceEvent;
    }
#endif

    virtual uint64_t readNextNPC()
    {
        return this->cpu->readNextNPC(this->thread->readTid());
    }

    virtual void setNextNPC(uint64_t val)
    {
        this->cpu->setNextNPC(val, this->thread->readTid());
    }

    virtual void changeRegFileContext(TheISA::RegContextParam param,
                                      TheISA::RegContextVal val)
    {
        panic("This doesn't make sense!\n");
    }


    /** This function exits the thread context in the CPU and returns
     * 1 if the CPU has no more active threads (meaning it's OK to exit);
     * Used in syscall-emulation mode when a thread executes the 'exit'
     * syscall.
     */
    virtual int exit()
    {
        this->deallocate();

        // If there are still threads executing in the system
        if (this->cpu->numActiveThreads())
            return 0; // don't exit simulation
        else
            return 1; // exit simulation
    }
};

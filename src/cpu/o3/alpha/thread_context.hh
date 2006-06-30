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
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#include "cpu/o3/thread_context.hh"

template <class Impl>
class AlphaTC : public O3ThreadContext<Impl>
{
#if FULL_SYSTEM
    /** Returns a pointer to the ITB. */
    virtual AlphaITB *getITBPtr() { return cpu->itb; }

    /** Returns a pointer to the DTB. */
    virtual AlphaDTB *getDTBPtr() { return cpu->dtb; }

    /** Returns pointer to the quiesce event. */
    virtual EndQuiesceEvent *getQuiesceEvent()
    {
        return thread->quiesceEvent;
    }

    /** Returns if the thread is currently in PAL mode, based on
     * the PC's value. */
    virtual bool inPalMode()
    { return TheISA::PcPAL(cpu->readPC(thread->readTid())); }
#endif

    virtual uint64_t readNextNPC()
    {
        panic("Alpha has no NextNPC!");
        return 0;
    }

    virtual void setNextNPC(uint64_t val)
    {
        panic("Alpha has no NextNPC!");
    }

    virtual void changeRegFileContext(TheISA::RegFile::ContextParam param,
                                      TheISA::RegFile::ContextVal val)
    { panic("Not supported on Alpha!"); }
};

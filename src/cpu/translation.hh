/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2009 The University of Edinburgh
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
 *          Timothy M. Jones
 */

#ifndef __CPU_TRANSLATION_HH__
#define __CPU_TRANSLATION_HH__

#include "sim/tlb.hh"

class WholeTranslationState
{
  protected:
    int outstanding;
    Fault faults[2];

  public:
    bool isSplit;
    RequestPtr mainReq;
    RequestPtr sreqLow;
    RequestPtr sreqHigh;
    uint8_t *data;
    uint64_t *res;
    BaseTLB::Mode mode;

    /** Single translation state. */
    WholeTranslationState(RequestPtr _req, uint8_t *_data, uint64_t *_res,
                          BaseTLB::Mode _mode)
        : outstanding(1), isSplit(false), mainReq(_req), sreqLow(NULL),
          sreqHigh(NULL), data(_data), res(_res), mode(_mode)
    {
        faults[0] = faults[1] = NoFault;
        assert(mode == BaseTLB::Read || mode == BaseTLB::Write);
    }

    /** Split translation state. */
    WholeTranslationState(RequestPtr _req, RequestPtr _sreqLow,
                          RequestPtr _sreqHigh, uint8_t *_data, uint64_t *_res,
                          BaseTLB::Mode _mode)
        : outstanding(2), isSplit(true), mainReq(_req), sreqLow(_sreqLow),
          sreqHigh(_sreqHigh), data(_data), res(_res), mode(_mode)
    {
        faults[0] = faults[1] = NoFault;
        assert(mode == BaseTLB::Read || mode == BaseTLB::Write);
    }

    bool
    finish(Fault fault, int index)
    {
        assert(outstanding);
        faults[index] = fault;
        outstanding--;
        if (isSplit && outstanding == 0) {

            // For ease later, we copy some state to the main request.
            if (faults[0] == NoFault) {
                mainReq->setPaddr(sreqLow->getPaddr());
            }
            mainReq->setFlags(sreqLow->getFlags());
            mainReq->setFlags(sreqHigh->getFlags());
        }
        return outstanding == 0;
    }

    Fault
    getFault() const
    {
        if (!isSplit)
            return faults[0];
        else if (faults[0] != NoFault)
            return faults[0];
        else if (faults[1] != NoFault)
            return faults[1];
        else
            return NoFault;
    }

    void
    setNoFault()
    {
        faults[0] = faults[1] = NoFault;
    }

    bool
    isUncacheable() const
    {
        return mainReq->isUncacheable();
    }

    bool
    isPrefetch() const
    {
        return mainReq->isPrefetch();
    }

    Addr
    getPaddr() const
    {
        return mainReq->getPaddr();
    }

    unsigned
    getFlags()
    {
        return mainReq->getFlags();
    }

    void
    deleteReqs()
    {
        delete mainReq;
        if (isSplit) {
            delete sreqLow;
            delete sreqHigh;
        }
    }
};

template <class ExecContext>
class DataTranslation : public BaseTLB::Translation
{
  protected:
    ExecContext *xc;
    WholeTranslationState *state;
    int index;

  public:
    DataTranslation(ExecContext *_xc, WholeTranslationState* _state)
        : xc(_xc), state(_state), index(0)
    {
    }

    DataTranslation(ExecContext *_xc, WholeTranslationState* _state,
                    int _index)
        : xc(_xc), state(_state), index(_index)
    {
    }

    void
    finish(Fault fault, RequestPtr req, ThreadContext *tc,
           BaseTLB::Mode mode)
    {
        assert(state);
        assert(mode == state->mode);
        if (state->finish(fault, index)) {
            xc->finishTranslation(state);
        }
        delete this;
    }
};

#endif // __CPU_TRANSLATION_HH__

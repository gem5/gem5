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
 * Authors: Gabe Black
 */

#ifndef __ARCH_ARM_NATIVETRACE_HH__
#define __ARCH_ARM_NATIVETRACE_HH__

#include "base/types.hh"
#include "cpu/nativetrace.hh"
#include "params/ArmNativeTrace.hh"

namespace Trace {

class ArmNativeTrace : public NativeTrace
{
  public:
    enum StateID {
        STATE_R0,
        STATE_R1,
        STATE_R2,
        STATE_R3,
        STATE_R4,
        STATE_R5,
        STATE_R6,
        STATE_R7,
        STATE_R8,
        STATE_R9,
        STATE_R10,
        STATE_R11,
        STATE_FP = STATE_R11,
        STATE_R12,
        STATE_R13,
        STATE_SP = STATE_R13,
        STATE_R14,
        STATE_LR = STATE_R14,
        STATE_R15,
        STATE_PC = STATE_R15,
        STATE_CPSR,
        STATE_F0, STATE_F1, STATE_F2, STATE_F3, STATE_F4, STATE_F5, STATE_F6,
        STATE_F7, STATE_F8, STATE_F9, STATE_F10, STATE_F11, STATE_F12,
        STATE_F13, STATE_F14, STATE_F15, STATE_F16, STATE_F17, STATE_F18,
        STATE_F19, STATE_F20, STATE_F21, STATE_F22, STATE_F23, STATE_F24,
        STATE_F25, STATE_F26, STATE_F27, STATE_F28, STATE_F29, STATE_F30,
        STATE_F31, STATE_FPSCR,
        STATE_NUMVALS
    };

  protected:
    struct ThreadState {
        bool changed[STATE_NUMVALS];
        uint64_t state[2][STATE_NUMVALS];
        uint64_t *newState;
        uint64_t *oldState;
        int current;
        void update(NativeTrace *parent);
        void update(ThreadContext *tc);

        ThreadState()
        {
            for (int i = 0; i < STATE_NUMVALS; i++) {
                changed[i] = false;
                state[0][i] = state[1][i] = 0;
                current = 0;
                newState = state[0];
                oldState = state[1];
            }
        }
    };

    ThreadState nState, mState;

    bool stopOnPCError;

  public:
    typedef ArmNativeTraceParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    ArmNativeTrace(const Params *p) :
        NativeTrace(p), stopOnPCError(p->stop_on_pc_error)
    {}

    void check(NativeTraceRecord *record);
};

} // namespace Trace

#endif // __ARCH_ARM_NATIVETRACE_HH__

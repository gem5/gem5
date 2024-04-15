/*
 * Copyright (c) 2022-2023 The University of Edinburgh
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

/* @file
 * A helper for branch type information
 */

#ifndef __CPU_PRED_BRANCH_TYPE_HH__
#define __CPU_PRED_BRANCH_TYPE_HH__

#include "cpu/static_inst.hh"
#include "enums/BranchType.hh"

namespace gem5
{

namespace branch_prediction
{

typedef enums::BranchType BranchType;

inline BranchType
getBranchType(StaticInstPtr inst)
{
    if (inst->isReturn()) {
        return BranchType::Return;
    }

    if (inst->isCall()) {
        return inst->isDirectCtrl() ? BranchType::CallDirect :
                                      BranchType::CallIndirect;
    }

    if (inst->isDirectCtrl()) {
        return inst->isCondCtrl() ? BranchType::DirectCond :
                                    BranchType::DirectUncond;
    }

    if (inst->isIndirectCtrl()) {
        return inst->isCondCtrl() ? BranchType::IndirectCond :
                                    BranchType::IndirectUncond;
    }
    return BranchType::NoBranch;
}

inline std::string
toString(BranchType type)
{
    return std::string(enums::BranchTypeStrings[type]);
}

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_BRANCH_TYPE_HH__

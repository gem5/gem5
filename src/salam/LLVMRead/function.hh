/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef __SALAM_FUNCTION_HH__
#define __SALAM_FUNCTION_HH__

#include "basic_block.hh"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "value.hh"

namespace SALAM
{
class Function : public Value
{
  private:
    std::vector<std::shared_ptr<SALAM::BasicBlock>> bbList;
    std::vector<std::shared_ptr<SALAM::Value>> arguments;
    bool top;
    unsigned activeInstances = 0;
    unsigned instanceLimit = 1;

  public:
    Function(uint64_t id, gem5::SimObject *owner, bool dbg);
    using SALAM::Value::initialize;
    void initialize(llvm::Value *irval, irvmap *vmap,
                    SALAM::valueListTy *valueList, std::string topName);
    bool
    isTop()
    {
        return top;
    }
    void
    setTop(bool isTop)
    {
        top = isTop;
    }
    virtual bool
    isFunction()
    {
        return true;
    }
    std::vector<std::shared_ptr<SALAM::BasicBlock>> *
    getBBList()
    {
        return &bbList;
    }
    std::vector<std::shared_ptr<SALAM::Value>> *
    getArguments()
    {
        return &arguments;
    }
    std::shared_ptr<SALAM::BasicBlock>
    entry()
    {
        return bbList.front();
    }
    void
    addInstance()
    {
        activeInstances++;
    }
    void
    removeInstance()
    {
        if (activeInstances > 0) {
            activeInstances--;
        }
    }
    bool
    canLaunch()
    {
        return activeInstances < instanceLimit;
    }
};
} // namespace SALAM

#endif //__SALAM_FUNCTION_HH__

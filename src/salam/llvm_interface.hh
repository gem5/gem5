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

#ifndef __SALAM_LLVM_INTERFACE_HH__
#define __SALAM_LLVM_INTERFACE_HH__

// C++ Includes
#include <algorithm>
#include <chrono>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <ratio>
#include <type_traits>
#include <typeinfo>

// LLVM Includes
#include <llvm-c/Core.h>
#include <llvm/Analysis/LoopInfo.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Dominators.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Support/SourceMgr.h>
#include <llvm/Transforms/Utils/Cloning.h>

// SALAM Includes
#include "params/LLVMInterface.hh"
#include "salam/HWModeling/hw_interface.hh"
#include "salam/LLVMRead/basic_block.hh"
#include "salam/LLVMRead/debug_flags.hh"
#include "salam/LLVMRead/function.hh"
#include "salam/LLVMRead/operand.hh"
#include "salam/acc_compute_unit.hh"

class LLVMInterface : public AccComputeUnit
{
  private:
    std::string filename;
    std::string topName;
    uint32_t scheduling_threshold;
    int32_t clock_period;
    int cycle;
    int stalls;

    bool running;
    bool loadOpScheduled;
    bool storeOpScheduled;
    bool compOpScheduled;
    bool lockstep;
    bool dbg;
    std::chrono::duration<float> setupTime;
    std::chrono::duration<float> simTotal;
    std::chrono::duration<float> simTime;
    std::chrono::duration<float> schedulingTime;
    std::chrono::duration<float> queueProcessTime;
    std::chrono::duration<float> computeTime;
    std::chrono::duration<float> hwTime;
    std::chrono::high_resolution_clock::time_point simStop;
    std::chrono::high_resolution_clock::time_point setupStop;
    std::chrono::high_resolution_clock::time_point timeStart;

    class ActiveFunction
    {
        friend class LLVMInterface;

      private:
        LLVMInterface *owner;
        HWInterface *hw;
        std::shared_ptr<SALAM::Function> func;
        std::shared_ptr<SALAM::Instruction> caller;
        std::list<std::shared_ptr<SALAM::Instruction>> reservation;
        std::map<uint64_t, std::shared_ptr<SALAM::Instruction>> readQueue;
        std::map<MemoryRequest *, uint64_t> readQueueMap;
        std::map<uint64_t, std::shared_ptr<SALAM::Instruction>> writeQueue;
        std::map<MemoryRequest *, uint64_t> writeQueueMap;
        std::map<uint64_t, std::shared_ptr<SALAM::Instruction>> computeQueue;
        std::shared_ptr<SALAM::BasicBlock> previousBB;
        HW_Cycle_Stats hw_cycle_stats;
        uint32_t scheduling_threshold;
        bool returned = false;
        bool lockstep;
        bool dbg;

        inline bool
        uidActive(uint64_t id)
        {
            return computeUIDActive(id) || readUIDActive(id) ||
                   writeUIDActive(id);
        }

        std::map<Addr, std::shared_ptr<SALAM::Instruction>> activeWrites;
        inline void
        trackWrite(Addr writeAddr,
                   std::shared_ptr<SALAM::Instruction> writeInst)
        {
            activeWrites.insert({writeAddr, writeInst});
        }
        inline void
        untrackWrite(uint64_t writeAddr)
        {
            auto it = activeWrites.find(writeAddr);
            if (it != activeWrites.end()) {
                activeWrites.erase(it);
            }
        }
        inline bool
        writeActive(uint64_t writeAddr)
        {
            return (activeWrites.find(writeAddr) != activeWrites.end());
        }

        inline std::shared_ptr<SALAM::Instruction>
        getActiveWrite(uint64_t writeAddr)
        {
            return activeWrites.find(writeAddr)->second;
        }
        inline bool
        writeUIDActive(uint64_t uid)
        {
            return (writeQueue.find(uid) != writeQueue.end());
        }
        inline bool
        readUIDActive(uint64_t uid)
        {
            return (readQueue.find(uid) != readQueue.end());
        }
        inline bool
        computeUIDActive(uint64_t uid)
        {
            return (computeQueue.find(uid) != computeQueue.end());
        }

      public:
        ActiveFunction(LLVMInterface *_owner,
                       std::shared_ptr<SALAM::Function> _func,
                       std::shared_ptr<SALAM::Instruction> _caller)
            : owner(_owner), func(_func), caller(_caller), previousBB(nullptr)
        {
            scheduling_threshold = owner->getSchedulingThreshold();
            lockstep = (owner->getLockstepStatus());
            dbg = owner->debug();
        }
        void readCommit(MemoryRequest *req);
        void writeCommit(MemoryRequest *req);
        void findDynamicDeps(std::shared_ptr<SALAM::Instruction> inst);
        void scheduleBB(std::shared_ptr<SALAM::BasicBlock> bb);
        void processQueues();
        void launch();
        inline bool
        queuesClear()
        {
            return readQueue.empty() && writeQueue.empty() &&
                   computeQueue.empty();
        }
        inline bool
        lockstepReady()
        {
            return !lockstep || queuesClear();
        }
        inline bool
        canReturn()
        {
            return queuesClear() && reservation.front()->isReturn();
        }
        void launchRead(std::shared_ptr<SALAM::Instruction> readInst);
        void launchWrite(std::shared_ptr<SALAM::Instruction> writeInst);
        bool
        hasReturned()
        {
            return returned;
        }
    };

    std::list<ActiveFunction> activeFunctions;
    std::map<MemoryRequest *, ActiveFunction *> globalReadQueue;
    std::map<MemoryRequest *, ActiveFunction *> globalWriteQueue;

    std::vector<std::shared_ptr<SALAM::Function>> functions;
    std::vector<std::shared_ptr<SALAM::Value>> values;

  protected:
    virtual bool
    debug()
    {
        return comm->debug();
    }

  public:
    PARAMS(LLVMInterface);
    LLVMInterface(const LLVMInterfaceParams &p);
    void tick();
    void constructStaticGraph();
    void startup();
    void initialize();
    void finalize();
    void debug(uint64_t flags);
    bool
    getLockstepStatus()
    {
        return lockstep;
    }
    void readCommit(MemoryRequest *req);
    void writeCommit(MemoryRequest *req);
    void dumpModule(llvm::Module *m);
    void printResults();
    void launchFunction(std::shared_ptr<SALAM::Function> callee,
                        std::shared_ptr<SALAM::Instruction> caller);
    void launchTopFunction();
    void endFunction(ActiveFunction *afunc);
    void launchRead(MemoryRequest *memReq, ActiveFunction *func);
    void launchWrite(MemoryRequest *memReq, ActiveFunction *func);
    std::shared_ptr<SALAM::Instruction>
    createInstruction(llvm::Instruction *inst, uint64_t id);
    void dumpQueues();
    uint32_t
    getSchedulingThreshold()
    {
        return scheduling_threshold;
    }
    void
    addSchedulingTime(std::chrono::duration<float> timeDelta)
    {
        schedulingTime = schedulingTime + timeDelta;
    }
    void
    addQueueTime(std::chrono::duration<float> timeDelta)
    {
        queueProcessTime = queueProcessTime + timeDelta;
    }
    void
    addComputeTime(std::chrono::duration<float> timeDelta)
    {
        computeTime = computeTime + timeDelta;
    }
    void
    addHWTime(std::chrono::duration<float> timeDelta)
    {
        hwTime = hwTime + timeDelta;
    }
};

#endif //__SALAM_LLVM_INTERFACE_HH__

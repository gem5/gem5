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

#include "salam/llvm_interface.hh"

#include <functional>
#include <set>

#include "debug/SALAMFUInventory.hh"
#include "debug/SALAMIssue.hh"
#include "debug/SALAMLaunch.hh"
#include "llvm/IR/DataLayout.h"
#include "llvm/Support/raw_ostream.h"

namespace
{

constexpr const char *OpClassNames[] = {"compute", "load", "store", "call",
                                        "control"};

} // anonymous namespace

LLVMInterface::LLVMInterface(const LLVMInterfaceParams &p)
    : AccComputeUnit(p),
      filename(p.in_file),
      topName(p.top_name),
      scheduling_threshold(p.sched_threshold),
      lockstep(p.lockstep_mode),
      execStats(this)
{
    dbg = comm->debug();
}

LLVMInterface::ExecStats::ExecStats(LLVMInterface *owner)
    : statistics::Group(owner),
      ADD_STAT(invocations, statistics::units::Count::get(),
               "Accelerator invocations started"),
      ADD_STAT(cycles, statistics::units::Cycle::get(),
               "Compute passes while running"),
      ADD_STAT(cycleCategories, statistics::units::Cycle::get(),
               "Compute passes by what they did: issued an operation, had "
               "one denied a functional unit, waited for memory, waited "
               "for compute in flight, or none of these"),
      ADD_STAT(issued, statistics::units::Count::get(),
               "Operations issued, by class"),
      ADD_STAT(completed, statistics::units::Count::get(),
               "Operations completed, by class"),
      ADD_STAT(opcodesIssued, statistics::units::Count::get(),
               "Operations issued, by LLVM opcode"),
      ADD_STAT(fuDenials, statistics::units::Count::get(),
               "Times a ready operation waited for a functional unit"),
      ADD_STAT(
          memoryOrderWaits, statistics::units::Count::get(),
          "Ready load/store scheduling observations blocked by older access"),
      ADD_STAT(reservedOps, statistics::units::Count::get(),
               "Time-weighted average of operations waiting to issue"),
      ADD_STAT(computeOps, statistics::units::Count::get(),
               "Time-weighted average of compute operations in flight"),
      ADD_STAT(readOps, statistics::units::Count::get(),
               "Time-weighted average of loads in flight"),
      ADD_STAT(writeOps, statistics::units::Count::get(),
               "Time-weighted average of stores in flight")
{}

void
LLVMInterface::ExecStats::regStats()
{
    statistics::Group::regStats();
    static const char *categories[NumCycleCategories] = {
        "issue", "fuStall", "memoryWait", "computeWait", "other"};
    cycleCategories.init(NumCycleCategories);
    for (int i = 0; i < NumCycleCategories; i++) {
        cycleCategories.subname(i, categories[i]);
    }
    for (auto *vector : {&issued, &completed}) {
        vector->init(NumOpClasses);
        for (int i = 0; i < NumOpClasses; i++) {
            vector->subname(i, OpClassNames[i]);
        }
    }
    opcodesIssued.init(llvm::Instruction::OtherOpsEnd)
        .flags(statistics::nozero);
    for (unsigned op = 1; op < llvm::Instruction::OtherOpsEnd; op++) {
        opcodesIssued.subname(op, llvm::Instruction::getOpcodeName(op));
    }
}

void
LLVMInterface::countCycleCategory()
{
    bool memory = pass.memoryOrderWait;
    bool compute = false;
    for (auto &function : activeFunctions) {
        memory = memory || !function.readQueue.empty() ||
                 !function.writeQueue.empty();
        compute = compute || !function.computeQueue.empty();
    }
    CycleCategory category = pass.issued     ? IssueCycle
                             : pass.fuDenied ? FuStallCycle
                             : memory        ? MemoryWaitCycle
                             : compute       ? ComputeWaitCycle
                                             : OtherCycle;
    execStats.cycleCategories[category]++;
    totals.cycleCategories[category]++;
    pass = PassEvents();
}

void
LLVMInterface::countIssue(OpClass cls, uint64_t opcode)
{
    DPRINTF(SALAMIssue, "issue %s %s\n", OpClassNames[cls],
            opcode < llvm::Instruction::OtherOpsEnd
                ? llvm::Instruction::getOpcodeName(opcode)
                : "?");
    pass.issued = true;
    execStats.issued[cls]++;
    totals.issued[cls]++;
    if (opcode < llvm::Instruction::OtherOpsEnd) {
        execStats.opcodesIssued[opcode]++;
    }
}

void
LLVMInterface::countCompletion(OpClass cls)
{
    DPRINTF(SALAMIssue, "complete %s\n", OpClassNames[cls]);
    execStats.completed[cls]++;
}

void
LLVMInterface::countFuDenial()
{
    DPRINTF(SALAMIssue, "wait: no functional unit\n");
    pass.fuDenied = true;
    execStats.fuDenials++;
    totals.fuDenials++;
}

void
LLVMInterface::countMemoryOrderWait()
{
    DPRINTF(SALAMIssue, "wait: older memory access\n");
    pass.memoryOrderWait = true;
    execStats.memoryOrderWaits++;
}

void
LLVMInterface::sampleOccupancy()
{
    // Averages integrate until the next assignment. Also update at
    // launches and memory callbacks, which can run between compute passes.
    size_t reserved = 0;
    size_t compute = 0;
    size_t reads = 0;
    size_t writes = 0;
    for (auto &function : activeFunctions) {
        reserved += function.reservation.size();
        compute += function.computeQueue.size();
        reads += function.readQueue.size();
        writes += function.writeQueue.size();
    }
    execStats.reservedOps = reserved;
    execStats.computeOps = compute;
    execStats.readOps = reads;
    execStats.writeOps = writes;
}

std::shared_ptr<SALAM::Value>
createClone(const std::shared_ptr<SALAM::Value> &b)
{
    std::shared_ptr<SALAM::Value> clone = b->clone();
    return clone;
}

void
LLVMInterface::ActiveFunction::scheduleBB(
    std::shared_ptr<SALAM::BasicBlock> bb)
{
    auto schedulingStart = std::chrono::high_resolution_clock::now();
    if (dbg) {
        DPRINTFS(Runtime, owner, "|---[Schedule BB - UID:%i ]\n",
                 bb->getUID());
    }
    bool needToScheduleBranch = false;
    std::shared_ptr<SALAM::BasicBlock> nextBB;
    auto instruction_list = *(bb->Instructions());
    // The phis at the start of a block read their incoming values as of
    // block entry. Resolve all of them before any is queued, so that one
    // phi never reads another phi's new value.
    std::vector<std::shared_ptr<SALAM::Instruction>> entryPhis;
    for (auto inst : instruction_list) {
        if (!entryPhis.empty() && !inst->isPhi()) {
            for (auto phi : entryPhis) {
                reservation.push_back(phi);
            }
            entryPhis.clear();
        }
        std::shared_ptr<SALAM::Instruction> clone_inst = inst->clone();
        if (dbg) {
            DPRINTFS(Runtime, owner, "\t\t Instruction Cloned [UID: %d] \n",
                     inst->getUID());
        }
        if (clone_inst->isBr()) {
            if (dbg) {
                DPRINTFS(Runtime, owner, "\t\t Branch Instruction Found\n");
            }
            auto branch = std::dynamic_pointer_cast<SALAM::Br>(clone_inst);
            if (branch && !(branch->isConditional())) {
                if (dbg) {
                    DPRINTFS(
                        Runtime, owner,
                        "\t\t Unconditional Branch, Scheduling Next BB\n");
                }
                nextBB = branch->getTarget();
                if (dbg) {
                    DPRINTFS(RuntimeCompute, owner,
                             "\t\t Branching to %s from %s\n",
                             nextBB->getIRStub(), bb->getIRStub());
                }
                needToScheduleBranch = true;
            } else {
                findDynamicDeps(clone_inst);
                reservation.push_back(clone_inst);
            }
        } else {
            if (clone_inst->isPhi()) {
                if (dbg) {
                    DPRINTFS(Runtime, owner, "\t\t Phi Instruction Found\n");
                }
                auto phi = std::dynamic_pointer_cast<SALAM::Phi>(clone_inst);
                if (phi) {
                    phi->setPrevBB(previousBB);
                }
                findDynamicDeps(clone_inst);
                entryPhis.push_back(clone_inst);
                continue;
            }
            findDynamicDeps(clone_inst);
            reservation.push_back(clone_inst);
        }
    }
    for (auto phi : entryPhis) {
        reservation.push_back(phi);
    }
    previousBB = bb;
    auto schedulingStop = std::chrono::high_resolution_clock::now();
    owner->addSchedulingTime(schedulingStop - schedulingStart);
    if (needToScheduleBranch) {
        scheduleBB(nextBB);
    }
}

void
LLVMInterface::ActiveFunction::processQueues()
{
    auto queueStart = std::chrono::high_resolution_clock::now();

    if (dbg) {
        DPRINTFS(Runtime, owner, "\t\t  |-[Process Queues]--------\n");
        DPRINTFS(RuntimeQueues, owner,
                 "\t\t[Runtime Queue] Reservation:%d, Compute:%d, Read:%d, "
                 "Write:%d\n",
                 reservation.size(), computeQueue.size(), readQueue.size(),
                 writeQueue.size());
    }
    // First pass, computeQueue is empty
    for (auto queue_iter = computeQueue.begin();
         queue_iter != computeQueue.end();) {
        if (dbg) {
            DPRINTFS(Runtime, owner, "\n\t\t %s \n\t\t %s%s%s%d%s \n",
                     " |-[Compute Queue]--------------", " | Instruction: ",
                     llvm::Instruction::getOpcodeName(
                         (queue_iter->second)->getOpode()),
                     " | UID[", (queue_iter->first), "]");
        }

        if ((queue_iter->second)->commit()) {
            auto token = fuTokens.find(queue_iter->first);
            if (token != fuTokens.end()) {
                owner->hw->fuPool.release(token->second);
                owner->hw->fuReleased(token->second.unit);
                fuTokens.erase(token);
            }
            owner->countCompletion(queue_iter->second->isCall() ? CallOp
                                                                : ComputeOp);
            (queue_iter->second)->reset();
            queue_iter = computeQueue.erase(queue_iter);
        } else {
            ++queue_iter;
        }
    }
    if (canReturn()) {
        owner->countIssue(ControlOp, reservation.front()->getOpode());
        owner->countCompletion(ControlOp);
        // Handle function return
        if (dbg) {
            DPRINTFS(Runtime, owner, "[[Function Return]]\n\n");
        }
        if (caller != nullptr) {
            // Pass the return value; the call completes from the
            // caller's compute queue, once, after this.
            if (caller->getSize() > 0) {
                auto retInst = reservation.front();
                auto retOperand = retInst->getOperands()->front();
                caller->setRegisterValue(retOperand.getOpRegister());
            }
            func->removeInstance();
            auto callInst = std::dynamic_pointer_cast<SALAM::Call>(caller);
            assert(callInst);
            callInst->calleeReturn();
        }
        returned = true;
        return;
    } else if (lockstepReady()) {
        for (auto queue_iter = reservation.begin();
             queue_iter != reservation.end();) {
            if (owner->debug()) {
                if (dbg) {
                    DPRINTFS(Runtime, owner, "Debug Breakpoint");
                }
            }
            auto inst = *queue_iter;
            SALAM::FUPool::Token token;
            if (dbg) {
                DPRINTFS(Runtime, owner, "\n\t\t %s \n\t\t %s%s%s%d%s \n",
                         " |-[Reserve Queue]--------------",
                         " | Instruction: ",
                         llvm::Instruction::getOpcodeName((inst)->getOpode()),
                         " | UID[", (inst)->getUID(), "]");
            }
            if (!(inst)->isReturn()) {
                if ((inst)->isTerminator() &&
                    reservation.size() >= scheduling_threshold) {
                    ++queue_iter;
                } else if (((inst)->ready()) && !uidActive((inst)->getUID())) {
                    if ((inst)->isLoad()) {
                        // RAW protection to ensure a writeback finishes
                        // before reading that location
                        if (mustWaitForOlder(queue_iter)) {
                            ++queue_iter;
                            owner->countMemoryOrderWait();
                        } else if (inst->isLoadingInternal() ||
                                   !writeActive(inst->getPtrOperandValue(0),
                                                inst->getSizeInBytes())) {
                            launchRead(inst);
                            if (dbg) {
                                DPRINTFS(
                                    Runtime, owner,
                                    "\t\t  |-Erase From Queue: %s - UID[%i]\n",
                                    llvm::Instruction::getOpcodeName(
                                        (*queue_iter)->getOpode()),
                                    (*queue_iter)->getUID());
                            }
                            queue_iter = reservation.erase(queue_iter);
                        } else {
                            auto activeWrite =
                                getActiveWrite(inst->getPtrOperandValue(0),
                                               inst->getSizeInBytes());
                            inst->addRuntimeDependency(activeWrite);
                            activeWrite->addRuntimeUser(inst);
                            owner->countMemoryOrderWait();
                            ++queue_iter;
                        }
                    } else if ((inst)->isStore() &&
                               mustWaitForOlder(queue_iter)) {
                        ++queue_iter;
                        owner->countMemoryOrderWait();
                    } else if ((inst)->isStore()) {
                        launchWrite(inst);
                        if (dbg) {
                            DPRINTFS(
                                Runtime, owner,
                                "\t\t  |-Erase From Queue: %s - UID[%i]\n",
                                llvm::Instruction::getOpcodeName(
                                    (*queue_iter)->getOpode()),
                                (*queue_iter)->getUID());
                        }
                        queue_iter = reservation.erase(queue_iter);
                    } else if ((inst)->isLatchingBrExiting() &&
                               ((reservation.size() > 1) || !queuesClear())) {
                        ++queue_iter;
                    } else if ((inst)->isTerminator()) {
                        owner->countIssue(ControlOp, inst->getOpode());
                        (inst)->launch();
                        auto nextBB = inst->getTarget();
                        if (dbg) {
                            DPRINTFS(RuntimeCompute, owner,
                                     "\t\t Branching to %s from %s\n",
                                     nextBB->getIRStub(),
                                     previousBB->getIRStub());
                        }
                        scheduleBB(nextBB);
                        if (dbg) {
                            DPRINTFS(
                                Runtime, owner,
                                "\t\t  | Branch Scheduled: %s - UID[%i]\n",
                                llvm::Instruction::getOpcodeName(
                                    (inst)->getOpode()),
                                (inst)->getUID());
                        }
                        (inst)->commit();
                        owner->countCompletion(ControlOp);
                        if (dbg) {
                            DPRINTFS(
                                Runtime, owner,
                                "\t\t  |-Erase From Queue: %s - UID[%i]\n",
                                llvm::Instruction::getOpcodeName(
                                    (*queue_iter)->getOpode()),
                                (*queue_iter)->getUID());
                        }
                        queue_iter = reservation.erase(queue_iter);
                    } else if ((*queue_iter)->isCall()) {
                        auto callInst =
                            std::dynamic_pointer_cast<SALAM::Call>(inst);
                        assert(callInst);
                        auto calleeValue = callInst->getCalleeValue();
                        auto callee =
                            std::dynamic_pointer_cast<SALAM::Function>(
                                calleeValue);
                        assert(callee);
                        if (callee->canLaunch() &&
                            !mustWaitForOlder(queue_iter)) {
                            owner->countIssue(CallOp, inst->getOpode());
                            owner->launchFunction(callee, callInst);
                            computeQueue.insert({(inst)->getUID(), inst});
                            if (dbg) {
                                DPRINTFS(
                                    Runtime, owner,
                                    "\t\t  |-Erase From Queue: %s - UID[%i]\n",
                                    llvm::Instruction::getOpcodeName(
                                        (*queue_iter)->getOpode()),
                                    (*queue_iter)->getUID());
                            }
                            queue_iter = reservation.erase(queue_iter);
                        } else {
                            ++queue_iter;
                        }
                    } else if (!admitToUnit(inst, token)) {
                        ++queue_iter;
                    } else {
                        auto computeStart =
                            std::chrono::high_resolution_clock::now();
                        owner->countIssue(ComputeOp, inst->getOpode());
                        if ((inst)->launch()) {
                            // Completed at once: release its unit now.
                            owner->countCompletion(ComputeOp);
                            if (token.id != 0) {
                                owner->hw->fuPool.release(token);
                                owner->hw->fuReleased(token.unit);
                            }
                        } else {
                            if (token.id != 0) {
                                fuTokens.emplace((inst)->getUID(), token);
                            }
                            if (dbg) {
                                DPRINTFS(Runtime, owner,
                                         "\t\t  | Added to Comp Queue: %s - "
                                         "UID[%i]\n",
                                         llvm::Instruction::getOpcodeName(
                                             (inst)->getOpode()),
                                         (inst)->getUID());
                            }
                            computeQueue.insert({(inst)->getUID(), inst});
                        }
                        auto computeStop =
                            std::chrono::high_resolution_clock::now();
                        owner->addComputeTime(computeStop - computeStart);
                        if (dbg) {
                            DPRINTFS(
                                Runtime, owner,
                                "\t\t  | Erase From Queue: %s - UID[%i]\n",
                                llvm::Instruction::getOpcodeName(
                                    (*queue_iter)->getOpode()),
                                (*queue_iter)->getUID());
                        }
                        queue_iter = reservation.erase(queue_iter);
                    }
                } else {
                    ++queue_iter;
                }
            } else {
                ++queue_iter;
            }
        }
    }

    auto queueStop = std::chrono::high_resolution_clock::now();
    owner->addQueueTime(queueStop - queueStart);
}

/*****************************************************************************
 CN Scheduling

 As CNs are scheduled they are added to an in-flight queue depending on
 operation type.
 Loads and Stores are maintained in separate queues, and are committed
 by the comm_interface.
 Branch and phi instructions evaluate and commit immediately.
 All other CN types are added to an in-flight compute queue.

 Each tick we must first check our in-flight compute queue.
 Each node should have its cycle count incremented,
 and should commit if max cycle is reached.

 New CNs are added to the reservation table whenever a new BB is encountered.
 This may occur during device init, or when a br op commits.
 For each CN in a BB we reset the CN, evaluate if it is a phi or uncond br,
 and add it to our reservation table otherwise.
*****************************************************************************/

void
LLVMInterface::tick()
{
    auto tickStart = std::chrono::high_resolution_clock::now();

    if (dbg) {
        DPRINTF(LLVMInterface, "\n%s\n%s %d\n%s\n",
                "*************************************************************"
                "*****",
                "   Cycle", cycle,
                "*************************************************************"
                "*****");
    }
    cycle++;
    lastPass = curTick();
    execStats.cycles++;
    totals.cycles++;

    // Process Queues in Active Functions
    for (auto func_iter = activeFunctions.begin();
         func_iter != activeFunctions.end();) {
        func_iter->processQueues();
        if (!(func_iter->hasReturned())) {
            func_iter++;
        } else {
            func_iter = activeFunctions.erase(func_iter);
        }
    }
    countCycleCategory();
    sampleOccupancy();
    if (activeFunctions.empty()) {
        // We are finished executing all functions.
        // Signal completion to the CommInterface
        running = false;
        finalize();
        return;
    }
    //////////////// Schedule Next Cycle ////////////////////////
    if (running && !tickEvent.scheduled()) {
        schedule(tickEvent, clockEdge(Cycles(1)));
    }
    auto tickStop = std::chrono::high_resolution_clock::now();
    simTime = simTime + (tickStop - tickStart);
}

/*****************************************************************************
- findDynamicDeps(std::list<std::shared_ptr<SALAM::Instructions>,
  std::shared_ptr<SALAM::Instruction>)
- only parse queue once for each instruction until all dependencies are found
- include self in dependency list
- Register dynamicUser/dynamicDependencies
  std::deque<std::shared_ptr<SALAM::Instructon> >
*****************************************************************************/
void // Add third argument, previous BB
LLVMInterface::ActiveFunction::findDynamicDeps(
    std::shared_ptr<SALAM::Instruction> inst)
{
    if (dbg) {
        DPRINTFS(Runtime, owner, "Linking Dynamic Dependencies [%s]\n",
                 llvm::Instruction::getOpcodeName(inst->getOpode()));
    }
    // The list of UIDs for any dependencies we want to find
    std::vector<uint64_t> dep_uids = inst->runtimeInitialize();
    // Find dependencies currently in queues
    // Reverse search the reservation queue because we want to link
    // only the last instance of each dep
    auto queue_iter = reservation.rbegin();
    while ((queue_iter != reservation.rend()) && !dep_uids.empty()) {
        auto queued_inst = *queue_iter;
        // Look at each instruction in runtime queue once
        for (auto dep_it = dep_uids.begin(); dep_it != dep_uids.end();) {
            // Check if any of the instruction to be scheduled dependencies
            // match the current instruction from queue
            if (queued_inst->getUID() == *dep_it) {
                // If dependency found, create two way link
                inst->addRuntimeDependency(queued_inst);
                queued_inst->addRuntimeUser(inst);
                dep_it = dep_uids.erase(dep_it);
            } else {
                dep_it++;
            }
        }
        queue_iter++;
    }

    // The other queues do not need to be reverse-searched since only 1
    // instance of any instruction can exist in them
    // Check the compute queue
    for (auto dep_it = dep_uids.begin(); dep_it != dep_uids.end();) {
        auto queue_iter = computeQueue.find(*dep_it);
        if (queue_iter != computeQueue.end()) {
            auto queued_inst = queue_iter->second;
            inst->addRuntimeDependency(queued_inst);
            queued_inst->addRuntimeUser(inst);
            dep_it = dep_uids.erase(dep_it);
        } else {
            dep_it++;
        }
    }
    // Check the memory read queue
    for (auto dep_it = dep_uids.begin(); dep_it != dep_uids.end();) {
        auto queue_iter = readQueue.find(*dep_it);
        if (queue_iter != readQueue.end()) {
            auto queued_inst = queue_iter->second;
            inst->addRuntimeDependency(queued_inst);
            queued_inst->addRuntimeUser(inst);
            dep_it = dep_uids.erase(dep_it);
        } else {
            dep_it++;
        }
    }

    // Check the memory write queue
    for (auto dep_it = dep_uids.begin(); dep_it != dep_uids.end();) {
        auto queue_iter = writeQueue.find(*dep_it);
        if (queue_iter != writeQueue.end()) {
            auto queued_inst = queue_iter->second;
            inst->addRuntimeDependency(queued_inst);
            queued_inst->addRuntimeUser(inst);
            dep_it = dep_uids.erase(dep_it);
        } else {
            dep_it++;
        }
    }

    // Fetch values for resolved dependencies,
    // static elements, and immediate values
    if (!dep_uids.empty()) {
        for (auto resolved : dep_uids) {
            // If this dependency exists, then lock value into operand
            inst->setOperandValue(resolved);
        }
    }
}

void
LLVMInterface::dumpModule(llvm::Module *M)
{
    M->print(llvm::outs(), nullptr);
    for (const llvm::Function &F : *M) {
        for (const llvm::BasicBlock &BB : F) {
            for (const llvm::Instruction &I : BB) {
                I.print(llvm::outs());
            }
        }
    }
}

namespace
{

// Functions have one register frame and one active instance. A cycle in
// the reachable direct-call graph cannot execute with that model.
void
rejectRecursiveCalls(llvm::Function &entry)
{
    std::set<const llvm::Function *> active;
    std::set<const llvm::Function *> complete;
    std::function<void(llvm::Function &)> visit = [&](llvm::Function &func) {
        active.insert(&func);
        for (auto &bb : func) {
            for (auto &inst : bb) {
                auto *call = llvm::dyn_cast<llvm::CallInst>(&inst);
                auto *callee = call ? call->getCalledFunction() : nullptr;
                if (!callee || callee->isDeclaration()) {
                    continue;
                }
                fatal_if(active.count(callee),
                         "SALAM does not support recursive LLVM calls: "
                         "%s calls %s",
                         func.getName().str(), callee->getName().str());
                if (!complete.count(callee)) {
                    visit(*callee);
                }
            }
        }
        active.erase(&func);
        complete.insert(&func);
    };
    visit(entry);
}

} // anonymous namespace

void
LLVMInterface::constructStaticGraph()
{
    /************************************************************************
     Constructing the Static CDFG

     Parses LLVM file and creates the CDFG passed to runtime simulation
     engine.
    ************************************************************************/
    auto parseStart = std::chrono::high_resolution_clock::now();
    if (dbg) {
        DPRINTF(LLVMInterface, "Constructing Static Dependency Graph\n");
    }
    llvm::StringRef file = filename;
    std::unique_ptr<llvm::LLVMContext> context(new llvm::LLVMContext());
    std::unique_ptr<llvm::SMDiagnostic> error(new llvm::SMDiagnostic());
    std::unique_ptr<llvm::Module> m;
    std::unique_ptr<llvm::DominatorTree> dt(new llvm::DominatorTree());
    std::unique_ptr<llvm::LoopInfoBase<llvm::BasicBlock, llvm::Loop>> loopInfo(
        new llvm::LoopInfoBase<llvm::BasicBlock, llvm::Loop>());

    m = llvm::parseIRFile(file, *error, *context);
    if (!m) {
        panic("Error reading LLVM module '%s': %s", filename,
              error->getMessage().str());
    }

    auto *entry = m->getFunction(topName);
    if (m->size() == 1) {
        entry = &*m->begin();
    }
    if (entry) {
        rejectRecursiveCalls(*entry);
    }

    // Construct the LLVM::Value to SALAM::Value map
    uint64_t valueID = 0;
    SALAM::irvmap vmap;
    // Generate SALAM::Values for llvm::GlobalVariables
    DPRINTF(LLVMParse, "Instantiate SALAM::GlobalConstants\n");
    for (auto glob_iter = m->global_begin(); glob_iter != m->global_end();
         glob_iter++) {
        llvm::GlobalVariable &glb = *glob_iter;
        std::shared_ptr<SALAM::GlobalConstant> sglb =
            std::make_shared<SALAM::GlobalConstant>(valueID, this, debug());
        values.push_back(sglb);
        vmap.insert(SALAM::irvmaptype(&glb, sglb));
        valueID++;
    }
    // Generate SALAM::Functions
    DPRINTF(LLVMParse, "Instantiate SALAM::Functions\n");
    for (auto func_iter = m->begin(); func_iter != m->end(); func_iter++) {
        llvm::Function &func = *func_iter;
        std::shared_ptr<SALAM::Function> sfunc =
            std::make_shared<SALAM::Function>(valueID, this, debug());
        values.push_back(sfunc);
        functions.push_back(sfunc);
        vmap.insert(SALAM::irvmaptype(&func, sfunc));
        valueID++;
        // Generate args for SALAM:Functions
        DPRINTF(LLVMParse, "Instantiate SALAM::Functions::Arguments\n");
        for (auto arg_iter = func.arg_begin(); arg_iter != func.arg_end();
             arg_iter++) {
            llvm::Argument &arg = *arg_iter;
            std::shared_ptr<SALAM::Argument> sarg =
                std::make_shared<SALAM::Argument>(valueID, this, debug());
            values.push_back(sarg);
            vmap.insert(SALAM::irvmaptype(&arg, sarg));
            valueID++;
        }
        // Generate SALAM::BasicBlocks
        DPRINTF(LLVMParse, "Instantiate SALAM::Functions::BasicBlocks\n");
        for (auto bb_iter = func.begin(); bb_iter != func.end(); bb_iter++) {
            llvm::BasicBlock &bb = *bb_iter;
            std::shared_ptr<SALAM::BasicBlock> sbb =
                std::make_shared<SALAM::BasicBlock>(valueID, this, debug());
            values.push_back(sbb);
            vmap.insert(SALAM::irvmaptype(&bb, sbb));
            valueID++;
            // Generate SALAM::Instructions
            DPRINTF(
                LLVMParse,
                "Instantiate SALAM::Functions::BasicBlocks::Instructions\n");
            for (auto inst_iter = bb.begin(); inst_iter != bb.end();
                 inst_iter++) {
                llvm::Instruction &inst = *inst_iter;
                std::shared_ptr<SALAM::Instruction> sinst =
                    createInstruction(&inst, valueID);
                values.push_back(sinst);
                vmap.insert(SALAM::irvmaptype(&inst, sinst));
                valueID++;
            }
        }
    }

    // Use value map to initialize SALAM::Values
    DPRINTF(LLVMParse, "Initialize SALAM::GlobalConstants\n");
    for (auto glob_iter = m->global_begin(); glob_iter != m->global_end();
         glob_iter++) {
        llvm::GlobalVariable &glb = *glob_iter;
        std::shared_ptr<SALAM::Value> glbval = vmap.find(&glb)->second;
        assert(glbval);
        std::shared_ptr<SALAM::GlobalConstant> sglb =
            std::dynamic_pointer_cast<SALAM::GlobalConstant>(glbval);
        assert(sglb);
        sglb->initialize(&glb, &vmap, &values);
    }
    // Functions initialize BasicBlocks, which will initialize Instructions
    DPRINTF(LLVMParse, "Initialize SALAM::Functions\n");
    for (auto func_iter = m->begin(); func_iter != m->end(); func_iter++) {
        llvm::Function &func = *func_iter;
        std::shared_ptr<SALAM::Value> funcval = vmap.find(&func)->second;
        assert(funcval);
        std::shared_ptr<SALAM::Function> sfunc =
            std::dynamic_pointer_cast<SALAM::Function>(funcval);
        assert(sfunc);
        sfunc->initialize(&func, &vmap, &values, topName);
    }
    if (functions.size() == 1) {
        functions.front()->setTop(true);
    }

    const auto &layout = m->getDataLayout();
    auto typeName = [&layout](llvm::Type *type) -> std::string {
        if (type->isVoidTy()) {
            return "v";
        }
        if (type->isPointerTy()) {
            return "p" + std::to_string(layout.getPointerTypeSizeInBits(type));
        }
        if (type->isIntegerTy()) {
            return "i" + std::to_string(type->getIntegerBitWidth());
        }
        if (type->isFloatTy() || type->isDoubleTy()) {
            return "f" + std::to_string(type->getScalarSizeInBits());
        }
        return "unavailable";
    };
    for (const auto &entry : vmap) {
        entry.second->setStorageType(typeName(entry.first->getType()));
        if (auto *instruction =
                llvm::dyn_cast<llvm::Instruction>(entry.first)) {
            std::string signature;
            for (const auto &operand : instruction->operands()) {
                if (!signature.empty()) {
                    signature += ',';
                }
                signature += typeName(operand->getType());
            }
            signature += "->" + typeName(instruction->getType());
            std::static_pointer_cast<SALAM::Instruction>(entry.second)
                ->setPPASignature(signature);
        }
    }

    // Detect Loop Latches
    for (auto func_iter = m->begin(); func_iter != m->end(); func_iter++) {
        llvm::Function &func = *func_iter;
        // Declarations have no basic blocks and cannot contain loop latches.
        if (func.isDeclaration()) {
            continue;
        }
        dt->recalculate(func);
        loopInfo->releaseMemory();
        loopInfo->analyze(*dt);
        for (auto loop = loopInfo->begin(); loop != loopInfo->end(); ++loop) {
            if (llvm::BasicBlock *exBB = (*loop)->getExitingBlock()) {
                auto latchingBr = exBB->getTerminator();
                auto mapIt = vmap.find(latchingBr);
                if (mapIt != vmap.end()) {
                    auto salamValue = mapIt->second;
                    if (std::shared_ptr<SALAM::Br> sBr =
                            std::dynamic_pointer_cast<SALAM::Br>(salamValue)) {
                        sBr->setLatching(true);
                    }
                }
            }
        }
    }
    auto parseStop = std::chrono::high_resolution_clock::now();
    setupTime = parseStop - parseStart;
}

void
LLVMInterface::launchRead(MemoryRequest *memReq, ActiveFunction *func)
{
    globalReadQueue.insert({memReq, func});
    sampleOccupancy();
    comm->enqueueRead(memReq);
}

void
LLVMInterface::ActiveFunction::launchRead(
    std::shared_ptr<SALAM::Instruction> readInst)
{
    auto rdInst = std::dynamic_pointer_cast<SALAM::Load>(readInst);
    owner->countIssue(LoadOp, readInst->getOpode());
    if (rdInst->isLoadingInternal()) {
        rdInst->loadInternal();
        owner->countCompletion(LoadOp);
    } else {
        auto memReq = (readInst)->createMemoryRequest();
        auto rd_uid = readInst->getUID();
        readQueue.insert({rd_uid, (readInst)});
        readQueueMap.insert({memReq, rd_uid});
        DPRINTFS(SALAMIssue, owner,
                 "memory issue load uid %d address %#x volatile %d\n", rd_uid,
                 memReq->getAddress(), readInst->isVolatileAccess());
        owner->launchRead(memReq, this);
    }
}

void
LLVMInterface::launchWrite(MemoryRequest *memReq, ActiveFunction *func)
{
    globalWriteQueue.insert({memReq, func});
    sampleOccupancy();
    comm->enqueueWrite(memReq);
}

// Memory accesses issue in program order wherever they may overlap.
// Whether the ready load, store or call at pos must wait because of an
// older access: one not yet issued whose address is unknown or
// overlaps, or, for a store, an overlapping one still in flight. A call
// waits for every older access to finish, and every access and call
// waits for an older call to return. No value is forwarded.
bool
LLVMInterface::ActiveFunction::mustWaitForOlder(
    std::list<std::shared_ptr<SALAM::Instruction>>::iterator pos)
{
    // A launched call is older than everything still reserved.
    for (auto &entry : computeQueue) {
        if (entry.second->isCall()) {
            return true;
        }
    }
    auto inst = *pos;
    bool isCall = inst->isCall();
    bool isStore = inst->isStore();
    bool internalLoad = inst->isLoad() && inst->isLoadingInternal();
    Addr start = 0;
    Addr end = 0;
    if (!isCall && !internalLoad && !inst->accessRange(start, end)) {
        panic("%s is ready without an address", inst->getIRString());
    }
    for (auto it = reservation.begin(); it != pos; ++it) {
        auto older = *it;
        if (older->isCall()) {
            return true;
        }
        // Volatile operations can access distinct MMIO registers with
        // related side effects. Preserve their execution order regardless
        // of address overlap; ordinary accesses retain the alias rules.
        if (inst->isVolatileAccess() && older->isVolatileAccess()) {
            return true;
        }
        if (internalLoad) {
            continue;
        }
        bool olderLoad = older->isLoad() && !older->isLoadingInternal();
        if (!older->isStore() && !(olderLoad && (isStore || isCall))) {
            continue;
        }
        if (isCall) {
            return true;
        }
        Addr olderStart;
        Addr olderEnd;
        if (!older->accessRange(olderStart, olderEnd) ||
            (olderStart < end && start < olderEnd)) {
            return true;
        }
    }
    if (isCall) {
        return !readQueue.empty() || !writeQueue.empty();
    }
    if (inst->isVolatileAccess()) {
        for (const auto &entry : readQueue) {
            if (entry.second->isVolatileAccess()) {
                return true;
            }
        }
        for (const auto &entry : writeQueue) {
            if (entry.second->isVolatileAccess()) {
                return true;
            }
        }
    }
    if (isStore) {
        if (writeActive(start, end - start)) {
            return true;
        }
        for (auto &entry : readQueue) {
            Addr readStart;
            Addr readEnd;
            entry.second->accessRange(readStart, readEnd);
            if (readStart < end && start < readEnd) {
                return true;
            }
        }
    }
    return false;
}

// Admit a ready compute operation to its functional unit. False if
// every instance is busy: the operation stays ready and is not issued.
// An operation without a unit is admitted with an empty token.
bool
LLVMInterface::ActiveFunction::admitToUnit(
    std::shared_ptr<SALAM::Instruction> inst, SALAM::FUPool::Token &token)
{
    uint32_t unit = inst->getFunctionalUnit();
    if (unit == 0) {
        // PHIs route incoming SSA values through this admission path,
        // but have no datapath unit, as in resolveFunctionalUnits(). Their
        // operand reads and result writes are observed independently.
        // Observation never binds a scheduling resource for unit zero.
        if (!inst->isPhi()) {
            owner->hw->salam_power_model->unboundOperation(
                llvm::Instruction::getOpcodeName(inst->getOpode()),
                inst->getPPASignature());
        }
        return true;
    }
    SALAM::FUPool &pool = owner->hw->fuPool;
    fatal_if(pool.instances(unit) == 0,
             "%s: %s needs functional unit %d, which has no instances",
             owner->name(), inst->getIRString(), unit);
    if (!pool.canAccept(unit, curTick())) {
        owner->countFuDenial();
        owner->hw->fuDenied(unit);
        return false;
    }
    token = pool.acquire(unit, curTick());
    owner->hw->fuAdmitted(unit);
    owner->hw->salam_power_model->operation(
        owner->hw->findFunctionalUnit(unit)->get_alias(),
        llvm::Instruction::getOpcodeName(inst->getOpode()),
        inst->getPPASignature());
    return true;
}

void
LLVMInterface::ActiveFunction::launchWrite(
    std::shared_ptr<SALAM::Instruction> writeInst)
{
    owner->countIssue(StoreOp, writeInst->getOpode());
    auto memReq = (writeInst)->createMemoryRequest();
    trackWrite(memReq, writeInst);
    auto wr_uid = writeInst->getUID();
    writeQueue.insert({wr_uid, (writeInst)});
    writeQueueMap.insert({memReq, wr_uid});
    DPRINTFS(SALAMIssue, owner,
             "memory issue store uid %d address %#x volatile %d\n", wr_uid,
             memReq->getAddress(), writeInst->isVolatileAccess());
    owner->launchWrite(memReq, this);
}

void
LLVMInterface::readCommit(MemoryRequest *req)
{
    /************************************************************************
     Commit Memory Read Request
    ************************************************************************/
    auto queue_iter = globalReadQueue.find(req);
    if (queue_iter != globalReadQueue.end()) {
        queue_iter->second->readCommit(req);
        DPRINTF(Runtime, "Global Read Commit\n");
        globalReadQueue.erase(queue_iter);
        sampleOccupancy();
    } else {
        panic("No memory request in global read queue!");
    }
}

void
LLVMInterface::ActiveFunction::readCommit(MemoryRequest *req)
{
    /************************************************************************
     Commit Memory Read Request
    ************************************************************************/
    auto map_iter = readQueueMap.find(req);
    if (map_iter != readQueueMap.end()) {
        auto queue_iter = readQueue.find(map_iter->second);
        if (queue_iter != readQueue.end()) {
            DPRINTFS(SALAMIssue, owner, "memory complete uid %d\n",
                     map_iter->second);
            auto load_inst = queue_iter->second;
            uint8_t *readBuff = req->getBuffer();
            load_inst->setRegisterValue(readBuff);
            load_inst->compute();
            if (dbg) {
                DPRINTFS(Runtime, owner, "Local Read Commit\n");
            }
            load_inst->commit();
            owner->countCompletion(LoadOp);
            readQueue.erase(queue_iter);
            readQueueMap.erase(map_iter);
        } else {
            panic("No memory request in read queue for function %u!",
                  func->getUID());
        }
    } else {
        panic("No memory request in read queue for function %u!",
              func->getUID());
    }
}

void
LLVMInterface::writeCommit(MemoryRequest *req)
{
    /*************************************************************************
     Commit Memory Write Request
    *************************************************************************/
    auto queue_iter = globalWriteQueue.find(req);
    if (queue_iter != globalWriteQueue.end()) {
        queue_iter->second->writeCommit(req);
        globalWriteQueue.erase(queue_iter);
        sampleOccupancy();
    } else {
        panic("No memory request in global write queue!");
    }
}

void
LLVMInterface::ActiveFunction::writeCommit(MemoryRequest *req)
{
    /************************************************************************
     Commit Memory Write Request
    ************************************************************************/
    auto map_iter = writeQueueMap.find(req);
    if (map_iter != writeQueueMap.end()) {
        auto queue_iter = writeQueue.find(map_iter->second);
        if (queue_iter != writeQueue.end()) {
            DPRINTFS(SALAMIssue, owner, "memory complete uid %d\n",
                     map_iter->second);
            queue_iter->second->commit();
            owner->countCompletion(StoreOp);
            untrackWrite(map_iter->second);
            writeQueue.erase(queue_iter);
            writeQueueMap.erase(map_iter);
        } else {
            panic("No memory request in write queue for function %u!",
                  func->getUID());
        }
    } else {
        panic("No memory request in write queue for function %u!",
              func->getUID());
    }
}

void
LLVMInterface::initialize()
{
    /************************************************************************
     Initialize the Runtime Engine

     Calls function that constructs the basic block list, initializes the
     reservation table and read, write, and compute queues.
     Set all data collection variables to zero.
    ************************************************************************/
    if (dbg) {
        DPRINTF(LLVMInterface, "Initializing LLVM Runtime Engine!\n");
    }
    setupTime = std::chrono::seconds(0);
    simTime = std::chrono::seconds(0);
    schedulingTime = std::chrono::seconds(0);
    queueProcessTime = std::chrono::seconds(0);
    computeTime = std::chrono::seconds(0);
    hwTime = std::chrono::seconds(0);
    constructStaticGraph();
    resolveFunctionalUnits();
    timeStart = std::chrono::high_resolution_clock::now();
    if (dbg) {
        DPRINTF(LLVMInterface, "=============================================="
                               "==================\n");
    }
    hw->salam_power_model->beginInvocation(curTick());
    launchTopFunction();

    if (dbg) {
        DPRINTF(
            LLVMInterface, "\n%s\n%s\n%s\n",
            "****************************************************************",
            "*          Begin Runtime Simulation Computation Engine         *",
            "***************************************************************"
            "*");
    }
    running = true;
    cycle = 0;
    totals = InvocationTotals();
    totals.startTick = curTick();
    execStats.invocations++;
    // The first compute pass is on the first compute edge at or after
    // the launch, or the next one if a pass already ran on that edge.
    Tick first = clockEdge(Cycles(0));
    if (first == lastPass) {
        first = clockEdge(Cycles(1));
    }
    DPRINTF(SALAMLaunch,
            "launch accepted at %d; first compute pass at %d "
            "(period %d)\n",
            curTick(), first, clockPeriod());
    firstPass = first;
    panic_if(tickEvent.scheduled(), "%s: launched while running", name());
    schedule(tickEvent, first);
}

void
LLVMInterface::debug(uint64_t flags)
{
    // Dump
    for (auto func_iter = functions.begin(); func_iter != functions.end();
         func_iter++) {
        // Function Level
        for (auto bb_iter = (*func_iter)->getBBList()->begin();
             bb_iter != (*func_iter)->getBBList()->end(); bb_iter++) {
            // Basic Block Level
            (*bb_iter)->dump();
            for (auto inst_iter = (*bb_iter)->Instructions()->begin();
                 inst_iter != (*bb_iter)->Instructions()->end(); inst_iter++) {
                // Instruction Level
                (*inst_iter)->dump();
            }
        }
    }
}

void
LLVMInterface::startup()
{
    /************************************************************************
     Initialize communications between gem5 interface and simulator
    ************************************************************************/
    comm->registerCompUnit(this);
}

void
LLVMInterface::finalize()
{
    totals.finishTick = curTick();
    hw->salam_power_model->finishInvocation(curTick());
    // Simulation Times
    simStop = std::chrono::high_resolution_clock::now();
    simTotal = simStop - timeStart;
    panic_if(hw->fuPool.inFlight() != 0,
             "%s: %d functional unit operations still in flight", name(),
             hw->fuPool.inFlight());
    DPRINTF(SALAMFUInventory, "%d functional unit denials\n",
            totals.fuDenials);
    sampleOccupancy();
    DPRINTF(SALAMLaunch, "finished at %d after %d cycles (%d ticks)\n",
            curTick(), cycle, curTick() - firstPass);
    printResults();
    assert(activeFunctions.empty());
    for (auto &value : values) {
        value->releaseGraphEdges();
    }
    for (auto &func : functions) {
        func->releaseGraphEdges();
    }
    functions.clear();
    values.clear();
    comm->finish();
}

void
LLVMInterface::printResults()
{
    std::cout << "*********************************************" << std::endl;
    std::cout << name() << std::endl;
    hw->salam_power_model->printInvocation(std::cout);

    // Compute clock period in nanoseconds.
    double cycle_time = clockPeriod() / sim_clock::as_float::ns;

    auto hwTimingMS =
        std::chrono::duration_cast<std::chrono::milliseconds>(hwTime);
    auto hwHours = std::chrono::duration_cast<std::chrono::hours>(hwTimingMS);
    hwTimingMS -= std::chrono::duration_cast<std::chrono::seconds>(hwHours);
    auto hwMins = std::chrono::duration_cast<std::chrono::minutes>(hwTimingMS);
    hwTimingMS -= std::chrono::duration_cast<std::chrono::seconds>(hwMins);
    auto hwSecs = std::chrono::duration_cast<std::chrono::seconds>(hwTimingMS);
    hwTimingMS -= std::chrono::duration_cast<std::chrono::seconds>(hwSecs);

    auto setupUS =
        std::chrono::duration_cast<std::chrono::microseconds>(setupTime);
    auto setupHours = std::chrono::duration_cast<std::chrono::hours>(setupUS);
    setupUS -= std::chrono::duration_cast<std::chrono::seconds>(setupHours);
    auto setupMins = std::chrono::duration_cast<std::chrono::minutes>(setupUS);
    setupUS -= std::chrono::duration_cast<std::chrono::seconds>(setupMins);
    auto setupSecs = std::chrono::duration_cast<std::chrono::seconds>(setupUS);
    setupUS -= std::chrono::duration_cast<std::chrono::seconds>(setupSecs);

    auto setupMS =
        std::chrono::duration_cast<std::chrono::milliseconds>(setupUS);

    setupUS -= std::chrono::duration_cast<std::chrono::milliseconds>(setupMS);

    auto totalMS =
        std::chrono::duration_cast<std::chrono::milliseconds>(simTotal);
    auto totalHours = std::chrono::duration_cast<std::chrono::hours>(totalMS);
    totalMS -= std::chrono::duration_cast<std::chrono::seconds>(totalHours);
    auto totalMins = std::chrono::duration_cast<std::chrono::minutes>(totalMS);
    totalMS -= std::chrono::duration_cast<std::chrono::seconds>(totalMins);
    auto totalSecs = std::chrono::duration_cast<std::chrono::seconds>(totalMS);
    totalMS -= std::chrono::duration_cast<std::chrono::seconds>(totalSecs);

    auto simMS =
        std::chrono::duration_cast<std::chrono::milliseconds>(simTime);
    auto simHours = std::chrono::duration_cast<std::chrono::hours>(simMS);
    simMS -= std::chrono::duration_cast<std::chrono::seconds>(simHours);
    auto simMins = std::chrono::duration_cast<std::chrono::minutes>(simMS);
    simMS -= std::chrono::duration_cast<std::chrono::seconds>(simMins);
    auto simSecs = std::chrono::duration_cast<std::chrono::seconds>(simMS);
    simMS -= std::chrono::duration_cast<std::chrono::seconds>(simSecs);

    auto queueMS = std::chrono::duration_cast<std::chrono::milliseconds>(
        queueProcessTime);
    auto queueHours = std::chrono::duration_cast<std::chrono::hours>(queueMS);
    queueMS -= std::chrono::duration_cast<std::chrono::seconds>(queueHours);
    auto queueMins = std::chrono::duration_cast<std::chrono::minutes>(queueMS);
    queueMS -= std::chrono::duration_cast<std::chrono::seconds>(queueMins);
    auto queueSecs = std::chrono::duration_cast<std::chrono::seconds>(queueMS);
    queueMS -= std::chrono::duration_cast<std::chrono::seconds>(queueSecs);

    auto schedMS =
        std::chrono::duration_cast<std::chrono::milliseconds>(schedulingTime);
    auto schedHours = std::chrono::duration_cast<std::chrono::hours>(schedMS);
    schedMS -= std::chrono::duration_cast<std::chrono::seconds>(schedHours);
    auto schedMins = std::chrono::duration_cast<std::chrono::minutes>(schedMS);
    schedMS -= std::chrono::duration_cast<std::chrono::seconds>(schedMins);
    auto schedSecs = std::chrono::duration_cast<std::chrono::seconds>(schedMS);
    schedMS -= std::chrono::duration_cast<std::chrono::seconds>(schedSecs);

    auto computeMS =
        std::chrono::duration_cast<std::chrono::milliseconds>(computeTime);
    auto computeHours =
        std::chrono::duration_cast<std::chrono::hours>(computeMS);
    computeMS -=
        std::chrono::duration_cast<std::chrono::seconds>(computeHours);
    auto computeMins =
        std::chrono::duration_cast<std::chrono::minutes>(computeMS);
    computeMS -= std::chrono::duration_cast<std::chrono::seconds>(computeMins);
    auto computeSecs =
        std::chrono::duration_cast<std::chrono::seconds>(computeMS);
    computeMS -= std::chrono::duration_cast<std::chrono::seconds>(computeSecs);

    /************************************************************************
     Usage stats of how many times each instruction was accessed at runtime
    ************************************************************************/

    std::cout << "   ========= Performance Analysis =========" << std::endl;
    std::cout << "   Setup Time:                      " << setupHours.count()
              << "h " << setupMins.count() << "m " << setupSecs.count() << "s "
              << setupMS.count() << "ms " << setupUS.count() << "us"
              << std::endl;
    std::cout << "   Simulation Time (Total):         " << totalHours.count()
              << "h " << totalMins.count() << "m " << totalSecs.count() << "s "
              << totalMS.count() << "ms" << std::endl;
    std::cout << "   Simulation Time (Active):        " << simHours.count()
              << "h " << simMins.count() << "m " << simSecs.count() << "s "
              << simMS.count() << "ms" << std::endl;
    std::cout << "        Queue Processing Time:      " << queueHours.count()
              << "h " << queueMins.count() << "m " << queueSecs.count() << "s "
              << queueMS.count() << "ms" << std::endl;
    std::cout << "             Scheduling Time:       " << schedHours.count()
              << "h " << schedMins.count() << "m " << schedSecs.count() << "s "
              << schedMS.count() << "ms" << std::endl;
    std::cout << "             Computation Time:      " << computeHours.count()
              << "h " << computeMins.count() << "m " << computeSecs.count()
              << "s " << computeMS.count() << "ms" << std::endl;
    std::cout << "   System Clock:                    " << 1.0 / (cycle_time)
              << "GHz" << std::endl;
    // This invocation's totals, which statistics resets do not clear.
    const Tick elapsed = totals.finishTick - totals.startTick;
    std::cout << "   Compute passes:                  " << totals.cycles
              << std::endl;
    std::cout << "   Invocation duration:             " << elapsed << " ticks"
              << std::endl;
    std::cout << "   Runtime:                         "
              << elapsed / sim_clock::as_float::us << " us" << std::endl;
    static const char *categories[NumCycleCategories] = {
        "issuing", "waiting for a functional unit", "waiting for memory",
        "waiting for compute", "other"};
    for (int i = 0; i < NumCycleCategories; i++) {
        std::cout << "      " << std::left << std::setw(31)
                  << (std::string(categories[i]) + ":") << std::right
                  << totals.cycleCategories[i] << " cycles" << std::endl;
    }
    uint64_t issued = 0;
    for (int i = 0; i < NumOpClasses; i++) {
        issued += totals.issued[i];
    }
    std::cout << "   Operations Issued:               " << issued << std::endl;
    std::cout << "   Functional Unit Denials:         " << totals.fuDenials
              << std::endl;
    std::cout << std::endl;
}

void
LLVMInterface::launchFunction(std::shared_ptr<SALAM::Function> callee,
                              std::shared_ptr<SALAM::Instruction> caller)
{
    // Add the callee to our list of active functions
    activeFunctions.push_back(ActiveFunction(this, callee, caller));
    activeFunctions.back().launch();
    sampleOccupancy();
}

void
LLVMInterface::launchTopFunction()
{
    for (auto it = functions.begin(); it != functions.end(); it++) {
        if ((*it)->isTop()) {
            // Launch the top level function
            launchFunction((*it), nullptr);
            return;
        }
    }
    // Fallback if no function was marked as the top-level
    panic("No top-level function (set top_name param for LLVMInterface)\n");
}

void
LLVMInterface::ActiveFunction::launch()
{
    if (dbg) {
        DPRINTFS(LLVMInterface, owner, "Launching Function: %s\n",
                 func->getIRStub());
    }
    // Fetch the arguments
    std::vector<std::shared_ptr<SALAM::Value>> funcArgs =
        *(func->getArguments());
    if (func->isTop()) {
        // We need to fetch argument values from the memory mapped registers
        if (dbg) {
            DPRINTFS(LLVMInterface, owner, "Connecting CommInterface\n");
        }
        CommInterface *comm = owner->getCommInterface();
        if (dbg) {
            DPRINTFS(LLVMInterface, owner, "Connecting HWInterface\n");
        }
        hw = owner->getHWInterface();

        unsigned argOffset = 0;
        for (auto arg : funcArgs) {
            uint64_t argSizeInBytes = arg->getSizeInBytes();
            uint64_t regValue = comm->getGlobalVar(argOffset, argSizeInBytes);
            arg->setRegisterValue(regValue);
            arg->publishStorage();
            argOffset += argSizeInBytes;
        }
    } else {
        // We need to fetch argument values from the calling function
        std::vector<SALAM::Operand> callerArgs = *caller->getOperands();
        if (funcArgs.size() != callerArgs.size()) {
            panic("Function expects %d args. Got %d args.", funcArgs.size(),
                  callerArgs.size());
        }
        for (auto i = 0; i < callerArgs.size(); i++) {
            funcArgs.at(i)->setRegisterValue(callerArgs.at(i).getOpRegister());
            funcArgs.at(i)->publishStorage();
        }
    }
    func->addInstance();
    // Schedule the first BB
    scheduleBB(func->entry());
}

// Count each unit's instances. A unit with a nonzero limit has that
// many. A unit with limit 0 has one per static instruction that uses
// it in top and the functions top reaches through direct calls; a
// function called from several places is counted once.
void
LLVMInterface::resolveFunctionalUnits()
{
    std::vector<std::shared_ptr<SALAM::Function>> reached;
    std::set<SALAM::Function *> seen;
    for (auto &function : functions) {
        if (function->isTop()) {
            reached.push_back(function);
            seen.insert(function.get());
        }
    }
    HWInterface::FUInventory inventory;
    std::map<uint32_t, uint32_t> uses;
    std::map<uint32_t, SALAMPowerModel::Operations> operations;
    for (size_t i = 0; i < reached.size(); i++) {
        for (auto &bb : *reached[i]->getBBList()) {
            for (auto &inst : *bb->Instructions()) {
                if (inst->isCall()) {
                    auto call = std::dynamic_pointer_cast<SALAM::Call>(inst);
                    auto callee = std::dynamic_pointer_cast<SALAM::Function>(
                        call->getCalleeValue());
                    if (callee && seen.insert(callee.get()).second) {
                        reached.push_back(callee);
                    }
                }
                uint32_t unit = inst->getFunctionalUnit();
                if (unit == 0) {
                    if (!(inst->isLoad() || inst->isStore() ||
                          inst->isTerminator() || inst->isReturn() ||
                          inst->isCall() || inst->isPhi())) {
                        const std::string identity =
                            std::string("fu:unbound:") +
                            llvm::Instruction::getOpcodeName(
                                inst->getOpode()) +
                            ":" + inst->getPPASignature();
                        hw->salam_power_model->markStaticIncomplete(identity);
                        hw->salam_power_model->markIncomplete(identity +
                                                              ":energy");
                    }
                    continue;
                }
                fatal_if(inst->isLoad() || inst->isStore() ||
                             inst->isTerminator() || inst->isCall() ||
                             inst->isPhi(),
                         "%s: %s uses functional unit %d, but memory and "
                         "control instructions cannot",
                         name(), inst->getIRString(), unit);
                uses[unit]++;
                operations[unit].push_back(
                    {llvm::Instruction::getOpcodeName(inst->getOpode()),
                     inst->getPPASignature(),
                     double(cyclesToTicks(Cycles(inst->getCycleCount()))) /
                         sim_clock::as_float::s});
                inventory.mapping.emplace_back(inst->getUID(), unit);
            }
        }
    }
    for (auto unit : hw->functional_units->functional_unit_list) {
        uint32_t id = unit->get_enum_value();
        uint32_t count = uses[id];
        uint32_t instances = unit->get_limit() > 0 ? unit->get_limit() : count;
        inventory.instances[id] = instances;
        DPRINTF(SALAMFUInventory,
                "%s (%d): %d instances, %d static instructions\n",
                unit->get_alias(), id, instances, count);
    }
    hw->setInventory(inventory);
    configurePowerModel(reached, inventory, operations);
    std::map<uint32_t, SALAM::FUPool::UnitConfig> pool;
    for (auto unit : hw->functional_units->functional_unit_list) {
        uint32_t id = unit->get_enum_value();
        pool[id] = {inventory.instances.at(id),
                    cyclesToTicks(Cycles(unit->get_initiation_interval()))};
    }
    hw->fuPool.configure(pool, clockPeriod());
}

void
LLVMInterface::configurePowerModel(
    const std::vector<std::shared_ptr<SALAM::Function>> &reached,
    const HWInterface::FUInventory &inventory,
    const std::map<uint32_t, SALAMPowerModel::Operations> &operations)
{
    auto *model = hw->salam_power_model;
    const SALAMPowerModel::Operations no_operations;
    for (auto *unit : hw->functional_units->functional_unit_list) {
        const auto id = unit->get_enum_value();
        const auto found = operations.find(id);
        model->configureFunctionalUnit(
            unit->get_alias(), inventory.instances.at(id),
            found == operations.end() ? no_operations : found->second,
            double(cyclesToTicks(Cycles(unit->get_initiation_interval()))) /
                sim_clock::as_float::s,
            double(clockPeriod()) / sim_clock::as_float::s);
    }
    std::set<SALAM::Register *> storage;
    std::map<std::string, uint64_t> slots;
    auto addStorage = [&](const std::shared_ptr<SALAM::Value> &value) {
        if (!model->isEnabled() || !value->getReg() || value->isConstant()) {
            return;
        }
        const auto type = value->getStorageType();
        if (type.size() < 2 || type == "unavailable" || type == "v") {
            model->markStaticIncomplete(
                "register:unsupported_materialized_type");
            model->markIncomplete("register:unsupported_materialized_type");
            return;
        }
        const unsigned bits = std::stoul(type.substr(1));
        if (!bits || bits > 64) {
            model->markStaticIncomplete("register:unsupported_width:" + type);
            model->markIncomplete("register:unsupported_width:" + type);
            return;
        }
        if (storage.insert(value->getReg().get()).second) {
            slots[type]++;
        }
        value->setStorageCallbacks(
            [model, type] { model->registerRead(type); },
            [model, type] { model->registerWrite(type); });
    };
    for (auto &function : reached) {
        // The runtime has one backing slot per reachable SSA value and
        // permits one active instance of each function. Clones share it.
        for (auto &argument : *function->getArguments()) {
            addStorage(argument);
        }
        for (auto &bb : *function->getBBList()) {
            for (auto &instruction : *bb->Instructions()) {
                addStorage(instruction);
            }
        }
    }
    model->configureRegisters(slots);
}

std::shared_ptr<SALAM::Instruction>
LLVMInterface::createInstruction(llvm::Instruction *inst, uint64_t id)
{
    // LLVM synchronization semantics are not modeled by the ordinary
    // accelerator memory requests. This does not restrict gem5's atomic
    // memory-system simulation mode.
    if (inst->isAtomic() || llvm::isa<llvm::FenceInst>(inst)) {
        std::string ir_text;
        llvm::raw_string_ostream ss(ir_text);
        ss << *inst;
        fatal("SALAM does not support LLVM atomic instructions: %s",
              ss.str().c_str());
    }
    uint64_t OpCode = inst->Instruction::getOpcode();

    uint64_t functional_unit = 0;
    for (auto hw_inst : hw->inst_config->inst_list) {
        if (OpCode == hw_inst->get_opcode_num()) {
            functional_unit = hw_inst->get_functional_unit();
            // 0 is no functional unit.
            if (functional_unit != 0) {
                FunctionalUnitBase *unit =
                    hw->findFunctionalUnit(functional_unit);
                fatal_if(!unit,
                         "%s: instruction '%s' uses functional unit %d, "
                         "which the hardware profile does not define",
                         name(), inst->getOpcodeName(), functional_unit);
            }
            break;
        }
    }

    switch (OpCode) {
        case llvm::Instruction::Ret:
            return SALAM::createRetInst(id, this, debug(), OpCode,
                                        hw->cycle_counts->ret_inst,
                                        functional_unit);
            break;
        case llvm::Instruction::Br:
            return SALAM::createBrInst(id, this, debug(), OpCode,
                                       hw->cycle_counts->br_inst,
                                       functional_unit);
            break;
        case llvm::Instruction::Switch:
            return SALAM::createSwitchInst(id, this, debug(), OpCode,
                                           hw->cycle_counts->switch_inst,
                                           functional_unit);
            break;
        case llvm::Instruction::Add:
            return SALAM::createAddInst(id, this, debug(), OpCode,
                                        hw->cycle_counts->add_inst,
                                        functional_unit);
            break;
        case llvm::Instruction::FAdd:
            return SALAM::createFAddInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->fadd_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::Sub:
            return SALAM::createSubInst(id, this, debug(), OpCode,
                                        hw->cycle_counts->sub_inst,
                                        functional_unit);
            break;
        case llvm::Instruction::FSub:
            return SALAM::createFSubInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->fsub_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::Mul:
            return SALAM::createMulInst(id, this, debug(), OpCode,
                                        hw->cycle_counts->mul_inst,
                                        functional_unit);
            break;
        case llvm::Instruction::FMul:
            return SALAM::createFMulInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->fmul_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::UDiv:
            return SALAM::createUDivInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->udiv_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::SDiv:
            return SALAM::createSDivInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->sdiv_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::FDiv:
            return SALAM::createFDivInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->fdiv_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::URem:
            return SALAM::createURemInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->urem_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::SRem:
            return SALAM::createSRemInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->srem_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::FRem:
            return SALAM::createFRemInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->frem_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::Shl:
            return SALAM::createShlInst(id, this, debug(), OpCode,
                                        hw->cycle_counts->shl_inst,
                                        functional_unit);
            break;
        case llvm::Instruction::LShr:
            return SALAM::createLShrInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->lshr_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::AShr:
            return SALAM::createAShrInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->ashr_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::And:
            return SALAM::createAndInst(id, this, debug(), OpCode,
                                        hw->cycle_counts->and_inst,
                                        functional_unit);
            break;
        case llvm::Instruction::Or:
            return SALAM::createOrInst(id, this, debug(), OpCode,
                                       hw->cycle_counts->or_inst,
                                       functional_unit);
            break;
        case llvm::Instruction::Xor:
            return SALAM::createXorInst(id, this, debug(), OpCode,
                                        hw->cycle_counts->xor_inst,
                                        functional_unit);
            break;
        case llvm::Instruction::Load:
            return SALAM::createLoadInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->load_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::Store:
            return SALAM::createStoreInst(id, this, debug(), OpCode,
                                          hw->cycle_counts->store_inst,
                                          functional_unit);
            break;
        case llvm::Instruction::GetElementPtr:
            return SALAM::createGetElementPtrInst(id, this, debug(), OpCode,
                                                  hw->cycle_counts->gep_inst,
                                                  functional_unit);
            break;
        case llvm::Instruction::Trunc:
            return SALAM::createTruncInst(id, this, debug(), OpCode,
                                          hw->cycle_counts->trunc_inst,
                                          functional_unit);
            break;
        case llvm::Instruction::ZExt:
            return SALAM::createZExtInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->zext_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::SExt:
            return SALAM::createSExtInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->sext_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::FPToUI:
            return SALAM::createFPToUIInst(id, this, debug(), OpCode,
                                           hw->cycle_counts->fptoui_inst,
                                           functional_unit);
            break;
        case llvm::Instruction::FPToSI:
            return SALAM::createFPToSIInst(id, this, debug(), OpCode,
                                           hw->cycle_counts->fptosi_inst,
                                           functional_unit);
            break;
        case llvm::Instruction::UIToFP:
            return SALAM::createUIToFPInst(id, this, debug(), OpCode,
                                           hw->cycle_counts->uitofp_inst,
                                           functional_unit);
            break;
        case llvm::Instruction::SIToFP:
            return SALAM::createSIToFPInst(id, this, debug(), OpCode,
                                           hw->cycle_counts->sitofp_inst,
                                           functional_unit);
            break;
        case llvm::Instruction::FPTrunc:
            return SALAM::createFPTruncInst(id, this, debug(), OpCode,
                                            hw->cycle_counts->fptrunc_inst,
                                            functional_unit);
            break;
        case llvm::Instruction::FPExt:
            return SALAM::createFPExtInst(id, this, debug(), OpCode,
                                          hw->cycle_counts->fpext_inst,
                                          functional_unit);
            break;
        case llvm::Instruction::PtrToInt:
            return SALAM::createPtrToIntInst(id, this, debug(), OpCode,
                                             hw->cycle_counts->ptrtoint_inst,
                                             functional_unit);
            break;
        case llvm::Instruction::IntToPtr:
            return SALAM::createIntToPtrInst(id, this, debug(), OpCode,
                                             hw->cycle_counts->inttoptr_inst,
                                             functional_unit);
            break;
        case llvm::Instruction::ICmp:
            return SALAM::createICmpInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->icmp_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::FCmp:
            return SALAM::createFCmpInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->fcmp_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::PHI:
            return SALAM::createPHIInst(id, this, debug(), OpCode,
                                        hw->cycle_counts->phi_inst,
                                        functional_unit);
            break;
        case llvm::Instruction::Call:
            return SALAM::createCallInst(id, this, debug(), OpCode,
                                         hw->cycle_counts->call_inst,
                                         functional_unit);
            break;
        case llvm::Instruction::Select:
            return SALAM::createSelectInst(id, this, debug(), OpCode,
                                           hw->cycle_counts->select_inst,
                                           functional_unit);
            break;
        default: {
            std::string ir_text;
            llvm::raw_string_ostream ss(ir_text);
            ss << *inst;
            fatal("SALAM does not support LLVM instruction '%s' "
                  "(opcode %u): %s",
                  llvm::Instruction::getOpcodeName(OpCode), OpCode,
                  ss.str().c_str());
        }
    }
}

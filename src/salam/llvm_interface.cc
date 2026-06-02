/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// LLVMInterface Includes
#include "salam/llvm_interface.hh"

LLVMInterface::LLVMInterface(const LLVMInterfaceParams &p)
    : AccComputeUnit(p),
      filename(p.in_file),
      topName(p.top_name),
      scheduling_threshold(p.sched_threshold),
      clock_period(p.clock_period),
      lockstep(p.lockstep_mode)
{
    clock_period = clock_period * 1000;
    dbg = comm->debug();
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
    for (auto inst : instruction_list) {
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
            }
            findDynamicDeps(clone_inst);
            reservation.push_back(clone_inst);
        }
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

    if (owner->hw->hw_statistics->use_cycle_tracking()) {
        auto hwStart = std::chrono::high_resolution_clock::now();
        hw_cycle_stats.reset();
        owner->hw->hw_statistics->updateHWStatsCycleStart();

        // Update Params
        hw_cycle_stats.cycle = owner->cycle;
        hw_cycle_stats.resInFlight = reservation.size();
        hw_cycle_stats.loadInFlight = readQueue.size();
        hw_cycle_stats.storeInFlight = writeQueue.size();
        hw_cycle_stats.compInFlight = computeQueue.size();

        //
        auto hwStop = std::chrono::high_resolution_clock::now();
        owner->addHWTime(hwStop - hwStart);
    }

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
            (queue_iter->second)->reset();
            queue_iter = computeQueue.erase(queue_iter);
            hw_cycle_stats.compCommited++;
        } else {
            ++queue_iter;
            hw_cycle_stats.compFUStall++;
        }
    }
    if (canReturn()) {
        // Handle function return
        if (dbg) {
            DPRINTFS(Runtime, owner, "[[Function Return]]\n\n");
        }
        if (caller != nullptr) {
            // Signal the calling instruction
            if (caller->getSize() > 0) {
                auto retInst = reservation.front();
                auto retOperand = retInst->getOperands()->front();
                caller->setRegisterValue(retOperand.getOpRegister());
            }
            func->removeInstance();
            caller->commit();
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
                        if (inst->isLoadingInternal()) {
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
                            hw_cycle_stats.loadInternal++;
                        } else if (!writeActive(inst->getPtrOperandValue(0))) {
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
                            hw_cycle_stats.loadAcitve++;
                        } else {
                            auto activeWrite =
                                getActiveWrite(inst->getPtrOperandValue(0));
                            inst->addRuntimeDependency(activeWrite);
                            activeWrite->addRuntimeUser(inst);
                            ++queue_iter;
                            hw_cycle_stats.loadRawStall++;
                        }
                    } else if ((inst)->isStore()) {
                        // WAR Protection to insure reading
                        // finishes before a write
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
                        hw_cycle_stats.storeActive++;
                    } else if ((inst)->isLatchingBrExiting() &&
                               ((reservation.size() > 1) || !queuesClear())) {
                        ++queue_iter;
                    } else if ((inst)->isTerminator()) {
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
                        if (callee->canLaunch()) {
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
                    } else {
                        auto computeStart =
                            std::chrono::high_resolution_clock::now();
                        if (!(inst)->launch()) {
                            if (dbg) {
                                DPRINTFS(Runtime, owner,
                                         "\t\t  | Added to Comp Queue: %s - "
                                         "UID[%i]\n",
                                         llvm::Instruction::getOpcodeName(
                                             (inst)->getOpode()),
                                         (inst)->getUID());
                            }
                            computeQueue.insert({(inst)->getUID(), inst});
                            hw_cycle_stats.compLaunched++;
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
                        hw_cycle_stats.compActive++;
                    }
                } else {
                    ++queue_iter;
                }
            } else {
                ++queue_iter;
            }
        }
    }

    if (owner->hw->hw_statistics->use_cycle_tracking()) {
        auto hwStart = std::chrono::high_resolution_clock::now();
        for (auto fu : hw->functional_units->functional_unit_list) {
            std::cout << fu->get_alias() << " - " << fu->get_in_use() << "\n";
        }

        owner->hw->hw_statistics->updateHWStatsCycleEnd(owner->cycle);
        auto hwStop = std::chrono::high_resolution_clock::now();
        owner->addHWTime(hwStop - hwStart);
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
    if (activeFunctions.empty()) {
        // We are finished executing all functions.
        // Signal completion to the CommInterface
        running = false;
        finalize();
        return;
    }
    //////////////// Schedule Next Cycle ////////////////////////
    if (running && !tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + clock_period); // * process_delay);
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
        panic("Error reading Module");
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

    // Detect Loop Latches
    for (auto func_iter = m->begin(); func_iter != m->end(); func_iter++) {
        llvm::Function &func = *func_iter;
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
    comm->enqueueRead(memReq);
}

void
LLVMInterface::ActiveFunction::launchRead(
    std::shared_ptr<SALAM::Instruction> readInst)
{
    auto rdInst = std::dynamic_pointer_cast<SALAM::Load>(readInst);
    if (rdInst->isLoadingInternal()) {
        rdInst->loadInternal();
    } else {
        auto memReq = (readInst)->createMemoryRequest();
        auto rd_uid = readInst->getUID();
        readQueue.insert({rd_uid, (readInst)});
        readQueueMap.insert({memReq, rd_uid});
        owner->launchRead(memReq, this);
    }
}

void
LLVMInterface::launchWrite(MemoryRequest *memReq, ActiveFunction *func)
{
    globalWriteQueue.insert({memReq, func});
    comm->enqueueWrite(memReq);
}

void
LLVMInterface::ActiveFunction::launchWrite(
    std::shared_ptr<SALAM::Instruction> writeInst)
{
    auto memReq = (writeInst)->createMemoryRequest();
    trackWrite(memReq->getAddress(), writeInst);
    auto wr_uid = writeInst->getUID();
    writeQueue.insert({wr_uid, (writeInst)});
    writeQueueMap.insert({memReq, wr_uid});
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
            auto load_inst = queue_iter->second;
            uint8_t *readBuff = req->getBuffer();
            load_inst->setRegisterValue(readBuff);
            load_inst->compute();
            if (dbg) {
                DPRINTFS(Runtime, owner, "Local Read Commit\n");
            }
            load_inst->commit();
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
            queue_iter->second->commit();
            Addr addressWritten = map_iter->first->getAddress();
            untrackWrite(addressWritten);
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
    timeStart = std::chrono::high_resolution_clock::now();
    if (dbg) {
        DPRINTF(LLVMInterface, "=============================================="
                               "==================\n");
    }
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
    stalls = 0;
    tick();
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
    // Simulation Times
    simStop = std::chrono::high_resolution_clock::now();
    simTotal = simStop - timeStart;
    printResults();
    functions.clear();
    values.clear();
    comm->finish();
}

void
LLVMInterface::printResults()
{
    std::map<uint64_t, uint64_t> totals_reads;
    std::map<uint64_t, uint64_t> totals_writes;

    std::cout << "*********************************************" << std::endl;
    std::cout << name() << std::endl;

    for (auto it : values) {
        if (it->isInstruction()) {
            if (it->getReg()) {
                totals_reads[it->getOpode()] += it->getReg()->getReads();
                totals_writes[it->getOpode()] += it->getReg()->getWrites();
            }
        }
    }

    double adder_area =
        (hw->opcodes->get_usage(13) + hw->opcodes->get_usage(20) +
         hw->opcodes->get_usage(15)) *
        1.794430e+02;
    double adder_reads = (totals_reads[13] + totals_reads[15]);
    double adder_writes = (totals_writes[13] + totals_writes[15]);
    double adder_power_static = adder_reads * 2.380803e-03;
    double adder_power_dynamic = adder_writes * (8.115300e-03 + 6.162853e-03);

    double bitwise_area =
        (hw->opcodes->get_usage(29) + hw->opcodes->get_usage(30) +
         hw->opcodes->get_usage(25) + hw->opcodes->get_usage(26) +
         hw->opcodes->get_usage(27) + hw->opcodes->get_usage(28)) *
        5.036996e+01;
    double bitwise_reads =
        (totals_reads[25] + totals_reads[26] + totals_reads[27] +
         totals_reads[28] + totals_reads[29] + totals_reads[30]);
    double bitwise_writes =
        (totals_writes[25] + totals_writes[26] + totals_writes[27] +
         totals_writes[28] + totals_writes[29] + totals_writes[30]);
    double bitwise_power_static = bitwise_reads * 6.111633e-04;
    double bitwise_power_dynamic =
        bitwise_writes * (1.680942e-03 + 1.322420e-03);

    double multiplier_area =
        (hw->opcodes->get_usage(17) + hw->opcodes->get_usage(19) +
         hw->opcodes->get_usage(20)) *
        4.595000e+03;
    double multiplier_reads =
        totals_reads[17] + totals_reads[19] + totals_reads[20];
    double multiplier_writes =
        totals_writes[17] + totals_writes[19] + totals_writes[20];
    double multiplier_power_static = multiplier_reads * 4.817683e-02;
    double multiplier_power_dynamic =
        multiplier_writes * (5.725752e-01 + 8.662890e-01);

    double reg_area = hw->opcodes->get_usage(34) * 32 * 5.981433e+00;
    double reg_reads = totals_reads[34] * 32;
    double reg_writes = totals_writes[34] * 32;
    double register_power_static = reg_reads * 7.395312e-05;
    double register_power_dynamic = reg_writes * (1.322600e-03 + 1.792126e-04);

    double total_area = adder_area + bitwise_area + multiplier_area + reg_area;
    double total_power_static = adder_power_static + bitwise_power_static +
                                multiplier_power_static +
                                register_power_static;
    double total_power_dynamic = adder_power_dynamic + bitwise_power_dynamic +
                                 multiplier_power_dynamic +
                                 register_power_dynamic;

    std::cout << "Total Area: " << total_area;
    std::cout << "\nTotal Power Static: " << total_power_static << "\n";
    std::cout << "\nTotal Power Dynamic: " << total_power_dynamic << "\n";

    Tick cycle_time = clock_period / 1000;

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
    std::cout << "   Runtime:                         " << cycle << " cycles"
              << std::endl;
    std::cout << "   Runtime:                         "
              << (cycle * cycle_time * (1e-3)) << " us" << std::endl;
    std::cout << "   Stalls:                          " << stalls << " cycles"
              << std::endl;
    std::cout << "   Executed Nodes:                  " << (cycle - stalls - 1)
              << " cycles" << std::endl;
    std::cout << std::endl;
}

void
LLVMInterface::dumpQueues()
{}

void
LLVMInterface::launchFunction(std::shared_ptr<SALAM::Function> callee,
                              std::shared_ptr<SALAM::Instruction> caller)
{
    // Add the callee to our list of active functions
    activeFunctions.push_back(ActiveFunction(this, callee, caller));
    activeFunctions.back().launch();
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
        }
    }
    func->addInstance();
    // Schedule the first BB
    scheduleBB(func->entry());
}

std::shared_ptr<SALAM::Instruction>
LLVMInterface::createInstruction(llvm::Instruction *inst, uint64_t id)
{
    uint64_t OpCode = inst->Instruction::getOpcode();
    hw->opcodes->update_usage(OpCode);

    uint64_t functional_unit = 0;
    for (auto hw_inst : hw->inst_config->inst_list) {
        if (OpCode == hw_inst->get_opcode_num()) {
            functional_unit = hw_inst->get_functional_unit();
            for (auto hw_fu : hw->functional_units->functional_unit_list) {
                if (hw_fu->get_enum_value() == functional_unit) {
                    hw_fu->inc_functional_unit_limit();
                    break;
                }
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
            warn("Tried to create instance of undefined instruction type!");
            return SALAM::createBadInst(id, this, dbg, OpCode, 0, 0);
            break;
        }
    }
}

/*
 * Copyright (c) 2012 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <ostream>

#include "arch/mmapped_ipr.hh"
#include "arch/utility.hh"
#include "cpu/kvm/base.hh"
#include "debug/Checkpoint.hh"
#include "debug/Drain.hh"
#include "debug/Kvm.hh"
#include "debug/KvmIO.hh"
#include "debug/KvmRun.hh"
#include "params/BaseKvmCPU.hh"
#include "sim/process.hh"
#include "sim/system.hh"

#include <signal.h>

/* Used by some KVM macros */
#define PAGE_SIZE pageSize

static volatile __thread bool timerOverflowed = false;

BaseKvmCPU::BaseKvmCPU(BaseKvmCPUParams *params)
    : BaseCPU(params),
      vm(*params->kvmVM),
      _status(Idle),
      dataPort(name() + ".dcache_port", this),
      instPort(name() + ".icache_port", this),
      threadContextDirty(true),
      kvmStateDirty(false),
      vcpuID(vm.allocVCPUID()), vcpuFD(-1), vcpuMMapSize(0),
      _kvmRun(NULL), mmioRing(NULL),
      pageSize(sysconf(_SC_PAGE_SIZE)),
      tickEvent(*this),
      activeInstPeriod(0),
      perfControlledByTimer(params->usePerfOverflow),
      hostFactor(params->hostFactor),
      drainManager(NULL),
      ctrInsts(0)
{
    if (pageSize == -1)
        panic("KVM: Failed to determine host page size (%i)\n",
              errno);

    thread = new SimpleThread(this, 0, params->system,
                              params->itb, params->dtb, params->isa[0]);
    thread->setStatus(ThreadContext::Halted);
    tc = thread->getTC();
    threadContexts.push_back(tc);
}

BaseKvmCPU::~BaseKvmCPU()
{
    if (_kvmRun)
        munmap(_kvmRun, vcpuMMapSize);
    close(vcpuFD);
}

void
BaseKvmCPU::init()
{
    BaseCPU::init();

    if (numThreads != 1)
        fatal("KVM: Multithreading not supported");

    tc->initMemProxies(tc);

    // initialize CPU, including PC
    if (FullSystem && !switchedOut())
        TheISA::initCPU(tc, tc->contextId());

    mmio_req.setThreadContext(tc->contextId(), 0);
}

void
BaseKvmCPU::startup()
{
    const BaseKvmCPUParams * const p(
        dynamic_cast<const BaseKvmCPUParams *>(params()));

    Kvm &kvm(vm.kvm);

    BaseCPU::startup();

    assert(vcpuFD == -1);

    // Tell the VM that a CPU is about to start.
    vm.cpuStartup();

    // We can't initialize KVM CPUs in BaseKvmCPU::init() since we are
    // not guaranteed that the parent KVM VM has initialized at that
    // point. Initialize virtual CPUs here instead.
    vcpuFD = vm.createVCPU(vcpuID);

    // Map the KVM run structure */
    vcpuMMapSize = kvm.getVCPUMMapSize();
    _kvmRun = (struct kvm_run *)mmap(0, vcpuMMapSize,
                                     PROT_READ | PROT_WRITE, MAP_SHARED,
                                     vcpuFD, 0);
    if (_kvmRun == MAP_FAILED)
        panic("KVM: Failed to map run data structure\n");

    // Setup a pointer to the MMIO ring buffer if coalesced MMIO is
    // available. The offset into the KVM's communication page is
    // provided by the coalesced MMIO capability.
    int mmioOffset(kvm.capCoalescedMMIO());
    if (!p->useCoalescedMMIO) {
        inform("KVM: Coalesced MMIO disabled by config.\n");
    } else if (mmioOffset) {
        inform("KVM: Coalesced IO available\n");
        mmioRing = (struct kvm_coalesced_mmio_ring *)(
            (char *)_kvmRun + (mmioOffset * pageSize));
    } else {
        inform("KVM: Coalesced not supported by host OS\n");
    }

    thread->startup();

    Event *startupEvent(
        new EventWrapper<BaseKvmCPU,
                         &BaseKvmCPU::startupThread>(this, true));
    schedule(startupEvent, curTick());
}

void
BaseKvmCPU::startupThread()
{
    // Do thread-specific initialization. We need to setup signal
    // delivery for counters and timers from within the thread that
    // will execute the event queue to ensure that signals are
    // delivered to the right threads.
    const BaseKvmCPUParams * const p(
        dynamic_cast<const BaseKvmCPUParams *>(params()));

    // Setup signal handlers. This has to be done after the vCPU is
    // created since it manipulates the vCPU signal mask.
    setupSignalHandler();

    setupCounters();

    if (p->usePerfOverflow)
        runTimer.reset(new PerfKvmTimer(hwCycles,
                                        KVM_TIMER_SIGNAL,
                                        p->hostFactor,
                                        p->hostFreq));
    else
        runTimer.reset(new PosixKvmTimer(KVM_TIMER_SIGNAL, CLOCK_MONOTONIC,
                                         p->hostFactor,
                                         p->hostFreq));

}

void
BaseKvmCPU::regStats()
{
    using namespace Stats;

    BaseCPU::regStats();

    numInsts
        .name(name() + ".committedInsts")
        .desc("Number of instructions committed")
        ;

    numVMExits
        .name(name() + ".numVMExits")
        .desc("total number of KVM exits")
        ;

    numVMHalfEntries
        .name(name() + ".numVMHalfEntries")
        .desc("number of KVM entries to finalize pending operations")
        ;

    numExitSignal
        .name(name() + ".numExitSignal")
        .desc("exits due to signal delivery")
        ;

    numMMIO
        .name(name() + ".numMMIO")
        .desc("number of VM exits due to memory mapped IO")
        ;

    numCoalescedMMIO
        .name(name() + ".numCoalescedMMIO")
        .desc("number of coalesced memory mapped IO requests")
        ;

    numIO
        .name(name() + ".numIO")
        .desc("number of VM exits due to legacy IO")
        ;

    numHalt
        .name(name() + ".numHalt")
        .desc("number of VM exits due to wait for interrupt instructions")
        ;

    numInterrupts
        .name(name() + ".numInterrupts")
        .desc("number of interrupts delivered")
        ;

    numHypercalls
        .name(name() + ".numHypercalls")
        .desc("number of hypercalls")
        ;
}

void
BaseKvmCPU::serializeThread(std::ostream &os, ThreadID tid)
{
    if (DTRACE(Checkpoint)) {
        DPRINTF(Checkpoint, "KVM: Serializing thread %i:\n", tid);
        dump();
    }

    assert(tid == 0);
    assert(_status == Idle);
    thread->serialize(os);
}

void
BaseKvmCPU::unserializeThread(Checkpoint *cp, const std::string &section,
                              ThreadID tid)
{
    DPRINTF(Checkpoint, "KVM: Unserialize thread %i:\n", tid);

    assert(tid == 0);
    assert(_status == Idle);
    thread->unserialize(cp, section);
    threadContextDirty = true;
}

unsigned int
BaseKvmCPU::drain(DrainManager *dm)
{
    if (switchedOut())
        return 0;

    DPRINTF(Drain, "BaseKvmCPU::drain\n");
    switch (_status) {
      case Running:
        // The base KVM code is normally ready when it is in the
        // Running state, but the architecture specific code might be
        // of a different opinion. This may happen when the CPU been
        // notified of an event that hasn't been accepted by the vCPU
        // yet.
        if (!archIsDrained()) {
            drainManager = dm;
            return 1;
        }

        // The state of the CPU is consistent, so we don't need to do
        // anything special to drain it. We simply de-schedule the
        // tick event and enter the Idle state to prevent nasty things
        // like MMIOs from happening.
        if (tickEvent.scheduled())
            deschedule(tickEvent);
        _status = Idle;

        /** FALLTHROUGH */
      case Idle:
        // Idle, no need to drain
        assert(!tickEvent.scheduled());

        // Sync the thread context here since we'll need it when we
        // switch CPUs or checkpoint the CPU.
        syncThreadContext();

        return 0;

      case RunningServiceCompletion:
        // The CPU has just requested a service that was handled in
        // the RunningService state, but the results have still not
        // been reported to the CPU. Now, we /could/ probably just
        // update the register state ourselves instead of letting KVM
        // handle it, but that would be tricky. Instead, we enter KVM
        // and let it do its stuff.
        drainManager = dm;

        DPRINTF(Drain, "KVM CPU is waiting for service completion, "
                "requesting drain.\n");
        return 1;

      case RunningService:
        // We need to drain since the CPU is waiting for service (e.g., MMIOs)
        drainManager = dm;

        DPRINTF(Drain, "KVM CPU is waiting for service, requesting drain.\n");
        return 1;

      default:
        panic("KVM: Unhandled CPU state in drain()\n");
        return 0;
    }
}

void
BaseKvmCPU::drainResume()
{
    assert(!tickEvent.scheduled());

    // We might have been switched out. In that case, we don't need to
    // do anything.
    if (switchedOut())
        return;

    DPRINTF(Kvm, "drainResume\n");
    verifyMemoryMode();

    // The tick event is de-scheduled as a part of the draining
    // process. Re-schedule it if the thread context is active.
    if (tc->status() == ThreadContext::Active) {
        schedule(tickEvent, nextCycle());
        _status = Running;
    } else {
        _status = Idle;
    }
}

void
BaseKvmCPU::switchOut()
{
    DPRINTF(Kvm, "switchOut\n");

    BaseCPU::switchOut();

    // We should have drained prior to executing a switchOut, which
    // means that the tick event shouldn't be scheduled and the CPU is
    // idle.
    assert(!tickEvent.scheduled());
    assert(_status == Idle);
}

void
BaseKvmCPU::takeOverFrom(BaseCPU *cpu)
{
    DPRINTF(Kvm, "takeOverFrom\n");

    BaseCPU::takeOverFrom(cpu);

    // We should have drained prior to executing a switchOut, which
    // means that the tick event shouldn't be scheduled and the CPU is
    // idle.
    assert(!tickEvent.scheduled());
    assert(_status == Idle);
    assert(threadContexts.size() == 1);

    // Force an update of the KVM state here instead of flagging the
    // TC as dirty. This is not ideal from a performance point of
    // view, but it makes debugging easier as it allows meaningful KVM
    // state to be dumped before and after a takeover.
    updateKvmState();
    threadContextDirty = false;
}

void
BaseKvmCPU::verifyMemoryMode() const
{
    if (!(system->isAtomicMode() && system->bypassCaches())) {
        fatal("The KVM-based CPUs requires the memory system to be in the "
              "'atomic_noncaching' mode.\n");
    }
}

void
BaseKvmCPU::wakeup()
{
    DPRINTF(Kvm, "wakeup()\n");

    if (thread->status() != ThreadContext::Suspended)
        return;

    thread->activate();
}

void
BaseKvmCPU::activateContext(ThreadID thread_num, Cycles delay)
{
    DPRINTF(Kvm, "ActivateContext %d (%d cycles)\n", thread_num, delay);

    assert(thread_num == 0);
    assert(thread);

    assert(_status == Idle);
    assert(!tickEvent.scheduled());

    numCycles += ticksToCycles(thread->lastActivate - thread->lastSuspend);

    schedule(tickEvent, clockEdge(delay));
    _status = Running;
}


void
BaseKvmCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(Kvm, "SuspendContext %d\n", thread_num);

    assert(thread_num == 0);
    assert(thread);

    if (_status == Idle)
        return;

    assert(_status == Running);

    // The tick event may no be scheduled if the quest has requested
    // the monitor to wait for interrupts. The normal CPU models can
    // get their tick events descheduled by quiesce instructions, but
    // that can't happen here.
    if (tickEvent.scheduled())
        deschedule(tickEvent);

    _status = Idle;
}

void
BaseKvmCPU::deallocateContext(ThreadID thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}

void
BaseKvmCPU::haltContext(ThreadID thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}

ThreadContext *
BaseKvmCPU::getContext(int tn)
{
    assert(tn == 0);
    syncThreadContext();
    return tc;
}


Counter
BaseKvmCPU::totalInsts() const
{
    return ctrInsts;
}

Counter
BaseKvmCPU::totalOps() const
{
    hack_once("Pretending totalOps is equivalent to totalInsts()\n");
    return ctrInsts;
}

void
BaseKvmCPU::dump()
{
    inform("State dumping not implemented.");
}

void
BaseKvmCPU::tick()
{
    Tick delay(0);
    assert(_status != Idle);

    switch (_status) {
      case RunningService:
        // handleKvmExit() will determine the next state of the CPU
        delay = handleKvmExit();

        if (tryDrain())
            _status = Idle;
        break;

      case RunningServiceCompletion:
      case Running: {
          EventQueue *q = curEventQueue();
          Tick ticksToExecute(q->nextTick() - curTick());

          // We might need to update the KVM state.
          syncKvmState();

          // Setup any pending instruction count breakpoints using
          // PerfEvent.
          setupInstStop();

          DPRINTF(KvmRun, "Entering KVM...\n");
          if (drainManager) {
              // Force an immediate exit from KVM after completing
              // pending operations. The architecture-specific code
              // takes care to run until it is in a state where it can
              // safely be drained.
              delay = kvmRunDrain();
          } else {
              delay = kvmRun(ticksToExecute);
          }

          // Entering into KVM implies that we'll have to reload the thread
          // context from KVM if we want to access it. Flag the KVM state as
          // dirty with respect to the cached thread context.
          kvmStateDirty = true;

          // Enter into the RunningService state unless the
          // simulation was stopped by a timer.
          if (_kvmRun->exit_reason !=  KVM_EXIT_INTR) {
              _status = RunningService;
          } else {
              ++numExitSignal;
              _status = Running;
          }

          // Service any pending instruction events. The vCPU should
          // have exited in time for the event using the instruction
          // counter configured by setupInstStop().
          comInstEventQueue[0]->serviceEvents(ctrInsts);
          system->instEventQueue.serviceEvents(system->totalNumInsts);

          if (tryDrain())
              _status = Idle;
      } break;

      default:
        panic("BaseKvmCPU entered tick() in an illegal state (%i)\n",
              _status);
    }

    // Schedule a new tick if we are still running
    if (_status != Idle)
        schedule(tickEvent, clockEdge(ticksToCycles(delay)));
}

Tick
BaseKvmCPU::kvmRunDrain()
{
    // By default, the only thing we need to drain is a pending IO
    // operation which assumes that we are in the
    // RunningServiceCompletion state.
    assert(_status == RunningServiceCompletion);

    // Deliver the data from the pending IO operation and immediately
    // exit.
    return kvmRun(0);
}

uint64_t
BaseKvmCPU::getHostCycles() const
{
    return hwCycles.read();
}

Tick
BaseKvmCPU::kvmRun(Tick ticks)
{
    Tick ticksExecuted;
    DPRINTF(KvmRun, "KVM: Executing for %i ticks\n", ticks);
    timerOverflowed = false;

    if (ticks == 0) {
        // Settings ticks == 0 is a special case which causes an entry
        // into KVM that finishes pending operations (e.g., IO) and
        // then immediately exits.
        DPRINTF(KvmRun, "KVM: Delivering IO without full guest entry\n");

        ++numVMHalfEntries;

        // This signal is always masked while we are executing in gem5
        // and gets unmasked temporarily as soon as we enter into
        // KVM. See setSignalMask() and setupSignalHandler().
        raise(KVM_TIMER_SIGNAL);

        // Enter into KVM. KVM will check for signals after completing
        // pending operations (IO). Since the KVM_TIMER_SIGNAL is
        // pending, this forces an immediate exit into gem5 again. We
        // don't bother to setup timers since this shouldn't actually
        // execute any code in the guest.
        ioctlRun();

        // We always execute at least one cycle to prevent the
        // BaseKvmCPU::tick() to be rescheduled on the same tick
        // twice.
        ticksExecuted = clockPeriod();
    } else {
        if (ticks < runTimer->resolution()) {
            DPRINTF(KvmRun, "KVM: Adjusting tick count (%i -> %i)\n",
                    ticks, runTimer->resolution());
            ticks = runTimer->resolution();
        }

        // Get hardware statistics after synchronizing contexts. The KVM
        // state update might affect guest cycle counters.
        uint64_t baseCycles(getHostCycles());
        uint64_t baseInstrs(hwInstructions.read());

        // Arm the run timer and start the cycle timer if it isn't
        // controlled by the overflow timer. Starting/stopping the cycle
        // timer automatically starts the other perf timers as they are in
        // the same counter group.
        runTimer->arm(ticks);
        if (!perfControlledByTimer)
            hwCycles.start();

        ioctlRun();

        runTimer->disarm();
        if (!perfControlledByTimer)
            hwCycles.stop();

        // The timer signal may have been delivered after we exited
        // from KVM. It will be pending in that case since it is
        // masked when we aren't executing in KVM. Discard it to make
        // sure we don't deliver it immediately next time we try to
        // enter into KVM.
        discardPendingSignal(KVM_TIMER_SIGNAL);
        discardPendingSignal(KVM_INST_SIGNAL);

        const uint64_t hostCyclesExecuted(getHostCycles() - baseCycles);
        const uint64_t simCyclesExecuted(hostCyclesExecuted * hostFactor);
        const uint64_t instsExecuted(hwInstructions.read() - baseInstrs);
        ticksExecuted = runTimer->ticksFromHostCycles(hostCyclesExecuted);

        if (ticksExecuted < ticks &&
            timerOverflowed &&
            _kvmRun->exit_reason == KVM_EXIT_INTR) {
            // TODO: We should probably do something clever here...
            warn("KVM: Early timer event, requested %i ticks but got %i ticks.\n",
                 ticks, ticksExecuted);
        }

        /* Update statistics */
        numCycles += simCyclesExecuted;;
        numInsts += instsExecuted;
        ctrInsts += instsExecuted;
        system->totalNumInsts += instsExecuted;

        DPRINTF(KvmRun,
                "KVM: Executed %i instructions in %i cycles "
                "(%i ticks, sim cycles: %i).\n",
                instsExecuted, hostCyclesExecuted, ticksExecuted, simCyclesExecuted);
    }

    ++numVMExits;

    return ticksExecuted + flushCoalescedMMIO();
}

void
BaseKvmCPU::kvmNonMaskableInterrupt()
{
    ++numInterrupts;
    if (ioctl(KVM_NMI) == -1)
        panic("KVM: Failed to deliver NMI to virtual CPU\n");
}

void
BaseKvmCPU::kvmInterrupt(const struct kvm_interrupt &interrupt)
{
    ++numInterrupts;
    if (ioctl(KVM_INTERRUPT, (void *)&interrupt) == -1)
        panic("KVM: Failed to deliver interrupt to virtual CPU\n");
}

void
BaseKvmCPU::getRegisters(struct kvm_regs &regs) const
{
    if (ioctl(KVM_GET_REGS, &regs) == -1)
        panic("KVM: Failed to get guest registers\n");
}

void
BaseKvmCPU::setRegisters(const struct kvm_regs &regs)
{
    if (ioctl(KVM_SET_REGS, (void *)&regs) == -1)
        panic("KVM: Failed to set guest registers\n");
}

void
BaseKvmCPU::getSpecialRegisters(struct kvm_sregs &regs) const
{
    if (ioctl(KVM_GET_SREGS, &regs) == -1)
        panic("KVM: Failed to get guest special registers\n");
}

void
BaseKvmCPU::setSpecialRegisters(const struct kvm_sregs &regs)
{
    if (ioctl(KVM_SET_SREGS, (void *)&regs) == -1)
        panic("KVM: Failed to set guest special registers\n");
}

void
BaseKvmCPU::getFPUState(struct kvm_fpu &state) const
{
    if (ioctl(KVM_GET_FPU, &state) == -1)
        panic("KVM: Failed to get guest FPU state\n");
}

void
BaseKvmCPU::setFPUState(const struct kvm_fpu &state)
{
    if (ioctl(KVM_SET_FPU, (void *)&state) == -1)
        panic("KVM: Failed to set guest FPU state\n");
}


void
BaseKvmCPU::setOneReg(uint64_t id, const void *addr)
{
#ifdef KVM_SET_ONE_REG
    struct kvm_one_reg reg;
    reg.id = id;
    reg.addr = (uint64_t)addr;

    if (ioctl(KVM_SET_ONE_REG, &reg) == -1) {
        panic("KVM: Failed to set register (0x%x) value (errno: %i)\n",
              id, errno);
    }
#else
    panic("KVM_SET_ONE_REG is unsupported on this platform.\n");
#endif
}

void
BaseKvmCPU::getOneReg(uint64_t id, void *addr) const
{
#ifdef KVM_GET_ONE_REG
    struct kvm_one_reg reg;
    reg.id = id;
    reg.addr = (uint64_t)addr;

    if (ioctl(KVM_GET_ONE_REG, &reg) == -1) {
        panic("KVM: Failed to get register (0x%x) value (errno: %i)\n",
              id, errno);
    }
#else
    panic("KVM_GET_ONE_REG is unsupported on this platform.\n");
#endif
}

std::string
BaseKvmCPU::getAndFormatOneReg(uint64_t id) const
{
#ifdef KVM_GET_ONE_REG
    std::ostringstream ss;

    ss.setf(std::ios::hex, std::ios::basefield);
    ss.setf(std::ios::showbase);
#define HANDLE_INTTYPE(len)                      \
    case KVM_REG_SIZE_U ## len: {                \
        uint ## len ## _t value;                 \
        getOneReg(id, &value);                   \
        ss << value;                             \
    }  break

#define HANDLE_ARRAY(len)                       \
    case KVM_REG_SIZE_U ## len: {               \
        uint8_t value[len / 8];                 \
        getOneReg(id, value);                   \
        ss << "[" << value[0];                  \
        for (int i = 1; i < len  / 8; ++i)      \
            ss << ", " << value[i];             \
        ss << "]";                              \
      } break

    switch (id & KVM_REG_SIZE_MASK) {
        HANDLE_INTTYPE(8);
        HANDLE_INTTYPE(16);
        HANDLE_INTTYPE(32);
        HANDLE_INTTYPE(64);
        HANDLE_ARRAY(128);
        HANDLE_ARRAY(256);
        HANDLE_ARRAY(512);
        HANDLE_ARRAY(1024);
      default:
        ss << "??";
    }

#undef HANDLE_INTTYPE
#undef HANDLE_ARRAY

    return ss.str();
#else
    panic("KVM_GET_ONE_REG is unsupported on this platform.\n");
#endif
}

void
BaseKvmCPU::syncThreadContext()
{
    if (!kvmStateDirty)
        return;

    assert(!threadContextDirty);

    updateThreadContext();
    kvmStateDirty = false;
}

void
BaseKvmCPU::syncKvmState()
{
    if (!threadContextDirty)
        return;

    assert(!kvmStateDirty);

    updateKvmState();
    threadContextDirty = false;
}

Tick
BaseKvmCPU::handleKvmExit()
{
    DPRINTF(KvmRun, "handleKvmExit (exit_reason: %i)\n", _kvmRun->exit_reason);
    assert(_status == RunningService);

    // Switch into the running state by default. Individual handlers
    // can override this.
    _status = Running;
    switch (_kvmRun->exit_reason) {
      case KVM_EXIT_UNKNOWN:
        return handleKvmExitUnknown();

      case KVM_EXIT_EXCEPTION:
        return handleKvmExitException();

      case KVM_EXIT_IO:
        _status = RunningServiceCompletion;
        ++numIO;
        return handleKvmExitIO();

      case KVM_EXIT_HYPERCALL:
        ++numHypercalls;
        return handleKvmExitHypercall();

      case KVM_EXIT_HLT:
        /* The guest has halted and is waiting for interrupts */
        DPRINTF(Kvm, "handleKvmExitHalt\n");
        ++numHalt;

        // Suspend the thread until the next interrupt arrives
        thread->suspend();

        // This is actually ignored since the thread is suspended.
        return 0;

      case KVM_EXIT_MMIO:
        _status = RunningServiceCompletion;
        /* Service memory mapped IO requests */
        DPRINTF(KvmIO, "KVM: Handling MMIO (w: %u, addr: 0x%x, len: %u)\n",
                _kvmRun->mmio.is_write,
                _kvmRun->mmio.phys_addr, _kvmRun->mmio.len);

        ++numMMIO;
        return doMMIOAccess(_kvmRun->mmio.phys_addr, _kvmRun->mmio.data,
                            _kvmRun->mmio.len, _kvmRun->mmio.is_write);

      case KVM_EXIT_IRQ_WINDOW_OPEN:
        return handleKvmExitIRQWindowOpen();

      case KVM_EXIT_FAIL_ENTRY:
        return handleKvmExitFailEntry();

      case KVM_EXIT_INTR:
        /* KVM was interrupted by a signal, restart it in the next
         * tick. */
        return 0;

      case KVM_EXIT_INTERNAL_ERROR:
        panic("KVM: Internal error (suberror: %u)\n",
              _kvmRun->internal.suberror);

      default:
        dump();
        panic("KVM: Unexpected exit (exit_reason: %u)\n", _kvmRun->exit_reason);
    }
}

Tick
BaseKvmCPU::handleKvmExitIO()
{
    panic("KVM: Unhandled guest IO (dir: %i, size: %i, port: 0x%x, count: %i)\n",
          _kvmRun->io.direction, _kvmRun->io.size,
          _kvmRun->io.port, _kvmRun->io.count);
}

Tick
BaseKvmCPU::handleKvmExitHypercall()
{
    panic("KVM: Unhandled hypercall\n");
}

Tick
BaseKvmCPU::handleKvmExitIRQWindowOpen()
{
    warn("KVM: Unhandled IRQ window.\n");
    return 0;
}


Tick
BaseKvmCPU::handleKvmExitUnknown()
{
    dump();
    panic("KVM: Unknown error when starting vCPU (hw reason: 0x%llx)\n",
          _kvmRun->hw.hardware_exit_reason);
}

Tick
BaseKvmCPU::handleKvmExitException()
{
    dump();
    panic("KVM: Got exception when starting vCPU "
          "(exception: %u, error_code: %u)\n",
          _kvmRun->ex.exception, _kvmRun->ex.error_code);
}

Tick
BaseKvmCPU::handleKvmExitFailEntry()
{
    dump();
    panic("KVM: Failed to enter virtualized mode (hw reason: 0x%llx)\n",
          _kvmRun->fail_entry.hardware_entry_failure_reason);
}

Tick
BaseKvmCPU::doMMIOAccess(Addr paddr, void *data, int size, bool write)
{
    ThreadContext *tc(thread->getTC());
    syncThreadContext();

    mmio_req.setPhys(paddr, size, Request::UNCACHEABLE, dataMasterId());
    // Some architectures do need to massage physical addresses a bit
    // before they are inserted into the memory system. This enables
    // APIC accesses on x86 and m5ops where supported through a MMIO
    // interface.
    BaseTLB::Mode tlb_mode(write ? BaseTLB::Write : BaseTLB::Read);
    Fault fault(tc->getDTBPtr()->finalizePhysical(&mmio_req, tc, tlb_mode));
    if (fault != NoFault)
        warn("Finalization of MMIO address failed: %s\n", fault->name());


    const MemCmd cmd(write ? MemCmd::WriteReq : MemCmd::ReadReq);
    Packet pkt(&mmio_req, cmd);
    pkt.dataStatic(data);

    if (mmio_req.isMmappedIpr()) {
        const Cycles ipr_delay(write ?
                             TheISA::handleIprWrite(tc, &pkt) :
                             TheISA::handleIprRead(tc, &pkt));
        return clockPeriod() * ipr_delay;
    } else {
        return dataPort.sendAtomic(&pkt);
    }
}

void
BaseKvmCPU::setSignalMask(const sigset_t *mask)
{
    std::unique_ptr<struct kvm_signal_mask> kvm_mask;

    if (mask) {
        kvm_mask.reset((struct kvm_signal_mask *)operator new(
                           sizeof(struct kvm_signal_mask) + sizeof(*mask)));
        // The kernel and the user-space headers have different ideas
        // about the size of sigset_t. This seems like a massive hack,
        // but is actually what qemu does.
        assert(sizeof(*mask) >= 8);
        kvm_mask->len = 8;
        memcpy(kvm_mask->sigset, mask, kvm_mask->len);
    }

    if (ioctl(KVM_SET_SIGNAL_MASK, (void *)kvm_mask.get()) == -1)
        panic("KVM: Failed to set vCPU signal mask (errno: %i)\n",
              errno);
}

int
BaseKvmCPU::ioctl(int request, long p1) const
{
    if (vcpuFD == -1)
        panic("KVM: CPU ioctl called before initialization\n");

    return ::ioctl(vcpuFD, request, p1);
}

Tick
BaseKvmCPU::flushCoalescedMMIO()
{
    if (!mmioRing)
        return 0;

    DPRINTF(KvmIO, "KVM: Flushing the coalesced MMIO ring buffer\n");

    // TODO: We might need to do synchronization when we start to
    // support multiple CPUs
    Tick ticks(0);
    while (mmioRing->first != mmioRing->last) {
        struct kvm_coalesced_mmio &ent(
            mmioRing->coalesced_mmio[mmioRing->first]);

        DPRINTF(KvmIO, "KVM: Handling coalesced MMIO (addr: 0x%x, len: %u)\n",
                ent.phys_addr, ent.len);

        ++numCoalescedMMIO;
        ticks += doMMIOAccess(ent.phys_addr, ent.data, ent.len, true);

        mmioRing->first = (mmioRing->first + 1) % KVM_COALESCED_MMIO_MAX;
    }

    return ticks;
}

/**
 * Cycle timer overflow when running in KVM. Forces the KVM syscall to
 * exit with EINTR and allows us to run the event queue.
 *
 * @warn This function might not be called since some kernels don't
 * seem to deliver signals when the signal is only unmasked when
 * running in KVM. This doesn't matter though since we are only
 * interested in getting KVM to exit, which happens as expected. See
 * setupSignalHandler() and kvmRun() for details about KVM signal
 * handling.
 */
static void
onTimerOverflow(int signo, siginfo_t *si, void *data)
{
    timerOverflowed = true;
}

/**
 * Instruction counter overflow when running in KVM. Forces the KVM
 * syscall to exit with EINTR and allows us to handle instruction
 * count events.
 */
static void
onInstEvent(int signo, siginfo_t *si, void *data)
{
}

void
BaseKvmCPU::setupSignalHandler()
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = onTimerOverflow;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    if (sigaction(KVM_TIMER_SIGNAL, &sa, NULL) == -1)
        panic("KVM: Failed to setup vCPU timer signal handler\n");

    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = onInstEvent;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    if (sigaction(KVM_INST_SIGNAL, &sa, NULL) == -1)
        panic("KVM: Failed to setup vCPU instruction signal handler\n");

    sigset_t sigset;
    if (pthread_sigmask(SIG_BLOCK, NULL, &sigset) == -1)
        panic("KVM: Failed get signal mask\n");

    // Request KVM to setup the same signal mask as we're currently
    // running with except for the KVM control signals. We'll
    // sometimes need to raise the KVM_TIMER_SIGNAL to cause immediate
    // exits from KVM after servicing IO requests. See kvmRun().
    sigdelset(&sigset, KVM_TIMER_SIGNAL);
    sigdelset(&sigset, KVM_INST_SIGNAL);
    setSignalMask(&sigset);

    // Mask our control signals so they aren't delivered unless we're
    // actually executing inside KVM.
    sigaddset(&sigset, KVM_TIMER_SIGNAL);
    sigaddset(&sigset, KVM_INST_SIGNAL);
    if (pthread_sigmask(SIG_SETMASK, &sigset, NULL) == -1)
        panic("KVM: Failed mask the KVM control signals\n");
}

bool
BaseKvmCPU::discardPendingSignal(int signum) const
{
    int discardedSignal;

    // Setting the timeout to zero causes sigtimedwait to return
    // immediately.
    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 0;

    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, signum);

    do {
        discardedSignal = sigtimedwait(&sigset, NULL, &timeout);
    } while (discardedSignal == -1 && errno == EINTR);

    if (discardedSignal == signum)
        return true;
    else if (discardedSignal == -1 && errno == EAGAIN)
        return false;
    else
        panic("Unexpected return value from sigtimedwait: %i (errno: %i)\n",
              discardedSignal, errno);
}

void
BaseKvmCPU::setupCounters()
{
    DPRINTF(Kvm, "Attaching cycle counter...\n");
    PerfKvmCounterConfig cfgCycles(PERF_TYPE_HARDWARE,
                                PERF_COUNT_HW_CPU_CYCLES);
    cfgCycles.disabled(true)
        .pinned(true);

    // Try to exclude the host. We set both exclude_hv and
    // exclude_host since different architectures use slightly
    // different APIs in the kernel.
    cfgCycles.exclude_hv(true)
        .exclude_host(true);

    if (perfControlledByTimer) {
        // We need to configure the cycles counter to send overflows
        // since we are going to use it to trigger timer signals that
        // trap back into m5 from KVM. In practice, this means that we
        // need to set some non-zero sample period that gets
        // overridden when the timer is armed.
        cfgCycles.wakeupEvents(1)
            .samplePeriod(42);
    }

    hwCycles.attach(cfgCycles,
                    0); // TID (0 => currentThread)

    setupInstCounter();
}

bool
BaseKvmCPU::tryDrain()
{
    if (!drainManager)
        return false;

    if (!archIsDrained()) {
        DPRINTF(Drain, "tryDrain: Architecture code is not ready.\n");
        return false;
    }

    if (_status == Idle || _status == Running) {
        DPRINTF(Drain,
                "tryDrain: CPU transitioned into the Idle state, drain done\n");
        drainManager->signalDrainDone();
        drainManager = NULL;
        return true;
    } else {
        DPRINTF(Drain, "tryDrain: CPU not ready.\n");
        return false;
    }
}

void
BaseKvmCPU::ioctlRun()
{
    if (ioctl(KVM_RUN) == -1) {
        if (errno != EINTR)
            panic("KVM: Failed to start virtual CPU (errno: %i)\n",
                  errno);
    }
}

void
BaseKvmCPU::setupInstStop()
{
    if (comInstEventQueue[0]->empty()) {
        setupInstCounter(0);
    } else {
        const uint64_t next(comInstEventQueue[0]->nextTick());

        assert(next > ctrInsts);
        setupInstCounter(next - ctrInsts);
    }
}

void
BaseKvmCPU::setupInstCounter(uint64_t period)
{
    // No need to do anything if we aren't attaching for the first
    // time or the period isn't changing.
    if (period == activeInstPeriod && hwInstructions.attached())
        return;

    PerfKvmCounterConfig cfgInstructions(PERF_TYPE_HARDWARE,
                                         PERF_COUNT_HW_INSTRUCTIONS);

    // Try to exclude the host. We set both exclude_hv and
    // exclude_host since different architectures use slightly
    // different APIs in the kernel.
    cfgInstructions.exclude_hv(true)
        .exclude_host(true);

    if (period) {
        // Setup a sampling counter if that has been requested.
        cfgInstructions.wakeupEvents(1)
            .samplePeriod(period);
    }

    // We need to detach and re-attach the counter to reliably change
    // sampling settings. See PerfKvmCounter::period() for details.
    if (hwInstructions.attached())
        hwInstructions.detach();
    assert(hwCycles.attached());
    hwInstructions.attach(cfgInstructions,
                          0, // TID (0 => currentThread)
                          hwCycles);

    if (period)
        hwInstructions.enableSignals(KVM_INST_SIGNAL);

    activeInstPeriod = period;
}

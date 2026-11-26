/*
 * Copyright (c) 2026 The Board of Trustees of the Leland Stanford
 * Junior University
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
 */

#include "arch/x86/pin/cpu.hh"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/times.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>

#include "arch/x86/cpuid.hh"
#include "arch/x86/isa.hh"
#include "arch/x86/page_size.hh"
#include "arch/x86/pin/message.h"
#include "arch/x86/pin/regfile.h"
#include "arch/x86/regs/int.hh"
#include "arch/x86/utility.hh"
#include "base/loader/symtab.hh"
#include "base/output.hh"
#include "cpu/simple_thread.hh"
#include "debug/PinCPU.hh"
#include "params/X86PinCPU.hh"
#include "sim/faults.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

namespace X86ISA
{

static std::string
gem5BinaryDir()
{
    char buf[PATH_MAX];
    const ssize_t len = ::readlink("/proc/self/exe", buf, sizeof buf - 1);
    fatal_if(len < 0, "readlink(/proc/self/exe) failed: %s",
             std::strerror(errno));
    buf[len] = '\0';
    const std::string path(buf);
    const size_t slash = path.rfind('/');
    if (slash == std::string::npos) {
        return ".";
    }
    return path.substr(0, slash);
}

static std::vector<std::string_view>
split_by_spaces(std::string_view str)
{
    std::vector<std::string_view> result;
    size_t pos = 0;
    size_t size = str.size();

    while (pos < size) {
        // Skip any leading spaces
        while (pos < size &&
               std::isspace(static_cast<unsigned char>(str[pos]))) {
            ++pos;
        }

        if (pos >= size) {
            break;
        }

        // Find the end of the current word
        size_t start = pos;
        while (pos < size &&
               !std::isspace(static_cast<unsigned char>(str[pos]))) {
            ++pos;
        }

        // Create a string_view for the current word
        result.emplace_back(str.substr(start, pos - start));
    }

    return result;
}

PinCPU::PinCPU(const X86PinCPUParams &params)
    : BaseCPU(params),
      tickEvent([this] { tick(); }, "X86PinCPU tick", false,
                Event::CPU_Tick_Pri),
      _status(Idle),
      dataPort(name() + ".dcache_port", this),
      instPort(name() + ".icache_port", this),
      pinExe(params.pinExe),
      pinGuest(params.pinGuest.empty()
                   ? gem5BinaryDir() + "/util/pincpu/guest.elf"
                   : params.pinGuest),
      pinTool(params.pinTool.empty()
                  ? gem5BinaryDir() + "/util/pincpu/libpintool.so"
                  : params.pinTool),
      pinPid(-1),
      system(params.system)
{
    thread = std::make_unique<SimpleThread>(
        this, /*thread_num*/ 0, params.system, params.workload[0], params.mmu,
        params.isa[0], params.decoder[0]);
    thread->setStatus(ThreadContext::Halted);
    tc = thread->getTC();
    threadContexts.push_back(tc);

    if (params.countInsts) {
        ctrInsts = 0;
    }

    // Parse PinTool arguments.
    for (std::string_view sv : split_by_spaces(params.pinArgs)) {
        pinArgs.emplace_back(sv);
    }
    for (std::string_view sv : split_by_spaces(params.pinToolArgs)) {
        pinToolArgs.emplace_back(sv);
    }
}

void
PinCPU::haltContext()
{
    DPRINTF(PinCPU, "Halting Pin process\n");
    // Tell Pin to exit.
    assert(pinPid >= 0 && reqFd >= 0 && respFd >= 0);

    Message msg;
    msg.type = Message::Exit;
    msg.send(reqFd);

    close(reqFd);
    close(respFd);

    if (waitpid(pinPid, nullptr, 0) < 0) {
        panic("waitpid failed!\n");
    }

    // Any later unmap callback must not try to talk to the dead pintool.
    pinPid = -1;
    reqFd = -1;
    respFd = -1;

    // Dump times.
    struct tms tms;
    if (times(&tms) < 0) {
        panic("times(2) failed\n");
    }
    const auto tick = sysconf(_SC_CLK_TCK);
    DPRINTF(PinCPU,
            "user.pin: %fs, sys.pin: %fs, user.gem5: %fs, sys.gem5: %fs\n",
            static_cast<double>(tms.tms_cutime) / tick,
            static_cast<double>(tms.tms_cstime) / tick,
            static_cast<double>(tms.tms_utime) / tick,
            static_cast<double>(tms.tms_stime) / tick);
}

bool
PinCPU::PinRequestPort::recvTimingResp(PacketPtr pkt)
{ fatal("Unsupported: %s", __func__); }

void
PinCPU::PinRequestPort::recvReqRetry()
{ fatal("Unsupported: %s", __func__); }

Port &
PinCPU::getDataPort()
{ return dataPort; }

Port &
PinCPU::getInstPort()
{ return instPort; }

void
PinCPU::wakeup(ThreadID tid)
{ fatal("Unsupported: %s", __func__); }

Counter
PinCPU::totalInsts() const
{ return ctrInsts ? *ctrInsts : 0; }

Counter
PinCPU::totalOps() const
{
    // Pin counts instructions, not micro-ops, so the two are reported as
    // equal. Statistics derived from op counts are approximate.
    warn_once("PinCPU reports op count as instruction count\n");
    return totalInsts();
}

const std::string &
PinCPU::getPinExe() const
{ return pinExe; }

const std::string &
PinCPU::getPinTool() const
{ return pinTool; }

const std::string &
PinCPU::getGuest() const
{ return pinGuest; }

void
PinCPU::init()
{
    BaseCPU::init();
    fatal_if(numThreads != 1, "Pin: Multithreading not supported");
}

void
PinCPU::startup()
{
    BaseCPU::startup();

    // Create pipes for bidirectional communication.
    int req_fds[2];
    if (pipe(req_fds) < 0) {
        fatal("pipe failed: %s", std::strerror(errno));
    }
    int resp_fds[2];
    if (pipe(resp_fds) < 0) {
        fatal("pipe failed: %s", std::strerror(errno));
    }
    reqFd = req_fds[1];
    respFd = resp_fds[0];
    const int remote_req_fd = req_fds[0];
    const int remote_resp_fd = resp_fds[1];

    char req_path[32];
    std::sprintf(req_path, "/dev/fd/%d", remote_req_fd);
    char resp_path[32];
    std::sprintf(resp_path, "/dev/fd/%d", remote_resp_fd);
    const std::string pin_tool = getPinTool();
    const std::string pin_exe = getPinExe();
    const std::string guest_prog = getGuest();

    std::stringstream shm_path_ss;
    const auto &backing_store = system->getPhysMem().getBackingStore();
    fatal_if(backing_store.size() != 1,
             "Pin CPU supports only one backing store entry");
    const int shm_fd = backing_store[0].shmFd;
    fatal_if(shm_fd < 0, "Pin CPU requires shared memory backing store");
    shm_path_ss << "/dev/fd/" << shm_fd;
    const std::string shm_path = shm_path_ss.str();

    int shm_fd_flags;
    if ((shm_fd_flags = fcntl(shm_fd, F_GETFD)) < 0) {
        fatal("fcntl FD_GETFD failed");
    }
    shm_fd_flags &= ~FD_CLOEXEC;
    if (fcntl(shm_fd, F_SETFD, shm_fd_flags) < 0) {
        fatal("fcntl FD_SETFD failed");
    }

    pinPid = fork();
    if (pinPid < 0) {
        fatal("fork: %s", std::strerror(errno));
    } else if (pinPid == 0) {
        // Create the log files for the guest.
        const std::string guestout_path = simout.resolve("guestout.txt");
        const int guestout_fd =
            open(guestout_path.c_str(),
                 O_WRONLY | O_APPEND | O_TRUNC | O_CREAT, 0664);
        if (guestout_fd < 0) {
            panic("Failed to create guestout.txt\n");
        }
        if (dup2(guestout_fd, STDOUT_FILENO) < 0) {
            panic("dup2 failed\n");
        }

        const std::string guesterr_path = simout.resolve("guesterr.txt");
        const int guesterr_fd =
            open(guesterr_path.c_str(),
                 O_WRONLY | O_APPEND | O_TRUNC | O_CREAT, 0664);
        if (guesterr_fd < 0) {
            panic("Failed to create guesterr.txt\n");
        }
        if (dup2(guesterr_fd, STDERR_FILENO) < 0) {
            panic("dup2 failed\n");
        }

        // This is the Pin subprocess. Execute pin.
        std::vector<std::string> args;
        auto it = std::back_inserter(args);

        // Pin executable.
        *it++ = pin_exe;

        // Pin args.
        if (std::getenv("PIN_APPDEBUG")) {
            *it++ = "-appdebug";
            *it++ = "1";
        }
        if (std::getenv("PIN_TOOLDEBUG")) {
            *it++ = "-pause_tool";
            *it++ = "30";
        }

        it = std::copy(pinArgs.begin(), pinArgs.end(), it);

        // Pintool.
        *it++ = "-t";
        *it++ = pin_tool;

        // Pintool args.
        *it++ = "-log";
        *it++ = simout.resolve("pin.log");
        *it++ = "-req_path";
        *it++ = req_path;
        *it++ = "-resp_path";
        *it++ = resp_path;
        *it++ = "-mem_path";
        *it++ = shm_path;
        *it++ = "-instcount";
        *it++ = ctrInsts ? "1" : "0";

        // Custom Pintool args.
        it = std::copy(pinToolArgs.begin(), pinToolArgs.end(), it);

        // Workload.
        *it++ = "--";
        *it++ = guest_prog;

        std::vector<char *> args_c;
        for (const std::string &s : args) {
            args_c.push_back(const_cast<char *>(s.c_str()));
        }
        args_c.push_back(nullptr);

        std::stringstream cmd_ss;
        for (const std::string &arg : args) {
            cmd_ss << arg << " ";
        }
        const std::string cmd_s = cmd_ss.str();
        dprintf(guestout_fd, "Starting Pin: %s\n", cmd_s.c_str());

        execvp(args_c[0], args_c.data());
        fatal("execvp failed: %s: %s", args_c[0], std::strerror(errno));
    }

    // Close the remote end of the socket; it will remain open in the Pin
    // subprocess.
    close(req_fds[0]);  // Close read-end of request pipe.
    close(resp_fds[1]); // Close write-end of response pipe.

    // Send initial ACK.
    Message msg;
    msg.type = Message::Ack;
    DPRINTF(PinCPU, "Sending initial ACK\n");
    msg.send(reqFd);
    DPRINTF(PinCPU, "Receiving initial ACK\n");
    msg.recv(respFd);
    panic_if(msg.type != Message::Ack,
             "Received message other than ACK at pintool startup!\n");
    DPRINTF(PinCPU, "received ACK from pintool\n");

    // Pin discovers new mappings by faulting, but nothing tells it when gem5
    // tears one down, so have MemState notify us.
    tc->getProcessPtr()->memState->setUnmapCallback(
        [this](Addr vaddr, Addr size) { unmapFromPin(vaddr, size); });

    // Copy over initial state.
    syncStateToPin(true);

    // Map in code.
    mapCode();
}

void
PinCPU::mapCode()
{
    const Addr min = tc->getProcessPtr()->image.minAddr();
    const Addr max = tc->getProcessPtr()->image.maxAddr();

    // Map every part of the process image that translates. Holes in the image
    // fault, and we want to skip them; incrementing the iterator would instead
    // reattempt the same address, assuming the caller had fixed the fault. So
    // restart the generator just past a faulting page rather than advancing
    // through it. Note that the generator clamps each range to a page before
    // attempting the translation, so a faulting range's size is the size of
    // the page that faulted, which guarantees forward progress here.
    Addr vaddr = min;
    while (vaddr < max) {
        const TranslationGenPtr ptr = tc->getMMUPtr()->translateFunctional(
            vaddr, max - vaddr, tc, BaseMMU::Execute, 0);
        TranslationGenConstIterator it = ptr->begin();
        for (; it != ptr->end(); ++it) {
            const TranslationGen::Range &range = *it;
            if (range.fault != NoFault) {
                break;
            }
            // Map with the permissions the range actually has. Mapping the
            // whole image read/execute would fault again on the first write
            // to a data page, forcing Pin to replace a mapping it already
            // holds.
            int prot = PROT_READ | PROT_EXEC;
            const TranslationGenPtr w = tc->getMMUPtr()->translateFunctional(
                range.vaddr, range.size, tc, BaseMMU::Write, 0);
            if (w->begin()->fault == NoFault) {
                prot |= PROT_WRITE;
            }

            DPRINTF(PinCPU,
                    "Mapping in code range vaddr=%#x paddr=%#x size=%#x "
                    "prot=%#x\n",
                    range.vaddr, range.paddr, range.size, prot);
            Message msg;
            msg.type = Message::Map;
            msg.map.vaddr = range.vaddr;
            msg.map.paddr = range.paddr;
            msg.map.size = range.size;
            msg.map.prot = prot;
            msg.send(reqFd);
            msg.recv(respFd);
            panic_if(msg.type != Message::Ack, "unexpected response\n");
        }
        if (it == ptr->end()) {
            break;
        }
        vaddr = it->vaddr + it->size;
    }
}

void
PinCPU::activateContext(ThreadID tid)
{
    assert(tid == 0);
    assert(thread);

    schedule(tickEvent, clockEdge(Cycles(0)));
    _status = Running;
}

void
PinCPU::tick()
{
    assert(_status != Idle);
    assert(_status == Running);

    pinRun();

    if (tc->status() == ThreadContext::Halting ||
        tc->status() == ThreadContext::Halted) {
        haltContext();
    }

    // This model is functional only. Pin reports what happened, not how long
    // it took, so each interaction advances the clock by a single cycle and
    // the resulting timing is not meaningful.
    if (_status != Idle) {
        schedule(tickEvent, clockEdge(ticksToCycles(1)));
    }
}

void
PinCPU::syncStateToPin(bool full)
{
    using namespace X86ISA;
    Message msg;
    msg.type = Message::SetRegs;
    PinRegFile &rf = msg.regfile;

    // Set integer registers.
    rf.rax = tc->getReg(int_reg::Rax);
    rf.rbx = tc->getReg(int_reg::Rbx);
    rf.rcx = tc->getReg(int_reg::Rcx);
    rf.rdx = tc->getReg(int_reg::Rdx);
    rf.rdi = tc->getReg(int_reg::Rdi);
    rf.rsi = tc->getReg(int_reg::Rsi);
    rf.rbp = tc->getReg(int_reg::Rbp);
    rf.rsp = tc->getReg(int_reg::Rsp);
    rf.r8 = tc->getReg(int_reg::R8);
    rf.r9 = tc->getReg(int_reg::R9);
    rf.r10 = tc->getReg(int_reg::R10);
    rf.r11 = tc->getReg(int_reg::R11);
    rf.r12 = tc->getReg(int_reg::R12);
    rf.r13 = tc->getReg(int_reg::R13);
    rf.r14 = tc->getReg(int_reg::R14);
    rf.r15 = tc->getReg(int_reg::R15);
    rf.rip = tc->pcState().instAddr();

    // Set floating-point registers.
    for (int i = 0; i < 8; ++i) {
        const double value64 = bitsToFloat64(tc->getReg(float_reg::fpr(i)));
        storeFloat80(&rf.fprs[i][0], value64);
    }
    for (int i = 0; i < 16; ++i) {
        rf.xmms[i][0] = tc->getReg(float_reg::xmmLow(i));
        rf.xmms[i][1] = tc->getReg(float_reg::xmmHigh(i));
    }
    rf.fcw = tc->readMiscRegNoEffect(misc_reg::Fcw);
    rf.fsw = tc->readMiscRegNoEffect(misc_reg::Fsw);
    rf.ftag = tc->readMiscRegNoEffect(misc_reg::Ftag);

    // Misc registers.
    rf.rflags = getRFlags(tc);
    rf.fs = tc->readMiscRegNoEffect(misc_reg::Fs);
    rf.gs = tc->readMiscRegNoEffect(misc_reg::Gs);
    rf.fs_base = tc->readMiscRegNoEffect(misc_reg::FsBase);
    rf.gs_base = tc->readMiscRegNoEffect(misc_reg::GsBase);

    // Send message.
    msg.send(reqFd);
    msg.recv(respFd);
    panic_if(msg.type != Message::Ack,
             "Got message other than ACK for SetRegs!\n");
}

void
PinCPU::syncStateFromPin(bool full)
{
    using namespace X86ISA;

    Message msg;
    msg.type = Message::GetRegs;
    msg.send(reqFd);
    msg.recv(respFd);
    panic_if(msg.type != Message::SetRegs,
             "Got response other than SetRegs in response to GetRegs!\n");
    const PinRegFile &rf = msg.regfile;

    // Copy all integer registers.
    tc->setReg(int_reg::Rax, rf.rax);
    tc->setReg(int_reg::Rbx, rf.rbx);
    tc->setReg(int_reg::Rcx, rf.rcx);
    tc->setReg(int_reg::Rdx, rf.rdx);
    tc->setReg(int_reg::Rdi, rf.rdi);
    tc->setReg(int_reg::Rsi, rf.rsi);
    tc->setReg(int_reg::Rbp, rf.rbp);
    tc->setReg(int_reg::Rsp, rf.rsp);
    tc->setReg(int_reg::R8, rf.r8);
    tc->setReg(int_reg::R9, rf.r9);
    tc->setReg(int_reg::R10, rf.r10);
    tc->setReg(int_reg::R11, rf.r11);
    tc->setReg(int_reg::R12, rf.r12);
    tc->setReg(int_reg::R13, rf.r13);
    tc->setReg(int_reg::R14, rf.r14);
    tc->setReg(int_reg::R15, rf.r15);
    tc->pcState(rf.rip);

    // Floating-point registers.
    for (int i = 0; i < 8; ++i) {
        const double value64 = loadFloat80(rf.fprs[i]);
        const uint64_t bits64 = floatToBits64(value64);
        tc->setReg(float_reg::fpr(i), bits64);
    }
    for (int i = 0; i < 16; ++i) {
        tc->setReg(float_reg::xmmLow(i), rf.xmms[i][0]);
        tc->setReg(float_reg::xmmHigh(i), rf.xmms[i][1]);
    }
    tc->setMiscRegNoEffect(misc_reg::Fcw, rf.fcw);
    tc->setMiscRegNoEffect(misc_reg::Fsw, rf.fsw);
    tc->setMiscRegNoEffect(misc_reg::Ftag, rf.ftag);
    tc->setMiscRegNoEffect(
        misc_reg::Ftw,
        rf.ftag); // Derived from ftag, as the KVM CPU also does.

    // Misc registers.
    setRFlags(tc, rf.rflags);
    tc->setMiscRegNoEffect(misc_reg::Fs, rf.fs);
    tc->setMiscRegNoEffect(misc_reg::Gs, rf.gs);
    tc->setMiscRegNoEffect(misc_reg::FsBase, rf.fs_base);
    tc->setMiscRegNoEffect(misc_reg::GsBase, rf.gs_base);
}

void
PinCPU::pinRun()
{
    syncStateToPin(false);

    // Tell it to run.
    Message msg;
    msg.type = Message::Run;
    msg.send(reqFd);
    msg.recv(respFd);
    if (ctrInsts) {
        const std::string instcount_s = executePinCommand("instcount");
        const auto new_instcount = std::stoull(instcount_s);
        assert(*ctrInsts <= new_instcount);
        ctrInsts = new_instcount;
    }

    switch (msg.type) {
        case Message::PageFault:
            handlePageFault(msg.faultaddr);
            break;

        case Message::Syscall:
            handleSyscall();
            break;

        case Message::Cpuid:
            handleCPUID();
            break;

        case Message::Break:
            syncStateFromPin(false);
            exitSimLoopNow("pin-breakpoint");
            break;

        case Message::Ack:
            break;

        default:
            panic("unhandled run response type (%d)\n", msg.type);
    }
}

void
PinCPU::handlePageFault(Addr vaddr)
{
    syncStateFromPin(false);

    DPRINTF(PinCPU, "vaddr=%x\n", vaddr);
    assert(vaddr);
    vaddr &= ~(Addr)0xfff;

    // New approach:
    // Just start grabbing mappings until they aren't mergeable.
    struct Entry
    {
        Addr vaddr;
        Addr paddr;
        size_t size;
        int prot;
    };
    std::list<Entry> mappings;
    const MemState &mem_state = *tc->getProcessPtr()->memState;
    size_t size = PageBytes;
    if (const VMA *vma = mem_state.getVMA(vaddr)) {
        vaddr = vma->start();
        size = vma->size();
        DPRINTF(PinCPU, "VMA: %#x %#x %s\n", vma->start(), vma->end(),
                vma->getName());
    }
    DPRINTF(PinCPU, "Preparing to map starting at vaddr=%#x size=%#x\n", vaddr,
            size);

    // Collect list of page mappings with permissions.
    const auto translate_with_mode =
        [&](BaseMMU::Mode mode) -> TranslationGenPtr {
        return tc->getMMUPtr()->translateFunctional(vaddr, size, tc, mode, 0);
    };
    const auto r = translate_with_mode(BaseMMU::Read);
    const auto w = translate_with_mode(BaseMMU::Write);
    const auto x = translate_with_mode(BaseMMU::Execute);
    for (auto r_it = r->begin(), w_it = w->begin(), x_it = x->begin();
         r_it != r->end(); ++r_it, ++w_it, ++x_it) {
        assert(w_it != w->end());
        assert(x_it != x->end());
        panic_if(r_it->fault != NoFault, "Page fault: vaddr=%#x fault=%s\n",
                 r_it->vaddr, r_it->fault->name());
        Entry entry;
        entry.vaddr = r_it->vaddr;
        entry.paddr = r_it->paddr;
        entry.size = r_it->size;
        entry.prot = PROT_READ;
        if (w_it->fault == NoFault) {
            entry.prot |= PROT_WRITE;
        }
        if (x_it->fault == NoFault) {
            entry.prot |= PROT_EXEC;
        }
        mappings.push_back(entry);
    }

    // Combine into ranges.
    assert(!mappings.empty());
    for (auto it1 = mappings.begin(); std::next(it1) != mappings.end();) {
        const auto it2 = std::next(it1);

        // Can we combine the entries pointed to by it1 and it2?
        Entry &e1 = *it1;
        Entry &e2 = *it2;
        assert(e1.vaddr + e1.size == e2.vaddr);
        if (e1.prot == e2.prot && e1.paddr + e1.size == e2.paddr) {
            // Yes, can combine!
            e1.size += e2.size;
            mappings.erase(it2);
        } else {
            // No, we can't combine, so advance.
            ++it1;
        }
    }

    // Send ranges over.
    for (const Entry &e : mappings) {
        DPRINTF(PinCPU, "Mapping vaddr=%#x paddr=%#x size=%#x prot=%#x\n",
                e.vaddr, e.paddr, e.size, e.prot);
        Message msg;
        msg.type = Message::Map;
        msg.map.vaddr = e.vaddr;
        msg.map.paddr = e.paddr;
        msg.map.size = e.size;
        msg.map.prot = e.prot;
        msg.send(reqFd);
        msg.recv(respFd);
        panic_if(msg.type != Message::Ack, "unexpected response\n");
    }
}

void
PinCPU::handleSyscall()
{
    syncStateFromPin(false);

    tc->getSystemPtr()->workload->syscall(tc);
}

void
PinCPU::unmapFromPin(Addr vaddr, Addr size)
{
    // MemState outlives the Pin process, and unmaps can happen before the
    // pintool is up or after it has exited.
    if (!isPinRunning()) {
        return;
    }

    DPRINTF(PinCPU, "Pin: unmapping vaddr %#x-%#x\n", vaddr, vaddr + size);
    Message msg;
    msg.type = Message::Unmap;
    msg.map.vaddr = vaddr;
    msg.map.size = size;
    msg.send(reqFd);
    msg.recv(respFd);
    panic_if(msg.type != Message::Ack, "unexpected response\n");
}

void
PinCPU::handleCPUID()
{
    syncStateFromPin(false);

    // Get function (EAX).
    const uint32_t func = tc->getReg(X86ISA::int_reg::Rax);

    // Get index (ECX).
    const uint32_t index = tc->getReg(X86ISA::int_reg::Rcx);

    DPRINTF(PinCPU, "CPUID: EAX=0x%x ECX=0x%x\n", func, index);

    // Do CPUID.
    X86ISA::ISA *isa = dynamic_cast<X86ISA::ISA *>(tc->getIsaPtr());
    X86ISA::CpuidResult result;
    isa->cpuid->doCpuid(tc, func, index, result);

    // Set RAX, RBX, RCX, RDX.
    tc->setReg(X86ISA::int_reg::Rax, result.rax);
    tc->setReg(X86ISA::int_reg::Rbx, result.rbx);
    tc->setReg(X86ISA::int_reg::Rdx, result.rdx);
    tc->setReg(X86ISA::int_reg::Rcx, result.rcx);
}

void
PinCPU::serializeThread(CheckpointOut &cp, ThreadID tid) const
{
    assert(tid == 0);
    thread->serialize(cp);
}

std::string
PinCPU::executePinCommand(const std::string &command)
{
    fatal_if(!isPinRunning(), "PinCPU has not been started up yet!\n");
    Message msg;
    msg.type = Message::ExecCommand;
    fatal_if(command.size() >= sizeof msg.command, "Command too long!\n");
    std::strcpy(msg.command, command.c_str());
    msg.send(reqFd);
    msg.recv(respFd);
    panic_if(msg.type != Message::CommandResult,
             "Received message other than CommandResult!\n");

    size_t rem = msg.command_result_size;
    std::string s;
    while (rem > 0) {
        char buf[1024];
        const ssize_t bytes = read(respFd, buf, std::min(rem, sizeof buf));
        if (bytes <= 0) {
            panic("read failed: rem=%u\n", rem);
        }
        s.insert(s.end(), &buf[0], &buf[bytes]);
        rem -= bytes;
    }
    assert(s.size() == msg.command_result_size);
    return s;
}

bool
PinCPU::isPinRunning() const
{
    if (pinPid < 0) {
        return false;
    }
    assert(reqFd >= 0);
    assert(respFd >= 0);
    return true;
}

} // namespace X86ISA
} // namespace gem5

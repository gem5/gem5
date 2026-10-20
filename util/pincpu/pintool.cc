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

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <unordered_map>

#pragma GCC diagnostic pop
#include <cstdint>
#include <unordered_set>

#include "arch/x86/pin/regfile.h"
#include "debug.hh"
#include "ops.h"

#include "pin.H"
#include "plugin.hh"

static const char *prog;
static KNOB<std::string> log_path(KNOB_MODE_WRITEONCE, "pintool", "log", "",
                                  "specify path to log file");
static std::ofstream log_;
static KNOB<std::string> req_path(KNOB_MODE_WRITEONCE, "pintool", "req_path",
                                  "",
                                  "specify path to CPU communciation FIFO");
static KNOB<std::string> resp_path(KNOB_MODE_WRITEONCE, "pintool", "resp_path",
                                   "", "specify path to response FIFO");
static KNOB<std::string> mem_path(KNOB_MODE_WRITEONCE, "pintool", "mem_path",
                                  "", "specify path to physmem file");
static KNOB<bool> enable_inst_count(KNOB_MODE_WRITEONCE, "pintool",
                                    "inst_count", "1",
                                    "enable instruction counting");
static KNOB<bool> enable_trace(KNOB_MODE_WRITEONCE, "pintool", "trace", "0",
                               "enable instruction tracing");

static CONTEXT user_ctx;
static CONTEXT saved_guest_ctx;
static std::unordered_set<ADDRINT> guest_pages;
static ADDRINT virtual_vsyscall_base = 0;
static ADDRINT physical_vsyscall_base = 0;
static uint64_t inst_count = 0;
static std::unordered_map<ADDRINT, std::string> symbol_table;

static uint64_t pinops_count = 0;

#define EXTRA_SAFE_AND_SLOW 0

static void CopyOutRunResult(CONTEXT *ctx, const RunResult &result);

std::ofstream &
log()
{ return log_; }

static ADDRINT
getpage(ADDRINT addr)
{ return addr & ~(ADDRINT)0xFFF; }

void
ContextSwitchToGuest(CONTEXT *ctx, RunResult result)
{
    // Save the user context.
    PIN_SaveContext(ctx, &user_ctx);

    // Swap in the guest context.
    PIN_SaveContext(&saved_guest_ctx, ctx);

    // Set the return value.
    CopyOutRunResult(ctx, result);

#if EXTRA_SAFE_AND_SLOW
    // TODO: Probably too conservative.
    PIN_RemoveInstrumentation();
#endif
}

bool
IsGuestCode(ADDRINT pc)
{
    // FIXME: Don't hard-code.
    if (getpage(pc) == 0xdead0000000ULL) {
        return true;
    }
    return guest_pages.count(getpage(pc)) != 0;
}

bool
IsGuestCode(INS ins)
{ return IsGuestCode(INS_Address(ins)); }

bool
IsGuestCode(TRACE trace)
{ return IsGuestCode(TRACE_Address(trace)); }

[[noreturn]] static void
Abort()
{
    log_.close();
    PIN_ExitApplication(1);
}

static std::string
CopyUserString(ADDRINT addr)
{
    std::string s;
    while (true) {
        char c;
        // TODO: Consider using PIN_SafeCopyEx.
        if (PIN_SafeCopy(&c, (const void *)addr, 1) != 1) {
            log_ << "error: failed to copy user string\n";
            Abort();
        }
        if (c == '\0') {
            break;
        }
        s.push_back(c);
        ++addr;
    }
    return s;
}

static void
CopyOutRunResult(CONTEXT *ctx, const RunResult &result)
{
    PIN_SafeCopy((RunResult *)PIN_GetContextReg(ctx, REG_RDI), &result,
                 sizeof result);
}

static void
HandleSyscall(CONTEXT *ctx, ADDRINT pc)
{
    dbgs() << "PINTOOL: handling syscall: 0x" << std::hex << pc
           << ": number=" << std::dec << PIN_GetContextReg(ctx, REG_RAX)
           << "\n";
    assert(!IsGuestCode(pc));
    PIN_SaveContext(ctx, &user_ctx);
    PIN_SaveContext(&saved_guest_ctx, ctx);

    // Update PC.
    PIN_SetContextReg(&user_ctx, REG_RIP, pc);

    // Run result is syscall.
    RunResult result;
    result.result = RunResult::RUNRESULT_SYSCALL;
    CopyOutRunResult(ctx, result);
    PIN_ExecuteAt(ctx);
}

static void
HandleCPUID(CONTEXT *ctx, ADDRINT next_pc)
{
    dbgs() << "PINTOOL: handling cpuid: 0x" << std::hex << next_pc << "\n";

    // TODO: Share with HandleSyscall.
    assert(!IsGuestCode(next_pc));
    PIN_SaveContext(ctx, &user_ctx);
    PIN_SaveContext(&saved_guest_ctx, ctx);

    // Update PC.
    PIN_SetContextReg(&user_ctx, REG_RIP, next_pc);

    // Run result is cpyid.
    RunResult result;
    result.result = RunResult::RUNRESULT_CPUID;
    CopyOutRunResult(ctx, result);
    PIN_ExecuteAt(ctx);
}

static ADDRINT
HandleFSGSAccess(ADDRINT effaddr)
{
#if 0
    dbgs() << "Translating FS/GS access: 0x" << effaddr << "\n";
#endif
    return effaddr;
}

static std::unordered_map<ADDRINT, PinOp> pinops_blacklist;

static void
HandleOp_RESETUSER(const CONTEXT *guest_ctx)
{ PIN_SaveContext(guest_ctx, &user_ctx); }

static ADDRINT
HandleOp_GET_INSTCOUNT()
{ return inst_count; }

static void
HandleOp_GetPath(const char *user_ptr, size_t size, std::string path)
{
    path.push_back('\0');
    if (path.size() > size) {
        std::cerr << "PINTOOL: path too large to fit in guest buffer: " << path
                  << "\n";
        Abort();
    }
    if (PIN_SafeCopy((void *)user_ptr, path.data(), path.size()) !=
        path.size()) {
        std::cerr << "PINTOOL: error: failed to copy\n";
        Abort();
    }
}

static void
HandleOp_GET_REQPATH(const char *user_ptr, size_t size)
{ HandleOp_GetPath(user_ptr, size, req_path.Value()); }

static void
HandleOp_GET_RESPPATH(const char *user_ptr, size_t size)
{ HandleOp_GetPath(user_ptr, size, resp_path.Value()); }

static void
HandleOp_GET_MEMPATH(const char *user_ptr, size_t size)
{ HandleOp_GetPath(user_ptr, size, mem_path.Value()); }

static void
HandleOp_SET_VSYSCALL_BASE(void *virt, void *phys)
{
    virtual_vsyscall_base = (ADDRINT)virt;
    physical_vsyscall_base = (ADDRINT)phys;
}

[[noreturn]] static void
HandleOp_EXIT(int32_t code)
{
    std::cerr << "Exiting " << std::dec << code << "\n";
    PIN_ExitApplication(code);
    std::abort(); // TODO: Unreachable
}

[[noreturn]] static void
HandleOp_ABORT(const char *user_msg, size_t line, void *pc)
{
    std::cerr << "Aborting at pc " << pc << "\n";
    std::cerr << CopyUserString(reinterpret_cast<ADDRINT>(user_msg)) << ":"
              << line << "\n";
    PIN_ExitApplication(1);
    std::abort(); // TODO: Unreachable.
}

static void
HandleOp_RUN(const CONTEXT *guest_ctx_ptr, ADDRINT next_pc)
{
    PIN_SaveContext(guest_ctx_ptr, &saved_guest_ctx);
    PIN_SetContextReg(&saved_guest_ctx, REG_RIP, next_pc);
    std::cerr << __FUNCTION__ << ": switching to user context\n";
    PIN_ExecuteAt(&user_ctx);
    std::abort(); // TODO: UNREACHABLE
}

static void
HandleOp_SET_REGS(const PinRegFile *user_regfile_ptr)
{
    PinRegFile rf;
    if (PIN_SafeCopy(&rf, user_regfile_ptr, sizeof rf) != sizeof rf) {
        std::cerr << "PINTOOL: Failed to copy regfile\n";
        Abort();
    }
    const auto set_reg = [](REG reg, uint64_t value) {
        PIN_SetContextReg(&user_ctx, reg, value);
    };

    // Set integer registers.
    set_reg(REG_RAX, rf.rax);
    set_reg(REG_RBX, rf.rbx);
    set_reg(REG_RCX, rf.rcx);
    set_reg(REG_RDX, rf.rdx);
    set_reg(REG_RDI, rf.rdi);
    set_reg(REG_RSI, rf.rsi);
    set_reg(REG_RSP, rf.rsp);
    set_reg(REG_RBP, rf.rbp);
    set_reg(REG_R8, rf.r8);
    set_reg(REG_R9, rf.r9);
    set_reg(REG_R10, rf.r10);
    set_reg(REG_R11, rf.r11);
    set_reg(REG_R12, rf.r12);
    set_reg(REG_R13, rf.r13);
    set_reg(REG_R14, rf.r14);
    set_reg(REG_R15, rf.r15);
    set_reg(REG_RIP, rf.rip);

    // Set floating-point registers.
    for (int i = 0; i < 8; ++i) {
        PIN_SetContextRegval(&user_ctx, REG(REG_ST0 + i),
                             (const uint8_t *)&rf.fprs[i]);
    }
    for (int i = 0; i < 16; ++i) {
        PIN_SetContextRegval(&user_ctx, REG(REG_XMM0 + i),
                             (const uint8_t *)&rf.xmms[i]);
    }
    set_reg(REG_FPCW, rf.fcw);
    set_reg(REG_FPSW, rf.fsw);
    set_reg(REG_FPTAG, rf.ftag);

    // Misc regs.
    set_reg(REG_RFLAGS, rf.rflags);
    set_reg(REG_SEG_FS, rf.fs);
    set_reg(REG_SEG_GS, rf.gs);
    set_reg(REG_SEG_FS_BASE, rf.fs_base);
    set_reg(REG_SEG_GS_BASE, rf.gs_base);

    dbgs() << "DEBUG: setting REG_SEG_FS_BASE to 0x" << std::hex << rf.fs_base
           << "\n";
}

static void
HandleOp_GET_REGS(PinRegFile *user_regfile_ptr)
{
    PinRegFile rf;
    std::memset(&rf, 0, sizeof rf);

    const auto get_reg = [](REG reg, auto &value) {
        value = PIN_GetContextReg(&user_ctx, reg);
    };

    // Get integer registers.
    get_reg(REG_RAX, rf.rax);
    get_reg(REG_RBX, rf.rbx);
    get_reg(REG_RCX, rf.rcx);
    get_reg(REG_RDX, rf.rdx);
    get_reg(REG_RDI, rf.rdi);
    get_reg(REG_RSI, rf.rsi);
    get_reg(REG_RSP, rf.rsp);
    get_reg(REG_RBP, rf.rbp);
    get_reg(REG_R8, rf.r8);
    get_reg(REG_R9, rf.r9);
    get_reg(REG_R10, rf.r10);
    get_reg(REG_R11, rf.r11);
    get_reg(REG_R12, rf.r12);
    get_reg(REG_R13, rf.r13);
    get_reg(REG_R14, rf.r14);
    get_reg(REG_R15, rf.r15);
    get_reg(REG_RIP, rf.rip);

    // Get floating-point registers.
    for (int i = 0; i < 8; ++i) {
        PIN_GetContextRegval(&user_ctx, REG(REG_ST0 + i),
                             (uint8_t *)&rf.fprs[i]);
    }
    for (int i = 0; i < 16; ++i) {
        PIN_GetContextRegval(&user_ctx, REG(REG_XMM0 + i),
                             (uint8_t *)&rf.xmms[i]);
    }
    get_reg(REG_FPCW, rf.fcw);
    get_reg(REG_FPSW, rf.fsw);
    get_reg(REG_FPTAG, rf.ftag);

    // Misc regs.
    get_reg(REG_RFLAGS, rf.rflags);
    get_reg(REG_SEG_FS, rf.fs);
    get_reg(REG_SEG_GS, rf.gs);
    get_reg(REG_SEG_FS_BASE, rf.fs_base);
    get_reg(REG_SEG_GS_BASE, rf.gs_base);

    // Copy out.
    if (PIN_SafeCopy(user_regfile_ptr, &rf, sizeof rf) != sizeof rf) {
        std::cerr << "PINTOOL: failed to copy regfile\n";
        Abort();
    }

    dbgs() << "DEBUG: getting FS_BASE: 0x" << std::hex << rf.fs_base << "\n";
}

static void
HandleOp_ADD_SYMBOL(ADDRINT name_vptr, ADDRINT vaddr)
{
    const std::string name = CopyUserString(name_vptr);
    symbol_table[vaddr] = name;
    // FIXME: Disabled because it's high-overhead.
    // Should instead add a new pinop so gem5 can control when this happens.
    // PIN_RemoveInstrumentation(); // So that any analyses depending on
    // symbols will get to re-analyze with symbols.
}

static std::string
ExecCommand(const std::string &cmd, const std::vector<std::string> &args)
{
    std::cerr << __FUNCTION__ << ": executing command: " << cmd;
    for (const std::string &arg : args) {
        std::cerr << " " << arg;
    }
    std::cerr << "\n";
    for (Plugin *plugin : plugins) {
        if (plugin->enabled()) {
            std::string result;
            if (plugin->command(cmd, args, result)) {
                return result;
            }
        }
    }
    std::cerr << __FUNCTION__ << ": no plugin handled command: " << cmd
              << "\n";
    Abort();
}

static std::string gCommandResult;

static ADDRINT
HandleOp_EXEC_COMMAND(ADDRINT cmd_vptr)
{
    const std::string cmdline = CopyUserString(cmd_vptr);

    // Tokenize.
    std::vector<std::string> tokens;
    tokens.emplace_back();
    for (char c : cmdline) {
        if (std::isspace(c)) {
            tokens.emplace_back();
        } else {
            tokens.back().push_back(c);
        }
    }
    if (tokens.empty()) {
        std::cerr << __func__ << ": got empty command!\n";
        Abort();
    }
    const std::string cmd = tokens.front();
    tokens.erase(tokens.begin());

    // Try to find plugin to handle the command.
    gCommandResult = ExecCommand(cmd, tokens);
    return gCommandResult.size();
}

static void
HandleOp_READ_COMMAND_RESULT(ADDRINT buf_vptr, ADDRINT idx, ADDRINT len)
{
    assert(idx + len <= gCommandResult.size());
    PIN_SafeCopy(reinterpret_cast<void *>(buf_vptr),
                 gCommandResult.data() + idx, len);
}

const std::string *
GetSymbol(ADDRINT addr)
{
    const auto it = symbol_table.find(addr);
    return it == symbol_table.end() ? nullptr : &it->second;
}

static void
Instrument_Instruction_PinOps(INS ins, void *)
{
    const ADDRINT pc = INS_Address(ins);
    const auto it = pinops_blacklist.find(pc);
    if (it == pinops_blacklist.end()) {
        return;
    }

    const PinOp op = it->second;

    dbgs() << "PINTOOL: instrumenting pinop instruction: 0x" << std::hex
           << INS_Address(ins) << ": op=" << std::dec << op << "\n";

    assert(INS_MemoryOperandCount(ins) == 1);

    switch (op) {
        case PinOp::OP_RESETUSER:
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE,
                                     (AFUNPTR)HandleOp_RESETUSER,
                                     IARG_CONST_CONTEXT, IARG_END);
            break;

        case PinOp::OP_GET_INSTCOUNT:
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE,
                                     (AFUNPTR)HandleOp_GET_INSTCOUNT,
                                     IARG_RETURN_REGS, REG_RAX, IARG_END);
            break;

        case PinOp::OP_GET_REQPATH:
            INS_InsertPredicatedCall(
                ins, IPOINT_BEFORE, (AFUNPTR)HandleOp_GET_REQPATH,
                IARG_REG_VALUE, REG_RDI, IARG_REG_VALUE, REG_RSI, IARG_END);
            break;

        case PinOp::OP_GET_RESPPATH:
            INS_InsertPredicatedCall(
                ins, IPOINT_BEFORE, (AFUNPTR)HandleOp_GET_RESPPATH,
                IARG_REG_VALUE, REG_RDI, IARG_REG_VALUE, REG_RSI, IARG_END);
            break;

        case PinOp::OP_GET_MEMPATH:
            INS_InsertPredicatedCall(
                ins, IPOINT_BEFORE, (AFUNPTR)HandleOp_GET_MEMPATH,
                IARG_REG_VALUE, REG_RDI, IARG_REG_VALUE, REG_RSI, IARG_END);
            break;

        case PinOp::OP_SET_VSYSCALL_BASE:
            INS_InsertPredicatedCall(
                ins, IPOINT_BEFORE, (AFUNPTR)HandleOp_SET_VSYSCALL_BASE,
                IARG_REG_VALUE, REG_RDI, IARG_REG_VALUE, REG_RSI, IARG_END);
            break;

        case PinOp::OP_EXIT:
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE,
                                     (AFUNPTR)HandleOp_EXIT, IARG_REG_VALUE,
                                     REG_RDI, IARG_END);
            break;

        case PinOp::OP_ABORT:
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE,
                                     (AFUNPTR)HandleOp_ABORT, IARG_REG_VALUE,
                                     REG_RDI, IARG_REG_VALUE, REG_RSI,
                                     IARG_REG_VALUE, REG_RDX, IARG_END);
            break;

        case PinOp::OP_RUN:
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE, (AFUNPTR)HandleOp_RUN,
                                     IARG_CONST_CONTEXT, IARG_ADDRINT,
                                     pc + INS_Size(ins), IARG_END);
            break;

        case PinOp::OP_SET_REGS:
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE,
                                     (AFUNPTR)HandleOp_SET_REGS,
                                     IARG_REG_VALUE, REG_RDI, IARG_END);
            break;

        case PinOp::OP_GET_REGS:
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE,
                                     (AFUNPTR)HandleOp_GET_REGS,
                                     IARG_REG_VALUE, REG_RDI, IARG_END);
            break;

        case PinOp::OP_ADD_SYMBOL:
            INS_InsertPredicatedCall(
                ins, IPOINT_BEFORE, (AFUNPTR)HandleOp_ADD_SYMBOL,
                IARG_REG_VALUE, REG_RDI, IARG_REG_VALUE, REG_RSI, IARG_END);
            break;

        case PinOp::OP_EXEC_COMMAND:
            INS_InsertPredicatedCall(
                ins, IPOINT_BEFORE, (AFUNPTR)HandleOp_EXEC_COMMAND,
                IARG_REG_VALUE, REG_RDI, IARG_RETURN_REGS, REG_RAX, IARG_END);
            break;

        case PinOp::OP_READ_COMMAND_RESULT:
            INS_InsertPredicatedCall(
                ins, IPOINT_BEFORE, (AFUNPTR)HandleOp_READ_COMMAND_RESULT,
                IARG_REG_VALUE, REG_RDI, IARG_REG_VALUE, REG_RSI,
                IARG_REG_VALUE, REG_RDX, IARG_END);
            break;

        default:
            std::cerr << "PINTOOL: fatal: unimplemented pinop " << std::dec
                      << op << "\n";
            Abort();
    }

    INS_Delete(ins);
}

// TODO: Break this into mini-instrumentation functions.
static void
Instruction(INS ins, void *)
{
    if (guest_pages.empty()) {
        IMG img = APP_ImgHead();
        assert(IMG_Valid(img));
        assert(IMG_IsMainExecutable(img));
        assert(IMG_IsStaticExecutable(img));
        for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec)) {
            if (!SEC_Mapped(sec)) {
                continue;
            }
            log_ << "section: " << std::hex << SEC_Address(sec) << "\n";
            const ADDRINT start = SEC_Address(sec);
            const ADDRINT end = start + SEC_Size(sec);
            for (ADDRINT page = start & ~(ADDRINT)0xFFF; page < end;
                 page += 0x1000) {
                guest_pages.insert(page);
            }
        }
        assert(!guest_pages.empty());
    }

    const ADDRINT addr = INS_Address(ins);
    if (IsGuestCode(addr)) {
        return;
    }

    // Application instruction.
    dbgs() << "INSTRUMENT: 0x" << std::hex << INS_Address(ins) << std::endl;

    // Instrument system calls. Replace them with traps into gem5.
    if (INS_IsSyscall(ins)) {
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)HandleSyscall,
                       IARG_CONTEXT, IARG_ADDRINT,
                       INS_Address(ins) + INS_Size(ins), IARG_END);
    }

    // Handle CPUIDs.
    if (INS_Opcode(ins) == XED_ICLASS_CPUID) {
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)HandleCPUID, IARG_CONTEXT,
                       IARG_ADDRINT, INS_Address(ins) + INS_Size(ins),
                       IARG_END);
    }

    // Accesses via the FS_BASE and GS_BASE registers are sensitive.
    for (uint32_t i = 0; i < INS_MemoryOperandCount(ins); ++i) {
        if (!(INS_MemoryOperandIsRead(ins, i) ||
              INS_MemoryOperandIsWritten(ins, i))) {
            continue;
        }
        REG seg_reg = INS_OperandMemorySegmentReg(
            ins, INS_MemoryOperandIndexToOperandIndex(ins, i));
        if (!REG_valid(seg_reg)) {
            continue;
        }
        switch (seg_reg) {
            case REG_SEG_FS:
            case REG_SEG_GS:
                break;
            default:
                std::cerr << "PINTOOL: error: unexpected segment register: "
                          << REG_StringShort(seg_reg) << "\n";
                std::abort();
        }
        dbgs() << "PINTOOL: found sensitive FS/GS instruction: 0x"
               << INS_Address(ins) << ": " << INS_Disassemble(ins) << "\n";
        // TODO: Shuold probably be predicated.
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)HandleFSGSAccess,
                       IARG_MEMORYOP_EA, i, IARG_RETURN_REGS, REG_INST_G0 + i,
                       IARG_CALL_ORDER, CALL_ORDER_LAST, IARG_END);
        INS_RewriteMemoryOperand(ins, i, (REG)(REG_INST_G0 + i));
    }
}

static ADDRINT
HandleVsyscallAccess(ADDRINT effaddr, ADDRINT effsize, ADDRINT next_pc,
                     const CONTEXT *ctx)
{
    assert(virtual_vsyscall_base && physical_vsyscall_base);
    assert((virtual_vsyscall_base <= effaddr &&
            effaddr + effsize <= virtual_vsyscall_base + 0x1000) ||
           effaddr + effsize <= virtual_vsyscall_base ||
           virtual_vsyscall_base + 0x1000 <= effaddr);
    if (virtual_vsyscall_base <= effaddr &&
        effaddr + effsize <= virtual_vsyscall_base + 0x1000) {
        const ADDRINT offset = effaddr - virtual_vsyscall_base;
        assert(physical_vsyscall_base);
        return physical_vsyscall_base + offset;
    } else {
        return effaddr;
    }
}

static std::unordered_set<ADDRINT> vsyscall_blacklist;

// Fixup vsyscalls.
static void
Instruction_Vsyscall(INS ins, void *)
{
    if (IsGuestCode(ins) || vsyscall_blacklist.count(INS_Address(ins)) == 0) {
        return;
    }

    dbgs()
        << "PINTOOL: instrumenting instruction that has accessed vsyscall: 0x"
        << INS_Address(ins) << "\n";

    // FIXME: If we have a FS/GS access too this breaks.
    for (uint32_t i = 0; i < INS_MemoryOperandCount(ins); ++i) {
        INS_InsertPredicatedCall(
            ins, IPOINT_BEFORE, (AFUNPTR)HandleVsyscallAccess,
            IARG_MEMORYOP_EA, i, IARG_MEMORYOP_SIZE, i, IARG_ADDRINT,
            INS_Address(ins) + INS_Size(ins), IARG_CONST_CONTEXT,
            IARG_RETURN_REGS, REG_INST_G0 + i, IARG_CALL_ORDER,
            CALL_ORDER_LAST, IARG_END);
        INS_RewriteMemoryOperand(ins, i, (REG)(REG_INST_G0 + i));
    }
}

static void
HandleInstCount(ADDRINT num_insts)
{ inst_count += num_insts; }

static void
Instrument_Trace_InstCount(TRACE trace, void *)
{
    if (IsGuestCode(trace)) {
        return;
    }
    for (BBL bbl = TRACE_BblHead(trace); BBL_Valid(bbl); bbl = BBL_Next(bbl)) {
        BBL_InsertCall(bbl, IPOINT_BEFORE, (AFUNPTR)HandleInstCount,
                       IARG_ADDRINT, BBL_NumIns(bbl), IARG_END);
    }
}

static void
HandleTrace(ADDRINT pc)
{
    std::cerr << "TRACE: 0x" << std::hex << pc << std::endl;
    log_ << "TRACE: 0x" << std::hex << pc << std::endl;
}

static void
Instruction_Trace(INS ins, void *)
{
    if (IsGuestCode(ins)) {
        return;
    }
    INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)HandleTrace, IARG_INST_PTR,
                   IARG_END);
}

static bool
InterceptSEGV(THREADID tid, int32_t sig, CONTEXT *ctx, bool has_handler,
              const EXCEPTION_INFO *info, void *)
{
    std::cerr << "PINTOOL: Encountered SEGV: " << info->ToString() << "\n";

    const auto code = PIN_GetExceptionCode(info);
    const auto ex_class = PIN_GetExceptionClass(code);
    if (ex_class != EXCEPTCLASS_ACCESS_FAULT) {
        std::cerr << "PINTOOL: unexpected exception class (" << ex_class
                  << ")\n";
        return true;
    }

    assert(info->IsAccessFault());

    ADDRINT fault_pc = info->GetExceptAddress();
    ADDRINT fault_addr;
    [[maybe_unused]] const bool fault_addr_result =
        info->GetFaultyAccessAddress(&fault_addr);
    assert(fault_addr_result);

    std::cerr << "PINTOOL: SEGV at address 0x" << std::hex << fault_addr
              << "\n";
    std::cerr << "    at pc 0x" << fault_pc << "\n";

    if (fault_addr == 0) {
        std::cerr << "PINTOOL: null pointer dereference; aborting\n";
        return true;
    }

    // Was this segfault in guest code?
    if (IsGuestCode(fault_pc)) {
        // If this is a pinop fault, then we just need to reinstrument it and
        // add it to the pinop instruction list.
        if (is_pinop_addr((void *)fault_addr)) {
            std::cerr << "PINTOOL: detected new pinop instruction: 0x"
                      << fault_pc << "\n";
            pinops_blacklist[fault_pc] =
                static_cast<PinOp>(fault_addr - pinops_addr_base);
            PIN_RemoveInstrumentation(); // FIXME: Try just removing
                                         // insturmentation in range, benchmark
                                         // performance diff.
            return false;
        }

        // Otherwise, we have an unknown guest fault.
        std::cerr << "PINTOOL: guest faulted, aborting\n";
        return true;
    }

    RunResult result;

    // Was this a vsyscall access?
    // NOTE: The proper thing to do here if the virtual vsyscall base hasn't
    // been set yet is simply to deliver a page fault back to gem5.
    if (virtual_vsyscall_base && virtual_vsyscall_base <= fault_addr &&
        fault_addr < virtual_vsyscall_base + 0x1000) {
        // Detected vsyscall access.
        // Trick Pin into re-instrumenting the instruction.
        std::cerr << "PINTOOL: detected vsyscall access\n";
        vsyscall_blacklist.insert(fault_pc);
        PIN_RemoveInstrumentation(); // FIXME: Try out removing instrumentation
                                     // in range again.
        return false;
    } else {
        result.result = RunResult::RUNRESULT_PAGEFAULT;
        result.addr = fault_addr;
    }

    ContextSwitchToGuest(ctx, result);

    return false;
}

static void
usage(std::ostream &os)
{
    std::cerr << prog << ": gem5 pin CPU client\n";
    std::cerr << KNOB_BASE::StringKnobSummary() << "\n";
}

static void
Fini(int32_t code, void *)
{
    log_ << "Exiting: code = " << code << std::endl;
    log_ << prog << ": Finished running the program, Pin exiting!"
         << std::endl;
    log_ << "STATS: total pinops: " << std::dec << pinops_count << std::endl;
    log_ << "inst-count " << inst_count << std::endl;
}

template <class T>
static int
CheckPathArg(const T &arg)
{
    const std::string &value = arg.Value();
    if (value.empty()) {
        std::cerr << "error: required option: " << arg.Name() << "\n";
        return -1;
    }
    OS_FILE_ATTRIBUTES attr;
    if (OS_GetFileAttributes(value.c_str(), &attr).generic_err !=
        OS_RETURN_CODE_NO_ERROR) {
        std::cerr << "error: failed to open file: " << value << "\n";
        return -1;
    }
    return 0;
}

static void
HandleOOM(size_t size, void *)
{
    std::cerr << "Pin ran out of memory! Allocation size: " << size << "\n";

    int fd = open("/proc/self/maps", O_RDONLY);
    if (fd < 0) {
        std::cerr << "error: failed to open /proc/self/maps\n";
        return;
    }
    while (true) {
        char buf[1024];
        const ssize_t bytes = read(fd, buf, sizeof buf - 1);
        if (bytes < 0) {
            std::cerr << "error reading /proc/self/maps\n";
            return;
        }
        if (bytes == 0) {
            break;
        }
        buf[bytes] = '\0';
        std::cerr << buf;
    }
    close(fd);
}

int
main(int argc, char *argv[])
{
#if 0
    PIN_InitSymbols();
#endif

    prog = argv[0];
    if (PIN_Init(argc, argv)) {
        usage(std::cerr);
        return EXIT_FAILURE;
    }

    if (log_path.Value().empty()) {
        std::cerr << "error: required option: -log\n";
        return EXIT_FAILURE;
    }
    log_.open(log_path.Value());

    if (CheckPathArg(req_path) < 0) {
        return EXIT_FAILURE;
    }
    if (CheckPathArg(resp_path) < 0) {
        return EXIT_FAILURE;
    }

    OS_FILE_ATTRIBUTES attr;
    if (mem_path.Value().empty()) {
        std::cerr << "error: required option: -mem_path\n";
        return EXIT_FAILURE;
    }
    if (OS_GetFileAttributes(mem_path.Value().c_str(), &attr).generic_err !=
        OS_RETURN_CODE_NO_ERROR) {
        std::cerr << "error: failed to open file: " << mem_path.Value()
                  << "\n";
        return EXIT_FAILURE;
    }

    // TODO: Reason better about ordering here.
    // INS_AddInstrumentFunction(Instrument_Instruction_PrintCall, nullptr);

    if (enable_trace.Value()) {
        INS_AddInstrumentFunction(Instruction_Trace, nullptr);
    }
    INS_AddInstrumentFunction(Instruction_Vsyscall, nullptr);

    // TODO: Remove this!
    if (enable_inst_count.Value()) {
        TRACE_AddInstrumentFunction(Instrument_Trace_InstCount, nullptr);
    }

    // Register plugins.
    std::map<int, std::vector<Plugin *>> prioritized_plugins;
    for (Plugin *plugin : plugins) {
        if (plugin->enabled()) {
            prioritized_plugins[plugin->priority()].push_back(plugin);
        }
    }
    for (const auto &[_, plugins] : prioritized_plugins) {
        for (Plugin *plugin : plugins) {
            std::cerr << "Registering plugin '" << plugin->name() << "'\n";
            plugin->reg();
        }
    }

    INS_AddInstrumentFunction(Instruction, nullptr);
    INS_AddInstrumentFunction(Instrument_Instruction_PinOps, nullptr);
    PIN_AddFiniFunction(Fini, nullptr);

    PIN_InterceptSignal(SIGSEGV, InterceptSEGV, nullptr);

    PIN_AddOutOfMemoryFunction(HandleOOM, nullptr);

#if 0
    PIN_SetSmcSupport(SMC_DISABLE);
#endif

    std::cerr << "runtime: starting program\n";

    PIN_StartProgram();
}

/*
 * Copyright (c) 2018 ARM Limited
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

#include "arch/arm/semihosting.hh"

#include <cstdio>

#include "arch/arm/utility.hh"
#include "base/logging.hh"
#include "base/time.hh"
#include "debug/Semihosting.hh"
#include "dev/serial/serial.hh"
#include "mem/physical.hh"
#include "mem/port_proxy.hh"
#include "params/ArmSemihosting.hh"
#include "sim/byteswap.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

const std::map<uint32_t, ArmSemihosting::SemiCall> ArmSemihosting::calls{
    { 0x01, { "SYS_OPEN", &ArmSemihosting::callOpen, 3, 3 } },
    { 0x02, { "SYS_CLOSE", &ArmSemihosting::callClose, 1, 1 } },

    // Write(C|0) are special since we want to read the character
    // manually. We therefore declare them as having 0 params.
    { 0x03, { "SYS_WRITEC", &ArmSemihosting::callWriteC, 0, 0 } },
    { 0x04, { "SYS_WRITE0", &ArmSemihosting::callWrite0, 1, 1 } },

    { 0x05, { "SYS_WRITE", &ArmSemihosting::callWrite, 3, 3 } },
    { 0x06, { "SYS_READ", &ArmSemihosting::callRead, 3, 3 } },
    { 0x07, { "SYS_READC", &ArmSemihosting::callReadC, 0, 0 } },
    { 0x08, { "SYS_ISERROR", &ArmSemihosting::callIsError, 1, 1 } },
    { 0x09, { "SYS_ISTTY", &ArmSemihosting::callIsTTY, 1, 1 } },
    { 0x0A, { "SYS_SEEK", &ArmSemihosting::callSeek, 2, 2 } },
    { 0x0C, { "SYS_FLEN", &ArmSemihosting::callFLen, 1, 1 } },
    { 0x0D, { "SYS_TMPNAM", &ArmSemihosting::callTmpNam, 3, 3 } },
    { 0x0E, { "SYS_REMOVE", &ArmSemihosting::callRemove, 2, 2} },
    { 0x0F, { "SYS_RENAME", &ArmSemihosting::callRename, 4, 4} },
    { 0x10, { "SYS_CLOCK", &ArmSemihosting::callClock, 0, 0} },
    { 0x11, { "SYS_TIME", &ArmSemihosting::callTime, 0, 0} },
    { 0x12, { "SYS_SYSTEM", &ArmSemihosting::callSystem, 2, 2} },
    { 0x13, { "SYS_ERRNO", &ArmSemihosting::callErrno, 0, 0 } },
    { 0x15, { "SYS_GET_CMDLINE", &ArmSemihosting::callGetCmdLine, 2, 2} },
    { 0x16, { "SYS_HEAPINFO", &ArmSemihosting::callHeapInfo, 1, 1} },

    // Exit is special and requires custom handling in aarch32.
    { 0x18, { "SYS_EXIT", &ArmSemihosting::callExit, 0, 2 } },
    { 0x20, { "SYS_EXIT_EXTENDED", &ArmSemihosting::callExitExtended, 2, 2 } },

    { 0x30, { "SYS_ELAPSED", &ArmSemihosting::callElapsed, 0, 0 } },
    { 0x31, { "SYS_TICKFREQ", &ArmSemihosting::callTickFreq, 0, 0 } },
};

const std::vector<const char *> ArmSemihosting::fmodes{
    "r", "rb", "r+", "r+b",
    "w", "wb", "w+", "w+b",
    "a", "ab", "a+", "a+b",
};

const std::map<uint64_t, const char *> ArmSemihosting::exitCodes{
    { 0x20000, "semi:ADP_Stopped_BranchThroughZero" },
    { 0x20001, "semi:ADP_Stopped_UndefinedInstr" },
    { 0x20002, "semi:ADP_Stopped_SoftwareInterrupt" },
    { 0x20003, "semi:ADP_Stopped_PrefetchAbort" },
    { 0x20004, "semi:ADP_Stopped_DataAbort" },
    { 0x20005, "semi:ADP_Stopped_AddressException" },
    { 0x20006, "semi:ADP_Stopped_IRQ" },
    { 0x20007, "semi:ADP_Stopped_FIQ" },

    { 0x20020, "semi:ADP_Stopped_BreakPoint" },
    { 0x20021, "semi:ADP_Stopped_WatchPoint" },
    { 0x20022, "semi:ADP_Stopped_StepComplete" },
    { 0x20023, "semi:ADP_Stopped_RunTimeErrorUnknown" },
    { 0x20024, "semi:ADP_Stopped_InternalError" },
    { 0x20025, "semi:ADP_Stopped_UserInterruption" },
    { 0x20026, "semi:ADP_Stopped_ApplicationExit" },
    { 0x20027, "semi:ADP_Stopped_StackOverflow" },
    { 0x20028, "semi:ADP_Stopped_DivisionByZero" },
    { 0x20029, "semi:ADP_Stopped_DivisionByZero" },
};


const std::vector<uint8_t> ArmSemihosting::features{
    0x53, 0x48, 0x46, 0x42, // Magic
    0x3,                    // EXT_EXIT_EXTENDED, EXT_STDOUT_STDERR
};

const std::map<const std::string, FILE *> ArmSemihosting::stdioMap{
    {"cin",    ::stdin},
    {"stdin",  ::stdin},
    {"cout",   ::stdout},
    {"stdout", ::stdout},
    {"cerr",   ::stderr},
    {"stderr", ::stderr},
};

ArmSemihosting::ArmSemihosting(const ArmSemihostingParams *p)
    : SimObject(p),
      cmdLine(p->cmd_line),
      memReserve(p->mem_reserve),
      stackSize(p->stack_size),
      timeBase([p]{ struct tm t = p->time; return mkutctime(&t); }()),
      tickShift(calcTickShift()),
      semiErrno(0),
      stdin(getSTDIO("stdin", p->stdin, "r")),
      stdout(getSTDIO("stdout", p->stdout, "w")),
      stderr(p->stderr == p->stdout ?
             stdout : getSTDIO("stderr", p->stderr, "w"))
{
    // Create an empty place-holder file for position 0 as semi-hosting
    // calls typically expect non-zero file handles.
    files.push_back(nullptr);

    if (tickShift > 0)
        inform("Semihosting: Shifting elapsed ticks by %i bits.",
               tickShift);
}

uint64_t
ArmSemihosting::call64(ThreadContext *tc, uint32_t op, uint64_t param)
{
    const SemiCall *call = getCall(op, true);
    if (!call) {
        warn("Unknown aarch64 semihosting call: op = 0x%x, param = 0x%x",
             op, param);

        return (uint64_t)-1;
    } else if (!call->implemented64()) {
        warn("Unimplemented aarch64 semihosting call: "
             "%s (op = 0x%x, param = 0x%x)",
             call->name, op, param);

        return (uint64_t)-1;
    }

    std::vector<uint64_t> argv(call->argc64 + 1);
    PortProxy &proxy = physProxy(tc);
    ByteOrder endian = ArmISA::byteOrder(tc);

    DPRINTF(Semihosting, "Semihosting call64: %s(0x%x)\n", call->name, param);
    argv[0] = param;
    for (int i = 0; i < call->argc64; ++i) {
        argv[i + 1] = proxy.readGtoH<uint64_t>(param + i * 8, endian);
        DPRINTF(Semihosting, "\t: 0x%x\n", argv[i + 1]);
    }

    auto ret_errno = (this->*call->call)(tc, true, argv);
    semiErrno = ret_errno.second;
    DPRINTF(Semihosting, "\t ->: 0x%x, %i\n",
            ret_errno.first, ret_errno.second);
    return ret_errno.first;
}

uint32_t
ArmSemihosting::call32(ThreadContext *tc, uint32_t op, uint32_t param)
{
    const SemiCall *call = getCall(op, false);
    if (!call) {
        warn("Unknown aarch32 semihosting call: op = 0x%x, param = 0x%x",
             op, param);

        return (uint32_t)-1;
    } else if (!call->implemented32()) {
        warn("Unimplemented aarch32 semihosting call: "
             "%s (op = 0x%x, param = 0x%x)",
             call->name, op, param);

        return (uint32_t)-1;
    }

    std::vector<uint64_t> argv(call->argc32 + 1);
    PortProxy &proxy = physProxy(tc);
    ByteOrder endian = ArmISA::byteOrder(tc);

    DPRINTF(Semihosting, "Semihosting call32: %s(0x%x)\n", call->name, param);
    argv[0] = param;
    for (int i = 0; i < call->argc32; ++i) {
        argv[i + 1] = proxy.readGtoH<uint32_t>(param + i * 4, endian);
        DPRINTF(Semihosting, "\t: 0x%x\n", argv[i + 1]);
    }

    auto ret_errno = (this->*call->call)(tc, false, argv);
    semiErrno = ret_errno.second;
    DPRINTF(Semihosting, "\t ->: 0x%x, %i\n",
            ret_errno.first, ret_errno.second);
    return ret_errno.first;
}

void
ArmSemihosting::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(semiErrno);

    paramOut(cp, "num_files", files.size());
    for (int i = 0; i < files.size(); i++) {
        // File closed?
        if (!files[i])
            continue;

        files[i]->serializeSection(cp, csprintf("file%i", i));
    }
}

void
ArmSemihosting::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(semiErrno);

    size_t num_files;
    paramIn(cp, "num_files", num_files);
    files.resize(num_files);
    for (int i = 0; i < num_files; i++)
        files[i] = FileBase::create(*this, cp, csprintf("file%i", i));
}

PortProxy &
ArmSemihosting::physProxy(ThreadContext *tc)
{
    if (ArmISA::inSecureState(tc)) {
        if (!physProxyS) {
            System *sys = tc->getSystemPtr();
            physProxyS.reset(new SecurePortProxy(
                                 sys->getSystemPort(),
                                 sys->cacheLineSize()));
        }
        return *physProxyS;
    } else {
        return tc->getPhysProxy();
    }
}


std::string
ArmSemihosting::readString(ThreadContext *tc, Addr ptr, size_t len)
{
    std::vector<char> buf(len + 1);

    buf[len] = '\0';
    physProxy(tc).readBlob(ptr, (uint8_t *)buf.data(), len);

    return std::string(buf.data());
}

ArmSemihosting::RetErrno
ArmSemihosting::callOpen(ThreadContext *tc, bool aarch64,
                         std::vector<uint64_t> &argv)
{
    const Addr name_base = argv[1];
    const char *mode = argv[2] < fmodes.size() ? fmodes[argv[2]] : nullptr;
    const Addr name_size = argv[3];

    DPRINTF(Semihosting, "Semihosting SYS_OPEN(0x%x, %i[%s], %i)\n",
            name_base, argv[2], mode ? mode : "-", name_size);
    if (!mode || !name_base)
        return retError(EINVAL);

    std::string fname = readString(tc, name_base, name_size);

    std::unique_ptr<ArmSemihosting::FileBase> file =
        FileBase::create(*this, fname, mode);
    int64_t ret = file->open();
    DPRINTF(Semihosting, "Semihosting SYS_OPEN(\"%s\", %i[%s]): %i\n",
            fname, argv[2], mode, ret);
    if (ret < 0) {
        return retError(-ret);
    } else {
        files.push_back(std::move(file));
        return retOK(files.size() - 1);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callClose(ThreadContext *tc, bool aarch64,
                          std::vector<uint64_t> &argv)
{
    if (argv[1] > files.size()) {
        DPRINTF(Semihosting, "Semihosting SYS_CLOSE(%i): Illegal file\n");
        return retError(EBADF);
    }

    std::unique_ptr<FileBase> &file = files[argv[1]];
    int64_t error = file->close();
    DPRINTF(Semihosting, "Semihosting SYS_CLOSE(%i[%s]): %i\n",
            argv[1], file->fileName(), error);
    if (error < 0) {
        return retError(-error);
    } else {
        // Zap the pointer and free the entry in the file table as
        // well.
        files[argv[1]].reset();
        return retOK(0);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callWriteC(ThreadContext *tc, bool aarch64,
                           std::vector<uint64_t> &argv)
{
    const char c = physProxy(tc).read<char>(argv[0]);

    DPRINTF(Semihosting, "Semihosting SYS_WRITEC('%c')\n", c);
    std::cout.put(c);

    return retOK(0);
}

ArmSemihosting::RetErrno
ArmSemihosting::callWrite0(ThreadContext *tc, bool aarch64,
                           std::vector<uint64_t> &argv)
{
    DPRINTF(Semihosting, "Semihosting SYS_WRITE0(...)\n");
    PortProxy &proxy = physProxy(tc);
    for (Addr addr = (Addr)argv[0]; ; ++addr) {
        char data = proxy.read<char>(addr);
        if (data == 0)
            break;

        std::cout.put(data);
    }

    return retOK(0);
}

ArmSemihosting::RetErrno
ArmSemihosting::callWrite(ThreadContext *tc, bool aarch64,
                          std::vector<uint64_t> &argv)
{
    if (argv[1] > files.size() || !files[argv[1]])
        return RetErrno(argv[3], EBADF);

    std::vector<uint8_t> buffer(argv[3]);
    physProxy(tc).readBlob(argv[2], buffer.data(), buffer.size());

    int64_t ret = files[argv[1]]->write(buffer.data(), buffer.size());
    if (ret < 0) {
        // No bytes written (we're returning the number of bytes not
        // written)
        return RetErrno(argv[3], -ret);
    } else {
        // Return the number of bytes not written
        return RetErrno(argv[3] - ret, 0);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callRead(ThreadContext *tc, bool aarch64,
                         std::vector<uint64_t> &argv)
{
    if (argv[1] > files.size() || !files[argv[1]])
        return RetErrno(argv[3], EBADF);

    std::vector<uint8_t> buffer(argv[3]);
    int64_t ret = files[argv[1]]->read(buffer.data(), buffer.size());
    if (ret < 0) {
        return RetErrno(argv[3], -ret);
    } else {
        panic_if(ret > buffer.size(), "Read longer than buffer size.");

        physProxy(tc).writeBlob(argv[2], buffer.data(), ret);

        // Return the number of bytes not written
        return retOK(argv[3] - ret);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callReadC(ThreadContext *tc, bool aarch64,
                           std::vector<uint64_t> &argv)
{
    return retOK((char)std::cin.get());
}

ArmSemihosting::RetErrno
ArmSemihosting::callIsError(ThreadContext *tc, bool aarch64,
                            std::vector<uint64_t> &argv)
{
    // Sign extend from a 32 bit integer in aarch32 since the argument
    // reader zero extends to a uint64_t.
    const int64_t status = (int64_t)(aarch64 ? argv[1] :sext<32>(argv[1]));
    // Assume there was an error if the status value is negative.
    return retOK(status < 0 ? 1 : 0);
}

ArmSemihosting::RetErrno
ArmSemihosting::callIsTTY(ThreadContext *tc, bool aarch64,
                          std::vector<uint64_t> &argv)
{
    if (argv[1] > files.size() || !files[argv[1]])
        return retError(EBADF);

    int64_t ret = files[argv[1]]->isTTY();
    if (ret < 0) {
        return retError(-ret);
    } else {
        return retOK(ret ? 1 : 0);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callSeek(ThreadContext *tc, bool aarch64,
                          std::vector<uint64_t> &argv)
{
    if (argv[1] > files.size() || !files[argv[1]])
        return retError(EBADF);

    int64_t ret = files[argv[1]]->seek(argv[2]);
    if (ret < 0) {
        return retError(-ret);
    } else {
        return retOK(0);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callFLen(ThreadContext *tc, bool aarch64,
                          std::vector<uint64_t> &argv)
{
    if (argv[1] > files.size() || !files[argv[1]])
        return retError(EBADF);

    int64_t ret = files[argv[1]]->isTTY();
    if (ret < 0) {
        return retError(-ret);
    } else {
        return retOK(0);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callTmpNam(ThreadContext *tc, bool aarch64,
                           std::vector<uint64_t> &argv)
{
    const Addr guest_buf = argv[1];
    //const uint64_t id = argv[2];
    const uint64_t max_len = argv[3];

    std::vector<char> buf(L_tmpnam);
    char *path = tmpnam(buf.data());
    if (!path)
        return retError(EINVAL);

    const size_t path_len = strlen(path);
    if (path_len >= max_len)
        return retError(ENOSPC);

    physProxy(tc).writeBlob(
        guest_buf, (const uint8_t *)path, path_len + 1);
    return retOK(0);
}

ArmSemihosting::RetErrno
ArmSemihosting::callRemove(ThreadContext *tc, bool aarch64,
                           std::vector<uint64_t> &argv)
{
    std::string fname = readString(tc, argv[1], argv[2]);

    if (remove(fname.c_str()) != 0) {
        return retError(errno);
    } else {
        return retOK(0);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callRename(ThreadContext *tc, bool aarch64,
                           std::vector<uint64_t> &argv)
{
    std::string from = readString(tc, argv[1], argv[2]);
    std::string to = readString(tc, argv[3], argv[4]);

    if (rename(from.c_str(), to.c_str()) != 0) {
        return retError(errno);
    } else {
        return retOK(0);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callClock(ThreadContext *tc, bool aarch64,
                          std::vector<uint64_t> &argv)
{
    return retOK(curTick() / (SimClock::Int::s / 100));
}

ArmSemihosting::RetErrno
ArmSemihosting::callTime(ThreadContext *tc, bool aarch64,
                         std::vector<uint64_t> &argv)
{
    return retOK(timeBase + round(curTick() / SimClock::Float::s));
}

ArmSemihosting::RetErrno
ArmSemihosting::callSystem(ThreadContext *tc, bool aarch64,
                         std::vector<uint64_t> &argv)
{
    const std::string cmd = readString(tc, argv[1], argv[2]);
    warn("Semihosting: SYS_SYSTEM not implemented. Guest tried to run: %s\n",
         cmd);
    return retError(EINVAL);

}

ArmSemihosting::RetErrno
ArmSemihosting::callErrno(ThreadContext *tc, bool aarch64,
                          std::vector<uint64_t> &argv)
{
    // Preserve errno by returning it in errno as well.
    return RetErrno(semiErrno, semiErrno);
}

ArmSemihosting::RetErrno
ArmSemihosting::callGetCmdLine(ThreadContext *tc, bool aarch64,
                               std::vector<uint64_t> &argv)
{
    if (cmdLine.size() + 1 < argv[2]) {
        PortProxy &proxy = physProxy(tc);
        ByteOrder endian = ArmISA::byteOrder(tc);
        proxy.writeBlob(
            (Addr)argv[1],
            (const uint8_t *)cmdLine.c_str(), cmdLine.size() + 1);

        if (aarch64)
            proxy.writeHtoG<uint64_t>(argv[0] + 1 * 8, cmdLine.size(), endian);
        else
            proxy.writeHtoG<uint32_t>(argv[0] + 1 * 4, cmdLine.size(), endian);
        return retOK(0);
    } else {
        return retError(0);
    }
}

ArmSemihosting::RetErrno
ArmSemihosting::callHeapInfo(ThreadContext *tc, bool aarch64,
                             std::vector<uint64_t> &argv)
{
    const PhysicalMemory &phys = tc->getSystemPtr()->getPhysMem();
    const AddrRangeList memories = phys.getConfAddrRanges();
    fatal_if(memories.size() < 1, "No memories reported from System");
    warn_if(memories.size() > 1, "Multiple physical memory ranges available. "
            "Using first range heap/stack.");
    const AddrRange memory = *memories.begin();
    const Addr mem_start = memory.start() + memReserve;
    Addr mem_end = memory.end();

    // Make sure that 32-bit guests can access their memory.
    if (!aarch64) {
        const Addr phys_max = (1ULL << 32) - 1;
        panic_if(mem_start > phys_max,
                 "Physical memory out of range for a 32-bit guest.");
        if (mem_end > phys_max) {
            warn("Some physical memory out of range for a 32-bit guest.");
            mem_end = phys_max;
        }
    }

    fatal_if(mem_start + stackSize >= mem_end,
             "Physical memory too small to fit desired stack and a heap.");

    const Addr heap_base = mem_start;
    const Addr heap_limit = mem_end - stackSize + 1;
    const Addr stack_base = (mem_end + 1) & ~0x7ULL; // 8 byte stack alignment
    const Addr stack_limit = heap_limit;


    inform("Reporting heap/stack info to guest:\n"
           "\tHeap base: 0x%x\n"
           "\tHeap limit: 0x%x\n"
           "\tStack base: 0x%x\n"
           "\tStack limit: 0x%x\n",
           heap_base, heap_limit, stack_base, stack_limit);

    Addr base = argv[1];
    PortProxy &proxy = physProxy(tc);
    ByteOrder endian = ArmISA::byteOrder(tc);
    if (aarch64) {
        proxy.writeHtoG<uint64_t>(base + 0 * 8, heap_base, endian);
        proxy.writeHtoG<uint64_t>(base + 1 * 8, heap_limit, endian);
        proxy.writeHtoG<uint64_t>(base + 2 * 8, stack_base, endian);
        proxy.writeHtoG<uint64_t>(base + 3 * 8, stack_limit, endian);
    } else {
        proxy.writeHtoG<uint32_t>(base + 0 * 4, heap_base, endian);
        proxy.writeHtoG<uint32_t>(base + 1 * 4, heap_limit, endian);
        proxy.writeHtoG<uint32_t>(base + 2 * 4, stack_base, endian);
        proxy.writeHtoG<uint32_t>(base + 3 * 4, stack_limit, endian);
    }

    return retOK(0);
}

ArmSemihosting::RetErrno
ArmSemihosting::callExit(ThreadContext *tc, bool aarch64,
                         std::vector<uint64_t> &argv)
{
    if (aarch64) {
        semiExit(argv[1], argv[2]);
    } else {
        semiExit(argv[0], 0);
    }

    return retOK(0);
}

ArmSemihosting::RetErrno
ArmSemihosting::callExitExtended(ThreadContext *tc, bool aarch64,
                                 std::vector<uint64_t> &argv)
{
    semiExit(argv[1], argv[2]);

    return retOK(0);
}

void
ArmSemihosting::semiExit(uint64_t code, uint64_t subcode)
{
    auto it = exitCodes.find(code);
    if (it != exitCodes.end()) {
        exitSimLoop(it->second, subcode);
    } else {
        exitSimLoop(csprintf("semi:0x%x", code), subcode);
    }
}


ArmSemihosting::RetErrno
ArmSemihosting::callElapsed(ThreadContext *tc, bool aarch64,
                            std::vector<uint64_t> &argv)
{
    PortProxy &proxy = physProxy(tc);
    ByteOrder endian = ArmISA::byteOrder(tc);
    const uint64_t tick = semiTick(curTick());

    if (aarch64) {
        proxy.writeHtoG<uint64_t>(argv[0], tick, endian);
    } else {
        proxy.writeHtoG<uint32_t>(argv[0] + 0 * 4, tick, endian);
        proxy.writeHtoG<uint32_t>(argv[0] + 1 * 4, tick >> 32, endian);
    }

    return retOK(0);
}


ArmSemihosting::RetErrno
ArmSemihosting::callTickFreq(ThreadContext *tc, bool aarch64,
                             std::vector<uint64_t> &argv)
{
    return retOK(semiTick(SimClock::Frequency));
}

const ArmSemihosting::SemiCall *
ArmSemihosting::getCall(uint32_t op, bool aarch64)
{
    auto it = calls.find(op);
    if (it == calls.end())
        return nullptr;
    else {
        return &it->second;
    }
}

FILE *
ArmSemihosting::getSTDIO(const char *stream_name,
                         const std::string &name, const char *mode)
{
    auto it = stdioMap.find(name);
    if (it == stdioMap.end()) {
        FILE *f = fopen(name.c_str(), mode);
        if (!f) {
            fatal("Failed to open %s (%s): %s\n",
                  stream_name, name, strerror(errno));
        }
        return f;
    } else {
        return it->second;
    }
}

std::unique_ptr<ArmSemihosting::FileBase>
ArmSemihosting::FileBase::create(
    ArmSemihosting &parent, const std::string &fname, const char *mode)
{
    std::unique_ptr<FileBase> file;
    if (fname == ":semihosting-features") {
        file.reset(new FileFeatures(parent, fname.c_str(), mode));
    } else {
        file.reset(new File(parent, fname.c_str(), mode));
    }

    return file;
}

std::unique_ptr<ArmSemihosting::FileBase>
ArmSemihosting::FileBase::create(ArmSemihosting &parent,
                                 CheckpointIn &cp, const std::string &sec)
{
    std::unique_ptr<FileBase> file;
    ScopedCheckpointSection _sec(cp, sec);

    // Was the file open when the checkpoint was created?
    if (!cp.sectionExists(Serializable::currentSection()))
        return file;

    std::string fname, mode;
    paramIn(cp, "name", fname);
    paramIn(cp, "mode", mode);
    file = create(parent, fname, mode.c_str());
    assert(file);
    file->unserialize(cp);

    return file;
}

void
ArmSemihosting::FileBase::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "name", _name);
    SERIALIZE_SCALAR(mode);
}

void
ArmSemihosting::FileBase::unserialize(CheckpointIn &cp)
{
    /* Unserialization of name and mode happens in
     * ArmSemihosting::FileBase::create() */
}

int64_t
ArmSemihosting::FileBase::read(uint8_t *buffer, uint64_t size)
{
    return -EINVAL;
}

int64_t
ArmSemihosting::FileBase::write(const uint8_t *buffer, uint64_t size)
{
    return -EINVAL;
}

int64_t
ArmSemihosting::FileBase::seek(uint64_t pos)
{
    return -EINVAL;
}

int64_t
ArmSemihosting::FileBase::flen()
{
    return -EINVAL;
}


ArmSemihosting::FileFeatures::FileFeatures(
    ArmSemihosting &_parent, const char *_name, const char *_mode)
    : FileBase(_parent, _name, _mode)
{
}

int64_t
ArmSemihosting::FileFeatures::read(uint8_t *buffer, uint64_t size)
{
    int64_t len = 0;

    for (; pos < size && pos < ArmSemihosting::features.size(); pos++)
        buffer[len++] = ArmSemihosting::features[pos];

    return len;
}

int64_t
ArmSemihosting::FileFeatures::seek(uint64_t _pos)
{
    if (_pos < ArmSemihosting::features.size()) {
        pos = _pos;
        return 0;
    } else {
        return -ENXIO;
    }
}

void
ArmSemihosting::FileFeatures::serialize(CheckpointOut &cp) const
{
    FileBase::serialize(cp);
    SERIALIZE_SCALAR(pos);
}

void
ArmSemihosting::FileFeatures::unserialize(CheckpointIn &cp)
{
    FileBase::unserialize(cp);
    UNSERIALIZE_SCALAR(pos);
}



ArmSemihosting::File::File(ArmSemihosting &_parent,
                           const char *_name, const char *_perms)
    : FileBase(_parent, _name, _perms),
      file(nullptr)
{
}

ArmSemihosting::File::~File()
{
    if (file)
        close();
}

int64_t
ArmSemihosting::File::openImpl(bool in_cpt)
{
    panic_if(file, "Trying to open an already open file.\n");

    if (_name == ":tt") {
        if (mode[0] == 'r') {
            file = parent.stdin;
        } else if (mode[0] == 'w') {
            file = parent.stdout;
        } else if (mode[0] == 'a') {
            file = parent.stderr;
        } else {
            warn("Unknown file mode for the ':tt' special file");
            return -EINVAL;
        }
    } else {
        std::string real_mode(this->mode);
        // Avoid truncating the file if we are restoring from a
        // checkpoint.
        if (in_cpt && real_mode[0] == 'w')
            real_mode[0] = 'a';

        file = fopen(_name.c_str(), real_mode.c_str());
    }

    return file ? 0 : -errno;
}

int64_t
ArmSemihosting::File::close()
{
    panic_if(!file, "Trying to close an already closed file.\n");

    if (needClose()) {
        fclose(file);
    }
    file = nullptr;

    return 0;
}

bool
ArmSemihosting::File::isTTY() const
{
    return file == parent.stdout ||
        file == parent.stderr ||
        file == parent.stdin;
}

int64_t
ArmSemihosting::File::read(uint8_t *buffer, uint64_t size)
{
    panic_if(!file, "Trying to read from a closed file");

    size_t ret = fread(buffer, 1, size, file);
    if (ret == 0) {
        // Error or EOF. Assume errors are due to invalid file
        // operations (e.g., reading a write-only stream).
        return ferror(file) ? -EINVAL : 0;
    } else {
        return ret;
    }
}

int64_t
ArmSemihosting::File::write(const uint8_t *buffer, uint64_t size)
{
    panic_if(!file, "Trying to write to a closed file");


    size_t ret = fwrite(buffer, 1, size, file);
    if (ret == 0) {
        // Assume errors are due to invalid file operations (e.g.,
        // writing a read-only stream).
        return -EINVAL;
    } else {
        return ret;
    }
}

int64_t
ArmSemihosting::File::seek(uint64_t _pos)
{
    panic_if(!file, "Trying to seek in a closed file");

    errno = 0;
    if (fseek(file, _pos, SEEK_SET) == 0)
        return 0;
    else
        return -errno;
}

int64_t
ArmSemihosting::File::flen()
{
    errno = 0;
    long pos = ftell(file);
    if (pos < 0)
        return -errno;

    if (fseek(file, 0, SEEK_END) != 0)
        return -errno;

    long len = ftell(file);
    if (len < 0)
        return -errno;

    if (fseek(file, pos, SEEK_SET) != 0)
        return -errno;

    return len;
}


void
ArmSemihosting::File::serialize(CheckpointOut &cp) const
{
    FileBase::serialize(cp);

    if (!isTTY()) {
        long pos = file ? ftell(file) : 0;
        panic_if(pos < 0, "Failed to get file position.");
        SERIALIZE_SCALAR(pos);
    }
}

void
ArmSemihosting::File::unserialize(CheckpointIn &cp)
{
    FileBase::unserialize(cp);

    if (openImpl(true) < 0) {
        fatal("Failed to open file: %s", _name);
    }

    if (!isTTY()) {
        long pos = 0;
        UNSERIALIZE_SCALAR(pos);
        if (fseek(file, pos, SEEK_SET) != 0) {
            fatal("Failed seek to current position (%i) in '%s'", pos, _name);
        }
    }
}


ArmSemihosting *
ArmSemihostingParams::create()
{
    return new ArmSemihosting(this);
}

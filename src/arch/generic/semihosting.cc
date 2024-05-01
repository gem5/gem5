/*
 * Copyright (c) 2018, 2019 ARM Limited
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

#include "arch/generic/semihosting.hh"

#include <unistd.h>

#include <cerrno>
#include <cstdio>

#include "base/logging.hh"
#include "base/output.hh"
#include "base/time.hh"
#include "debug/Semihosting.hh"
#include "dev/serial/serial.hh"
#include "mem/physical.hh"
#include "params/BaseSemihosting.hh"
#include "sim/byteswap.hh"
#include "sim/pseudo_inst.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

const std::vector<const char *> BaseSemihosting::fmodes{
        "r",
        "rb",
        "r+",
        "r+b",
        "w",
        "wb",
        "w+",
        "w+b",
        "a",
        "ab",
        "a+",
        "a+b",
};

const std::map<uint64_t, const char *> BaseSemihosting::exitCodes{
        {0x20000, "semi:ADP_Stopped_BranchThroughZero"},
        {0x20001, "semi:ADP_Stopped_UndefinedInstr"},
        {0x20002, "semi:ADP_Stopped_SoftwareInterrupt"},
        {0x20003, "semi:ADP_Stopped_PrefetchAbort"},
        {0x20004, "semi:ADP_Stopped_DataAbort"},
        {0x20005, "semi:ADP_Stopped_AddressException"},
        {0x20006, "semi:ADP_Stopped_IRQ"},
        {0x20007, "semi:ADP_Stopped_FIQ"},

        {0x20020, "semi:ADP_Stopped_BreakPoint"},
        {0x20021, "semi:ADP_Stopped_WatchPoint"},
        {0x20022, "semi:ADP_Stopped_StepComplete"},
        {0x20023, "semi:ADP_Stopped_RunTimeErrorUnknown"},
        {0x20024, "semi:ADP_Stopped_InternalError"},
        {0x20025, "semi:ADP_Stopped_UserInterruption"},
        {0x20026, "semi:ADP_Stopped_ApplicationExit"},
        {0x20027, "semi:ADP_Stopped_StackOverflow"},
        {0x20028, "semi:ADP_Stopped_DivisionByZero"},
        {0x20029, "semi:ADP_Stopped_DivisionByZero"},
};

const std::array<uint8_t, 5> BaseSemihosting::features{
        0x53, 0x48, 0x46, 0x42, // Magic
        0x3, // EXT_EXIT_EXTENDED, EXT_STDOUT_STDERR
};

const std::map<const std::string, FILE *> BaseSemihosting::stdioMap{
        {"cin", ::stdin},
        {"stdin", ::stdin},
        {"cout", ::stdout},
        {"stdout", ::stdout},
        {"cerr", ::stderr},
        {"stderr", ::stderr},
};

BaseSemihosting::BaseSemihosting(const BaseSemihostingParams &p)
    : SimObject(p), cmdLine(p.cmd_line), memReserve(p.mem_reserve),
    stackSize(p.stack_size), timeBase([p] {
        struct tm t = p.time;
        return mkutctime(&t);
    }()),
    tickShift(calcTickShift()), semiErrno(0),
    filesRootDir(!p.files_root_dir.empty() && p.files_root_dir.back() != '/' ?
                         p.files_root_dir + '/' :
                         p.files_root_dir),
    stdin(getSTDIO("stdin", p.stdin, "r")),
    stdout(getSTDIO("stdout", p.stdout, "w")),
    stderr(p.stderr == p.stdout ? stdout : getSTDIO("stderr", p.stderr, "w"))
{
    // Create an empty place-holder file for position 0 as semi-hosting
    // calls typically expect non-zero file handles.
    files.push_back(nullptr);

    if (tickShift > 0)
        inform("Semihosting: Shifting elapsed ticks by %i bits.", tickShift);
}

void
BaseSemihosting::serialize(CheckpointOut &cp) const
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
BaseSemihosting::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(semiErrno);

    size_t num_files;
    paramIn(cp, "num_files", num_files);
    files.resize(num_files);
    for (int i = 0; i < num_files; i++)
        files[i] = FileBase::create(*this, cp, csprintf("file%i", i));
}

std::string
BaseSemihosting::readString(ThreadContext *tc, Addr ptr, size_t len)
{
    std::vector<char> buf(len + 1);

    buf[len] = '\0';
    portProxy(tc).readBlob(ptr, buf.data(), len);

    return std::string(buf.data());
}

BaseSemihosting::RetErrno
BaseSemihosting::callOpen(
        ThreadContext *tc, const Addr name_base, int fmode, size_t name_size)
{
    const char *mode = fmode < fmodes.size() ? fmodes[fmode] : nullptr;

    DPRINTF(Semihosting, "Semihosting SYS_OPEN(0x%x, %i[%s], %i)\n", name_base,
            fmode, mode ? mode : "-", name_size);
    if (!mode || !name_base)
        return retError(EINVAL);

    std::string fname = readString(tc, name_base, name_size);
    if (!fname.empty() && fname.front() != '/')
        fname = filesRootDir + fname;

    std::unique_ptr<BaseSemihosting::FileBase> file =
            FileBase::create(*this, fname, mode);
    int64_t ret = file->open();
    DPRINTF(Semihosting, "Semihosting SYS_OPEN(\"%s\", %i[%s]): %i\n", fname,
            fmode, mode, ret);
    if (ret < 0) {
        return retError(-ret);
    } else {
        files.push_back(std::move(file));
        return retOK(files.size() - 1);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callClose(ThreadContext *tc, Handle handle)
{
    if (handle > files.size()) {
        DPRINTF(Semihosting, "Semihosting SYS_CLOSE(%i): Illegal file\n");
        return retError(EBADF);
    }

    std::unique_ptr<FileBase> &file = files[handle];
    int64_t error = file->close();
    DPRINTF(Semihosting, "Semihosting SYS_CLOSE(%i[%s]): %i\n", handle,
            file->fileName(), error);
    if (error < 0) {
        return retError(-error);
    } else {
        // Zap the pointer and free the entry in the file table as
        // well.
        files[handle].reset();
        return retOK(0);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callWriteC(ThreadContext *tc, InPlaceArg arg)
{
    const char c = portProxy(tc).read<char>(arg.addr);

    DPRINTF(Semihosting, "Semihosting SYS_WRITEC('%c')\n", c);
    std::cout.put(c);

    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callWrite0(ThreadContext *tc, InPlaceArg arg)
{
    DPRINTF(Semihosting, "Semihosting SYS_WRITE0(...)\n");
    PortProxy &proxy = portProxy(tc);
    std::string str;
    proxy.readString(str, arg.addr);
    std::cout.write(str.c_str(), str.size());

    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callWrite(
        ThreadContext *tc, Handle handle, Addr addr, size_t size)
{
    if (handle > files.size() || !files[handle])
        return RetErrno(size, EBADF);

    std::vector<uint8_t> buffer(size);
    portProxy(tc).readBlob(addr, buffer.data(), buffer.size());

    int64_t ret = files[handle]->write(buffer.data(), buffer.size());
    if (ret < 0) {
        // No bytes written (we're returning the number of bytes not
        // written)
        return RetErrno(size, -ret);
    } else {
        // Return the number of bytes not written
        return RetErrno(size - ret, 0);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callRead(
        ThreadContext *tc, Handle handle, Addr addr, size_t size)
{
    if (handle > files.size() || !files[handle])
        return RetErrno(size, EBADF);

    std::vector<uint8_t> buffer(size);
    int64_t ret = files[handle]->read(buffer.data(), buffer.size());
    if (ret < 0) {
        return RetErrno(size, -ret);
    } else {
        panic_if(ret > buffer.size(), "Read longer than buffer size.");

        portProxy(tc).writeBlob(addr, buffer.data(), ret);

        // Return the number of bytes not written
        return retOK(size - ret);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callReadC(ThreadContext *tc)
{
    return retOK((char)std::cin.get());
}

BaseSemihosting::RetErrno
BaseSemihosting::callIsError(ThreadContext *tc, int64_t status)
{
    return retOK(status < 0 ? 1 : 0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callIsTTY(ThreadContext *tc, Handle handle)
{
    if (handle > files.size() || !files[handle])
        return retError(EBADF);

    int64_t ret = files[handle]->isTTY();
    if (ret < 0) {
        return retError(-ret);
    } else {
        return retOK(ret ? 1 : 0);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callSeek(ThreadContext *tc, Handle handle, uint64_t pos)
{
    if (handle > files.size() || !files[handle])
        return retError(EBADF);

    int64_t ret = files[handle]->seek(pos);
    if (ret < 0) {
        return retError(-ret);
    } else {
        return retOK(0);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callFLen(ThreadContext *tc, Handle handle)
{
    if (handle > files.size() || !files[handle])
        return retError(EBADF);

    int64_t ret = files[handle]->flen();
    if (ret < 0) {
        return retError(-ret);
    } else {
        return retOK(ret);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callTmpNam(
        ThreadContext *tc, Addr addr, uint64_t id, size_t size)
{
    std::string path = "";
    int64_t unlink_call_ret = 0;

    do {
        path = simout.resolve(csprintf("%s.tmp%05i", name(), tmpNameIndex++));
        // remove the (potentially existing) file of the given path
        unlink_call_ret = unlink(path.c_str());
        // if the file is busy, find another name
    } while ((unlink_call_ret < 0) && (errno == EBUSY));

    const size_t path_len = path.length();
    if (path_len >= size)
        return retError(ENOSPC);

    portProxy(tc).writeBlob(addr, path.c_str(), path_len + 1);
    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callRemove(
        ThreadContext *tc, Addr name_base, size_t name_size)
{
    std::string fname = readString(tc, name_base, name_size);

    if (remove(fname.c_str()) != 0) {
        return retError(errno);
    } else {
        return retOK(0);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callRename(ThreadContext *tc, Addr from_addr,
        size_t from_size, Addr to_addr, size_t to_size)
{
    std::string from = readString(tc, from_addr, from_size);
    std::string to = readString(tc, to_addr, to_size);

    if (rename(from.c_str(), to.c_str()) != 0) {
        return retError(errno);
    } else {
        return retOK(0);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callClock(ThreadContext *tc)
{
    return retOK(curTick() / (sim_clock::as_int::s / 100));
}

BaseSemihosting::RetErrno
BaseSemihosting::callTime(ThreadContext *tc)
{
    return retOK(timeBase + round(curTick() / sim_clock::as_float::s));
}

BaseSemihosting::RetErrno
BaseSemihosting::callSystem(ThreadContext *tc, Addr cmd_addr, size_t cmd_size)
{
    const std::string cmd = readString(tc, cmd_addr, cmd_size);
    warn("Semihosting: SYS_SYSTEM not implemented. Guest tried to run: %s\n",
            cmd);
    return retError(EINVAL);
}

BaseSemihosting::RetErrno
BaseSemihosting::callErrno(ThreadContext *tc)
{
    // Preserve errno by returning it in errno as well.
    return RetErrno(semiErrno, semiErrno);
}

BaseSemihosting::RetErrno
BaseSemihosting::callGetCmdLine(
        ThreadContext *tc, Addr addr, InPlaceArg size_arg)
{
    PortProxy &proxy = portProxy(tc);
    ByteOrder endian = byteOrder(tc);
    size_t size = size_arg.read(tc, proxy, endian);

    if (cmdLine.size() + 1 < size) {
        proxy.writeBlob(addr, cmdLine.c_str(), cmdLine.size() + 1);
        size_arg.write(tc, proxy, cmdLine.size(), endian);
        return retOK(0);
    } else {
        return retError(0);
    }
}

void
BaseSemihosting::gatherHeapInfo(ThreadContext *tc, bool aarch64,
        Addr &heap_base, Addr &heap_limit, Addr &stack_base, Addr &stack_limit)
{
    const memory::PhysicalMemory &phys = tc->getSystemPtr()->getPhysMem();
    const AddrRangeList memories = phys.getConfAddrRanges();
    fatal_if(memories.size() < 1, "No memories reported from System");
    warn_if(memories.size() > 1,
            "Multiple physical memory ranges available. "
            "Using first range heap/stack.");
    const AddrRange mem = *memories.begin();
    const Addr mem_start = mem.start() + memReserve;
    Addr mem_end = mem.end();

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

    heap_base = mem_start;
    heap_limit = mem_end - stackSize + 1;
    stack_base = (mem_end + 1) & ~0x7ULL; // 8 byte stack alignment
    stack_limit = heap_limit;

    inform("Reporting heap/stack info to guest:\n"
           "\tHeap base: 0x%x\n"
           "\tHeap limit: 0x%x\n"
           "\tStack base: 0x%x\n"
           "\tStack limit: 0x%x\n",
            heap_base, heap_limit, stack_base, stack_limit);
}

BaseSemihosting::RetErrno
BaseSemihosting::callHeapInfo32(ThreadContext *tc, Addr block_addr)
{
    uint64_t heap_base, heap_limit, stack_base, stack_limit;
    gatherHeapInfo(tc, false, heap_base, heap_limit, stack_base, stack_limit);

    std::array<uint32_t, 4> block = {
            {(uint32_t)heap_base, (uint32_t)heap_limit, (uint32_t)stack_base,
                    (uint32_t)stack_limit}};
    portProxy(tc).write(block_addr, block, byteOrder(tc));

    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callHeapInfo64(ThreadContext *tc, Addr block_addr)
{
    uint64_t heap_base, heap_limit, stack_base, stack_limit;
    gatherHeapInfo(tc, true, heap_base, heap_limit, stack_base, stack_limit);

    std::array<uint64_t, 4> block = {
            {heap_base, heap_limit, stack_base, stack_limit}};
    portProxy(tc).write(block_addr, block, byteOrder(tc));

    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callExit32(ThreadContext *tc, InPlaceArg code)
{
    semiExit(code.addr, 0);
    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callExit64(ThreadContext *tc, uint64_t code, uint64_t subcode)
{
    semiExit(code, subcode);
    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callExitExtended(
        ThreadContext *tc, uint64_t code, uint64_t subcode)
{
    semiExit(code, subcode);
    return retOK(0);
}

void
BaseSemihosting::semiExit(uint64_t code, uint64_t subcode)
{
    auto it = exitCodes.find(code);
    if (it != exitCodes.end()) {
        exitSimLoop(it->second, subcode);
    } else {
        exitSimLoop(csprintf("semi:0x%x", code), subcode);
    }
}

BaseSemihosting::RetErrno
BaseSemihosting::callElapsed32(
        ThreadContext *tc, InPlaceArg low, InPlaceArg high)
{
    PortProxy &proxy = portProxy(tc);
    ByteOrder endian = byteOrder(tc);
    uint64_t tick = semiTick(curTick());

    low.write(tc, proxy, tick, endian);
    high.write(tc, proxy, tick >> 32, endian);

    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callElapsed64(ThreadContext *tc, InPlaceArg ticks)
{
    ticks.write(tc, portProxy(tc), semiTick(curTick()), byteOrder(tc));
    return retOK(0);
}

BaseSemihosting::RetErrno
BaseSemihosting::callTickFreq(ThreadContext *tc)
{
    return retOK(semiTick(sim_clock::Frequency));
}

FILE *
BaseSemihosting::getSTDIO(
        const char *stream_name, const std::string &name, const char *mode)
{
    auto it = stdioMap.find(name);
    if (it == stdioMap.end()) {
        FILE *f = fopen(name.c_str(), mode);
        if (!f) {
            fatal("Failed to open %s (%s): %s\n", stream_name, name,
                    strerror(errno));
        }
        return f;
    } else {
        return it->second;
    }
}

std::unique_ptr<BaseSemihosting::FileBase>
BaseSemihosting::FileBase::create(
        BaseSemihosting &parent, const std::string &fname, const char *mode)
{
    std::unique_ptr<FileBase> file;
    if (fname == ":semihosting-features") {
        file.reset(new FileFeatures(parent, fname.c_str(), mode));
    } else {
        file.reset(new File(parent, fname.c_str(), mode));
    }

    return file;
}

std::unique_ptr<BaseSemihosting::FileBase>
BaseSemihosting::FileBase::create(
        BaseSemihosting &parent, CheckpointIn &cp, const std::string &sec)
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
BaseSemihosting::FileBase::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "name", _name);
    SERIALIZE_SCALAR(mode);
}

void
BaseSemihosting::FileBase::unserialize(CheckpointIn &cp)
{
    /* Unserialization of name and mode happens in
     * BaseSemihosting::FileBase::create() */
}

int64_t
BaseSemihosting::FileBase::read(uint8_t *buffer, uint64_t size)
{
    return -EINVAL;
}

int64_t
BaseSemihosting::FileBase::write(const uint8_t *buffer, uint64_t size)
{
    return -EINVAL;
}

int64_t
BaseSemihosting::FileBase::seek(uint64_t pos)
{
    return -EINVAL;
}

int64_t
BaseSemihosting::FileBase::flen()
{
    return -EINVAL;
}

BaseSemihosting::FileFeatures::
FileFeatures(BaseSemihosting &_parent, const char *_name, const char *_mode) :
    FileBase(_parent, _name, _mode)
{}

int64_t
BaseSemihosting::FileFeatures::flen()
{
    return features.size();
}

int64_t
BaseSemihosting::FileFeatures::read(uint8_t *buffer, uint64_t size)
{
    int64_t len = 0;

    for (; len < size && pos < features.size(); pos++)
        buffer[len++] = features[pos];

    return len;
}

int64_t
BaseSemihosting::FileFeatures::seek(uint64_t _pos)
{
    if (_pos < BaseSemihosting::features.size()) {
        pos = _pos;
        return 0;
    } else {
        return -ENXIO;
    }
}

void
BaseSemihosting::FileFeatures::serialize(CheckpointOut &cp) const
{
    FileBase::serialize(cp);
    SERIALIZE_SCALAR(pos);
}

void
BaseSemihosting::FileFeatures::unserialize(CheckpointIn &cp)
{
    FileBase::unserialize(cp);
    UNSERIALIZE_SCALAR(pos);
}

BaseSemihosting::File::
File(BaseSemihosting &_parent, const char *_name, const char *_perms) :
    FileBase(_parent, _name, _perms), file(nullptr)
{}

BaseSemihosting::File::~
File()
{
    if (file)
        close();
}

int64_t
BaseSemihosting::File::openImpl(bool in_cpt)
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
BaseSemihosting::File::close()
{
    panic_if(!file, "Trying to close an already closed file.\n");

    if (needClose()) {
        fclose(file);
    }
    file = nullptr;

    return 0;
}

bool
BaseSemihosting::File::isTTY() const
{
    return file == parent.stdout || file == parent.stderr ||
           file == parent.stdin;
}

int64_t
BaseSemihosting::File::read(uint8_t *buffer, uint64_t size)
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
BaseSemihosting::File::write(const uint8_t *buffer, uint64_t size)
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
BaseSemihosting::File::seek(uint64_t _pos)
{
    panic_if(!file, "Trying to seek in a closed file");

    errno = 0;
    if (fseek(file, _pos, SEEK_SET) == 0)
        return 0;
    else
        return -errno;
}

int64_t
BaseSemihosting::File::flen()
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
BaseSemihosting::File::serialize(CheckpointOut &cp) const
{
    FileBase::serialize(cp);

    if (!isTTY()) {
        long pos = file ? ftell(file) : 0;
        panic_if(pos < 0, "Failed to get file position.");
        SERIALIZE_SCALAR(pos);
    }
}

void
BaseSemihosting::File::unserialize(CheckpointIn &cp)
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

std::ostream &
operator<<(std::ostream &os, const BaseSemihosting::InPlaceArg &ipa)
{
    ccprintf(os, "[%#x-%#x)", ipa.addr, ipa.addr + ipa.size - 1);
    return os;
}

} // namespace gem5

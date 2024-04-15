/*
 * Copyright 2020 Google Inc.
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

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <csetjmp>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <gem5/asm/generic/m5ops.h>

#include "args.hh"
#include "call_type.hh"
#include "dispatch_table.hh"
#include "m5_mmap.h"

class DefaultCallType : public CallType
{
  private:
    DispatchTable dt;

  public:
    DefaultCallType() : CallType("default") {}

    bool initCalled = false;

    void
    init() override
    {
        initCalled = true;
    }

    bool
    isDefault() const override
    {
        return true;
    }

    void
    printDesc(std::ostream &os) const override
    {}

    const DispatchTable &
    getDispatch() const override
    {
        return dt;
    }
};

DefaultCallType defaultCallType;

#if defined(M5OP_ADDR)
const bool DefaultAddrDefined = true;
constexpr uint64_t DefaultAddress = M5OP_ADDR;
#else
const bool DefaultAddrDefined = false;
constexpr uint64_t DefaultAddress = 0;
#endif

class AddrCallTypeTest : public testing::Test
{
  protected:
    CallType *ct = nullptr;

    void
    SetUp() override
    {
        m5_mmap_dev = "/dev/zero";
        m5op_addr = 2;
    }

    void
    TearDown() override
    {
        unmap_m5_mem();
    }
};

TEST_F(AddrCallTypeTest, EmptyArgs)
{
    // Addr should not be selected if there are no arguments.
    Args empty({});
    defaultCallType.initCalled = false;
    ct = CallType::detect(empty);
    EXPECT_EQ(ct, &defaultCallType);
    EXPECT_TRUE(defaultCallType.initCalled);
}

TEST_F(AddrCallTypeTest, OneArgMismatch)
{
    // Addr should not be selected if --addr isn't the first argument.
    Args one_arg({ "one" });
    defaultCallType.initCalled = false;
    ct = CallType::detect(one_arg);
    EXPECT_EQ(ct, &defaultCallType);
    EXPECT_TRUE(defaultCallType.initCalled);
    EXPECT_EQ(one_arg.size(), 1);
}

TEST_F(AddrCallTypeTest, OneArgSelected)
{
    // Addr should be selected if --addr is the first argument.
    Args selected({ "--addr=3" });
    defaultCallType.initCalled = false;
    ct = CallType::detect(selected);
    EXPECT_NE(ct, &defaultCallType);
    EXPECT_NE(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, 3);
}

TEST_F(AddrCallTypeTest, SplitSelected)
{
    Args split({ "--addr", "3" });
    defaultCallType.initCalled = false;
    ct = CallType::detect(split);
    EXPECT_NE(ct, &defaultCallType);
    EXPECT_NE(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, 3);
}

TEST_F(AddrCallTypeTest, OneArgSelectedExtra)
{
    Args selected_extra({ "--addr=3", "foo" });
    defaultCallType.initCalled = false;
    ct = CallType::detect(selected_extra);
    EXPECT_NE(ct, &defaultCallType);
    EXPECT_NE(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, 3);
}

TEST_F(AddrCallTypeTest, SplitSelectedExtra)
{
    Args split_extra({ "--addr", "3", "foo" });
    defaultCallType.initCalled = false;
    ct = CallType::detect(split_extra);
    EXPECT_NE(ct, &defaultCallType);
    EXPECT_NE(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, 3);
}

TEST_F(AddrCallTypeTest, SupersetOneArg)
{
    // Nothing should be selected if an argument starts with --addr which is
    // followed by something other than '=' and then a number.
    Args no_equal({ "--address" });
    defaultCallType.initCalled = false;
    ct = CallType::detect(no_equal);
    EXPECT_EQ(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, 2);
}

TEST_F(AddrCallTypeTest, NonNumberAddr)
{
    Args no_number({ "--addr=foo" });
    defaultCallType.initCalled = false;
    ct = CallType::detect(no_number);
    EXPECT_EQ(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, 2);
}

TEST_F(AddrCallTypeTest, DetectDefaultAddr)
{
    if (!DefaultAddrDefined)
        return;

    // Verify that the default address is set up in m5op_addr.
    Args noaddr({ "--addr" });
    defaultCallType.initCalled = false;
    m5op_addr = DefaultAddress;
    ct = CallType::detect(noaddr);
    EXPECT_NE(ct, &defaultCallType);
    EXPECT_NE(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, DefaultAddress);
}

TEST_F(AddrCallTypeTest, DetectDefaultAddrExtra)
{
    if (!DefaultAddrDefined)
        return;

    Args noaddr_foo({ "--addr", "foo" });
    defaultCallType.initCalled = false;
    m5op_addr = DefaultAddress;
    ct = CallType::detect(noaddr_foo);
    EXPECT_NE(ct, &defaultCallType);
    EXPECT_NE(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, DefaultAddress);
}

TEST_F(AddrCallTypeTest, DetectNoDefault)
{
    if (DefaultAddrDefined)
        return;

    // Verify that the address must be specified since there's no default.
    Args noaddr({ "--addr" });
    defaultCallType.initCalled = false;
    m5op_addr = DefaultAddress + 1;
    ct = CallType::detect(noaddr);
    EXPECT_EQ(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, DefaultAddress + 1);
}

TEST_F(AddrCallTypeTest, DetectNoDefaultExtra)
{
    if (DefaultAddrDefined)
        return;

    Args noaddr_foo({ "--addr", "foo" });
    defaultCallType.initCalled = false;
    m5op_addr = DefaultAddress + 1;
    ct = CallType::detect(noaddr_foo);
    EXPECT_EQ(ct, nullptr);
    EXPECT_FALSE(defaultCallType.initCalled);
    EXPECT_EQ(m5op_addr, DefaultAddress + 1);
}

TEST_F(AddrCallTypeTest, NotFirstArg)
{
    // Addr should not be selected if --addr isn't first.
    Args not_first({ "foo", "--addr" });
    defaultCallType.initCalled = false;
    ct = CallType::detect(not_first);
    EXPECT_EQ(ct, &defaultCallType);
    EXPECT_TRUE(defaultCallType.initCalled);
    EXPECT_EQ(not_first.size(), 2);
}

sigjmp_buf interceptEnv;
siginfo_t interceptSiginfo;

void
sigsegv_handler(int sig, siginfo_t *info, void *ucontext)
{
    std::memcpy(&interceptSiginfo, info, sizeof(interceptSiginfo));
    siglongjmp(interceptEnv, 1);
}

const uint64_t MmapPhysAddr = 0x1000000;

// A class to create and clean up a sparse temporary file of a given size.
class TempFile
{
  private:
    size_t _size;
    int fd;
    std::string _path;

  public:
    TempFile(size_t _size) : _size(_size)
    {
        // Generate a temporary filename.
        char *tmp_name = strdup("/tmp/addr.test.XXXXXXXX");
        fd = mkstemp(tmp_name);
        _path = tmp_name;
        free(tmp_name);

        // Make the file the appropriate length.
        assert(!ftruncate(fd, _size));
    };

    ~TempFile()
    {
        unlink(path().c_str());
        close(fd);
    }

    const std::string &
    path() const
    {
        return _path;
    }
};

// Sparse dummy mmap file if we're not in gem5.
TempFile mmapDummyFile(MmapPhysAddr * 2);

void
verify_mmap()
{
    // Look for the proc file that lists all our mmap-ed files.
    pid_t pid = getpid();

    std::ostringstream os;
    os << "/proc/" << pid << "/maps";
    auto maps_path = os.str();

    if (access(maps_path.c_str(), R_OK) == -1) {
        std::cout << "Unable to access " << maps_path << ", can't verify mmap."
                  << std::endl;
        return;
    }

    // Verify that the right area is mmap-ed.
    std::ifstream maps(maps_path);
    EXPECT_TRUE(maps);

    uint64_t start, end, offset, inode;
    std::string path, permissions, device;

    std::istringstream line_ss;
    bool found = false;
    while (!maps.eof()) {
        std::string line;
        if (getline(maps, line).fail()) {
            std::cout << "Error reading from \"" << maps_path << "\"."
                      << std::endl;
            return;
        }

        line_ss.str(line);

        line_ss >> std::hex >> start >> std::dec;

        char c;
        line_ss.get(c);
        if (c != '-') {
            std::cout << "Badly formatted maps line." << std::endl;
            continue;
        }

        // Is this the mapping we're interested in?
        if (start == (uintptr_t)m5_mem) {
            found = true;
            break;
        }
    }

    if (maps.eof() && !found) {
        std::cout << "Did not find entry for temp file \""
                  << mmapDummyFile.path() << "\" in \"" << maps_path << "\"."
                  << std::endl;
        ADD_FAILURE() << "No mapping for our mmapped file.";
        return;
    }

    // We found our mapping. Try to extract the remaining fields.
    line_ss >> std::hex >> end >> std::dec;
    line_ss >> permissions;
    line_ss >> std::hex >> offset >> std::dec;
    line_ss >> device;
    line_ss >> inode;

    // Everything left on the line goes into "path".
    getline(line_ss, path);

    // Strip off whitespace on either end of the path.
    const char *ws = " \t\n\r\f\v";
    // If nothing would be left, don't bother.
    if (path.find_first_not_of(ws) != std::string::npos) {
        path.erase(path.find_last_not_of(ws) + 1);
        path.erase(0, path.find_first_not_of(ws));
    }

    // Use stat to make sure this is the right file, in case the path
    // strings are immaterially different.
    struct stat stata, statb;
    EXPECT_EQ(stat(path.c_str(), &stata), 0);
    EXPECT_EQ(stat(mmapDummyFile.path().c_str(), &statb), 0);
    EXPECT_EQ(stata.st_dev, statb.st_dev);
    EXPECT_EQ(stata.st_ino, statb.st_ino);

    // Make sure the mapping is in the right place and the right size.
    EXPECT_EQ(end - start, 0x10000);
    EXPECT_EQ(offset, MmapPhysAddr);
}

TEST(AddrCallType, Sum)
{
    // Determine if we're running within gem5 by checking whether a flag is
    // set in the environment.
    bool in_gem5 = (std::getenv("RUNNING_IN_GEM5") != nullptr);
    if (in_gem5)
        std::cout << "In gem5, m5 ops should work." << std::endl;
    else
        std::cout << "Not in gem5, m5 ops won't work." << std::endl;

    // Get the addr call type, which is in an anonymous namespace. Set the
    // address to a well known constant that's nicely aligned.
    Args args({ "--addr=0x1000000" });
    if (!in_gem5) {
        // Change the file to be mmap-ed to something not dangerous, but only
        // if we're not in gem5. Otherwise we'll need this to really work.
        m5_mmap_dev = mmapDummyFile.path().c_str();
    }
    CallType *addr_call_type = CallType::detect(args);
    EXPECT_NE(addr_call_type, nullptr);
    EXPECT_EQ(m5op_addr, MmapPhysAddr);

    verify_mmap();

    // Get the dispatch table associated with it.
    const auto &dt = addr_call_type->getDispatch();

    // If we're in gem5, then we should be able to run the "sum" command.
    if (in_gem5) {
        EXPECT_EQ((*dt.m5_sum)(2, 2, 0, 0, 0, 0), 4);
        return;
    }

    // If not, then we'll need to try to catch the fall out from trying to run
    // an m5 op and verify that what we were trying looks correct.

    // Block access to the page that was mapped.
    mprotect(m5_mem, 0x10000, 0);

    struct sigaction sigsegv_action;
    std::memset(&sigsegv_action, 0, sizeof(sigsegv_action));
    sigsegv_action.sa_sigaction = &sigsegv_handler;
    sigsegv_action.sa_flags = SA_SIGINFO | SA_RESETHAND;

    struct sigaction old_sigsegv_action;

    sigaction(SIGSEGV, &sigsegv_action, &old_sigsegv_action);

    if (!sigsetjmp(interceptEnv, 1)) {
        (*dt.m5_sum)(2, 2, 0, 0, 0, 0);
        sigaction(SIGSEGV, &old_sigsegv_action, nullptr);
        ADD_FAILURE() << "Didn't die when attempting to run \"sum\".";
        return;
    }

    // Restore access to the page that was mapped.
    mprotect(m5_mem, 0x10000, PROT_READ | PROT_WRITE);

    // Back from siglongjump.
    auto &info = interceptSiginfo;

    EXPECT_EQ(info.si_signo, SIGSEGV);
    EXPECT_EQ(info.si_code, SEGV_ACCERR);

    uintptr_t access_addr = (uintptr_t)info.si_addr;
    uintptr_t virt_addr = (uintptr_t)m5_mem;

    // Verify that the address was in the right area.
    EXPECT_LT(access_addr, virt_addr + 0x10000);
    EXPECT_GE(access_addr, virt_addr);

    // Extract the func number.
    uintptr_t offset = access_addr - virt_addr;
    int func = (offset & 0xff00) >> 8;
    EXPECT_EQ(func, M5OP_SUM);
}

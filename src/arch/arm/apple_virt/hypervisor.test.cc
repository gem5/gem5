/*
 * Copyright (c) 2026 The Regents of The University of California
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

#include <Hypervisor/hv.h>
#include <Hypervisor/hv_vcpu.h>
#include <Hypervisor/hv_vm.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <array>
#include <cstdint>

namespace
{

class HypervisorTest : public testing::Test
{
  protected:
    void
    SetUp() override
    {
        ASSERT_EQ(hv_vm_create(nullptr), HV_SUCCESS);
        vmCreated = true;
    }

    void
    TearDown() override
    {
        if (vcpuCreated) {
            EXPECT_EQ(hv_vcpu_destroy(vcpu), HV_SUCCESS);
        }
        if (memoryMapped) {
            EXPECT_EQ(hv_vm_unmap(GuestAddress, pageSize), HV_SUCCESS);
        }
        if (hostPage != MAP_FAILED) {
            EXPECT_EQ(munmap(hostPage, pageSize), 0);
        }
        if (vmCreated) {
            EXPECT_EQ(hv_vm_destroy(), HV_SUCCESS);
        }
    }

    void
    mapGuestPage()
    {
        pageSize = sysconf(_SC_PAGESIZE);
        ASSERT_GT(pageSize, 0);
        hostPage = mmap(nullptr, pageSize, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANON, -1, 0);
        ASSERT_NE(hostPage, MAP_FAILED);
        ASSERT_EQ(hv_vm_map(hostPage, GuestAddress, pageSize,
                            HV_MEMORY_READ | HV_MEMORY_EXEC),
                  HV_SUCCESS);
        memoryMapped = true;
    }

    void
    createVCPU()
    {
        ASSERT_EQ(hv_vcpu_create(&vcpu, &exit, nullptr), HV_SUCCESS);
        vcpuCreated = true;
    }

    static constexpr hv_ipa_t GuestAddress = 0x100000;
    bool vmCreated = false;
    bool memoryMapped = false;
    bool vcpuCreated = false;
    long pageSize = 0;
    void *hostPage = MAP_FAILED;
    hv_vcpu_t vcpu = 0;
    hv_vcpu_exit_t *exit = nullptr;
};

TEST_F(HypervisorTest, CreatesVCPUAndSynchronizesRegisters)
{
    createVCPU();

    constexpr uint64_t value = 0x123456789abcdef0;
    ASSERT_EQ(hv_vcpu_set_reg(vcpu, HV_REG_X0, value), HV_SUCCESS);

    uint64_t observed = 0;
    ASSERT_EQ(hv_vcpu_get_reg(vcpu, HV_REG_X0, &observed), HV_SUCCESS);
    EXPECT_EQ(observed, value);
}

TEST_F(HypervisorTest, SynchronizesPointerAuthenticationKeys)
{
    createVCPU();

    constexpr std::array keys = {
        HV_SYS_REG_APIAKEYLO_EL1, HV_SYS_REG_APIAKEYHI_EL1,
        HV_SYS_REG_APIBKEYLO_EL1, HV_SYS_REG_APIBKEYHI_EL1,
        HV_SYS_REG_APDAKEYLO_EL1, HV_SYS_REG_APDAKEYHI_EL1,
        HV_SYS_REG_APDBKEYLO_EL1, HV_SYS_REG_APDBKEYHI_EL1,
        HV_SYS_REG_APGAKEYLO_EL1, HV_SYS_REG_APGAKEYHI_EL1,
    };

    constexpr uint64_t value = 0x123456789abcdef0;
    for (const auto key : keys) {
        ASSERT_EQ(hv_vcpu_set_sys_reg(vcpu, key, value), HV_SUCCESS);
        uint64_t observed = 0;
        ASSERT_EQ(hv_vcpu_get_sys_reg(vcpu, key, &observed), HV_SUCCESS);
        EXPECT_EQ(observed, value);
    }
}

TEST_F(HypervisorTest, SynchronizesCacheSelector)
{
    createVCPU();

    constexpr uint64_t value = 2;
    ASSERT_EQ(hv_vcpu_set_sys_reg(vcpu, HV_SYS_REG_CSSELR_EL1, value),
              HV_SUCCESS);

    uint64_t observed = 0;
    ASSERT_EQ(hv_vcpu_get_sys_reg(vcpu, HV_SYS_REG_CSSELR_EL1, &observed),
              HV_SUCCESS);
    EXPECT_EQ(observed, value);
}

TEST_F(HypervisorTest, ExecutesGuestCodeAndReportsExit)
{
    mapGuestPage();
    createVCPU();

    // HVC #0. Hypervisor.framework runs the instruction at EL1 and reports
    // the resulting HVC64 exception to the host.
    constexpr uint32_t hvc = 0xd4000002;
    *static_cast<uint32_t *>(hostPage) = hvc;

    ASSERT_EQ(hv_vcpu_set_reg(vcpu, HV_REG_PC, GuestAddress), HV_SUCCESS);
    ASSERT_EQ(hv_vcpu_set_reg(vcpu, HV_REG_CPSR, 0x3c5), HV_SUCCESS);
    ASSERT_EQ(hv_vcpu_run(vcpu), HV_SUCCESS);

    ASSERT_NE(exit, nullptr);
    EXPECT_EQ(exit->reason, HV_EXIT_REASON_EXCEPTION);
    EXPECT_EQ(exit->exception.syndrome >> 26, 0x16);
}

} // anonymous namespace

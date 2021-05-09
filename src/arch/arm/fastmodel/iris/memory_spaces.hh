/*
 * Copyright 2019 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_IRIS_MEMORY_SPACES_HH__
#define __ARCH_ARM_FASTMODEL_IRIS_MEMORY_SPACES_HH__

namespace gem5
{

namespace Iris
{

enum CanonicalMsn
{
    SecureMonitorMsn = 0x1000,
    GuestMsn = 0x1001,
    NsHypMsn = 0x1002,
    MemoryMsn = 0x1003,
    HypAppMsn = 0x1004,
    HostMsn = 0x1005,
    CurrentMsn = 0x10ff,
    IpaMsn = 0x1100,
    PhysicalMemorySecureMsn = 0x1200,
    PhysicalMemoryNonSecureMsn = 0x1201,
    PhysicalMemoryMsn = 0x1202
};

} // namespace Iris
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_IRIS_MEMORY_SPACES_HH__

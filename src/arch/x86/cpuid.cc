/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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

#include "arch/x86/cpuid.hh"

#include "arch/x86/isa.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"
#include "debug/X86.hh"

namespace gem5
{

namespace X86ISA
{

X86CPUID::X86CPUID(const std::string& vendor, const std::string& name)
    : vendorString(vendor), nameString(name)
{
    fatal_if(vendorString.size() != 12,
             "CPUID vendor string must be 12 characters\n");
}

void
X86CPUID::addStandardFunc(uint32_t func, std::vector<uint32_t> values)
{
    capabilities[func] = values;
}

void
X86CPUID::addExtendedFunc(uint32_t func, std::vector<uint32_t> values)
{
    // Extended functions begin with 8000_0000h, but the enum is based from
    // zero, so we need to add that to the function value.
    capabilities[func | 0x80000000] = values;
}

bool
X86CPUID::doCpuid(ThreadContext * tc, uint32_t function, uint32_t index,
                  CpuidResult &result)
{
    constexpr uint32_t ext = 0x80000000;

    DPRINTF(X86, "Calling CPUID function %x with index %d\n", function, index);

    // Handle the string-related CPUID functions specially
    if (function == VendorAndLargestStdFunc) {
        result = CpuidResult(NumStandardCpuidFuncs - 1,
                             stringToRegister(vendorString.c_str()),
                             stringToRegister(vendorString.c_str() + 4),
                             stringToRegister(vendorString.c_str() + 8));

        return true;
    } else if (function == (ext | VendorAndLargestExtFunc)) {
        result = CpuidResult(0x80000000 + NumExtendedCpuidFuncs - 1,
                             stringToRegister(vendorString.c_str()),
                             stringToRegister(vendorString.c_str() + 4),
                             stringToRegister(vendorString.c_str() + 8));

        return true;
    } else if ((function == (ext | NameString1)) ||
               (function == (ext | NameString2)) ||
               (function == (ext | NameString3))) {
        // Zero fill anything beyond the end of the string. This
        // should go away once the string is a vetted parameter.
        char cleanName[nameStringSize];
        memset(cleanName, '\0', nameStringSize);
        strncpy(cleanName, nameString.c_str(), nameStringSize-1);

        int funcNum = bits(function, 15, 0);
        int offset = (funcNum - NameString1) * 16;
        assert(nameStringSize >= offset + 16);
        result = CpuidResult(
                stringToRegister(cleanName + offset + 0),
                stringToRegister(cleanName + offset + 4),
                stringToRegister(cleanName + offset + 12),
                stringToRegister(cleanName + offset + 8));

        return true;
    }

    // Ignore anything not in the map of supported CPUID functions.
    // This is checked after the string-related functions as those are not
    // in the capabilities map.
    if (!capabilities.count(function)) {
        return false;
    }

    int cap_offset = 0;

    // Ignore index values for functions that do not take index values.
    if (hasSignificantIndex(function)) {
        cap_offset = index * 4;
    }

    // Ensure we have the offset and 4 dwords after it.
    assert(capabilities[function].size() >= (cap_offset + 4));

    auto &cap_vec = capabilities[function];
    result = CpuidResult(cap_vec[cap_offset + 0], cap_vec[cap_offset + 1],
                         cap_vec[cap_offset + 2], cap_vec[cap_offset + 3]);
    DPRINTF(X86, "CPUID function %x returning (%x, %x, %x, %x)\n",
            function, result.rax, result.rbx, result.rdx, result.rcx);

    return true;
}

uint64_t
X86CPUID::stringToRegister(const char *str)
{
    uint64_t reg = 0;
    for (int pos = 3; pos >=0; pos--) {
        reg <<= 8;
        reg |= str[pos];
    }
    return reg;
}

// Return true if the CPUID function takes ECX index as an input AND
// those multiple index values are supported in gem5.
bool
X86CPUID::hasSignificantIndex(uint32_t function)
{
    return false;
}

} // namespace X86ISA
} // namespace gem5

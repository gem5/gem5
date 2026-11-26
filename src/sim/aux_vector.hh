/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#ifndef __AUX_VECTOR_HH__
#define __AUX_VECTOR_HH__

#include "base/compiler.hh"
#include "sim/byteswap.hh"

namespace gem5
{

namespace auxv {

template<class IntType>
class AuxVector
{
  public:
    AuxVector() = default;
    AuxVector(IntType _type, IntType _val) : type(_type), val(_val) {}

    IntType type = 0;
    IntType val = 0;
};

// Ensure the global versions of swap_byte are visible.
using gem5::swap_byte;

// Define swap_byte in this namespace, so argument dependent resolution can
// find it.
template <class IntType>
inline AuxVector<IntType>
swap_byte(const AuxVector<IntType> &av)
{
    return AuxVector<IntType>(swap_byte(av.type), swap_byte(av.val));
}

enum Type
{
    Null = 0,          // End of vector.
    Ignore = 1,        // Ignored.
    Execfd = 2,        // File descriptor of program if interpreter used.
    Phdr = 3,          // Address of program header tables in memory.
    Phent = 4,         // Size in bytes of one program header entry.
    Phnum = 5,         // Number of entries in program header table.
    Pagesz = 6,        // System page size.
    Base = 7,          // Base address of interpreter program in memory.
    Flags = 8,         // Unused.
    Entry = 9,         // Entry point of program after interpreter setup.
    Notelf = 10,       // Non-zero if format is different than ELF.
    Uid = 11,          // Address of real user ID of thread.
    Euid = 12,         // Address of effective user ID of thread.
    Gid = 13,          // Address of real group ID of thread.
    Egid = 14,         // Address of effective group ID of thread.
    Platform = 15,     // Platform string for the architecture.
    Hwcap = 16,        // Bits which describe the hardware capabilities.
    Clktck = 17,       // Frequency at which times() syscall increments.
    Secure = 23,       // Whether to enable "secure mode" in executable.
    BasePlatform = 24, // Platform string (differs on PowerPC only).
    Random = 25,       // Pointer to 16 bytes of random data.
    Hwcap2 = 26,       // Extension of AT_HWCAP.
    Execfn = 31,       // Filename of the program.
    VectorSize = 44
};

} // namespace auxv

} // namespace gem5

#endif // __AUX_VECTOR_HH__

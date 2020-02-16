/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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

template<class IntType>
class AuxVector
{
  public:
    AuxVector() = default;
    AuxVector(IntType _type, IntType _val) : type(_type), val(_val) {}

    IntType type = 0;
    IntType val = 0;
};

template<class IntType>
inline AuxVector<IntType>
swap_byte(AuxVector<IntType> av)
{
    av.type = swap_byte(av.type);
    av.val = swap_byte(av.val);
    return av;
}

enum AuxiliaryVectorType {
    M5_AT_NULL = 0,        // End of vector.
    M5_AT_IGNORE = 1,      // Ignored.
    M5_AT_EXECFD = 2,      // File descriptor of program if interpreter used.
    M5_AT_PHDR = 3,        // Address of program header tables in memory.
    M5_AT_PHENT = 4,       // Size in bytes of one program header entry.
    M5_AT_PHNUM = 5,       // Number of entries in program header table.
    M5_AT_PAGESZ = 6,      // System page size.
    M5_AT_BASE = 7,        // Base address of interpreter program in memory.
    M5_AT_FLAGS = 8,       // Unused.
    M5_AT_ENTRY = 9,       // Entry point of program after interpreter setup.
    M5_AT_NOTELF = 10,     // Non-zero if format is different than ELF.
    M5_AT_UID = 11,        // Address of real user ID of thread.
    M5_AT_EUID = 12,       // Address of effective user ID of thread.
    M5_AT_GID = 13,        // Address of real group ID of thread.
    M5_AT_EGID = 14,       // Address of effective group ID of thread.
    M5_AT_PLATFORM = 15,   // Platform string for the architecture.
    M5_AT_HWCAP = 16,      // Bits which describe the hardware capabilities.
    M5_AT_CLKTCK = 17,     // Frequency at which times() syscall increments.
    M5_AT_SECURE = 23,     // Whether to enable "secure mode" in executable.
    M5_BASE_PLATFORM = 24, // Platform string (differs on PowerPC only).
    M5_AT_RANDOM = 25,     // Pointer to 16 bytes of random data.
    M5_AT_HWCAP2 = 26,     // Extension of AT_HWCAP.
    M5_AT_EXECFN = 31,     // Filename of the program.
    M5_AT_VECTOR_SIZE = 44
};

#endif // __AUX_VECTOR_HH__

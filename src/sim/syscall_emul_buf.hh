/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __SIM_SYSCALL_EMUL_BUF_HH__
#define __SIM_SYSCALL_EMUL_BUF_HH__

///
/// @file syscall_emul_buf.hh
///
/// This file defines buffer classes used to handle pointer arguments
/// in emulated syscalls.

#include <cstring>

#include "base/types.hh"
#include "mem/se_translating_port_proxy.hh"

namespace gem5
{

/**
 * Base class for BufferArg and TypedBufferArg, Not intended to be
 * used directly.
 *
 * The BufferArg classes represent buffers in target user space that
 * are passed by reference to an (emulated) system call.  Each
 * instance provides an internal (simulator-space) buffer of the
 * appropriate size and tracks the user-space address.  The copyIn()
 * and copyOut() methods copy the user-space buffer to and from the
 * simulator-space buffer, respectively.
 */
class BaseBufferArg
{
  public:
    /**
     * Allocate a buffer of size 'size' representing the memory at
     * target address 'addr'.
     */
    BaseBufferArg(Addr _addr, int _size)
        : addr(_addr), size(_size), bufPtr(new uint8_t[size])
    {
        // clear out buffer: in case we only partially populate this,
        // and then do a copyOut(), we want to make sure we don't
        // introduce any random junk into the simulated address space
        memset(bufPtr, 0, size);
    }

    ~BaseBufferArg() { delete[] bufPtr; }

    /**
     * copy data into simulator space (read from target memory)
     */
    bool
    copyIn(const PortProxy &memproxy)
    {
        memproxy.readBlob(addr, bufPtr, size);
        return true; // no EFAULT detection for now
    }

    /**
     * copy data out of simulator space (write to target memory)
     */
    bool
    copyOut(const PortProxy &memproxy)
    {
        memproxy.writeBlob(addr, bufPtr, size);
        return true; // no EFAULT detection for now
    }

  protected:
    const Addr addr;       ///< address of buffer in target address space
    const int size;        ///< buffer size
    uint8_t *const bufPtr; ///< pointer to buffer in simulator space
};

/**
 * BufferArg represents an untyped buffer in target user space that is
 * passed by reference to an (emulated) system call.
 */
class BufferArg : public BaseBufferArg
{
  public:
    /**
     * Allocate a buffer of size 'size' representing the memory at
     * target address 'addr'.
     */
    BufferArg(Addr _addr, int _size) : BaseBufferArg(_addr, _size) {}

    /**
     * Return a pointer to the internal simulator-space buffer.
     */
    void *
    bufferPtr()
    {
        return bufPtr;
    }
};

/**
 * TypedBufferArg is a class template; instances of this template
 * represent typed buffers in target user space that are passed by
 * reference to an (emulated) system call.
 *
 * This template provides operator overloads for convenience, allowing
 * for example the use of '->' to reference fields within a struct
 * type.
 */
template <class T>
class TypedBufferArg : public BaseBufferArg
{
  public:
    /**
     * Allocate a buffer of type T representing the memory at target
     * address 'addr'. The user can optionally specify a specific
     * number of bytes to allocate to deal with structs that have
     * variable-size arrays at the end.
     */
    TypedBufferArg(Addr _addr, int _size = sizeof(T))
        : BaseBufferArg(_addr, _size)
    {}

    /**
     * Convert TypedBufferArg<T> to a pointer to T that points to the
     * internal buffer.
     */
    operator T *() { return (T *)bufPtr; }

    /**
     * Convert TypedBufferArg<T> to a reference to T that references the
     * internal buffer value.
     */
    T &
    operator*()
    {
        return *((T *)bufPtr);
    }

    /**
     * Enable the use of '->' to reference fields where T is a struct
     * type.
     */
    T *
    operator->()
    {
        return (T *)bufPtr;
    }

    /**
     * Enable the use of '[]' to reference fields where T is an array
     * type.
     */
    T &
    operator[](int i)
    {
        return ((T *)bufPtr)[i];
    }
};

} // namespace gem5

#endif // __SIM_SYSCALL_EMUL_BUF_HH__

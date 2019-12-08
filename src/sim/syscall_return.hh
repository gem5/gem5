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

#ifndef __SIM_SYSCALLRETURN_HH__
#define __SIM_SYSCALLRETURN_HH__

#include <inttypes.h>

/**
 * This class represents the return value from an emulated system call,
 * including any errno setting.
 *
 * On some platforms, the return value and errno are encoded in a
 * single signed integer.  A value less than zero but greater than
 * -4096 indicates an error, and the value is the negation of the
 * errno value.  Otherwise, the call was successful and the integer is
 * the return value.  (Large negative numbers are considered
 * successful to allow syscalls to return pointers to high memory,
 * e.g., stack addresses.)  See, for example, Appendix A of the AMD64
 * ABI spec at http://www.x86-64.org/documentation/abi.pdf.
 *
 * Other platforms use a more complex interface, returning a value and
 * an error code in separate registers.
 *
 * This class is designed to support both types of interfaces.
 */
class SyscallReturn
{
  public:

    /// For simplicity, allow the object to be initialized with a
    /// single signed integer using the same positive=success,
    /// negative=-errno convention described above.
    ///
    /// Typically this constructor is used as a default type
    /// conversion, so a bare integer is used where a SyscallReturn
    /// value is expected, e.g., as the return value from a system
    /// call emulation function ('return 0;' or 'return -EFAULT;').
    SyscallReturn(int64_t v) : _value(v), _count(1) {}

    /// A SyscallReturn constructed with no value means don't return anything.
    SyscallReturn() : _count(0) {}

    /// A SyscallReturn constructed with two values means put the second value
    /// in additional return registers as defined by the ABI, if they exist.
    SyscallReturn(int64_t v1, int64_t v2) :
        _value(v1), _value2(v2), _count(2)
    {}

    /// Pseudo-constructor to create an instance with the retry flag set.
    static SyscallReturn
    retry()
    {
        SyscallReturn s(0);
        s.retryFlag = true;
        return s;
    }

    ~SyscallReturn() {}

    /// Was the system call successful?
    bool
    successful() const
    {
        return (_value >= 0 || _value <= -4096);
    }

    /// Does the syscall need to be retried?
    bool needsRetry() const { return retryFlag; }

    /// Should returning this value be suppressed?
    bool suppressed() const { return _count == 0; }

    /// How many values did the syscall attempt to return?
    int count() const { return _count; }

    /// The return value
    int64_t
    returnValue() const
    {
        assert(successful());
        return _value;
    }

    /// The errno value
    int
    errnoValue() const
    {
        assert(!successful());
        return -_value;
    }

    /// The encoded value (as described above)
    int64_t encodedValue() const { return _value; }
    int64_t value2() const { return _value2; }

  private:
    int64_t _value, _value2;
    int _count;

    bool retryFlag = false;
};

#endif

/*
 * Copyright (c) 2011, 2017 ARM Limited
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

#ifndef __ARGS_HH__
#define __ARGS_HH__

#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

class Args
{
  private:
    std::vector<std::string> args;
    size_t offset = 0;

  public:
    Args(int argc, const char **argv)
    {
        for (int i = 0; i < argc; i++)
            args.push_back(argv[i]);
    }

    Args(std::initializer_list<std::string> strings) : args(strings) {}

    /*
     * Attempt to convert str into an integer.
     *
     * Return whether that succeeded.
     */
    static bool
    stoi(const std::string &str, uint64_t &val)
    {
        try {
            val = std::stoi(str, nullptr, 0);
            return true;
        } catch (const std::invalid_argument &e) {
            return false;
        } catch (const std::out_of_range &e) {
            return false;
        }
    }

    /*
     * Attempt to convert str into an integer.
     *
     * Return whether that suceeded. If not, val will be set to def.
     */
    static bool
    stoi(const std::string &str, uint64_t &val, uint64_t def)
    {
        val = def;
        return stoi(str, val);
    }

    /*
     * Attempt to pack str as a sequence of bytes in to regs in little endian
     * byte order.
     *
     * Return whether that succeeded.
     */
    static bool pack(const std::string &str, uint64_t regs[], int num_regs);

    /*
     * If there are any arguments in the list, remove and return the first
     * one. If not, return def instead.
     */
    const std::string &
    pop(const std::string &def = "")
    {
        if (!size())
            return def;
        return args[offset++];
    }

    /*
     * If there are any arguments in the list, attempt to convert the first
     * one to an integer. If successful, remove it and store its value in val.
     *
     * Return whether that succeeded.
     */
    bool
    pop(uint64_t &val)
    {
        if (!size() || !stoi(args[offset], val)) {
            return false;
        } else {
            offset++;
            return true;
        }
    }

    /*
     * If there are any arguments in the list, attempt to convert the first
     * one to an integer. If successful, remove it and store its value in val.
     * If there are no arguments or the conversion failed, set val to def.
     *
     * Return true if there were no arguments, or there were and the conversion
     * succeeded.
     */
    bool
    pop(uint64_t &val, uint64_t def)
    {
        val = def;
        if (!size())
            return true;
        if (stoi(args[offset], val)) {
            offset++;
            return true;
        }
        return false;
    }

    /*
     * If there are any arguments in the list, attempt to pack the first one
     * as a sequence of bytes into the integers in regs in little endian byte
     * order.
     *
     * Return whether that succeeded.
     */
    bool
    pop(uint64_t regs[], int num_regs)
    {
        if (!size() || !pack(args[offset], regs, num_regs)) {
            return false;
        } else {
            offset++;
            return true;
        }
    }

    size_t
    size()
    {
        return args.size() - offset;
    }

    const std::string &
    operator[](size_t idx)
    {
        return args[offset + idx];
    }
};

#endif // __ARGS_HH__

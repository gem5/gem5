/*
 * Copyright 1990 Hewlett-Packard Development Company, L.P.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __ARCH_MIPS_LINUX_HWRPB_HH__
#define __ARCH_MIPS_LINUX_HWRPB_HH__

#include "arch/mips/linux/aligned.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Linux, linux);
namespace linux
{
    struct pcb_struct
    {
        uint64_ta rpb_ksp;
        uint64_ta rpb_usp;
        uint64_ta rpb_ptbr;
        uint32_t rpb_cc;
        uint32_t rpb_psn;
        uint64_ta rpb_unique;
        uint64_ta rpb_fen;
        uint64_ta res1, res2;
    };
} // namespace linux
} // namespace gem5

#endif // __ARCH_MIPS_LINUX_HWRPB_HH__

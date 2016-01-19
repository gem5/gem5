// University of Illinois/NCSA
// Open Source License
//
// Copyright (c) 2013, Advanced Micro Devices, Inc.
// All rights reserved.
//
// Developed by:
//
//     HSA Team
//
//     Advanced Micro Devices, Inc
//
//     www.amd.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal with
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//
//     * Redistributions of source code must retain the above copyright notice,
//       this list of conditions and the following disclaimers.
//
//     * Redistributions in binary form must reproduce the above copyright notice,
//       this list of conditions and the following disclaimers in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the names of the LLVM Team, University of Illinois at
//       Urbana-Champaign, nor the names of its contributors may be used to
//       endorse or promote products derived from this Software without specific
//       prior written permission.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE
// SOFTWARE.
#ifndef INTERNAL_BRIG_H
#define INTERNAL_BRIG_H

#include <stdint.h>

namespace Brig {
#include "Brig_new.hpp"

// These typedefs provide some backward compatibility with earlier versions
// of Brig.h, reducing the number of code changes. The distinct names also
// increase legibility by showing the code's intent.
typedef BrigBase BrigDirective;
typedef BrigBase BrigOperand;

enum BrigMemoryFenceSegments { // for internal use only
    //.mnemo={ s/^BRIG_MEMORY_FENCE_SEGMENT_//;lc }
    //.mnemo_token=_EMMemoryFenceSegments
    //.mnemo_context=EInstModifierInstFenceContext
    BRIG_MEMORY_FENCE_SEGMENT_GLOBAL = 0,
    BRIG_MEMORY_FENCE_SEGMENT_GROUP = 1,
    BRIG_MEMORY_FENCE_SEGMENT_IMAGE = 2,
    BRIG_MEMORY_FENCE_SEGMENT_LAST = 3 //.skip
};

}

#endif // defined(INTERNAL_BRIG_H)

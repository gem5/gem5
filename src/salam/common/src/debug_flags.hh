/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef LLVMREAD_DEBUG_HH
#define LLVMREAD_DEBUG_HH

// Doxygen setup
/**
 * @defgroup flags Flags group
 */

// File description
/**
 * @file
 * Contains include information for all M5 debug flags and base
 * implementation of the Debugger class used within gem5-SALAM.
 */

// Global includes
// - debug_flags.hh is a common header for all files, so all files included
// - here will available throughout the entire application
#include "macros.hh"

// Debug includes
#include "debug/AddrRanges.hh"
#include "debug/CommInterface.hh"
#include "debug/CommInterfaceQueues.hh"
#include "debug/DMA.hh"
#include "debug/DeviceMMR.hh"
#include "debug/LLVMInterface.hh"
#include "debug/NoncoherentDma.hh"
#include "debug/Runtime.hh"
#include "debug/RuntimeCompute.hh"
#include "debug/RuntimeQueues.hh"
#include "debug/SALAM_Debug.hh"
#include "debug/Step.hh"
#include "debug/StreamDma.hh"
#include "debug/Trace.hh"

// M5 includes
#include "base/trace.hh"

/**
 * @namespace SALAM
 */
namespace SALAM
{

/**
 * @class Debugger
 * Base implementation of the debugger used in gem5-SALAM
 */
class Debugger
{
  private:
    bool dbg = false;

  protected:
  public:
    Debugger();
    ~Debugger() = default;
    virtual void dumper() {};
    bool
    enabled()
    {
        return dbg;
    }
};
} // namespace SALAM
#endif //__LLVMREAD_DEBUG_HH__

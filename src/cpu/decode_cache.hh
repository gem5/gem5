/*
 * Copyright (c) 2011 Google
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
 *
 * Authors: Gabe Black
 */

#ifndef __CPU_DECODE_CACHE_HH__
#define __CPU_DECODE_CACHE_HH__

#include "arch/isa_traits.hh"
#include "arch/types.hh"
#include "base/hashmap.hh"
#include "config/the_isa.hh"
#include "cpu/static_inst.hh"

namespace TheISA
{
    class Decoder;
}

class DecodeCache
{
  private:
    typedef TheISA::ExtMachInst ExtMachInst;

    /// Hash of decoded instructions.
    typedef m5::hash_map<ExtMachInst, StaticInstPtr> InstMap;
    InstMap instMap;
    struct DecodePage {
        StaticInstPtr insts[TheISA::PageBytes];
    };

    /// A store of DecodePages. Basically a slightly smarter hash_map.
    class DecodePages
    {
      protected:
        typedef typename m5::hash_map<Addr, DecodePage *> PageMap;
        typedef typename PageMap::iterator PageIt;
        PageIt recent[2];
        PageMap pageMap;

        /// Update the small cache of recent lookups.
        /// @param recentest The most recent result;
        void update(PageIt recentest);
        void addPage(Addr addr, DecodePage *page);

      public:
        /// Constructor
        DecodePages();

        /// Attempt to find the DecodePage which goes with a particular
        /// address. First check the small cache of recent results, then
        /// actually look in the hash_map.
        /// @param addr The address to look up.
        DecodePage *getPage(Addr addr);
    } decodePages;

  public:
    /// Decode a machine instruction.
    /// @param mach_inst The binary instruction to decode.
    /// @retval A pointer to the corresponding StaticInst object.
    StaticInstPtr decode(TheISA::Decoder * const decoder,
            ExtMachInst mach_inst, Addr addr);
};

#endif // __CPU_DECODE_CACHE_HH__

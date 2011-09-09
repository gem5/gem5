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

typedef StaticInstPtr (*DecodeInstFunc)(TheISA::ExtMachInst);

template <DecodeInstFunc decodeInstFunc>
class DecodeCache
{
  private:
    typedef TheISA::ExtMachInst ExtMachInst;

    /// Hash of decoded instructions.
    typedef m5::hash_map<ExtMachInst, StaticInstPtr> InstMap;
    InstMap instMap;

    /// A table of instructions which are already been decoded, indexed by
    /// page offset.
    class DecodePage
    {
      protected:
        StaticInstPtr instructions[TheISA::PageBytes];

        // A helper function to compute the index of an address in the table.
        Addr offset(Addr addr) { return addr & (TheISA::PageBytes - 1); }

      public:
        /// Decode the given instruction. First attempt to find it in the
        /// table, then in the generic decode cache, and finally call the
        /// actual decode function.
        ///
        /// @param mach_inst The predecoded instruction to decode.
        /// @param addr The address the instruction came from.
        /// @param cache A cache of already decoded instructions.
        /// @retval The decoded instruction object.
        StaticInstPtr
        decode(const ExtMachInst &mach_inst, Addr addr, InstMap &instMap)
        {
            StaticInstPtr si = instructions[offset(addr)];
            if (si && (si->machInst == mach_inst)) {
                return si;
            }

            InstMap::iterator iter = instMap.find(mach_inst);
            if (iter != instMap.end()) {
                si = iter->second;
            } else {
                si = decodeInstFunc(mach_inst);
                instMap[mach_inst] = si;
            }

            instructions[offset(addr)] = si;
            return si;
        }
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
        void
        update(PageIt recentest)
        {
            recent[1] = recent[0];
            recent[0] = recentest;
        }

      public:
        /// Constructor
        DecodePages()
        {
            recent[0] = recent[1] = pageMap.end();
        }

        /// Attempt to find the DecodePage which goes with a particular
        /// address. First check the small cache of recent results, then
        /// actually look in the hash_map.
        /// @param addr The address to look up.
        DecodePage *
        findPage(Addr addr)
        {
            Addr page_addr = addr & ~(TheISA::PageBytes - 1);

            // Check against recent lookups.
            if (recent[0] != pageMap.end()) {
                if (recent[0]->first == page_addr)
                    return recent[0]->second;
                if (recent[1] != pageMap.end() &&
                        recent[1]->first == page_addr) {
                    update(recent[1]);
                    // recent[1] has just become recent[0].
                    return recent[0]->second;
                }
            }

            // Actually look in the has_map.
            PageIt it = pageMap.find(page_addr);
            if (it != pageMap.end()) {
                update(it);
                return it->second;
            }

            // Didn't find it so return NULL.
            return NULL;
        }

        void
        addPage(Addr addr, DecodePage *page)
        {
            Addr page_addr = addr & ~(TheISA::PageBytes - 1);
            typename PageMap::value_type to_insert(page_addr, page);
            update(pageMap.insert(to_insert).first);
        }
    } decodePages;

  public:
    /// Decode a machine instruction.
    /// @param mach_inst The binary instruction to decode.
    /// @retval A pointer to the corresponding StaticInst object.
    StaticInstPtr
    decode(ExtMachInst mach_inst, Addr addr)
    {
        // Try to find a matching address based table of instructions.
        DecodePage *page = decodePages.findPage(addr);
        if (!page) {
            // Nothing was found, so create a new one.
            page = new DecodePage;
            decodePages.addPage(addr, page);
        }

        // Use the table to decode the instruction. It will fall back to other
        // mechanisms if it needs to.
        return page->decode(mach_inst, addr, instMap);
    }
};

#endif // __CPU_DECODE_CACHE_HH__

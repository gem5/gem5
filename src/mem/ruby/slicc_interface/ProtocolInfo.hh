/* Copyright (c) 2024 Jason Lowe-Power
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

#ifndef __MEM_RUBY_SLICC_INTERFACE_PROTOCOL_INFO_HH__
#define __MEM_RUBY_SLICC_INTERFACE_PROTOCOL_INFO_HH__

#include <string>

namespace gem5 {

namespace ruby {

/*
 * This class is used to store information about a protocol.
 * Each protocol will inherit from this class and define the values
 * To add a new option, update this file and slicc/parser.py self.options
 */
class ProtocolInfo
{
  private:
    const std::string name;

  protected:
    const bool partialFuncReads;
    const bool useSecondaryLoadLinked;
    const bool useSecondaryStoreConditional;

  public:
    ProtocolInfo(std::string name, bool partial_func_reads,
                 bool use_secondary_load_linked,
                 bool use_secondary_store_conditional) :
        name(name),
        partialFuncReads(partial_func_reads),
        useSecondaryLoadLinked(use_secondary_load_linked),
        useSecondaryStoreConditional(use_secondary_store_conditional)
    {
    }

    std::string getName() const { return name; }
    bool getPartialFuncReads() const { return partialFuncReads; }
    bool
    getUseSecondaryLoadLinked() const
    {
        return useSecondaryLoadLinked;
    }
    bool
    getUseSecondaryStoreConditional() const
    {
        return useSecondaryStoreConditional;
    }

};

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_SLICC_INTERFACE_PROTOCOL_INFO_HH__

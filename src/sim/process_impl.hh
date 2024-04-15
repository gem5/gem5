/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

#ifndef __SIM_PROCESS_IMPL_HH__
#define __SIM_PROCESS_IMPL_HH__

#include <string>
#include <vector>

#include "mem/se_translating_port_proxy.hh"

namespace gem5
{

// This needs to be templated for cases where 32 bit pointers are needed.
template <class AddrType>
void
copyStringArray(std::vector<std::string> &strings, AddrType array_ptr,
                AddrType data_ptr, const ByteOrder bo, PortProxy &memProxy)
{
    AddrType data_ptr_swap;
    for (std::vector<std::string>::size_type i = 0; i < strings.size(); ++i) {
        data_ptr_swap = htog(data_ptr, bo);
        memProxy.writeBlob(array_ptr, &data_ptr_swap, sizeof(AddrType));
        memProxy.writeString(data_ptr, strings[i].c_str());
        array_ptr += sizeof(AddrType);
        data_ptr += strings[i].size() + 1;
    }
    // add NULL terminator
    data_ptr = 0;

    memProxy.writeBlob(array_ptr, &data_ptr, sizeof(AddrType));
}

} // namespace gem5

#endif

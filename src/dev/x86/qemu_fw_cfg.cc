/*
 * Copyright 2022 Google, Inc.
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

#include "dev/x86/qemu_fw_cfg.hh"

#include "arch/x86/bios/e820.hh"
#include "base/compiler.hh"
#include "base/logging.hh"
#include "sim/byteswap.hh"

namespace gem5
{

namespace qemu
{

FwCfgItemE820::FwCfgItemE820(const QemuFwCfgItemE820Params &p)
    : FwCfgItemFixed(p.path, p.arch_specific, p.index)
{
    struct GEM5_PACKED Entry
    {
        uint64_t addr;
        uint64_t size;
        uint32_t type;
    };

    uint32_t count = p.entries.size();

    panic_if(count >= 128, "Too many E820 entries (%d).", count);

    size_t bytes = count * sizeof(Entry);
    data.resize(bytes);

    uint8_t *ptr = data.data();

    // Write out the e820 entries.
    for (auto *e : p.entries) {
        Entry entry{ htole(e->addr), htole(e->size), htole(e->type) };
        std::memcpy(ptr, &entry, sizeof(entry));
        ptr += sizeof(entry);
    }
};

} // namespace qemu
} // namespace gem5

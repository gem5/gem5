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

#include "dev/qemu/fw_cfg.hh"

#include <cstring>
#include <sstream>
#include <string>

#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/QemuFwCfg.hh"
#include "debug/QemuFwCfgVerbose.hh"
#include "mem/packet_access.hh"
#include "sim/byteswap.hh"

namespace gem5
{

namespace qemu
{

void
FwCfgItemFixed::read(void *buf, uint64_t offset, uint32_t to_read)
{
    // Get access to the data we need to fill this buffer.
    const void *data = bytes();
    const uint64_t total_length = length();

    if (offset > total_length) {
        // We're completely off the end, so return only zeroes.
        std::memset(buf, 0, to_read);
        return;
    }

    if (offset + to_read > total_length) {
        // We're partially off the end, truncate this read and zero fill.

        // Figure out how far past the end we're attempting to read.
        uint64_t overflow = offset + to_read - total_length;

        // Reduce the requested read size to what we can actually fill.
        to_read -= overflow;

        // Zero out the part we won't read data into.
        std::memset((uint8_t *)buf + to_read, 0, overflow);
    }

    // Do the read.
    std::memcpy(buf, (uint8_t *)data + offset, to_read);
}

FwCfg::FwCfg(const Params &p, const AddrRangeList &addr_ranges)
    : PioDevice(p),
      signature(".[FW_CFG_SIGNATURE]", false, "QEMU CFG", 0),
      // The ID says we support the traditional interface but not DMA. To
      // enable DMA, this should be equal to 3.
      id(".[FW_CFG_ID]", false, "\x1", 1),
      addrRanges(addr_ranges)
{
    // Add the unnamed, fixed items.
    addItem(&signature);
    addItem(&id);

    for (auto factory : p.items) {
        // Process named items and add them to the index.
        auto &item = factory->item();

        uint32_t &next_index =
            item.archSpecific() ? nextArchIndex : nextGenericIndex;
        const uint32_t &max_index =
            item.archSpecific() ? MaxArchIndex : MaxGenericIndex;

        // Automatically assign an ID if a fixed one wasn't specified.
        if (!item.index())
            item.index(next_index++);

        panic_if(item.index() >= max_index,
                 "Firmware config device out of %s indexes.",
                 item.archSpecific() ? "arch" : "generic");

        addItem(&item);
    }

    directory.update(names, numbers);
    addItem(&directory);
};

void
FwCfg::addItem(FwCfgItem *item)
{
    const auto [kit, ksuccess] =
        numbers.insert(std::make_pair(item->index(), item));

    panic_if(!ksuccess,
             "Duplicate firmware config item key %#x, "
             "paths %s and %s.",
             item->index(), item->path(), kit->second->path());

    const std::string &path = item->path();
    if (path.empty() || path[0] != '.') {
        const auto res =
            names.insert(std::make_pair(item->path(), item->index()));

        panic_if(!res.second, "Duplicate firmware config item path %s.",
                 item->path());
    }
}

void
FwCfg::select(uint16_t key)
{
    DPRINTF(QemuFwCfg, "Selecting item with key %#x.\n", key);

    // Clear any previous selection.
    offset = 0;
    current = nullptr;

    auto iter = numbers.find(key);
    if (iter == numbers.end()) {
        warn("Firmware config failed to select item with key %#x.", key);
        return;
    }

    auto item = iter->second;

    current = item;
    if (current)
        DPRINTF(QemuFwCfg, "Selected item with path %s.\n", item->path());
    else
        DPRINTF(QemuFwCfg, "No item is currently selected.\n");
}

void
FwCfg::readItem(void *buf, uint32_t length)
{
    if (!current) {
        DPRINTF(QemuFwCfgVerbose,
                "Tried to read while nothing was selected.\n");
        std::memset(buf, 0, length);
        return;
    }

    current->read(buf, offset, length);

    if (gem5::debug::QemuFwCfgVerbose) {
        std::stringstream data_str;
        for (int idx = 0; idx < length; idx++)
            ccprintf(data_str, " %02x", ((uint8_t *)buf)[idx]);

        DPRINTF(QemuFwCfgVerbose, "Read [%#x-%#x) =>%s.\n", offset,
                offset + length, data_str.str());
    }

    offset += length;
}

FwCfg::Directory::Directory()
    : FwCfgItemFixed(".[FW_CFG_FILE_DIR]", false, 0x19)
{}

void
FwCfg::Directory::update(const std::map<std::string, uint16_t> &names,
                         const std::map<uint16_t, FwCfgItem *> &numbers)
{
    uint32_t count = names.size();

    struct GEM5_PACKED File
    {
        uint32_t size;
        uint16_t select;
        uint16_t reserved;
        char name[56];
    };

    uint64_t bytes = sizeof(count) + sizeof(File) * count;
    data.resize(bytes);

    uint8_t *ptr = data.data();

    uint32_t be_count = htobe(count);
    std::memcpy(ptr, &be_count, sizeof(be_count));
    ptr += sizeof(be_count);

    for (auto &[name, index] : names) {
        // Fill in the entry.
        File file{ (uint32_t)numbers.at(index)->length(), index, 0, {} };
        std::memset(file.name, 0, sizeof(file.name));
        std::strncpy(file.name, name.c_str(), sizeof(file.name) - 1);

        // Fix endianness.
        file.size = htobe(file.size);
        file.select = htobe(file.select);

        // Copy it to the buffer and update ptr.
        std::memcpy(ptr, &file, sizeof(file));
        ptr += sizeof(file);
    }
}

FwCfgIo::FwCfgIo(const Params &p)
    : FwCfg(p,
            { // This covers both the 16 bit selector, and the 8 bit data reg
              // which
              // overlaps it.
              { p.selector_addr, p.selector_addr + 2 } }),
      selectorAddr(p.selector_addr),
      dataAddr(p.selector_addr + 1)
{}

Tick
FwCfgIo::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const auto size = pkt->getSize();

    pkt->makeResponse();
    // The default response is all zeroes.
    std::memset(pkt->getPtr<uint8_t>(), 0, size);

    if (addr == selectorAddr) {
        warn("Read from firmware config selector register not supported.");
    } else if (addr == dataAddr) {
        if (size == 1) {
            readItem(pkt->getPtr<void>(), size);
        } else {
            warn("Read from firmware config data register with width %d not "
                 "supported.",
                 size);
        }
    } else {
        panic("Unregognized firmware config read [%#x-%#x).", addr,
              addr + size);
    }

    return 0;
}

Tick
FwCfgIo::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const auto size = pkt->getSize();

    pkt->makeResponse();

    if (addr == selectorAddr) {
        if (size != 2) {
            warn("Write to firmware config selector register with width %d "
                 "not supported.",
                 size);
        } else {
            auto key = pkt->getLE<uint16_t>();
            select(key);
        }
    } else if (addr == dataAddr) {
        // Writes to the firmware config data can only be done through the
        // DMA interface.
        warn("Write to firmware config data register not supported.");
    } else {
        panic("Unrecognized firmware config write [%#x-%#x).", addr,
              addr + size);
    }

    return 0;
}

FwCfgMmio::FwCfgMmio(const Params &p)
    : FwCfg(p, { { p.selector_addr, p.selector_addr + 2 },
                 { p.data_addr_range } }),
      selectorAddr(p.selector_addr),
      dataAddr(p.data_addr_range.start()),
      dataSize(p.data_addr_range.size())
{}

Tick
FwCfgMmio::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const auto size = pkt->getSize();

    pkt->makeResponse();
    // The default response is all zeroes.
    std::memset(pkt->getPtr<uint8_t>(), 0, size);

    if (addr == selectorAddr) {
        warn("Read from firmware config selector register not supported.");
    } else if (addr == dataAddr) {
        if (size == dataSize) {
            readItem(pkt->getPtr<void>(), size);
        } else {
            warn("Read from firmware config data register with width %d not "
                 "supported.",
                 size);
        }
    } else {
        panic("Unregognized firmware config read [%#x-%#x).", addr,
              addr + size);
    }

    return 0;
}

Tick
FwCfgMmio::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const auto size = pkt->getSize();

    pkt->makeResponse();

    if (addr == selectorAddr) {
        if (size != 2) {
            warn("Write to firmware config selector register with width %d "
                 "not supported.",
                 size);
        } else {
            auto key = pkt->getBE<uint16_t>();
            select(key);
        }
    } else if (addr == dataAddr) {
        // Writes to the firmware config data can only be done through the
        // DMA interface.
        warn("Write to firmware config data register not supported.");
    } else {
        panic("Unrecognized firmware config write [%#x-%#x).", addr,
              addr + size);
    }

    return 0;
}

} // namespace qemu
} // namespace gem5

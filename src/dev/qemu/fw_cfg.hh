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

#ifndef __DEV_QEMU_FW_CFG_HH__
#define __DEV_QEMU_FW_CFG_HH__

#include <cstdint>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

#include "base/loader/image_file_data.hh"
#include "base/types.hh"
#include "dev/io_device.hh"
#include "params/QemuFwCfg.hh"
#include "params/QemuFwCfgIo.hh"
#include "params/QemuFwCfgItem.hh"
#include "params/QemuFwCfgItemBytes.hh"
#include "params/QemuFwCfgItemFile.hh"
#include "params/QemuFwCfgItemString.hh"
#include "params/QemuFwCfgMmio.hh"

namespace gem5
{

namespace qemu
{

/*
 * Items which can be reported by the firmware config device.
 */

class FwCfgItem
{
  protected:
    uint16_t _index;
    const std::string _path;
    bool _archSpecific;

    FwCfgItem(const std::string &new_path, bool arch_specific,
            uint16_t new_index=0) :
        _index(new_index), _path(new_path), _archSpecific(arch_specific)
    {}

  public:
    uint16_t index() const { return _index; }
    void index(uint16_t new_index) { _index = new_index; }

    const std::string &path() const { return _path; }
    bool archSpecific() const { return _archSpecific; }

    virtual uint64_t length() const = 0;

    virtual void read(void *buf, uint64_t offset, uint32_t to_read) = 0;
};

// Read only items with precomputed data.
class FwCfgItemFixed : public FwCfgItem
{
  protected:
    virtual const void *bytes() const = 0;

  public:
    using FwCfgItem::FwCfgItem;

    void read(void *buf, uint64_t offset, uint32_t to_read) override;
};

// An item who's value comes from a file.
class FwCfgItemFile : public FwCfgItemFixed
{
  private:
    const gem5::loader::ImageFileData data;

  public:
    FwCfgItemFile(const std::string &new_path, bool arch_specific,
            const std::string path, uint16_t new_index=0) :
        FwCfgItemFixed(new_path, arch_specific, new_index), data(path)
    {}

    FwCfgItemFile(const QemuFwCfgItemFileParams &p) :
        FwCfgItemFile(p.path, p.arch_specific, p.file, p.index)
    {}

    const void *bytes() const override { return data.data(); }
    uint64_t length() const override { return data.len(); }
};

// An item who's value comes from a string.
class FwCfgItemString : public FwCfgItemFixed
{
  private:
    std::string str;

  public:
    FwCfgItemString(const std::string &new_path, bool arch_specific,
            const std::string _str, uint16_t new_index=0) :
        FwCfgItemFixed(new_path, arch_specific, new_index), str(_str)
    {}

    FwCfgItemString(const QemuFwCfgItemStringParams &p) :
        FwCfgItemString(p.path, p.arch_specific, p.string, p.index)
    {}

    const void *bytes() const override { return (void *)str.data(); }
    uint64_t
    length() const override
    {
        return sizeof(std::string::value_type) * str.length();
    }
};

// An item who's value comes from an array of bytes.
class FwCfgItemBytes : public FwCfgItemFixed
{
  private:
    std::vector<uint8_t> data;

  public:
    FwCfgItemBytes(const std::string &new_path, bool arch_specific,
            const std::vector<uint8_t> &_data, uint16_t new_index=0) :
        FwCfgItemFixed(new_path, arch_specific, new_index), data(_data)
    {}

    FwCfgItemBytes(const QemuFwCfgItemBytesParams &p) :
        FwCfgItemBytes(p.path, p.arch_specific, p.data, p.index)
    {}

    const void *bytes() const override { return (void *)data.data(); }
    uint64_t length() const override { return data.size(); }
};

/*
 * Base and template classes for creating SimObject wrappers for item types.
 */

class FwCfgItemFactoryBase : public SimObject
{
  public:
    PARAMS(QemuFwCfgItem);
    FwCfgItemFactoryBase(const Params &p) : SimObject(p) {}

    virtual FwCfgItem &item() = 0;
};

template <class ItemType>
class FwCfgItemFactory : public FwCfgItemFactoryBase
{
  private:
    ItemType _item;

  public:
    template <class PType, class = typename std::enable_if_t<
        std::is_base_of_v<SimObjectParams, PType>>>
    FwCfgItemFactory(const PType &p) : FwCfgItemFactoryBase(p), _item(p) {}

    FwCfgItem &item() override { return _item; }
};

/*
 * The actual firmware config device itself.
 */

// The base class.
class FwCfg : public PioDevice
{
  private:
    std::map<std::string, uint16_t> names;
    std::map<uint16_t, FwCfgItem *> numbers;

    uint32_t nextGenericIndex = 0x20;
    static inline const uint32_t MaxGenericIndex = 0x3fff;

    uint32_t nextArchIndex = 0x8020;
    static inline const uint32_t MaxArchIndex = 0xbfff;

    uint64_t offset = 0;
    FwCfgItem *current = nullptr;

    class Directory : public FwCfgItemFixed
    {
      private:
        std::vector<uint8_t> data;

      public:
        Directory();

        void update(const std::map<std::string, uint16_t> &names,
                const std::map<uint16_t, FwCfgItem *> &numbers);

        const void *bytes() const override { return (void *)data.data(); }
        uint64_t length() const override { return data.size(); }
    };

    FwCfgItemString signature;
    FwCfgItemString id;
    Directory directory;

    void addItem(FwCfgItem *item);

  protected:
    const AddrRangeList addrRanges;

    void select(uint16_t key);

  public:
    PARAMS(QemuFwCfg);
    FwCfg(const Params &p, const AddrRangeList &addr_ranges);

    AddrRangeList getAddrRanges() const override { return addrRanges; }

    void readItem(void *buf, uint32_t length);
};

// A version which uses IO ports.
class FwCfgIo : public FwCfg
{
  private:
    const Addr selectorAddr;
    const Addr dataAddr;

  public:
    PARAMS(QemuFwCfgIo);
    FwCfgIo(const Params &p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

// A version which uses memory mapped IO.
class FwCfgMmio : public FwCfg
{
  private:
    const Addr selectorAddr;
    const Addr dataAddr;
    const Addr dataSize;

  public:
    PARAMS(QemuFwCfgMmio);
    FwCfgMmio(const Params &p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace qemu
} // namespace gem5

#endif //__DEV_QEMU_FW_CFG_HH__

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

/** @file
 * Disk Image Definitions
 */

#include "dev/storage/disk_image.hh"

#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <string>

#include "base/callback.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/DiskImageRead.hh"
#include "debug/DiskImageWrite.hh"
#include "sim/byteswap.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

////////////////////////////////////////////////////////////////////////
//
// Raw Disk image
//
RawDiskImage::RawDiskImage(const Params &p)
    : DiskImage(p), disk_size(0)
{
    open(p.image_file, p.read_only);
}

RawDiskImage::~RawDiskImage()
{
    close();
}

void
RawDiskImage::notifyFork()
{
    if (initialized && !readonly)
        panic("Attempting to fork system with read-write raw disk image.");

    const Params &p = dynamic_cast<const Params &>(params());
    close();
    open(p.image_file, p.read_only);
}

void
RawDiskImage::open(const std::string &filename, bool rd_only)
{
    if (!filename.empty()) {
        initialized = true;
        readonly = rd_only;
        file = filename;

        std::ios::openmode mode = std::ios::in | std::ios::binary;
        if (!readonly)
            mode |= std::ios::out;
        stream.open(file.c_str(), mode);
        if (!stream.is_open())
            panic("Error opening %s", filename);
    }
}

void
RawDiskImage::close()
{
    stream.close();
}

std::streampos
RawDiskImage::size() const
{
    if (disk_size == 0) {
        if (!stream.is_open())
            panic("file not open!\n");
        stream.seekg(0, std::ios::end);
        disk_size = stream.tellg();
    }

    return disk_size / SectorSize;
}

std::streampos
RawDiskImage::read(uint8_t *data, std::streampos offset) const
{
    if (!initialized)
        panic("RawDiskImage not initialized");

    if (!stream.is_open())
        panic("file not open!\n");

    stream.seekg(offset * SectorSize, std::ios::beg);
    if (!stream.good())
        panic("Could not seek to location in file");

    std::streampos pos = stream.tellg();
    stream.read((char *)data, SectorSize);

    DPRINTF(DiskImageRead, "read: offset=%d\n", (uint64_t)offset);
    DDUMP(DiskImageRead, data, SectorSize);

    return stream.tellg() - pos;
}

std::streampos
RawDiskImage::write(const uint8_t *data, std::streampos offset)
{
    if (!initialized)
        panic("RawDiskImage not initialized");

    if (readonly)
        panic("Cannot write to a read only disk image");

    if (!stream.is_open())
        panic("file not open!\n");

    stream.seekp(offset * SectorSize, std::ios::beg);
    if (!stream.good())
        panic("Could not seek to location in file");

    DPRINTF(DiskImageWrite, "write: offset=%d\n", (uint64_t)offset);
    DDUMP(DiskImageWrite, data, SectorSize);

    std::streampos pos = stream.tellp();
    stream.write((const char *)data, SectorSize);
    return stream.tellp() - pos;
}

////////////////////////////////////////////////////////////////////////
//
// Copy on Write Disk image
//
const uint32_t CowDiskImage::VersionMajor = 1;
const uint32_t CowDiskImage::VersionMinor = 0;

CowDiskImage::CowDiskImage(const Params &p)
    : DiskImage(p), filename(p.image_file), child(p.child), table(NULL)
{
    if (filename.empty()) {
        initSectorTable(p.table_size);
    } else {
        if (!open(filename)) {
            if (p.read_only)
                fatal("could not open read-only file");
            initSectorTable(p.table_size);
        }

        if (!p.read_only)
            registerExitCallback([this]() { save(); });
    }
}

CowDiskImage::~CowDiskImage()
{
    SectorTable::iterator i = table->begin();
    SectorTable::iterator end = table->end();

    while (i != end) {
        delete (*i).second;
        ++i;
    }
}

void
CowDiskImage::notifyFork()
{
    if (!dynamic_cast<const Params &>(params()).read_only &&
        !filename.empty()) {
        inform("Disabling saving of COW image in forked child process.\n");
        filename = "";
    }
}

void
SafeRead(std::ifstream &stream, void *data, int count)
{
    stream.read((char *)data, count);
    if (!stream.is_open())
        panic("file not open");

    if (stream.eof())
        panic("premature end-of-file");

    if (stream.bad() || stream.fail())
        panic("error reading cowdisk image");
}

template<class T>
void
SafeRead(std::ifstream &stream, T &data)
{
    SafeRead(stream, &data, sizeof(data));
}

template<class T>
void
SafeReadSwap(std::ifstream &stream, T &data)
{
    SafeRead(stream, &data, sizeof(data));
    data = letoh(data); //is this the proper byte order conversion?
}

bool
CowDiskImage::open(const std::string &file)
{
    std::ifstream stream(file.c_str());
    if (!stream.is_open())
        return false;

    if (stream.fail() || stream.bad())
        panic("Error opening %s", file);

    uint64_t magic;
    SafeRead(stream, magic);

    if (memcmp(&magic, "COWDISK!", sizeof(magic)) != 0)
        panic("Could not open %s: Invalid magic", file);

    uint32_t major_version, minor_version;
    SafeReadSwap(stream, major_version);
    SafeReadSwap(stream, minor_version);

    if (major_version != VersionMajor && minor_version != VersionMinor)
        panic("Could not open %s: invalid version %d.%d != %d.%d",
              file, major_version, minor_version, VersionMajor, VersionMinor);

    uint64_t sector_count;
    SafeReadSwap(stream, sector_count);
    table = new SectorTable(sector_count);


    for (uint64_t i = 0; i < sector_count; i++) {
        uint64_t offset;
        SafeReadSwap(stream, offset);

        Sector *sector = new Sector;
        SafeRead(stream, sector, sizeof(Sector));

        assert(table->find(offset) == table->end());
        (*table)[offset] = sector;
    }

    stream.close();

    initialized = true;
    return true;
}

void
CowDiskImage::initSectorTable(int hash_size)
{
    table = new SectorTable(hash_size);

    initialized = true;
}

void
SafeWrite(std::ofstream &stream, const void *data, int count)
{
    stream.write((const char *)data, count);
    if (!stream.is_open())
        panic("file not open");

    if (stream.eof())
        panic("premature end-of-file");

    if (stream.bad() || stream.fail())
        panic("error reading cowdisk image");
}

template<class T>
void
SafeWrite(std::ofstream &stream, const T &data)
{
    SafeWrite(stream, &data, sizeof(data));
}

template<class T>
void
SafeWriteSwap(std::ofstream &stream, const T &data)
{
    T swappeddata = letoh(data); //is this the proper byte order conversion?
    SafeWrite(stream, &swappeddata, sizeof(data));
}
void
CowDiskImage::save() const
{
    // filename will be set to the empty string to disable saving of
    // the COW image in a forked child process. Save will still be
    // called because there is no easy way to unregister the exit
    // callback.
    if (!filename.empty())
        save(filename);}

void
CowDiskImage::save(const std::string &file) const
{
    if (!initialized)
        panic("RawDiskImage not initialized");

    std::ofstream stream(file.c_str());
    if (!stream.is_open() || stream.fail() || stream.bad())
        panic("Error opening %s", file);

    uint64_t magic;
    memcpy(&magic, "COWDISK!", sizeof(magic));
    SafeWrite(stream, magic);

    SafeWriteSwap(stream, (uint32_t)VersionMajor);
    SafeWriteSwap(stream, (uint32_t)VersionMinor);
    SafeWriteSwap(stream, (uint64_t)table->size());

    uint64_t size = table->size();
    SectorTable::iterator iter = table->begin();
    SectorTable::iterator end = table->end();

    for (uint64_t i = 0; i < size; i++) {
        if (iter == end)
            panic("Incorrect Table Size during save of COW disk image");

        SafeWriteSwap(stream, (uint64_t)(*iter).first);
        SafeWrite(stream, (*iter).second->data, sizeof(Sector));
        ++iter;
    }

    stream.close();
}

void
CowDiskImage::writeback()
{
    SectorTable::iterator i = table->begin();
    SectorTable::iterator end = table->end();

    while (i != end) {
        child->write((*i).second->data, (*i).first);
        ++i;
    }
}

std::streampos
CowDiskImage::size() const
{ return child->size(); }

std::streampos
CowDiskImage::read(uint8_t *data, std::streampos offset) const
{
    if (!initialized)
        panic("CowDiskImage not initialized");

    if (offset > size())
        panic("access out of bounds");

    SectorTable::const_iterator i = table->find(offset);
    if (i == table->end())
        return child->read(data, offset);
    else {
        memcpy(data, (*i).second->data, SectorSize);
        DPRINTF(DiskImageRead, "read: offset=%d\n", (uint64_t)offset);
        DDUMP(DiskImageRead, data, SectorSize);
        return SectorSize;
    }
}

std::streampos
CowDiskImage::write(const uint8_t *data, std::streampos offset)
{
    if (!initialized)
        panic("RawDiskImage not initialized");

    if (offset > size())
        panic("access out of bounds");

    SectorTable::iterator i = table->find(offset);
    if (i == table->end()) {
        Sector *sector = new Sector;
        memcpy(sector, data, SectorSize);
        table->insert(make_pair(offset, sector));
    } else {
        memcpy((*i).second->data, data, SectorSize);
    }

    DPRINTF(DiskImageWrite, "write: offset=%d\n", (uint64_t)offset);
    DDUMP(DiskImageWrite, data, SectorSize);

    return SectorSize;
}

void
CowDiskImage::serialize(CheckpointOut &cp) const
{
    std::string cowFilename = name() + ".cow";
    SERIALIZE_SCALAR(cowFilename);
    save(CheckpointIn::dir() + "/" + cowFilename);
}

void
CowDiskImage::unserialize(CheckpointIn &cp)
{
    std::string cowFilename;
    UNSERIALIZE_SCALAR(cowFilename);
    cowFilename = cp.getCptDir() + "/" + cowFilename;
    open(cowFilename);
}

} // namespace gem5

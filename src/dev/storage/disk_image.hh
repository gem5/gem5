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
 * Disk Image Interfaces
 */

#ifndef __DEV_STORAGE_DISK_IMAGE_HH__
#define __DEV_STORAGE_DISK_IMAGE_HH__

#include <fstream>
#include <unordered_map>

#include "params/CowDiskImage.hh"
#include "params/DiskImage.hh"
#include "params/RawDiskImage.hh"
#include "sim/sim_object.hh"

#define SectorSize (512)

namespace gem5
{

/**
 * Basic interface for accessing a disk image.
 */
class DiskImage : public SimObject
{
  protected:
    bool initialized;

  public:
    typedef DiskImageParams Params;

    DiskImage(const Params &p) : SimObject(p), initialized(false) {}

    virtual ~DiskImage() {}

    virtual std::streampos size() const = 0;

    virtual std::streampos read(uint8_t *data,
                                std::streampos offset) const = 0;
    virtual std::streampos write(const uint8_t *data,
                                 std::streampos offset) = 0;
};

/**
 * Specialization for accessing a raw disk image
 */
class RawDiskImage : public DiskImage
{
  protected:
    mutable std::fstream stream;
    std::string file;
    bool readonly;
    mutable std::streampos disk_size;

  public:
    typedef RawDiskImageParams Params;
    RawDiskImage(const Params &p);
    ~RawDiskImage();

    void notifyFork() override;

    void close();
    void open(const std::string &filename, bool rd_only = false);

    std::streampos size() const override;

    std::streampos read(uint8_t *data, std::streampos offset) const override;
    std::streampos write(const uint8_t *data, std::streampos offset) override;
};

/**
 * Specialization for accessing a copy-on-write disk image layer.
 * A copy-on-write(COW) layer must be stacked on top of another disk
 * image layer this layer can be another CowDiskImage, or a
 * RawDiskImage.
 *
 * This object is designed to provide a mechanism for persistant
 * changes to a main disk image, or to provide a place for temporary
 * changes to the image to take place that later may be thrown away.
 */
class CowDiskImage : public DiskImage
{
  public:
    static const uint32_t VersionMajor;
    static const uint32_t VersionMinor;

  protected:
    struct Sector
    {
        uint8_t data[SectorSize];
    };

    typedef std::unordered_map<uint64_t, Sector *> SectorTable;

  protected:
    std::string filename;
    DiskImage *child;
    SectorTable *table;

  public:
    typedef CowDiskImageParams Params;
    CowDiskImage(const Params &p);
    ~CowDiskImage();

    void notifyFork() override;

    void initSectorTable(int hash_size);
    bool open(const std::string &file);
    void save() const;
    void save(const std::string &file) const;
    void writeback();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    std::streampos size() const override;

    std::streampos read(uint8_t *data, std::streampos offset) const override;
    std::streampos write(const uint8_t *data, std::streampos offset) override;
};

void SafeRead(std::ifstream &stream, void *data, int count);

template <class T>
void SafeRead(std::ifstream &stream, T &data);

template <class T>
void SafeReadSwap(std::ifstream &stream, T &data);

void SafeWrite(std::ofstream &stream, const void *data, int count);

template <class T>
void SafeWrite(std::ofstream &stream, const T &data);

template <class T>
void SafeWriteSwap(std::ofstream &stream, const T &data);

} // namespace gem5

#endif // __DEV_STORAGE_DISK_IMAGE_HH__

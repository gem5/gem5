/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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
 *
 * Authors: Brandon Potter
 */

#ifndef __FD_ENTRY_HH__
#define __FD_ENTRY_HH__

#include <memory>
#include <ostream>
#include <string>

#include "sim/serialize.hh"

class EmulatedDriver;

/**
 * Holds a single file descriptor mapping and that mapping's data for
 * processes running in syscall emulation mode.
 */
class FDEntry : public Serializable
{
  public:
    FDEntry(bool close_on_exec = false)
        : _closeOnExec(close_on_exec)
    { }

    virtual std::shared_ptr<FDEntry> clone() const = 0;

    bool getCOE() const { return _closeOnExec; }

    void setCOE(bool close_on_exec) { _closeOnExec = close_on_exec; }

    virtual void serialize(CheckpointOut &cp) const;
    virtual void unserialize(CheckpointIn &cp);

  protected:
    bool _closeOnExec;
};

/**
 * Extends the base class to include a host-backed file descriptor field
 * that records the integer used to represent the file descriptor on the host
 * and the file's flags.
 */
class HBFDEntry: public FDEntry
{
  public:
    HBFDEntry(int flags, int sim_fd, bool close_on_exec = false)
        : FDEntry(close_on_exec), _flags(flags), _simFD(sim_fd)
    { }

    int getFlags() const { return _flags; }
    int getSimFD() const { return _simFD; }

    void setFlags(int flags) { _flags = flags; }
    void setSimFD(int sim_fd) { _simFD = sim_fd; }

  protected:
    int _flags;
    int _simFD;
};

/**
 * Holds file descriptors for host-backed files; host-backed files are
 * files which were opened on the physical machine where the simulation
 * is running (probably the thing on/under your desk). All regular files
 * are redirected to make it appear that the file descriptor assignment
 * starts at file descriptor '3' (not including stdin, stdout, stderr) and
 * then grows upward.
 */
class FileFDEntry: public HBFDEntry
{
  public:
    FileFDEntry(int sim_fd, int flags, std::string const& file_name,
                uint64_t file_offset, bool close_on_exec = false)
        : HBFDEntry(flags, sim_fd, close_on_exec),
          _fileName(file_name), _fileOffset(file_offset)
    { }

    FileFDEntry(FileFDEntry const& reg, bool close_on_exec = false)
        : HBFDEntry(reg._flags, reg._simFD, close_on_exec),
          _fileName(reg._fileName), _fileOffset(reg._fileOffset)
    { }

    std::shared_ptr<FDEntry>
    clone() const override
    {
        return std::make_shared<FileFDEntry>(*this);
    }

    std::string const& getFileName() const { return _fileName; }
    uint64_t getFileOffset() const { return _fileOffset; }

    void setFileName(std::string const& file_name) { _fileName = file_name; }
    void setFileOffset(uint64_t f_off) { _fileOffset = f_off; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    std::string _fileName;
    uint64_t _fileOffset;
};

/**
 * Holds the metadata needed to maintain the mappings for file descriptors
 * allocated with the pipe() system calls and its variants.
 */
class PipeFDEntry: public HBFDEntry
{
  public:
    enum EndType {
        read = 0,
        write = 1
    };

    PipeFDEntry(int sim_fd, int flags, EndType pipe_end_type,
                bool close_on_exec = false)
        : HBFDEntry(flags, sim_fd, close_on_exec), _pipeReadSource(-1),
          _pipeEndType(pipe_end_type)
    { }

    PipeFDEntry(PipeFDEntry const& pipe, bool close_on_exec = false)
        : HBFDEntry(pipe._flags, pipe._simFD, close_on_exec),
          _pipeReadSource(pipe._pipeReadSource),
          _pipeEndType(pipe._pipeEndType)
    { }

    std::shared_ptr<FDEntry>
    clone() const override
    {
        return std::make_shared<PipeFDEntry>(*this);
    }

    EndType getEndType() const { return _pipeEndType; }
    int getPipeReadSource() const { return _pipeReadSource; }

    void setPipeReadSource(int tgt_fd) { _pipeReadSource = tgt_fd; }
    void setEndType(EndType type) { _pipeEndType = type; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    int _pipeReadSource;
    EndType _pipeEndType;
};

/**
 * Holds file descriptors needed to simulate devices opened with pseudo
 * files (commonly with calls to ioctls).
 */
class DeviceFDEntry : public FDEntry
{
  public:
    DeviceFDEntry(EmulatedDriver *driver, std::string const& file_name,
                  bool close_on_exec = false)
        : FDEntry(close_on_exec), _driver(driver), _fileName(file_name)
    { }

    DeviceFDEntry(DeviceFDEntry const& dev, bool close_on_exec = false)
        : FDEntry(close_on_exec), _driver(dev._driver),
          _fileName(dev._fileName)
    { }

    std::shared_ptr<FDEntry>
    clone() const override
    {
        return std::make_shared<DeviceFDEntry>(*this);
    }

    EmulatedDriver *getDriver() const { return _driver; }
    std::string const& getFileName() const { return _fileName; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    EmulatedDriver *_driver;
    std::string _fileName;
};

class SocketFDEntry: public HBFDEntry
{
  public:
    SocketFDEntry(int sim_fd, int domain, int type, int protocol,
                  bool close_on_exec = false)
        : HBFDEntry(0, sim_fd, close_on_exec),
          _domain(domain), _type(type), _protocol(protocol)
    { }

    SocketFDEntry(SocketFDEntry const& reg, bool close_on_exec = false)
        : HBFDEntry(reg._flags, reg._simFD, close_on_exec),
          _domain(reg._domain), _type(reg._type), _protocol(reg._protocol)
    { }

    std::shared_ptr<FDEntry>
    clone() const override
    {
        return std::make_shared<SocketFDEntry>(*this);
    }

    int _domain;
    int _type;
    int _protocol;
};

#endif // __FD_ENTRY_HH__

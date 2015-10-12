/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __FD_ENTRY_HH__
#define __FD_ENTRY_HH__

#include <ostream>
#include <string>

#include "sim/emul_driver.hh"

/**
 * FDEntry is used to manage a single file descriptor mapping and metadata
 * for processes.
 * Note that the fields are all declared publicly since system calls have a
 * habit of only needing to access a single field at a time and accessor
 * methods seem like overkill.
 */
class FDEntry : public Serializable
{
  public:
    /**
     * Constructor contains default values
     * The file descriptor is free.
     */
    FDEntry()
        : fd(-1), mode(0), flags(0), isPipe(false), readPipeSource(0),
          fileOffset(0), filename(""), driver(NULL)
    { }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Check if the target file descriptor is in use.
     * @return value denoting if target file descriptor already used
     */
    bool isFree();

    /**
     * Fill in members for this file descriptor entry.
     * @param sim_fd host file descriptor
     * @param name filename string
     * @param flags current flags of the file descriptor
     * @param mode current mode of the file descriptor
     * @param pipe denotes whether the file descriptor belongs to a pipe
     */
    void set(int sim_fd, const std::string name, int flags, int mode,
             bool pipe);

    /** Reset members to their default values. */
    void reset();

    int fd;
    int mode;
    int flags;
    bool isPipe;
    int readPipeSource;
    uint64_t fileOffset;
    std::string filename;
    EmulatedDriver *driver;
};

#endif // __FD_ENTRY_HH__

/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 */

#include "sim/fd_array.hh"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <memory>
#include <string>

#include "base/logging.hh"
#include "base/output.hh"
#include "params/Process.hh"
#include "sim/fd_entry.hh"
#include "sim/process.hh"

namespace gem5
{

FDArray::FDArray(std::string const& input, std::string const& output,
                 std::string const& errout)
    :  _fdArray(), _input(input), _output(output), _errout(errout),
      _imap {{"",       -1},
             {"cin",    STDIN_FILENO},
             {"stdin",  STDIN_FILENO}},
      _oemap{{"",       -1},
             {"cout",   STDOUT_FILENO},
             {"stdout", STDOUT_FILENO},
             {"cerr",   STDERR_FILENO},
             {"stderr", STDERR_FILENO}}
{
    int sim_fd;
    std::map<std::string, int>::iterator it;

    /**
     * Search through the input options and setup the default fd if match is
     * found; otherwise, open an input file and seek to location.
     */
    if ((it = _imap.find(input)) != _imap.end())
        sim_fd = it->second;
    else
        sim_fd = openInputFile(input);

    auto ffd = std::make_shared<FileFDEntry>(sim_fd, O_RDONLY, input, false);
    _fdArray[STDIN_FILENO] = ffd;

    /**
     * Search through the output/error options and setup the default fd if
     * match is found; otherwise, open an output file and seek to location.
     */
    if ((it = _oemap.find(output)) != _oemap.end())
        sim_fd = it->second;
    else
        sim_fd = openOutputFile(output);

    ffd = std::make_shared<FileFDEntry>(sim_fd, O_WRONLY | O_CREAT | O_TRUNC,
                                        output, false);
    _fdArray[STDOUT_FILENO] = ffd;

    if (output == errout)
        ; /* Reuse the same file descriptor if these match. */
    else if ((it = _oemap.find(errout)) != _oemap.end())
        sim_fd = it->second;
    else
        sim_fd = openOutputFile(errout);

    ffd = std::make_shared<FileFDEntry>(sim_fd, O_WRONLY | O_CREAT | O_TRUNC,
                                        errout, false);
    _fdArray[STDERR_FILENO] = ffd;
}

void
FDArray::updateFileOffsets()
{
    for (auto& fdp : _fdArray) {
        /**
         * It only makes sense to check the offsets if the file descriptor
         * type is 'File' (which indicates that this file is backed by a
         * file on the host). If the type is File, then record the offset.
         */
        auto ffd = std::dynamic_pointer_cast<FileFDEntry>(fdp);

        if (!ffd)
            continue;

        /**
         * Use lseek with SEEK_CUR with offset 0 to figure out where the
         * offset currently resides and pass that back to our setter.
         */
        int sim_fd = ffd->getSimFD();
        ffd->setFileOffset(lseek(sim_fd, 0, SEEK_CUR));
    }
}

void
FDArray::restoreFileOffsets()
{
    /**
     * Use this lambda to highlight what we mean to do with the seek.
     * Notice that this either seeks correctly (sets the file location on the
     * host) or it fails with a fatal. The error is fatal because it's not
     * possible to guarantee that the simulation will proceed as it should
     * have in the same way that it would have proceeded sans checkpoints.
     */
    auto seek = [] (std::shared_ptr<FileFDEntry> ffd)
    {
        if (lseek(ffd->getSimFD(), ffd->getFileOffset(), SEEK_SET) < 0)
            fatal("Unable to seek to location in %s", ffd->getFileName());
    };

    std::map<std::string, int>::iterator it;

    /**
     * Search through the input options and set fd if match is found;
     * otherwise, open an input file and seek to location.
     * Check if user has specified a different input file, and if so, use it
     * instead of the file specified in the checkpoint. This also resets the
     * file offset from the checkpointed value
     */
    std::shared_ptr<FDEntry> stdin_fde = _fdArray[STDIN_FILENO];
    auto stdin_ffd = std::dynamic_pointer_cast<FileFDEntry>(stdin_fde);

    if (_input != stdin_ffd->getFileName()) {
        warn("Using new input file (%s) rather than checkpointed (%s)\n",
             _input, stdin_ffd->getFileName());
        stdin_ffd->setFileName(_input);
        stdin_ffd->setFileOffset(0);
    }

    if ((it = _imap.find(stdin_ffd->getFileName())) != _imap.end()) {
        stdin_ffd->setSimFD(it->second);
    } else {
        stdin_ffd->setSimFD(openInputFile(stdin_ffd->getFileName()));
        seek(stdin_ffd);
    }

    /**
     * Search through the output options and set fd if match is found;
     * otherwise, open an output file and seek to location.
     * Check if user has specified a different output file, and if so, use it
     * instead of the file specified in the checkpoint. This also resets the
     * file offset from the checkpointed value
     */
    std::shared_ptr<FDEntry> stdout_fde = _fdArray[STDOUT_FILENO];
    auto stdout_ffd = std::dynamic_pointer_cast<FileFDEntry>(stdout_fde);

    if (_output != stdout_ffd->getFileName()) {
        warn("Using new output file (%s) rather than checkpointed (%s)\n",
             _output, stdout_ffd->getFileName());
        stdout_ffd->setFileName(_output);
        stdout_ffd->setFileOffset(0);
    }

    if ((it = _oemap.find(stdout_ffd->getFileName())) != _oemap.end()) {
        stdout_ffd->setSimFD(it->second);
    } else {
        stdout_ffd->setSimFD(openOutputFile(stdout_ffd->getFileName()));
        seek(stdout_ffd);
    }

    /**
     * Search through the error options and set fd if match is found;
     * otherwise, open an error file and seek to location.
     * Check if user has specified a different error file, and if so, use it
     * instead of the file specified in the checkpoint. This also resets the
     * file offset from the checkpointed value
     */
    std::shared_ptr<FDEntry> stderr_fde = _fdArray[STDERR_FILENO];
    auto stderr_ffd = std::dynamic_pointer_cast<FileFDEntry>(stderr_fde);

    if (_errout != stderr_ffd->getFileName()) {
        warn("Using new error file (%s) rather than checkpointed (%s)\n",
             _errout, stderr_ffd->getFileName());
        stderr_ffd->setFileName(_errout);
        stderr_ffd->setFileOffset(0);
    }

    if (stdout_ffd->getFileName() == stderr_ffd->getFileName()) {
        /* Reuse the same sim_fd file descriptor if these match. */
        stderr_ffd->setSimFD(stdout_ffd->getSimFD());
    } else if ((it = _oemap.find(stderr_ffd->getFileName())) != _oemap.end()) {
        stderr_ffd->setSimFD(it->second);
    } else {
        stderr_ffd->setSimFD(openOutputFile(stderr_ffd->getFileName()));
        seek(stderr_ffd);
    }

    for (int tgt_fd = 3; tgt_fd < _fdArray.size(); tgt_fd++) {
        std::shared_ptr<FDEntry> fdp = _fdArray[tgt_fd];
        if (!fdp)
            continue;

        /* Need to reconnect pipe ends. */
        if (auto pfd = std::dynamic_pointer_cast<PipeFDEntry>(fdp)) {
            /**
             * Check which end of the pipe we are looking at; we only want
             * to setup the pipe once so we arbitrarily choose the read
             * end to be the end that sets up the pipe.
             */
            if (pfd->getEndType() == PipeFDEntry::EndType::write)
                continue;

            /* Setup the pipe or fatal out of the simulation. */
            int fd_pair[2];
            if (pipe(fd_pair) < 0)
                fatal("Unable to create new pipe");

            /**
             * Reconstruct the ends of the pipe by reassigning the pipe
             * that we created on the host. This one is the read end.
             */
            pfd->setSimFD(fd_pair[0]);

            /**
             * Grab the write end by referencing the read ends source and
             * using that tgt_fd to index the array.
             */
            int prs = pfd->getPipeReadSource();
            std::shared_ptr<FDEntry> write_fdp = _fdArray[prs];

            /* Now cast it and make sure that we are still sane. */
            auto write_pfd = std::dynamic_pointer_cast<PipeFDEntry>(write_fdp);

            /* Hook up the write end back to the right side of the pipe. */
            write_pfd->setSimFD(fd_pair[1]);
        }

        /* Need to reassign 'driver'. */
        if (auto dfd = std::dynamic_pointer_cast<DeviceFDEntry>(fdp)) {
            /**
             * Yeah, how does one retain the entire driver state from this
             * particular set of code? If you figure it out, add some code
             * here to rectify the issue.
             */
            fatal("Unable to restore checkpoints with emulated drivers");
        }

        /* Need to open files and seek. */
        if (auto ffd = std::dynamic_pointer_cast<FileFDEntry>(fdp)) {
            /**
             * Assume that this has the mode of an output file so there's no
             * need to worry about properly recording the mode. If you're
             * reading this and this happens to be your issue, at least be
             * happy that you've discovered the issue (and not mad at me).
             * Onward ho!
             */
            int sim_fd = openFile(ffd->getFileName(), ffd->getFlags(), 0664);
            ffd->setSimFD(sim_fd);
            seek(ffd);
        }
    }
}

int
FDArray::allocFD(std::shared_ptr<FDEntry> in)
{
    for (int i = 0; i < _fdArray.size(); i++) {
        std::shared_ptr<FDEntry> fdp = _fdArray[i];
        if (!fdp) {
            _fdArray[i] = in;
            return i;
        }
    }
    fatal("Out of target file descriptors");
}

int
FDArray::openFile(std::string const& filename, int flags, mode_t mode) const
{
    int sim_fd = open(filename.c_str(), flags, mode);
    if (sim_fd != -1)
        return sim_fd;
    fatal("Unable to open %s with mode %d", filename, mode);
}

int
FDArray::openInputFile(std::string const& filename) const
{
    return openFile(filename, O_RDONLY, 00);
}

int
FDArray::openOutputFile(std::string const& filename) const
{
    return openFile(simout.resolve(filename),
                    O_WRONLY | O_CREAT | O_TRUNC, 0664);
}

std::shared_ptr<FDEntry>
FDArray::getFDEntry(int tgt_fd)
{
    assert(0 <= tgt_fd && tgt_fd < _fdArray.size());
    return _fdArray[tgt_fd];
}

void
FDArray::setFDEntry(int tgt_fd, std::shared_ptr<FDEntry> fdep)
{
    assert(0 <= tgt_fd && tgt_fd < _fdArray.size());
    _fdArray[tgt_fd] = fdep;
}

int
FDArray::closeFDEntry(int tgt_fd)
{
    if (tgt_fd >= _fdArray.size() || tgt_fd < 0)
        return -EBADF;

    int sim_fd = -1;
    auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>(_fdArray[tgt_fd]);
    if (hbfdp)
        sim_fd = hbfdp->getSimFD();

    int status = 0;
    if (sim_fd > 2)
        status = close(sim_fd);

    if (status == 0)
        _fdArray[tgt_fd] = nullptr;

    return status;
}

void
FDArray::serialize(CheckpointOut &cp) const {
    ScopedCheckpointSection sec(cp, "fdarray");
    paramOut(cp, "size", _fdArray.size());
    for (int tgt_fd = 0; tgt_fd < _fdArray.size(); tgt_fd++) {
        auto fd = _fdArray[tgt_fd];
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", tgt_fd));
        if (!fd) {
            paramOut(cp, "class", FDEntry::FDClass::fd_null);
            continue;
        }
        paramOut(cp, "class", fd->getClass());
        fd->serialize(cp);
    }
}

void
FDArray::unserialize(CheckpointIn &cp, SimObject* process_ptr) {
    ScopedCheckpointSection sec(cp, "fdarray");
    uint64_t size;
    paramIn(cp, "size", size);
    assert(_fdArray.size() == size &&
            "FDArray sizes do not match at unserialize!");

    for (int tgt_fd = 0; tgt_fd < _fdArray.size(); tgt_fd++) {
        if (tgt_fd == STDIN_FILENO || tgt_fd == STDOUT_FILENO ||
                tgt_fd == STDERR_FILENO)
            continue;
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", tgt_fd));
        FDEntry::FDClass fd_class;
        paramIn(cp, "class", fd_class);
        std::shared_ptr<FDEntry> fdep;

        switch (fd_class) {
            case FDEntry::FDClass::fd_base:
                panic("Abstract fd entry was serialized");
                break;
            case FDEntry::FDClass::fd_hb:
                fdep = std::make_shared<HBFDEntry>(0, 0);
                break;
            case FDEntry::FDClass::fd_file:
                fdep = std::make_shared<FileFDEntry>(0, 0, "", 0, 00);
                break;
            case FDEntry::FDClass::fd_device:
                fdep = std::make_shared<DeviceFDEntry>(nullptr, "");
                break;
            case FDEntry::FDClass::fd_pipe:
                fdep = std::make_shared<PipeFDEntry>(
                        0, 0, PipeFDEntry::EndType::read);
                break;
            case FDEntry::FDClass::fd_socket:
                fdep = std::make_shared<SocketFDEntry>(0, 0, 0, 0);
                break;
            case FDEntry::FDClass::fd_null:
                continue;
            default:
                panic("Unrecognized fd class");
                break;
        }

        fdep->unserialize(cp);

        auto this_ffd = std::dynamic_pointer_cast<FileFDEntry>(fdep);
        if (!this_ffd)
            continue;
        setFDEntry(tgt_fd, fdep);

        mode_t mode = this_ffd->getFileMode();

        std::string const& path = this_ffd->getFileName();

        int flags = this_ffd->getFlags();

        // Re-open the file and assign a new sim_fd
        int sim_fd;
        if (process_ptr)
        {
            Process* ptr = static_cast<Process*>(process_ptr);
            std::string const& host_path =
                            ptr->checkPathRedirect(this_ffd->getFileName());
            sim_fd = openFile(host_path, flags, mode);
        }
        else
        {
            sim_fd = openFile(path, flags, mode);
        }

        this_ffd->setSimFD(sim_fd);

        // Restore the file offset to the proper value
        uint64_t file_offset = this_ffd->getFileOffset();
        lseek(sim_fd, file_offset, SEEK_SET);
    }
}

} // namespace gem5

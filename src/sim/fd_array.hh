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

#ifndef __FD_ARRAY_HH__
#define __FD_ARRAY_HH__

#include <array>
#include <map>
#include <memory>
#include <string>
#include "sim/fd_entry.hh"
#include "sim/serialize.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class FDArray : public Serializable
{
  public:
    /**
     * Initialize the file descriptor array and set the standard file
     * descriptors to defaults or values passed in with the process
     * params.
     * @param input Used to initialize the stdin file descriptor
     * @param output Used to initialize the stdout file descriptor
     * @param errout Used to initialize the stderr file descriptor
     */
    FDArray(std::string const& input, std::string const& output,
            std::string const& errout);

    /**
     * Figure out the file offsets for all currently open files and save them
     * the offsets during the calls to drain by the owning process.
     */
    void updateFileOffsets();

    /**
     * Restore all offsets for currently open files during the unserialize
     * phase for the owning process class.
     */
    void restoreFileOffsets();

    /**
     * Put the pointer specified by fdep into the _fdArray entry indexed
     * by tgt_fd.
     * @param tgt_fd Use target file descriptors to index the array.
     * @param fdep Incoming pointer used to set the entry pointed to by tgt_fd.
     */
    void setFDEntry(int tgt_fd, std::shared_ptr<FDEntry> fdep);

    /**
     * Treat this object like a normal array in using the subscript operator
     * to pull entries out of it.
     * @param tgt_fd Use target file descriptors to index the array.
     */
    std::shared_ptr<FDEntry>
    operator[](int tgt_fd)
    {
        return getFDEntry(tgt_fd);
    }

    /**
     * Return the size of the _fdArray field
     */
    int getSize() const { return _fdArray.size(); }

    /**
     * Step through the file descriptor array and find the first available
     * entry which is denoted as being free by being a 'nullptr'. That file
     * descriptor entry is the new target file descriptor entry that we
     * return as the return parameter.
     * @param fdp Allocated beforehand and passed into this method;
     * the fdp is meant to be a generic pointer capable of pointing to
     * different types of file descriptors. Must cast the pointer to the
     * correct type before dereferencing to access the needed fields.
     */
    int allocFD(std::shared_ptr<FDEntry> fdp);

    /**
     * Try to close the host file descriptor. If successful, set the
     * specified file descriptor entry object pointer to nullptr.
     * Used to "close" the target file descriptor.
     * @param tgt_fd Use target file descriptors to index the array.
     */
    int closeFDEntry(int tgt_fd);

    /*
     * Serialization methods for file descriptors
     */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp, SimObject* process_ptr );
    void unserialize(CheckpointIn &cp) override {
      unserialize(cp, nullptr);
    };


  private:
    /**
     * Help clarify our intention when opening files in the init and
     * restoration code. These are helper functions which are not meant to
     * be exposed to other objects or files.
     */
    int openFile(std::string const& file_name, int flags, mode_t mode) const;
    int openInputFile(std::string const& file_name) const;
    int openOutputFile(std::string const& file_name) const;

    /**
     * Return the file descriptor entry object associated with the index
     * provided. (The index is protected with bounds checking on the array
     * size without the use of the array's at operator.)
     * @param tgt_fd Use target file descriptors to index the array.
     */
    std::shared_ptr<FDEntry> getFDEntry(int tgt_fd);

    /**
     * Hold pointers to the file descriptor entries. The array size is
     * statically defined by the operating system.
     */
    static constexpr size_t _numFDs {1024};
    std::array<std::shared_ptr<FDEntry>, _numFDs> _fdArray;

    /**
     * Hold param strings passed from the Process class which indicate
     * the filename for each of the corresponding files or some keyword
     * indicating the use of standard file descriptors.
     */
    std::string _input;
    std::string _output;
    std::string _errout;

    /**
     * Hold strings which represent the default values which are checked
     * against to initialize the standard file descriptors. If the string
     * provided doesn't hit against these maps, then a file is opened on the
     * host instead of using the host's standard file descriptors.
     */
    std::map<std::string, int> _imap;
    std::map<std::string, int> _oemap;
};

} // namespace gem5

#endif // __FD_ARRAY_HH__

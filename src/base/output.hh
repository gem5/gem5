/*
 * Copyright (c) 2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2013 Andreas Sandberg
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#ifndef __BASE_OUTPUT_HH__
#define __BASE_OUTPUT_HH__

#include <ios>
#include <map>
#include <string>

#include "base/compiler.hh"

class OutputDirectory;

class OutputStream
{
  public:
    virtual ~OutputStream();

    /** Get the output underlying output stream */
    std::ostream *stream() const { return _stream; };

    /**
     * Can the file be recreated if the output directory is moved?
     *
     * @return true if the file will be created in the new location,
     * false otherwise.
     */
    virtual bool recreateable() const { return false; }

    /** Get the file name in the output directory */
    const std::string &name() const { return _name; }

  protected:
    friend class OutputDirectory;

    /** Wrap an existing stream */
    OutputStream(const std::string &name,
                 std::ostream *stream);

    /* Prevent copying */
    OutputStream(const OutputStream &f);

    /** Re-create the in a new location if recreateable. */
    virtual void relocate(const OutputDirectory &dir);

    /** Name in output directory */
    const std::string _name;

    /** Underlying output stream */
    std::ostream *const _stream;
};

template<class StreamType>
class OutputFile
    : public OutputStream
{
  public:
    typedef StreamType stream_type_t;

    virtual ~OutputFile();

    /**
     * Can the file be recreated if the output directory is moved?
     *
     * @return true if the file will be created in the new location,
     * false otherwise.
     */
    bool recreateable() const override { return _recreateable; }

  protected:
    friend class OutputDirectory;

    OutputFile(const OutputDirectory &dir,
               const std::string &name,
               std::ios_base::openmode mode,
               bool recreateable);

    /* Prevent copying */
    OutputFile(const OutputFile<StreamType> &f);

    /** Re-create the file in a new location if it is relocatable. */
    void relocate(const OutputDirectory &dir) override;

    /** File mode when opened */
    const std::ios_base::openmode _mode;

    /** Can the file be recreated in a new location? */
    const bool _recreateable;

    /** Pointer to the file stream */
    stream_type_t *const _fstream;
};

/** Interface for creating files in a gem5 output directory. */
class OutputDirectory
{
  private:
    /** File names and associated stream handles */
    typedef std::map<std::string, OutputStream *> file_map_t;

    /** Output subdirectories */
    typedef std::map<std::string, OutputDirectory *> dir_map_t;

    /** Open file streams within this directory */
    file_map_t files;

    /** Output sub-directories */
    dir_map_t dirs;

    /** Name of this directory */
    std::string dir;

    /** System-specific path separator character */
    static const char PATH_SEPARATOR = '/';

    static OutputStream stdout;
    static OutputStream stderr;

  protected:
    /**
     * Determines whether given file name corresponds to standard output
     * streams.
     *
     * @param name name of file to check
     * @return output stream for standard output or error stream if name
     *         corresponds to one or the other; NULL otherwise
     */
    static OutputStream *checkForStdio(const std::string &name);

  public:
    /** Constructor. */
    OutputDirectory();

    /** Constructor. */
    OutputDirectory(const std::string &name);

    /** Destructor. */
    ~OutputDirectory();

    /**
     * Returns relative file names prepended with name of this directory.
     * Returns absolute file names unaltered.
     *
     * @param name file name to prepend with directory name
     * @return file name prepended with base directory name or unaltered
     *          absolute file name
     */
    std::string resolve(const std::string &name) const;

    /**
     * Sets name of this directory.
     * @param dir name of this directory
     */
    void setDirectory(const std::string &dir);

    /**
     * Gets name of this directory.
     * @return name of this directory
     */
    const std::string &directory() const;

    /**
     * Creates a file in this directory (optionally compressed).
     *
     * Will open a file as a compressed stream if filename ends in .gz, unless
     * explicitly disabled.
     *
     * Relative output paths will result in the creation of a
     * recreateable (see OutputFile) output file in the current output
     * directory. Files created with an absolute path will not be
     * recreateable.
     *
     * @param name name of file to create (without this directory's name
     *          leading it)
     * @param binary true to create a binary file; false otherwise
     * @param no_gz true to disable opening the file as a gzip compressed output
     *     stream; false otherwise
     * @return OutputStream instance representing the created file
     */
    OutputStream *create(const std::string &name,
                         bool binary = false,
                         bool no_gz = false);

    /**
     * Open a file in this directory (optionally compressed).
     *
     * Will open a file as a compressed stream if filename ends in .gz, unless
     * explicitly disabled.
     *
     * @param filename file to open
     * @param mode attributes to open file with
     * @param recreateable Set to true if the file can be recreated in a new
     *     location.
     * @param no_gz true to disable opening the file as a gzip compressed output
     *     stream; false otherwise
     * @return OutputStream instance representing the opened file
     */
    OutputStream *open(const std::string &name,
                       std::ios_base::openmode mode,
                       bool recreateable = true,
                       bool no_gz = false);

    /**
     * Closes an output file and free the corresponding OutputFile.
     *
     * The output file must have been opened by the same
     * OutputDirectory instance as the one closing it, or sim will
     * fail.
     *
     * @param file OutputStream instance in this OutputDirectory.
     */
    void close(OutputStream *file);

    /**
     * Finds stream associated with an open file or stdout/stderr.
     *
     * @param name of file
     * @return stream to specified file or NULL if file does not exist
     */
    OutputStream *find(const std::string &name) const;

    OutputStream *findOrCreate(const std::string &name, bool binary = false);

    /**
     * Determines whether a file name corresponds to a file in this directory.
     * @param name name of file to evaluate
     * @return true iff file has been opened in this directory or exists on the
     *          file system within this directory
     */
    bool isFile(const std::string &name) const;

    /**
     * Test if a path is absolute.
     */
    static inline bool isAbsolute(const std::string &name) {
        return name[0] == PATH_SEPARATOR;
    }

    /**
     * Creates a subdirectory within this directory.
     * @param name name of subdirectory
     * @return the new subdirectory's name suffixed with a path separator
     */
    OutputDirectory *createSubdirectory(const std::string &name);

    /**
     * Removes a specified file or subdirectory.
     *
     * Will cause sim to fail for most errors.  However, it will only warn the
     * user if a directory could not be removed.  This is in place to
     * accommodate slow file systems where file deletions within a subdirectory
     * may not be recognized quickly enough thereby causing the subsequent call
     * to remove the directory to fail (seemingly unempty directory).
     *
     * @param name name of file or subdirectory to remove; name should not
     *              be prepended with the name of this directory object
     * @param recursive set to true to attempt to recursively delete a
     *                  subdirectory and its contents
     */
    void remove(const std::string &name, bool recursive=false);
};

extern OutputDirectory simout;

#endif // __BASE_OUTPUT_HH__

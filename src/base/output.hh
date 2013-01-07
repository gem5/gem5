/*
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
 *
 * Authors: Nathan Binkert
 *          Chris Emmons
 */

#ifndef __BASE_OUTPUT_HH__
#define __BASE_OUTPUT_HH__

#include <ios>
#include <map>
#include <string>

/** Interface for creating files in a gem5 output directory. */
class OutputDirectory
{
  private:
    /** File names and associated stream handles */
    typedef std::map<std::string, std::ostream *> map_t;

    /** Open file streams within this directory */
    map_t files;

    /** Name of this directory */
    std::string dir;

    /** System-specific path separator character */
    static const char PATH_SEPARATOR = '/';

  protected:
    /**
     * Determines whether given file name corresponds to standard output
     * streams.
     *
     * @param name name of file to check
     * @return output stream for standard output or error stream if name
     *         corresponds to one or the other; NULL otherwise
     */
    std::ostream *checkForStdio(const std::string &name) const;

  public:
    /** Constructor. */
    OutputDirectory();

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

    /** Opens a file (optionally compressed).
     *
     * Will open a file as a compressed stream if filename ends in .gz.
     *
     * @param filename file to open
     * @param mode attributes to open file with
     * @return stream pointer to opened file; will cause sim fail on error
     */
    std::ostream *openFile(const std::string &filename,
                        std::ios_base::openmode mode = std::ios::trunc);

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
     * Will open a file as a compressed stream if filename ends in .gz.
     *
     * @param name name of file to create (without this directory's name
     *          leading it)
     * @param binary true to create a binary file; false otherwise
     * @return stream to the opened file
     */
    std::ostream *create(const std::string &name, bool binary = false);

    /**
     * Closes a file stream.
     *
     * Stream must have been opened through this interface, or sim will fail.
     *
     * @param openStream open stream to close
     */
    void close(std::ostream *openStream);

    /**
     * Finds stream associated with a file.
     * @param name of file
     * @return stream to specified file or NULL if file does not exist
     */
    std::ostream *find(const std::string &name) const;

    /**
     * Returns true if stream is open and not standard output or error.
     * @param os output stream to evaluate
     * @return true if os is non-NULL and not cout or cerr
     */
    static bool isFile(const std::ostream *os);

    /**
     * Determines whether a file name corresponds to a file in this directory.
     * @param name name of file to evaluate
     * @return true iff file has been opened in this directory or exists on the
     *          file system within this directory
     */
    bool isFile(const std::string &name) const;

    /**
     * Returns true if stream is open and not standard output or error.
     * @param os output stream to evaluate
     * @return true if os is non-NULL and not cout or cerr
     */
    static inline bool isFile(const std::ostream &os) {
        return isFile(&os);
    }

    /**
     * Creates a subdirectory within this directory.
     * @param name name of subdirectory
     * @return the new subdirectory's name suffixed with a path separator
     */
    std::string createSubdirectory(const std::string &name) const;

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

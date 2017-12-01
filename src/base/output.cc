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
 *
 * Authors: Nathan Binkert
 *          Chris Emmons
 *          Andreas Sandberg
 *          Sascha Bischoff
 */

#include "base/output.hh"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <zfstream.h>

#include <cassert>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <fstream>

#include "base/logging.hh"

using namespace std;

OutputDirectory simout;


OutputStream::OutputStream(const std::string &name, std::ostream *stream)
    : _name(name), _stream(stream)
{
}

OutputStream::~OutputStream()
{
}

void
OutputStream::relocate(const OutputDirectory &dir)
{
}

template<class StreamType>
OutputFile<StreamType>::OutputFile(const OutputDirectory &dir,
                                   const std::string &name,
                                   std::ios_base::openmode mode,
                                   bool recreateable)
  : OutputStream(name, new stream_type_t()),
    _mode(mode), _recreateable(recreateable),
    _fstream(static_cast<stream_type_t *>(_stream))
{
    _fstream->open(dir.resolve(_name).c_str(), _mode);

    assert(_fstream->is_open());
}

template<class StreamType>
OutputFile<StreamType>::~OutputFile()
{
    if (_fstream->is_open())
        _fstream->close();
}

template<class StreamType>
void
OutputFile<StreamType>::relocate(const OutputDirectory &dir)
{
    if (_recreateable) {
        _fstream->close();
        _fstream->open(dir.resolve(_name).c_str(), _mode);
    }
}

OutputStream OutputDirectory::stdout("stdout", &cout);
OutputStream OutputDirectory::stderr("stderr", &cerr);

/**
 * @file This file manages creating / deleting output files for the simulator.
 */
OutputDirectory::OutputDirectory()
{}

OutputDirectory::OutputDirectory(const std::string &name)
{
    setDirectory(name);
}

OutputDirectory::~OutputDirectory()
{
    for (auto& f: files) {
        if (f.second)
            delete f.second;
    }
}

OutputStream *
OutputDirectory::checkForStdio(const string &name)
{
    if (name == "cerr" || name == "stderr")
        return &stderr;

    if (name == "cout" || name == "stdout")
        return &stdout;

    return NULL;
}

void
OutputDirectory::close(OutputStream *file)
{
    auto i = files.find(file->name());
    if (i == files.end())
        fatal("Attempted to close an unregistred file stream");

    files.erase(i);

    delete file;
}

void
OutputDirectory::setDirectory(const string &d)
{
    const string old_dir(dir);

    dir = d;

    // guarantee that directory ends with a path separator
    if (dir[dir.size() - 1] != PATH_SEPARATOR)
        dir += PATH_SEPARATOR;

    // Try to create the directory. If it already exists, that's ok;
    // otherwise, fail if we couldn't create it.
    if ((mkdir(dir.c_str(), 0755) != 0) && (errno != EEXIST))
        fatal("Failed to create new output subdirectory '%s'\n", dir);

    // Check if we need to recreate anything
    if (!old_dir.empty()) {
        // Recreate output files
        for (file_map_t::iterator i = files.begin(); i != files.end(); ++i) {
            i->second->relocate(*this);
        }

        // Relocate sub-directories
        for (dir_map_t::iterator i = dirs.begin(); i != dirs.end(); ++i) {
            i->second->setDirectory(dir + PATH_SEPARATOR + i->first);
        }
    }

}

const string &
OutputDirectory::directory() const
{
    if (dir.empty())
        panic("Output directory not set!");

    return dir;
}

string
OutputDirectory::resolve(const string &name) const
{
    return !isAbsolute(name) ? dir + name : name;
}

OutputStream *
OutputDirectory::create(const string &name, bool binary, bool no_gz)
{
    OutputStream *file = checkForStdio(name);
    if (file)
        return file;

    const ios_base::openmode mode(
        ios::trunc | (binary ? ios::binary : (ios::openmode)0));
    const bool recreateable(!isAbsolute(name));

    return open(name, mode, recreateable, no_gz);
}

OutputStream *
OutputDirectory::open(const std::string &name,
                      ios_base::openmode mode,
                      bool recreateable,
                      bool no_gz)
{
    OutputStream *os;

    if (!no_gz && name.find(".gz", name.length() - 3) < name.length()) {
        // Although we are creating an output stream, we still need to pass the
        // correct mode for gzofstream as this used directly to set the file
        // mode.
        mode |= std::ios::out;
        os = new OutputFile<gzofstream>(*this, name, mode, recreateable);
    } else {
        os = new OutputFile<ofstream>(*this, name, mode, recreateable);
    }

    files[name] = os;

    return os;
}

OutputStream *
OutputDirectory::find(const string &name) const
{
    OutputStream *file = checkForStdio(name);
    if (file)
        return file;

    auto i = files.find(name);
    if (i != files.end())
        return (*i).second;

    return NULL;
}


OutputStream *
OutputDirectory::findOrCreate(const std::string &name, bool binary)
{
    OutputStream *os(find(name));
    if (os)
        return os;
    else
        return create(name, binary);
}

bool
OutputDirectory::isFile(const string &name) const
{
    // definitely a file if in our data structure
    if (find(name) != NULL) return true;

    struct stat st_buf;
    int st = stat(name.c_str(), &st_buf);
    return (st == 0) && S_ISREG(st_buf.st_mode);
}

OutputDirectory *
OutputDirectory::createSubdirectory(const string &name)
{
    const string new_dir = resolve(name);
    if (new_dir.find(directory()) == string::npos)
        fatal("Attempting to create subdirectory not in m5 output dir\n");

    OutputDirectory *dir(new OutputDirectory(new_dir));
    dirs[name] = dir;

    return dir;
}

void
OutputDirectory::remove(const string &name, bool recursive)
{
    const string fname = resolve(name);

    if (fname.find(directory()) == string::npos)
        fatal("Attempting to remove file/dir not in output dir\n");

    if (isFile(fname)) {
        // close and release file if we have it open
        auto i = files.find(fname);
        if (i != files.end()) {
            delete i->second;
            files.erase(i);
        }

        if (::remove(fname.c_str()) != 0)
            fatal("Could not erase file '%s'\n", fname);
    } else {
        // assume 'name' is a directory
        if (recursive) {
            DIR *subdir = opendir(fname.c_str());

            // silently ignore removal request for non-existent directory
            if ((!subdir) && (errno == ENOENT))
                return;

            // fail on other errors
            if (!subdir) {
                perror("opendir");
                fatal("Error opening directory for recursive removal '%s'\n",
                      fname);
            }

            struct dirent *de = readdir(subdir);
            while (de != NULL) {
                // ignore files starting with a '.'; user must delete those
                //   manually if they really want to
                if (de->d_name[0] != '.')
                    remove(name + PATH_SEPARATOR + de->d_name, recursive);

                de = readdir(subdir);
            }

            closedir(subdir);
        }

        // try to force recognition that we deleted the files in the directory
        sync();

        if (::remove(fname.c_str()) != 0) {
            perror("Warning!  'remove' failed.  Could not erase directory.");
        }
    }
}

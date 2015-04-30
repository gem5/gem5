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

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>

#include <cassert>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <fstream>

#include <gzstream.hh>

#include "base/misc.hh"
#include "base/output.hh"

using namespace std;

OutputDirectory simout;

/**
 * @file This file manages creating / deleting output files for the simulator.
 */
OutputDirectory::OutputDirectory()
{}

OutputDirectory::~OutputDirectory()
{
    for (map_t::iterator i = files.begin(); i != files.end(); i++) {
        if (i->second)
            delete i->second;
    }
}

std::ostream *
OutputDirectory::checkForStdio(const string &name) const
{
    if (name == "cerr" || name == "stderr")
        return &cerr;

    if (name == "cout" || name == "stdout")
        return &cout;

    return NULL;
}

ostream *
OutputDirectory::openFile(const string &filename,
                          ios_base::openmode mode)
{
    if (filename.find(".gz", filename.length()-3) < filename.length()) {
        ogzstream *file = new ogzstream(filename.c_str(), mode);
        if (!file->is_open())
            fatal("Cannot open file %s", filename);
        assert(files.find(filename) == files.end());
        files[filename] = file;
        return file;
    } else {
        ofstream *file = new ofstream(filename.c_str(), mode);
        if (!file->is_open())
            fatal("Cannot open file %s", filename);
        assert(files.find(filename) == files.end());
        files[filename] = file;
        return file;
    }
}

void
OutputDirectory::close(ostream *openStream) {
    map_t::iterator i;
    for (i = files.begin(); i != files.end(); i++) {
        if (i->second != openStream)
            continue;

        ofstream *fs = dynamic_cast<ofstream*>(i->second);
        if (fs) {
            fs->close();
            delete i->second;
            break;
        } else {
            ogzstream *gfs = dynamic_cast<ogzstream*>(i->second);
            if (gfs) {
                gfs->close();
                delete i->second;
                break;
            }
        }
    }

    if (i == files.end())
        fatal("Attempted to close an unregistred file stream");

    files.erase(i);
}

void
OutputDirectory::setDirectory(const string &d)
{
    if (!dir.empty())
        panic("Output directory already set!\n");

    dir = d;

    // guarantee that directory ends with a path separator
    if (dir[dir.size() - 1] != PATH_SEPARATOR)
        dir += PATH_SEPARATOR;
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
    return (name[0] != PATH_SEPARATOR) ? dir + name : name;
}

ostream *
OutputDirectory::create(const string &name, bool binary)
{
    ostream *file = checkForStdio(name);
    if (file)
        return file;

    string filename = resolve(name);
    ios_base::openmode mode =
        ios::trunc | (binary ? ios::binary : (ios::openmode)0);
    file = openFile(filename, mode);

    return file;
}

ostream *
OutputDirectory::find(const string &name) const
{
    ostream *file = checkForStdio(name);
    if (file)
        return file;

    const string filename = resolve(name);
    map_t::const_iterator i = files.find(filename);
    if (i != files.end())
        return (*i).second;

    return NULL;
}

bool
OutputDirectory::isFile(const std::ostream *os)
{
    return os && os != &cerr && os != &cout;
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

string
OutputDirectory::createSubdirectory(const string &name) const
{
    const string new_dir = resolve(name);
    if (new_dir.find(directory()) == string::npos)
        fatal("Attempting to create subdirectory not in m5 output dir\n");

    // if it already exists, that's ok; otherwise, fail if we couldn't create
    if ((mkdir(new_dir.c_str(), 0755) != 0) && (errno != EEXIST))
        fatal("Failed to create new output subdirectory '%s'\n", new_dir);

    return name + PATH_SEPARATOR;
}

void
OutputDirectory::remove(const string &name, bool recursive)
{
    const string fname = resolve(name);

    if (fname.find(directory()) == string::npos)
        fatal("Attempting to remove file/dir not in output dir\n");

    if (isFile(fname)) {
        // close and release file if we have it open
        map_t::iterator itr = files.find(fname);
        if (itr != files.end()) {
            delete itr->second;
            files.erase(itr);
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

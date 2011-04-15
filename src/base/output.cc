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
 */

#include <sys/stat.h>
#include <sys/types.h>

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
 *
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
                          ios_base::openmode mode) const
{
    if (filename.find(".gz", filename.length()-3) < filename.length()) {
        ogzstream *file = new ogzstream(filename.c_str(), mode);

        if (!file->is_open())
            fatal("Cannot open file %s", filename);

        return file;
    } else {
        ofstream *file = new ofstream(filename.c_str(), mode);

        if (!file->is_open())
            fatal("Cannot open file %s", filename);

        return file;
    }
}

void
OutputDirectory::setDirectory(const string &d)
{
    if (!dir.empty())
        panic("Output directory already set!\n");

    dir = d;

    // guarantee that directory ends with a '/'
    if (dir[dir.size() - 1] != '/')
        dir += "/";
}

const string &
OutputDirectory::directory() const
{
    if (dir.empty())
        panic("Output directory not set!");

    return dir;
}

inline string
OutputDirectory::resolve(const string &name) const
{
    return (name[0] != '/') ? dir + name : name;
}

ostream *
OutputDirectory::create(const string &name, bool binary)
{
    ostream *file = checkForStdio(name);
    if (file)
        return file;

    string filename = resolve(name);
    ios_base::openmode mode =
        ios::trunc | binary ?  ios::binary : (ios::openmode)0;
    file = openFile(filename, mode);

    return file;
}

ostream *
OutputDirectory::find(const string &name)
{
    ostream *file = checkForStdio(name);
    if (file)
        return file;

    string filename = resolve(name);
    map_t::iterator i = files.find(filename);
    if (i != files.end())
        return (*i).second;

    file = openFile(filename);
    files[filename] = file;
    return file;
}

bool
OutputDirectory::isFile(const std::ostream *os)
{
    return os && os != &cerr && os != &cout;
}

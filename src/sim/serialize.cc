/*
 * Copyright (c) 2015, 2020 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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

#include "sim/serialize.hh"

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <cassert>
#include <cerrno>

#include "base/trace.hh"
#include "debug/Checkpoint.hh"

namespace gem5
{

int ckptMaxCount = 0;
int ckptCount = 0;
int ckptPrevCount = -1;
std::stack<std::string> Serializable::path;

/////////////////////////////

Serializable::Serializable()
{
}

Serializable::~Serializable()
{
}

void
Serializable::serializeSection(CheckpointOut &cp, const char *name) const
{
    Serializable::ScopedCheckpointSection sec(cp, name);
    serialize(cp);
}

void
Serializable::unserializeSection(CheckpointIn &cp, const char *name)
{
    Serializable::ScopedCheckpointSection sec(cp, name);
    unserialize(cp);
}

void
Serializable::generateCheckpointOut(const std::string &cpt_dir,
        std::ofstream &outstream)
{
    std::string dir = CheckpointIn::setDir(cpt_dir);
    if (mkdir(dir.c_str(), 0775) == -1 && errno != EEXIST)
            fatal("couldn't mkdir %s\n", dir);

    std::string cpt_file = dir + CheckpointIn::baseFilename;
    outstream = std::ofstream(cpt_file.c_str());
    time_t t = time(NULL);
    if (!outstream)
        fatal("Unable to open file %s for writing\n", cpt_file.c_str());
    outstream << "## checkpoint generated: " << ctime(&t);
}

Serializable::ScopedCheckpointSection::~ScopedCheckpointSection()
{
    assert(!path.empty());
    DPRINTF(Checkpoint, "Popping: %s\n", path.top());
    path.pop();
}

void
Serializable::ScopedCheckpointSection::pushName(const char *obj_name)
{
    if (path.empty()) {
        path.push(obj_name);
    } else {
        path.push(csprintf("%s.%s", path.top(), obj_name));
    }
    DPRINTF(Checkpoint, "ScopedCheckpointSection::pushName: %s\n", obj_name);
}

void
Serializable::ScopedCheckpointSection::nameOut(CheckpointOut &cp)
{
    DPRINTF(Checkpoint, "ScopedCheckpointSection::nameOut: %s\n",
            Serializable::currentSection());
    cp << "\n[" << Serializable::currentSection() << "]\n";
}

const std::string &
Serializable::currentSection()
{
    assert(!path.empty());

    return path.top();
}

const char *CheckpointIn::baseFilename = "m5.cpt";

std::string CheckpointIn::currentDirectory;

std::string
CheckpointIn::setDir(const std::string &name)
{
    // use csprintf to insert curTick() into directory name if it
    // appears to have a format placeholder in it.
    currentDirectory = (name.find("%") != std::string::npos) ?
        csprintf(name, curTick()) : name;
    if (currentDirectory[currentDirectory.size() - 1] != '/')
        currentDirectory += "/";
    return currentDirectory;
}

std::string
CheckpointIn::dir()
{
    return currentDirectory;
}

CheckpointIn::CheckpointIn(const std::string &cpt_dir)
    : db(), _cptDir(setDir(cpt_dir))
{
    std::string filename = getCptDir() + "/" + CheckpointIn::baseFilename;
    if (!db.load(filename)) {
        fatal("Can't load checkpoint file '%s'\n", filename);
    }
}

/**
 * @param section Here we mention the section we are looking for
 * (example: currentsection).
 * @param entry Mention the entry we are looking for (example: interrupt
 * time) in the section.
 *
 * @return Returns true if the entry exists in the named section
 * we are looking in.
 */
bool
CheckpointIn::entryExists(const std::string &section, const std::string &entry)
{
    return db.entryExists(section, entry);
}
/**
 * @param section Here we mention the section we are looking for
 * (example: currentsection).
 * @param entry Mention the entry we are looking for (example: Cache
 * line size etc) in the section.
 * @param value Give the value at the said entry.
 *
 * @return Returns true if the searched parameter exists with
 * the value, given the section .
 */
bool
CheckpointIn::find(const std::string &section, const std::string &entry,
        std::string &value)
{
    return db.find(section, entry, value);
}

bool
CheckpointIn::sectionExists(const std::string &section)
{
    return db.sectionExists(section);
}

void
CheckpointIn::visitSection(const std::string &section,
    IniFile::VisitSectionCallback cb)
{
    db.visitSection(section, cb);
}

} // namespace gem5

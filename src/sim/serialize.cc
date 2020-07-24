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

#include <cerrno>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "sim/eventq.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"

// For stat reset hack
#include "sim/stat_control.hh"

using namespace std;

int ckptMaxCount = 0;
int ckptCount = 0;
int ckptPrevCount = -1;
std::stack<std::string> Serializable::path;

/////////////////////////////

/// Container for serializing global variables (not associated with
/// any serialized object).
class Globals : public Serializable
{
  public:
    Globals()
        : unserializedCurTick(0) {}

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    Tick unserializedCurTick;
};

/// The one and only instance of the Globals class.
Globals globals;

/// The version tags for this build of the simulator, to be stored in the
/// Globals section during serialization and compared upon unserialization.
extern std::set<std::string> version_tags;

void
Globals::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "curTick", curTick());
    SERIALIZE_CONTAINER(version_tags);
}

void
Globals::unserialize(CheckpointIn &cp)
{
    paramIn(cp, "curTick", unserializedCurTick);

    const std::string &section(Serializable::currentSection());
    std::string str;
    if (!cp.find(section, "version_tags", str)) {
        warn("**********************************************************\n");
        warn("!!!! Checkpoint uses an old versioning scheme.        !!!!\n");
        warn("Run the checkpoint upgrader (util/cpt_upgrader.py) on your "
             "checkpoint\n");
        warn("**********************************************************\n");
        return;
    }

    std::set<std::string> cpt_tags;
    arrayParamIn(cp, "version_tags", cpt_tags); // UNSERIALIZE_CONTAINER

    bool err = false;
    for (const auto& t : version_tags) {
        if (cpt_tags.find(t) == cpt_tags.end()) {
            // checkpoint is missing tag that this binary has
            if (!err) {
                warn("*****************************************************\n");
                warn("!!!! Checkpoint is missing the following version tags:\n");
                err = true;
            }
            warn("  %s\n", t);
        }
    }
    if (err) {
        warn("You might experience some issues when restoring and should run "
             "the checkpoint upgrader (util/cpt_upgrader.py) on your "
             "checkpoint\n");
        warn("**********************************************************\n");
    }

    err = false;
    for (const auto& t : cpt_tags) {
        if (version_tags.find(t) == version_tags.end()) {
            // gem5 binary is missing tag that this checkpoint has
            if (!err) {
                warn("*****************************************************\n");
                warn("!!!! gem5 is missing the following version tags:\n");
                err = true;
            }
            warn("  %s\n", t);
        }
    }
    if (err) {
        warn("Running a checkpoint with incompatible version tags is not "
             "supported. While it might work, you may experience incorrect "
             "behavior or crashes.\n");
        warn("**********************************************************\n");
     }
}

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
Serializable::serializeAll(const string &cpt_dir)
{
    string dir = CheckpointIn::setDir(cpt_dir);
    if (mkdir(dir.c_str(), 0775) == -1 && errno != EEXIST)
            fatal("couldn't mkdir %s\n", dir);

    string cpt_file = dir + CheckpointIn::baseFilename;
    ofstream outstream(cpt_file.c_str());
    time_t t = time(NULL);
    if (!outstream.is_open())
        fatal("Unable to open file %s for writing\n", cpt_file.c_str());
    outstream << "## checkpoint generated: " << ctime(&t);

    globals.serializeSection(outstream, "Globals");

    SimObject::serializeAll(outstream);
}

void
Serializable::unserializeGlobals(CheckpointIn &cp)
{
    globals.unserializeSection(cp, "Globals");

    for (uint32_t i = 0; i < numMainEventQueues; ++i)
        mainEventQueue[i]->setCurTick(globals.unserializedCurTick);
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

string CheckpointIn::currentDirectory;

string
CheckpointIn::setDir(const string &name)
{
    // use csprintf to insert curTick() into directory name if it
    // appears to have a format placeholder in it.
    currentDirectory = (name.find("%") != string::npos) ?
        csprintf(name, curTick()) : name;
    if (currentDirectory[currentDirectory.size() - 1] != '/')
        currentDirectory += "/";
    return currentDirectory;
}

string
CheckpointIn::dir()
{
    return currentDirectory;
}

CheckpointIn::CheckpointIn(const string &cpt_dir, SimObjectResolver &resolver)
    : db(new IniFile), objNameResolver(resolver), _cptDir(setDir(cpt_dir))
{
    string filename = getCptDir() + "/" + CheckpointIn::baseFilename;
    if (!db->load(filename)) {
        fatal("Can't load checkpoint file '%s'\n", filename);
    }
}

CheckpointIn::~CheckpointIn()
{
    delete db;
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
CheckpointIn::entryExists(const string &section, const string &entry)
{
    return db->entryExists(section, entry);
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
CheckpointIn::find(const string &section, const string &entry, string &value)
{
    return db->find(section, entry, value);
}
/**
 * @param section Here we mention the section we are looking for
 * (example: currentsection).
 * @param entry Mention the SimObject we are looking for (example:
 * interruput time) in the section.
 * @param value Give the value at the said entry.
 *
 * @return Returns true if a SimObject exists in the section.
 *
 */
bool
CheckpointIn::findObj(const string &section, const string &entry,
                    SimObject *&value)
{
    string path;

    if (!db->find(section, entry, path))
        return false;

    value = objNameResolver.resolveSimObject(path);
    return true;
}

bool
CheckpointIn::sectionExists(const string &section)
{
    return db->sectionExists(section);
}

void
objParamIn(CheckpointIn &cp, const string &name, SimObject * &param)
{
    const string &section(Serializable::currentSection());
    if (!cp.findObj(section, name, param)) {
        fatal("Can't unserialize '%s:%s'\n", section, name);
    }
}

void
debug_serialize(const string &cpt_dir)
{
    Serializable::serializeAll(cpt_dir);
}

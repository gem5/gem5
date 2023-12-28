/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

#include "base/inifile.hh"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "base/str.hh"

namespace gem5
{

IniFile::IniFile()
{}

bool
IniFile::load(const std::string &file)
{
    std::ifstream f(file.c_str());

    if (!f.is_open())
        return false;

    return load(f);
}


const std::string &
IniFile::Entry::getValue() const
{
    referenced = true;
    return value;
}


void
IniFile::Section::addEntry(const std::string &entryName,
                           const std::string &value,
                           bool append)
{
    EntryTable::iterator ei = table.find(entryName);

    if (ei == table.end()) {
        // new entry
        table.emplace(entryName, value);
    }
    else if (append) {
        // append new reult to old entry
        ei->second.appendValue(value);
    }
    else {
        // override old entry
        ei->second.setValue(value);
    }
}


bool
IniFile::Section::add(const std::string &assignment)
{
    std::string::size_type offset = assignment.find('=');
    if (offset == std::string::npos) {
        // no '=' found
        std::cerr << "Can't parse .ini line " << assignment << std::endl;
        return false;
    }

    // if "+=" rather than just "=" then append value
    bool append = (assignment[offset-1] == '+');

    std::string entryName = assignment.substr(0, append ? offset-1 : offset);
    std::string value = assignment.substr(offset + 1);

    eat_white(entryName);
    eat_white(value);

    addEntry(entryName, value, append);
    return true;
}


IniFile::Entry *
IniFile::Section::findEntry(const std::string &entryName)
{
    return const_cast<IniFile::Entry *>(
        std::as_const(*this).findEntry(entryName));
}

const IniFile::Entry *
IniFile::Section::findEntry(const std::string &entryName) const
{
    referenced = true;

    auto ei = table.find(entryName);

    return (ei == table.end()) ? nullptr : &ei->second;
}


IniFile::Section *
IniFile::addSection(const std::string &sectionName)
{
    return &table[sectionName];
}

IniFile::Section *
IniFile::findSection(const std::string &sectionName)
{
    return const_cast<IniFile::Section*>(
        std::as_const(*this).findSection(sectionName));
}

const IniFile::Section *
IniFile::findSection(const std::string &sectionName) const
{
    auto i = table.find(sectionName);

    return (i == table.end()) ? nullptr : &i->second;
}


// Take string of the form "<section>:<parameter>=<value>" and add to
// database.  Return true if successful, false if parse error.
bool
IniFile::add(const std::string &str)
{
    // find ':'
    std::string::size_type offset = str.find(':');
    if (offset == std::string::npos)  // no ':' found
        return false;

    std::string sectionName = str.substr(0, offset);
    std::string rest = str.substr(offset + 1);

    eat_white(sectionName);
    Section *s = addSection(sectionName);

    return s->add(rest);
}

bool
IniFile::load(std::istream &f)
{
    Section *section = NULL;

    while (!f.eof()) {
        f >> std::ws; // Eat whitespace
        if (f.eof()) {
            break;
        }

        std::string line;
        getline(f, line);
        if (line.size() == 0)
            continue;

        eat_end_white(line);
        int last = line.size() - 1;

        if (line[0] == '[' && line[last] == ']') {
            std::string sectionName = line.substr(1, last - 1);
            eat_white(sectionName);
            section = addSection(sectionName);
            continue;
        }

        if (section == NULL)
            continue;

        if (!section->add(line))
            return false;
    }

    return true;
}

bool
IniFile::find(const std::string &sectionName, const std::string &entryName,
              std::string &value) const
{
    auto* section = findSection(sectionName);
    if (section == NULL)
        return false;

    auto* entry = section->findEntry(entryName);
    if (entry == NULL)
        return false;

    value = entry->getValue();

    return true;
}

bool
IniFile::entryExists(const std::string &sectionName,
        const std::string &entryName) const
{
    auto* section = findSection(sectionName);

    if (!section)
        return false;
    else
        return section->findEntry(entryName);
}

bool
IniFile::sectionExists(const std::string &sectionName) const
{
    return findSection(sectionName) != NULL;
}


bool
IniFile::Section::printUnreferenced(const std::string &sectionName) const
{
    bool unref = false;
    bool search_unref_entries = false;
    std::vector<std::string> unref_ok_entries;

    auto* entry = findEntry("unref_entries_ok");
    if (entry != NULL) {
        tokenize(unref_ok_entries, entry->getValue(), ' ');
        if (unref_ok_entries.size()) {
            search_unref_entries = true;
        }
    }

    for (auto& ei: table) {
        const std::string &entryName = ei.first;
        entry = &ei.second;

        if (entryName == "unref_section_ok" ||
            entryName == "unref_entries_ok")
        {
            continue;
        }

        if (!entry->isReferenced()) {
            if (search_unref_entries &&
                (std::find(unref_ok_entries.begin(), unref_ok_entries.end(),
                           entryName) != unref_ok_entries.end()))
            {
                continue;
            }

            std::cerr << "Parameter " << sectionName << ":" << entryName
                      << " not referenced." << std::endl;
            unref = true;
        }
    }

    return unref;
}


void
IniFile::getSectionNames(std::vector<std::string> &list) const
{
    for (auto& entry: table) {
        auto& sectionName = entry.first;
        list.push_back(sectionName);
    }
}

bool
IniFile::printUnreferenced() const
{
    bool unref = false;

    for (auto& entry: table) {
        auto& [sectionName, section] = entry;

        if (!section.isReferenced()) {
            if (section.findEntry("unref_section_ok") == NULL) {
                std::cerr << "Section " << sectionName << " not referenced."
                          << std::endl;
                unref = true;
            }
        }
        else {
            if (section.printUnreferenced(sectionName)) {
                unref = true;
            }
        }
    }

    return unref;
}


void
IniFile::Section::dump(const std::string &sectionName) const
{
    for (auto& ei: table) {
        std::cout << sectionName << ": " << ei.first << " => "
                  << ei.second.getValue() << "\n";
    }
}

void
IniFile::dump()
{
    for (SectionTable::iterator i = table.begin();
         i != table.end(); ++i) {
        i->second.dump(i->first);
    }
}

IniFile::Section::EntryTable::const_iterator
IniFile::Section::begin() const
{
    return table.begin();
}

IniFile::Section::EntryTable::const_iterator
IniFile::Section::end() const
{
    return table.end();
}

void
IniFile::visitSection(const std::string &sectionName,
    IniFile::VisitSectionCallback cb)
{
    const auto& section = table.at(sectionName);
    for (const auto& pair : section) {
        cb(pair.first, pair.second.getValue());
    }
}

} // namespace gem5

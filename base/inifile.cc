/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#define USE_CPP
// #define CPP_PIPE


#ifdef USE_CPP
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#endif

#include <fstream>
#include <iostream>
#if __GNUC__ >= 3
#include <ext/stdio_filebuf.h>
#endif

#include <vector>
#include <string>

#include "base/inifile.hh"
#include "base/str.hh"

using namespace std;

IniFile::IniFile()
{}

IniFile::~IniFile()
{
    ConfigTable::iterator i = table.begin();
    ConfigTable::iterator end = table.end();

    while (i != end) {
        delete (*i).second;
        ++i;
    }
}


#ifdef USE_CPP
bool
IniFile::loadCPP(const string &file, vector<char *> &cppArgs)
{
    int fd[2];

#ifdef CPP_PIPE
    if (pipe(fd) == -1)
        return false;
#else
    char tempfile[] = "/tmp/configXXXXXX";
    fd[0] = fd[1] = mkstemp(tempfile);
#endif

    int pid = fork();

    if (pid == -1)
        return 1;

    if (pid == 0) {
        char filename[FILENAME_MAX];
        string::size_type i = file.copy(filename, sizeof(filename) - 1);
        filename[i] = '\0';

        int arg_count = cppArgs.size();

        char **args = new char *[arg_count + 20];

        int nextArg = 0;
        args[nextArg++] = "g++";
        args[nextArg++] = "-E";
        args[nextArg++] = "-P";
        args[nextArg++] = "-nostdinc";
        args[nextArg++] = "-nostdinc++";
        args[nextArg++] = "-x";
        args[nextArg++] = "c++";
        args[nextArg++] = "-undef";

        for (int i = 0; i < arg_count; i++)
            args[nextArg++] = cppArgs[i];

        args[nextArg++] = filename;
        args[nextArg++] = NULL;

        close(STDOUT_FILENO);
        if (dup2(fd[1], STDOUT_FILENO) == -1)
            return 1;

        execvp("g++", args);

        exit(1);
    }

    int retval;
    waitpid(pid, &retval, 0);

    // check for normal completion of CPP
    if (!WIFEXITED(retval) || WEXITSTATUS(retval) != 0)
        return false;

#ifdef CPP_PIPE
    close(fd[1]);
#else
    lseek(fd[0], 0, SEEK_SET);
#endif

    bool status = false;

#if __GNUC__ >= 3
    using namespace __gnu_cxx;
    stdio_filebuf<char> fbuf(fd[0], ios_base::in, true,
        static_cast<stdio_filebuf<char>::int_type>(BUFSIZ));

    if (fbuf.is_open()) {
        istream f(&fbuf);
        status = load(f);
    }

#else
    ifstream f(fd[0]);
    if (f.is_open())
        status = load(f);
#endif

#ifndef CPP_PIPE
    unlink(tempfile);
#endif

    return status;
}
#endif

bool
IniFile::load(const string &file)
{
    ifstream f(file.c_str());

    if (!f.is_open())
        return false;

    return load(f);
}


const string &
IniFile::Entry::getValue() const
{
    referenced = true;
    return value;
}


void
IniFile::Section::addEntry(const std::string &entryName,
                           const std::string &value)
{
    EntryTable::iterator ei = table.find(entryName);

    if (ei == table.end()) {
        // new entry
        table[entryName] = new Entry(value);
    }
    else {
        // override old entry
        ei->second->setValue(value);
    }
}


IniFile::Entry *
IniFile::Section::findEntry(const std::string &entryName) const
{
    referenced = true;

    EntryTable::const_iterator ei = table.find(entryName);

    return (ei == table.end()) ? NULL : ei->second;
}


IniFile::Section *
IniFile::addSection(const string &sectionName)
{
    ConfigTable::iterator ci = table.find(sectionName);

    if (ci != table.end()) {
        return ci->second;
    }
    else {
        // new entry
        Section *sec = new Section();
        table[sectionName] = sec;
        return sec;
    }
}


IniFile::Section *
IniFile::findSection(const string &sectionName) const
{
    ConfigTable::const_iterator ci = table.find(sectionName);

    return (ci == table.end()) ? NULL : ci->second;
}


// Take string of the form "<section>:<parameter>=<value>" and add to
// database.  Return true if successful, false if parse error.
bool
IniFile::add(const string &str)
{
    // find ':'
    string::size_type offset = str.find(':');
    if (offset == string::npos)  // no ':' found
        return false;

    string sectionName = str.substr(0, offset);
    string rest = str.substr(offset + 1);

    offset = rest.find('=');
    if (offset == string::npos)  // no '='found
        return false;

    string entryName = rest.substr(0, offset);
    string value = rest.substr(offset + 1);

    eat_white(sectionName);
    eat_white(entryName);
    eat_white(value);

    Section *s = addSection(sectionName);
    s->addEntry(entryName, value);

    return true;
}

bool
IniFile::load(istream &f)
{
    Section *section = NULL;

    while (!f.eof()) {
        f >> ws; // Eat whitespace
        if (f.eof()) {
            break;
        }

        string line;
        getline(f, line);
        if (line.size() == 0)
            continue;

        eat_end_white(line);
        int last = line.size() - 1;

        if (line[0] == '[' && line[last] == ']') {
            string sectionName = line.substr(1, last - 1);
            eat_white(sectionName);
            section = addSection(sectionName);
            continue;
        }

        if (section == NULL)
            continue;

        string::size_type offset = line.find('=');
        string entryName = line.substr(0, offset);
        string value = line.substr(offset + 1);

        eat_white(entryName);
        eat_white(value);

        section->addEntry(entryName, value);
    }

    return true;
}

bool
IniFile::find(const string &sectionName, const string &entryName,
              string &value) const
{
    Section *section = findSection(sectionName);
    if (section == NULL)
        return false;

    Entry *entry = section->findEntry(entryName);
    if (entry == NULL)
        return false;

    value = entry->getValue();

    return true;
}

bool
IniFile::findDefault(const string &_section, const string &entry,
                     string &value) const
{
    string section = _section;
    while (!find(section, entry, value)) {
        if (!find(section, "default", section))
            return false;
    }

    return true;
}


bool
IniFile::Section::printUnreferenced(const string &sectionName)
{
    bool unref = false;
    bool search_unref_entries = false;
    vector<string> unref_ok_entries;

    Entry *entry = findEntry("unref_entries_ok");
    if (entry != NULL) {
        tokenize(unref_ok_entries, entry->getValue(), ' ');
        if (unref_ok_entries.size()) {
            search_unref_entries = true;
        }
    }

    for (EntryTable::iterator ei = table.begin();
         ei != table.end(); ++ei) {
        const string &entryName = ei->first;
        Entry *entry = ei->second;

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

            cerr << "Parameter " << sectionName << ":" << entryName
                 << " not referenced." << endl;
            unref = true;
        }
    }

    return unref;
}


bool
IniFile::printUnreferenced()
{
    bool unref = false;

    for (ConfigTable::iterator ci = table.begin();
         ci != table.end(); ++ci) {
        const string &sectionName = ci->first;
        Section *section = ci->second;

        if (!section->isReferenced()) {
            if (section->findEntry("unref_section_ok") == NULL) {
                cerr << "Section " << sectionName << " not referenced."
                     << endl;
                unref = true;
            }
        }
        else {
#if 0
            if (section->findEntry("unref_entries_ok") == NULL) {
                bool unrefEntries = section->printUnreferenced(sectionName);
                unref = unref || unrefEntries;
            }
#else
            if (section->printUnreferenced(sectionName)) {
                unref = true;
            }
#endif
        }
    }

    return unref;
}


void
IniFile::Section::dump(const string &sectionName)
{
    for (EntryTable::iterator ei = table.begin();
         ei != table.end(); ++ei) {
        cout << sectionName << ": " << (*ei).first << " => "
             << (*ei).second << "\n";
    }
}

void
IniFile::dump()
{
    for (ConfigTable::iterator ci = table.begin();
         ci != table.end(); ++ci) {
        ci->second->dump(ci->first);
    }
}

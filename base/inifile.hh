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

#ifndef __INIFILE_HH__
#define __INIFILE_HH__

#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "base/hashmap.hh"

class IniFile
{
  protected:
    class Entry
    {
        std::string	value;
        mutable bool	referenced;

      public:
        Entry(const std::string &v)
            : value(v), referenced(false)
        {
        }

        bool isReferenced() { return referenced; }

        const std::string &getValue() const;

        void setValue(const std::string &v) { value = v; }
    };

    class Section
    {
        typedef m5::hash_map<std::string, Entry *> EntryTable;

        EntryTable	table;
        mutable bool	referenced;

      public:
        Section()
            : table(), referenced(false)
        {
        }

        bool isReferenced() { return referenced; }

        void addEntry(const std::string &entryName, const std::string &value);
        Entry *findEntry(const std::string &entryName) const;

        bool printUnreferenced(const std::string &sectionName);
        void dump(const std::string &sectionName);
    };

    typedef m5::hash_map<std::string, Section *> ConfigTable;

  protected:
    ConfigTable table;

    Section *addSection(const std::string &sectionName);
    Section *findSection(const std::string &sectionName) const;

    bool load(std::istream &f);

  public:
    IniFile();
    ~IniFile();

    bool loadCPP(const std::string &file, std::vector<char *> &cppFlags);
    bool load(const std::string &file);

    bool add(const std::string &s);

    bool find(const std::string &section, const std::string &entry,
              std::string &value) const;
    bool findDefault(const std::string &section, const std::string &entry,
                     std::string &value) const;

    bool printUnreferenced();

    void dump();
};

#endif // __INIFILE_HH__

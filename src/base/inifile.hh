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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __INIFILE_HH__
#define __INIFILE_HH__

#include <fstream>
#include <list>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file
 * Declaration of IniFile object.
 * @todo Change comments to match documentation style.
 */

///
/// This class represents the contents of a ".ini" file.
///
/// It's basically a two level lookup table: a set of named sections,
/// where each section is a set of key/value pairs.  Section names,
/// keys, and values are all uninterpreted strings.
///
class IniFile
{
  protected:

    ///
    /// A single key/value pair.
    ///
    class Entry
    {
        std::string     value;          ///< The entry value.
        mutable bool    referenced;     ///< Has this entry been used?

      public:
        /// Constructor.
        Entry(const std::string &v)
            : value(v), referenced(false)
        {
        }

        /// Has this entry been used?
        bool isReferenced() { return referenced; }

        /// Fetch the value.
        const std::string &getValue() const;

        /// Set the value.
        void setValue(const std::string &v) { value = v; }

        /// Append the given string to the value.  A space is inserted
        /// between the existing value and the new value.  Since this
        /// operation is typically used with values that are
        /// space-separated lists of tokens, this keeps the tokens
        /// separate.
        void appendValue(const std::string &v) { value += " "; value += v; }
    };

    ///
    /// A section.
    ///
    class Section
    {
        /// EntryTable type.  Map of strings to Entry object pointers.
        typedef std::unordered_map<std::string, Entry *> EntryTable;

        EntryTable      table;          ///< Table of entries.
        mutable bool    referenced;     ///< Has this section been used?

      public:
        /// Constructor.
        Section()
            : table(), referenced(false)
        {
        }

        /// Has this section been used?
        bool isReferenced() { return referenced; }

        /// Add an entry to the table.  If an entry with the same name
        /// already exists, the 'append' parameter is checked If true,
        /// the new value will be appended to the existing entry.  If
        /// false, the new value will replace the existing entry.
        void addEntry(const std::string &entryName, const std::string &value,
                      bool append);

        /// Add an entry to the table given a string assigment.
        /// Assignment should be of the form "param=value" or
        /// "param+=value" (for append).  This funciton parses the
        /// assignment statment and calls addEntry().
        /// @retval True for success, false if parse error.
        bool add(const std::string &assignment);

        /// Find the entry with the given name.
        /// @retval Pointer to the entry object, or NULL if none.
        Entry *findEntry(const std::string &entryName) const;

        /// Print the unreferenced entries in this section to cerr.
        /// Messages can be suppressed using "unref_section_ok" and
        /// "unref_entries_ok".
        /// @param sectionName Name of this section, for use in output message.
        /// @retval True if any entries were printed.
        bool printUnreferenced(const std::string &sectionName);

        /// Print the contents of this section to cout (for debugging).
        void dump(const std::string &sectionName);
    };

    /// SectionTable type.  Map of strings to Section object pointers.
    typedef std::unordered_map<std::string, Section *> SectionTable;

  protected:
    /// Hash of section names to Section object pointers.
    SectionTable table;

    /// Look up section with the given name, creating a new section if
    /// not found.
    /// @retval Pointer to section object.
    Section *addSection(const std::string &sectionName);

    /// Look up section with the given name.
    /// @retval Pointer to section object, or NULL if not found.
    Section *findSection(const std::string &sectionName) const;

  public:
    /// Constructor.
    IniFile();

    /// Destructor.
    ~IniFile();

    /// Load parameter settings from given istream.  This is a helper
    /// function for load(string) and loadCPP(), which open a file
    /// and then pass it here.
    /// @retval True if successful, false if errors were encountered.
    bool load(std::istream &f);

    /// Load the specified file.
    /// Parameter settings found in the file will be merged with any
    /// already defined in this object.
    /// @param file The path of the file to load.
    /// @retval True if successful, false if errors were encountered.
    bool load(const std::string &file);

    /// Take string of the form "<section>:<parameter>=<value>" or
    /// "<section>:<parameter>+=<value>" and add to database.
    /// @retval True if successful, false if parse error.
    bool add(const std::string &s);

    /// Find value corresponding to given section and entry names.
    /// Value is returned by reference in 'value' param.
    /// @retval True if found, false if not.
    bool find(const std::string &section, const std::string &entry,
              std::string &value) const;

    /// Determine whether the entry exists within named section exists
    /// in the .ini file.
    /// @return True if the section exists.
    bool entryExists(const std::string &section,
                     const std::string &entry) const;

    /// Determine whether the named section exists in the .ini file.
    /// Note that the 'Section' class is (intentionally) not public,
    /// so all clients can do is get a bool that says whether there
    /// are any values in that section or not.
    /// @return True if the section exists.
    bool sectionExists(const std::string &section) const;

    /// Push all section names into the given vector
    void getSectionNames(std::vector<std::string> &list) const;

    /// Print unreferenced entries in object.  Iteratively calls
    /// printUnreferend() on all the constituent sections.
    bool printUnreferenced();

    /// Dump contents to cout.  For debugging.
    void dump();
};

#endif // __INIFILE_HH__

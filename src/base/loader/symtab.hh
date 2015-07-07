/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#ifndef __SYMTAB_HH__
#define __SYMTAB_HH__

#include <iosfwd>
#include <map>
#include <string>

#include "base/types.hh"
#include "sim/serialize.hh"

class SymbolTable
{
  public:
    typedef std::map<Addr, std::string> ATable;
    typedef std::map<std::string, Addr> STable;

  private:
    ATable addrTable;
    STable symbolTable;

  private:
    bool
    upperBound(Addr addr, ATable::const_iterator &iter) const
    {
        // find first key *larger* than desired address
        iter = addrTable.upper_bound(addr);

        // if very first key is larger, we're out of luck
        if (iter == addrTable.begin())
            return false;

        return true;
    }

  public:
    SymbolTable() {}
    SymbolTable(const std::string &file) { load(file); }
    ~SymbolTable() {}

    void clear();
    bool insert(Addr address, std::string symbol);
    bool load(const std::string &file);

    const ATable &getAddrTable() const { return addrTable; }
    const STable &getSymbolTable() const { return symbolTable; }

  public:
    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp);

  public:
    bool
    findSymbol(Addr address, std::string &symbol) const
    {
        ATable::const_iterator i = addrTable.find(address);
        if (i == addrTable.end())
            return false;

        symbol = (*i).second;
        return true;
    }

    bool
    findAddress(const std::string &symbol, Addr &address) const
    {
        STable::const_iterator i = symbolTable.find(symbol);
        if (i == symbolTable.end())
            return false;

        address = (*i).second;
        return true;
    }

    /// Find the nearest symbol equal to or less than the supplied
    /// address (e.g., the label for the enclosing function).
    /// @param addr     The address to look up.
    /// @param symbol   Return reference for symbol string.
    /// @param symaddr  Return reference for symbol address.
    /// @param nextaddr Address of following symbol (for
    ///                 determining valid range of symbol).
    /// @retval True if a symbol was found.
    bool
    findNearestSymbol(Addr addr, std::string &symbol, Addr &symaddr,
                      Addr &nextaddr) const
    {
        ATable::const_iterator i;
        if (!upperBound(addr, i))
            return false;

        nextaddr = i->first;
        --i;
        symaddr = i->first;
        symbol = i->second;
        return true;
    }

    /// Overload for findNearestSymbol() for callers who don't care
    /// about nextaddr.
    bool
    findNearestSymbol(Addr addr, std::string &symbol, Addr &symaddr) const
    {
        ATable::const_iterator i;
        if (!upperBound(addr, i))
            return false;

        --i;
        symaddr = i->first;
        symbol = i->second;
        return true;
    }


    bool
    findNearestAddr(Addr addr, Addr &symaddr, Addr &nextaddr) const
    {
        ATable::const_iterator i;
        if (!upperBound(addr, i))
            return false;

        nextaddr = i->first;
        --i;
        symaddr = i->first;
        return true;
    }

    bool
    findNearestAddr(Addr addr, Addr &symaddr) const
    {
        ATable::const_iterator i;
        if (!upperBound(addr, i))
            return false;

        --i;
        symaddr = i->first;
        return true;
    }
};

/// Global unified debugging symbol table (for target).  Conceptually
/// there should be one of these per System object for full system,
/// and per Process object for non-full-system, but so far one big
/// global one has worked well enough.
extern SymbolTable *debugSymbolTable;

#endif // __SYMTAB_HH__

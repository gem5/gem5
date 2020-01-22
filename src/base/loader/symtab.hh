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
 */

#ifndef __SYMTAB_HH__
#define __SYMTAB_HH__

#include <functional>
#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace Loader
{

struct Symbol
{
    enum class Binding {
        Global,
        Local,
        Weak
    };

    Binding binding;
    std::string name;
    Addr address;
};

class SymbolTable
{
  public:
    typedef std::shared_ptr<SymbolTable> SymbolTablePtr;

  private:
    typedef std::vector<Symbol> SymbolVector;
    // Map addresses to an index into the symbol vector.
    typedef std::multimap<Addr, int> AddrMap;
    // Map a symbol name to an index into the symbol vector.
    typedef std::map<std::string, int> NameMap;

    SymbolVector symbols;
    AddrMap addrMap;
    NameMap nameMap;

    bool
    upperBound(Addr addr, AddrMap::const_iterator &iter) const
    {
        // find first key *larger* than desired address
        iter = addrMap.upper_bound(addr);

        // if very first key is larger, we're out of luck
        if (iter == addrMap.begin())
            return false;

        return true;
    }

    typedef std::function<void(SymbolTable &symtab,
                               const Symbol &symbol)> SymTabOp;
    SymbolTablePtr
    operate(SymTabOp op) const
    {
        SymbolTablePtr symtab(new SymbolTable);
        for (const auto &symbol: symbols)
            op(*symtab, symbol);
        return symtab;
    }

    typedef std::function<bool(const Symbol &symbol)> SymTabFilter;
    SymbolTablePtr
    filter(SymTabFilter filter) const
    {
        SymTabOp apply_filter =
            [filter](SymbolTable &symtab, const Symbol &symbol) {
            if (filter(symbol)) {
                symtab.insert(symbol);
            }
        };
        return operate(apply_filter);
    }

    SymbolTablePtr
    filterByBinding(Symbol::Binding binding) const
    {
        auto filt = [binding](const Symbol &symbol) {
            return symbol.binding == binding;
        };
        return filter(filt);
    }

  public:
    typedef SymbolVector::iterator iterator;
    typedef SymbolVector::const_iterator const_iterator;

    const_iterator begin() const { return symbols.begin(); }
    const_iterator end() const { return symbols.end(); }

    void clear();
    // Insert either a single symbol or the contents of an entire symbol table
    // into this one.
    bool insert(const Symbol &symbol);
    bool insert(const SymbolTable &other);
    bool load(const std::string &file);
    bool empty() const { return symbols.empty(); }

    SymbolTablePtr
    offset(Addr by) const
    {
        SymTabOp op = [by](SymbolTable &symtab, const Symbol &symbol) {
            Symbol sym = symbol;
            sym.address += by;
            symtab.insert(sym);
        };
        return operate(op);
    }

    SymbolTablePtr
    mask(Addr m) const
    {
        SymTabOp op = [m](SymbolTable &symtab, const Symbol &symbol) {
            Symbol sym = symbol;
            sym.address &= m;
            symtab.insert(sym);
        };
        return operate(op);
    }

    SymbolTablePtr
    globals() const
    {
        return filterByBinding(Symbol::Binding::Global);
    }

    SymbolTablePtr
    locals() const
    {
        return filterByBinding(Symbol::Binding::Local);
    }

    SymbolTablePtr
    weaks() const
    {
        return filterByBinding(Symbol::Binding::Weak);
    }

    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp,
                     Symbol::Binding default_binding=Symbol::Binding::Global);

    const_iterator
    find(Addr address) const
    {
        AddrMap::const_iterator i = addrMap.find(address);
        if (i == addrMap.end())
            return end();

        // There are potentially multiple symbols that map to the same
        // address. For simplicity, just return the first one.
        return symbols.begin() + i->second;
    }

    const_iterator
    find(const std::string &name) const
    {
        NameMap::const_iterator i = nameMap.find(name);
        if (i == nameMap.end())
            return end();

        return symbols.begin() + i->second;
    }

    /// Find the nearest symbol equal to or less than the supplied
    /// address (e.g., the label for the enclosing function).
    /// @param addr     The address to look up.
    /// @param nextaddr Address of following symbol (for
    ///                 determining valid range of symbol).
    /// @retval A const_iterator which points to the symbol if found, or end.
    const_iterator
    findNearest(Addr addr, Addr &nextaddr) const
    {
        AddrMap::const_iterator i = addrMap.end();
        if (!upperBound(addr, i))
            return end();

        nextaddr = i->first;
        --i;
        return symbols.begin() + i->second;
    }

    /// Overload for findNearestSymbol() for callers who don't care
    /// about nextaddr.
    const_iterator
    findNearest(Addr addr) const
    {
        AddrMap::const_iterator i = addrMap.end();
        if (!upperBound(addr, i))
            return end();

        --i;
        return symbols.begin() + i->second;
    }
};

/// Global unified debugging symbol table (for target).  Conceptually
/// there should be one of these per System object for full system,
/// and per Process object for non-full-system, but so far one big
/// global one has worked well enough.
extern SymbolTable debugSymbolTable;

} // namespace Loader

#endif // __SYMTAB_HH__

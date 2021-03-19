/*
 * Copyright (c) 2021 Daniel R. Carvalho
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

#ifndef __BASE_LOADER_SYMTAB_HH__
#define __BASE_LOADER_SYMTAB_HH__

#include <functional>
#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "base/compiler.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Loader, loader);
namespace loader
{

struct Symbol
{
    enum class Binding
    {
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
    /** Vector containing all the symbols in the table. */
    typedef std::vector<Symbol> SymbolVector;
    /** Map addresses to an index into the symbol vector. */
    typedef std::multimap<Addr, int> AddrMap;
    /** Map a symbol name to an index into the symbol vector. */
    typedef std::map<std::string, int> NameMap;

    SymbolVector symbols;
    AddrMap addrMap;
    NameMap nameMap;

    /**
     * Get the first address larger than the given address, if any.
     *
     * @param addr The address to compare against.
     * @param iter An iterator to the larger-address entry.
     * @return True if successful; false if no larger addresses exist.
     */
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

    /**
     * A function that applies an operation on a symbol with respect to a
     * symbol table. The operation can, for example, simply add the symbol
     * to the table; modify and insert the symbol; do nothing at all; etc.
     */
    typedef std::function<void(SymbolTable &symtab,
                               const Symbol &symbol)> SymTabOp;

    /**
     * Create a derived symbol table by applying an operation on the symbols
     * of the current table. The current table is not modified.
     *
     * @param op The operation to be applied to the new table.
     * @return The new table.
     */
    SymbolTablePtr
    operate(SymTabOp op) const
    {
        SymbolTablePtr symtab(new SymbolTable);
        for (const auto &symbol: symbols)
            op(*symtab, symbol);
        return symtab;
    }

    /**
     * A function that applies a condition to the symbol provided to decide
     * whether the symbol is accepted, or if it must be filtered out.
     */
    typedef std::function<bool(const Symbol &symbol)> SymTabFilter;

    /**
     * Applies a filter to the symbols of the table to generate a new table.
     * The filter decides whether the symbols will be inserted in the new
     * table or not.
     *
     * @param filter The filter to be applied.
     * @return A new table, filtered.
     */
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

    /**
     * Generate a new table by applying a filter that only accepts the symbols
     * whose binding matches the given binding.
     *
     * @param The binding that must be matched.
     * @return A new table, filtered by binding.
     */
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

    /** @return An iterator to the beginning of the symbol vector. */
    const_iterator begin() const { return symbols.begin(); }

    /** @return An iterator to the end of the symbol vector. */
    const_iterator end() const { return symbols.end(); }

    /** Clears the table. */
    void clear();

    /**
     * Insert a new symbol in the table if it does not already exist. The
     * symbol must have a defined name.
     *
     * @param symbol The symbol to be inserted.
     * @return True if successful; false if table already contains the symbol.
     */
    bool insert(const Symbol &symbol);

    /**
     * Copies the symbols of another table to this table if there are no
     * common symbols between the tables.
     *
     * @param symbol The symbol to be inserted.
     * @return True if successful; false if tables contain any common symbols.
     */
    bool insert(const SymbolTable &other);

    /**
     * Verifies whether the table is empty.
     *
     * @return Whether the symbol table is empty.
     */
    bool empty() const { return symbols.empty(); }

    /**
     * Generate a new table by applying an offset to the symbols of the
     * current table. The current table is not modified.
     *
     * @param addr_offset The offset to be applied.
     * @return The new table.
     */
    SymbolTablePtr
    offset(Addr addr_offset) const
    {
        SymTabOp op =
            [addr_offset](SymbolTable &symtab, const Symbol &symbol) {
                Symbol sym = symbol;
                sym.address += addr_offset;
                symtab.insert(sym);
            };
        return operate(op);
    }

    /**
     * Generate a new table by a mask to the symbols of the current table.
     * The current table is not modified.
     *
     * @param m The mask to be applied.
     * @return The new table.
     */
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

    /**
     * Modify the symbols' name with a given transform function.
     *
     * @param func The transform function accepting the reference of the
     *             symbol's name.
     * @retval SymbolTablePtr A pointer to the modified SymbolTable copy.
     */
    SymbolTablePtr
    rename(std::function<void(std::string&)> func) const
    {
        SymTabOp op = [func](SymbolTable &symtab, const Symbol &symbol) {
            Symbol sym = symbol;
            func(sym.name);
            symtab.insert(sym);
        };
        return operate(op);
    }

    /**
     * Generates a new symbol table containing only global symbols.
     *
     * @return The new table.
     */
    SymbolTablePtr
    globals() const
    {
        return filterByBinding(Symbol::Binding::Global);
    }

    /**
     * Generates a new symbol table containing only local symbols.
     *
     * @return The new table.
     */
    SymbolTablePtr
    locals() const
    {
        return filterByBinding(Symbol::Binding::Local);
    }

    /**
     * Generates a new symbol table containing only weak symbols.
     *
     * @return The new table.
     */
    SymbolTablePtr
    weaks() const
    {
        return filterByBinding(Symbol::Binding::Weak);
    }

    /**
     * Serialize the table's contents.
     *
     * @param base The base section.
     * @param cp The checkpoint to use.
     */
    void serialize(const std::string &base, CheckpointOut &cp) const;

    /**
     * Populate the table by unserializing a checkpoint.
     *
     * @param base The base section.
     * @param cp The checkpoint to use.
     * @param default_binding The binding to be used if an unserialized
     *                        symbol's binding is not found.
     */
    void unserialize(const std::string &base, CheckpointIn &cp,
                     Symbol::Binding default_binding=Symbol::Binding::Global);

    /**
     * Search for a symbol by its address. Since many symbols can map to the
     * same address, this function returns the first found. If the symbol is
     * not found this function returns the end() iterator.
     *
     * @param address The address of the symbol being searched for.
     * @return A const iterator to the symbol. end() if not found.
     */
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

    /**
     * Search for a symbol by its name. If the symbol is not found this
     * function returns the end() iterator.
     *
     * @param name The name of the symbol being searched for.
     * @return A const iterator to the symbol. end() if not found.
     */
    const_iterator
    find(const std::string &name) const
    {
        NameMap::const_iterator i = nameMap.find(name);
        if (i == nameMap.end())
            return end();

        return symbols.begin() + i->second;
    }

    /**
     * Find the nearest symbol equal to or less than the supplied address
     * (e.g., the label for the enclosing function). If there is no valid
     * next address, next_addr is assigned 0.
     *
     * @param addr      The address to look up.
     * @param next_addr Address of following symbol (to determine the valid
     *                  range of the symbol).
     * @retval A const_iterator which points to the symbol if found, or end.
     */
    const_iterator
    findNearest(Addr addr, Addr &next_addr) const
    {
        AddrMap::const_iterator i = addrMap.end();
        if (!upperBound(addr, i))
            return end();

        // If there is no next address, make it 0 since 0 is not larger than
        // any other address, so it is clear that next is not valid
        if (i == addrMap.end()) {
            next_addr = 0;
        } else {
            next_addr = i->first;
        }
        --i;
        return symbols.begin() + i->second;
    }

    /**
     * Overload for findNearestSymbol() for callers who don't care
     * about nextaddr.
     */
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

/**
 * Global unified debugging symbol table (for target). Conceptually
 * there should be one of these per System object for full system,
 * and per Process object for non-full-system, but so far one big
 * global one has worked well enough.
 */
extern SymbolTable debugSymbolTable;

} // namespace loader
} // namespace gem5

#endif // __BASE_LOADER_SYMTAB_HH__

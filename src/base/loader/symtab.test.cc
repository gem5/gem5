/*
 * Copyright (c) 2021 Daniel R. Carvalho
 * All rights reserved
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

#include <gmock/gmock.h>
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <cassert>
#include <initializer_list>
#include <sstream>

#include "base/gtest/serialization_fixture.hh"
#include "base/loader/symtab.hh"

using namespace gem5;

/**
 * Checks if a symbol's contents matches the expected contents to generate
 * an error message. On matches an empty string is returned.
 *
 * @param symbol The symbol to check for a match.
 * @param expected The expected symbol value.
 * @return The error string, if any.
 */
std::string
getSymbolError(const loader::Symbol& symbol, const loader::Symbol& expected)
{
    std::stringstream ss;

    if (symbol.binding() != expected.binding()) {
        ss << "    symbols' bindings do not match: seen `" <<
            (int)symbol.binding() << "`, expected `" <<
            (int)expected.binding() << "`.\n";
    }

    if (symbol.type() != expected.type()) {
        ss << "    symbols' types do not match: seen `" <<
            (int)symbol.type() << "`, expected `" <<
            (int)expected.type() << "`.\n";
    }

    if (symbol.name() != expected.name()) {
        ss << "    symbols' names do not match: seen `" << symbol.name() <<
            "`, expected `" << expected.name() << "`.\n";
    }

    if (symbol.address() != expected.address()) {
        ss << "    symbols' addresses do not match: seen `" <<
            symbol.address() << "`, expected `" << expected.address() << "`.\n";
    }

    // No error, symbols match
    return ss.str();
}

/**
 * Checks that a symbol's contents matches the expected contents.
 *
 * @param m_symbol
 * @param m_expected
 * @param symbol The symbol to check for a match.
 * @param expected The expected symbol value.
 * @return A GTest's assertion result, with error message on failure.
 */
::testing::AssertionResult
checkSymbol(const char* m_symbol, const char* m_expected,
    const loader::Symbol& symbol, const loader::Symbol& expected)
{
    const std::string error = getSymbolError(symbol, expected);
    if (!error.empty()) {
        return ::testing::AssertionFailure() << "Symbols do not match (" <<
            m_symbol << " != " << m_expected << ")\n" << error;
    }
    return ::testing::AssertionSuccess();
}

/**
 * Checks that a symbol table contains only the expected symbols.
 *
 * @param symtab The table to check for matches.
 * @param expected The expected table's contents.
 * @return A GTest's assertion result, with error message on failure.
 */
::testing::AssertionResult
checkTable(const loader::SymbolTable& symtab,
    const std::initializer_list<loader::Symbol>& expected)
{
    if (expected.size() != (symtab.end() - symtab.begin())) {
        return ::testing::AssertionFailure() << "the number of symbols in "
            "the table does not match expectation (seen " <<
            (symtab.end() - symtab.begin()) << ",  expected " <<
            expected.size();
    }

    // @todo We should not assume that the symbols are seen in the given order
    auto it = symtab.begin();
    for (const auto& symbol : expected) {
        const std::string error = getSymbolError(*(it++), symbol);
        if (!error.empty()) {
            return ::testing::AssertionFailure() << error;
        }
    }

    return ::testing::AssertionSuccess();
}

/** Test that the constructor creates an empty table. */
TEST(LoaderSymtabTest, EmptyConstruction)
{
    loader::SymbolTable symtab;
    ASSERT_TRUE(symtab.empty());
    ASSERT_TRUE(checkTable(symtab, {}));
}

/** Test that the insertion of a symbol with no name fails. */
TEST(LoaderSymtabTest, InsertSymbolNoName)
{
    loader::SymbolTable symtab;

    loader::Symbol symbol = \
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "", 0x10};
    ASSERT_FALSE(symtab.insert(symbol));
    ASSERT_TRUE(checkTable(symtab, {}));
}

/** Test that the insertion of one symbol in an empty table works. */
TEST(LoaderSymtabTest, InsertOneSymbol)
{
    loader::SymbolTable symtab;

    loader::Symbol symbol = \
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10};
    ASSERT_TRUE(symtab.insert(symbol));

    ASSERT_FALSE(symtab.empty());
    ASSERT_TRUE(checkTable(symtab, {symbol}));
}

/** Test that the insertion of a symbol with an existing name fails. */
TEST(LoaderSymtabTest, InsertSymbolExistingName)
{
    loader::SymbolTable symtab;

    const std::string name = "symbol";
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            name, 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            name, 0x20},
    };
    ASSERT_TRUE(symtab.insert(symbols[0]));
    ASSERT_FALSE(symtab.insert(symbols[1]));

    // Check that the second symbol has not been inserted
    ASSERT_TRUE(checkTable(symtab, {symbols[0]}));
}

/** Test that the insertion of a symbol with an existing address works. */
TEST(LoaderSymtabTest, InsertSymbolExistingAddress)
{
    loader::SymbolTable symtab;

    const Addr addr = 0x10;
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", addr},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", addr},
    };
    ASSERT_TRUE(symtab.insert(symbols[0]));
    ASSERT_TRUE(symtab.insert(symbols[1]));

    // Check that all symbols are present
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1]}));
}

/** Test that the insertion of one symbol in a non-empty table works. */
TEST(LoaderSymtabTest, InsertMultipleSymbols)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2]}));
}

/**
 * Test that a table with multiple entries becomes empty after being cleared.
 */
TEST(LoaderSymtabTest, ClearMultiple)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    symtab.clear();
    ASSERT_TRUE(symtab.empty());
    ASSERT_TRUE(checkTable(symtab, {}));
}

/**
 * Test the creation of a new table with offsets applied to the original
 * symbols' addresses. Also verifies that the original table is kept the same.
 */
TEST(LoaderSymtabTest, Offset)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    Addr offset = 0x5;
    const auto symtab_new = symtab.offset(offset);

    // Check that the original table is not modified
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2]}));

    // Check that the new table is offset
    loader::Symbol expected_symbols[] = {
        {symbols[0].binding(), symbols[0].type(), symbols[0].name(),
            symbols[0].address() + offset},
        {symbols[1].binding(), symbols[1].type(), symbols[1].name(),
            symbols[1].address() + offset},
        {symbols[2].binding(), symbols[2].type(), symbols[2].name(),
            symbols[2].address() + offset},
    };
    ASSERT_TRUE(checkTable(*symtab_new, {expected_symbols[0],
        expected_symbols[1], expected_symbols[2]}));
}

/**
 * Test the creation of a new table with masks applied to the original
 * symbols' addresses. Also verifies that the original table is kept the same.
 */
TEST(LoaderSymtabTest, Mask)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x1310},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x2810},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x2920},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol4", 0x3C20},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));
    EXPECT_TRUE(symtab.insert(symbols[3]));

    Addr mask = 0x0110;
    const auto symtab_new = symtab.mask(mask);

    // Check that the original table is not modified
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2],
        symbols[3]}));

    // Check that the new table is masked
    loader::Symbol expected_symbols[] = {
        {symbols[0].binding(), symbols[0].type(), symbols[0].name(),
            symbols[0].address() & mask},
        {symbols[1].binding(), symbols[1].type(), symbols[1].name(),
            symbols[1].address() & mask},
        {symbols[2].binding(), symbols[2].type(), symbols[2].name(),
            symbols[2].address() & mask},
        {symbols[3].binding(), symbols[3].type(), symbols[3].name(),
            symbols[3].address() & mask},
    };
    ASSERT_TRUE(checkTable(*symtab_new, {expected_symbols[0],
        expected_symbols[1], expected_symbols[2], expected_symbols[3]}));
}

/**
 * Test the creation of a new table with renamed symbols. Also verifies
 * that the original table is kept the same.
 */
TEST(LoaderSymtabTest, Rename)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol4", 0x40},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));
    EXPECT_TRUE(symtab.insert(symbols[3]));

    const auto symtab_new =
        symtab.rename([](const std::string &name) { return name + "_suffix"; });

    // Check that the original table is not modified
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2],
        symbols[3]}));

    // Check that the new table's symbols have been renamed
    loader::Symbol expected_symbols[] = {
        {symbols[0].binding(), symbols[0].type(), symbols[0].name() + "_suffix",
            symbols[0].address()},
        {symbols[1].binding(), symbols[1].type(), symbols[1].name() + "_suffix",
            symbols[1].address()},
        {symbols[2].binding(), symbols[2].type(), symbols[2].name() + "_suffix",
            symbols[2].address()},
        {symbols[3].binding(), symbols[3].type(), symbols[3].name() + "_suffix",
            symbols[3].address()},
    };
    ASSERT_TRUE(checkTable(*symtab_new, {expected_symbols[0],
        expected_symbols[1], expected_symbols[2], expected_symbols[3]}));
}

/**
 * Tests that renaming symbols respects the rule that symbols in a table
 * must have unique names.
 */
TEST(LoaderSymtabTest, RenameNonUnique)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol4", 0x40},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));
    EXPECT_TRUE(symtab.insert(symbols[3]));

    int i = 0;
    const auto symtab_new = symtab.rename([&i](const std::string &name)
        {
            if ((i++ % 2) == 0) {
                return std::string("NonUniqueName");
            } else {
                return name;
            }
        });

    // Check that the original table is not modified
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2],
        symbols[3]}));

    // Check that the new table's symbols have been renamed, yet it does not
    // contain the symbols with duplicated names
    loader::Symbol expected_symbols[] = {
        {symbols[0].binding(), symbols[0].type(), "NonUniqueName",
            symbols[0].address()},
        {symbols[1].binding(), symbols[1].type(), symbols[1].name(),
            symbols[1].address()},
        {symbols[3].binding(), symbols[3].type(), symbols[3].name(),
            symbols[3].address()},
    };
    ASSERT_TRUE(checkTable(*symtab_new, {expected_symbols[0],
        expected_symbols[1], expected_symbols[2]}));
}

/**
 * Test the creation of a new filtered table containing only the global symbols
 * of the original table. Also verifies if the original table is kept the same.
 */
TEST(LoaderSymtabTest, Globals)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
        {loader::Symbol::Binding::Weak, loader::Symbol::SymbolType::Other,
            "symbol4", 0x40},
        {loader::Symbol::Binding::Weak, loader::Symbol::SymbolType::Other,
            "symbol5", 0x50}
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));
    EXPECT_TRUE(symtab.insert(symbols[3]));
    EXPECT_TRUE(symtab.insert(symbols[4]));

    const auto symtab_new = symtab.globals();

    // Check that the original table is not modified
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2],
        symbols[3], symbols[4]}));

    // Check that the new table only contains globals
    ASSERT_TRUE(checkTable(*symtab_new, {symbols[1]}));
}

/**
 * Test the creation of a new filtered table containing only the local symbols
 * of the original table. Also verifies if the original table is kept the same.
 */
TEST(LoaderSymtabTest, Locals)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
        {loader::Symbol::Binding::Weak, loader::Symbol::SymbolType::Other,
            "symbol4", 0x40},
        {loader::Symbol::Binding::Weak, loader::Symbol::SymbolType::Other,
            "symbol5", 0x50}
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));
    EXPECT_TRUE(symtab.insert(symbols[3]));
    EXPECT_TRUE(symtab.insert(symbols[4]));

    const auto symtab_new = symtab.locals();

    // Check that the original table is not modified
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2],
        symbols[3], symbols[4]}));

    // Check that the new table only contains locals
    ASSERT_TRUE(checkTable(*symtab_new, {symbols[0], symbols[2]}));
}

/**
 * Test the creation of a new filtered table containing only the weak symbols
 * of the original table. Also verifies if the original table is kept the same.
 */
TEST(LoaderSymtabTest, Weaks)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
        {loader::Symbol::Binding::Weak, loader::Symbol::SymbolType::Other,
            "symbol4", 0x40},
        {loader::Symbol::Binding::Weak, loader::Symbol::SymbolType::Other,
            "symbol5", 0x50}
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));
    EXPECT_TRUE(symtab.insert(symbols[3]));
    EXPECT_TRUE(symtab.insert(symbols[4]));

    const auto symtab_new = symtab.weaks();

    // Check that the original table is not modified
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2],
        symbols[3], symbols[4]}));

    // Check that the new table only contains weaks
    ASSERT_TRUE(checkTable(*symtab_new, {symbols[3], symbols[4]}));
}

/**
 * Test the creation of a new filtered table containing only function symbols
 * of the original table. Also verifies if the original table is kept the same.
 */
TEST(LoaderSymtabTest, FunctionSymbols)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::NoType,
            "symbol", 0x10},
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::File,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::Function,
            "symbol3", 0x30},
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::Object,
            "symbol4", 0x40},
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::Function,
            "symbol5", 0x50}
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));
    EXPECT_TRUE(symtab.insert(symbols[3]));
    EXPECT_TRUE(symtab.insert(symbols[4]));

    const auto symtab_new = symtab.functionSymbols();

    // Check that the original table is not modified
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2],
        symbols[3], symbols[4]}));

    // Check that the new table only contains function symbols
    ASSERT_TRUE(checkTable(*symtab_new, {symbols[2], symbols[4]}));
}

/** Test searching for a non-existent address. */
TEST(LoaderSymtabTest, FindNonExistentAddress)
{
    loader::SymbolTable symtab;

    loader::Symbol symbol = \
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10};
    EXPECT_TRUE(symtab.insert(symbol));

    ASSERT_EQ(symtab.find(0x0), symtab.end());
}

/** Test searching for a unique address. */
TEST(LoaderSymtabTest, FindUniqueAddress)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    const auto it = symtab.find(symbols[2].address());
    ASSERT_NE(it, symtab.end());
    ASSERT_PRED_FORMAT2(checkSymbol, *it, symbols[2]);
}

/**
 * Test that searching for a non-unique address returns the first occurrence.
 */
TEST(LoaderSymtabTest, FindNonUniqueAddress)
{
    loader::SymbolTable symtab;

    const Addr addr = 0x20;
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", addr},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", addr},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    const auto it = symtab.find(symbols[1].address());
    ASSERT_NE(it, symtab.end());
    ASSERT_PRED_FORMAT2(checkSymbol, *it, symbols[1]);
}

/** Test searching for a non-existent name. */
TEST(LoaderSymtabTest, FindNonExistentName)
{
    loader::SymbolTable symtab;

    loader::Symbol symbol = \
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10};
    EXPECT_TRUE(symtab.insert(symbol));

    const auto it = symtab.find("symbol2");
    ASSERT_EQ(it, symtab.end());
}

/** Test searching for an existing name. */
TEST(LoaderSymtabTest, FindExistingName)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    const auto it = symtab.find(symbols[1].name());
    ASSERT_NE(it, symtab.end());
    ASSERT_PRED_FORMAT2(checkSymbol, *it, symbols[1]);
}

/** Test searching for an existent address using findNearest. */
TEST(LoaderSymtabTest, FindNearestExact)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));

    const auto it = symtab.findNearest(symbols[1].address());
    ASSERT_NE(it, symtab.end());
    ASSERT_PRED_FORMAT2(checkSymbol, *it, symbols[1]);
}

/**
 * Test that, in a table containing address A, searching for the nearest
 * address of an address B where B=A+x returns A.
 */
TEST(LoaderSymtabTest, FindNearestRound)
{
    loader::SymbolTable symtab;

    loader::Symbol symbol = \
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10};
    EXPECT_TRUE(symtab.insert(symbol));

    const auto it = symtab.findNearest(symbol.address() + 0x1);
    ASSERT_NE(it, symtab.end());
    ASSERT_PRED_FORMAT2(checkSymbol, *it, symbol);
}

/**
 * Test that, in a table containing address A1 and A2, where A1<A2, searching
 * for the nearest address of an address B where B=A1+x and B<A2 returns A1,
 * and marks A2 as the next address.
 */
TEST(LoaderSymtabTest, FindNearestRoundWithNext)
{
    loader::SymbolTable symtab;

    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));

    Addr next_addr;
    const auto it = symtab.findNearest(symbols[0].address() + 0x1, next_addr);
    ASSERT_NE(it, symtab.end());
    ASSERT_PRED_FORMAT2(checkSymbol, *it, symbols[0]);
    ASSERT_EQ(next_addr, symbols[1].address());
}

/**
 * Test that, in a table containing address A, searching for the nearest
 * address of an address B where B=A+x returns A; however, the next address
 * is non-existent, so it is marked as not valid.
 */
TEST(LoaderSymtabTest, FindNearestRoundWithNextNonExistent)
{
    loader::SymbolTable symtab;

    loader::Symbol symbol = \
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10};
    EXPECT_TRUE(symtab.insert(symbol));

    Addr next_addr;
    const auto it = symtab.findNearest(symbol.address() + 0x1, next_addr);
    ASSERT_NE(it, symtab.end());
    ASSERT_PRED_FORMAT2(checkSymbol, *it, symbol);
    ASSERT_EQ(next_addr, 0);
}

/**
 * Test that searching for the nearest address of an address lower than the
 * lowest address fails.
 */
TEST(LoaderSymtabTest, FindNearestNonExistent)
{
    loader::SymbolTable symtab;

    loader::Symbol symbol = \
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10};
    EXPECT_TRUE(symtab.insert(symbol));

    const auto it = symtab.findNearest(symbol.address() - 0x1);
    ASSERT_EQ(it, symtab.end());
}

/**
 * Test that the insertion of a symbol table's symbols in another table works
 * when any symbol name conflicts.
 */
TEST(LoaderSymtabTest, InsertTableConflicting)
{
    const std::string name = "symbol";
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            name, 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol4", 0x40},
        // Introduce name conflict
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            name, 0x50},
    };

    // Populate table 1
    loader::SymbolTable symtab;
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    // Populate table 2
    loader::SymbolTable symtab2;
    EXPECT_TRUE(symtab2.insert(symbols[3]));
    EXPECT_TRUE(symtab2.insert(symbols[4]));

    // Do the insertion
    ASSERT_FALSE(symtab.insert(symtab2));

    // Check that none of the tables change
    ASSERT_TRUE(checkTable(symtab2, {symbols[3], symbols[4]}));
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2]}));
}

/**
 * Test that the insertion of a symbol table's symbols in another table works
 * when no symbols conflict.
 */
TEST(LoaderSymtabTest, InsertTable)
{
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol4", 0x40},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol5", 0x50},
    };

    // Populate table 1
    loader::SymbolTable symtab;
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    // Populate table 2
    loader::SymbolTable symtab2;
    EXPECT_TRUE(symtab2.insert(symbols[3]));
    EXPECT_TRUE(symtab2.insert(symbols[4]));

    // Do the insertion
    symtab.insert(symtab2);

    // Check that symtab2 does not change
    ASSERT_TRUE(checkTable(symtab2, {symbols[3], symbols[4]}));

    // Check that the symbols from symtab2 have been inserted in symtab
    ASSERT_TRUE(checkTable(symtab, {symbols[0], symbols[1], symbols[2],
        symbols[3], symbols[4]}));
}

using LoaderSymtabSerializationFixture = SerializationFixture;

/** Test serialization. */
TEST_F(LoaderSymtabSerializationFixture, Serialization)
{
    // Populate the table
    loader::SymbolTable symtab;
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    EXPECT_TRUE(symtab.insert(symbols[0]));
    EXPECT_TRUE(symtab.insert(symbols[1]));
    EXPECT_TRUE(symtab.insert(symbols[2]));

    // Serialization
    std::ostringstream cp;
    Serializable::ScopedCheckpointSection scs(cp, "Section1");
    symtab.serialize("test", cp);

    // Verify the output
    ASSERT_THAT(cp.str(), ::testing::StrEq("\n[Section1]\ntest.size=3\n"
        "test.addr_0=16\ntest.symbol_0=symbol\ntest.binding_0=1\n"
        "test.type_0=5\n"
        "test.addr_1=32\ntest.symbol_1=symbol2\ntest.binding_1=1\n"
        "test.type_1=5\n"
        "test.addr_2=48\ntest.symbol_2=symbol3\ntest.binding_2=1\n"
        "test.type_2=5\n"));
}

/** Test unserialization. */
TEST_F(LoaderSymtabSerializationFixture, Unserialization)
{
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    simulateSerialization("\n[Section1]\ntest.size=3\n"
        "test.addr_0=16\ntest.symbol_0=symbol\ntest.binding_0=1\n"
        "test.type_0=5\n"
        "test.addr_1=32\ntest.symbol_1=symbol2\ntest.binding_1=1\n"
        "test.type_1=5\n"
        "test.addr_2=48\ntest.symbol_2=symbol3\ntest.binding_2=1\n"
        "test.type_2=5\n");

    loader::SymbolTable unserialized_symtab;
    CheckpointIn cp(getDirName());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");
    unserialized_symtab.unserialize("test", cp);

    // Make sure that the symbols in symtab are present in the
    // unserialized table
    ASSERT_TRUE(checkTable(unserialized_symtab, {symbols[0], symbols[1],
        symbols[2]}));
}

/**
 * Test unserialization missing binding.
 * @todo Since there is no way to create a checkpoint without binding anymore,
 * this functionality should be deprecated at some point.
 */
TEST_F(LoaderSymtabSerializationFixture, UnserializationMissingBinding)
{
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Global, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    simulateSerialization("\n[Section1]\ntest.size=3\n"
        "test.addr_0=16\ntest.symbol_0=symbol\ntest.binding_0=1\n"
        "test.type_0=5\n"
        "test.addr_1=32\ntest.symbol_1=symbol2\ntest.type_1=5\n"
        "test.addr_2=48\ntest.symbol_2=symbol3\ntest.binding_2=1\n"
        "test.type_2=5\n");

    loader::SymbolTable unserialized_symtab;
    CheckpointIn cp(getDirName());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");

    unserialized_symtab.unserialize("test", cp);

    // Make sure that the symbols in symtab are present in the
    // unserialized table
    ASSERT_TRUE(checkTable(unserialized_symtab, {symbols[0], symbols[1],
        symbols[2]}));
}

/**
 * Test unserialization missing binding with a different default value.
 * @todo Since there is no way to create a checkpoint without binding anymore,
 * this functionality should be deprecated at some point.
 */
TEST_F(LoaderSymtabSerializationFixture,
    UnserializationMissingBindingChangeDefault)
{
    loader::Symbol symbols[] = {
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol", 0x10},
        {loader::Symbol::Binding::Weak, loader::Symbol::SymbolType::Other,
            "symbol2", 0x20},
        {loader::Symbol::Binding::Local, loader::Symbol::SymbolType::Other,
            "symbol3", 0x30},
    };
    simulateSerialization("\n[Section1]\ntest.size=3\n"
        "test.addr_0=16\ntest.symbol_0=symbol\ntest.binding_0=1\n"
        "test.type_0=5\n"
        "test.addr_1=32\ntest.symbol_1=symbol2\n"
        "test.type_1=5\n"
        "test.addr_2=48\ntest.symbol_2=symbol3\ntest.binding_2=1\n"
        "test.type_2=5\n");

    loader::SymbolTable unserialized_symtab;
    CheckpointIn cp(getDirName());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");

    unserialized_symtab.unserialize("test", cp,
        loader::Symbol::Binding::Weak);

    // Make sure that the symbols in symtab are present in the
    // unserialized table
    ASSERT_TRUE(checkTable(unserialized_symtab, {symbols[0], symbols[1],
        symbols[2]}));
}

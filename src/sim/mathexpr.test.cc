/*
 * Copyright (c) 2026 Sankur Kundu
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

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "sim/mathexpr.hh"

using namespace gem5;

namespace
{

double
evalExpr(const std::string &expr,
         const std::map<std::string, double> &vars = {})
{
    MathExpr e(expr);
    return e.eval([&vars](std::string name) { return vars.at(name); });
}

} // anonymous namespace

TEST(MathExprTest, Arithmetic)
{
    EXPECT_DOUBLE_EQ(evalExpr("2-1"), 1.0);
    EXPECT_DOUBLE_EQ(evalExpr("1+2*3"), 7.0);
    EXPECT_DOUBLE_EQ(evalExpr("(1+2)*3"), 9.0);
    EXPECT_DOUBLE_EQ(evalExpr("2^3"), 8.0);
    EXPECT_DOUBLE_EQ(evalExpr("-2"), -2.0);
    EXPECT_DOUBLE_EQ(evalExpr("1.8e9"), 1.8e9);
}

// A signed exponent is part of the number, not a binary + or -.
TEST(MathExprTest, SignedExponent)
{
    EXPECT_DOUBLE_EQ(evalExpr("1e-3"), 1e-3);
    EXPECT_DOUBLE_EQ(evalExpr("1.8e-09"), 1.8e-9);
    EXPECT_DOUBLE_EQ(evalExpr("1.0e+3"), 1.0e3);
    EXPECT_DOUBLE_EQ(evalExpr("2.5E-2"), 2.5e-2);
    EXPECT_DOUBLE_EQ(evalExpr(".5e-1"), 0.05);
    EXPECT_DOUBLE_EQ(evalExpr("-1e-3"), -1e-3);
    EXPECT_DOUBLE_EQ(evalExpr("1e-3-2e-3"), 1e-3 - 2e-3);
    EXPECT_DOUBLE_EQ(evalExpr("2*1e-3+1"), 2 * 1e-3 + 1);
    EXPECT_DOUBLE_EQ(evalExpr("(1e-3)"), 1e-3);
}

// The expression from gem5 issue #3629: a power model with an energy per
// access written in exponent form.
TEST(MathExprTest, SignedExponentWithVariables)
{
    std::map<std::string, double> vars = {{"x", 15839.0},
                                          {"simSeconds", 1e-4}};
    EXPECT_DOUBLE_EQ(evalExpr("x * 1.8e-09 / simSeconds", vars),
                     15839.0 * 1.8e-9 / 1e-4);

    MathExpr e("x*1.8e-09/simSeconds");
    std::vector<std::string> names = e.getVariables();
    EXPECT_EQ(names.size(), 2u);
}

// An 'e' at the end of a variable name is not an exponent.
TEST(MathExprTest, VariableEndingInE)
{
    std::map<std::string, double> vars = {
        {"x1e", 5.0}, {"size", 2.0}, {"a.b1e", 7.0}};
    EXPECT_DOUBLE_EQ(evalExpr("x1e-3", vars), 2.0);
    EXPECT_DOUBLE_EQ(evalExpr("size-1", vars), 1.0);
    EXPECT_DOUBLE_EQ(evalExpr("a.b1e+1", vars), 8.0);
}

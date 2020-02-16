/*
 * Copyright (c) 2019 The Regents of the University of California
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

#include <gtest/gtest.h>

#include "base/match.hh"

TEST(MatchTest, Add)
{
    /*
     * ObjectMatch.add will add all the expressions from one ObjectMatch to
     * the other.
     */
    ObjectMatch object_match_1("token1.token2");
    ObjectMatch object_match_2("token3");
    ObjectMatch object_match_3;

    object_match_3.add(object_match_1);
    object_match_3.add(object_match_2);

    std::vector<std::vector<std::string> > expressions =
        object_match_3.getExpressions();

    EXPECT_EQ(2, expressions.size());
    EXPECT_EQ(2, expressions[0].size());
    EXPECT_EQ(1, expressions[1].size());

    EXPECT_EQ(expressions[0][0], "token1");
    EXPECT_EQ(expressions[0][1], "token2");
    EXPECT_EQ(expressions[1][0], "token3");
}

TEST(MatchTest, SetExpression)
{
    ObjectMatch object_match;
    object_match.setExpression("A.B.C.D");

    std::vector<std::vector<std::string> > expressions =
        object_match.getExpressions();

    EXPECT_EQ(1, expressions.size());
    EXPECT_EQ(4, expressions[0].size());

    EXPECT_EQ("A", expressions[0][0]);
    EXPECT_EQ("B", expressions[0][1]);
    EXPECT_EQ("C", expressions[0][2]);
    EXPECT_EQ("D", expressions[0][3]);
}

TEST(MatchTest, SetExpressionVector)
{
    ObjectMatch object_match;

    std::vector<std::string> to_add;
    to_add.push_back("A.B.C.D");
    to_add.push_back("E.F.G");

    object_match.setExpression(to_add);

    std::vector<std::vector<std::string> > expressions =
        object_match.getExpressions();

    EXPECT_EQ(2, expressions.size());
    EXPECT_EQ(4, expressions[0].size());
    EXPECT_EQ(3, expressions[1].size());

    EXPECT_EQ("A", expressions[0][0]);
    EXPECT_EQ("B", expressions[0][1]);
    EXPECT_EQ("C", expressions[0][2]);
    EXPECT_EQ("D", expressions[0][3]);
    EXPECT_EQ("E", expressions[1][0]);
    EXPECT_EQ("F", expressions[1][1]);
    EXPECT_EQ("G", expressions[1][2]);
}

TEST(MatchTest, SimpleMatch)
{
    ObjectMatch object_match("this.is.a.perfect.match");
    EXPECT_TRUE(object_match.match("this.is.a.perfect.match"));
}

TEST(MatchTest, SimpleMismatch)
{
    ObjectMatch object_match("this.is.a.perfect.match");
    EXPECT_FALSE(object_match.match("this.is.a.perfect.--"));
}

TEST(MatchTest, MultipleExpressionsMatch)
{
    ObjectMatch object_match;
    std::vector<std::string> expressions;
    expressions.push_back("A.B.C.D");
    expressions.push_back("E.F.G");
    object_match.setExpression(expressions);

    EXPECT_TRUE(object_match.match("A.B.C.D"));
    EXPECT_TRUE(object_match.match("E.F.G"));
}

TEST(MatchTest, MultipleExpressionsMismatch)
{
    ObjectMatch object_match;
    std::vector<std::string> expressions;
    expressions.push_back("A.B.C.D");
    expressions.push_back("E.F.G");
    object_match.setExpression(expressions);

    EXPECT_FALSE(object_match.match("B.C.D"));
    EXPECT_FALSE(object_match.match("D.E.F.G"));
}

TEST(MatchTest, WildCardMatch)
{
    ObjectMatch object_match("this.is.a.*.match");

    /*
     * Note: the wildcard token can represent an empty token.
     */
    EXPECT_TRUE(object_match.match("this.is.a.match"));
    EXPECT_TRUE(object_match.match("this.is.a.perfect.match"));
    EXPECT_TRUE(object_match.match("this.is.a.great.match"));
}

TEST(MatchTest, WildCardMismatch)
{
    ObjectMatch object_match("this.is.a.*.match");

    EXPECT_FALSE(object_match.match("this.is.a.bla.bla.match"));
    EXPECT_FALSE(object_match.match("this.is.a.great.match--"));
}

TEST(MatchTest, TokensEmptyNoMatch)
{
    ObjectMatch object_match;
    EXPECT_FALSE(object_match.match("token1"));
}

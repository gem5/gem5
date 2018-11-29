/*
 * Copyright 2018 Google, Inc.
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
 * Authors: Gabe Black
 */

#include <gtest/gtest.h>

#include <initializer_list>
#include <iostream>
#include <vector>

#include "base/fiber.hh"

class TestFiber : public Fiber
{
  public:
    const char *name;
    std::vector<Fiber *> next;

    TestFiber(const char *name, std::initializer_list<Fiber *> l);

    void checkExpected();
    void main();
};

extern TestFiber a;
extern TestFiber b;
extern TestFiber c;

TestFiber a("A", { &b, &a, Fiber::primaryFiber(), &b, &c });
TestFiber b("B", { &a, &c });
TestFiber c("C", { &a, Fiber::primaryFiber(), Fiber::primaryFiber() });

std::vector<TestFiber *>::iterator expectedIt;
std::vector<TestFiber *> expected({
    &a, &b, &a, &a, /* main Fiber, */
    &a, &b, &c, &a, &c,
    /* main Fiber, */ &c, &c
});

TestFiber::TestFiber(
        const char *name, std::initializer_list<Fiber *> l) :
    name(name), next(l)
{}

void
TestFiber::checkExpected()
{
    ASSERT_NE(expectedIt, expected.end());
    TestFiber *e = *expectedIt++;
    EXPECT_EQ(e, this) << "Expected " << e->name << ", got " << name;
}

void
TestFiber::main()
{
    checkExpected();
    for (auto &n : next) {
        n->run();
        checkExpected();
    }
}

TEST(Fiber, Switching)
{
    expectedIt = expected.begin();

    a.run();
    EXPECT_EQ(expectedIt - expected.begin(), 4);

    a.run();
    EXPECT_EQ(expectedIt - expected.begin(), 9);

    c.run();
    EXPECT_EQ(expectedIt - expected.begin(), 10);

    EXPECT_FALSE(a.finished());
    EXPECT_FALSE(b.finished());
    EXPECT_FALSE(c.finished());

    c.run();
    EXPECT_EQ(expected.end(), expectedIt) <<
        "Didn't exactly use up the expected Fiber sequence";

    EXPECT_TRUE(c.finished());
}

int currentIndex = 0;

class LinkedFiber : public Fiber
{
  public:
    const int index;
    LinkedFiber(Fiber *link, int index) : Fiber(link), index(index) {}

    void
    main()
    {
        EXPECT_EQ(currentIndex, index);
        currentIndex++;
    }
};

TEST(Fiber, Linked)
{
    currentIndex = 0;

    LinkedFiber lf3(Fiber::primaryFiber(), 3);
    LinkedFiber lf2(&lf3, 2);
    LinkedFiber lf1(&lf2, 1);
    LinkedFiber lf0(&lf1, 0);

    lf0.run();

    EXPECT_EQ(currentIndex, 4);
}

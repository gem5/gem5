/*
 * Copyright (c) 2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 */

#include <gtest/gtest.h>

#include <initializer_list>
#include <iostream>
#include <vector>

#include "base/fiber.hh"

/** This test is checking if the "started" member has its expected
 * value before and after the fiber runs. In the test an empty fiber
 * is used since we are just interested on the _started member and
 * nothing more.
 */
TEST(Fiber, Starting)
{
    class StartingFiber : public Fiber
    {
      public:
        StartingFiber(Fiber *link) : Fiber(link) {}
        void main() { /** Do nothing */ }
    };

    StartingFiber fiber(Fiber::primaryFiber());

    ASSERT_FALSE(fiber.started());

    fiber.run();

    ASSERT_TRUE(fiber.started());
}

class SwitchingFiber : public Fiber
{
  public:
    const char *name;
    std::vector<Fiber *> next;

    SwitchingFiber(const char *name, std::initializer_list<Fiber *> l);

    void checkExpected();
    void main();
};

extern SwitchingFiber a;
extern SwitchingFiber b;
extern SwitchingFiber c;

SwitchingFiber a("A", { &b, &a, Fiber::primaryFiber(), &b, &c });
SwitchingFiber b("B", { &a, &c });
SwitchingFiber c("C", { &a, Fiber::primaryFiber(), Fiber::primaryFiber() });

std::vector<SwitchingFiber *>::iterator expectedIt;
std::vector<SwitchingFiber *> expected({
    &a, &b, &a, &a, /* main Fiber, */
    &a, &b, &c, &a, &c,
    /* main Fiber, */ &c, &c
});

SwitchingFiber::SwitchingFiber(
        const char *name, std::initializer_list<Fiber *> l) :
    name(name), next(l)
{}

void
SwitchingFiber::checkExpected()
{
    ASSERT_NE(expectedIt, expected.end());
    SwitchingFiber *e = *expectedIt++;
    EXPECT_EQ(e, this) << "Expected " << e->name << ", got " << name;
}

void
SwitchingFiber::main()
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

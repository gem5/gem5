/*
 * Copyright 2021 Daniel R. Carvalho
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
#include <gtest/gtest.h>

#include <iostream>
#include <sstream>
#include <streambuf>

#include "base/gtest/logging.hh"
#include "sim/port.hh"

using namespace gem5;

class TestPort : public Port
{
  public:
    TestPort(PortID _id) : Port("TestPort", _id) {}
};

/** Test getting the port id. */
TEST(PortTest, GetId)
{
    TestPort port(0), port2(2);
    ASSERT_EQ(port.getId(), 0);
    ASSERT_EQ(port2.getId(), 2);
}

/** Test connecting one of the ports. */
TEST(PortTest, OneSidedConnection)
{
    TestPort port(0), port2(2);

    // Both ports start unconnected
    ASSERT_FALSE(port.isConnected());
    ASSERT_FALSE(port2.isConnected());

    // Bind one of the ports to the other
    port.bind(port2);

    // Binding a single side does not bind both ports automatically
    ASSERT_TRUE(port.isConnected());
    ASSERT_EQ(&port.getPeer(), &port2);
    ASSERT_FALSE(port2.isConnected());

    // Unbind the port
    port.unbind();
    ASSERT_FALSE(port.isConnected());
    ASSERT_FALSE(port2.isConnected());
}

/** Test connecting both ports. */
TEST(PortTest, TwoSidedConnection)
{
    TestPort port(0), port2(2);

    // Both ports start unconnected
    ASSERT_FALSE(port.isConnected());
    ASSERT_FALSE(port2.isConnected());

    // Bind the ports
    port.bind(port2);
    port2.bind(port);
    ASSERT_TRUE(port.isConnected());
    ASSERT_EQ(&port.getPeer(), &port2);
    ASSERT_TRUE(port2.isConnected());
    ASSERT_EQ(&port2.getPeer(), &port);

    // Unbinding one port does not automatically unbind the other
    port.unbind();
    ASSERT_FALSE(port.isConnected());
    ASSERT_TRUE(port2.isConnected());
    ASSERT_EQ(&port2.getPeer(), &port);

    // Finish unbinding
    port2.unbind();
    ASSERT_FALSE(port.isConnected());
    ASSERT_FALSE(port2.isConnected());
}

/** Test that manually overwriting a bind is possible. */
TEST(PortTest, OverwriteConnection)
{
    TestPort port(0), port2(2), port3(6);

    // All ports start unconnected
    ASSERT_FALSE(port.isConnected());
    ASSERT_FALSE(port2.isConnected());
    ASSERT_FALSE(port3.isConnected());

    // Bind one of the ports to the other
    port.bind(port2);
    ASSERT_TRUE(port.isConnected());
    ASSERT_EQ(&port.getPeer(), &port2);
    ASSERT_FALSE(port2.isConnected());
    ASSERT_FALSE(port3.isConnected());

    // Bind one of the ports to a third
    port.bind(port3);
    ASSERT_TRUE(port.isConnected());
    ASSERT_EQ(&port.getPeer(), &port3);
    ASSERT_FALSE(port2.isConnected());
    ASSERT_FALSE(port3.isConnected());
}

/** Test that a take over must have a valid port. */
TEST(PortDeathTest, TakeOverNoPort)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif
    TestPort port(0);
    ASSERT_DEATH(port.takeOverFrom(nullptr), "");
}

/** Test that a port that is not connected cannot be taken over from. */
TEST(PortDeathTest, TakeOverDisconnected)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif
    TestPort port(0), port2(2);
    ASSERT_DEATH(port.takeOverFrom(&port2), "");
}

/**
 * Test that a port that is already connected cannot take over another port.
 *
 * Before the take over the connections are: port -> port2
 */
TEST(PortDeathTest, TakeOverConnected)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif
    TestPort port(0), port2(2);
    port.bind(port2);
    ASSERT_DEATH(port.takeOverFrom(&port2), "");
}

/**
 * Test that the peer of the port being taken over from must also be connected.
 *
 * Before the take over the connections are: port2 -> port3
 */
TEST(PortDeathTest, TakeOverOneSided)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif
    TestPort port(0), port2(2), port3(6);
    port2.bind(port3);
    ASSERT_DEATH(port.takeOverFrom(&port2), "");
}

/**
 * Test one-sided take over. This might be a bug caused by the fact that a
 * bind() does not automatically bind both ports.
 *
 * Before the take over the connections are: port2 -> port3 -> port4
 * After the take over the connections are: port <-> port3
 */
TEST(PortTest, TakeOverOneSided)
{
    TestPort port(0), port2(2), port3(6), port4(10);
    port2.bind(port3);
    port3.bind(port4);
    port.takeOverFrom(&port2);

    ASSERT_TRUE(port.isConnected());
    ASSERT_EQ(&port.getPeer(), &port3);
    ASSERT_FALSE(port2.isConnected());
    ASSERT_TRUE(port3.isConnected());
    ASSERT_EQ(&port3.getPeer(), &port);
    ASSERT_FALSE(port4.isConnected());
}

/**
 * Test proper take over.
 *
 * Before the take over the connections are: port2 <-> port3
 * After the take over the connections are: port <-> port3
*/
TEST(PortTest, TakeOver)
{
    TestPort port(0), port2(2), port3(6);
    port2.bind(port3);
    port3.bind(port2);
    port.takeOverFrom(&port2);

    ASSERT_TRUE(port.isConnected());
    ASSERT_EQ(&port.getPeer(), &port3);
    ASSERT_TRUE(port3.isConnected());
    ASSERT_EQ(&port3.getPeer(), &port);
}

/** Test that the ostream operator prints the port's name. */
TEST(PortTest, Print)
{
    // Temporarily redirect cout
    std::streambuf *old;
    std::ostringstream buffer;
    old = std::cout.rdbuf(buffer.rdbuf());

    // Must use EXPECT, so that cout is restored
    TestPort port(0);
    std::cout << port << std::endl;
    EXPECT_EQ(buffer.str(), "TestPort\n");

    // Restore cout's streambuf
    std::cout.rdbuf(old);
}

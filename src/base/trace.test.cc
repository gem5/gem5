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

#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "base/gtest/cur_tick_fake.hh"
#include "base/gtest/logging.hh"
#include "base/named.hh"
#include "base/trace.hh"

using namespace gem5;

// In test SetGetLogger this logger will be assigned to be the one returned
// by Tracer::getDebugLogger(). All tests before that test should assume
// that getDebugLogger() returns a cerr-based logger, and all tests after
// that test should assume that this logger is returned
std::stringstream ss;
trace::OstreamLogger main_logger(ss);

// Instantiate the mock class to have a valid curTick of 0
GTestTickHandler tickHandler;

namespace gem5
{
namespace debug
{
/** Debug flag used for the tests in this file. */
SimpleFlag TraceTestDebugFlag("TraceTestDebugFlag",
    "Exclusive debug flag for the trace tests");
} // namespace debug
} // namespace gem5

/** @return The ostream as a std::string. */
std::string
getString(std::ostream &os)
{
    auto buf = os.rdbuf();
    std::ostringstream oss;
    oss << buf;
    return oss.str();
}

/** @return The logger's ostream as a std::string. */
std::string
getString(trace::Logger *logger)
{
    return getString(logger->getOstream());
}

/** Test creating a simple log message. */
TEST(TraceTest, LogSimpleMessage)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    logger.logMessage(Tick(100), "", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Test message");
}

/** Test creating a simple log message with a name. */
TEST(TraceTest, LogMessageName)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    logger.logMessage(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
}

/** Test that when the tick is set to MaxTick it is not printed. */
TEST(TraceTest, LogMessageMaxTick)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    logger.logMessage(MaxTick, "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "Foo: Test message");
}

/** Test that by default the flag is not printed. */
TEST(TraceTest, LogMessageFlagDisabled)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    logger.logMessage(Tick(100), "Foo", "Bar", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
}

/**
 * Test creating a simple log message without tick information and that
 * enabling and disabling format flags work.
 */
TEST(TraceTest, LogMessageTickDisabledAndEnableDisable)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    logger.logMessage(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");

    trace::enable();
    EXPECT_TRUE(debug::changeFlag("FmtTicksOff", true));

    logger.logMessage(Tick(200), "Foo", "", "Test message");
#if TRACING_ON
    ASSERT_EQ(getString(&logger), "Foo: Test message");
#else
    ASSERT_EQ(getString(&logger), "    200: Foo: Test message");
#endif

    debug::changeFlag("FmtTicksOff", false);
    trace::disable();

    logger.logMessage(Tick(300), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "    300: Foo: Test message");
}

/**
 * Test creating a full log message to verify expected order of appearance.
 * This does not include the backtrace.
 */
TEST(TraceTest, LogMessageFlagEnabled)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);
    trace::enable();
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));

    logger.logMessage(Tick(100), "Foo", "Bar", "Test message");
#if TRACING_ON
    ASSERT_EQ(getString(&logger), "    100: Bar: Foo: Test message");
#else
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
#endif

    debug::changeFlag("FmtFlag", false);
    trace::disable();
}

/** Test that log messages are not displayed for ignored objects (single). */
TEST(TraceTest, LogMessageIgnoreOne)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    ObjectMatch ignore_foo("Foo");
    ObjectMatch ignore_bar("Bar");

    // Ignore foo
    logger.setIgnore(ignore_foo);
    logger.logMessage(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "");
    logger.logMessage(Tick(100), "Bar", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Bar: Test message");

    // Make sure that when setting a new ignore the old ignores are not kept
    logger.setIgnore(ignore_bar);
    logger.logMessage(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
    logger.logMessage(Tick(100), "Bar", "", "Test message");
    ASSERT_EQ(getString(&logger), "");
}

/** Test that log messages are not displayed for ignored objects (multiple). */
TEST(TraceTest, LogMessageIgnoreMultiple)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    ObjectMatch ignore_foo("Foo");
    ObjectMatch ignore_bar("Bar");
    ObjectMatch ignore_thy("Thy");

    // Ignore foo and bar
    logger.setIgnore(ignore_foo);
    logger.addIgnore(ignore_bar);
    logger.logMessage(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "");
    logger.logMessage(Tick(100), "Bar", "", "Test message");
    ASSERT_EQ(getString(&logger), "");
    logger.logMessage(Tick(100), "Thy", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Thy: Test message");

    // Make sure that when setting a new ignore, the old sub-ignores
    // are not kept
    logger.setIgnore(ignore_thy);
    logger.logMessage(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
    logger.logMessage(Tick(100), "Bar", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Bar: Test message");
    logger.logMessage(Tick(100), "Thy", "", "Test message");
    ASSERT_EQ(getString(&logger), "");
}

/** Test that dumping for an ignored name does not log anything. */
TEST(TraceTest, DumpIgnored)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    ObjectMatch ignore_foo("Foo");
    logger.setIgnore(ignore_foo);
    std::string message = "Test message";
    logger.dump(Tick(100), "Foo", message.c_str(), message.size(), "");
    ASSERT_EQ(getString(&logger), "");
}

/**
 * Test that a regular dump will change the message so that it contains both
 * the hex and char representation of every byte. There is a space between
 * every 8 bytes. For each 16-byte line of a message the order of appearance is
 *   <byte number> <2 spaces> <8 spaced bytes> <1 space> <8 spaced bytes>
 *   <3-spaces missing to fill 16 bytes> <1 space> <16 bytes as chars> <\n>
 */
TEST(TraceTest, DumpSimple)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    trace::enable();
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
    std::string message = "Test message";
    logger.dump(Tick(100), "Foo", message.c_str(), message.size(), "Bar");
#if TRACING_ON
    ASSERT_EQ(getString(&logger),
        // logMessage prefix
        "    100: Bar: Foo: "
        // Byte number + 2 spaces
        "00000000  "
        // 8 bytes + 1 space + 4 bytes + ((16 - 12) * 3) spaces
        "54 65 73 74 20 6d 65 73  73 61 67 65              "
        // 1 space + 12 chars + \n
        " Test message\n");
#else
    ASSERT_EQ(getString(&logger),
        // logMessage prefix
        "    100: Foo: "
        // Byte number + 2 spaces
        "00000000  "
        // 8 bytes + 1 space + 4 bytes + ((16 - 12) * 3) spaces
        "54 65 73 74 20 6d 65 73  73 61 67 65              "
        // 1 space + 12 chars + \n
        " Test message\n");
#endif
    debug::changeFlag("FmtFlag", false);
    trace::disable();
}

/**
 * Test that a regular dump will change the message so that it contains both
 * the hex and char representation of every byte, for a multi-line message.
 */
TEST(TraceTest, DumpMultiLine)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    std::string message =
        "This is a very long line that will span over multiple lines";
    logger.dump(Tick(100), "Foo", message.c_str(), message.size(), "");
    ASSERT_EQ(getString(&logger),
        "    100: Foo: 00000000  "
        "54 68 69 73 20 69 73 20  61 20 76 65 72 79 20 6c   This is a very l\n"
        "    100: Foo: 00000010  "
        "6f 6e 67 20 6c 69 6e 65  20 74 68 61 74 20 77 69   ong line that wi\n"
        "    100: Foo: 00000020  "
        "6c 6c 20 73 70 61 6e 20  6f 76 65 72 20 6d 75 6c   ll span over mul\n"
        "    100: Foo: 00000030  "
        "74 69 70 6c 65 20 6c 69  6e 65 73                  tiple lines\n");
}

/**
 * Test that when no logger exists a logger is created redirecting to cerr.
 * This is the only test that uses cerr. All other test will use main_logger.
 */
TEST(TraceTest, DISABLED_GetNullLogger)
{
    trace::Logger *logger = trace::getDebugLogger();
    ASSERT_FALSE(logger == nullptr);

    gtestLogOutput.str("");
    logger->logMessage(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(gtestLogOutput.str(), "    100: Foo: Test message");
}

/** Test that the logger is set properly. */
TEST(TraceTest, SetGetLogger)
{
    // NOTE: From now on getDebugLogger will use main_logger to avoid
    // having to check cerr. This assumes that tests are run in the order
    // they appear from line 1 to the last line of this file.
    trace::setDebugLogger(&main_logger);

    // Set message with local variable, and retrieve the string with
    // the debug-logger getter
    main_logger.logMessage(Tick(100), "Foo", "", "Test message");
    auto logger_from_getter = trace::getDebugLogger();
    ASSERT_EQ(getString(logger_from_getter), "    100: Foo: Test message");
}

/** Test that output() gets the ostream of the current debug logger. */
TEST(TraceTest, Output)
{
    trace::getDebugLogger()->logMessage(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(trace::output()), "    100: Foo: Test message");
}

/** Test dprintf_flag with ignored name. */
TEST(TraceTest, DprintfFlagIgnore)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    ObjectMatch ignore_foo("Foo");
    logger.setIgnore(ignore_foo);
    logger.dprintf_flag(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "");
}

/** Test dprintf_flag with zero args. */
TEST(TraceTest, DprintfFlagZeroArgs)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    logger.dprintf_flag(Tick(100), "Foo", "", "Test message");
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
}

/** Test dprintf_flag with one arg. */
TEST(TraceTest, DprintfFlagOneArg)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    logger.dprintf_flag(Tick(100), "Foo", "", "Test %s", "message");
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
}

/** Test dprintf_flag with multiple args. */
TEST(TraceTest, DprintfFlagMultipleArgs)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    logger.dprintf_flag(Tick(100), "Foo", "", "Test %s %c %d %x",
        "message", 'A', 217, 0x30);
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message A 217 30");
}

/** Test dprintf_flag with flag. */
TEST(TraceTest, DprintfFlagEnabled)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    trace::enable();
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
    logger.dprintf_flag(Tick(100), "Foo", "Bar", "Test %s", "message");
#if TRACING_ON
    ASSERT_EQ(getString(&logger), "    100: Bar: Foo: Test message");
#else
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
#endif
    debug::changeFlag("FmtFlag", false);
    trace::disable();
}

/** Test dprintf with ignored name. */
TEST(TraceTest, DprintfIgnore)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    ObjectMatch ignore_foo("Foo");
    logger.setIgnore(ignore_foo);
    logger.dprintf(Tick(100), "Foo", "Test message");
    ASSERT_EQ(getString(&logger), "");
}

/** Test that dprintf does not have a flag. */
TEST(TraceTest, DprintfEnabled)
{
    std::stringstream ss;
    trace::OstreamLogger logger(ss);

    trace::enable();
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
    logger.dprintf(Tick(100), "Foo", "Test %s", "message");
    ASSERT_EQ(getString(&logger), "    100: Foo: Test message");
    debug::changeFlag("FmtFlag", false);
    trace::disable();
}

/** Test that dprintf is just a flagless wrapper for dprintf_flag. */
TEST(TraceTest, DprintfWrapper)
{
    std::stringstream ss, ss_flag;
    trace::OstreamLogger logger(ss);
    trace::OstreamLogger logger_flag(ss_flag);

    logger.dprintf(Tick(100), "Foo", "Test %s %c %d %x",
        "message", 'A', 217, 0x30);
    logger_flag.dprintf_flag(Tick(100), "Foo", "", "Test %s %c %d %x",
        "message", 'A', 217, 0x30);
    ASSERT_EQ(getString(&logger), getString(&logger_flag));
}

/** Test DDUMP with tracing on. */
TEST(TraceTest, MacroDDUMP)
{
    StringWrap name("Foo");
    std::string message = "Test message";

    // Flag enabled
    trace::enable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", true));
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
    DDUMP(TraceTestDebugFlag, message.c_str(), message.size());
#if TRACING_ON
    ASSERT_EQ(getString(trace::output()),
        "      0: TraceTestDebugFlag: Foo: 00000000  "
        "54 65 73 74 20 6d 65 73  73 61 67 65               Test message\n");
#else
    ASSERT_EQ(getString(trace::output()), "");
#endif

    // Flag disabled
    trace::disable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", false));
    DDUMP(TraceTestDebugFlag, message.c_str(), message.size());
    ASSERT_EQ(getString(trace::output()), "");
}

/** Test DPRINTF with tracing on. */
TEST(TraceTest, MacroDPRINTF)
{
    StringWrap name("Foo");

    // Flag enabled
    trace::enable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", true));
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
    DPRINTF(TraceTestDebugFlag, "Test message");
#if TRACING_ON
    ASSERT_EQ(getString(trace::output()),
        "      0: TraceTestDebugFlag: Foo: Test message");
#else
    ASSERT_EQ(getString(trace::output()), "");
#endif

    // Flag disabled
    trace::disable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", false));
    DPRINTF(TraceTestDebugFlag, "Test message");
    ASSERT_EQ(getString(trace::output()), "");
}

/** Test DPRINTFS with tracing on. */
TEST(TraceTest, MacroDPRINTFS)
{
    Named named("Foo");
#if TRACING_ON
    Named *named_ptr = &named;
#endif

    // Flag enabled
    trace::enable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", true));
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
#if TRACING_ON
    DPRINTFS(TraceTestDebugFlag, named_ptr, "Test message");
    ASSERT_EQ(getString(trace::output()),
        "      0: TraceTestDebugFlag: Foo: Test message");
#endif

    // Flag disabled
    trace::disable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", false));
#if TRACING_ON
    DPRINTFS(TraceTestDebugFlag, named_ptr, "Test message");
    ASSERT_EQ(getString(trace::output()), "");
#endif
}

/** Test DPRINTFR with tracing on. */
TEST(TraceTest, MacroDPRINTFR)
{
    // Flag enabled
    trace::enable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", true));
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
    DPRINTFR(TraceTestDebugFlag, "Test message");
#if TRACING_ON
    ASSERT_EQ(getString(trace::output()), "TraceTestDebugFlag: Test message");
#else
    ASSERT_EQ(getString(trace::output()), "");
#endif

    // Flag disabled
    trace::disable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", false));
    DPRINTFR(TraceTestDebugFlag, "Test message");
    ASSERT_EQ(getString(trace::output()), "");
}

/** Test DPRINTFN with tracing on. */
TEST(TraceTest, MacroDPRINTFN)
{
    StringWrap name("Foo");
    DPRINTFN("Test message");
#if TRACING_ON
    ASSERT_EQ(getString(trace::output()), "      0: Foo: Test message");
#else
    ASSERT_EQ(getString(trace::output()), "");
#endif
}

/** Test DPRINTFNR with tracing on. */
TEST(TraceTest, MacroDPRINTFNR)
{
    DPRINTFNR("Test message");
#if TRACING_ON
    ASSERT_EQ(getString(trace::output()), "Test message");
#else
    ASSERT_EQ(getString(trace::output()), "");
#endif
}

/** Test DPRINTF_UNCONDITIONAL with tracing on. */
TEST(TraceTest, MacroDPRINTF_UNCONDITIONAL)
{
    StringWrap name("Foo");

    // Flag enabled
    trace::enable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", true));
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
    DPRINTF_UNCONDITIONAL(TraceTestDebugFlag, "Test message");
#if TRACING_ON
    ASSERT_EQ(getString(trace::output()),
        "      0: TraceTestDebugFlag: Foo: Test message");
#else
    ASSERT_EQ(getString(trace::output()), "");
#endif

    // Flag disabled
    trace::disable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", false));
    DPRINTF_UNCONDITIONAL(TraceTestDebugFlag, "Test message");
#if TRACING_ON
    ASSERT_EQ(getString(trace::output()), "      0: Foo: Test message");
#else
    ASSERT_EQ(getString(trace::output()), "");
#endif
}

/**
 * Test that there is a global name() to fall through when a locally scoped
 * name() is not defined.
 */
TEST(TraceTest, GlobalName)
{
    // Flag enabled
    trace::enable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", true));
    EXPECT_TRUE(debug::changeFlag("FmtFlag", true));
    DPRINTF(TraceTestDebugFlag, "Test message");
#if TRACING_ON
    ASSERT_EQ(getString(trace::output()),
        "      0: TraceTestDebugFlag: global: Test message");
#else
    ASSERT_EQ(getString(trace::output()), "");
#endif

    // Flag disabled
    trace::disable();
    EXPECT_TRUE(debug::changeFlag("TraceTestDebugFlag", false));
    DPRINTF(TraceTestDebugFlag, "Test message");
    ASSERT_EQ(getString(trace::output()), "");
}

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
#include <gtest/gtest.h>

#include "base/gtest/logging.hh"
#include "base/logging.hh"

using namespace gem5;

/** Temporarily redirects cerr to gtestLogOutput. */
class LoggingFixture : public ::testing::Test
{
  public:
    void SetUp() override { old = std::cerr.rdbuf(gtestLogOutput.rdbuf()); }
    void TearDown() override { std::cerr.rdbuf(old); }

  private:
    /** Used to restore cerr's streambuf. */
    std::streambuf *old;
};

/** Test the most basic print. */
TEST_F(LoggingFixture, BasicPrint)
{
    Logger logger("test: ");

    // Tests that the logger will automatically add '\n' to the end of the
    // message if it does not already have '\n' in the end
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: test: message\n");

    // Tests that the logger won't add '\n' if it already has '\n' in the end
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message\n");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: test: message\n");

    // Tests that the logger won't remove '\n's if multiple are provided
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "sample message\n\n");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: test: sample message\n\n");

    // A more complete/complex test case
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "sample message\nwith \n3 lines");
    ASSERT_EQ(gtestLogOutput.str(),
        "File:10: test: sample message\nwith \n3 lines\n");
}

/** Test the variadic-arg print for chars. */
TEST_F(LoggingFixture, VariadicCharPrint)
{
    Logger logger("test: ");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), (const char *)"%s message",
        "sample");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: test: sample message\n");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10),
        (const char *)"sample %s with %d arguments\n", "message", 2);
    ASSERT_EQ(gtestLogOutput.str(),
        "File:10: test: sample message with 2 arguments\n");
}

/** Test the variadic-arg print for strings. */
TEST_F(LoggingFixture, VariadicStringPrint)
{
    Logger logger("test: ");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), std::string("%s message"), "sample");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: test: sample message\n");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10),
        std::string("sample %s with %d arguments\n"), "message", 2);
    ASSERT_EQ(gtestLogOutput.str(),
        "File:10: test: sample message with 2 arguments\n");
}

/** Test the variadic-arg print for chars with arguments missing. */
TEST_F(LoggingFixture, VariadicCharMissingPrint)
{
    Logger logger("test: ");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), (const char *)"%s message");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: test: <extra arg>% message\n");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), (const char *)"%s mes%ca%cge",
        "sample", 's');
    ASSERT_EQ(gtestLogOutput.str(),
        "File:10: test: sample messa<extra arg>%ge\n");
}

/** Test the variadic-arg print for strings with arguments missing. */
TEST_F(LoggingFixture, VariadicStringMissingPrint)
{
    Logger logger("test: ");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), std::string("%s message"));
    ASSERT_EQ(gtestLogOutput.str(), "File:10: test: <extra arg>% message\n");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), std::string("%s mes%ca%cge"),
        "sample", 's');
    ASSERT_EQ(gtestLogOutput.str(),
        "File:10: test: sample messa<extra arg>%ge\n");
}

/** Test that no message is shown when printing with a disabled logger. */
TEST_F(LoggingFixture, DisabledPrint)
{
    class DisabledLogger : public Logger
    {
      public:
        DisabledLogger(const char *prefix)
          : Logger(prefix)
        {
            enabled = false;
        }
    } logger("test: ");

    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "");
}

/** Test printing with the warn logger, enabled and disabled. */
TEST_F(LoggingFixture, WarnLoggerPrint)
{
    Logger::setLevel(Logger::WARN);
    Logger &logger = Logger::getWarn();
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: warn: message\n");

    // PANIC does not include WARN
    Logger::setLevel(Logger::PANIC);
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "");

    Logger::setLevel(Logger::NUM_LOG_LEVELS);
}

/** Test printing with the info logger, enabled and disabled. */
TEST_F(LoggingFixture, InfoLoggerPrint)
{
    Logger::setLevel(Logger::INFO);
    Logger &logger = Logger::getInfo();
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: info: message\n");

    // PANIC does not include INFO
    Logger::setLevel(Logger::PANIC);
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "");

    Logger::setLevel(Logger::NUM_LOG_LEVELS);
}

/** Test printing with the hack logger, enabled and disabled. */
TEST_F(LoggingFixture, HackLoggerPrint)
{
    Logger::setLevel(Logger::HACK);
    Logger &logger = Logger::getHack();
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "File:10: hack: message\n");

    // PANIC does not include HACK
    Logger::setLevel(Logger::PANIC);
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "");

    Logger::setLevel(Logger::NUM_LOG_LEVELS);
}

/** Test printing with the fatal logger, enabled and disabled. */
TEST_F(LoggingFixture, FatalLoggerPrint)
{
    Logger &logger = Logger::getFatal();

    // The actual value of memory usage is not relevant
    Logger::setLevel(Logger::FATAL);
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_THAT(gtestLogOutput.str(),
        testing::HasSubstr("File:10: fatal: message\nMemory Usage:"));

    // PANIC does not include FATAL
    Logger::setLevel(Logger::PANIC);
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_EQ(gtestLogOutput.str(), "");

    Logger::setLevel(Logger::NUM_LOG_LEVELS);
}

/** Test printing with the panic logger, which cannot be disabled. */
TEST_F(LoggingFixture, PanicLoggerPrint)
{
    // The actual value of memory usage is not relevant
    Logger::setLevel(Logger::PANIC);
    Logger &logger = Logger::getPanic();
    gtestLogOutput.str("");
    logger.print(Logger::Loc("File", 10), "message");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("File:10: panic: message\nMemory Usage:"));

    Logger::setLevel(Logger::NUM_LOG_LEVELS);
}

/** Test macro base_message. */
TEST_F(LoggingFixture, BaseMessage)
{
    Logger logger("test: ");

    // The logger will automatically add '\n' to the end of the message
    // if it does not already have at least one.
    gtestLogOutput.str("");
    base_message(logger, "message");
    ASSERT_THAT(gtestLogOutput.str(), ::testing::HasSubstr("test: message\n"));

    gtestLogOutput.str("");
    base_message(logger, "message\n");
    ASSERT_THAT(gtestLogOutput.str(), ::testing::HasSubstr("test: message\n"));

    gtestLogOutput.str("");
    base_message(logger, "sample message\n\n");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("test: sample message\n\n"));

    gtestLogOutput.str("");
    base_message(logger, "sample message\nwith \n3 lines");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("test: sample message\nwith \n3 lines\n"));

    gtestLogOutput.str("");
    base_message(logger, "sample %s with %d arguments\n", "message", 2);
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("test: sample message with 2 arguments\n"));
}

/** Test that base_message_once only prints the message once in a loop. */
TEST_F(LoggingFixture, BaseMessageOnce)
{
    Logger logger("test: ");

    for (int i = 0; i < 10; i++) {
        gtestLogOutput.str("");
        base_message_once(logger, "message\n");
        if (i == 0) {
            ASSERT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("test: message\n"));
        } else {
            ASSERT_EQ(gtestLogOutput.str(), "");
        }
    }
}

/** Test macro warn. */
TEST_F(LoggingFixture, Warn)
{
    // The logger will automatically add '\n' to the end of the message
    // if it does not already have at least one.
    gtestLogOutput.str("");
    warn("message");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("warn: message\n"));

    gtestLogOutput.str("");
    warn("message\n");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("warn: message\n"));

    gtestLogOutput.str("");
    warn("sample message\n\n");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("warn: sample message\n\n"));

    gtestLogOutput.str("");
    warn("sample message\nwith \n3 lines");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("warn: sample message\nwith \n3 lines\n"));

    gtestLogOutput.str("");
    warn("sample %s with %d arguments\n", "message", 2);
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("warn: sample message with 2 arguments\n"));
}

/** Test macro inform. */
TEST_F(LoggingFixture, Inform)
{
    // The logger will automatically add '\n' to the end of the message
    // if it does not already have at least one.
    gtestLogOutput.str("");
    inform("message");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("info: message\n"));

    gtestLogOutput.str("");
    inform("message\n");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("info: message\n"));

    gtestLogOutput.str("");
    inform("sample message\n\n");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("info: sample message\n\n"));

    gtestLogOutput.str("");
    inform("sample message\nwith \n3 lines");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("info: sample message\nwith \n3 lines\n"));

    gtestLogOutput.str("");
    inform("sample %s with %d arguments\n", "message", 2);
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("info: sample message with 2 arguments\n"));
}

/** Test macro hack. */
TEST_F(LoggingFixture, Hack)
{
    // The logger will automatically add '\n' to the end of the message
    // if it does not already have at least one.
    gtestLogOutput.str("");
    hack("message");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("hack: message\n"));

    gtestLogOutput.str("");
    hack("message\n");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("hack: message\n"));

    gtestLogOutput.str("");
    hack("sample message\n\n");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("hack: sample message\n\n"));

    gtestLogOutput.str("");
    hack("sample message\nwith \n3 lines");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("hack: sample message\nwith \n3 lines\n"));

    gtestLogOutput.str("");
    hack("sample %s with %d arguments\n", "message", 2);
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("hack: sample message with 2 arguments\n"));
}

/** Test that warn_once only prints the message once in a loop. */
TEST_F(LoggingFixture, WarnOnce)
{
    for (int i = 0; i < 10; i++) {
        gtestLogOutput.str("");
        warn_once("message\n");
        if (i == 0) {
            ASSERT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("warn: message\n"));
        } else {
            ASSERT_EQ(gtestLogOutput.str(), "");
        }
    }
}

/** Test that inform_once only prints the message once in a loop. */
TEST_F(LoggingFixture, InformOnce)
{
    for (int i = 0; i < 10; i++) {
        gtestLogOutput.str("");
        inform_once("message\n");
        if (i == 0) {
            ASSERT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("info: message\n"));
        } else {
            ASSERT_EQ(gtestLogOutput.str(), "");
        }
    }
}

/** Test that hack_once only prints the message once in a loop. */
TEST_F(LoggingFixture, HackOnce)
{
    for (int i = 0; i < 10; i++) {
        gtestLogOutput.str("");
        hack_once("message\n");
        if (i == 0) {
            ASSERT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("hack: message\n"));
        } else {
            ASSERT_EQ(gtestLogOutput.str(), "");
        }
    }
}

/** Test that warn_if only prints the message when the condition is true. */
TEST_F(LoggingFixture, WarnIf)
{
    gtestLogOutput.str("");
    warn_if(true, "message\n");
    ASSERT_THAT(gtestLogOutput.str(),
        ::testing::HasSubstr("warn: message\n"));

    gtestLogOutput.str("");
    warn_if(false, "message\n");
    ASSERT_EQ(gtestLogOutput.str(), "");
}

/** Test that warn_if_once only prints the message once in a loop. */
TEST_F(LoggingFixture, WarnIfOnce)
{
    for (int i = 0; i < 10; i++) {
        gtestLogOutput.str("");
        warn_if_once(i == 3, "message\n");
        if (i == 3) {
            ASSERT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("warn: message\n"));
        } else {
            ASSERT_EQ(gtestLogOutput.str(), "");
        }
    }
}

/** Test that a logger cannot be created with an empty prefix. */
TEST(LoggingDeathTest, EmptyPrefix)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif
    ASSERT_DEATH(Logger(nullptr), "");
}

/** Test that the test logger's exit helper will end execution gracefully. */
TEST(LoggingDeathTest, ExitHelper)
{
    ASSERT_DEATH(Logger("test: ").exit_helper(), "");
}

/** Test that the warn logger's exit helper will end execution gracefully. */
TEST(LoggingDeathTest, WarnLoggerExitHelper)
{
    ASSERT_DEATH(Logger::getWarn().exit_helper(), "");
}

/** Test that the info logger's exit helper will end execution gracefully. */
TEST(LoggingDeathTest, InfoLoggerExitHelper)
{
    ASSERT_DEATH(Logger::getInfo().exit_helper(), "");
}

/** Test that the hack logger's exit helper will end execution gracefully. */
TEST(LoggingDeathTest, HackLoggerExitHelper)
{
    ASSERT_DEATH(Logger::getHack().exit_helper(), "");
}

/** Test that the fatal logger's exit helper will end execution with error. */
TEST(LoggingDeathTest, FatalLoggerExitHelper)
{
    ASSERT_DEATH(Logger::getFatal().exit_helper(), "");
}

/** Test that the panic logger's exit helper will end execution with error. */
TEST(LoggingDeathTest, PanicLoggerExitHelper)
{
    ASSERT_DEATH(Logger::getPanic().exit_helper(), "");
}

/** Test that exit_message prints a message and exits. */
TEST(LoggingDeathTest, ExitMessage)
{
    Logger logger("test: ");
    ASSERT_DEATH(exit_message(logger, "message\n"), "test: message\n");
}

/** Test macro panic. */
TEST(LoggingDeathTest, Panic)
{
    ASSERT_DEATH(panic("message\n"),
        ::testing::HasSubstr("panic: message\nMemory Usage:"));
}

/** Test macro fatal. */
TEST(LoggingDeathTest, Fatal)
{
    ASSERT_DEATH(fatal("message\n"),
        ::testing::HasSubstr("fatal: message\nMemory Usage:"));
}

/** Test that panic_if only prints the message when the condition is true. */
TEST(LoggingDeathTest, PanicIf)
{
    panic_if(false, "No death");
    ASSERT_DEATH(panic_if(true, "message\n"), ::testing::HasSubstr(
        "panic: panic condition true occurred: message\nMemory Usage:"));
}

/** Test that fatal_if only prints the message when the condition is true. */
TEST(LoggingDeathTest, FatalIf)
{
    fatal_if(false, "No death");
    ASSERT_DEATH(fatal_if(true, "message\n"), ::testing::HasSubstr(
        "fatal: fatal condition true occurred: message\nMemory Usage:"));
}

/** Test macro gem5_assert. */
TEST(LoggingDeathTest, gem5Assert)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif
    gem5_assert(true, "message\n");
    ASSERT_DEATH(gem5_assert(false, "message\n"), ::testing::HasSubstr(
        "panic: assert(false) failed: message\nMemory Usage:"));
    ASSERT_DEATH(gem5_assert(false, "%s, %s!\n", "Hello", "World"),
        ::testing::HasSubstr(
        "panic: assert(false) failed: Hello, World!\nMemory Usage:"));
    gem5_assert(true);
    ASSERT_DEATH(gem5_assert(false), ::testing::HasSubstr(
        "panic: assert(false) failed\nMemory Usage:"));
}

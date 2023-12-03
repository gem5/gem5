/*
 * Copyright 2021 Google Inc.
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

namespace gem5
{

// This custom exception type will help prevent fatal exceptions from being
// caught by other code in gem5 and let them escape to the gtest framework.
// Unfortunately that results in a somewhat confusing message about an unknown
// exception being thrown after the panic/fatal message has been printed, but
// there will at least be some indication what went wrong.
struct GTestException
{
};

class GTestLogOutput : public std::ostringstream
{
  private:
    class EventHook : public ::testing::EmptyTestEventListener
    {
      private:
        GTestLogOutput &stream;

      public:
        EventHook(GTestLogOutput &_stream);
        ~EventHook();

        void OnTestStart(const ::testing::TestInfo &test_info) override;
    };

    EventHook eventHook;

  public:
    template <typename... Args>
    GTestLogOutput(Args &&...args)
        : std::ostringstream(std::forward<Args>(args)...), eventHook(*this)
    {}
};

extern thread_local GTestLogOutput gtestLogOutput;

} // namespace gem5

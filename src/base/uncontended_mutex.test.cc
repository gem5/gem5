/*
 * Copyright 2020 Google, Inc.
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
#include <mutex>
#include <thread>
#include <vector>

#include "base/uncontended_mutex.hh"

using namespace gem5;

TEST(UncontendedMutex, Lock)
{
    int data = 0;
    UncontendedMutex m;

    std::thread t1([&]() {
        std::lock_guard<UncontendedMutex> g(m);
        // Simulate += operation with a racing change between read and write.
        int tmp = data;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        data = tmp + 1;
    });

    std::thread t2([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::lock_guard<UncontendedMutex> g(m);
        data = data + 1;
    });

    std::thread t3([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::lock_guard<UncontendedMutex> g(m);
        data = data + 1;
    });
    t1.join();
    t2.join();
    t3.join();

    EXPECT_EQ(data, 3);
}

TEST(UncontendedMutex, HeavyContention)
{
    int num_of_iter = 1000;
    int num_of_thread = 1000;
    std::vector<std::thread> threads;

    int data = 0;
    UncontendedMutex m;

    for (int t = 0; t < num_of_thread; ++t) {
        threads.emplace_back([&]() {
            for (int k = 0; k < num_of_iter; ++k) {
                std::lock_guard<UncontendedMutex> g(m);
                data++;
            }
        });
    }

    for (auto &t : threads) {
        t.join();
    }
    EXPECT_EQ(data, num_of_iter * num_of_thread);
}

TEST(UncontendedMutex, SingleThread)
{
    int num_of_iter = 100000000;

    UncontendedMutex m;
    int data = 0;
    for (int i = 0; i < num_of_iter; i++) {
        std::lock_guard<UncontendedMutex> g(m);
        ++data;
    }
    EXPECT_EQ(data, num_of_iter);
}

/*
 * FairUncontendedMutex solves the barging issue.
 * This test makes sure the lock is distributed fair enough.
 */
TEST(FairUncontendedMutex, NoBarging)
{
    int num_of_thread = 10;
    int total_work = 10000;
    int remaining_work = total_work;
    std::vector<int> workload(num_of_thread);
    std::vector<std::thread> threads;

    std::atomic<int> ready_threads = 0;
    FairUncontendedMutex m;

    for (int t = 0; t < num_of_thread; ++t) {
        threads.emplace_back([&, t]() {
            ++ready_threads;
            while (ready_threads.load() < num_of_thread) {
                std::this_thread::yield();
            }
            while (true) {
                std::scoped_lock<FairUncontendedMutex> g(m);

                if (remaining_work <= 0) {
                    break;
                }
                --remaining_work;
                ++workload[t];
            }
        });
    }

    for (auto &t : threads) {
        t.join();
    }

    int minimum_workload = total_work / num_of_thread / 2;
    EXPECT_EQ(remaining_work, 0);
    for (int t = 0; t < num_of_thread; ++t) {
        EXPECT_GT(workload[t], minimum_workload)
            << "Thread " << t << " did not get enough workload.";
    }
}

TEST(FairUncontendedMutex, SingleThread)
{
    int num_of_iter = 100000000;

    FairUncontendedMutex m;
    int data = 0;
    for (int i = 0; i < num_of_iter; i++) {
        std::lock_guard<FairUncontendedMutex> g(m);
        ++data;
    }
    EXPECT_EQ(data, num_of_iter);
}

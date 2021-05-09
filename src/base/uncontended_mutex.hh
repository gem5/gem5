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

#ifndef __BASE_UNCONTENDED_MUTEX_HH__
#define __BASE_UNCONTENDED_MUTEX_HH__

#include <atomic>
#include <condition_variable>
#include <mutex>

namespace gem5
{

/*
 * The std::mutex implementation is slower than expected because of many mode
 * checking and legacy support.
 *
 * The UncontendedMutex uses an atomic flag to check if we really need to
 * obtain a mutex lock. For most cases without multi-threads event queues,
 * e.g. non-KVM simulation, this avoid the usage of mutex and speed up the
 * simulation.
 */
class UncontendedMutex
{
  private:
    /*
     * A flag to record the current status:
     * 0: no one has the lock
     * 1: exactly one thread has the lock
     * >1: one or more threads are waiting for the lock
     */
    std::atomic<int> flag;
    std::mutex m;
    std::condition_variable cv;

    bool
    testAndSet(int expected, int desired)
    {
        return flag.compare_exchange_strong(expected, desired);
    }

  public:
    UncontendedMutex() : flag(0) {}

    void
    lock()
    {
        /*
         * Here we use 'flag' to check if we are the first thread to get the
         * lock. If not, we try to obtain the real mutex, and use the condition
         * variable to wait for the thread who has the lock to release it.
         *
         * The flag will be updated to more than 1, so the thread with lock
         * knows that there is another thread waiting for the lock.
         */
        while (!testAndSet(0, 1)) {
            std::unique_lock<std::mutex> ul(m);
            /*
             * It is possible that just before we obtain the mutex lock, the
             * first thread releases the flag and thus flag becomes zero. In
             * such case, we shouldn't wait for the condition variable because
             * there is no the other thread to notify us.
             */
            if (flag++ == 0)
                break;
            cv.wait(ul);
        }
    }

    void
    unlock()
    {
        /* In case there are no other threads waiting, we will just clear the
         * flag and return.
         */
        if (testAndSet(1, 0))
            return;

        /*
         * Otherwise, clear the flag and notify all the waiting threads. We
         * need to protect the flag by mutex here so that there won't be
         * another thread waiting but the flag is already set to 0.
         */
        {
            std::lock_guard<std::mutex> g(m);
            flag = 0;
        }
        /*
         * It's possible to update the algorithm and use notify_one() here.
         * However, tests show that notify_one() is much slower than
         * notify_all() in this case. Here we choose to use notify_all().
         */
        cv.notify_all();
    }
};

} // namespace gem5

#endif // __BASE_UNCONTENDED_MUTEX_HH__

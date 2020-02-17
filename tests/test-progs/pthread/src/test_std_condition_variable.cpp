/*
  * Copyright (c) 2018, Cornell University
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or
  * without modification, are permitted provided that the following
  * conditions are met:
  *
  * Redistributions of source code must retain the above copyright
  * notice, this list of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above
  * copyright notice, this list of conditions and the following
  * disclaimer in the documentation and/or other materials provided
  * with the distribution.
  *
  * Neither the name of Cornell University nor the names of its
  * contributors may be used to endorse or promote products derived
  * from this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
  * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  */

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

//------------------------------------------------------------------------
// Test std::condition_variable
//------------------------------------------------------------------------
// The master thread creates N threads, each of which waits on a
// condition variable of a signal to start. The master thread then set
// the signal and notifies all other threads to begin.

#define MAX_N_WORKER_THREADS 10

std::mutex mtx;
std::condition_variable cv;
bool ready = false;

void print_id( size_t id )
{
    std::unique_lock<std::mutex> lck(mtx);
    while (!ready)
        cv.wait(lck);
    // ...
    std::cout << "thread " << id << '\n';
}

void go()
{
    std::unique_lock<std::mutex> lck(mtx);
    ready = true;
    cv.notify_all();
}

int main( int argc, char* argv[] )
{
    size_t n_worker_threads = 0;

    std::vector< std::thread > threads;

    for ( size_t i = 0; i < MAX_N_WORKER_THREADS; i++ ) {
        try {
            threads.push_back( std::thread( print_id, i ) );
        } catch ( const std::system_error& err ) {
            break;
        }
        n_worker_threads++;
    }

    std::cout << n_worker_threads << " threads ready to race...\n";
    go();                       // go!

    for (int i = 0; i < n_worker_threads; ++i) {
        threads[i].join();
    }

    // if there is no timeout (i.e., threads are all waken up properly, this
    // test always succeeds)
    return EXIT_SUCCESS;
}

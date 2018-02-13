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
  *
  * Author: Moyang Wang
  */

#include <pthread.h>

#include <cstdlib>
#include <iostream>
#include <mutex>
#include <vector>

//------------------------------------------------------------------------
// Test pthread_cond
//------------------------------------------------------------------------
// The master thread creates N threads, each of which waits on a
// condition variable of a signal to start. The master thread then set
// the signal and notifies all other threads to begin.

#define MAX_N_WORKER_THREADS 10

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  cv = PTHREAD_COND_INITIALIZER;

bool ready = false;

void* print_id( void* arg_vptr )
{
    pthread_mutex_lock( &mutex );
    long id = (long)arg_vptr;

    while (!ready) {
        pthread_cond_wait( &cv, &mutex );
    }
    // ...
    std::cout << "thread " << id << '\n';

    pthread_mutex_unlock( &mutex );

    return nullptr;
}

void go()
{
    pthread_mutex_lock( &mutex );
    ready = true;
    pthread_cond_broadcast( &cv );
    pthread_mutex_unlock( &mutex );
}

int main( int argc, char* argv[] )
{
    size_t n_worker_threads = 0;

    std::vector< pthread_t > threads( MAX_N_WORKER_THREADS );

    int ret = 0;
    for ( size_t i = 0; i < MAX_N_WORKER_THREADS; i++ ){
        ret = pthread_create( &threads[i], nullptr, print_id, (void*)i );
        if (ret != 0) {
            break;
        }
        n_worker_threads++;
    }

    std::cout << n_worker_threads << " threads ready to race...\n";

    go();

    for ( size_t i = 1; i < n_worker_threads; i++ ) {
        pthread_join( threads[i], nullptr );
    }

    if (n_worker_threads < 1) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

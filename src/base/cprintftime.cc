/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
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

#include <unistd.h>

#include <csignal>
#include <iostream>
#include <list>
#include <sstream>
#include <string>

#include "base/cprintf.hh"

volatile int stop = false;

void
handle_alarm(int signal)
{
    stop = true;
}

void
do_test(int seconds)
{
    stop = false;
    alarm(seconds);
}

int
main()
{
    std::stringstream result;
    int iterations = 0;

    signal(SIGALRM, handle_alarm);

    do_test(10);
    while (!stop) {
        std::stringstream result;
        gem5::ccprintf(result,
                       "this is a %s of %d iterations %3.2f %p\n",
                       "test", iterations, 51.934, &result);

        iterations += 1;
    }

    gem5::cprintf(
            "completed %d iterations of ccprintf in 10s, %f iterations/s\n",
            iterations, iterations / 10.0);

    do_test(10);
    while (!stop) {
        char result[1024];
        int dummy;
        sprintf(result,
                 "this is a %s of %d iterations %3.2f %p\n",
                 "test", iterations, 51.934, &dummy);

        iterations += 1;
    }

    gem5::cprintf(
            "completed %d iterations of sprintf in 10s, %f iterations/s\n",
            iterations, iterations / 10.0);

    return 0;
}

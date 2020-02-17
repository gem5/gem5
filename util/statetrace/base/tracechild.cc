/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
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

#include <sys/ptrace.h>
#include <sys/wait.h>

#include <cerrno>
#include <cstring>
#include <iostream>

#include "tracechild.hh"

using namespace std;

bool
TraceChild::startTracing(const char * pathToFile, char * const argv[])
{
    instructions = 0;
    pid = fork();
    if (pid == -1) {
        cout << "fork failed" << endl;
        return false;
    } else if (pid == 0) {
        //We're the child. Get things ready and then exec the program to trace.
        //Let our parent trace us
        if (ptrace(PTRACE_TRACEME, 0, 0, 0) == -1) {
            cout << "Failure calling TRACEME\n" << strerror(errno) << endl;
            return false;
        }

        //Set up an empty environment for the child... We would want to
        //specify this somehow at some point
        char * env[] = {NULL};

        //Start the program to trace
        execve(pathToFile, argv, env);

        //We should never get here, so this is an error!
        cout << "Exec failed\n" <<  strerror(errno) << endl;
        return false;
    }

    //From this point forward, we know we're in the parent process.
    if (!doWait()) {
        cout << "Didn't wait successfully" << endl;
        return false;
    }
    tracing = true;
    return true;
}

bool
TraceChild::stopTracing()
{
    if (ptrace(PTRACE_KILL, pid, 0, 0) != 0)
        return false;
    tracing = false;
    return true;
}

bool
TraceChild::step()
{
    ptraceSingleStep();
}

bool
TraceChild::ptraceSingleStep()
{
    if (!tracing) {
        cout << "Not tracing!" << endl;
        return false;
    }
    if (ptrace(PTRACE_SINGLESTEP, pid, 0, 0) != 0) {
        switch (errno) {
          case EBUSY: cout << "EBUSY" << endl; break;
          case EFAULT: cout << "EFAULT" << endl; break;
          case EIO: cout << "EIO" << endl; break;
          case EPERM: cout << "EPERM" << endl; break;
          case ESRCH: cout << "ESRCH" << endl; break;
          default: cout << "Unknown error" << endl; break;
        }
        cout << "Not able to single step!" << endl;
        tracing = false;
        return false;
    }
    doWait();
    update(pid);
}

bool
TraceChild::doWait()
{
    int wait_val;
    wait(&wait_val);
    if (WIFEXITED(wait_val)) {
        cerr << "Program exited! Exit status is "
             << WEXITSTATUS(wait_val) << endl;
        cerr << "Executed " << instructions
             << " instructions." << endl;
        tracing = false;
        return false;
    }
    if (WIFSIGNALED(wait_val)) {
        if (WTERMSIG(wait_val))
            cerr << "Program terminated by signal "
                 << WTERMSIG(wait_val) << endl;
        if (WCOREDUMP(wait_val))
            cerr << "Program core dumped!" << endl;
        tracing = false;
        cerr << "Executed " << instructions
             << " instructions." << endl;
        return false;
    }
    if (WIFSTOPPED(wait_val) && WSTOPSIG(wait_val) != SIGTRAP) {
        cerr << "Program stopped by signal " << WSTOPSIG(wait_val) << endl;
        tracing = false;
        cerr << "Executed " << instructions << " instructions." << endl;
            return false;
    }
    return true;
}

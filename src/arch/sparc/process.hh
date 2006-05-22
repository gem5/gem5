/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#ifndef __SPARC_PROCESS_HH__
#define __SPARC_PROCESS_HH__

#include <string>
#include <vector>
#include "sim/process.hh"

class ObjectFile;
class System;

typedef struct
{
    int64_t a_type;
    union {
        int64_t a_val;
        Addr    a_ptr;
        Addr    a_fcn;
    };
} m5_auxv_t;

class SparcLiveProcess : public LiveProcess
{
  protected:

    static const Addr StackBias = 2047;

    std::vector<m5_auxv_t> auxv;

    SparcLiveProcess(const std::string &nm, ObjectFile *objFile,
                System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
                std::vector<std::string> &argv,
                std::vector<std::string> &envp);

    void startup();

  public:
    // this function is used to create the LiveProcess object, since
    // we can't tell which subclass of LiveProcess to use until we
    // open and look at the object file.
    static SparcLiveProcess *create(const std::string &nm,
                               System *_system,
                               int stdin_fd, int stdout_fd, int stderr_fd,
                               std::string executable,
                               std::vector<std::string> &argv,
                               std::vector<std::string> &envp);

    void argsInit(int intSize, int pageSize);

};

#endif // __SPARC_PROCESS_HH__

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

#include "arch/alpha/process.hh"
#include "arch/alpha/linux/process.hh"
#include "arch/alpha/tru64/process.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"

namespace AlphaISA
{

LiveProcess *
createProcess(const std::string &nm, ObjectFile * objFile, System *system,
        int stdin_fd, int stdout_fd, int stderr_fd,
        std::vector<std::string> &argv, std::vector<std::string> &envp)
{
    LiveProcess * process = NULL;
    if (objFile->getArch() != ObjectFile::Alpha)
        fatal("Object file does not match architecture.");
    switch (objFile->getOpSys()) {
      case ObjectFile::Tru64:
        process = new AlphaTru64Process(nm, objFile, system,
                                        stdin_fd, stdout_fd, stderr_fd,
                                        argv, envp);
        break;

      case ObjectFile::Linux:
        process = new AlphaLinuxProcess(nm, objFile, system,
                                        stdin_fd, stdout_fd, stderr_fd,
                                        argv, envp);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
    return process;
}

} // namespace AlphaISA

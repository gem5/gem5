# Copyright (c) 2021 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from ...resources.resource import AbstractResource

from m5.objects import SEWorkload, Process

class SEBinaryWorkload:
    """
    This class is used to enable simple Syscall-Execution (SE) mode execution
    of a binary.

    For this to function correctly the SEBinaryWorkload class should be added
    as a superclass to a board (i.e., something that inherits from
    AbstractBoard).
    """

    def set_se_binary_workload(self, binary: AbstractResource) -> None:
        """Set up the system to run a specific binary.

        **Limitations**
        * Only supports single threaded applications
        * Dynamically linked executables are partially supported when the host
          ISA and the simulated ISA are the same.

        :param binary: The resource encapsulating the binary to be run.
        """

        self.workload = SEWorkload.init_compatible(binary.get_local_path())

        process = Process()
        process.cmd = [binary.get_local_path()]
        self.get_processor().get_cores()[0].set_workload(process)

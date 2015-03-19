# Copyright (c) 2015 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
#
# Authors: Andreas Sandberg

from multiprocessing import Process
import sys
import os

import m5
m5.util.addToPath('../configs/common')

_exit_normal = (
    "target called exit()",
    "m5_exit instruction encountered",
    )

_exit_limit = (
    "simulate() limit reached",
    )

_exitcode_done = 0
_exitcode_fail = 1
_exitcode_checkpoint = 42


def _run_step(name, restore=None, interval=0.5):
    """
    Instantiate (optionally from a checkpoint if restore is set to the
    checkpoitn name) the system and run for interval seconds of
    simulated time. At the end of the simulation interval, create a
    checkpoint and exit.

    As this function is intended to run in its own process using the
    multiprocessing framework, the exit is a true call to exit which
    terminates the process. Exit codes are used to pass information to
    the parent.
    """
    if restore is not None:
        m5.instantiate(restore)
    else:
        m5.instantiate()

    e = m5.simulate(m5.ticks.fromSeconds(interval))
    cause = e.getCause()
    if cause in _exit_limit:
        m5.checkpoint(name)
        sys.exit(_exitcode_checkpoint)
    elif cause in _exit_normal:
        sys.exit(_exitcode_done)
    else:
        print "Test failed: Unknown exit cause: %s" % cause
        sys.exit(_exitcode_fail)

def run_test(root, interval=0.5, max_checkpoints=5):
    """
    Run the simulated system for a fixed amount of time and take a
    checkpoint, then restore from the same checkpoint and run until
    the system calls m5 exit.
    """

    cpt_name = os.path.join(m5.options.outdir, "test.cpt")
    restore = None

    for cpt_no in range(max_checkpoints):
        # Create a checkpoint from a separate child process. This enables
        # us to get back to a (mostly) pristine state and restart
        # simulation from the checkpoint.
        p = Process(target=_run_step,
                    args=(cpt_name, ),
                    kwargs={
                "restore" : restore,
                "interval" : interval,
                })
        p.start()

        # Wait for the child to return
        p.join()

        # Restore from the checkpoint next iteration
        restore = cpt_name

        if p.exitcode == _exitcode_done:
            print >> sys.stderr, "Test done."
            sys.exit(0)
        elif p.exitcode == _exitcode_checkpoint:
            pass
        else:
            print >> sys.stderr, "Test failed."
            sys.exit(1)

    # Maximum number of checkpoints reached. Just run full-speed from
    # now on.
    m5.instantiate()
    e = m5.simulate()
    cause = e.getCause()
    if cause in _exit_normal:
        sys.exit(0)
    else:
        print "Test failed: Unknown exit cause: %s" % cause
        sys.exit(1)

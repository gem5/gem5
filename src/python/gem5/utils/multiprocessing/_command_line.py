# Copyright (c) 2022 The Regents of The University of California
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

"""
This file contains extensions of the multiprocessing module to be used with gem5.
Specifically, it contains the code to produce the command line for spawned processes.
Some code inspired by the Python standard library implementation of the
multiprocessing module (i.e., cpython/Lib/multiprocessing/).
"""

import sys
from multiprocessing import spawn, util


def _gem5_args_for_multiprocessing(name):
    from m5 import options

    # Options that are disallowed with multiprocessing
    disallowed = [
        options.build_info,
        options.copyright,
        options.readme,
        options.interactive,
        options.pdb,
        options.verbose,
        options.debug_break,
        options.debug_help,
        options.debug_flags,
        options.debug_start,
        options.debug_end,
        options.debug_ignore,
        options.list_sim_objects,
    ]
    if any(disallowed):
        raise Exception(
            f"Disallowed option for multiprocessing. "
            f"See {__file__} for details."
        )

    # Options not forwarded:
    # --allow-remote-connections, --listener-mode, --dump-config, --json-config
    # --dot-config, --dot-dvfs-config, --debug-file, --remote-gdb-port, -c

    arguments = [
        f"--outdir={options.outdir}/{name}",
        f"--stdout-file={options.stdout_file}",
        f"--stderr-file={options.stderr_file}",
        f"--stats-file={options.stats_file}",
    ]
    if options.redirect_stdout:
        arguments.append("--redirect-stdout")
    if options.redirect_stderr:
        arguments.append("--redirect-stderr")
    if options.silent_redirect:
        arguments.append("--silent-redirect")
    if options.path:
        arguments.append(f"--path={':'.join(options.path)}")
    if options.quiet:
        arguments.append("--quiet")

    return arguments


def get_command_line(name, **kwds):
    """
    Returns prefix of command line used for spawning a child process
    """
    if getattr(sys, "frozen", False):
        return [sys.executable, "--multiprocessing-fork"] + [
            "%s=%r" % item for item in kwds.items()
        ]
    else:
        prog = "from multiprocessing.spawn import spawn_main; spawn_main(%s)"
        prog %= ", ".join("%s=%r" % item for item in kwds.items())
        opts = util._args_from_interpreter_flags()
        opts.extend(_gem5_args_for_multiprocessing(name))
        exe = spawn.get_executable()
        return [exe] + opts + ["-c", prog, "--multiprocessing-fork"]

# Copyright (c) 2024 The Regents of the University of California
# All Rights Reserved.
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
This module is the entry point for the multi-simulation (MultiSim) framework.
It provides a CLI using argparse to obtain the path to the simulation
configuration script and the number of processes to run in parallel.
"""
from gem5.utils.multisim.multisim import (
    module_run,
    run,
)


def main():
    import argparse
    from pathlib import Path

    global module_run
    module_run = True

    parser = argparse.ArgumentParser(
        description="Pass the config script specifying the simulations to run "
        "using multisim."
    )
    parser.add_argument(
        "config",
        type=str,
        help="The path to the config script specifying the simulations to run using multisim.",
    )
    parser.add_argument(
        "-p",
        "--processes",
        type=int,
        required=False,
        help="The maximum number of gem5 processes to be run in parallel."
        "If not set the number of processes will be equal to the number "
        "of available threads in the system.",
    )

    args = parser.parse_args()
    run(module_path=Path(args.config), processes=args.processes)


if __name__ == "__m5_main__":
    main()

# Copyright (c) 2025 Ayrton Chililbeck
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

import json

from m5.stats.gem5stats import SimStat

from gem5.utils.multisim.multisim import (
    module_run,
    run,
)


def main():

    # ----- Multisim entrypoint ----- #
    import argparse
    from pathlib import Path

    global module_run
    module_run = True

    multisim_parser = argparse.ArgumentParser(
        description="The geof parser for geof args",
    )

    multisim_parser.add_argument(
        "config",
        help="Path to the config file to run",
    )

    multisim_args = multisim_parser.parse_args()

    results = run(module_path=Path(multisim_args.config))
    # ----- End of multisim ----- #

    print(
        "##################### END OF SIMULATIONS ############################"
    )

    for sim, stats in results.items():
        print(
            f"Host simulation time for {sim}: {stats['hostSeconds']['value']}"
        )


if __name__ == "__m5_main__":
    main()

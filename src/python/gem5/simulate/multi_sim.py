# Copyright (c) 2024 The Regents of the University of California
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

import os
import signal
from time import sleep
from typing import (
    Callable,
    List,
    Tuple,
)

from gem5.utils.multiprocessing import Process

from .simulator import Simulator


class MultiSim:
    """
    This class is used to run multiple simulations in parallel. It takes a list of
    callables that return a Simulator object and runs them in parallel. The number of
    simulations that can be run in parallel is limited to the value of the num_cpus parameter
    """

    def __init__(
        self, sim_callables: List[Tuple[Callable[[], Simulator], str]]
    ):
        """
        :param sim_callables: A list of tuples of callables that return a Simulator object
                              and the name of the output directory
        """
        self.sim_callables = sim_callables

    def run_all(self, num_cpus: int):
        """
        Run all the simulations in parallel
        :param num_cpus: The number of simulations to run in parallel at a time
        """
        processes = []

        for sim_callable, output_name in self.sim_callables:
            if len(processes) > num_cpus:
                for process in processes:
                    if not process.is_alive():
                        processes.remove(process)
                    os.kill(process.pid, signal.SIGTERM)
                sleep(1)
            process = Process(
                target=run_simulator,
                args=(sim_callable,),
                name=output_name,
            )
            print(f"Starting process {output_name}")

            process.start()
            processes.append(process)

        while processes:
            for process in processes:
                if not process.is_alive():
                    processes.remove(process)
            sleep(1)

        print("All simulations have finished")


def run_simulator(
    sim_callable: Callable[[], Simulator],
):
    """
    Run the simulation
    :param sim_callable: A callable that returns a Simulator object
    """
    sim = sim_callable()
    sim.run()

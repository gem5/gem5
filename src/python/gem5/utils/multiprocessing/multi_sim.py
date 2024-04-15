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

import itertools
import os
import signal
from functools import partial
from time import sleep
from typing import (
    Any,
    Callable,
    List,
    Tuple,
)

from m5.util import inform

from gem5.resources.resource import AbstractResource
from gem5.utils.multiprocessing import Process

from ...simulate.simulator import Simulator


class MultiSim:
    """
    This class is used to run multiple simulations in parallel. It takes a list of
    callables that return a Simulator object and runs them in parallel. The number of
    simulations that can be run in parallel is limited to the value of the num_cpus parameter
    """

    def __init__(self, sim_callables: List[Callable[[], Simulator]]):
        """
        :param sim_callables: A list of callables that return a Simulator object
        """
        self.sim_callables = sim_callables

    def run(self, num_processes: int):
        """
        Run a specified number of simulations in parallel
        :param num_processes: The number of simulations to run in parallel at a time
        """
        active_processes = []
        remaiing_sim_callables = self.sim_callables.copy()
        while remaiing_sim_callables or active_processes:
            while (
                remaiing_sim_callables
                and len(active_processes) < num_processes
            ):
                sim_callable = remaiing_sim_callables.pop(0)
                process_name = f"{sim_callable.func.__name__}_{sim_callable.args[0].get_id()}"
                process = Process(
                    target=_run_simulator,
                    args=(sim_callable,),
                    name=process_name,
                )
                inform(f"Starting process {process_name}")

                process.start()
                print("===================================")
                active_processes.append(process)

            for process in active_processes:
                if not process.is_alive():
                    active_processes.remove(process)
            sleep(1)

        inform("All simulations have finished")


def _run_simulator(
    sim_callable: Callable[[], Simulator],
):
    """
    Run the simulation
    :param sim_callable: A callable that returns a Simulator object
    """
    sim = sim_callable()
    sim.run()

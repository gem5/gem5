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
from datetime import datetime

from gem5.simulate.simulator import Simulator
from gem5.utils.multiprocessing import Process


class GemaSimulationManager:
    """Class for managing gEMA simulations."""

    def __init__(self, root) -> None:
        """Initialize the gEMA simulation manager class."""
        self.root = root

    def get_lowest_sim_id(self, config_id):
        """Returns the lowest possible simulation ID that is not already in use."""
        sim_id = 1
        simulations = self.root.sims[f"config_{config_id}"]["simulations"]
        existing_sim_ids = {simulation["sim_id"] for simulation in simulations}

        while sim_id in existing_sim_ids:
            sim_id += 1
        return sim_id

    def generate_log_path(self, sim_id, config_id):
        # Probably a better way to do this
        import inspect
        from pathlib import Path

        here = Path(inspect.getfile(inspect.currentframe())).resolve()
        gem5_home = here.parents[5]
        return gem5_home / f"m5out/config_{config_id}_sim_{sim_id}"

    def save_simulation(self, sim_id, config_id, path):
        """Saves the simulation information tied to the configuration."""
        if self.root.sims.get(f"config_{config_id}") is not None:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            tmp_storage = dict(
                sim_id=sim_id, simulated_on=timestamp, logs=str(path)
            )
            self.root.sims[f"config_{config_id}"]["simulations"].append(
                tmp_storage
            )

    def start_subprocess(self, sim_id, config_id):
        """Uses gem5 multiprocessing to start the simulation as a subprocess."""
        path = self.generate_log_path(sim_id, config_id)
        self.save_simulation(sim_id, config_id, path)
        process = Process(
            target=self.run_gem5_simulator,
            args=(sim_id, config_id, path),
            name=(f"config_{config_id}_sim_{sim_id}"),
        )
        process.start()

    def run_gem5_simulator(self, sim_id, config_id, path):
        """Generates the config and runs the simulation."""
        print(
            f"Simulation ID: {sim_id} PPID: {os.getppid()} PID: {os.getpid()}"
        )
        config = self.root.sims.get(f"config_{config_id}").get("config")
        board = self.root.configurator.generate_config(config)

        simulator = Simulator(board=board)
        simulator.override_outdir(path)
        print(f"simulator id: {simulator.get_id()}")
        simulator.run()
        print(
            f"Exiting @ tick {simulator.get_current_tick()} because {simulator.get_last_exit_event_cause()}."
        )

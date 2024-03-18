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

import importlib.util
import os
import signal
from time import sleep
from typing import List

from gem5.utils.multiprocessing import Process

from .simulator import Simulator

"""
config = [{
    "File": "path/to/file",
    "function": "function_name",
    "config_args": ["arg1", "arg2"],
}]
"""


class MultiSim:
    def __init__(self, configs: dict):
        self.configs = configs

    def run_all(self, num_cpus: int):
        processes = []
        i = 1
        for config in self.configs:
            config_file_path = config["file"]
            config_args = config["config_args"]
            config_function = config["function"]
            print(f"Running simulation: {config_file_path}")
            print(f"Running with args: {config_args}")
            print(f"Running with function: {config_function}")
            if len(processes) > num_cpus:
                for process in processes:
                    if not process.is_alive():
                        processes.remove(process)
                    os.kill(process.pid, signal.SIGTERM)
                sleep(1)
                process = Process(
                    target=run_simulator,
                    args=(config_file_path, config_function, config_args),
                    name=f"sim-{i}",
                )
                print(process.pid)
                print(f"Starting process {i}")
                i += 1
                process.start()
                processes.append(process)

        while processes:
            for process in processes:
                if not process.is_alive():
                    processes.remove(process)
            sleep(1)

        print("All simulations have finished")


def run_simulator(
    config_file_path: str, function: str, config_args: List[str]
):
    sleep(10000)
    print(f"Running simulation: {config_file_path}")
    print(f"args: {config_args}")
    print(f"function: {function}")
    spec = importlib.util.spec_from_file_location("config", config_file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    target_function = getattr(module, function)
    target_function(*config_args)

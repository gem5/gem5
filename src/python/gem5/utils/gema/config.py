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

import inspect
from datetime import datetime

from gem5.components import *
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.cachehierarchies.classic.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.memory import *
from gem5.components.memory import (
    multi_channel,
    single_channel,
)
from gem5.components.processors.cpu_types import *
from gem5.components.processors.cpu_types import get_cpu_types_str_set
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.resources.resource import *


class GemaConfigRetreiver:
    """Obtains available configuration options from gem5."""

    def __init__(self, root) -> None:
        """Initialize the gEMA configuration retreiver class."""
        self.root = root
        self.single_channel_memory = [
            name
            for name, obj in inspect.getmembers(single_channel)
            if inspect.isfunction(obj)
        ]
        self.multi_channel_memory = [
            name
            for name, obj in inspect.getmembers(multi_channel)
            if inspect.isfunction(obj)
        ]
        self.cache_types = [
            "NoCache",
            "PrivateL1SharedL2CacheHierarchy",
            "PrivateL1PrivateL2CacheHierarchy",
            "PrivateL1CacheHierarchy",
        ]  # TODO: Fetch the cache types dynamically.

    def _get_init_parameters(self, *classes):
        """Returns a dictionary of init parameters from a class."""
        params_dict = {
            cls.__name__: [
                param
                for param in inspect.signature(cls.__init__).parameters
                if param not in ("self", "cls", "*args", "**kwargs")
            ]
            for cls in classes
        }
        return params_dict

    def get_config_options(self):
        """Obtains all available configuration options from the standard library."""
        cache_classes = [
            globals()[name] for name in self.cache_types if name in globals()
        ]
        classes_to_inspect = [
            SimpleBoard,
            X86Board,
            SimpleProcessor,
            *cache_classes,
        ]
        try:
            class_params = self._get_init_parameters(*classes_to_inspect)

            config = {}
            for board_class in [SimpleBoard, X86Board]:
                board_name = board_class.__name__
                config[board_name] = {
                    "board": class_params[board_name][0],
                    "memory": self.single_channel_memory
                    + self.multi_channel_memory,
                    "processor": list(get_cpu_types_str_set()),
                    "cache_hierarchy": {
                        name: class_params[name]
                        for name in self.cache_types
                        if name in class_params
                    },
                }
            return config
        except KeyError as e:
            print(
                f"Key error: {e} - Check if cache class names are correct and imported"
            )
        except Exception as e:
            print(f"An unexpected error occurred: {e}")


class GemaConfigGenerator:
    """Configures a user defined simulation object."""

    def __init__(self, root):
        """Initialize the gEMA configuration generator class."""
        self.root = root

    def generate_config(self, data: dict):
        """Returns a configuration given a provided json."""
        brd = eval(data["board"]["type"])
        clk = f"{data['board']['clk']}GHz"
        proc = eval(data["processor"]["type"])
        cpu_type = CPUTypes[data["processor"]["cpu"].upper()]
        isa = ISA[data["processor"]["isa"].upper()]
        ncores = int(data["processor"]["ncores"])
        mem_type = eval(data["memory"]["type"])
        msize = f"{data['memory']['size']}MB"
        cache = self.get_cache_configuration(data["cache"])

        configuration = brd(
            clk_freq=clk,
            processor=proc(cpu_type=cpu_type, isa=isa, num_cores=ncores),
            memory=mem_type(size=msize),
            cache_hierarchy=cache,
        )

        resource_type = data["resource"][0]
        resource = data["resource"][1]
        self.set_resource(configuration, resource_type, resource)

        self.print_config_summary(
            brd, clk, proc, cpu_type, isa, ncores, mem_type, msize, cache
        )

        return configuration

    def save_config(self, id, data=None):
        if self.root.sims.get(f"config_{id}") is not None:
            if data is None:
                print(
                    f"Regenerating configuration for id {id} using previously saved configuration."
                )
                data = self.root.sims.get(f"config_{id}").get("config")
            else:
                print(
                    f"Regenerating configuration for id {id} using new data."
                )
            del self.root.sims[f"config_{id}"]
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        tmp_storage = dict(generated_on=timestamp, config=data, simulations=[])
        self.root.sims[f"config_{id}"] = tmp_storage

    def get_cache_configuration(self, cache_config):
        cache_opts = {
            "class": globals()[cache_config["type"]],
            "l1d_size": f"{cache_config['l1d_size']}KiB",
            "l1i_size": f"{cache_config['l1i_size']}KiB",
            "l2_size": f"{cache_config['l2_size']}KiB",
            "l1d_assoc": cache_config["l1d_assoc"],
            "l1i_assoc": cache_config["l1i_assoc"],
            "l2_assoc": cache_config["l2_assoc"],
        }

        cache_params = [
            param
            for param in inspect.signature(
                cache_opts["class"].__init__
            ).parameters
            if param not in ("self", "cls", "*args", "**kwargs")
        ]

        init_params = {
            key: value
            for key, value in cache_opts.items()
            if key in cache_params
        }

        try:
            return cache_opts["class"](**init_params)
        except ValueError:
            print(f"Failed to generate cache: {cache_opts['class']}")

    def set_resource(self, config, resource_type, resource):
        """Sets the resource of a given configuration.
        default: uses gem5-resources, custom: sets a path to a binary"""
        if resource_type == "default":
            config.set_se_binary_workload(obtain_resource(resource))
        elif resource_type == "custom":
            config.set_se_binary_workload(BinaryResource(resource))
        else:
            print("Invalid resource type specified")

    def print_config_summary(
        self, board, clk, proc, cpu, isa, cores, mem_type, mem_size, cache
    ):
        """Prints the selected configuration."""
        print("\n======CONFIGURATION======")
        print(
            f"Board: {board}, \nClock Frequency: {clk}, \nProcessor: {proc} \nCPU Type: {cpu}, \nISA: {isa}, "
            f"\nNumber of Cores: {cores}, \nMemory Type: {mem_type}, \nMemory Size: {mem_size}, \nCache Type: {cache}\n"
        )

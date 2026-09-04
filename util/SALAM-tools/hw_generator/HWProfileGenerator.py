# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

import os

import yaml
from SALAMArgs import HWArgs
from SALAMClassGenerator import (
    FunctionalUnitGenerator,
    InstConfigGenerator,
)


def resolve_bench_dir(acc_bench_path, benchname, bench_path=None):
    if bench_path:
        return os.path.join(acc_bench_path, bench_path)
    return os.path.join(acc_bench_path, benchname)


class HWModel:
    def __init__(
        self,
        model="40nm_model",
        latency="5ns",
        profile="default_profile",
        bench_dir=None,
    ):
        self.bench_dir = bench_dir
        self.model = model
        self.latency = latency
        self.profile = profile
        self.yaml_dir = os.path.join(
            bench_dir,
            "configs",
            "hw_interface",
            "functional_units",
            model,
            latency,
            profile,
        )
        self.inst_list_yaml = os.path.join(
            bench_dir,
            "configs",
            "hw_interface",
            "instructions",
            "inst_list.yml",
        )

        self.fu_list = os.listdir(self.yaml_dir)
        self.inst_list = []

    def get_fu_list(self):
        return self.fu_list

    def get_instruction_list(self):
        self.fu_yaml = open(self.inst_list_yaml)
        self.data = yaml.load(self.fu_yaml, Loader=yaml.FullLoader)
        self.buffer = dict()
        with open(self.inst_list_yaml) as yaml_inst_list:
            buffer = yaml.safe_load(yaml_inst_list)
            inst_list = buffer["instructions"].keys()
        return buffer

    def generate_hw(self, fu):
        self.fu_yaml_path = self.yaml_dir + "/" + fu + "/" + fu + ".yml"
        self.fu_yaml = open(self.fu_yaml_path)
        self.data = yaml.load(self.fu_yaml, Loader=yaml.FullLoader)
        self.alias = self.data["functional_unit"]["parameters"]["alias"]
        self.stages = self.data["functional_unit"]["parameters"]["stages"]
        self.cycles = self.data["functional_unit"]["parameters"]["cycles"]
        self.enum_value = self.data["functional_unit"]["parameters"][
            "enum_value"
        ]
        self.int_size = self.data["functional_unit"]["parameters"][
            "datatypes"
        ]["integer"]["size"]
        self.int_sign = self.data["functional_unit"]["parameters"][
            "datatypes"
        ]["integer"]["sign"]
        self.int_apmode = self.data["functional_unit"]["parameters"][
            "datatypes"
        ]["integer"]["APMode"]
        self.fp_size = self.data["functional_unit"]["parameters"]["datatypes"][
            "floating_point"
        ]["size"]
        self.fp_sign = self.data["functional_unit"]["parameters"]["datatypes"][
            "floating_point"
        ]["sign"]
        self.fp_apmode = self.data["functional_unit"]["parameters"][
            "datatypes"
        ]["floating_point"]["APMode"]
        self.ptr_size = self.data["functional_unit"]["parameters"][
            "datatypes"
        ]["pointer"]["size"]
        self.ptr_sign = self.data["functional_unit"]["parameters"][
            "datatypes"
        ]["pointer"]["sign"]
        self.ptr_apmode = self.data["functional_unit"]["parameters"][
            "datatypes"
        ]["pointer"]["APMode"]
        self.instructions_list = self.data["functional_unit"]["parameters"][
            "instructions"
        ]
        with open(self.inst_list_yaml) as yaml_inst_list:
            buffer = yaml.safe_load(yaml_inst_list)
            for instruction in self.instructions_list:
                if (instruction != "any") and (instruction != "none"):
                    buffer["instructions"][instruction][
                        "functional_unit"
                    ] = self.enum_value
        with open(self.inst_list_yaml, "w") as yaml_inst_list:
            yaml.safe_dump(buffer, yaml_inst_list, default_flow_style=False)
        self.limit = self.data["functional_unit"]["parameters"]["limit"]

    def generate_power_model(self, fu):
        self.fu_yaml_path = self.yaml_dir + "/" + fu + "/" + fu + ".yml"
        self.fu_yaml = open(self.fu_yaml_path)
        self.data = yaml.load(self.fu_yaml, Loader=yaml.FullLoader)
        self.units_dict = self.data["functional_unit"]["power_model"]["units"]
        self.power_units = self.units_dict["power"]
        self.energy_units = self.units_dict["energy"]
        self.time_units = self.units_dict["time"]
        self.area_units = self.units_dict["area"]
        self.fu_latency = self.data["functional_unit"]["power_model"][
            "latency"
        ]
        self.internal_power = self.data["functional_unit"]["power_model"][
            "internal_power"
        ]
        self.switch_power = self.data["functional_unit"]["power_model"][
            "switch_power"
        ]
        self.dynamic_power = self.data["functional_unit"]["power_model"][
            "dynamic_power"
        ]
        self.dynamic_energy = self.data["functional_unit"]["power_model"][
            "dynamic_energy"
        ]
        self.leakage_power = self.data["functional_unit"]["power_model"][
            "leakage_power"
        ]
        self.area = self.data["functional_unit"]["power_model"]["area"]
        self.path_delay = self.data["functional_unit"]["power_model"][
            "path_delay"
        ]


def generate_hw_profile(m5_path, acc_bench_path, bench, bench_path=None):
    salam_dir = os.path.join(m5_path, "src", "salam")
    gen_base = os.path.join(salam_dir, "HWModeling")
    os.makedirs(os.path.join(gen_base, "functional_units"), exist_ok=True)
    os.makedirs(os.path.join(gen_base, "instructions"), exist_ok=True)

    bench_dir = resolve_bench_dir(acc_bench_path, bench, bench_path)
    generate_hw_models = HWModel(bench_dir=bench_dir, latency="5ns")
    fu_file_generator = FunctionalUnitGenerator(
        fu_directory=os.path.join(salam_dir, "FunctionalUnits.py")
    )
    fu_file_generator.initialize_functional_unit_base_header_file()
    fu_file_generator.initalize_fu_list_header(
        generate_hw_models.get_fu_list()
    )
    fu_file_generator.initialize_simobject_file(
        generate_hw_models.get_fu_list()
    )

    for functional_unit in generate_hw_models.get_fu_list():
        generate_hw_models.generate_hw(functional_unit)
        generate_hw_models.generate_power_model(functional_unit)
        fu_file_generator.set_fu(functional_unit)
        fu_file_generator.functional_unit_header_generator(generate_hw_models)
        fu_file_generator.simobject_generator(generate_hw_models)

    inst_cfg_gen = InstConfigGenerator()

    inst_cfg_gen.initialize_inst_config_base_header_file()
    inst_cfg_gen.instruction_simobject_generator(generate_hw_models)
    inst_cfg_gen.initalize_inst_config_header(
        generate_hw_models.get_instruction_list()
    )
    for inst in generate_hw_models.get_instruction_list()[
        "instructions"
    ].keys():
        inst_cfg_gen.inst_config_header_generator(inst)

    # source + SCons
    inst_cfg_gen.generate_inst_config_source(
        generate_hw_models.get_instruction_list()
    )
    inst_cfg_gen.generate_inst_config_sconscript(
        generate_hw_models.get_instruction_list()["instructions"]
    )
    fu_file_generator.generate_fu_list_source(generate_hw_models.get_fu_list())
    fu_file_generator.generate_functional_unit_sconscript(
        generate_hw_models.get_fu_list()
    )


def main():
    m5_path = os.environ.get("M5_PATH")
    if not m5_path:
        raise RuntimeError("Environment variable M5_PATH must be set")

    acc_bench_path = os.environ.get("ACC_BENCH_PATH")
    if not acc_bench_path:
        raise RuntimeError("Environment variable ACC_BENCH_PATH must be set")

    args = HWArgs()
    generate_hw_profile(m5_path, acc_bench_path, args.bench, args.bench_path)


if __name__ == "__main__":
    main()

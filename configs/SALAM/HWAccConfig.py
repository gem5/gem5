# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from configparser import ConfigParser
from pathlib import Path

import yaml

import m5
from m5.objects import *
from m5.util import *


def AccConfig(acc, bench_file, config_file):
    # Initialize LLVMInterface Objects
    acc.llvm_interface = LLVMInterface()

    # Benchmark path
    acc.llvm_interface.in_file = bench_file
    M5_Path = os.getenv("ACC_BENCH_PATH")
    benchname = os.path.splitext(os.path.basename(bench_file))[0]

    # lenet config launcher custom stuff
    benchPath = Path(bench_file).parts
    m5PathLen = len(Path(M5_Path).parts)

    # Set scheduling constraints
    # acc.llvm_interface.sched_threshold =
    #     ConfigSectionMap("Scheduler")['sched_threshold']
    # acc.llvm_interface.clock_period =
    #     ConfigSectionMap("AccConfig")['clock_period']
    # acc.llvm_interface.lockstep_mode =
    #     Config.getboolean("Scheduler", 'lockstep_mode')

    # Initialize HWInterface Objects
    acc.hw_interface = HWInterface()
    # Define HW Counts
    acc.hw_interface.cycle_counts = CycleCounts()
    # acc.hw_interface.cycle_counts

    if benchPath[m5PathLen + 1] == "mobilenetv2":
        fu_yaml = open(config_file)
        for yaml_inst_list in yaml.safe_load_all(fu_yaml):
            document = yaml_inst_list["hw_config"]
            current_acc = yaml_inst_list["hw_config"]["name"] + "_" + benchname
            if benchPath[9] == current_acc:
                print(current_acc + " Profile Loaded")
                print(yaml_inst_list["hw_config"][benchname])
                inst_list = yaml_inst_list["hw_config"][current_acc][
                    "instructions"
                ].keys()
                for instruction in inst_list:
                    setattr(
                        acc.hw_interface.cycle_counts,
                        instruction,
                        yaml_inst_list["hw_config"][current_acc][
                            "instructions"
                        ][instruction]["runtime_cycles"],
                    )
        fu_yaml.close()

    else:
        fu_yaml = open(config_file)
        yaml_inst_list = yaml.safe_load(fu_yaml)
        if yaml_inst_list["hw_config"][benchname] is not None:
            inst_list = yaml_inst_list["hw_config"][benchname][
                "instructions"
            ].keys()
            for instruction in inst_list:
                setattr(
                    acc.hw_interface.cycle_counts,
                    instruction,
                    yaml_inst_list["hw_config"][benchname]["instructions"][
                        instruction
                    ]["runtime_cycles"],
                )
        fu_yaml.close()

    #  Functional Units
    acc.hw_interface.functional_units = FunctionalUnits()
    acc.hw_interface.functional_units.double_multiplier = DoubleMultiplier()
    acc.hw_interface.functional_units.bit_register = BitRegister()
    acc.hw_interface.functional_units.bitwise_operations = BitwiseOperations()
    acc.hw_interface.functional_units.double_adder = DoubleAdder()
    acc.hw_interface.functional_units.float_divider = FloatDivider()
    acc.hw_interface.functional_units.bit_shifter = BitShifter()
    acc.hw_interface.functional_units.integer_multiplier = IntegerMultiplier()
    acc.hw_interface.functional_units.integer_adder = IntegerAdder()
    acc.hw_interface.functional_units.double_divider = DoubleDivider()
    acc.hw_interface.functional_units.float_adder = FloatAdder()
    acc.hw_interface.functional_units.float_multiplier = FloatMultiplier()

    #  Instructions
    acc.hw_interface.inst_config = InstConfig()
    acc.hw_interface.inst_config.add = Add()
    acc.hw_interface.inst_config.addrspacecast = Addrspacecast()
    acc.hw_interface.inst_config.alloca = Alloca()
    acc.hw_interface.inst_config.and_inst = AndInst()
    acc.hw_interface.inst_config.ashr = Ashr()
    acc.hw_interface.inst_config.bitcast = Bitcast()
    acc.hw_interface.inst_config.br = Br()
    acc.hw_interface.inst_config.call = Call()
    acc.hw_interface.inst_config.fadd = Fadd()
    acc.hw_interface.inst_config.fcmp = Fcmp()
    acc.hw_interface.inst_config.fdiv = Fdiv()
    acc.hw_interface.inst_config.fence = Fence()
    acc.hw_interface.inst_config.fmul = Fmul()
    acc.hw_interface.inst_config.fmuladd = Fmuladd()
    acc.hw_interface.inst_config.fneg = Fneg()
    acc.hw_interface.inst_config.fpext = Fpext()
    acc.hw_interface.inst_config.fptosi = Fptosi()
    acc.hw_interface.inst_config.fptoui = Fptoui()
    acc.hw_interface.inst_config.fptrunc = Fptrunc()
    acc.hw_interface.inst_config.frem = Frem()
    acc.hw_interface.inst_config.fsub = Fsub()
    acc.hw_interface.inst_config.gep = Gep()
    acc.hw_interface.inst_config.icmp = Icmp()
    acc.hw_interface.inst_config.indirectbr = Indirectbr()
    acc.hw_interface.inst_config.inttoptr = Inttoptr()
    acc.hw_interface.inst_config.invoke = Invoke()
    acc.hw_interface.inst_config.landingpad = Landingpad()
    acc.hw_interface.inst_config.load = Load()
    acc.hw_interface.inst_config.lshr = Lshr()
    acc.hw_interface.inst_config.mul = Mul()
    acc.hw_interface.inst_config.or_inst = OrInst()
    acc.hw_interface.inst_config.phi = Phi()
    acc.hw_interface.inst_config.ptrtoint = Ptrtoint()
    acc.hw_interface.inst_config.resume = Resume()
    acc.hw_interface.inst_config.ret = Ret()
    acc.hw_interface.inst_config.sdiv = Sdiv()
    acc.hw_interface.inst_config.select = Select()
    acc.hw_interface.inst_config.sext = Sext()
    acc.hw_interface.inst_config.shl = Shl()
    acc.hw_interface.inst_config.srem = Srem()
    acc.hw_interface.inst_config.store = Store()
    acc.hw_interface.inst_config.sub = Sub()
    acc.hw_interface.inst_config.switch_inst = SwitchInst()
    acc.hw_interface.inst_config.trunc = Trunc()
    acc.hw_interface.inst_config.udiv = Udiv()
    acc.hw_interface.inst_config.uitofp = Uitofp()
    acc.hw_interface.inst_config.unreachable = Unreachable()
    acc.hw_interface.inst_config.urem = Urem()
    acc.hw_interface.inst_config.vaarg = Vaarg()
    acc.hw_interface.inst_config.xor_inst = XorInst()
    acc.hw_interface.inst_config.zext = Zext()

    acc.hw_interface.salam_power_model = SALAMPowerModel()
    acc.hw_interface.hw_statistics = HWStatistics()
    acc.hw_interface.simulator_config = SimulatorConfig()
    acc.hw_interface.opcodes = InstOpCodes()

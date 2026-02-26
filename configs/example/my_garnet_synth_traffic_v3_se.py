# Copyright (c) 2016 Georgia Institute of Technology
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
#
# Author: Tushar Krishna

import argparse
import os
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath

addToPath("../")

from common import Options
from ruby import Ruby

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)

parser = argparse.ArgumentParser()
Options.addNoISAOptions(parser)

parser.add_argument(
    "--temporal-traffic",
    type=str,
    default="by_default",
    help="Temporal traffic type and arguments",
)

parser.add_argument(
    "--synthetic",
    default="uniform_random",
    choices=[
        "uniform_random",
        "tornado",
        "bit_complement",
        "bit_reverse",
        "bit_rotation",
        "neighbor",
        "shuffle",
        "transpose",
    ],
)

parser.add_argument(
    "-i",
    "--injectionrate",
    type=float,
    default=0.1,
    metavar="I",
    help="Injection rate in packets per cycle per node. \
                        Takes decimal value between 0 to 1 (eg. 0.225). \
                        Number of digits after 0 depends upon --precision.",
)

parser.add_argument(
    "--precision",
    type=int,
    default=3,
    help="Number of digits of precision after decimal point\
                        for injection rate",
)

parser.add_argument(
    "--sim-cycles", type=int, default=1000, help="Number of simulation cycles"
)

parser.add_argument(
    "--num-packets-max",
    type=int,
    default=-1,
    help="Stop injecting after --num-packets-max.\
                        Set to -1 to disable.",
)

parser.add_argument(
    "--single-sender-id",
    type=int,
    default=-1,
    help="Only inject from this sender.\
                        Set to -1 to disable.",
)

parser.add_argument(
    "--single-dest-id",
    type=int,
    default=-1,
    help="Only send to this destination.\
                        Set to -1 to disable.",
)

parser.add_argument(
    "--inj-vnet",
    type=int,
    default=-1,
    choices=[-1, 0, 1, 2],
    help="Only inject in this vnet (0, 1 or 2).\
                        0 and 1 are 1-flit, 2 is 5-flit.\
                        Set to -1 to inject randomly in all vnets.",
)

parser.add_argument(
    "--buffers-per-data-vc",
    type=int,
    default=4,
    choices=[1, 2, 3, 4, 5],
    help="The size of each data vc in terms of \
                        no of flits.",
)

parser.add_argument(
    "--response-limit",
    type=int,
    default=5000,
    help="the simulation will stop if \
            here is no progress for that many cycles",
)
'''
parser.add_argument(
    "--simple-physical-channels",
    action="store_true",
    default=False,
    help="""Simplenetwork links uses a seperate physical channel
            for each virtual network""",
)
'''
#
# Add the ruby specific and protocol specific options
#
Ruby.define_options(parser)

args = parser.parse_args()
print(f"args.cacheline_size:{args.cacheline_size}")
"""
args.num_cpus=64
args.num_dirs=64
args.network="garnet"
args.topology="Mesh_XY"
args.inj_vney=-1
args.router_latency=5
args.link_latency=1
args.mesh_rows=8
args.sim_cycles=1000000
args.num_packets_max=-1
args.single_sender_id=-1
args.single_dest_id=-1
args.link_width_bits=128
args.vcs_per_vnet=16
args.garnet_deadlock_threshold=5000
args.routing_algorithm=1
args.mem_size="512MiB"
args.synthetic="uniform_random"
args.injection_rate=0.01
"""
"""
inj_rates=[
        0.025, 0.03, 0.03, 0.005, 0.005, 0.03, 0.03, 0.03,
        0.03, 0.005, 0.03, 0.005, 0.03, 0.03, 0.03, 0.03,
        0.03, 0.03, 0.005, 0.005, 0.005, 0.005, 0.03, 0.03,
        0.005, 0.03, 0.005, 0.005, 0.005, 0.005, 0.03, 0.005,
        0.005, 0.03, 0.005, 0.005, 0.005, 0.005, 0.03, 0.005,
        0.03, 0.005, 0.005, 0.005, 0.005, 0.005, 0.03, 0.03,
        0.03, 0.005, 0.03, 0.005, 0.03, 0.03, 0.005, 0.03,
        0.03, 0.03, 0.03, 0.005, 0.005, 0.03, 0.005, 0.03,
        ]"""
inj_rates = [
    0.025,
    0.03,
    0.03,
    0.005,
    0.005,
    0.03,
    0.025,
    0.020,
    0.03,
    0.005,
    0.03,
    0.005,
    0.03,
    0.03,
    0.03,
    0.03,
    0.04,
    0.03,
    0.005,
    0.005,
    0.005,
    0.005,
    0.03,
    0.03,
    0.005,
    0.03,
    0.005,
    0.005,
    0.005,
    0.005,
    0.03,
    0.005,
    0.005,
    0.03,
    0.005,
    0.005,
    0.005,
    0.005,
    0.025,
    0.005,
    0.03,
    0.005,
    0.005,
    0.005,
    0.005,
    0.005,
    0.04,
    0.02,
    0.03,
    0.005,
    0.025,
    0.005,
    0.04,
    0.03,
    0.005,
    0.03,
    0.025,
    0.03,
    0.025,
    0.001,
    0.005,
    0.03,
    0.005,
    0.04,
]
import random

random.seed(23)
x_n = [random.random() for i in range(args.num_cpus)]

# temporal_traffic=['liebovitch,'+str(random.random())+\
#','+'1.5'+','+'1.0125'+','+'0.25'+','+'0.5' for i in range(args.num_cpus)]

# temporal_traffic=["liebovitch,"+str(random.random())+","+"1.5"+\
# ","+"1.0125"+","+"0.25"+","+"0.5"+"," for i in range(args.num_cpus)]

# temporal_traffic=["intermittency_map,"+str((random.random()/2)+0.07)+\
# ","+"0.0001"+","+"0.7"+","+"2"+"," for i in range(args.num_cpus)]
# high injection

# temporal_traffic=["intermittency_map,"+str((random.random()/2)+0.001)+\
# ","+"0.0001"+","+"0.9"+","+"2"+"," for i in range(args.num_cpus)]

# temporal_traffic=["bernoulli_shift"+","+str(x_n[i])+\
# ","+str(inj_rates[i]) for i in range(args.num_cpus)]
# inj_rates=[0.001 if(i%2==0) else 0.03 for i in range(args.num_cpus)]

# temporal_traffic=["intermittency_map,"+str((random.random()/2)+0.09)+\
# ","+"0.0001"+","+"0.9"+","+"2"+"," for i in range(args.num_cpus)]
# high injection
temporal_traffic = [
    "intermittency_map,"
    + str((random.random() / 2) + 0.001)
    + ","
    + "0.00001"
    + ","
    + "0.9"
    + ","
    + "3"
    + ","
    for i in range(args.num_cpus)
]
# temporal_traffic=["intermittency_map,"+str((random.random()/2)+0.07)+\
# ","+"0.00001"+","+"0.9"+","+"3"+"," for i in range(args.num_cpus)]
"""
cpus = [
#    MyGarnetSyntheticTraffic(
    #GarnetSyntheticTraffic(
    #GarnetSyntheticTrafficV2(
    #GarnetSyntheticTrafficV4(temp_traffic_type=args.temporal_traffic,
    #GarnetSyntheticTrafficV4(temp_traffic_args=args.temporal_traffic,
    GarnetSyntheticTrafficV4(
    temp_traffic_args=temporal_traffic[args.num_cpus-1-i],
        num_packets_max=args.num_packets_max,
        single_sender=args.single_sender_id,
        single_dest=args.single_dest_id,
        sim_cycles=args.sim_cycles,
        traffic_type=args.synthetic,
        #inj_rate=args.injectionrate,
        inj_rate=inj_rates[args.num_cpus-1-i],
        inj_vnet=args.inj_vnet,
        precision=args.precision,
        num_dest=args.num_dirs,
        response_limit=args.response_limit,
    )
    for i in range(args.num_cpus)
]
"""
# cpus=[BaseMinorCPU() for i in range(args.num_cpus)]
# cpus=[BaseTimingSimpleCPU() for i in range(args.num_cpus)]
# cpus=[X86CPU() for i in range(args.num_cpus)]
# buildEnv["PROTOCOL"]='MESI_Two_Level';
buildEnv["PROTOCOL"] = "MESI_Three_Level"
# args.num_clusters=1;
# args.l0i_size='32KiB'; args.l0d_size='32KiB'; args.l0i_assoc=4;
# args.l0d_assoc=4;
args.network = "garnet"
args.ruby = True
args.num_cpus = 9
args.topology = "Mesh_XY"
args.mesh_rows = 3
args.num_l2caches = 9
cpus = [TimingSimpleCPU() for i in range(args.num_cpus)]

print(buildEnv["PROTOCOL"])
print(f"----buildEnv----:{buildEnv}")
print("----args.num_dirs:", args.num_dirs)
print(f"----args----:{args}")

# create the desired simulated system
# system = System(cpu=cpus, mem_ranges=[AddrRange(args.mem_size)])
system = System(cpu=cpus, mem_ranges=[AddrRange(args.mem_size)])

# Create a top-level voltage domain and clock domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)

Ruby.create_system(args, False, system)

# Create a seperate clock domain for Ruby
system.ruby.clk_domain = SrcClockDomain(
    clock=args.ruby_clock, voltage_domain=system.voltage_domain
)
"""
i = 0
for ruby_port in system.ruby._cpu_ports:
    #
    # Tie the cpu test ports to the ruby cpu port
    #
    cpus[i].test = ruby_port.in_ports
    i += 1
"""
for i, cpu in enumerate(cpus):
    cpu.clk_domain = system.clk_domain
    cpu.createThreads()
    ruby_port = system.ruby._cpu_ports[i]
    cpu.createInterruptController()
    ruby_port.connectCpuPorts(cpu)

binary = [
    "/home/docker_share/gem5_mar15/materials/02-using-gem5/\
            03-running-in-gem5/02-annotate-this-x86"
    for i in range(len(cpus))
]
arguments = [[i] for i in range(len(cpus))]
# these are arguments to processes

# set the workload
from m5.objects import Process

process = [Process(pid=100 + i) for i in range(len(binary))]
for bin_file, proc, args_var in zip(binary, process, arguments):
    binary_path = bin_file
    # binary_path=bin_file.get_local_path()
    system.workload = SEWorkload.init_compatible(binary_path)
    proc.executable = binary_path
    proc.cmd = [binary_path] + args_var

for i, cpu in enumerate(cpus):
    # cpu.set_workload(process[i])
    cpu.workload = process[i]


# -----------------------
# run simulation
# -----------------------

root = Root(full_system=False, system=system)
# root = Root(full_system=True, system=system)
root.system.mem_mode = "timing"


# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency("1ps")
print("\n----numa_high_bit----\n")
print(args.numa_high_bit)

# instantiate configuration
m5.instantiate()
print("\n----\n")
print(buildEnv["PROTOCOL"])


# simulate until program terminates
# exit_event = m5.simulate(args.abs_max_tick)
# exit_event = m5.simulate(20000000)
# print("Exiting @ tick", m5.curTick(), "because", exit_event.getCause())

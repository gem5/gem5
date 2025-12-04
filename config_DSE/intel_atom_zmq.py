# GEM5 ZeroMQ integration for Intel Atom simulation
import argparse
import os
import sys

import m5
from m5.objects import *

# Adding zeroMQ python libs to the path
current_dir = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
python_lib_path = os.path.join(current_dir, "python_libs")
sys.path.insert(0, python_lib_path)

import zmq


# parse the arguments
def argument_parse():
    parser = argparse.ArgumentParser(description="DSE arguments")

    # location of the binary file
    parser.add_argument(
        "--binary",
        type=str,
        default="/home/abg309/PhD/RCL/gem5-dse/workload/defaultGait/x86/controller_code_cpp",
        help="Location of the binary file",
    )

    # input array
    parser.add_argument(
        "--input",
        type=str,
        nargs="+",
        default=["1.0", "2.0", "3.0"],
        help="Input array",
    )

    args = parser.parse_args()

    return args


# Create a system
args = argument_parse()


system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "3.4GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "atomic"
system.mem_ranges = [AddrRange("6MB")]

# system.cpu = X86TimingSimpleCPU()
system.cpu = X86AtomicSimpleCPU()


system.membus = SystemXBar()

system.cpu.icache_port = system.membus.cpu_side_ports
system.cpu.dcache_port = system.membus.cpu_side_ports
system.cpu.createInterruptController()
system.cpu.interrupts[0].pio = system.membus.mem_side_ports
system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

binary = args.binary

cmd = [binary]
cmd.extend(args.input)


# for gem5 V21 and beyond
system.workload = SEWorkload.init_compatible(binary)


process = Process()
process.cmd = cmd
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system=False, system=system)
m5.instantiate()


# Create the socket
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

message = "Simulation started"
print(f"[GEM5] Sending message to controller: {message}")
socket.send_string(message)

# Wait for the reply
reply = socket.recv_string()

# Starting sending the message
while reply != "ENDING":
    print(f"[GEM5] Received reply from controller: {reply}")

    # Advance the simulation by one cycle
    # exit_event = m5.simulate(1)

    # Send the next message
    message = "Next cycle completed"
    print(f"[GEM5] Sending message to controller: {message}")
    socket.send_string(message)

    # Wait for the reply
    reply = socket.recv_string()

print("[GEM5] Simulation ending as per controller request")

# exit_event = m5.simulate()

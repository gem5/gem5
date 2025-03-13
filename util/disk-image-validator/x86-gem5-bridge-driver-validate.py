import argparse

import m5.options

from gem5.prebuilt.demo.x86_demo_board import X86DemoBoard
from gem5.resources.resource import (
    BinaryResource,
    obtain_resource,
)
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

"""
This script is used to test that the gem5-bridge driver is correctly installed.
This test runs a simple C program that called m5 hypercall 8 without sudo.
"""

parser = argparse.ArgumentParser()
parser.add_argument("--workload", help="The workload to run")
parser.add_argument(
    "--resource_version", help="The version of the workload to run"
)


args = parser.parse_args()
workload_id = args.workload
resource_version = args.resource_version

# Here we setup the board. The prebuilt X86DemoBoard allows for Full-System X86
# simulation.
board = X86DemoBoard()

board.set_workload(
    obtain_resource(workload_id, resource_version=resource_version)
)
board.set_binary_to_run(
    application=BinaryResource(
        "./util/disk-image-validator/test_gem5_bridge.sh"
    ),
    args=[],
)

from gem5.simulate.exit_handler import ExitHandler


class Gem5BrideDriverTestExitHandler(ExitHandler, hypercall_num=8):
    def _process(self, simulator):
        print("Successfully called m5 hypercall without superuser privileges")
        print("Test passed")

    def _exit_simulation(self):
        return False


simulator = Simulator(board=board)

simulator.run()

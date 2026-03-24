"""
Local SE hello-world smoke test for CMake build verification.

Uses the checked-in x86 hello binary with explicit ISA binding via
gem5-stdlib. Does NOT require network access or obtain_resource().

Usage:
    gem5 tests/gem5/cmake/configs/se_hello_local.py
"""

import os
import sys

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(isa_required=ISA.X86)

# Resolve the local hello binary relative to the source root.
# gem5 sets M5_SRC_DIR or we can infer from this script's location.
_script_dir = os.path.dirname(os.path.abspath(__file__))
_src_root = os.path.normpath(os.path.join(_script_dir, "..", "..", "..", ".."))
_hello_bin = os.path.join(
    _src_root, "tests", "test-progs", "hello", "bin", "x86", "linux", "hello"
)
if not os.path.isfile(_hello_bin):
    sys.exit(f"ERROR: hello binary not found at {_hello_bin}")

processor = SimpleProcessor(cpu_type=CPUTypes.ATOMIC, isa=ISA.X86, num_cores=1)
memory = SingleChannelDDR3_1600(size="256MiB")
cache_hierarchy = NoCache()

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)
board.set_se_binary_workload(BinaryResource(local_path=_hello_bin))

simulator = Simulator(board=board)
simulator.run()

print(
    f"Exiting @ tick {simulator.get_current_tick()} "
    f"because {simulator.get_last_exit_event_cause()}"
)

import argparse

import m5
from m5.objects import Root

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.exit_event import ExitEvent
from gem5.utils.requires import requires

# Require X86 + MESI_TWO_LEVEL
requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
)


# --------------------
# Parse command-line arguments
# --------------------
def argument_parse():
    parser = argparse.ArgumentParser(description="FS Mode gem5 Example")

    # Linux kernel bzImage
    parser.add_argument(
        "--kernel",
        type=str,
        default="/path/to/bzImage",
        help="FS-mode Linux kernel image",
    )

    # Disk image
    parser.add_argument(
        "--disk",
        type=str,
        default="/path/to/fs.img",
        help="FS-mode disk image",
    )

    args = parser.parse_args()
    return args


args = argument_parse()

# --------------------
# Create processor
# --------------------
processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.TIMING,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=2,
)

# --------------------
# Memory
# --------------------
memory = DualChannelDDR4_2400(size="2GiB")

# --------------------
# X86 board
# --------------------
board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
)

# --------------------
# FS workload
# --------------------
from gem5.components.boards.fs_workload import X86FsLinux

board.workload = X86FsLinux(
    kernel=args.kernel,
    disk_image=args.disk,
)

# --------------------
# Instantiate root and simulate
# --------------------
root = Root(full_system=True, system=board)

m5.instantiate()

print("Starting FS-mode simulation...")
exit_event = m5.simulate()
print("Exiting @ tick", m5.curTick(), "because", exit_event.getCause())

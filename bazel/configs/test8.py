import os
from collections.abc import Sequence

from absl import (
    app,
    flags,
)
from google3.third_party.gem5.develop.src.python import m5
from google3.third_party.gem5.develop.src.python.gem5.components.boards.arm_board import (
    ArmBoard,
)
from google3.third_party.gem5.develop.src.python.gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from google3.third_party.gem5.develop.src.python.gem5.components.memory.multi_channel import (
    DualChannelDDR4_2400,
)
from google3.third_party.gem5.develop.src.python.gem5.components.processors.cpu_types import (
    CPUTypes,
)
from google3.third_party.gem5.develop.src.python.gem5.components.processors.simple_processor import (
    SimpleProcessor,
)
from google3.third_party.gem5.develop.src.python.gem5.isas import ISA
from google3.third_party.gem5.develop.src.python.gem5.resources.resource import (
    obtain_resource,
)
from google3.third_party.gem5.develop.src.python.gem5.simulate.exit_handler import (
    ExitHandler,
    KernelBootedExitHandler,
)
from google3.third_party.gem5.develop.src.python.gem5.simulate.simulator import (
    Simulator,
)
from google3.third_party.gem5.develop.src.python.gem5.utils.override import (
    overrides,
)
from google3.third_party.gem5.develop.src.python.m5.objects.ArmSystem import (
    ArmDefaultRelease,
)
from google3.third_party.gem5.develop.src.python.m5.objects.RealView import (
    VExpress_GEM5_Foundation,
)

_OUTDIR = flags.DEFINE_string(
    "outdir",
    "/tmp/gem5_out",
    "Output directory for gem5 simulation.",
)


def m5_setup():
    m5.event.mainq = m5.event.getEventQueue(0)
    m5.event.setEventQueue(m5.event.mainq)
    outdir = _OUTDIR.value
    os.makedirs(outdir, exist_ok=True)
    m5.options.outdir = outdir
    m5.options.show_exit_event_messages = True
    m5.options.dump_config = "config.ini"
    m5.options.json_config = ""
    m5.options.dot_config = ""
    m5.options.dot_dvfs_config = ""
    m5.stats.addStatVisitor(outdir + "/stats.txt")


def main(argv: Sequence[str]) -> None:
    if len(argv) > 1:
        raise app.UsageError("Too many command-line arguments.")

    # Here we set up the parameters of the l1 and l2 caches.
    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
    )

    # Memory: Dual Channel DDR4 2400 DRAM device.
    memory = DualChannelDDR4_2400(size="2GiB")

    # Here we set up the processor. We use a simple processor with TIMING cores.
    # This config script was also tested with ATOMIC cores.
    processor = SimpleProcessor(
        cpu_type=CPUTypes.TIMING, num_cores=2, isa=ISA.ARM
    )

    # The ArmBoard requires a `release` to be specified. This adds all the
    # extensions or features to the system. We are setting this to Armv8
    # (ArmDefaultRelease) in this example config script.
    release = ArmDefaultRelease()

    # The platform sets up the memory ranges of all the on-chip and off-chip
    # devices present on the ARM system.
    platform = VExpress_GEM5_Foundation()

    # Here we set up the board. The ArmBoard allows for Full-System ARM simulation.
    board = ArmBoard(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
        release=release,
        platform=platform,
    )

    # Here we set a full system workload. The workload
    # "arm-ubuntu-24.04-boot-with-systemd" boots Ubuntu 24.04.
    workload = obtain_resource(
        "arm-ubuntu-24.04-boot-with-systemd", resource_version="3.0.0"
    )
    board.set_workload(workload)

    # Examples of how you can override the default exit handler behaviors.
    # Exit handlers don't have to be specified in the config script if you don't
    # want to modify/override their default behaviors.

    # You can inherit from either the class that handles a certain hypercall by
    # default, or inherit directly from ExitHandler and specify a hypercall number.
    # See src/python/gem5/simulate/exit_handler.py for more information on which
    # handlers map to which hypercalls, and what the default behaviors are.

    class CustomKernelBootedExitHandler(KernelBootedExitHandler):

        @overrides(KernelBootedExitHandler)
        def _process(self, simulator: "Simulator") -> None:
            print("First exit: kernel booted")

        @overrides(KernelBootedExitHandler)
        def _exit_simulation(self) -> bool:
            # Returning True here will terminate after loading but before booting.
            return False

    class CustomAfterBootExitHandler(ExitHandler, hypercall_num=2):

        @overrides(ExitHandler)
        def _process(self, simulator: "Simulator") -> None:
            print("Second exit: Started `after_boot.sh` script")

        @overrides(ExitHandler)
        def _exit_simulation(self) -> bool:
            return False

    class AfterBootScriptExitHandler(ExitHandler, hypercall_num=3):

        @overrides(ExitHandler)
        def _process(self, simulator: "Simulator") -> None:
            print(f"Third exit: {self.get_handler_description()}")

        @overrides(ExitHandler)
        def _exit_simulation(self) -> bool:
            return True

    m5_setup()

    simulator = Simulator(board=board, outdir=_OUTDIR.value)

    simulator.run()

    print(
        "Exiting @ tick {} because {}.".format(
            simulator.get_current_tick(), simulator.get_last_exit_event_cause()
        )
    )


if __name__ == "__main__":
    app.run(main)

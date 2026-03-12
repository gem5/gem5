"""A slightly more complex test for gem5.

This test creates a more complex system than the test.py, but it is still
intended to be a simple test that checks that gem5 can be imported and run.
It imports components from the stdlib and calls the simulation loop.
This test only depends on the generators and the simple memory.
"""

import os
import sys

import m5
from src.python.gem5.components.boards.test_board import TestBoard
from src.python.gem5.components.cachehierarchies.classic.no_cache import NoCache
from src.python.gem5.components.memory.simple import SingleChannelSimpleMemory
from src.python.gem5.components.processors.linear_generator import LinearGenerator
from src.python.gem5.simulate.simulator import Simulator

def m5_setup(outdir):
    m5.event.mainq = m5.event.getEventQueue(0)
    m5.event.setEventQueue(m5.event.mainq)
    os.makedirs(outdir, exist_ok=True)
    m5.options.outdir = outdir
    m5.options.show_exit_event_messages = True
    m5.options.dump_config = "config.ini"
    m5.options.json_config = ""
    m5.options.dot_config = ""
    m5.options.dot_dvfs_config = ""
    m5.stats.addStatVisitor(outdir + "/stats.txt")


def main():
    outdir = "/tmp/gem5_out"
    if len(sys.argv) > 1:
        outdir = sys.argv[1]

    generator = LinearGenerator(
        rate="32GiB/s", max_addr=1024*1024*1024, rd_perc=100
    )
    memory = SingleChannelSimpleMemory(
        latency="10ns",
        latency_var="0ps",
        bandwidth="1GiB/s",
        size="1GiB",
    )
    board = TestBoard(
        clk_freq="1GHz",  # Ignored for these generators
        generator=generator,  # We pass the traffic generator as the processor.
        memory=memory,
        # With no cache hierarchy the test board will directly connect the
        # generator to the memory
        cache_hierarchy=NoCache(),
    )

    m5_setup(outdir)

    simulator = Simulator(board=board, outdir=outdir)
    simulator.run()

    print(
        "Exiting @ tick {} because {}.".format(
            simulator.get_current_tick(), simulator.get_last_exit_event_cause()
        )
    )

if __name__ == "__main__":
    main()

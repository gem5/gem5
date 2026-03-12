"""This gem5 configuation script creates a simple board to run an ARM binary.

This setup is close to the simplest setup possible using the gem5
library. It does not contain any kind of caching, IO, or any non-essential
components.
"""

import os
import sys

import m5
import src.python.m5.debug as debug
import src.python.m5.event as event
import src.python.m5.stats as stats
import src.python.m5.trace as trace
from src.python.gem5.components.boards.simple_board import SimpleBoard
from src.python.gem5.components.cachehierarchies.classic.no_cache import NoCache
from src.python.gem5.components.memory.simple import SingleChannelSimpleMemory
from src.python.gem5.components.processors.cpu_types import CPUTypes
from src.python.gem5.components.processors.simple_processor import SimpleProcessor
from src.python.gem5.isas import ISA
from src.python.gem5.resources.resource import obtain_resource
from src.python.gem5.simulate.simulator import Simulator
from src.python.gem5.utils.requires import requires

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

  trace.enable()
  trace.output(outdir + "/trace.txt")


def main():
  outdir = "/tmp/gem5_out"
  if len(sys.argv) > 1:
      outdir = sys.argv[1]

  # This check ensures the gem5 binary contains the ARM ISA target. If not, an
  # exception will be thrown.
  requires(isa_required=ISA.ARM)

  # In this setup we don't have a cache. `NoCache` can be used for such setups.
  cache_hierarchy = NoCache()

  # We use a simple memory system
  memory = SingleChannelSimpleMemory(
      latency="10ns", latency_var="0ns", bandwidth="100GiB/s", size="32MiB"
  )

  # We use a simple Timing processor with one core.
  processor = SimpleProcessor(
      cpu_type=CPUTypes.O3, isa=ISA.ARM, num_cores=1
  )

  # The gem5 library simple board which can be used to run SE-mode simulations.
  board = SimpleBoard(
      clk_freq="3GHz",
      processor=processor,
      memory=memory,
      cache_hierarchy=cache_hierarchy,
  )

  # Here we set the workload. In this case we want to run a simple Hello World
  # program compiled to the ARM ISA. The `obtain_resource` function will
  # automatically download the binary from the gem5 Resources cloud bucket if
  # it's not already present.
  board.set_se_binary_workload(
      obtain_resource("arm-hello64-static", resource_version="1.0.0")
  )

  m5_setup(outdir)

  # Lastly we run the simulation.
  simulator = Simulator(board=board, outdir=outdir)
  simulator.run()


if __name__ == "__main__":
  main()

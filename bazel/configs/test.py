"""Very simple test to check that gem5 can be imported and run."""

import os
import sys

import m5
from m5 import (
    core,
    event,
    options,
    stats,
)
from m5.objects import (
    GoodbyeObject,
    HelloObject,
    Root,
    SimpleObject,
)


def main():
    outdir = "/tmp/gem5_out"
    if len(sys.argv) > 1:
        outdir = sys.argv[1]

    root = Root(full_system=False)

    # Create an instantiation of the simobject you created
    root.hello = HelloObject(time_to_wait="2us", number_of_fires=1)
    root.hello.goodbye_object = GoodbyeObject(buffer_size="100B")

    event.mainq = event.getEventQueue(0)
    event.setEventQueue(event.mainq)

    m5.options["outdir"] = outdir
    m5.options["dump_config"] = "config.ini"
    m5.options["json_config"] = ""
    m5.options["dot_config"] = ""
    m5.options["dot_dvfs_config"] = ""

    os.makedirs(outdir, exist_ok=True)
    core.setOutputDir(outdir)
    stats.addStatVisitor(outdir + "/stats.txt")

    # instantiate all of the objects we've created above
    m5.instantiate()

    print("SYS MODULES AFTER:")
    for k in sys.modules.keys():
        if "param_Root" in k or "param_System" in k or "SimObject" in k:
            print("  ", k)

    print("Beginning simulation!")
    exit_event = m5.simulate()
    print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")


if __name__ == "__main__":
    main()

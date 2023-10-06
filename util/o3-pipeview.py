#! /usr/bin/env python3
# Copyright (c) 2011 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
# Pipeline activity viewer for the O3 CPU model.
import argparse
import copy
import os
import sys

# Temporary storage for instructions. The queue is filled in out-of-order
# until it reaches 'max_threshold' number of instructions. It is then
# sorted out and instructions are printed out until their number drops to
# 'min_threshold'.
# It is assumed that the instructions are not out of order for more then
# 'min_threshold' places - otherwise they will appear out of order.
insts = {
    "queue": [],  # Instructions to print.
    "max_threshold": 2000,  # Instructions are sorted out and printed when
    # their number reaches this threshold.
    "min_threshold": 1000,  # Printing stops when this number is reached.
    "sn_start": 0,  # The first instruction seq. number to be printed.
    "sn_stop": 0,  # The last instruction seq. number to be printed.
    "tick_start": 0,  # The first tick to be printed
    "tick_stop": 0,  # The last tick to be printed
    "tick_drift": 2000,  # Used to calculate the start and the end of main
    # loop. We assume here that the instructions are not
    # out of order for more then 2000 CPU ticks,
    # otherwise the print may not start/stop
    # at the time specified by tick_start/stop.
    "only_committed": 0,  # Set if only committed instructions are printed.
}


def process_trace(
    trace,
    outfile,
    cycle_time,
    width,
    color,
    timestamps,
    committed_only,
    store_completions,
    start_tick,
    stop_tick,
    start_sn,
    stop_sn,
):
    global insts

    insts["sn_start"] = start_sn
    insts["sn_stop"] = stop_sn
    insts["tick_start"] = start_tick
    insts["tick_stop"] = stop_tick
    insts["tick_drift"] = insts["tick_drift"] * cycle_time
    insts["only_committed"] = committed_only
    line = None
    fields = None

    # Skip lines up to the starting tick
    if start_tick != 0:
        while True:
            line = trace.readline()
            if not line:
                return
            fields = line.split(":")
            if fields[0] != "O3PipeView":
                continue
            if int(fields[2]) >= start_tick:
                break
    elif start_sn != 0:
        while True:
            line = trace.readline()
            if not line:
                return
            fields = line.split(":")
            if fields[0] != "O3PipeView":
                continue
            if fields[1] == "fetch" and int(fields[5]) >= start_sn:
                break
    else:
        line = trace.readline()
        if not line:
            return
        fields = line.split(":")

    # Skip lines up to next instruction fetch
    while fields[0] != "O3PipeView" or fields[1] != "fetch":
        line = trace.readline()
        if not line:
            return
        fields = line.split(":")

    # Print header
    outfile.write(
        "// f = fetch, d = decode, n = rename, p = dispatch, "
        "i = issue, c = complete, r = retire",
    )

    if store_completions:
        outfile.write(", s = store-complete")
    outfile.write("\n\n")

    outfile.write(
        " "
        + "timeline".center(width)
        + "   "
        + "tick".center(15)
        + "  "
        + "pc.upc".center(12)
        + "  "
        + "disasm".ljust(25)
        + "  "
        + "seq_num".center(10),
    )
    if timestamps:
        outfile.write("timestamps".center(25))
    outfile.write("\n")

    # Region of interest
    curr_inst = {}
    while True:
        if fields[0] == "O3PipeView":
            curr_inst[fields[1]] = int(fields[2])
            if fields[1] == "fetch":
                if (
                    stop_tick > 0
                    and int(fields[2]) > stop_tick + insts["tick_drift"]
                ) or (
                    stop_sn > 0
                    and int(fields[5]) > (stop_sn + insts["max_threshold"])
                ):
                    print_insts(
                        outfile,
                        cycle_time,
                        width,
                        color,
                        timestamps,
                        store_completions,
                        0,
                    )
                    return
                (curr_inst["pc"], curr_inst["upc"]) = fields[3:5]
                curr_inst["sn"] = int(fields[5])
                curr_inst["disasm"] = " ".join(fields[6][:-1].split())
            elif fields[1] == "retire":
                if curr_inst["retire"] == 0:
                    curr_inst["disasm"] = "-----" + curr_inst["disasm"]
                if store_completions:
                    curr_inst[fields[3]] = int(fields[4])
                queue_inst(
                    outfile,
                    curr_inst,
                    cycle_time,
                    width,
                    color,
                    timestamps,
                    store_completions,
                )

        line = trace.readline()
        if not line:
            print_insts(
                outfile,
                cycle_time,
                width,
                color,
                timestamps,
                store_completions,
                0,
            )
            return
        fields = line.split(":")


# Puts new instruction into the print queue.
# Sorts out and prints instructions when their number reaches threshold value
def queue_inst(
    outfile,
    inst,
    cycle_time,
    width,
    color,
    timestamps,
    store_completions,
):
    global insts
    l_copy = copy.deepcopy(inst)
    insts["queue"].append(l_copy)
    if len(insts["queue"]) > insts["max_threshold"]:
        print_insts(
            outfile,
            cycle_time,
            width,
            color,
            timestamps,
            store_completions,
            insts["min_threshold"],
        )


# Sorts out and prints instructions in print queue
def print_insts(
    outfile,
    cycle_time,
    width,
    color,
    timestamps,
    store_completions,
    lower_threshold,
):
    global insts
    # sort the list of insts by sequence numbers
    insts["queue"].sort(key=lambda inst: inst["sn"])
    while len(insts["queue"]) > lower_threshold:
        print_item = insts["queue"].pop(0)
        # As the instructions are processed out of order the main loop starts
        # earlier then specified by start_sn/tick and finishes later then what
        # is defined in stop_sn/tick.
        # Therefore, here we have to filter out instructions that reside out of
        # the specified boundaries.
        if insts["sn_start"] > 0 and print_item["sn"] < insts["sn_start"]:
            continue
            # earlier then the starting sequence number
        if insts["sn_stop"] > 0 and print_item["sn"] > insts["sn_stop"]:
            continue
            # later then the ending sequence number
        if (
            insts["tick_start"] > 0
            and print_item["fetch"] < insts["tick_start"]
        ):
            continue
            # earlier then the starting tick number
        if insts["tick_stop"] > 0 and print_item["fetch"] > insts["tick_stop"]:
            continue
            # later then the ending tick number

        if insts["only_committed"] != 0 and print_item["retire"] == 0:
            continue
            # retire is set to zero if it hasn't been completed
        print_inst(
            outfile,
            print_item,
            cycle_time,
            width,
            color,
            timestamps,
            store_completions,
        )


# Prints a single instruction
def print_inst(
    outfile,
    inst,
    cycle_time,
    width,
    color,
    timestamps,
    store_completions,
):
    if color:
        from m5.util.terminal import termcap
    else:
        from m5.util.terminal import no_termcap as termcap
    # Pipeline stages
    stages = [
        {
            "name": "fetch",
            "color": termcap.Blue + termcap.Reverse,
            "shorthand": "f",
        },
        {
            "name": "decode",
            "color": termcap.Yellow + termcap.Reverse,
            "shorthand": "d",
        },
        {
            "name": "rename",
            "color": termcap.Magenta + termcap.Reverse,
            "shorthand": "n",
        },
        {
            "name": "dispatch",
            "color": termcap.Green + termcap.Reverse,
            "shorthand": "p",
        },
        {
            "name": "issue",
            "color": termcap.Red + termcap.Reverse,
            "shorthand": "i",
        },
        {
            "name": "complete",
            "color": termcap.Cyan + termcap.Reverse,
            "shorthand": "c",
        },
        {
            "name": "retire",
            "color": termcap.Blue + termcap.Reverse,
            "shorthand": "r",
        },
    ]
    if store_completions:
        stages.append(
            {
                "name": "store",
                "color": termcap.Yellow + termcap.Reverse,
                "shorthand": "s",
            },
        )

    # Print

    time_width = width * cycle_time
    base_tick = (inst["fetch"] // time_width) * time_width

    # Find out the time of the last event - it may not
    # be 'retire' if the instruction is not comlpeted.
    last_event_time = max(
        inst["fetch"],
        inst["decode"],
        inst["rename"],
        inst["dispatch"],
        inst["issue"],
        inst["complete"],
        inst["retire"],
    )
    if store_completions:
        last_event_time = max(last_event_time, inst["store"])

    # Timeline shorter then time_width is printed in compact form where
    # the print continues at the start of the same line.
    if (last_event_time - inst["fetch"]) < time_width:
        num_lines = 1  # compact form
    else:
        num_lines = ((last_event_time - base_tick) // time_width) + 1

    curr_color = termcap.Normal

    # This will visually distinguish completed and abandoned intructions.
    if inst["retire"] == 0:
        dot = "="  # abandoned instruction
    else:
        dot = "."  # completed instruction

    for i in range(num_lines):
        start_tick = base_tick + i * time_width
        end_tick = start_tick + time_width
        if num_lines == 1:  # compact form
            end_tick += inst["fetch"] - base_tick
        events = []
        for stage_idx in range(len(stages)):
            tick = inst[stages[stage_idx]["name"]]
            if tick != 0:
                if tick >= start_tick and tick < end_tick:
                    events.append(
                        (
                            tick % time_width,
                            stages[stage_idx]["name"],
                            stage_idx,
                            tick,
                        ),
                    )
        events.sort()
        outfile.write("[")
        pos = 0
        if num_lines == 1 and events[0][2] != 0:  # event is not fetch
            curr_color = stages[events[0][2] - 1]["color"]
        for event in events:
            if (
                stages[event[2]]["name"] == "dispatch"
                and inst["dispatch"] == inst["issue"]
            ):
                continue
            outfile.write(curr_color + dot * ((event[0] // cycle_time) - pos))
            outfile.write(
                stages[event[2]]["color"] + stages[event[2]]["shorthand"],
            )

            if event[3] != last_event_time:  # event is not the last one
                curr_color = stages[event[2]]["color"]
            else:
                curr_color = termcap.Normal

            pos = (event[0] // cycle_time) + 1
        outfile.write(
            curr_color
            + dot * (width - pos)
            + termcap.Normal
            + "]-("
            + str(base_tick + i * time_width).rjust(15)
            + ") ",
        )
        if i == 0:
            outfile.write(
                "%s.%s %s [%s]"
                % (
                    inst["pc"].rjust(10),
                    inst["upc"],
                    inst["disasm"].ljust(25),
                    str(inst["sn"]).rjust(10),
                ),
            )
            if timestamps:
                outfile.write(f"  f={inst['fetch']}, r={inst['retire']}")
            outfile.write("\n")
        else:
            outfile.write("...".center(12) + "\n")


def validate_range(my_range):
    my_range = [int(i) for i in my_range.split(":")]
    if (
        len(my_range) != 2
        or my_range[0] < 0
        or my_range[1] > 0
        and my_range[0] >= my_range[1]
    ):
        return None
    return my_range


def main():
    # Parse args
    usage = "%(prog)s [OPTION]... TRACE_FILE"
    parser = argparse.ArgumentParser(
        usage=usage,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-o",
        dest="outfile",
        default=os.path.join(os.getcwd(), "o3-pipeview.out"),
        help="output file",
    )
    parser.add_argument(
        "-t",
        dest="tick_range",
        default="0:-1",
        help="tick range (-1 == inf.)",
    )
    parser.add_argument(
        "-i",
        dest="inst_range",
        default="0:-1",
        help="instruction range (-1 == inf.)",
    )
    parser.add_argument(
        "-w",
        dest="width",
        type=int,
        default=80,
        help="timeline width",
    )
    parser.add_argument(
        "--color",
        action="store_true",
        default=False,
        help="enable colored output",
    )
    parser.add_argument(
        "-c",
        "--cycle-time",
        type=int,
        default=1000,
        help="CPU cycle time in ticks",
    )
    parser.add_argument(
        "--timestamps",
        action="store_true",
        default=False,
        help="print fetch and retire timestamps",
    )
    parser.add_argument(
        "--only_committed",
        action="store_true",
        default=False,
        help="display only committed (completed) instructions",
    )
    parser.add_argument(
        "--store_completions",
        action="store_true",
        default=False,
        help="additionally display store completion ticks",
    )
    parser.add_argument("tracefile")

    args = parser.parse_args()
    tick_range = validate_range(args.tick_range)
    if not tick_range:
        parser.error("invalid range")
        sys.exit(1)
    inst_range = validate_range(args.inst_range)
    if not inst_range:
        parser.error("invalid range")
        sys.exit(1)
    # Process trace
    print("Processing trace... ", end=" ")
    with open(args.tracefile, "r") as trace:
        with open(args.outfile, "w") as out:
            process_trace(
                trace,
                out,
                args.cycle_time,
                args.width,
                args.color,
                args.timestamps,
                args.only_committed,
                args.store_completions,
                *(tick_range + inst_range),
            )
    print("done!")


if __name__ == "__main__":
    sys.path.append(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..",
            "src",
            "python",
        ),
    )
    main()

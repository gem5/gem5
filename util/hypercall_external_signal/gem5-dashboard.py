import argparse
import json
import os
import sys
import time

import tqdm
from orchestrator_request import (
    find_gem5_pids,
    send_and_receive_hypercall,
)


class ProgressBar:
    bar_pos = 0

    def __init__(self, pid):

        init_stats_str = send_and_receive_hypercall(
            pid, "get_progress_bar_init_stats"
        )
        # If the external signal handler times out getting the initial stats,
        # which might happen if a gem5 simulation is launched while the
        # dashboard is running, return without initializing anything so the
        # pid will be excluded from the current loop
        if init_stats_str == "timeout":
            return
        try:
            init_stats = json.loads(init_stats_str)

        except json.decoder.JSONDecodeError:
            print(f"Failed to load the initial stats for pid {pid}!")
            return

        self.total_insts = init_stats["total_insts"]
        self.sim_id = init_stats["sim_id"]
        self.workload = init_stats["workload"]
        self.pid = pid
        self.prev_insts = 0
        self.prog_bar = tqdm.tqdm(
            total=int(self.total_insts),
            desc=f" {pid} | {self.sim_id} | {self.workload}",
            position=self.bar_pos,
        )
        ProgressBar.bar_pos += 1


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pid",
        help="Enter the pid of the gem5 process for which you would like to"
        "observe the progress",
        type=int,
    )

    args = parser.parse_args()

    # clear screen, print dashboard
    print_dashboard_label()
    progress_bar_dict = {}

    if args.pid:
        gem5_pids = [int(args.pid)]
    else:
        gem5_pids = []

    pids_to_remove = []
    pids_to_add = []
    while True:

        if not args.pid:
            pids_to_add = [
                pid for pid in find_gem5_pids() if pid not in gem5_pids
            ]
        for pid in pids_to_add:
            temp_bar = ProgressBar(pid)
            if hasattr(temp_bar, "total_insts"):
                progress_bar_dict[pid] = temp_bar
                gem5_pids.append(pid)

        for pid in pids_to_remove:
            progress_bar_dict[pid].prog_bar.set_description(
                f" {pid} (exited) | {progress_bar_dict[pid].sim_id} | {progress_bar_dict[pid].workload}"
            )
            progress_bar_dict[pid].prog_bar.bar_format = (
                "{desc}: {percentage:3.0f}%|{bar}| {n_fmt}/{total_fmt} [{elapsed}<00:00,{rate_fmt}{postfix}]"
            )
            gem5_pids.remove(pid)
        pids_to_remove = []
        if not gem5_pids:
            break

        for pid, bar_obj in progress_bar_dict.items():
            if pid in gem5_pids:
                curr_status = send_and_receive_hypercall(
                    pid, "get_progress_bar_inst_count"
                )
                if "ended" in curr_status:
                    pids_to_remove.append(pid)
                    continue
                elif "timeout" in curr_status:
                    continue
                curr_insts = int(json.loads(curr_status))
                bar_obj.prog_bar.update(curr_insts - bar_obj.prev_insts)
                bar_obj.prev_insts = curr_insts

        time.sleep(5.0)


def print_dashboard_label() -> None:
    screen_width = os.get_terminal_size()[0]
    os.system("cls||clear")
    left_justified_label = " pid | simulation id | workload: "
    print(left_justified_label, end="")
    right_justified_label = (
        " current insts / total insts [elapsed<remaining, rate]"
    )
    print(
        right_justified_label.rjust(screen_width - len(left_justified_label))
    )


if __name__ == "__main__":
    main()

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
        # get static stats for the progress bar
        init_stats_str = send_and_receive_hypercall(
            pid, "get_progress_bar_init_stats"
        )
        # print(f"init_stats_str: {init_stats_str}")
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
        gem5_pids = find_gem5_pids()
        # print(f"gem5_pids: {gem5_pids}")
    for pid in gem5_pids:
        if pid not in progress_bar_dict.keys():
            # print(f"Adding progress bar for pid {pid}")
            progress_bar_dict[pid] = ProgressBar(pid)
            # If the initial stats for the workload didn't load, remove it from
            # the list
            if not hasattr(progress_bar_dict[pid], "total_insts"):
                progress_bar_dict.pop(pid)
    while True:
        # If new gem5 processes are launched while the dashboard is running,
        # add them. Only do this if --pid is not passed, as we assume the user
        # only wants to monitor a specific PID in that case.
        # For now, this code is commented out because it doesn't work
        # if not args.pid:
        #     gem5_pids = find_gem5_pids()
        #     # print(f"gem5_pids: {gem5_pids}")
        # for pid in gem5_pids:
        #     if pid not in progress_bar_dict.keys():
        #         # print(f"Adding progress bar for pid {pid}")
        #         progress_bar_dict[pid] = ProgressBar(pid, bar_pos)
        #         bar_pos += 1

        pids_to_remove = []
        for pid, bar_obj in progress_bar_dict.items():
            curr_status = send_and_receive_hypercall(
                pid, "get_progress_bar_inst_count"
            )
            if "ended" in curr_status:
                # print("current status is ended!")
                pids_to_remove.append(
                    pid
                )  # Can't pop the progress bar here, as it will cause an error if it is popped while Python is still iterating over the dict
                # break
                continue
            elif "timeout" in curr_status:
                # print("current status is timeout!")
                continue
            # print(f"curr_status: {curr_status}")
            curr_insts = int(json.loads(curr_status))
            bar_obj.prog_bar.update(curr_insts - bar_obj.prev_insts)
            # print(f"updating progress bar for pid {pid}")
            bar_obj.prev_insts = curr_insts

        for pid in pids_to_remove:
            progress_bar_dict[pid].prog_bar.set_description(
                f" {pid} (exited) | {bar_obj.sim_id} | {bar_obj.workload}"
            )
            progress_bar_dict.pop(pid)

        if not progress_bar_dict:
            break
        time.sleep(10.0)


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

import argparse
import json
import os
import time

import tqdm
from orchestrator_request import (
    find_gem5_pids,
    send_and_receive_hypercall,
)

# from transmitter import send_signal


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pid",
        help="Enter the pid of the gem5 process for which you would like to"
        "observe the progress",
        type=int,
    )

    args = parser.parse_args()

    if not args.pid:
        gem5_pids = find_gem5_pids()
    else:
        gem5_pids = [int(args.pid)]

    # for pid in gem5_simulations:
    print(f"gem5 pids: {gem5_pids}")
    init_stats = {
        pid: json.loads(
            send_and_receive_hypercall(pid, "get_progress_bar_init_stats")
        )
        for pid in gem5_pids
    }
    # init_stats = json.loads(send_and_receive_hypercall(args.pid, "get_progress_bar_init_stats"))
    # total_insts = int(init_stats["total_insts"])
    print(f"init_stats: {init_stats}")

    # clear screen, print dashboard

    print_dashboard_label()

    prev_insts = {pid: 0 for pid in gem5_pids}
    bar_pos = 0
    progress_bar_dict = {}
    for pid, stats in init_stats.items():
        # total_insts = int(stats["total_insts"])
        progress_bar_dict[pid] = tqdm.tqdm(
            total=int(stats["total_insts"]),
            desc=f" {pid} | {stats["sim_id"]} | {stats["workload"]}",
            position=bar_pos,
        )
        bar_pos += 1
        # print(f"bar pos: {bar_pos}")
        # while curr_status := send_and_receive_hypercall(args.pid, "status"):
    while (
        init_stats
    ):  # continue updating progress bars until all pids have been popped from init_stats
        pids_to_remove = []
        for pid, stats in init_stats.items():
            # while curr_status := send_and_receive_hypercall(pid, "get_progress_bar_inst_count"):
            curr_status = send_and_receive_hypercall(
                pid, "get_progress_bar_inst_count"
            )
            if curr_status == "ended":
                # gem5_pids.remove(pid)
                pids_to_remove.append(pid)
                # progress_bar_dict[pid].close()
                # continue
                break
            curr_insts = int(json.loads(curr_status))
            progress_bar_dict[pid].update(curr_insts - prev_insts[pid])
            prev_insts[pid] = curr_insts
            time.sleep(0.5)

        for pid in pids_to_remove:
            progress_bar_dict[pid].set_description(
                f" {pid} (exited) | {stats["sim_id"]} | {stats["workload"]}"
            )
            init_stats.pop(pid)
        # if pids_to_remove:
        #     print_dashboard_label()
    # for bar in progress_bar_dict.values():
    #     bar.close()


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

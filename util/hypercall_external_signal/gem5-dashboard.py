import argparse
import json
import logging
import os
import sys
import time

import tqdm
from orchestrator_request import (
    find_gem5_pids,
    send_and_receive_hypercall,
)

from gem5.resources.resource import obtain_resource

logger = logging.getLogger(__name__)


class ProgressBar:
    # bar_pos = 0

    def __init__(
        self, pid, position, max_retries: int = 3, remove_on_exit: bool = False
    ):
        self.pid = pid
        self.prev_insts = 0
        self.position = position
        self.max_retries = max_retries
        self.curr_retries = 0
        self.bar = None
        self.remove_on_exit = remove_on_exit
        self._initialize_bar()

    def _initialize_bar(self):
        # init_stats_str = send_and_receive_hypercall(
        #     pid, "get_progress_bar_init_stats"
        # )
        self.curr_retries += 1
        init_status_str = send_and_receive_hypercall(self.pid, "status")
        print(f"init status string is: {init_status_str}")
        # If the external signal handler times out getting the initial stats,
        # which might happen if a gem5 simulation is launched while the
        # dashboard is running, return without initializing anything so the
        # pid will be excluded from the current loop
        # if init_status_str == "timeout":
        #     return
        try:
            init_status = json.loads(init_status_str)
            # Get workload information
            workload = obtain_resource(init_status["workload"])
            self.max_insts = workload.get_estimated_instructions() or 0
            print(f"self.max_insts: {self.max_insts}")
            # self.total_insts = init_stats["total_insts"]
            self.sim_id = init_status["sim_id"]
            self.workload = init_status["workload"]
            self.curr_tick = init_status["tick"]

            desc_format = f" {self.pid} | {self.sim_id} | {self.workload}"
            self.prog_bar = tqdm.tqdm(
                total=self.max_insts,
                desc=desc_format,
                position=self.position,
                unit="insts",
                leave=self.remove_on_exit,  # need to add an argparse argument to support this + a function that handles exit behavior
            )

            self.prog_bar.set_postfix({"SimTime": self.curr_tick})
            # ProgressBar.bar_pos += 1
            self.curr_retries = 0
        # except json.decoder.JSONDecodeError:
        #     print(f"Failed to load the initial stats for pid {pid}!")
        #     return
        except Exception as e:
            if self.curr_retries < self.max_retries:
                logger.warning(
                    f"Error initializing progress bar for PID {self.pid} (attempt {self.curr_retries}/{self.max_retries}): {e}"
                )
                logger.warning(
                    f"Will retry initialization during next update cycle"
                )
                # Keep active so update() will try again
                self.active = True
            else:
                logger.error(
                    f"Failed to initialize progress bar for PID {self.pid} after {self.max_retries} attempts: {e}"
                )
                self.active = False


class ProgressBarManager:
    def __init__(self, gem5_pids, remove_bar_on_exit: bool = False):
        self.progress_bar_dict = {}
        self.pids_to_remove = []
        self.pids_to_add = []
        self.gem5_pids = gem5_pids
        if not gem5_pids:
            self.auto_add = True
        else:
            self.auto_add = False

    def run(self):
        self._print_dashboard_label()
        while True:
            print("top of dashboard loop")
            self.add_new_processes()
            print(f"added new processes, {self.gem5_pids}")
            self.remove_old_processes()
            print("removed old processes")
            if not self.gem5_pids:
                print("breaking out of dashboard update loop")
                break
            self.update()
            print("updated dashboard")
            time.sleep(5.0)

    # def _initialize_bars(self):

    def add_new_processes(self):
        # if not args.pid:
        if self.auto_add:
            self.pids_to_add = [
                pid for pid in find_gem5_pids() if pid not in self.gem5_pids
            ]
            for pid in self.pids_to_add:
                temp_bar = ProgressBar(pid, len(self.progress_bar_dict))
                # if hasattr(temp_bar, "total_insts"):
                if temp_bar.max_insts > 0:
                    self.progress_bar_dict[pid] = temp_bar
                    self.gem5_pids.append(pid)
            print(f"self.gem5_pids: {self.gem5_pids}")

    def remove_old_processes(self):
        for pid in self.pids_to_remove:
            self.progress_bar_dict[pid].prog_bar.set_description(
                f" {pid} (exited) | {self.progress_bar_dict[pid].sim_id} | {self.progress_bar_dict[pid].workload}"
            )
            self.progress_bar_dict[pid].prog_bar.bar_format = (
                "{desc}: {percentage:3.0f}%|{bar}| {n_fmt}/{total_fmt} [{elapsed}<00:00,{rate_fmt}{postfix}]"
            )
            self.gem5_pids.remove(pid)
        self.pids_to_remove = []

    def update(self):
        for pid, bar_obj in self.progress_bar_dict.items():
            if pid in self.gem5_pids:
                curr_status = send_and_receive_hypercall(
                    pid, "get_progress_bar_inst_count"
                )
                if "ended" in curr_status:
                    self.pids_to_remove.append(pid)
                    continue
                elif "timeout" in curr_status:
                    continue
                curr_insts = int(json.loads(curr_status))
                bar_obj.prog_bar.update(curr_insts - bar_obj.prev_insts)
                bar_obj.prev_insts = curr_insts

    def _print_dashboard_label(self) -> None:
        screen_width = os.get_terminal_size()[0]
        os.system("cls||clear")
        left_justified_label = " pid | simulation id | workload: "
        print(left_justified_label, end="")
        right_justified_label = (
            " current insts / total insts [elapsed<remaining, rate]"
        )
        print(
            right_justified_label.rjust(
                screen_width - len(left_justified_label)
            )
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pid",
        help="Enter the pid of the gem5 process for which you would like to"
        "observe the progress",
    )
    parser.add_argument(
        "--remove-bar-on-exit",
        help="If you would like to remove progress bars for exited simulations",
    )

    args = parser.parse_args()

    # clear screen, print dashboard
    if args.pid:
        if args.pid[0] == "[":
            try:
                gem5_pids = [int(pid) for pid in args.pid[1:-1].split(",")]
            except:
                print("Incorrect format for list of gem5 pids, exiting...")
                sys.exit(1)
        else:
            gem5_pids = [int(args.pid)]
        # pids_to_add = gem5_pids
    else:
        gem5_pids = []
    print(f"In main.py, gem5_pids is: {gem5_pids}")
    print(f"starting progress bar manager")
    manager = ProgressBarManager(gem5_pids, args.remove_bar_on_exit)
    manager.run()

    # while True:

    # if not args.pid:
    #     pids_to_add = [
    #         pid for pid in find_gem5_pids() if pid not in gem5_pids
    #     ]
    # for pid in pids_to_add:
    #     temp_bar = ProgressBar(pid)
    #     if hasattr(temp_bar, "total_insts"):
    #         progress_bar_dict[pid] = temp_bar
    #         gem5_pids.append(pid)

    # for pid in pids_to_remove:
    #     progress_bar_dict[pid].prog_bar.set_description(
    #         f" {pid} (exited) | {progress_bar_dict[pid].sim_id} | {progress_bar_dict[pid].workload}"
    #     )
    #     progress_bar_dict[pid].prog_bar.bar_format = (
    #         "{desc}: {percentage:3.0f}%|{bar}| {n_fmt}/{total_fmt} [{elapsed}<00:00,{rate_fmt}{postfix}]"
    #     )
    #     gem5_pids.remove(pid)
    # pids_to_remove = []
    # if not gem5_pids:
    #     break

    # for pid, bar_obj in progress_bar_dict.items():
    #     if pid in gem5_pids:
    #         curr_status = send_and_receive_hypercall(
    #             pid, "get_progress_bar_inst_count"
    #         )
    #         if "ended" in curr_status:
    #             pids_to_remove.append(pid)
    #             continue
    #         elif "timeout" in curr_status:
    #             continue
    #         curr_insts = int(json.loads(curr_status))
    #         bar_obj.prog_bar.update(curr_insts - bar_obj.prev_insts)
    #         bar_obj.prev_insts = curr_insts


# if __name__ == "__main__":
#     main()

main()

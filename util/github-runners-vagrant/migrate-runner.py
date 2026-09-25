#!/usr/bin/env python3
# Copyright (c) 2023 The Regents of the University of California
# All rights reserved.
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

"""Replace the legacy loop after its current (or next assigned) job exits.

Run as vagrant in a dedicated guest, from a staging directory containing the
new scripts. Only the legacy parent shell is paused. Its listener, worker,
containers, and build processes keep running. An idle listener can accept one
last job; this intentionally favors job completion over immediate downtime.
"""

import fcntl
import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

FILES = (
    "action-run.sh",
    "runner-health.sh",
    "runner-cleanup.py",
    "provision_runner.sh",
    "install-runner-service.sh",
    "gem5-runner.service",
)


def processes():
    found = []
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = path.read_bytes().decode().strip("\0").split("\0")
            status = (path.parent / "status").read_text().splitlines()
            fields = dict(line.split(":", 1) for line in status)
            found.append(
                (
                    int(path.parent.name),
                    args,
                    int(fields["PPid"]),
                    fields["State"].strip()[0],
                )
            )
        except (OSError, ValueError, UnicodeError):
            continue
    return found


def main():
    stage = Path(__file__).resolve().parent
    home = Path.home()
    with (home / "runner-migration.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

        def interrupted(signum, frame):
            raise InterruptedError(f"Migration interrupted by signal {signum}")

        signal.signal(signal.SIGTERM, interrupted)
        signal.signal(signal.SIGHUP, interrupted)
        if (
            subprocess.run(
                ["systemctl", "is-active", "--quiet", "gem5-runner.service"]
            ).returncode
            == 0
        ):
            raise RuntimeError(
                "Runner already uses the service; drain it normally"
            )
        for name in FILES:
            if not (stage / name).is_file():
                raise RuntimeError(f"Missing staged file: {name}")
        loops = [
            (pid, args)
            for pid, args, _, _ in processes()
            if len(args) >= 4
            and Path(args[1]).name == "action-run.sh"
            and Path(args[0]).name == "bash"
        ]
        if len(loops) != 1:
            raise RuntimeError(
                f"Expected exactly one legacy loop; found {len(loops)}"
            )
        pid, args = loops[0]
        backup = (
            home
            / "runner-backups"
            / time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        )
        backup.mkdir(mode=0o700, parents=True, exist_ok=False)
        for name in FILES:
            if (home / name).exists():
                shutil.copy2(home / name, backup / name)
        # Private recovery state. Do not print, upload, or put it in an image.
        recovery = backup / "legacy-arguments.json"
        recovery.write_text(json.dumps(args))
        recovery.chmod(0o600)
        print(f"Backup: {backup}; pausing legacy parent {pid}", flush=True)
        replaced = False
        try:
            os.kill(pid, signal.SIGSTOP)
            while True:
                running = processes()
                current = next((p for p in running if p[0] == pid), None)
                if current is None or current[1] != args:
                    raise RuntimeError(
                        "Legacy controller changed during drain"
                    )
                children = [p for p in running if p[2] == pid and p[3] != "Z"]
                workers = [
                    p
                    for p in running
                    if p[1]
                    and Path(p[1][0]).name
                    in ("Runner.Listener", "Runner.Worker")
                ]
                if not children and not workers:
                    break
                time.sleep(10)
            print(
                "Legacy job and child processes finished; installing",
                flush=True,
            )
            # Kill only the paused shell, after its work has finished.
            os.kill(pid, signal.SIGKILL)
            replaced = True
            for name in FILES:
                temp = home / (name + ".new")
                shutil.copy2(stage / name, temp)
                temp.chmod(0o755)
                temp.replace(home / name)
            token, org = args[2:4]
            labels = args[4] if len(args) > 4 else ""
            for value in (token, org, labels):
                if "\n" in value or "\r" in value:
                    raise RuntimeError("Invalid environment value")
            env = (
                f"PERSONAL_ACCESS_TOKEN={token}\nGITHUB_ORG={org}\n"
                f"RUNNER_LABELS={labels}\n"
                "RUNNER_RESOURCE_CACHE=/gem5-resource-cache\n"
            )
            subprocess.run(
                [
                    "sudo",
                    "-n",
                    "sh",
                    "-c",
                    "umask 077; cat > /etc/gem5-runner.env",
                ],
                input=env,
                text=True,
                check=True,
            )
            # Refresh supplementary groups in case Docker was just provisioned.
            subprocess.run(
                [
                    "sudo",
                    "-n",
                    "-u",
                    "vagrant",
                    str(home / "provision_runner.sh"),
                ],
                check=True,
            )
            for command in ("runner-cleanup.py", "runner-health.sh"):
                subprocess.run(
                    ["sudo", "-n", "-u", "vagrant", str(home / command)],
                    check=True,
                )
            subprocess.run(
                ["sudo", "-n", str(home / "install-runner-service.sh")],
                check=True,
            )
            print(
                "Service installed; inspect runner-state/status and journal",
                flush=True,
            )
        except BaseException:
            if not replaced and any(
                p[0] == pid and p[1] == args for p in processes()
            ):
                os.kill(pid, signal.SIGCONT)
            raise


if __name__ == "__main__":
    main()

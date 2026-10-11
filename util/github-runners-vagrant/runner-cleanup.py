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

"""Clean a dedicated idle runner; retain a bounded cache of common images."""

import json
import os
import shutil
import subprocess
from pathlib import Path


def docker(*args):
    return subprocess.check_output(["docker", *args], text=True).strip()


def cleanup():
    os.chdir(Path(__file__).resolve().parent)
    worker_status = subprocess.run(
        ["pgrep", "-x", "Runner.Worker"], stdout=subprocess.DEVNULL
    ).returncode
    if worker_status != 1:
        raise RuntimeError(
            "Refusing cleanup: a worker exists or its state is unavailable"
        )
    subprocess.run(["sudo", "-n", "rm", "-rf", "--", "_work"], check=True)
    containers = docker("ps", "-aq").split()
    if containers:
        docker("rm", "-f", *containers)
    docker("volume", "prune", "--all", "--force")
    docker("network", "prune", "--force")
    docker("builder", "prune", "--all", "--force", "--keep-storage", "4GB")
    ids = list(dict.fromkeys(docker("image", "ls", "-aq").split()))
    images = json.loads(docker("image", "inspect", *ids)) if ids else []
    health = docker(
        "image",
        "inspect",
        "--format",
        "{{.Id}}",
        os.environ.get("HEALTH_IMAGE", "ubuntu:24.04"),
    )
    retained = []
    for image in images:
        tags = image.get("RepoTags") or []
        if image["Id"] == health or any(
            tag.startswith("ghcr.io/gem5/") for tag in tags
        ):
            retained.append(image)
        else:
            docker("image", "rm", "--force", image["Id"])
    # Sum full image sizes conservatively: shared layers count more than once.
    # This is a byte cap, not Docker's 'until' (image creation time) filter.
    cap = int(os.environ.get("RUNNER_IMAGE_CACHE_GIB", "32")) * 1024**3
    floor = int(os.environ.get("RUNNER_MIN_FREE_GIB", "20")) * 1024**3
    size = sum(image["Size"] for image in retained)
    for image in sorted(retained, key=lambda item: item["Created"]):
        if size <= cap and shutil.disk_usage(".").free >= floor:
            break
        if image["Id"] == health:
            continue
        docker("image", "rm", "--force", image["Id"])
        size -= image["Size"]
    print(f"Retained image size (shared layers counted repeatedly): {size}")
    # Diagnostics are rotated only after jobs, preserving recent failure logs.
    import time

    for log in Path("_diag").glob("*.log"):
        if log.stat().st_mtime < time.time() - 7 * 86400:
            log.unlink()


if __name__ == "__main__":
    cleanup()

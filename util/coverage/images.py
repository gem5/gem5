# Copyright (c) 2026 The Regents of The University of California
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

"""Resolve the Linux x86-64 images used by a coverage campaign."""

import argparse
import json
import subprocess
from pathlib import Path


def inspect(image, field):
    return json.loads(
        subprocess.check_output(
            [
                "docker",
                "buildx",
                "imagetools",
                "inspect",
                image,
                "--format",
                "{{json ." + field + "}}",
            ],
            text=True,
        )
    )


def pin(image):
    manifest = inspect(image, "Manifest")
    if "manifests" in manifest:
        matches = [
            entry["digest"]
            for entry in manifest["manifests"]
            if entry.get("platform", {}).get("os") == "linux"
            and entry.get("platform", {}).get("architecture") == "amd64"
        ]
        if len(matches) != 1:
            raise ValueError(f"Expected one Linux amd64 image: {image}")
        digest = matches[0]
    else:
        config = inspect(image, "Image")
        if (config.get("os"), config.get("architecture")) != (
            "linux",
            "amd64",
        ):
            raise ValueError(f"Expected a Linux amd64 image: {image}")
        digest = manifest["digest"]
    return image.removesuffix(":latest") + "@" + digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    catalog = Path(__file__).resolve().parents[1] / "ci/native.json"
    images = {item["image"] for item in json.loads(catalog.read_text())}
    args.output.write_text(
        json.dumps({image: pin(image) for image in sorted(images)}) + "\n"
    )


if __name__ == "__main__":
    main()

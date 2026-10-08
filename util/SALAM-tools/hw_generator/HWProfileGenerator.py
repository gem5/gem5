# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

import argparse
import os

from SALAMGeneratedSources import (
    generate_sources,
    load_generated_sources,
    load_source_config,
    require_compatible_functional_units,
    write_content_files,
)


def repo_root():
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..")
    )


def export_hw_sources(config_path, output_dir):
    destination = os.path.abspath(output_dir)
    source_root = os.path.join(repo_root(), "src")
    if os.path.commonpath([source_root, destination]) == source_root:
        raise RuntimeError(
            "refusing to write generated SALAM sources under src: "
            + destination
        )
    spec = load_source_config(os.path.abspath(config_path))
    model = load_generated_sources(spec)
    require_compatible_functional_units(model)
    write_content_files(destination, generate_sources(model))


def main():
    parser = argparse.ArgumentParser(
        description="Export generated SALAM sources to an output directory"
    )
    parser.add_argument(
        "--hw-config",
        type=str,
        required=True,
        help="Complete hardware-profile YAML",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        required=True,
        help="Directory for generated sources; must not be under src/",
    )
    args = parser.parse_args()
    export_hw_sources(args.hw_config, args.output_dir)


if __name__ == "__main__":
    main()

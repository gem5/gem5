# Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
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

"""Read hardware-profile expectations in an isolated host Python process."""

import argparse
import dataclasses
import hashlib
import json
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("generated", "profile"))
    parser.add_argument("generator_dir")
    parser.add_argument("profile")
    args = parser.parse_args()

    sys.path.insert(0, args.generator_dir)
    import SALAMGeneratedSources as generator

    model = generator.load_generated_sources(
        generator.load_source_config(args.profile)
    )
    if args.kind == "generated":
        outputs = generator.generate_sources(model)
        result = {
            name: hashlib.sha256(text.encode()).hexdigest()
            for name, text in outputs.items()
        }
    else:
        metadata = {
            "path",
            "instruction_names",
            "characterization",
            "characterization_path",
        }
        units = [
            (
                unit.alias,
                {
                    name: value
                    for name, value in dataclasses.asdict(unit).items()
                    if name not in metadata
                },
            )
            for unit in model.functional_units
        ]
        instructions = [
            (
                instruction.name,
                {
                    name: value
                    for name, value in dataclasses.asdict(instruction).items()
                    if name != "name"
                },
            )
            for instruction in model.instructions
        ]
        result = dict(
            profile=args.profile,
            functional_units=units,
            instructions=instructions,
            characterization=generator.power_model.merge(
                unit.characterization for unit in model.functional_units
            ),
        )
    print(json.dumps(result))


if __name__ == "__main__":
    main()

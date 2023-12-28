# Copyright (c) 2023 The Regents of the University of California
# All Rights Reserved.
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

"""
Obtain a resource from gem5 resource.

Usage
-----

```sh
scons build/ALL/gem5.opt -j$(nproc)
build/ALL/gem5.opt util/obtain-resource.py <resource_id> [-p <path>] [-q]
# Example:
# `build/ALL/gem5.opt util/obtain-resource.py arm-hello64-static -p arm-hello`
# This will download the resource with id `arm-hello64-static` to the
# "arm-hello" in the CWD.
```
"""

if __name__ == "__m5_main__":
    import argparse

    from gem5.resources.resource import obtain_resource

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "id",
        type=str,
        help="The resource id to download.",
    )

    parser.add_argument(
        "-p",
        "--path",
        type=str,
        required=False,
        help="The path the resource is to be downloaded to. If not specified, "
        "the resource will be downloaded to the default location in the "
        "gem5 local cache of resources",
    )

    parser.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        default=False,
        help="Suppress output.",
    )

    args = parser.parse_args()

    resource = obtain_resource(
        resource_id=args.id,
        quiet=args.quiet,
        to_path=args.path,
    )

    if not args.quiet:
        print(f"Resource at: '" + str(resource.get_local_path()) + "'")

    exit(0)

print("Error: This script is meant to be run with the gem5 binary")
exit(1)

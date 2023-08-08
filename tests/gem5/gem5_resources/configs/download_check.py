# Copyright (c) 2022 The Regents of the University of California
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

from gem5.resources.downloader import (
    list_resources,
    get_resource,
)

from gem5.resources.client import get_resource_json_obj

from gem5.resources.md5_utils import md5

import os
import shutil
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(
    description="A script that will checks that input resource IDs will "
    "download a resource and that resources md5 value is correct. "
    "If no resource IDs are specified, all will be checked."
)

parser.add_argument(
    "ids",
    nargs="*",  # Accepts 0 or more arguments.
    type=str,
    help="The resource IDs to check. If not set, all resources will be "
    "checked",
)

parser.add_argument(
    "--skip",
    nargs="+",  # Accepts 1 or more arguments.
    type=str,
    help="The resource IDs to skip. If not set, no resources will be skipped.",
    required=False,
)

parser.add_argument(
    "--gem5-version",
    type=str,
    required=False,
    help="The gem5 version to check the resources against. Resources not "
    "compatible with this version will be ignored. If not set, no "
    "compatibility tests are performed.",
)

parser.add_argument(
    "--download-directory",
    type=str,
    required=True,
    help="The directory to download the resources as part of these test. The "
    "contents of this directory will be wiped after running the tests.",
)

args = parser.parse_args()

# If the directory doesn't exist, create it.
if not Path(args.download_directory).exists():
    os.makedirs(args.download_directory)


ids = args.ids
resource_list = list_resources(gem5_version=args.gem5_version)
if len(ids) == 0:
    ids = resource_list

# We log all the errors as they occur then dump them at the end. This means we
# can be aware of all download errors in a single failure.
errors = str()

for id in ids:
    if args.skip and id in args.skip:
        continue
    if id not in resource_list:
        errors += (
            f"Resource with ID '{id}' not found in "
            + f"`list_resources()`.{os.linesep}"
        )
        continue

    for resource_version in ids[id]:
        resource_json = get_resource_json_obj(
            resource_id=id,
            resource_version=resource_version,
            gem5_version=args.gem5_version,
        )
        if resource_json["category"] == "workload":
            # Workloads are not downloaded as part of this test.
            continue
        download_path = os.path.join(
            args.download_directory, f"{id}-v{resource_version}"
        )
        try:
            get_resource(
                resource_name=id,
                resource_version=resource_version,
                gem5_version=args.gem5_version,
                to_path=download_path,
            )
        except Exception as e:
            errors += (
                f"Failure to download resource '{id}', "
                + f"v{resource_version}.{os.linesep}"
            )
            errors += f"Exception message:{os.linesep}{str(e)}"
            errors += f"{os.linesep}{os.linesep}"
            continue

        if md5(Path(download_path)) != resource_json["md5sum"]:
            errors += (
                f"Downloaded resource '{id}' md5 "
                + f"({md5(Path(download_path))}) differs to that recorded in "
                + f" gem5-resources ({resource_json['md5sum']}).{os.linesep}"
            )
        # Remove the downloaded resource.
        if os.path.isfile(download_path):
            os.remove(download_path)
        elif os.path.isdir(download_path):
            shutil.rmtree(download_path, ignore_errors=True)
        else:
            raise Exception("{download_path} is not a file or directory.")


# If errors exist, raise an exception highlighting them.
if errors:
    raise Exception(errors)

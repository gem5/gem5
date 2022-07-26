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
    get_resources_json_obj,
    get_resource,
)

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
if len(ids) == 0:
    ids = list_resources()

# We log all the errors as they occur then dump them at the end. This means we
# can be aware of all download errors in a single failure.
errors = str()

for id in ids:
    if id not in list_resources():
        errors += (
            f"Resource with ID '{id}' not found in "
            + f"`list_resources()`.{os.linesep}"
        )
        continue

    resource_json = get_resources_json_obj(id)
    download_path = os.path.join(args.download_directory, id)
    try:
        get_resource(resource_name=id, to_path=download_path)
    except Exception as e:
        errors += f"Failure to download resource '{id}'.{os.linesep}"
        errors += f"Exception message:{os.linesep}{str(e)}"
        errors += f"{os.linesep}{os.linesep}"
        continue

    if md5(Path(download_path)) != resource_json["md5sum"]:
        errors += (
            f"Downloaded resource '{id}' md5 "
            + f"({md5(Path(download_path))}) differs to that in the "
            + f"JSON ({resource_json['md5sum']}).{os.linesep}"
        )

    # Remove the downloaded resource.
    shutil.rmtree(download_path, ignore_errors=True)

# If errors exist, raise an exception highlighting them.
if errors:
    raise Exception(errors)

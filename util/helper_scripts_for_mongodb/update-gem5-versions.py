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

"""
 This script grabs all resources categorically from a specified collection in a MongoDB
 database and adds a new gem5 version to the gem5_versions field of each
 resource.

 To run this script you use the following command:
 python3 ./update-gem5-versions.py --uri <uri> --db <db_name> /
    --collection <collection_name> --version <version> /
    --category <category> --outfile <outfile>
"""
import argparse
import json

from bson import json_util
from helper import (
    get_database,
    save_to_json,
)

parser = argparse.ArgumentParser(
    description="Get all resources from a "
    "specified collection in a MongoDB database "
    "and add a new gem5 version to the "
    "gem5_versions field of each resource."
)
parser.add_argument(
    "--uri", help="URI of the database", type=str, required=True
)
parser.add_argument(
    "--version",
    help="Version to add to gem5_versions",
    type=str,
    required=True,
)
parser.add_argument(
    "--db", help="Name of the database", type=str, default="gem5-vision"
)
parser.add_argument(
    "--collection",
    help="Name of the collection",
    type=str,
    default="resources",
)
parser.add_argument(
    "--category", help="Category to not update", action="append"
)
parser.add_argument(
    "--outfile",
    help="Name of the output json file",
    type=str,
    default="resources_update_gem5_versions.json",
)

if __name__ == "__main__":
    args = parser.parse_args()

    # if no category is excluded, set category to empty list
    if args.category is None:
        args.category = []

    collection = get_database(args.uri, args.db, args.collection)

    # get all documents from resources collection
    resources = collection.find({}, {"_id": 0})
    resources = json.loads(json_util.dumps(resources))

    for resource in resources:
        if resource["category"] in args.category:
            continue
        if args.version in resource["gem5_versions"]:
            continue
        resource["gem5_versions"].append(args.version)

    save_to_json(resources, args.outfile)

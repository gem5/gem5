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
 This script adds a list of resources from a JSON file to a specified
 collection in a MongoDB database. The JSON file should be in the format
 of a list of dictionaries, where each dictionary represents a resource.

 To run this script you use the following command:
 python3 ./add-json-to-mongo.py --uri <uri> --db_name <db_name> /
    --collection_name <collection_name> --json_file <json_file>
"""

import argparse
import json

from helper import get_database

parser = argparse.ArgumentParser(
    description="Add a list of resources from a "
    "JSON file to a specified collection in a "
    "MongoDB database"
)
parser.add_argument(
    "--uri", help="URI of the database", type=str, required=True
)
parser.add_argument(
    "--db_name", help="Name of the database", type=str, default="gem5-vision"
)
parser.add_argument(
    "--collection_name",
    help="Name of the collection",
    type=str,
    default="resources",
)
parser.add_argument(
    "--json_file", help="Name of the json file", type=str, required=True
)

if __name__ == "__main__":
    args = parser.parse_args()

    uri = args.uri
    db_name = args.db_name
    collection_name = args.collection_name
    json_file = args.json_file

    # get resources from json file
    with open(json_file) as f:
        resources = json.load(f)

    collection = get_database(uri, db_name, collection_name)

    # insert resources into collection
    collection.insert_many(resources)

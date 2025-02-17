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
This script grabs all documents from a specified collection in a MongoDB
database and creates a new collection with the same documents.

To run this script you use the following command:
python3 ./create-new-collection.py --uri <uri> --db_name <db_name> --collection_name <collection_name> /
   --new_collection_name <new_collection_name>
"""
import argparse
import json

from bson import json_util
from helper import get_database

parser = argparse.ArgumentParser(
    description="Get all documents from a "
    "specified collection in a MongoDB database "
    "and create a new collection with the same "
    "documents."
)
parser.add_argument(
    "--uri", help="URI of the database", type=str, required=True
)
parser.add_argument(
    "--new_collection_name",
    help="Name of the new collection",
    type=str,
    required=True,
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

args = parser.parse_args()

uri = args.uri
db_name = args.db_name
collection_name = args.collection_name
new_collection_name = args.new_collection_name


collection = get_database(uri, db_name, collection_name)

new_collection = get_database(uri, db_name, new_collection_name)

# get all documents from collection
# Suppress _id field from being returned
# as new collection will have its own _id field
resources = collection.find({}, {"_id": 0})

# convert to json
resources = json.loads(json_util.dumps(resources))

new_collection.insert_many(resources)

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
This is a script to get all resources from the database and save them to
a json file and download all resources to a directory. The resources in
the resources.json file that is generated will be updated to point to the
local resources instead of the database resources. This script is used to
create the offline database. To use the resources.json with gem5 you need
to update the gem5 config to point to the resources.json file.
"""

import argparse
import json
from urllib import request, parse
import pathlib
from typing import List
import hashlib
from tqdm import tqdm


def get_token(auth_url: str, api_key: str) -> str:
    """
    This function gets the token from the database using the api key
    :param auth_url: Authentication url to use Atlas data API
    :param api_key: API key to access the database
    :return: Token to access the database
    """

    data = {"key": api_key}
    data = json.dumps(data).encode("utf-8")

    req = request.Request(
        auth_url, data=data, headers={"content-type": "application/json"}
    )

    try:
        response = request.urlopen(req)
    except Exception as e:
        raise Exception(f"Failed to get token: {e}")

    token = json.loads(response.read().decode("utf-8"))["access_token"]
    return token


def get_all_resources(
    url: str, data_source: str, collection: str, database: str, token: str
) -> List:
    """
    This function gets all the JSON objects for resources from the database
    :param url: Database url to use Atlas data API
    :param data_source: Data source name for the mongoDB database
    :param collection: Collection name for the mongoDB database
    :param database: Database name for the mongoDB database
    :param token: Token to access the database

    :return: List of JSON objects for resources
    """

    url = f"{url}/action/find"
    data = {
        "dataSource": data_source,
        "collection": collection,
        "database": database,
    }
    data = json.dumps(data).encode("utf-8")
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
    }

    req = request.Request(url, data=data, headers=headers)

    try:
        response = request.urlopen(req)
    except Exception as e:
        raise Exception(f"Failed to get resources: {e}")

    return json.loads(response.read().decode("utf-8"))["documents"]


def save_resources_to_file(resources: List, output: pathlib.Path):
    """
    This function saves all the JSON objects for resources to a file.
    :param resources: List of JSON objects for resources
    :param output: Output directory absolute path
    """

    path = output.joinpath("resources.json")

    with open(path, "w") as f:
        json.dump(resources, f, indent=4)


def progress_hook(t):
    last_b = [0]

    def update_to(b=1, bsize=1, tsize=None):
        if tsize not in (None, -1):
            t.total = tsize
        displayed = t.update((b - last_b[0]) * bsize)
        last_b[0] = b
        return displayed

    return update_to


def download_resources(output: pathlib.Path):
    """
    This function downloads the resources which have a url field and
    updates the url field to point to the local download of the resource.
    :param output: Output directory absolute path
    """

    path = output.joinpath("offline_resources")
    if not path.exists():
        path.mkdir()

    with open(output.joinpath("resources.json")) as f:
        resources = json.load(f)

    for resource in resources:
        if "url" in resource.keys():
            url = resource["url"]
            filename = pathlib.Path(
                *pathlib.Path(parse.urlsplit(url).path).parts[3:]
            )
            filepath = pathlib.Path(path).joinpath(filename)
            filepath.parent.mkdir(parents=True, exist_ok=True)

            if (
                not filepath.exists()
                or hashlib.md5(filepath.read_bytes()).hexdigest()
                != resource["md5sum"]
            ):
                with tqdm(
                    unit="B",
                    unit_scale=True,
                    unit_divisor=1024,
                    miniters=1,
                    desc=f"Downloading {url}",
                ) as t:
                    request.urlretrieve(url, filepath, progress_hook(t))
            resource["url"] = filepath.absolute().as_uri()

    with open(output.joinpath("resources.json"), "w") as f:
        json.dump(resources, f, indent=4)


if __name__ == "__main__" or __name__ == "__m5_main__":
    parser = argparse.ArgumentParser(
        description="Get resources from the database"
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=pathlib.Path.cwd(),
        help="Output directory absolute path, default is the cwd."
        "The resources.json and all resources will be saved in this directory",
    )
    args = parser.parse_args()

    output_path = pathlib.Path(args.output_dir)

    # Get the gem5 config from the stable branch of gem5
    gem5_config = request.urlopen(
        "https://raw.githubusercontent.com/gem5/gem5/"
        "stable/src/python/gem5_default_config.py"
    )
    gem5_config = gem5_config.read().decode("utf-8").split("=")[-1]
    gem5_config = eval(gem5_config)
    gem5_config = gem5_config["sources"]["gem5-resources"]

    # Parse the gem5 config
    db_url = gem5_config["url"]
    data_source = gem5_config["dataSource"]
    collection_name = gem5_config["collection"]
    db_name = gem5_config["database"]
    auth_url = gem5_config["authUrl"]
    api_key = gem5_config["apiKey"]

    if not output_path.exists():
        output_path = output_path.mkdir()

    token = get_token(auth_url, api_key)

    resources = get_all_resources(
        db_url,
        data_source,
        collection_name,
        db_name,
        token,
    )

    save_resources_to_file(
        resources,
        output_path,
    )

    download_resources(output_path)

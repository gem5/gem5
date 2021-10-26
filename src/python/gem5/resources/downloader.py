# Copyright (c) 2021 The Regents of the University of California
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

import json
import urllib.request
import hashlib
import os
import shutil
import gzip
import hashlib
import base64
from typing import List, Dict

from ..utils.filelock import FileLock

"""
This Python module contains functions used to download, list, and obtain
information about resources from resources.gem5.org.
"""


def _get_resources_json_uri() -> str:
    # TODO: This is hardcoded to develop. This will need updated for each
    # release to the stable branch.
    uri = (
        "https://gem5.googlesource.com/public/gem5-resources/"
        + "+/refs/heads/develop/resources.json?format=TEXT"
    )

    return uri


def _get_resources_json() -> Dict:
    """
    Gets the Resources JSON.

    :returns: The Resources JSON (as a Python Dictionary).
    """

    # Note: Google Source does not properly support obtaining files as raw
    # text. Therefore when we open the URL we receive the JSON in base64
    # format. Conversion is needed before it can be loaded.
    with urllib.request.urlopen(_get_resources_json_uri()) as url:
        return json.loads(base64.b64decode(url.read()).decode("utf-8"))


def _get_url_base() -> str:
    """
    Obtains the "url_base" string from the resources.json file.

    :returns: The "url_base" string value from the resources.json file.
    """
    json = _get_resources_json()
    if "url_base" in json.keys():
        return json["url_base"]
    return ""


def _get_resources(resources_group: Dict) -> Dict[str, Dict]:
    """
    A recursive function to get all the resources.

    :returns: A dictionary of resource names to the resource JSON objects.
    """

    to_return = {}
    for resource in resources_group:
        # 'artifact' is the old naming, we keep it here for
        # backwards compatibility, but it can be removed with time:
        # https://gem5-review.googlesource.com/c/public/gem5-resources/+/51169.
        if resource["type"] == "artifact" or resource["type"] == "resource":
            # If the type is "resource" then we add it directly to the map
            # after a check that the name is unique.
            if resource["name"] in to_return.keys():
                raise Exception(
                    "Error: Duplicate resource with name '{}'.".format(
                        resource["name"]
                    )
                )
            to_return[resource["name"]] = resource
        elif resource["type"] == "group":
            # If it's a group we get recursive. We then check to see if there
            # are any duplication of keys.
            new_map = _get_resources(resource["contents"])
            intersection = set(new_map.keys()).intersection(to_return.keys())
            if len(intersection) > 0:
                # Note: if this error is received it's likely an error with
                # the resources.json file. The resources names need to be
                # unique keyes.
                raise Exception(
                    "Error: Duplicate resources with names: {}.".format(
                        str(intersection)
                    )
                )
            to_return.update(new_map)
        else:
            raise Exception(
                "Error: Unknown type '{}'.".format(resource["type"])
            )

    return to_return


def _get_md5(file: str) -> str:
    """
    Gets the md5 of a file.

    :param file: The file needing an md5 value.

    :returns: The md5 of the input file.
    """

    # Note: This code is slightly more complex than you might expect as
    # `hashlib.md5(<file>)` returns malloc errors for large files (such as
    # disk images).
    md5_object = hashlib.md5()
    block_size = 128 * md5_object.block_size
    a_file = open(file, "rb")
    chunk = a_file.read(block_size)

    while chunk:
        md5_object.update(chunk)
        chunk = a_file.read(block_size)

    return md5_object.hexdigest()


def _download(url: str, download_to: str) -> None:
    """
    Downloads a file.

    :param url: The URL of the file to download.

    :param download_to: The location the downloaded file is to be stored.
    """

    # TODO: This whole setup will only work for single files we can get via
    # wget. We also need to support git clones going forward.
    urllib.request.urlretrieve(url, download_to)


def list_resources() -> List[str]:
    """
    Lists all available resources by name.

    :returns: A list of resources by name.
    """
    return _get_resources(_get_resources_json()["resources"]).keys()


def get_resources_json_obj(resource_name: str) -> Dict:
    """
    Get a JSON object of a specified resource.

    :param resource_name: The name of the resource.

    :returns: The JSON object (in the form of a dictionary).

    :raises Exception: An exception is raised if the specified resources does
    not exist.
    """
    resource_map = _get_resources(_get_resources_json()["resources"])

    if resource_name not in resource_map:
        raise Exception(
            "Error: Resource with name '{}' does not exist".format(
                resource_name
            )
        )

    return resource_map[resource_name]


def get_resource(
    resource_name: str,
    to_path: str,
    unzip: bool = True,
    download_md5_mismatch: bool = True,
) -> None:
    """
    Obtains a gem5 resource and stored it to a specified location. If the
    specified resource is already at the location, no action is taken.

    :param resource_name: The resource to be obtained.

    :param to_path: The location in the file system the resource is to be
    stored. The filename should be included.

    :param unzip: If true, gzipped resources will be unzipped prior to saving
    to `to_path`. True by default.

    :param download_md5_mismatch: If a resource is present with an incorrect
    hash (e.g., an outdated version of the resource is present), `get_resource`
    will delete this local resource and re-download it if this parameter is
    True. True by default.

    :raises Exception: An exception is thrown if a file is already present at
    `to_path` but it does not have the correct md5 sum. An exception will also
    be thrown is a directory is present at `to_path`
    """

    # We apply a lock for a specific resource. This is to avoid circumstances
    # where multiple instances of gem5 are running and trying to obtain the
    # same resources at once. The timeout here is somewhat arbitarily put at 15
    # minutes.Most resources should be downloaded and decompressed in this
    # timeframe, even on the most constrained of systems.
    with FileLock("{}.lock".format(to_path), timeout=900):

        resource_json = get_resources_json_obj(resource_name)

        if os.path.exists(to_path):

            if not os.path.isfile(to_path):
                raise Exception(
                    "There is a directory at '{}'.".format(to_path)
                )

            if _get_md5(to_path) == resource_json["md5sum"]:
                # In this case, the file has already been download, no need to
                # do so again.
                return
            elif download_md5_mismatch:
                os.remove(to_path)
            else:
                raise Exception(
                    "There already a file present at '{}' but "
                    "its md5 value is invalid.".format(to_path)
                )

        download_dest = to_path

        # This if-statement is remain backwards compatable with the older,
        # string-based way of doing things. It can be refactored away over
        # time:
        # https://gem5-review.googlesource.com/c/public/gem5-resources/+/51168
        if isinstance(resource_json["is_zipped"], str):
            run_unzip = unzip and resource_json["is_zipped"].lower() == "true"
        elif isinstance(resource_json["is_zipped"], bool):
            run_unzip = unzip and resource_json["is_zipped"]
        else:
            raise Exception(
                "The resource.json entry for '{}' has a value for the "
                "'is_zipped' field which is neither a string or a boolean."
                .format(
                    resource_name
                )
            )

        if run_unzip:
            download_dest += ".gz"

        # TODO: Might be nice to have some kind of download status bar here.
        # TODO: There might be a case where this should be silenced.
        print(
            "Resource '{}' was not found locally. Downloading to '{}'..."
            .format(
                resource_name, download_dest
            )
        )

        # Get the URL. The URL may contain '{url_base}' which needs replaced
        # with the correct value.
        url = resource_json["url"].format(url_base=_get_url_base())

        _download(url=url, download_to=download_dest)
        print("Finished downloading resource '{}'.".format(resource_name))

        if run_unzip:
            print(
                "Decompressing resource '{}' ('{}')...".format(
                    resource_name, download_dest
                )
            )
            with gzip.open(download_dest, "rb") as f:
                with open(to_path, "wb") as o:
                    shutil.copyfileobj(f, o)
            os.remove(download_dest)
            print(
                "Finished decompressing resource '{}'.".format(resource_name)
            )

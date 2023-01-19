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
import urllib.parse
import hashlib
import os
import shutil
import gzip
import hashlib
import base64
import time
import random
from pathlib import Path
import tarfile
from tempfile import gettempdir
from urllib.error import HTTPError
from typing import List, Dict, Set, Optional

from .md5_utils import md5_file, md5_dir

from ..utils.filelock import FileLock

"""
This Python module contains functions used to download, list, and obtain
information about resources from resources.gem5.org.
"""


def _resources_json_version_required() -> str:
    """
    Specifies the version of resources.json to obtain.
    """
    return "develop"


def _get_resources_json_uri() -> str:
    return "https://resources.gem5.org/resources.json"


def _url_validator(url):
    try:
        result = urllib.parse.urlparse(url)
        return all([result.scheme, result.netloc, result.path])
    except:
        return False


def _get_resources_json_at_path(path: str, use_caching: bool = True) -> Dict:
    """
    Returns a resource JSON, in the form of a Python Dict. The location
    of the JSON must be specified.

    If `use_caching` is True, and a URL is passed, a copy of the JSON will be
    cached locally, and used for up to an hour after retrieval.

    :param path: The URL or local path of the JSON file.
    :param use_caching: True if a cached file is to be used (up to an hour),
    otherwise the file will be retrieved from the URL regardless. True by
    default. Only valid in cases where a URL is passed.
    """

    # If a local valid path is passed, just load it.
    if Path(path).is_file():
        return json.load(open(path))

    # If it's not a local path, it should be a URL. We check this here and
    # raise an Exception if it's not.
    if not _url_validator(path):
        raise Exception(
            f"Resources location '{path}' is not a valid path or URL."
        )

    download_path = os.path.join(
        gettempdir(),
        f"gem5-resources-{hashlib.md5(path.encode()).hexdigest()}"
        f"-{str(os.getuid())}.json",
    )

    # We apply a lock on the resources file for when it's downloaded, or
    # re-downloaded, and read. This stops a corner-case from occuring where
    # the file is re-downloaded while being read by another gem5 thread.
    # Note the timeout is 120 so the `_download` function is given time to run
    # its Truncated Exponential Backoff algorithm
    # (maximum of roughly 1 minute). Typically this code will run quickly.
    with FileLock("{}.lock".format(download_path), timeout=120):

        # The resources.json file can change at any time, but to avoid
        # excessive retrieval we cache a version locally and use it for up to
        # an hour before obtaining a fresh copy.
        #
        # `time.time()` and `os.path.getmtime(..)` both return an unix epoch
        # time in seconds. Therefore, the value of "3600" here represents an
        # hour difference between the two values. `time.time()` gets the
        # current time, and `os.path.getmtime(<file>)` gets the modification
        # time of the file. This is the most portable solution as other ideas,
        # like "file creation time", are  not always the same concept between
        # operating systems.
        if (
            not use_caching
            or not os.path.exists(download_path)
            or (time.time() - os.path.getmtime(download_path)) > 3600
        ):
            _download(path, download_path)

    with open(download_path) as f:
        file_contents = f.read()

    try:
        to_return = json.loads(file_contents)
    except json.JSONDecodeError:
        # This is a bit of a hack. If the URL specified exists in a Google
        # Source repo (which is the case when on the gem5 develop branch) we
        # retrieve the JSON in base64 format. This cannot be loaded directly as
        # text. Conversion is therefore needed.
        to_return = json.loads(base64.b64decode(file_contents).decode("utf-8"))

    return to_return


def _get_resources_json() -> Dict:
    """
    Gets the Resources JSON.

    :returns: The Resources JSON (as a Python Dictionary).
    """

    path = os.getenv("GEM5_RESOURCE_JSON", _get_resources_json_uri())
    to_return = _get_resources_json_at_path(path=path)

    # If the current version pulled is not correct, look up the
    # "previous-versions" field to find the correct one.
    # If the resource JSON file does not have a "version" field or it's
    # null/None, then we will use this resource JSON file (this is usefull for
    # testing purposes).
    version = _resources_json_version_required()
    json_version = None if "version" not in to_return else to_return["version"]

    if json_version and json_version != version:
        if version in to_return["previous-versions"].keys():
            to_return = _get_resources_json_at_path(
                path=to_return["previous-versions"][version]
            )
        else:
            # This should never happen, but we thrown an exception to explain
            # that we can't find the version.
            raise Exception(
                f"Version '{version}' of resources.json cannot be found."
            )

    return to_return


def _get_url_base() -> str:
    """
    Obtains the "url_base" string from the resources.json file.

    :returns: The "url_base" string value from the resources.json file.
    """
    json = _get_resources_json()
    if "url_base" in json.keys():
        return json["url_base"]
    return ""


def _get_resources(
    valid_types: Set[str], resources_group: Optional[Dict] = None
) -> Dict[str, Dict]:
    """
    A recursive function to get all the workload/resource of the specified type
    in the resources.json file.

    :param valid_types: The type to return (i.e., "resource" or "workload).
    :param resource_group: Used for recursion: The current resource group being
    iterated through.

    :returns: A dictionary of artifact names to the resource JSON objects.
    """

    if resources_group is None:
        resources_group = _get_resources_json()["resources"]

    to_return = {}
    for resource in resources_group:
        if resource["type"] in valid_types:
            # If the type is valid then we add it directly to the map
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
            new_map = _get_resources(
                valid_types=valid_types, resources_group=resource["contents"]
            )
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

    return to_return


def _download(url: str, download_to: str, max_attempts: int = 6) -> None:
    """
    Downloads a file.

    The function will run a Truncated Exponential Backoff algorithm to retry
    the download if the HTTP Status Code returned is deemed retryable.

    :param url: The URL of the file to download.

    :param download_to: The location the downloaded file is to be stored.

    :param max_attempts: The max number of download attempts before stopping.
    The default is 6. This translates to roughly 1 minute of retrying before
    stopping.
    """

    # TODO: This whole setup will only work for single files we can get via
    # wget. We also need to support git clones going forward.

    attempt = 0
    while True:
        # The loop will be broken on a successful download, via a `return`, or
        # if an exception is raised. An exception will be raised if the maximum
        # number of download attempts has been reached or if a HTTP status code
        # other than 408, 429, or 5xx is received.
        try:
            # check to see if user requests a proxy connection
            use_proxy = os.getenv("GEM5_USE_PROXY")
            if use_proxy:
                # If the "use_proxy" variable is specified we setup a socks5
                # connection.

                import socks
                import socket
                import ssl

                IP_ADDR, host_port = use_proxy.split(":")
                PORT = int(host_port)
                socks.set_default_proxy(socks.SOCKS5, IP_ADDR, PORT)
                socket.socket = socks.socksocket

                # base SSL context for https connection
                ctx = ssl.create_default_context()
                ctx.check_hostname = False
                ctx.verify_mode = ssl.CERT_NONE

                # get the file as a bytes blob
                request = urllib.request.Request(url)
                with urllib.request.urlopen(request, context=ctx) as fr:
                    with open(download_to, "wb") as fw:
                        fw.write(fr.read())
            else:
                urllib.request.urlretrieve(url, download_to)
            return
        except HTTPError as e:
            # If the error code retrieved is retryable, we retry using a
            # Truncated Exponential backoff algorithm, truncating after
            # "max_attempts". We consider HTTP status codes 408, 429, and 5xx
            # as retryable. If any other is retrieved we raise the error.
            if e.code in (408, 429) or 500 <= e.code < 600:
                attempt += 1
                if attempt >= max_attempts:
                    raise Exception(
                        f"After {attempt} attempts, the resource json could "
                        "not be retrieved. HTTP Status Code retrieved: "
                        f"{e.code}"
                    )
                time.sleep((2**attempt) + random.uniform(0, 1))
            else:
                raise e
        except ValueError as e:
            raise Exception(
                "Environment variable GEM5_USE_PROXY is set to "
                f"'{use_proxy}'. The expected form is "
                "<host>:<port>'."
            )
        except ImportError as e:
            raise Exception(
                "An import error has occurred. This is likely due "
                "the Python SOCKS client module not being "
                "installed. It can be installed with "
                "`pip install PySocks`."
            )


def list_resources() -> List[str]:
    """
    Lists all available resources by name.

    :returns: A list of resources by name.
    """
    from .resource import _get_resource_json_type_map

    return _get_resources(
        valid_types=_get_resource_json_type_map.keys()
    ).keys()


def get_workload_json_obj(workload_name: str) -> Dict:
    """
    Get a JSON object of a specified workload.

    :param workload_name: The name of the workload.

    :raises Exception: An exception is raised if the specified workload does
    not exit.
    """
    workload_map = _get_resources(valid_types={"workload"})

    if workload_name not in workload_map:
        raise Exception(
            f"Error: Workload with name {workload_name} does not exist"
        )

    return workload_map[workload_name]


def get_resources_json_obj(resource_name: str) -> Dict:
    """
    Get a JSON object of a specified resource.

    :param resource_name: The name of the resource.

    :returns: The JSON object (in the form of a dictionary).

    :raises Exception: An exception is raised if the specified resources does
    not exist.
    """
    from .resource import _get_resource_json_type_map

    resource_map = _get_resources(
        valid_types=_get_resource_json_type_map.keys()
    )

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
    untar: bool = True,
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

    :param untar: If true, tar achieve resource will be unpacked prior to
    saving to `to_path`. True by default.

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

            if os.path.isfile(to_path):
                md5 = md5_file(Path(to_path))
            else:
                md5 = md5_dir(Path(to_path))

            if md5 == resource_json["md5sum"]:
                # In this case, the file has already been download, no need to
                # do so again.
                return
            elif download_md5_mismatch:
                if os.path.isfile(to_path):
                    os.remove(to_path)
                else:
                    shutil.rmtree(to_path)
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
                "'is_zipped' field which is neither a string or a boolean.".format(
                    resource_name
                )
            )

        run_tar_extract = (
            untar
            and "is_tar_archive" in resource_json
            and resource_json["is_tar_archive"]
        )

        tar_extension = ".tar"
        if run_tar_extract:
            download_dest += tar_extension

        zip_extension = ".gz"
        if run_unzip:
            download_dest += zip_extension

        # TODO: Might be nice to have some kind of download status bar here.
        # TODO: There might be a case where this should be silenced.
        print(
            "Resource '{}' was not found locally. Downloading to '{}'...".format(
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
            unzip_to = download_dest[: -len(zip_extension)]
            with gzip.open(download_dest, "rb") as f:
                with open(unzip_to, "wb") as o:
                    shutil.copyfileobj(f, o)
            os.remove(download_dest)
            download_dest = unzip_to
            print(
                "Finished decompressing resource '{}'.".format(resource_name)
            )

        if run_tar_extract:
            print(
                f"Unpacking the the resource '{resource_name}' "
                f"('{download_dest}')"
            )
            unpack_to = download_dest[: -len(tar_extension)]
            with tarfile.open(download_dest) as f:

                def is_within_directory(directory, target):

                    abs_directory = os.path.abspath(directory)
                    abs_target = os.path.abspath(target)

                    prefix = os.path.commonprefix([abs_directory, abs_target])

                    return prefix == abs_directory

                def safe_extract(
                    tar, path=".", members=None, *, numeric_owner=False
                ):

                    for member in tar.getmembers():
                        member_path = os.path.join(path, member.name)
                        if not is_within_directory(path, member_path):
                            raise Exception(
                                "Attempted Path Traversal in Tar File"
                            )

                    tar.extractall(path, members, numeric_owner=numeric_owner)

                safe_extract(f, unpack_to)
            os.remove(download_dest)

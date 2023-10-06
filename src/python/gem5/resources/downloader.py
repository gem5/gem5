# Copyright (c) 2021-2023 The Regents of the University of California
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
import gzip
import os
import random
import shutil
import tarfile
import time
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Dict
from typing import List
from typing import Optional
from urllib.error import HTTPError
from urllib.parse import urlparse

from _m5 import core

from ..utils.filelock import FileLock
from ..utils.progress_bar import progress_hook
from ..utils.progress_bar import tqdm
from .client import get_resource_json_obj
from .client import list_resources as client_list_resources
from .md5_utils import md5_dir
from .md5_utils import md5_file

"""
This Python module contains functions used to download, list, and obtain
information about resources from resources.gem5.org.
"""


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
                    with tqdm.wrapattr(
                        open(download_to, "wb"),
                        "write",
                        miniters=1,
                        desc="Downloading {download_to}",
                        total=getattr(fr, "length", None),
                    ) as fw:
                        for chunk in fr:
                            fw.write(chunk)
            else:
                with tqdm(
                    unit="B",
                    unit_scale=True,
                    unit_divisor=1024,
                    miniters=1,
                    desc=f"Downloading {download_to}",
                ) as t:
                    urllib.request.urlretrieve(
                        url,
                        download_to,
                        reporthook=progress_hook(t),
                    )
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
                        f"{e.code}",
                    )
                time.sleep((2**attempt) + random.uniform(0, 1))
            else:
                raise e
        except ConnectionResetError as e:
            # This catches the ConnectionResetError we see occassionally see
            # when accessing resources on GitHub Actions.  It retries using a
            # Truncated Exponential backoff algorithm, truncating after
            # "max_attempts". If any other is retrieved we raise the error.
            if e.errno == 104:
                attempt += 1
                if attempt >= max_attempts:
                    raise Exception(
                        f"After {attempt} attempts, the resource json could "
                        "not be retrieved. OS Error Code retrieved: "
                        f"{e.errno}",
                    )
                time.sleep((2**attempt) + random.uniform(0, 1))
            else:
                raise e
        except ValueError as e:
            raise Exception(
                f"ValueError: {e}\n"
                "Environment variable GEM5_USE_PROXY is set to "
                f"'{use_proxy}'. The expected form is "
                "<host>:<port>'.",
            )
        except ImportError as e:
            raise Exception(
                f"ImportError: {e}\n"
                "An import error has occurred. This is likely due "
                "the Python SOCKS client module not being "
                "installed. It can be installed with "
                "`pip install PySocks`.",
            )


def list_resources(
    clients: Optional[List] = None,
    gem5_version: Optional[str] = None,
) -> Dict[str, List[str]]:
    """
    Lists all available resources. Returns a dictionary where the key is the
    id of the resources and the value is a list of that resource's versions.

    :param clients: A list of clients to use when listing resources. If None,
    all clients will be used. None by default.

    :param gem5_version: The gem5 version to which all resources should be
    compatible with. If None, compatibility of resources is not considered and
    all resources will be returned.

    **Note**: This function is here for legacy reasons. The `list_resources`
    function was originally stored here. In order to remain backwards
    compatible, this function will call the `client_list_resources` function

    """
    return client_list_resources(clients=clients, gem5_version=gem5_version)


def get_resource(
    resource_name: str,
    to_path: str,
    unzip: bool = True,
    untar: bool = True,
    download_md5_mismatch: bool = True,
    resource_version: Optional[str] = None,
    clients: Optional[List] = None,
    gem5_version: Optional[str] = core.gem5Version,
    quiet: bool = False,
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

    :param resource_version: The version of the resource to be obtained. If
    None, the latest version of the resource compatible with the working
    directory's gem5 version will be obtained. None by default.

    :param clients: A list of clients to use when obtaining the resource. If
    None, all clients will be used. None by default.

    :param gem5_version: The gem5 version to use when obtaining the resource.
    By default, the version of gem5 being used is used. This is used primarily
    for testing purposes.

    :param quiet: If true, no output will be printed to the console (baring
    exceptions). False by default.

    :raises Exception: An exception is thrown if a file is already present at
    `to_path` but it does not have the correct md5 sum. An exception will also
    be thrown is a directory is present at `to_path`
    """

    # We apply a lock for a specific resource. This is to avoid circumstances
    # where multiple instances of gem5 are running and trying to obtain the
    # same resources at once. The timeout here is somewhat arbitarily put at 15
    # minutes.Most resources should be downloaded and decompressed in this
    # timeframe, even on the most constrained of systems.
    with FileLock(f"{to_path}.lock", timeout=900):
        resource_json = get_resource_json_obj(
            resource_name,
            resource_version=resource_version,
            clients=clients,
            gem5_version=gem5_version,
        )

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
                    "its md5 value is invalid.".format(to_path),
                )

        download_dest = to_path

        # This if-statement is remain backwards compatable with the older,
        # string-based way of doing things. It can be refactored away over
        # time:
        # https://gem5-review.googlesource.com/c/public/gem5-resources/+/51168
        run_unzip = False
        if "is_zipped" in resource_json:
            if isinstance(resource_json["is_zipped"], str):
                run_unzip = (
                    unzip and resource_json["is_zipped"].lower() == "true"
                )
            elif isinstance(resource_json["is_zipped"], bool):
                run_unzip = unzip and resource_json["is_zipped"]
            else:
                raise Exception(
                    "The resource.json entry for '{}' has a value for the "
                    "'is_zipped' field which is neither a string or a boolean.".format(
                        resource_name,
                    ),
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

        file_uri_path = _file_uri_to_path(resource_json["url"])
        if file_uri_path:
            if not file_uri_path.exists():
                raise Exception(
                    f"Could not find file at path '{file_uri_path}'",
                )
            print(
                "Resource '{}' is being copied from '{}' to '{}'...".format(
                    resource_name,
                    urlparse(resource_json["url"]).path,
                    download_dest,
                ),
            )
            shutil.copy(file_uri_path, download_dest)
        else:
            # TODO: Might be nice to have some kind of download status bar here..
            if not quiet:
                print(
                    f"Resource '{resource_name}' was not found locally. "
                    f"Downloading to '{download_dest}'...",
                )

            # Get the URL.
            url = resource_json["url"]

            _download(url=url, download_to=download_dest)
            if not quiet:
                print(f"Finished downloading resource '{resource_name}'.")

        if run_unzip:
            if not quiet:
                print(
                    f"Decompressing resource '{resource_name}' "
                    f"('{download_dest}')...",
                )
            unzip_to = download_dest[: -len(zip_extension)]
            with gzip.open(download_dest, "rb") as f:
                with open(unzip_to, "wb") as o:
                    shutil.copyfileobj(f, o)
            os.remove(download_dest)
            download_dest = unzip_to
            if not quiet:
                print(f"Finished decompressing resource '{resource_name}'.")

        if run_tar_extract:
            if not quiet:
                print(
                    f"Unpacking the the resource '{resource_name}' "
                    f"('{download_dest}')",
                )
            unpack_to = download_dest[: -len(tar_extension)]
            with tarfile.open(download_dest) as f:

                def is_within_directory(directory, target):
                    abs_directory = os.path.abspath(directory)
                    abs_target = os.path.abspath(target)

                    prefix = os.path.commonprefix([abs_directory, abs_target])

                    return prefix == abs_directory

                def safe_extract(
                    tar,
                    path=".",
                    members=None,
                    *,
                    numeric_owner=False,
                ):
                    for member in tar.getmembers():
                        member_path = os.path.join(path, member.name)
                        if not is_within_directory(path, member_path):
                            raise Exception(
                                "Attempted Path Traversal in Tar File",
                            )

                    tar.extractall(path, members, numeric_owner=numeric_owner)

                safe_extract(f, unpack_to)
            os.remove(download_dest)


def _file_uri_to_path(uri: str) -> Optional[Path]:
    """
    If the URI uses the File scheme (e.g, `file://host/path`) then
    a Path object for the local path is returned, otherwise None.

    **Note:** Only files from localhost are permitted. An exception
    is thrown otherwise.

    :param uri: The file URI to convert.

    :returns: The path to the file.
    """

    if urlparse(uri).scheme == "file":
        if urlparse(uri).netloc == "" or urlparse(uri).netloc == "localhost":
            local_path = urlparse(uri).path
            return Path(local_path)
        raise Exception(
            f"File URI '{uri}' specifies host '{urlparse(uri).netloc}'. "
            "Only localhost is permitted.",
        )
    return None

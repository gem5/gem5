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

import json
from pathlib import Path
import os
from typing import Optional, Dict, List
from .client_api.client_wrapper import ClientWrapper
from gem5.gem5_default_config import config
from m5.util import inform


def getFileContent(file_path: Path) -> Dict:
    """
    Get the content of the file at the given path
    :param file_path: The path of the file
    :return: The content of the file
    """
    if file_path.exists():
        with open(file_path, "r") as file:
            return json.load(file)
    else:
        raise Exception(f"File not found at {file_path}")


clientwrapper = None


def get_resource_json_obj(
    resource_id,
    resource_version: Optional[str] = None,
    clients: Optional[List[str]] = None,
) -> Dict:
    """
    Get the resource json object from the clients wrapper
    :param resource_id: The resource id
    :param resource_version: The resource version
    :param clients: The list of clients to query
    """
    global clientwrapper
    if clientwrapper is None:
        # First check if the config file path is provided in the environment variable
        if "GEM5_CONFIG" in os.environ:
            config_file_path = Path(os.environ["GEM5_CONFIG"])
            gem5_config = getFileContent(config_file_path)
            inform("Using config file specified by $GEM5_CONFIG")
            inform(f"Using config file at {os.environ['GEM5_CONFIG']}")
        # If not, check if the config file is present in the current directory
        elif (Path().cwd().resolve() / "gem5-config.json").exists():
            config_file_path = Path().resolve() / "gem5-config.json"
            gem5_config = getFileContent(config_file_path)
            inform(f"Using config file at {config_file_path}")
        # If not, use the default config in the build directory
        else:
            gem5_config = config
            inform("Using default config")
        clientwrapper = ClientWrapper(gem5_config)

    return clientwrapper.get_resource_json_obj_from_client(
        resource_id, resource_version, clients
    )

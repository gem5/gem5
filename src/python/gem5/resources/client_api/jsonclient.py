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
from urllib import request
from typing import Optional, Dict, Union, Type, Tuple, List, Any
from .abstract_client import AbstractClient
from urllib.error import URLError
from m5.util import warn


class JSONClient(AbstractClient):
    def __init__(self, path: str):
        """
        Initializes a JSON client.
        :param path: The path to the Resource, either URL or local.
        """
        self.path = path
        self.resources = []

        if Path(self.path).is_file():
            self.resources = json.load(open(self.path))
        elif not self._url_validator(self.path):
            raise Exception(
                f"Resources location '{self.path}' is not a valid path or URL."
            )
        else:
            req = request.Request(self.path)
            try:
                response = request.urlopen(req)
            except URLError as e:
                raise Exception(
                    f"Unable to open Resources location '{self.path}': {e}"
                )
            self.resources = json.loads(response.read().decode("utf-8"))

    def get_resources_by_id(self, resource_id: str) -> List[Dict[str, Any]]:
        """
        :param resource_id: The ID of the Resource.
        :return: A list of all the Resources with the given ID.
        """
        return [
            resource
            for resource in self.resources
            if resource["id"] == resource_id
        ]

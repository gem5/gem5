# Copyright (c) 2020 The Regents of the University of California
# All Rights Reserved.
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

import copy
import json
import requests
from types import SimpleNamespace
from urllib.parse import urljoin


class GerritResponseParser:
    @staticmethod
    def get_json_content(response):
        assert isinstance(response, requests.Response)

        # If the status code is not in the 200s range, it doesn't have content.
        if response.status_code >= 300:
            return None

        # Transform response.content to a Python3 string.
        # response.content is a byte array containing the response.
        # The first 4 bytes are b")]}\", which doesn't belong to JSON content.
        # The byte array is encoded by utf-8.
        content = response.content[4:].decode("utf-8")
        json_content = json.loads(content)
        return json_content

    # TODO: parsing method for each Gerrit data structure
    @staticmethod
    def parse(response):
        json_content = GerritResponseParser.get_json_content(response)
        if not json_content:
            return None
        return SimpleNamespace(**json_content)


class GerritRestAPI:
    def __init__(self, auth, api_entry_point, timeout):
        self.username = auth[0]
        self.password = auth[1]
        self.api_entry_point = api_entry_point
        self.timeout = timeout

    # helper methods for sending GET and POST requests
    def _get(self, endpoint, params=None):
        request_url = urljoin(self.api_entry_point, endpoint)
        return requests.get(
            request_url,
            params=params,
            timeout=self.timeout,
            auth=(self.username, self.password),
        )

    def _post(self, endpoint, json_content):
        request_url = urljoin(self.api_entry_point, endpoint)
        return requests.post(
            request_url,
            json=json_content,
            timeout=self.timeout,
            auth=(self.username, self.password),
        )

    # --------------- Account Endpoints ---------------
    # https://gerrit-review.googlesource.com/Documentation/
    # rest-api-accounts.html#get-account
    def get_account(self, account_id="self"):
        """get an account detail from an account_id"""
        return self._get(f"/accounts/{account_id}")

    # https://gerrit-review.googlesource.com/Documentation/
    # rest-api-accounts.html#query-account
    def query_account(self, query, limit=None):
        """get accounts based on the query"""
        params = {"q": query}
        if limit:
            params["n"] = str(limit)
        return self._get(f"/accounts/", params)

    # --------------- Changes Endpoints ---------------
    # https://gerrit-review.googlesource.com/Documentation/
    # rest-api-changes.html#list-changes
    def query_changes(self, query, limit=None, optional_field=None):
        """query changes with maximum limit returned queries"""
        endpoint = f"/changes/"
        params = {"q": query}
        if limit:
            params["n"] = str(limit)
        if optional_field:
            params["o"] = optional_field
        return self._get(endpoint, params)

    # --------------- Reviewer Endpoints ---------------
    # https://gerrit-review.googlesource.com/Documentation/
    # rest-api-changes.html#list-reviewers
    def list_reviewers(self, change_id):
        """list reviewers of a change"""
        return self._get(f"/changes/{change_id}/reviewers")

    def add_reviewer(self, change_id, reviewer_email):
        """add a reviewer using an email address"""
        data = {"reviewer": reviewer_email}
        return self._post(f"/changes/{change_id}/reviewers/", data)

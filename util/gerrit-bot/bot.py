#!/usr/bin/env python3

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

from gerrit import GerritResponseParser as Parser
from gerrit import GerritRestAPI
from util import add_maintainers_to_change, convert_time_in_seconds

import time

import sys
sys.path.append('..')
import maint.lib.maintainers


class GerritBotConfig:
    def __init__(self, config = {}):
        self.__dict__.update(config)
    @staticmethod
    def DefaultConfig():
        default_config = GerritBotConfig()
        default_config.auth_file_path = ".data/auth"
        default_config.time_tracker_file_path = ".data/prev_query_time"
        # query changes made within 2 days if not specified
        default_config.default_query_age = "2d"
        default_config.maintainers_file_path = None # the maintainers library
                                                    # will figure the path out
        default_config.api_entry_point = "https://gem5-review.googlesource.com"
        default_config.projects_prefix = "public/gem5"
        default_config.query_limit = 1000 # at most 1000 new changes per query
        default_config.request_timeout = 10 # seconds
        return default_config

class GerritBot:
    def __init__(self, config):
        self.config = config

        self.auth = self.__read_auth_file(self.config.auth_file_path)

        # Initalize the Gerrit API Object
        self.gerrit_api = GerritRestAPI(self.auth,
                                        self.config.api_entry_point,
                                        self.config.request_timeout)

        self.account_id = self.__get_bot_account_id()
        self.maintainers = maint.lib.maintainers.Maintainers.from_file(
                                            self.config.maintainers_file_path)

    def __read_auth_file(self, auth_file_path):
        username = ""
        password = ""
        with open(auth_file_path, "r") as f:
            lines = f.readlines()
            username = lines[0].strip()
            password = lines[1].strip()
        return (username, password)

    def __read_time_tracker_file(self, file_path):
        prev_query_time = 0

        try:
            with open(file_path, "r") as f:
                lines = f.readlines()
                prev_query_time = int(float(lines[0].strip()))
        except FileNotFoundError:
            print(f"warning: cannot find the time tracker file at "
                  f"`{file_path}`. Previous query time is set to 0.")
        except IndexError:
            print(f"warning: cannot find the content of the time tracker file "
                  f"at `{file_path}`. Previous query time is set 0.")

        return prev_query_time

    def __update_time_tracker_file(self, file_path):
        prev_query_time = time.time()
        with open(file_path, "w") as f:
            f.write(f"{prev_query_time}\n")
            f.write(f"# The above time is the result of calling time.time() "
                    f"in Python.")

    def __get_bot_account_id(self):
        account_info = Parser.parse(self.gerrit_api.get_account("self"))
        return account_info._account_id

    def __query_new_changes(self, query_age):
        query = (f"projects:{self.config.projects_prefix} "
                 f"status:open -is:wip -age:{query_age}")
        response = self.gerrit_api.query_changes(query,
                                                 self.config.query_limit,
                                                 "CURRENT_REVISION")

        if response.status_code >= 300:
            print("Error: Couldn't query new Gerrit changes")
            print(vars(query_new_gerrit_changes_response))
            raise Error()

        new_changes = Parser.get_json_content(response)

        return new_changes

    def _pre_run(self):
        self.prev_query_time = \
            self.__read_time_tracker_file(self.config.time_tracker_file_path)
        if self.prev_query_time > 0:
            self.query_age = \
              convert_time_in_seconds(int(time.time() - self.prev_query_time))
        else:
            self.query_age = self.config.default_query_age

    def _run(self):
        new_changes = self.__query_new_changes(self.query_age)
        for new_change in new_changes:
            add_maintainers_to_change(new_change, self.maintainers,
                                      self.gerrit_api)

    def _post_run(self):
        self.__update_time_tracker_file(self.config.time_tracker_file_path)

    def run(self):
        self._pre_run()
        self._run()
        self._post_run()

if __name__ == "__main__":
    default_config = GerritBotConfig.DefaultConfig()
    gerrit_bot = GerritBot(default_config)
    gerrit_bot.run()

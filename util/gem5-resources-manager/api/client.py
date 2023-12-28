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

from abc import (
    ABC,
    abstractmethod,
)
from typing import (
    Dict,
    List,
)


class Client(ABC):
    def __init__(self):
        self.__undo_stack = []
        self.__redo_stack = []
        self.__undo_limit = 10

    @abstractmethod
    def find_resource(self, query: Dict) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def get_versions(self, query: Dict) -> List[Dict]:
        raise NotImplementedError

    @abstractmethod
    def update_resource(self, query: Dict) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def check_resource_exists(self, query: Dict) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def insert_resource(self, query: Dict) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def delete_resource(self, query: Dict) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def save_session(self) -> Dict:
        raise NotImplementedError

    def undo_operation(self) -> Dict:
        """
        This function undoes the last operation performed on the database.
        """
        if len(self.__undo_stack) == 0:
            return {"status": "Nothing to undo"}
        operation = self.__undo_stack.pop()
        print(operation)
        if operation["operation"] == "insert":
            self.delete_resource(operation["resource"])
        elif operation["operation"] == "delete":
            self.insert_resource(operation["resource"])
        elif operation["operation"] == "update":
            self.update_resource(operation["resource"])
            temp = operation["resource"]["resource"]
            operation["resource"]["resource"] = operation["resource"][
                "original_resource"
            ]
            operation["resource"]["original_resource"] = temp
        else:
            raise Exception("Invalid Operation")
        self.__redo_stack.append(operation)
        return {"status": "Undone"}

    def redo_operation(self) -> Dict:
        """
        This function redoes the last operation performed on the database.
        """
        if len(self.__redo_stack) == 0:
            return {"status": "No operations to redo"}
        operation = self.__redo_stack.pop()
        print(operation)
        if operation["operation"] == "insert":
            self.insert_resource(operation["resource"])
        elif operation["operation"] == "delete":
            self.delete_resource(operation["resource"])
        elif operation["operation"] == "update":
            self.update_resource(operation["resource"])
            temp = operation["resource"]["resource"]
            operation["resource"]["resource"] = operation["resource"][
                "original_resource"
            ]
            operation["resource"]["original_resource"] = temp
        else:
            raise Exception("Invalid Operation")
        self.__undo_stack.append(operation)
        return {"status": "Redone"}

    def _add_to_stack(self, operation: Dict) -> Dict:
        if len(self.__undo_stack) == self.__undo_limit:
            self.__undo_stack.pop(0)
        self.__undo_stack.append(operation)
        self.__redo_stack.clear()
        return {"status": "Added to stack"}

    def get_revision_status(self) -> Dict:
        """
        This function saves the status of revision operations to a dictionary.

        The revision operations whose statuses are saved are undo and redo.

        If the stack of a given revision operation is empty, the status of
        that operation is set to 1 else the status is set to 0.

        :return: A dictionary containing the status of revision operations.
        """
        return {
            "undo": 1 if len(self.__undo_stack) == 0 else 0,
            "redo": 1 if len(self.__redo_stack) == 0 else 0,
        }

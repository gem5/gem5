from .abstract_data_source import AbstractDataSource
from .resource_category import ResourceCategory
from .exception import Gem5DataSourceSchemaViolation, Gem5DataSourceEntryNotFound

from typing import Any, Dict
import json
import os


class JSONDatabase(AbstractDataSource):
    def __init__(self, filename) -> None:
        super().__init__()
        self.filename = filename
        self._data = {}

    def open(self):
        with open(self.filename) as f:
            self._data = json.load(f)

    def close(self):
        pass

    def resource_exists(self, resource_id: str, resource_version: str) -> bool:
        for resource in self._data:
            if (
                resource["id"] == resource_id
                and resource["resource_version"] == resource_version
            ):
                return True
        return False

    def find_resource(self, resource_id: str, resource_version: str):
        for resource in self._data:
            if (
                resource["id"] == resource_id
                and resource["resource_version"] == resource_version
            ):
                return resource
        raise Gem5DataSourceEntryNotFound(resource_id, resource_version)

    def get_resource_category(
        self, resource_id: str, resource_version: str
    ) -> ResourceCategory:
        resource = self.find_resource(resource_id, resource_version)
        return ResourceCategory(resource["category"])

    def get_resource_from_file(self, file):
        # append resource to resources.json
        resources = None
        if os.path.exists(file):
            with open(file) as infile:
                resources = json.load(infile)
        return resources

    def save_file(self, resource, output):
        # check if output path is a directory
        if os.path.isdir(output):
            # if it is then create resources.json in that directory
            output = os.path.join(output, "resources.json")
        existing_resources = self.get_resource_from_file(output)
        if existing_resources is not None:
            existing_resources.append(resource)
            with open(output, "w") as outfile:
                json.dump(existing_resources, outfile, indent=4)
        else:
            with open(output, "w") as outfile:
                json.dump([resource], outfile, indent=4)
        print("final resource", json.dumps(resource, indent=4))

    def create_new_entry(self, resource: Dict) -> None:
        self.save_file(resource, "new_resources.json")

from .abstract_data_source import AbstractDataSource
from .resource_category import ResourceCategory
from .exception import Gem5DataSourceSchemaViolation, Gem5DataSourceEntryNotFound

from typing import Any


class DummyDatabase(AbstractDataSource):
    # A simple dummy database used to show how to implement the API in a
    # way which you can start development on other parts of the system.
    #
    # **Note**: This is just an example. Don't flesh it out too much. Think
    # carefully about what you need.

    def __init__(self) -> None:
        self._data = {}

        self._fields_and_types = {
            ResourceCategory.BINARY: {
                "url": str,
                "count": int,
            }
        }

    def resource_exists(self, resource_id: str, resource_version: str) -> bool:
        return resource_id in self._data and resource_version in self._data[resource_id]

    def get_resource_category(
        self, resource_id: str, resource_version: str
    ) -> ResourceCategory:
        # All DummyDatabase's resources are binary.
        return ResourceCategory.BINARY

    def field_exists(self, category: ResourceCategory, field: str) -> bool:
        return category in self._fields_and_types and field in self._fields_and_types

    def field_value_valid(
        self, category: ResourceCategory, field: str, value: Any
    ) -> bool:
        if not self.field_exists(category, field):
            raise Gem5DataSourceSchemaViolation(
                f"Field '{field}' is not valid for resource category "
                f"'{category}'.\nConsider using the `field_exists` function "
                "to check first."
            )
        return (
            category in self._fields_and_types
            and field in self._fields_and_types[category]
            and isinstance(value, self._fields_and_types[category][field])
        )

    def update_entry_field(
        self, resource_id: str, resource_version: str, field: str, value: Any
    ) -> None:
        if not self.resource_exists:
            raise Gem5DataSourceEntryNotFound(resource_id, resource_version)
        category = self.get_resource_category(resource_id, resource_version)
        if not self.field_value_valid(category, field, value):
            raise Gem5DataSourceSchemaViolation()

        self._data[resource_id][resource_version][field] = value

    def create_new_entry(
        self,
        resource_id: str,
        resource_version: str,
        category: ResourceCategory,
        **kwards,
    ) -> None:
        if self.resource_exists(resource_id, resource_version):
            raise Gem5DataSourceSchemaViolation(
                f"Resource already exists in this database: "
                f"'{resource_id}', v{resource_version}.\n"
                "Cannot create a new entry."
            )

        if resource_id not in self._data:
            self._data[resource_id] = {}

        for field in kwards:
            self.update_entry_field(resource_id, resource_version, field, kwards[field])

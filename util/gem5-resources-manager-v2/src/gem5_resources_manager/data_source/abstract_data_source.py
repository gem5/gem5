from abc import ABC

from typing import Any
from enum import Enum

from .resource_category import ResourceCategory


class AbstractDataSource(ABC):
    # Here is where we define the API. This what we interact with.
    #
    # **Note:** This is just a quick example. You must think carefully about
    # what API will work for you.

    def __enter__(self):
        self.open()

    def __exit__(self, *args):
        self.close()

    def open(self):
        pass

    def close(self):
        pass

    def resource_exists(self, resource_id: str, resource_version: str) -> bool:
        # Does the resources exist in the data source?
        raise NotImplementedError

    def get_resource_category(
        self, resource_id: str, resource_version: str
    ) -> ResourceCategory:
        # What the category of this resources? We'd expect an exception if
        # this resoruce does not exist.
        raise NotImplementedError

    def field_exists(self, category: ResourceCategory, field: str) -> bool:
        # Does a field exist for this category? This, and `field_value_valid`
        # are placeholders for a grander schema checker.
        raise NotImplementedError

    def field_value_valid(
        self, category: ResourceCategory, field: str, value: Any
    ) -> bool:
        # Does this field's value conform to our schema?
        raise NotImplementedError

    def update_entry_field(
        self, resource_id: str, resource_version: str, field: str, value: Any
    ) -> None:
        # Update this resource's field with the specified value. Will return
        # an exception if not possible.
        raise NotImplementedError

    def create_new_entry(
        self, resource_id: str, resource_version: str, **kwards
    ) -> None:
        # Creates a new entry.
        raise NotImplementedError

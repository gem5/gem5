from typing import Optional

"""
This class is a data class that represents a query to the client.
It encapsulates the fields required to query resources from the client.
Right now, it only contains the resource_id, resource_version, and gem5_version
fields, but it can be expanded to include more fields in the future, if needed.
"""


class ClientQuery:
    def __init__(
        self,
        resource_id: str,
        resource_version: Optional[str] = None,
        gem5_version: Optional[str] = None,
    ):
        self.resource_id = resource_id
        self.resource_version = resource_version
        self.gem5_version = gem5_version

    def get_resource_id(self) -> str:
        return self.resource_id

    def get_resource_version(self) -> Optional[str]:
        return self.resource_version

    def get_gem5_version(self) -> Optional[str]:
        return self.gem5_version

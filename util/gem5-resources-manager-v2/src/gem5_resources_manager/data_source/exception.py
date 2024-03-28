from typing import Optional


class Gem5DataSourceException(Exception):
    pass


class Gem5DataSourceConnectionFailure(Gem5DataSourceException):
    pass


class Gem5DataSourceSchemaViolation(Gem5DataSourceException):
    pass


class Gem5DataSourceEntryNotFound(Gem5DataSourceException):
    def __init__(self, resource_id: Optional[str], version: Optional[str]):
        exception_str = f"Resource entry not found"
        if resource_id:
            exception_str + f" ('{resource_id}'" + f", v{version})." if version else ".)"
        super().__init__(exception_str)

from .abstract_validator import AbstractValidator
from ..data_source.exception import Gem5DataSourceSchemaViolation

from typing import Any, Dict
import jsonschema
import json
import requests


class JSONValidator(AbstractValidator):
    def __init__(self, filelink) -> None:
        self.schema = json.loads(requests.get(filelink).content)
        super().__init__(self.schema)

    def get_fields(self, category) -> [Dict, Dict]:
        optional = {}
        required = {}
        for field in self.schema["properties"]:
            default = self.schema["properties"][field]
            if field in self.schema["required"]:
                required[field] = default
            else:
                optional[field] = default

        definitions = []
        for definition in self.schema["definitions"][category]["allOf"]:
            definitions.append(definition["$ref"].split("/")[-1])

        for definition in definitions:
            for field in self.schema["definitions"][definition]["properties"]:
                default = self.schema["definitions"][definition]["properties"][field]
                if field in optional.keys():
                    required[field] = optional[field]
                    del optional[field]
                elif (
                    "required" not in self.schema["definitions"][definition]
                    or field in self.schema["definitions"][definition]["required"]
                ):
                    required[field] = default
                else:
                    optional[field] = default
        if "architecture" in required:
            required["architecture"] = self.schema["definitions"]["architecture"]
        if "architecture" in optional:
            optional["architecture"] = self.schema["definitions"]["architecture"]

        return required, optional

    def validate(self, resources: Dict[str, Any]):
        for resource in resources:
            validator = jsonschema.Draft7Validator(self.schema)

            is_resource_valid = validator.is_valid(resource)
            if not is_resource_valid:
                print(
                    f"\nResource with 'id': '{resource['id']}' and 'resource_version': '{resource['resource_version']}' is invalid."
                )
                for error in validator.iter_errors(resource):
                    print(f"\n- {error.path}: {error.message}")

                return False
        return True

    def get_changed_fields(self, resource: Dict[str, Any]):
        changed_fields = []
        validator = jsonschema.Draft7Validator(self.schema)
        for error in validator.iter_errors(resource):
            changed_fields.append(error.path[0])
        return changed_fields

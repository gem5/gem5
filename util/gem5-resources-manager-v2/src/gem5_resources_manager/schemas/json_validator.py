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

    def get_category_specific_fields(
        self, category: str, optional: Dict[str, object], required: Dict[str, object]
    ):
        for field, field_value in self.schema["definitions"][category][
            "properties"
        ].items():
            if field in optional.keys():
                required[field] = optional[field]
                del optional[field]
            elif (
                "required" in self.schema["definitions"][category]
                and field in self.schema["definitions"][category]["required"]
            ):
                required[field] = field_value
            else:
                optional[field] = field_value

        parent_category = self.schema["definitions"][category].get("allOf")
        print("parent_category:", parent_category)
        if not parent_category:
            return

        parent_category = parent_category[0]["$ref"].split("/")[-1]
        self.get_category_specific_fields(parent_category, optional, required)

    def get_fields(self, category) -> [Dict, Dict]:
        optional = {}
        required = {}
        for field in self.schema["properties"]:
            default = self.schema["properties"][field]
            if field in self.schema["required"]:
                required[field] = default
            else:
                optional[field] = default

        self.get_category_specific_fields(category, optional, required)

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

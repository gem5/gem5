from argparse import ArgumentParser, Namespace
import json
import requests

from ..abstract_subtool import AbstractSubtool
from ..data_source.abstract_data_source import AbstractDataSource


class Updater(AbstractSubtool):
    def __init__(self) -> None:
        self.data_source = None

    # Implement the two abstract classes declared in AbstractSubtool.
    def get_arg_parser(self, subparser):
        resource_updater_parser = subparser.add_parser("updateResources")
        resource_updater_parser.add_argument(
            "id", type=str, help="id of resource to that was just updated"
        )
        resource_updater_parser.add_argument(
            "--update-non-existing-fields",
            "-u",
            action="store_true",
            help="update fields that are not defined in the JSON object",
        )
        resource_updater_parser.add_argument(
            "--output",
            "-o",
            type=str,
            help="output file",
            default="resources.json",
        )

    def execute(self, args: Namespace, data_source: AbstractDataSource):
        print("Updater.execute()", args)
        self.data_source = data_source
        print("Updater.execute()", args.id)
        self.update_resource(args)

    def update_resource(self, args: Namespace):
        resource = self.data_source.find_latest_resource(args.id)
        new_resource_version = self.get_updated_resource_version(
            resource["resource_version"]
        )
        updated_resource = self.update_resource_json_obj(
            resource, new_resource_version, args.update_non_existing_fields
        )
        self.data_source.create_new_entry(updated_resource)

    def update_resource_json_obj(
        self, resource, new_resource_version, update_non_existing_fields
    ):
        updated_resource = {}
        print(resource)
        for field_key, field_value in resource.items():
            if field_key == "resource_version":
                updated_resource[field_key] = new_resource_version
                print(f"{field_key}: {updated_resource[field_key]}")
                continue
            print(f"\n\nCurrent value of {field_key}")
            print(f"{field_key}: {field_value}")
            is_updated = input("Do you want to update this field? (Y/N): ")
            if is_updated.lower() == "y":
                updated_resource[field_key] = input(
                    f"Enter new value for {field_key}: "
                )
            else:
                updated_resource[field_key] = field_value

        if update_non_existing_fields:
            print("\n\nUpdating non-existing fields\n\n")
            _, optional_fields = self.data_source.get_fields(
                resource["category"],
                json.loads(
                    requests.get(
                        "https://resources.gem5.org/gem5-resources-schema.json"
                    ).content
                ),
            )
            for optional_field in optional_fields.keys():
                if optional_field not in resource.keys():
                    print(
                        f"Update the following optional field: {optional_field}? (Y/N):"
                    )
                    if input().lower() != "y":
                        continue
                    print(
                        f"The {optional_field} takes the following input: {json.dumps(optional_fields[optional_field], indent=4)}"
                    )
                    updated_resource[optional_field] = input(
                        f"Enter value for {optional_field}: "
                    )

        return updated_resource

    def get_updated_resource_version(self, current_resource_version):
        is_update = input("Is the resource version being updated? (Y/N): ")
        if is_update.lower() == "y":
            version_subparts = current_resource_version.split(".")
            version_subparts[0] = str(int(version_subparts[0]) + 1)
            new_version = (
                version_subparts[0]
                + "."
                + version_subparts[1]
                + "."
                + version_subparts[2]
            )
            return new_version
        return current_resource_version

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

    def update_resource(self, args):
        resource = self.data_source.find_latest_resource(args.id)
        changed_fields = self.data_source.get_changed_fields(resource)
        required_fields, optional_fields = self.data_source.get_fields(
            resource["category"]
        )
        updated_resource = {}
        for field, value in resource.items():
            if field in changed_fields:
                print("This field has been changed or is new:", field)
                print("Current value:", value)
                if field in required_fields:
                    field_type = json.dumps(required_fields[field], indent=4)
                    del required_fields[field]
                else:
                    field_type = json.dumps(optional_fields[field], indent=4)
                    del optional_fields[field]
                print(f"{field} now has a value: {field_type}")
                new_value = input("Enter new value: ")
                updated_resource[field] = new_value
            else:
                if field in required_fields:
                    field_type = json.dumps(required_fields[field], indent=4)
                    del required_fields[field]
                else:
                    field_type = json.dumps(optional_fields[field], indent=4)
                    del optional_fields[field]
                is_update = input(f"Is {field} being updated? (Y/N): ")
                if is_update.lower() == "y":
                    print("Current value:", value)

                    print(f"{field} now has a value: {field_type}")
                    new_value = input("Enter new value: ")
                    updated_resource[field] = new_value
                else:
                    updated_resource[field] = value

        if args.update_non_existing_fields:
            for field in optional_fields.keys():
                is_update = input(f"Do you want to add the {field} field? (Y/N): ")
                if is_update.lower() == "y":
                    print("Current value:", optional_fields[field])
                    new_value = input("Enter new value: ")
                    updated_resource[field] = new_value

        print("Updated resource:", json.dumps(updated_resource, indent=4))

    def get_dependent_resources(self, resource, dependent_resources):
        if resource["category"] == "suite":
            return

        if resource["category"] == "workload":
            all_suites = self.data_source.get_all_resources_by_category("suite")
            for suite in all_suites:
                new_suite = suite.copy()
                new_suite["workloads"] = []
                for workload in suite["workloads"]:
                    if workload["id"] == resource["id"]:
                        workload["resource_version"] = resource["resource_version"]
                        new_suite["workloads"].append(workload)
                    else:
                        new_suite["workloads"].append(workload)
                dependent_resources.append(new_suite)
        else:
            all_workloads = self.data_source.get_all_resources_by_category("workload")
            for workload in all_workloads:
                new_workload = workload.copy()
                for resources in workload["resources"]:
                    if resources[resource["category"]]["id"] == resource["id"]:
                        resources[resource["category"]]["resource_version"] = resource[
                            "resource_version"
                        ]
                        new_workload["resources"]

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

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
import base64
import json
import os

import requests
from jsonschema import validate


class ResourceJsonCreator:
    """
    This class generates the JSON which is pushed onto MongoDB.
    On a high-level, it does the following:
        - Adds certain fields to the JSON.
        - Populates those fields.
        - Makes sure the JSON follows the schema.
    """

    # Global Variables
    base_url = "https://github.com/gem5/gem5/tree/develop"  # gem5 GitHub URL
    resource_url_map = {
        "dev": (
            "https://gem5.googlesource.com/public/gem5-resources/+/refs/heads/"
            "develop/resources.json?format=TEXT"
        ),
        "22.1": (
            "https://gem5.googlesource.com/public/gem5-resources/+/refs/heads/"
            "stable/resources.json?format=TEXT"
        ),
        "22.0": (
            "http://resources.gem5.org/prev-resources-json/"
            "resources-21-2.json"
        ),
        "21.2": (
            "http://resources.gem5.org/prev-resources-json/"
            "resources-22-0.json"
        ),
    }

    def __init__(self):
        self.schema = {}
        with open("schema/schema.json", "r") as f:
            self.schema = json.load(f)

    def _get_file_data(self, url):
        json_data = None
        try:
            json_data = requests.get(url).text
            json_data = base64.b64decode(json_data).decode("utf-8")
            return json.loads(json_data)
        except:
            json_data = requests.get(url).json()
            return json_data

    def _get_size(self, url):
        """
        Helper function to return the size of a download through its URL.
        Returns 0 if URL has an error.

        :param url: Download URL
        """
        try:
            response = requests.head(url)
            size = int(response.headers.get("content-length", 0))
            return size
        except Exception as e:
            return 0

    def _search_folder(self, folder_path, id):
        """
        Helper function to find the instance of a string in a folder.
        This is recursive, i.e., subfolders will also be searched.

        :param folder_path: Path to the folder to begin searching
        :param id: Phrase to search in the folder

        :returns matching_files: List of file paths to the files containing id
        """
        matching_files = []
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            if os.path.isfile(file_path):
                with open(
                    file_path,
                    "r",
                    encoding="utf-8",
                    errors="ignore",
                ) as f:
                    contents = f.read()
                    if id in contents:
                        file_path = file_path.replace("\\", "/")
                        matching_files.append(file_path)
            elif os.path.isdir(file_path):
                matching_files.extend(self._search_folder(file_path, id))
        return matching_files

    def _change_type(self, resource):
        if resource["type"] == "workload":
            # get the architecture from the name and remove 64 from it
            resource["architecture"] = (
                resource["name"].split("-")[0].replace("64", "").upper()
            )
            return resource
        if "kernel" in resource["name"]:
            resource["type"] = "kernel"
        elif "bootloader" in resource["name"]:
            resource["type"] = "bootloader"
        elif "benchmark" in resource["documentation"]:
            resource["type"] = "disk-image"
            # if tags not in resource:
            if "tags" not in resource:
                resource["tags"] = []
            resource["tags"].append("benchmark")
            if (
                "additional_metadata" in resource
                and "root_partition" in resource["additional_metadata"]
                and resource["additional_metadata"]["root_partition"]
                is not None
            ):
                resource["root_partition"] = resource["additional_metadata"][
                    "root_partition"
                ]
            else:
                resource["root_partition"] = ""
        elif resource["url"] is not None and ".img.gz" in resource["url"]:
            resource["type"] = "disk-image"
            if (
                "additional_metadata" in resource
                and "root_partition" in resource["additional_metadata"]
                and resource["additional_metadata"]["root_partition"]
                is not None
            ):
                resource["root_partition"] = resource["additional_metadata"][
                    "root_partition"
                ]
            else:
                resource["root_partition"] = ""
        elif "binary" in resource["documentation"]:
            resource["type"] = "binary"
        elif "checkpoint" in resource["documentation"]:
            resource["type"] = "checkpoint"
        elif "simpoint" in resource["documentation"]:
            resource["type"] = "simpoint"
        return resource

    def _extract_code_examples(self, resource, source):
        """
        This function goes by IDs present in the resources DataFrame.
        It finds which files use those IDs in gem5/configs.
        It adds the GitHub URL of those files under "example".
        It finds whether those files are used in gem5/tests/gem5.
        If yes, it marks "tested" as True. If not, it marks "tested" as False.
        "example" and "tested" are made into a JSON for every code example.
        This list of JSONs is assigned to the 'code_examples' field of the
        DataFrame.

        :param resources: A DataFrame containing the current state of
        resources.
        :param source: Path to gem5

        :returns resources: DataFrame with ['code-examples'] populated.
        """
        id = resource["id"]
        # search for files in the folder tree that contain the 'id' value
        matching_files = self._search_folder(
            source + "/configs",
            '"' + id + '"',
        )
        filenames = [os.path.basename(path) for path in matching_files]
        tested_files = []
        for file in filenames:
            tested_files.append(
                True
                if len(self._search_folder(source + "/tests/gem5", file)) > 0
                else False,
            )

        matching_files = [
            file.replace(source, self.base_url) for file in matching_files
        ]

        code_examples = []

        for i in range(len(matching_files)):
            json_obj = {
                "example": matching_files[i],
                "tested": tested_files[i],
            }
            code_examples.append(json_obj)
        return code_examples

    def unwrap_resources(self, ver):
        data = self._get_file_data(self.resource_url_map[ver])
        resources = data["resources"]
        new_resources = []
        for resource in resources:
            if resource["type"] == "group":
                for group in resource["contents"]:
                    new_resources.append(group)
            else:
                new_resources.append(resource)
        return new_resources

    def _get_example_usage(self, resource):
        if resource["category"] == "workload":
            return f"Workload(\"{resource['id']}\")"
        else:
            return f"obtain_resource(resource_id=\"{resource['id']}\")"

    def _parse_readme(self, url):
        metadata = {
            "tags": [],
            "author": [],
            "license": "",
        }
        try:
            request = requests.get(url)
            content = request.text
            content = content.split("---")[1]
            content = content.split("---")[0]
            if "tags:" in content:
                tags = content.split("tags:\n")[1]
                tags = tags.split(":")[0]
                tags = tags.split("\n")[:-1]
                tags = [tag.strip().replace("- ", "") for tag in tags]
                if tags == [""] or tags == None:
                    tags = []
                metadata["tags"] = tags
            if "author:" in content:
                author = content.split("author:")[1]
                author = author.split("\n")[0]
                author = (
                    author.replace("[", "").replace("]", "").replace('"', "")
                )
                author = author.split(",")
                author = [a.strip() for a in author]
                metadata["author"] = author
            if "license:" in content:
                license = content.split("license:")[1].split("\n")[0]
                metadata["license"] = license
        except:
            pass
        return metadata

    def _add_fields(self, resources, source):
        new_resources = []
        for resource in resources:
            res = self._change_type(resource)
            res["gem5_versions"] = ["23.0"]
            res["resource_version"] = "1.0.0"
            res["category"] = res["type"]
            del res["type"]
            res["id"] = res["name"]
            del res["name"]
            res["description"] = res["documentation"]
            del res["documentation"]
            if "additional_metadata" in res:
                for k, v in res["additional_metadata"].items():
                    res[k] = v
                del res["additional_metadata"]
            res["example_usage"] = self._get_example_usage(res)
            if "source" in res:
                url = (
                    "https://raw.githubusercontent.com/gem5/"
                    "gem5-resources/develop/"
                    + str(res["source"])
                    + "/README.md"
                )
                res["source_url"] = (
                    "https://github.com/gem5/gem5-resources/tree/develop/"
                    + str(res["source"])
                )
            else:
                url = ""
                res["source_url"] = ""
            metadata = self._parse_readme(url)
            if "tags" in res:
                res["tags"].extend(metadata["tags"])
            else:
                res["tags"] = metadata["tags"]
            res["author"] = metadata["author"]
            res["license"] = metadata["license"]

            res["code_examples"] = self._extract_code_examples(res, source)

            if "url" in resource:
                download_url = res["url"].replace(
                    "{url_base}",
                    "http://dist.gem5.org/dist/develop",
                )
                res["url"] = download_url
                res["size"] = self._get_size(download_url)
            else:
                res["size"] = 0

            res = {k: v for k, v in res.items() if v is not None}

            new_resources.append(res)
        return new_resources

    def _validate_schema(self, resources):
        for resource in resources:
            try:
                validate(resource, schema=self.schema)
            except Exception as e:
                print(resource)
                raise e

    def create_json(self, version, source, output):
        resources = self.unwrap_resources(version)
        resources = self._add_fields(resources, source)
        self._validate_schema(resources)
        with open(output, "w") as f:
            json.dump(resources, f, indent=4)

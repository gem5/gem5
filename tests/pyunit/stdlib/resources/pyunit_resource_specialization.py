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
import os
import unittest
from pathlib import Path
from unittest.mock import patch

from gem5.isas import ISA
from gem5.resources.client_api.client_wrapper import ClientWrapper
from gem5.resources.looppoint import LooppointCsvLoader
from gem5.resources.looppoint import LooppointJsonLoader
from gem5.resources.resource import *

mock_json_path = Path(__file__).parent / "refs/resource-specialization.json"

mock_config_json = {
    "sources": {
        "baba": {
            "url": mock_json_path,
            "isMongo": False,
        }
    },
}


@patch(
    "gem5.resources.client.clientwrapper",
    ClientWrapper(mock_config_json),
)
class ResourceSpecializationSuite(unittest.TestCase):
    """This suite tests that `gem5.resource.resource` casts to the correct
    `AbstractResource` specialization when using the `obtain_resource`
    function.
    """

    def get_resource_dir(cls) -> str:
        """To ensure the resources are cached to the same directory as all
        other tests, this function returns the location of the testing
        directories "resources" directory.
        """
        return os.path.join(
            os.path.realpath(os.path.dirname(__file__)),
            os.pardir,
            os.pardir,
            os.pardir,
            "gem5",
            "resources",
        )

    def test_binary_resource(self) -> None:
        """Tests the loading of of a BinaryResource"""
        resource = obtain_resource(
            resource_id="binary-example",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, BinaryResource)

        self.assertEquals(
            "binary-example documentation.", resource.get_description()
        )
        self.assertEquals("src/simple", resource.get_source())
        self.assertEquals(ISA.ARM, resource.get_architecture())

    def test_kernel_resource(self) -> None:
        """Tests the loading of a KernelResource."""
        resource = obtain_resource(
            resource_id="kernel-example",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, KernelResource)

        self.assertEquals(
            "kernel-example documentation.", resource.get_description()
        )
        self.assertEquals("src/linux-kernel", resource.get_source())
        self.assertEquals(ISA.RISCV, resource.get_architecture())

    def test_bootloader_resource(self) -> None:
        """Tests the loading of a BootloaderResource."""
        resource = obtain_resource(
            resource_id="bootloader-example",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, BootloaderResource)

        self.assertEquals(
            "bootloader documentation.", resource.get_description()
        )
        self.assertIsNone(resource.get_source())
        self.assertIsNone(resource.get_architecture())

    def test_disk_image_resource(self) -> None:
        """Tests the loading of a DiskImageResource."""
        resource = obtain_resource(
            resource_id="disk-image-example",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, DiskImageResource)

        self.assertEquals(
            "disk-image documentation.", resource.get_description()
        )
        self.assertEquals("src/x86-ubuntu", resource.get_source())
        self.assertEquals("1", resource.get_root_partition())

    def test_checkpoint_resource(self) -> None:
        """Tests the loading of a CheckpointResource."""
        resource = obtain_resource(
            resource_id="checkpoint-example",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, CheckpointResource)

        self.assertEquals(
            "checkpoint-example documentation.", resource.get_description()
        )
        self.assertIsNone(resource.get_source())

    def test_git_resource(self) -> None:
        """Tests the loading of a GitResource."""
        resource = obtain_resource(
            resource_id="git-example",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, GitResource)

        self.assertIsNone(resource.get_description())
        self.assertIsNone(resource.get_source())

    def test_simpoint_directory_resource(self) -> None:
        """Tests the loading of a Simpoint directory resource."""
        resource = obtain_resource(
            resource_id="simpoint-directory-example",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, SimpointDirectoryResource)

        self.assertEquals(
            "simpoint directory documentation.", resource.get_description()
        )
        self.assertIsNone(resource.get_source())

        self.assertEquals(1000000, resource.get_simpoint_interval())
        self.assertEquals(1000000, resource.get_warmup_interval())
        self.assertEquals(
            Path(
                Path(self.get_resource_dir())
                / "simpoint-directory-example"
                / "simpoint.simpt"
            ),
            resource.get_simpoint_file(),
        )
        self.assertEquals(
            Path(
                Path(self.get_resource_dir())
                / "simpoint-directory-example"
                / "simpoint.weight"
            ),
            resource.get_weight_file(),
        )
        self.assertEquals("Example Workload", resource.get_workload_name())

    def test_simpoint_resource(self) -> None:
        """Tests the loading of a Simpoint resource."""
        resource = obtain_resource(
            resource_id="simpoint-example",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, SimpointResource)

        self.assertEquals(
            "simpoint documentation.", resource.get_description()
        )
        self.assertIsNone(resource.get_source())
        self.assertIsNone(resource.get_local_path())

        self.assertEquals(1000000, resource.get_simpoint_interval())
        self.assertEquals(23445, resource.get_warmup_interval())
        self.assertEquals([2, 3, 4, 15], resource.get_simpoint_list())
        self.assertEquals([0.1, 0.2, 0.4, 0.3], resource.get_weight_list())

    def test_file_resource(self) -> None:
        """Tests the loading of a FileResource."""
        resource = obtain_resource(
            resource_id="file-example",
            resource_directory=self.get_resource_dir(),
            resource_version="1.0.0",
            gem5_version="develop",
        )

        self.assertIsInstance(resource, FileResource)

        self.assertIsNone(resource.get_description())
        self.assertIsNone(resource.get_source())

    def test_directory_resource(self) -> None:
        """Tests the loading of a DirectoryResource."""
        resource = obtain_resource(
            resource_id="directory-example",
            resource_directory=self.get_resource_dir(),
        )

        self.assertIsInstance(resource, DirectoryResource)

        self.assertEquals(
            "directory-example documentation.", resource.get_description()
        )
        self.assertIsNone(resource.get_source())

    def test_looppoint_pinpoints_resource(self) -> None:
        """Tests the creation of LooppointCreatorCSVResource via a Looppoint
        pinpoints csv file."""

        resource = obtain_resource(
            resource_id="looppoint-pinpoint-csv-resource",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )

        self.assertIsInstance(resource, LooppointCsvResource)

        # The LooppointCreatorCSVResource should be a subtype of
        # LooppointCsvLoader.
        self.assertIsInstance(resource, LooppointCsvLoader)

        self.assertEquals(
            "A looppoint pinpoints csv file.", resource.get_description()
        )
        self.assertIsNone(resource.get_source())

    def test_looppoint_json_restore_resource(self) -> None:
        """Tests the creation of LooppointJsonResource via a
        Looppoint JSON file."""

        resource = obtain_resource(
            resource_id="looppoint-json-restore-resource-region-1-example",
            resource_directory=self.get_resource_dir(),
            resource_version="1.0.0",
            gem5_version="develop",
        )

        self.assertIsInstance(resource, LooppointJsonResource)
        self.assertIsInstance(resource, LooppointJsonLoader)

        self.assertEquals(1, len(resource.get_regions()))
        self.assertTrue("1" in resource.get_regions())

        self.assertEquals(
            "A looppoint json file resource.", resource.get_description()
        )
        self.assertIsNone(resource.get_source())

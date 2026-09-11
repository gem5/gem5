# Copyright (c) 2026 The Regents of The University of California
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

import gzip
import hashlib
import io
import os
import stat
import tarfile
import tempfile
import unittest
from pathlib import Path
from unittest.mock import (
    call,
    patch,
)

from gem5.resources.downloader import (
    _copy_to_sparse_file,
    _densify_file,
    _download_to_sparse_file,
    _file_has_holes,
    _sparsify_file,
    _write_sparse_file,
    get_resource,
)
from gem5.resources.resource import (
    _get_suite,
    _get_workload,
    obtain_resource,
)


class SparseResourceTestSuite(unittest.TestCase):
    """Tests sparse materialization through helpers and ``obtain_resource``."""

    chunk_size = 1024 * 1024
    contents = b"A" * chunk_size + b"\0" * (2 * chunk_size) + b"B" * chunk_size

    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.directory_path = Path(self.directory.name)
        self.compressed_path = self.directory_path / "image.raw.gz"
        with gzip.open(self.compressed_path, "wb") as output:
            output.write(self.contents)

        self.resource_json = {
            "category": "disk-image",
            "id": "test-sparse-disk-image",
            "resource_version": "1.0.0",
            "url": self.compressed_path.as_uri(),
            "is_zipped": True,
            "md5sum": hashlib.md5(self.contents).hexdigest(),
        }

    def tearDown(self) -> None:
        self.directory.cleanup()

    def _obtain(self, destination: Path, **resource_args) -> str:
        with patch(
            "gem5.resources.resource.get_resource_json_obj",
            return_value=self.resource_json,
        ), patch(
            "gem5.resources.downloader.get_resource_json_obj",
            return_value=self.resource_json,
        ):
            resource = obtain_resource(
                resource_id=self.resource_json["id"],
                resource_version=self.resource_json["resource_version"],
                to_path=str(destination),
                **resource_args,
            )
            return resource.get_local_path()

    def assert_contents_equal(self, path: Path) -> None:
        with open(path, "rb") as input_file:
            self.assertEqual(self.contents, input_file.read())

    def test_disk_image_is_sparse_by_default(self) -> None:
        """Disk images use the sparse writer when no option is supplied."""

        destination = self.directory_path / "automatic.raw"
        with patch(
            "gem5.resources.downloader._write_sparse_file",
            wraps=_write_sparse_file,
        ) as sparse_writer:
            self._obtain(destination)

        sparse_writer.assert_called_once()
        self.assert_contents_equal(destination)

    def test_sparse_copy_skips_zero_chunks(self) -> None:
        """Full zero chunks become seeks while logical contents are retained."""

        destination = self.directory_path / "sparse-copy.raw"
        with open(destination, "wb") as output:
            with patch.object(output, "seek", wraps=output.seek) as seek:
                _copy_to_sparse_file(io.BytesIO(self.contents), output)

        seek.assert_has_calls(
            [
                call(self.chunk_size, os.SEEK_CUR),
                call(self.chunk_size, os.SEEK_CUR),
            ]
        )
        self.assert_contents_equal(destination)

    def test_sparse_copy_preserves_trailing_zeroes(self) -> None:
        """A final hole still contributes to the image's logical file size."""

        contents = b"A" * self.chunk_size + b"\0" * self.chunk_size
        destination = self.directory_path / "trailing-zeroes.raw"

        with open(destination, "wb") as output:
            _copy_to_sparse_file(io.BytesIO(contents), output)

        self.assertEqual(len(contents), destination.stat().st_size)
        with open(destination, "rb") as input_file:
            self.assertEqual(contents, input_file.read())

    def test_non_disk_image_is_dense_when_sparse_true(self) -> None:
        """The sparse policy does not change ordinary file resources."""

        destination = self.directory_path / "regular-file"
        self.resource_json["category"] = "file"

        with patch(
            "gem5.resources.downloader._write_sparse_file",
            wraps=_write_sparse_file,
        ) as sparse_writer:
            self._obtain(destination)

        sparse_writer.assert_not_called()
        self.assert_contents_equal(destination)

    def test_sparse_false_uses_dense_decompression(self) -> None:
        """Users can explicitly request the legacy dense disk-image path."""

        destination = self.directory_path / "dense.raw"
        with patch(
            "gem5.resources.downloader._write_sparse_file",
            wraps=_write_sparse_file,
        ) as sparse_writer:
            self._obtain(destination, sparse=False)

        sparse_writer.assert_not_called()
        self.assert_contents_equal(destination)

    def test_disabled_gzip_extraction_preserves_archive(self) -> None:
        """A packaged image remains an archive when unzip is disabled."""

        destination = self.directory_path / "image.raw.gz.copy"
        with patch(
            "gem5.resources.downloader.get_resource_json_obj",
            return_value=self.resource_json,
        ), patch(
            "gem5.resources.downloader._write_sparse_file",
            wraps=_write_sparse_file,
        ) as sparse_writer:
            get_resource(
                resource_name=self.resource_json["id"],
                to_path=str(destination),
                resource_version=self.resource_json["resource_version"],
                unzip=False,
            )

        sparse_writer.assert_not_called()
        self.assertEqual(
            self.compressed_path.read_bytes(), destination.read_bytes()
        )

    def test_tar_disk_image_retains_dense_extraction(self) -> None:
        """Sparse-by-default does not reject legacy tar disk images."""

        archive = self.directory_path / "image.tar"
        archive_member = self.directory_path / "image.raw"
        archive_member.write_bytes(b"disk image contents")
        with tarfile.open(archive, "w") as output:
            output.add(archive_member, arcname=archive_member.name)

        destination = self.directory_path / "extracted"
        self.resource_json.update(
            {
                "url": archive.as_uri(),
                "is_zipped": False,
                "is_tar_archive": True,
            }
        )
        with patch(
            "gem5.resources.downloader.get_resource_json_obj",
            return_value=self.resource_json,
        ), patch(
            "gem5.resources.downloader._write_sparse_file",
            wraps=_write_sparse_file,
        ) as sparse_writer:
            get_resource(
                resource_name=self.resource_json["id"],
                to_path=str(destination),
                resource_version=self.resource_json["resource_version"],
            )

        sparse_writer.assert_not_called()
        self.assertEqual(
            b"disk image contents",
            (destination / archive_member.name).read_bytes(),
        )

    def test_disabled_tar_extraction_preserves_archive(self) -> None:
        """A packaged image remains an archive when untar is disabled."""

        archive = self.directory_path / "image.tar"
        archive_member = self.directory_path / "image.raw"
        archive_member.write_bytes(b"disk image contents")
        with tarfile.open(archive, "w") as output:
            output.add(archive_member, arcname=archive_member.name)

        destination = self.directory_path / "image.tar.copy"
        self.resource_json.update(
            {
                "url": archive.as_uri(),
                "is_zipped": False,
                "is_tar_archive": True,
            }
        )
        with patch(
            "gem5.resources.downloader.get_resource_json_obj",
            return_value=self.resource_json,
        ), patch(
            "gem5.resources.downloader._write_sparse_file",
            wraps=_write_sparse_file,
        ) as sparse_writer:
            get_resource(
                resource_name=self.resource_json["id"],
                to_path=str(destination),
                resource_version=self.resource_json["resource_version"],
                untar=False,
            )

        sparse_writer.assert_not_called()
        self.assertEqual(archive.read_bytes(), destination.read_bytes())

    def test_sparse_true_converts_a_valid_cached_file(self) -> None:
        """A valid dense cached disk image is replaced by a sparse copy."""

        destination = self.directory_path / "cached.raw"
        with open(destination, "wb") as output:
            output.write(self.contents)

        with patch(
            "gem5.resources.downloader._sparsify_file",
            wraps=_sparsify_file,
        ) as sparsifier:
            self._obtain(destination, sparse=True)

        sparsifier.assert_called_once()
        self.assert_contents_equal(destination)

    def test_sparse_false_converts_a_valid_cached_file(self) -> None:
        """An explicit dense request rewrites a cached sparse image."""

        destination = self.directory_path / "cached.raw"
        with open(destination, "wb") as output:
            _copy_to_sparse_file(io.BytesIO(self.contents), output)

        with patch(
            "gem5.resources.downloader._file_has_holes",
            return_value=True,
        ), patch(
            "gem5.resources.downloader._densify_file",
            wraps=_densify_file,
        ) as densifier:
            self._obtain(destination, sparse=False)

        densifier.assert_called_once()
        self.assert_contents_equal(destination)

    def test_legacy_disk_image_is_sparse_by_default(self) -> None:
        """Legacy generic resources with partitions are disk images."""

        destination = self.directory_path / "legacy.raw"
        self.resource_json.update(
            {
                "category": "resource",
                "root_partition": "1",
            }
        )
        with patch(
            "gem5.resources.downloader.get_resource_json_obj",
            return_value=self.resource_json,
        ), patch(
            "gem5.resources.downloader._write_sparse_file",
            wraps=_write_sparse_file,
        ) as sparse_writer:
            get_resource(
                resource_name=self.resource_json["id"],
                to_path=str(destination),
                resource_version=self.resource_json["resource_version"],
            )

        sparse_writer.assert_called_once()
        self.assert_contents_equal(destination)

    def test_sparse_local_copy_preserves_source_permissions(self) -> None:
        """A sparse file-URI copy retains the source file mode."""

        destination = self.directory_path / "mode.raw"
        os.chmod(self.compressed_path, 0o640)

        self._obtain(destination)

        self.assertEqual(0o640, stat.S_IMODE(destination.stat().st_mode))

    def test_sparse_download_uses_normal_file_permissions(self) -> None:
        """A remote sparse download is not left at mkstemp's 0600 mode."""

        response = io.BytesIO(self.contents)
        response.headers = {"Content-Length": str(len(self.contents))}
        destination = self.directory_path / "downloaded.raw"
        current_umask = os.umask(0)
        os.umask(current_umask)

        with patch(
            "gem5.resources.downloader.urllib.request.urlopen",
            return_value=response,
        ):
            _download_to_sparse_file(
                url="https://example.com/image.raw",
                to_path=str(destination),
                decompress=False,
                expected_md5=self.resource_json["md5sum"],
            )

        self.assertEqual(
            0o666 & ~current_umask,
            stat.S_IMODE(destination.stat().st_mode),
        )

    def test_dense_opt_out_propagates_through_workload(self) -> None:
        """Nested workload disk images receive the public sparse option."""

        workload = {
            "category": "workload",
            "id": "test-workload",
            "resource_version": "1.0.0",
            "function": "set_kernel_disk_workload",
            "resources": {
                "disk_image": {
                    "id": self.resource_json["id"],
                    "resource_version": self.resource_json["resource_version"],
                }
            },
        }
        disk_image = dict(self.resource_json)
        disk_image["root_partition"] = "1"

        with patch(
            "gem5.resources.resource.get_multiple_resource_json_obj",
            return_value=[disk_image],
        ):
            translated = _get_workload(
                workload=workload,
                local_path=str(self.directory_path / "nested.raw"),
                resource_directory=str(self.directory_path),
                download_md5_mismatch=True,
                clients=[],
                gem5_version="develop",
                quiet=True,
                sparse=False,
            )

        nested_resource = translated["parameters"]["disk_image"]
        self.assertFalse(nested_resource._downloader.keywords["sparse"])

    def test_dense_opt_out_propagates_through_suite(self) -> None:
        """Suite expansion forwards the public option to each workload."""

        suite = {
            "category": "suite",
            "id": "test-suite",
            "resource_version": "1.0.0",
            "workloads": [
                {
                    "id": "test-workload",
                    "resource_version": "1.0.0",
                    "input_group": ["test"],
                }
            ],
        }
        workload = {
            "category": "workload",
            "id": "test-workload",
            "resource_version": "1.0.0",
            "function": "set_kernel_disk_workload",
            "parameters": {},
        }
        with patch(
            "gem5.resources.resource.get_multiple_resource_json_obj",
            return_value=[workload],
        ), patch(
            "gem5.resources.resource._get_workload",
            return_value=workload,
        ) as get_workload:
            _get_suite(
                suite=suite,
                local_path=str(self.directory_path),
                resource_directory=str(self.directory_path),
                download_md5_mismatch=True,
                clients=[],
                gem5_version="develop",
                quiet=True,
                sparse=False,
            )

        self.assertFalse(get_workload.call_args.kwargs["sparse"])

    def test_checksum_failure_does_not_install_partial_file(self) -> None:
        """Bad logical contents leave neither a destination nor a part file."""

        destination = self.directory_path / "invalid.raw"
        self.resource_json["md5sum"] = "0" * 32

        with self.assertRaisesRegex(Exception, "invalid MD5 checksum"):
            self._obtain(destination)

        self.assertFalse(destination.exists())
        self.assertEqual([], list(self.directory_path.glob(".*.part")))

    def test_streaming_gzip_download(self) -> None:
        """Gzip input is decompressed into its destination without a gzip file."""

        destination = self.directory_path / "streamed.raw"
        _download_to_sparse_file(
            url=self.compressed_path.as_uri(),
            to_path=str(destination),
            decompress=True,
            expected_md5=self.resource_json["md5sum"],
        )

        self.assert_contents_equal(destination)
        self.assertFalse(Path(f"{destination}.gz").exists())

    def test_streaming_download_retries_connection_reset(self) -> None:
        """Connection resets use the existing retry path on every host OS."""

        class ResetResponse(io.BytesIO):
            headers = {"Content-Length": "1"}

            def read(self, *args, **kwargs):
                raise ConnectionResetError(54, "Connection reset by peer")

        compressed = gzip.compress(self.contents)
        successful_response = io.BytesIO(compressed)
        successful_response.headers = {"Content-Length": str(len(compressed))}
        destination = self.directory_path / "retried.raw"

        with patch(
            "gem5.resources.downloader.urllib.request.urlopen",
            side_effect=[ResetResponse(), successful_response],
        ) as urlopen, patch("gem5.resources.downloader.time.sleep"):
            _download_to_sparse_file(
                url="https://example.com/image.raw.gz",
                to_path=str(destination),
                decompress=True,
                expected_md5=self.resource_json["md5sum"],
            )

        self.assertEqual(2, urlopen.call_count)
        self.assert_contents_equal(destination)
        self.assertEqual([], list(self.directory_path.glob(".*.part")))

    def test_streaming_gzip_download_retries_truncated_response(self) -> None:
        """Truncated gzip responses are retried without installing a part."""

        compressed = gzip.compress(self.contents)
        truncated_response = io.BytesIO(compressed[:-8])
        truncated_response.headers = {
            "Content-Length": str(len(truncated_response.getvalue()))
        }
        successful_response = io.BytesIO(compressed)
        successful_response.headers = {"Content-Length": str(len(compressed))}
        destination = self.directory_path / "retried-truncated.raw"

        with patch(
            "gem5.resources.downloader.urllib.request.urlopen",
            side_effect=[truncated_response, successful_response],
        ) as urlopen, patch("gem5.resources.downloader.time.sleep"):
            _download_to_sparse_file(
                url="https://example.com/image.raw.gz",
                to_path=str(destination),
                decompress=True,
                expected_md5=self.resource_json["md5sum"],
            )

        self.assertEqual(2, urlopen.call_count)
        self.assert_contents_equal(destination)
        self.assertEqual([], list(self.directory_path.glob(".*.part")))

    def test_sparse_must_be_a_boolean(self) -> None:
        """The public option rejects the former ``None`` and non-bool values."""

        for invalid_value in (None, "yes"):
            with self.subTest(invalid_value=invalid_value):
                with self.assertRaisesRegex(
                    TypeError, "sparse must be a bool"
                ):
                    obtain_resource(
                        resource_id=self.resource_json["id"],
                        sparse=invalid_value,
                    )

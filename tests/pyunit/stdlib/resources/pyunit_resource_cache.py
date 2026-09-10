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

import contextlib
import copy
import gzip
import hashlib
import io
import runpy
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from gem5.resources.downloader import _write_sparse_file
from gem5.resources.resource import obtain_resource
from gem5.utils.filelock import (
    FileLock,
    FileLockException,
)


class ResourceCacheTestSuite(unittest.TestCase):
    """Exercise cache trust, download integrity, and real lock contention."""

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.destination = self.root / "test-image-1.0.0"
        self.contents = b"image contents" + b"\0" * 4096
        source = self.root / "source.gz"
        source.write_bytes(gzip.compress(self.contents))
        self.metadata = {
            "id": "test-image",
            "category": "disk-image",
            "resource_version": "1.0.0",
            "url": source.as_uri(),
            "is_zipped": True,
            "md5sum": hashlib.md5(self.contents).hexdigest(),
            "root_partition": "1",
        }
        for module in ("resource", "downloader"):
            patcher = patch(
                f"gem5.resources.{module}.get_resource_json_obj",
                return_value=self.metadata,
            )
            patcher.start()
            self.addCleanup(patcher.stop)

    def obtain(self, **kwargs):
        return obtain_resource(
            "test-image", resource_directory=str(self.root), **kwargs
        ).get_local_path()

    def run_resource_cli(self, *args):
        script = (
            Path(__file__).resolve().parents[4] / "util/obtain-resource.py"
        )
        stdout = io.StringIO()
        stderr = io.StringIO()
        argv = [str(script), "test-image", "-p", str(self.destination), *args]
        with patch.object(sys, "argv", argv), contextlib.redirect_stdout(
            stdout
        ), contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as exit_status:
                runpy.run_path(
                    str(script),
                    run_name="__m5_main__",
                    init_globals={"exit": sys.exit},
                )
        self.assertEqual(0, exit_status.exception.code)
        return stdout.getvalue(), stderr.getvalue()

    def test_cli_materializes_missing_resource_in_both_output_modes(self):
        for quiet in (False, True):
            with self.subTest(quiet=quiet), patch(
                "gem5.resources.downloader.md5_file"
            ) as cached_hash:
                stdout, _ = self.run_resource_cli(*(["-q"] if quiet else []))
                self.assertEqual(self.contents, self.destination.read_bytes())
                self.assertEqual(not quiet, "Resource at:" in stdout)
                # Printing the result must not trigger a second acquisition.
                cached_hash.assert_not_called()
                self.destination.unlink()

    def test_quiet_cli_warns_when_trusting_cached_resource(self):
        self.destination.write_bytes(b"trusted cache")
        with patch("gem5.resources.downloader.md5_file") as cached_hash:
            stdout, stderr = self.run_resource_cli(
                "-q", "--skip-cache-hash-check"
            )
        self.assertNotIn("Resource at:", stdout)
        self.assertEqual(1, stderr.count("without checking its hash"))
        cached_hash.assert_not_called()
        self.assertEqual(b"trusted cache", self.destination.read_bytes())

    def test_quiet_cli_respects_active_resource_lock(self):
        self.destination.write_bytes(self.contents)
        with FileLock(f"{self.destination}.lock") as owner:
            with self.assertRaises(FileLockException):
                self.run_resource_cli(
                    "-q", "--skip-cache-hash-check", "--lock-timeout", "0"
                )
            self.assertTrue(Path(owner.lockfile).exists())

    def test_default_rejects_bad_cached_contents(self):
        self.destination.write_bytes(b"bad image")
        with self.assertRaisesRegex(Exception, "md5 value is invalid"):
            self.obtain(download_md5_mismatch=False)
        self.assertFalse(Path(f"{self.destination}.lock.lock").exists())

    def test_opt_out_reuses_path_without_hashing_or_rewriting(self):
        self.destination.write_bytes(b"trusted without checking")
        for sparse in (True, False):
            with self.subTest(sparse=sparse), patch(
                "gem5.resources.downloader.md5_file"
            ) as hash_file, patch(
                "gem5.resources.downloader._sparsify_file"
            ) as sparsify, patch(
                "gem5.resources.downloader._densify_file"
            ) as densify, patch(
                "gem5.resources.downloader.warn"
            ) as warn:
                result = self.obtain(
                    skip_cache_hash_check=True,
                    download_md5_mismatch=False,
                    quiet=True,
                    sparse=sparse,
                )
                self.assertEqual(str(self.destination), result)
                hash_file.assert_not_called()
                sparsify.assert_not_called()
                densify.assert_not_called()
                warn.assert_called_once()
                self.assertIn(
                    "without checking its hash", warn.call_args.args[0]
                )
        self.assertEqual(
            b"trusted without checking", self.destination.read_bytes()
        )

    def test_opt_out_reuses_cached_directory(self):
        self.metadata["category"] = "directory"
        self.destination.mkdir()
        with patch("gem5.resources.downloader.md5_dir") as hash_dir, patch(
            "gem5.resources.downloader.warn"
        ) as warn:
            self.obtain(skip_cache_hash_check=True)
        hash_dir.assert_not_called()
        warn.assert_called_once()

    def test_quiet_opt_out_still_prints_warning(self):
        self.destination.write_bytes(self.contents)
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            self.obtain(skip_cache_hash_check=True, quiet=True)
        self.assertIn("without checking its hash", output.getvalue())

    def test_opt_out_still_downloads_missing_resource(self):
        with patch("gem5.resources.downloader.warn") as warn:
            self.obtain(skip_cache_hash_check=True)
        warn.assert_not_called()
        self.assertEqual(self.contents, self.destination.read_bytes())

    def test_opt_out_keeps_new_sparse_download_hash_validation(self):
        self.metadata["md5sum"] = "0" * 32
        with self.assertRaisesRegex(Exception, "invalid MD5 checksum"):
            self.obtain(skip_cache_hash_check=True)
        self.assertFalse(self.destination.exists())
        self.assertEqual([], list(self.root.glob(".*.part")))

    def test_opt_out_still_respects_active_lock(self):
        self.destination.write_bytes(self.contents)
        with FileLock(f"{self.destination}.lock") as owner:
            with self.assertRaisesRegex(FileLockException, "Timed out"):
                self.obtain(skip_cache_hash_check=True, lock_timeout=0)
            # A timed-out waiter must not delete the owner's lock.
            self.assertTrue(Path(owner.lockfile).exists())
        self.assertFalse(Path(f"{self.destination}.lock.lock").exists())

    def test_concurrent_opt_out_requests_materialize_once(self):
        started = threading.Event()
        publish = threading.Event()
        errors = []
        results = []

        def write_resource(*args, **kwargs):
            # Pause the real download writer while it owns the resource lock.
            started.set()
            if not publish.wait(5):
                raise RuntimeError("The second resource request did not wait")
            return _write_sparse_file(*args, **kwargs)

        def publisher():
            try:
                results.append(
                    self.obtain(skip_cache_hash_check=True, lock_timeout=5)
                )
            except Exception as error:
                errors.append(error)

        output = io.StringIO()
        real_write = output.write

        def write(message):
            result = real_write(message)
            if "Waiting for resource" in message:
                publish.set()
            return result

        with patch(
            "gem5.resources.downloader._write_sparse_file",
            side_effect=write_resource,
        ) as writer, patch(
            "gem5.resources.downloader.md5_file"
        ) as hash_file, patch(
            "gem5.resources.downloader.warn"
        ) as warn, patch.object(
            output, "write", side_effect=write
        ), contextlib.redirect_stdout(
            output
        ):
            thread = threading.Thread(target=publisher)
            thread.start()
            try:
                self.assertTrue(started.wait(5))
                # Contention output lets the owner finish its download. Both
                # callers use the real exclusive-create lock and downloader.
                results.append(
                    self.obtain(skip_cache_hash_check=True, lock_timeout=5)
                )
            finally:
                publish.set()
                thread.join(6)
            self.assertFalse(thread.is_alive())
            self.assertEqual([], errors)
            self.assertEqual([str(self.destination)] * 2, results)
            writer.assert_called_once()
            hash_file.assert_not_called()
            warn.assert_called_once()
        self.assertIn("Waiting for resource", output.getvalue())
        self.assertEqual(self.contents, self.destination.read_bytes())
        self.assertFalse(Path(f"{self.destination}.lock.lock").exists())

    def test_invalid_cache_option(self):
        for value in (None, "false", 1):
            with self.subTest(value=value), self.assertRaises(TypeError):
                self.obtain(skip_cache_hash_check=value)

    def test_invalid_lock_timeout(self):
        for value in (None, "60", True, -1, float("inf"), float("nan")):
            with self.subTest(value=value), self.assertRaises(
                (TypeError, ValueError)
            ):
                self.obtain(lock_timeout=value)

    def test_options_propagate_through_workload_and_suite(self):
        workload = {
            "category": "workload",
            "id": "test-workload",
            "resource_version": "1.0.0",
            "function": "set_kernel_disk_workload",
            "resources": {
                "disk_image": {"id": "test-image", "resource_version": "1.0.0"}
            },
        }
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
        self.destination.write_bytes(b"trusted cache")
        for top in (workload, suite):
            with self.subTest(category=top["category"]), patch(
                "gem5.resources.resource.get_resource_json_obj",
                return_value=copy.deepcopy(top),
            ), patch(
                "gem5.resources.resource.get_multiple_resource_json_obj",
                side_effect=(
                    [copy.deepcopy([workload]), [self.metadata]]
                    if top is suite
                    else [[self.metadata]]
                ),
            ), patch(
                "gem5.resources.downloader.warn"
            ) as warn:
                resource = obtain_resource(
                    top["id"],
                    resource_directory=str(self.root),
                    skip_cache_hash_check=True,
                    lock_timeout=0,
                )
                if top is suite:
                    resource = next(iter(resource))
                disk = resource.get_parameters()["disk_image"]
                self.assertEqual(str(self.destination), disk.get_local_path())
                warn.assert_called_once()
                with FileLock(f"{self.destination}.lock"), self.assertRaises(
                    FileLockException
                ):
                    disk.get_local_path()


class FileLockWaitTestSuite(unittest.TestCase):
    def test_reports_once_per_minute_and_preserves_owner_on_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            name = str(Path(directory) / "resource")
            notices = []
            with FileLock(name) as owner:
                with patch(
                    "gem5.utils.filelock.time.monotonic",
                    side_effect=[0, 0, 30, 60, 90],
                ), patch("gem5.utils.filelock.time.sleep"):
                    with self.assertRaisesRegex(
                        FileLockException, "90 seconds"
                    ):
                        with FileLock(
                            name, timeout=90, on_wait=notices.append
                        ):
                            self.fail("Acquired an already owned lock")
                self.assertTrue(Path(owner.lockfile).exists())
            self.assertEqual([0, 60], notices)

    def test_exception_releases_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            name = str(Path(directory) / "resource")
            with self.assertRaisesRegex(RuntimeError, "test"):
                with FileLock(name):
                    raise RuntimeError("test")
            with FileLock(name, timeout=0):
                pass

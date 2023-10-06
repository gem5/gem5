# Copyright (c) 2019, 2021 The Regents of the University of California
# All Rights Reserved.
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
"""
This file defines a gem5Run object which contains all information needed to
run a single gem5 test.

This class works closely with the artifact module to ensure that the gem5
experiment is reproducible and the output is saved to the database.
"""
import hashlib
import json
import os
import signal
import subprocess
import time
import zipfile
from pathlib import Path
from typing import Any
from typing import Callable
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple
from typing import Union
from uuid import UUID
from uuid import uuid4

from gem5art import artifact
from gem5art.artifact import Artifact
from gem5art.artifact._artifactdb import ArtifactDB


class gem5Run:
    """
    This class holds all of the info required to run gem5.
    """

    _id: UUID
    hash: str
    type: str
    name: str
    gem5_binary_path: Path
    run_script: Path
    gem5_artifact: Artifact
    gem5_git_artifact: Artifact
    run_script_git_artifact: Artifact
    params: Tuple[str, ...]
    timeout: int
    check_failure: Callable[["gem5Run"], bool]

    gem5_name: str
    script_name: str
    linux_name: str
    disk_name: str
    string: str

    outdir: Path

    linux_binary_path: Path
    disk_image_path: Path
    linux_binary_artifact: Artifact
    disk_image_artifact: Artifact

    command: List[str]

    running: bool
    enqueue_time: float
    start_time: float
    end_time: float
    return_code: int
    kill_reason: str
    status: str
    pid: int
    task_id: Any

    results: Optional[Artifact]
    artifacts: List[Artifact]

    rerunnable: bool

    @classmethod
    def _create(
        cls,
        name: str,
        run_script: Path,
        outdir: Path,
        gem5_artifact: Artifact,
        gem5_git_artifact: Artifact,
        run_script_git_artifact: Artifact,
        params: Tuple[str, ...],
        timeout: int,
        check_failure: Callable[["gem5Run"], bool],
    ) -> "gem5Run":
        """
        Shared code between SE and FS when creating a run object.
        """
        run = cls()
        run.name = name
        run.gem5_binary_path = gem5_artifact.path
        run.run_script = run_script
        run.gem5_artifact = gem5_artifact
        run.gem5_git_artifact = gem5_git_artifact
        run.run_script_git_artifact = run_script_git_artifact
        run.params = params
        run.timeout = timeout

        # Note: Mypy doesn't support monkey patching like this
        run.check_failure = check_failure  # type: ignore

        run._id = uuid4()

        run.outdir = outdir.resolve()  # ensure this is absolute

        # Assumes **/<gem5_name>/gem5.<anything>
        run.gem5_name = run.gem5_binary_path.parent.name
        # Assumes **/<script_name>.py
        run.script_name = run.run_script.stem

        # Info about the actual run
        run.running = False
        run.enqueue_time = time.time()
        run.start_time = 0.0
        run.end_time = 0.0
        run.return_code = 0
        run.kill_reason = ""
        run.status = "Created"
        run.pid = 0
        run.task_id = None

        # Initially, there are no results
        run.results = None

        run.rerunnable = False

        return run

    @classmethod
    def createSERun(
        cls,
        name: str,
        run_script: str,
        outdir: str,
        gem5_artifact: Artifact,
        gem5_git_artifact: Artifact,
        run_script_git_artifact: Artifact,
        *params: str,
        timeout: int = 60 * 15,
        check_failure: Callable[["gem5Run"], bool] = lambda run: False,
    ) -> "gem5Run":
        """
        name is the name of the run. The name is not necessarily unique. The
        name could be used to query the results of the run.

        run_script is the path to the run script to pass to gem5.

        The artifact parameters (gem5_artifact, gem5_git_artifact, and
        run_script_git_artifact) are used to ensure this is reproducible run.

        Further parameters can be passed via extra arguments. These
        parameters will be passed in order to the gem5 run script.
        timeout is the time in seconds to run the subprocess before killing it.

        Note: When instantiating this class for the first time, it will create
        a file `info.json` in the outdir which contains a serialized version
        of this class.
        """

        run = cls._create(
            name,
            Path(run_script),
            Path(outdir),
            gem5_artifact,
            gem5_git_artifact,
            run_script_git_artifact,
            params,
            timeout,
            check_failure,
        )

        run.artifacts = [
            gem5_artifact,
            gem5_git_artifact,
            run_script_git_artifact,
        ]

        run.string = f"{run.gem5_name} {run.script_name}"
        run.string += " ".join(run.params)

        run.command = [
            str(run.gem5_binary_path),
            "-re",
            f"--outdir={run.outdir}",
            str(run.run_script),
        ]
        run.command += list(params)

        run.hash = run._getHash()
        run.type = "gem5 run"

        # Make the directory if it doesn't exist
        os.makedirs(run.outdir, exist_ok=True)
        run.dumpJson("info.json")

        return run

    @classmethod
    def createFSRun(
        cls,
        name: str,
        run_script: str,
        outdir: str,
        gem5_artifact: Artifact,
        gem5_git_artifact: Artifact,
        run_script_git_artifact: Artifact,
        linux_binary_artifact: Artifact,
        disk_image_artifact: Artifact,
        *params: str,
        timeout: int = 60 * 15,
        check_failure: Callable[["gem5Run"], bool] = lambda run: False,
    ) -> "gem5Run":
        """
        name is the name of the run. The name is not necessarily unique. The
        name could be used to query the results of the run.

        run_script is the path to the run script to pass to gem5.

        Further parameters can be passed via extra arguments. These
        parameters will be passed in order to the gem5 run script.

        check_failure is a user-defined function that will be executed
        periodically (e.g., every 10 seconds) to check the health of the
        simulation. When it returns True, the simulation will be killed

        Note: When instantiating this class for the first time, it will create
        a file `info.json` in the outdir which contains a serialized version
        of this class.
        """
        run = cls._create(
            name,
            Path(run_script),
            Path(outdir),
            gem5_artifact,
            gem5_git_artifact,
            run_script_git_artifact,
            params,
            timeout,
            check_failure,
        )
        run.linux_binary_path = Path(linux_binary_artifact.path)
        run.disk_image_path = Path(disk_image_artifact.path)
        run.linux_binary_artifact = linux_binary_artifact
        run.disk_image_artifact = disk_image_artifact

        # Assumes **/<linux_name>
        run.linux_name = run.linux_binary_path.name
        # Assumes **/<disk_name>
        run.disk_name = disk_image_artifact.name

        run.artifacts = [
            gem5_artifact,
            gem5_git_artifact,
            run_script_git_artifact,
            linux_binary_artifact,
            disk_image_artifact,
        ]

        run.string = f"{run.gem5_name} {run.script_name} "
        run.string += f"{run.linux_name} {run.disk_name} "
        run.string += " ".join(run.params)
        run.command = [
            str(run.gem5_binary_path),
            "-re",
            f"--outdir={run.outdir}",
            str(run.run_script),
            str(run.linux_binary_path),
            str(run.disk_image_path),
        ]
        run.command += list(params)

        run.hash = run._getHash()
        run.type = "gem5 run fs"

        # Make the directory if it doesn't exist
        os.makedirs(run.outdir, exist_ok=True)
        run.dumpJson("info.json")

        return run

    @classmethod
    def loadJson(cls, filename: str) -> "gem5Run":
        with open(filename) as f:
            d = json.load(f)
            # Convert string version of UUID to UUID object
            for k, v in d.iteritems():
                if k.endswith("_artifact"):
                    d[k] = UUID(v)
            d["_id"] = UUID(d["_id"])
        try:
            return cls.loadFromDict(d)
        except KeyError:
            print(f"Incompatible json file: {filename}!")
            raise

    @classmethod
    def loadFromDict(cls, d: Dict[str, Union[str, UUID]]) -> "gem5Run":
        """Returns new gem5Run instance from the dictionary of values in d"""
        run = cls()
        run.artifacts = []
        for k, v in d.items():
            if isinstance(v, UUID) and k != "_id":
                a = Artifact(v)
                setattr(run, k, a)
                run.artifacts.append(a)
            else:
                setattr(run, k, v)
        return run

    def checkArtifacts(self, cwd: str) -> bool:
        """Checks to make sure all of the artifacts are up to date

        This should happen just before running gem5. This function will return
        False if the artifacts don't check and true if they are all the same.
        For the git repos, this checks the git hash, for binary artifacts this
        checks the md5 hash.
        """
        for v in self.artifacts:
            if v.type == "git repo":
                new = artifact.artifact.getGit(cwd / v.path)["hash"]
                old = v.git["hash"]
            else:
                new = artifact.artifact.getHash(cwd / v.path)
                old = v.hash

            if new != old:
                self.status = f"Failed artifact check for {cwd / v.path}"
                return False

        return True

    def __repr__(self) -> str:
        return str(self._getSerializable())

    def checkKernelPanic(self) -> bool:
        """
        Returns true if the gem5 instance specified in args has a kernel panic
        Note: this gets around the problem that gem5 doesn't exit on panics.
        """
        term_path = self.outdir / "system.pc.com_1.device"
        if not term_path.exists():
            return False

        with open(term_path, "rb") as f:
            try:
                f.seek(-1000, os.SEEK_END)
            except OSError:
                return False
            try:
                # There was a case where reading `term_path` resulted in a
                # UnicodeDecodeError. It is known that the terminal output
                # (content of 'system.pc.com_1.device') is written from a
                # buffer from gem5, and when gem5 stops, the content of the
                # buffer is stopped being copied to the file. The buffer is
                # not flushed as well. So, it might be a case that the content
                # of the `term_path` is corrupted as a Unicode character could
                # be longer than a byte.
                last = f.readlines()[-1].decode()
                if "Kernel panic" in last:
                    return True
                else:
                    return False
            except UnicodeDecodeError:
                return False

    def _getSerializable(self) -> Dict[str, Union[str, UUID]]:
        """Returns a dictionary that can be used to recreate this object

        Note: All artifacts are converted to a UUID instead of an Artifact.
        """
        # Grab all of the member variables
        d = vars(self).copy()

        # Remove list of artifacts
        del d["artifacts"]

        # Doesn't make sense to serialize the user-specified fail function
        if "check_failure" in d.keys():
            del d["check_failure"]

        # Replace the artifacts with their UUIDs
        for k, v in d.items():
            if isinstance(v, Artifact):
                d[k] = v._id
            if isinstance(v, Path):
                d[k] = str(v)

        return d

    def _getHash(self) -> str:
        """Return a single value that uniquely identifies this run

        To uniquely identify this run, the gem5 binary, gem5 scripts, and
        parameters should all match. Thus, let's make a single hash out of the
        artifacts + the runscript + parameters
        """
        to_hash = [art._id.bytes for art in self.artifacts]
        to_hash.append(str(self.run_script).encode())
        to_hash.append(" ".join(self.params).encode())

        return hashlib.md5(b"".join(to_hash)).hexdigest()

    @classmethod
    def _convertForJson(cls, d: Dict[str, Any]) -> Dict[str, str]:
        """Converts UUID objects to strings for json compatibility"""
        for k, v in d.items():
            if isinstance(v, UUID):
                d[k] = str(v)
        return d

    def dumpJson(self, filename: str) -> None:
        """Dump all info into a json file"""
        d = self._convertForJson(self._getSerializable())
        with open(self.outdir / filename, "w") as f:
            json.dump(d, f)

    def dumpsJson(self) -> str:
        """Like dumpJson except returns string"""
        d = self._convertForJson(self._getSerializable())
        return json.dumps(d)

    def _run(self, task: Any = None, cwd: str = ".") -> None:
        """Actually run the test.

        Calls Popen with the command to fork a new process.
        Then, this function polls the process every 5 seconds to check if it
        has finished or not. Each time it checks, it dumps the json info so
        other applications can poll those files.

        task is the celery task that is running this gem5 instance.

        cwd is the directory to change to before running. This allows a server
        process to run in a different directory than the running process. Note
        that only the spawned process runs in the new directory.
        """
        # Connect to the database
        db = artifact.getDBConnection()

        self.status = "Begin run"
        self.dumpJson("info.json")

        if not self.checkArtifacts(cwd):
            self.dumpJson("info.json")
            return

        self.status = "Spawning"

        self.start_time = time.time()
        self.task_id = task.request.id if task else None
        self.dumpJson("info.json")

        # Start running the gem5 command
        proc = subprocess.Popen(self.command, cwd=cwd)

        # Register handler in case this process is killed while the gem5
        # instance is running. Note: there's a bit of a race condition here,
        # but hopefully it's not a big deal
        def handler(signum, frame):
            proc.kill()
            self.kill_reason = "sigterm"
            self.dumpJson("info.json")
            # Note: We'll fall out of the while loop after this.

        # This makes it so if you term *this* process, it will actually kill
        # the subprocess and then this process will die.
        signal.signal(signal.SIGTERM, handler)

        # Do this until the subprocess is done (successfully or not)
        while proc.poll() is None:
            self.status = "Running"
            # Still running
            self.current_time = time.time()
            self.pid = proc.pid
            self.running = True

            if self.current_time - self.start_time > self.timeout:
                proc.kill()
                self.kill_reason = "timeout"

            if self.checkKernelPanic():
                proc.kill()
                self.kill_reason = "kernel panic"

            # Assigning a function/lambda to an object variable does not make
            # the function/lambda become a bound one. Therefore, the
            # user-defined function must pass `self` in.
            # Here, mypy classifies self.check_failure() as a bound function,
            # so we tell mypy to ignore it./
            if self.check_failure(self):  # type: ignore
                proc.kill()
                self.kill_reason = "User defined kill"

            self.dumpJson("info.json")

            # Check again in five seconds
            time.sleep(5)

        print(f"Done running {' '.join(self.command)}")

        # Done executing
        self.running = False
        self.end_time = time.time()
        self.return_code = proc.returncode

        if self.return_code == 0:
            self.status = "Finished"
        else:
            self.status = "Failed"

        self.dumpJson("info.json")

        self.saveResults()

        # Store current gem5 run in the database
        db.put(self._id, self._getSerializable())

        print(f"Done storing the results of {' '.join(self.command)}")

    def run(self, task: Any = None, cwd: str = ".") -> None:
        """Actually run the test.

        Calls Popen with the command to fork a new process.
        Then, this function polls the process every 5 seconds to check if it
        has finished or not. Each time it checks, it dumps the json info so
        other applications can poll those files.

        task is the celery task that is running this gem5 instance.

        cwd is the directory to change to before running. This allows a server
        process to run in a different directory than the running process. Note
        that only the spawned process runs in the new directory.
        """
        # Check if the run is already in the database
        db = artifact.getDBConnection()
        if self.hash in db:
            print(f"Error: Have already run {self.command}. Exiting!")
            return
        self._run(task, cwd)

    def rerun(self, task: Any = None, cwd: str = ".") -> None:
        """Rerun the test.

        Calls Popen with the command to fork a new process.
        Then, this function polls the process every 5 seconds to check if it
        has finished or not. Each time it checks, it dumps the json info so
        other applications can poll those files.

        task is the celery task that is running this gem5 instance.

        cwd is the directory to change to before running. This allows a server
        process to run in a different directory than the running process. Note
        that only the spawned process runs in the new directory.
        """
        # TODO: remove the old runs?
        self._run(task, cwd)

    def saveResults(self) -> None:
        """Zip up the output directory and store the results in the
        database."""

        with zipfile.ZipFile(
            self.outdir / "results.zip",
            "w",
            zipfile.ZIP_DEFLATED,
        ) as zipf:
            for path in self.outdir.glob("**/*"):
                if path.name == "results.zip":
                    continue
                zipf.write(path, path.relative_to(self.outdir.parent))

        self.results = Artifact.registerArtifact(
            command=f"zip results.zip -r {self.outdir}",
            name=self.name,
            typ="directory",
            path=self.outdir / "results.zip",
            cwd="./",
            documentation="Compressed version of the results directory",
        )

    def __str__(self) -> str:
        return self.string + " -> " + self.status


def getRuns(
    db: ArtifactDB,
    fs_only: bool = False,
    limit: int = 0,
) -> Iterable[gem5Run]:
    """Returns a generator of gem5Run objects.

    If fs_only is True, then only full system runs will be returned.
    Limit specifies the maximum number of runs to return.
    """

    if not fs_only:
        runs = db.searchByType("gem5 run", limit=limit)
        for run in runs:
            yield gem5Run.loadFromDict(run)

    fsruns = db.searchByType("gem5 run fs", limit=limit)
    for run in fsruns:
        yield gem5Run.loadFromDict(run)


def getRunsByName(
    db: ArtifactDB,
    name: str,
    fs_only: bool = False,
    limit: int = 0,
) -> Iterable[gem5Run]:
    """Returns a generator of gem5Run objects, which have the field "name"
    **exactly** the same as the name parameter. The name used in this query
    is case sensitive.

    If fs_only is True, then only full system runs will be returned.
    Limit specifies the maximum number of runs to return.
    """

    if not fs_only:
        seruns = db.searchByNameType(name, "gem5 run", limit=limit)
        for run in seruns:
            yield gem5Run.loadFromDict(run)

    fsruns = db.searchByNameType(name, "gem5 run fs", limit=limit)

    for run in fsruns:
        yield gem5Run.loadFromDict(run)


def getRunsByNameLike(
    db: ArtifactDB,
    name: str,
    fs_only: bool = False,
    limit: int = 0,
) -> Iterable[gem5Run]:
    """Return a generator of gem5Run objects, which have the field "name"
    containing the name parameter as a substring. The name used in this
    query is case sensitive.

    If fs_only is True, then only full system runs will be returned.
    Limit specifies the maximum number of runs to return.
    """

    if not fs_only:
        seruns = db.searchByLikeNameType(name, "gem5 run", limit=limit)

        for run in seruns:
            yield gem5Run.loadFromDict(run)

    fsruns = db.searchByLikeNameType(name, "gem5 run fs", limit=limit)

    for run in fsruns:
        yield gem5Run.loadFromDict(run)


def getRerunnableRunsByNameLike(
    db: ArtifactDB,
    name: str,
    fs_only: bool = False,
    limit: int = 0,
) -> Iterable[gem5Run]:
    """Returns a generator of gem5Run objects having rerunnable as true
    and the object "name" containing the name parameter as a substring. The
    parameter is case sensitive.

    If fs_only is True, then only full system runs will be returned.
    Limit specifies the maximum number of runs to return.
    """

    for run in getRunsByNameLike(db, name, fs_only, limit):
        if run.rerunnable:
            yield run

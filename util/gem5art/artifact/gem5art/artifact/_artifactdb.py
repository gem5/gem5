# Copyright (c) 2019-2021 The Regents of the University of California
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
"""This file defines the ArtifactDB type and some common implementations of
ArtifactDB.

The database interface defined here does not include any schema information.
The database "schema" is defined in the artifact.py file based on the types of
artifacts stored in the database.

Some common queries can be found in common_queries.py
"""
import copy
import json
import os
import shutil
from abc import ABC
from abc import abstractmethod
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Tuple
from typing import Type
from typing import Union
from urllib.parse import urlparse
from uuid import UUID

try:
    import gridfs  # type: ignore
    from pymongo import MongoClient  # type: ignore

    MONGO_SUPPORT = True
except ModuleNotFoundError:
    # If pymongo isn't installed, then disable support for it
    MONGO_SUPPORT = False


class ArtifactDB(ABC):
    """
    Abstract base class for all artifact DBs.
    """

    @abstractmethod
    def __init__(self, uri: str) -> None:
        """Initialize the database with a URI"""
        pass

    @abstractmethod
    def put(self, key: UUID, artifact: Dict[str, Union[str, UUID]]) -> None:
        """Insert the artifact into the database with the key"""
        pass

    @abstractmethod
    def upload(self, key: UUID, path: Path) -> None:
        """Upload the file at path to the database with _id of key"""
        pass

    @abstractmethod
    def __contains__(self, key: Union[UUID, str]) -> bool:
        """Key can be a UUID or a string. Returns true if item in DB"""
        pass

    @abstractmethod
    def get(self, key: Union[UUID, str]) -> Dict[str, str]:
        """Key can be a UUID or a string. Returns a dictionary to construct
        an artifact.
        """
        pass

    @abstractmethod
    def downloadFile(self, key: UUID, path: Path) -> None:
        """Download the file with the _id key to the path. Will overwrite the
        file if it currently exists."""
        pass

    def searchByName(self, name: str, limit: int) -> Iterable[Dict[str, Any]]:
        """Returns an iterable of all artifacts in the database that match
        some name. Note: Not all DB implementations will implement this
        function"""
        raise NotImplementedError()

    def searchByType(self, typ: str, limit: int) -> Iterable[Dict[str, Any]]:
        """Returns an iterable of all artifacts in the database that match
        some type. Note: Not all DB implementations will implement this
        function"""
        raise NotImplementedError()

    def searchByNameType(
        self,
        name: str,
        typ: str,
        limit: int,
    ) -> Iterable[Dict[str, Any]]:
        """Returns an iterable of all artifacts in the database that match
        some name and type. Note: Not all DB implementations will implement
        this function"""
        raise NotImplementedError()

    def searchByLikeNameType(
        self,
        name: str,
        typ: str,
        limit: int,
    ) -> Iterable[Dict[str, Any]]:
        """Returns an iterable of all artifacts in the database that match
        some type and a regex name. Note: Not all DB implementations will
        implement this function"""
        raise NotImplementedError()


class ArtifactMongoDB(ArtifactDB):
    """
    This is a mongodb database connector for storing Artifacts (as defined in
    artifact.py).

    This database stores the data in three collections:
    - artifacts: This stores the json serialized Artifact class
    - files and chunks: These two collections store the large files required
      for some artifacts. Within the files collection, the _id is the
      UUID of the artifact.
    """

    def __init__(self, uri: str) -> None:
        """Initialize the mongodb connection and grab pointers to the databases
        uri is the location of the database in a mongodb compatible form.
        http://dochub.mongodb.org/core/connections.
        """
        # Note: Need "connect=False" so that we don't connect until the first
        # time we interact with the database. Required for the gem5 running
        # celery server
        self.db = MongoClient(host=uri, connect=False).artifact_database
        self.artifacts = self.db.artifacts
        self.fs = gridfs.GridFSBucket(self.db, disable_md5=True)

    def put(self, key: UUID, artifact: Dict[str, Union[str, UUID]]) -> None:
        """Insert the artifact into the database with the key"""
        assert artifact["_id"] == key
        self.artifacts.insert_one(artifact)

    def upload(self, key: UUID, path: Path) -> None:
        """Upload the file at path to the database with _id of key"""
        with open(path, "rb") as f:
            self.fs.upload_from_stream_with_id(key, str(path), f)

    def __contains__(self, key: Union[UUID, str]) -> bool:
        """Key can be a UUID or a string. Returns true if item in DB"""
        if isinstance(key, UUID):
            count = self.artifacts.count_documents({"_id": key}, limit=1)
        else:
            # This is a hash. Count the number of matches
            count = self.artifacts.count_documents({"hash": key}, limit=1)

        return bool(count > 0)

    def get(self, key: Union[UUID, str]) -> Dict[str, str]:
        """Key can be a UUID or a string. Returns a dictionary to construct
        an artifact.
        """
        if isinstance(key, UUID):
            return self.artifacts.find_one({"_id": key}, limit=1)
        else:
            # This is a hash.
            return self.artifacts.find_one({"hash": key}, limit=1)

    def downloadFile(self, key: UUID, path: Path) -> None:
        """Download the file with the _id key to the path. Will overwrite the
        file if it currently exists."""
        with open(path, "wb") as f:
            self.fs.download_to_stream(key, f)

    def searchByName(self, name: str, limit: int) -> Iterable[Dict[str, Any]]:
        """Returns an iterable of all artifacts in the database that match
        some name."""
        yield from self.artifacts.find({"name": name}, limit=limit)

    def searchByType(self, typ: str, limit: int) -> Iterable[Dict[str, Any]]:
        """Returns an iterable of all artifacts in the database that match
        some type."""
        yield from self.artifacts.find({"type": typ}, limit=limit)

    def searchByNameType(
        self,
        name: str,
        typ: str,
        limit: int,
    ) -> Iterable[Dict[str, Any]]:
        """Returns an iterable of all artifacts in the database that match
        some name and type."""
        yield from self.artifacts.find(
            {"type": typ, "name": name},
            limit=limit,
        )

    def searchByLikeNameType(
        self,
        name: str,
        typ: str,
        limit: int,
    ) -> Iterable[Dict[str, Any]]:
        """Returns an iterable of all artifacts in the database that match
        some type and a regex name."""

        data = self.artifacts.find(
            {"type": typ, "name": {"$regex": f"{name}"}},
            limit=limit,
        )
        yield from data


class ArtifactFileDB(ArtifactDB):
    """
    This is a file-based database where Artifacts (as defined in artifacts.py)
    are stored in a JSON file.

    This database stores a list of serialized artifacts in a JSON file.
    This database is not thread-safe.

    If the user specifies a valid path in the environment variable
    GEM5ART_STORAGE then this database will copy all artifacts to that
    directory named with their UUIDs.
    """

    class ArtifactEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, UUID):
                return str(obj)
            return ArtifactFileDB.ArtifactEncoder(self, obj)

    _json_file: Path
    _uuid_artifact_map: Dict[str, Dict[str, str]]
    _hash_uuid_map: Dict[str, List[str]]
    _storage_enabled: bool
    _storage_path: Path

    def __init__(self, uri: str) -> None:
        """Initialize the file-driven database from a JSON file.
        If the file doesn't exist, a new file will be created.
        """
        parsed_uri = urlparse(uri)
        # using urlparse to parse relative/absolute file path
        # abs path: urlparse("file:///path/to/file") ->
        #           (netloc='', path='/path/to/file')
        # rel path: urlparse("file://path/to/file") ->
        #           (netloc='path', path='/to/file')
        # so, the filepath would be netloc+path for both cases
        self._json_file = Path(parsed_uri.netloc) / Path(parsed_uri.path)
        storage_path = os.environ.get("GEM5ART_STORAGE", "")
        self._storage_enabled = True if storage_path else False
        self._storage_path = Path(storage_path)
        if (
            self._storage_enabled
            and self._storage_path.exists()
            and not self._storage_path.is_dir()
        ):
            raise Exception(
                f"GEM5ART_STORAGE={storage_path} exists and is not a directory",
            )
        if self._storage_enabled:
            os.makedirs(self._storage_path, exist_ok=True)

        self._uuid_artifact_map, self._hash_uuid_map = self._load_from_file(
            self._json_file,
        )

    def put(self, key: UUID, artifact: Dict[str, Union[str, UUID]]) -> None:
        """Insert the artifact into the database with the key."""
        assert artifact["_id"] == key
        assert isinstance(artifact["hash"], str)
        self.insert_artifact(key, artifact["hash"], artifact)

    def upload(self, key: UUID, path: Path) -> None:
        """Copy the artifact to the folder specified by GEM5ART_STORAGE."""
        if not self._storage_enabled:
            return
        src_path = path
        dst_path = self._storage_path / str(key)
        if not dst_path.exists():
            shutil.copy2(src_path, dst_path)

    def __contains__(self, key: Union[UUID, str]) -> bool:
        """Key can be a UUID or a string. Returns true if item in DB"""
        if isinstance(key, UUID):
            return self.has_uuid(key)
        return self.has_hash(key)

    def get(self, key: Union[UUID, str]) -> Dict[str, str]:
        """Key can be a UUID or a string. Returns a dictionary to construct
        an artifact.
        """
        artifact: List[Dict[str, str]] = []
        if isinstance(key, UUID):
            artifact = list(self.get_artifact_by_uuid(key))
        else:
            # This is a hash.
            artifact = list(self.get_artifact_by_hash(key))
        return artifact[0]

    def downloadFile(self, key: UUID, path: Path) -> None:
        """Copy the file from the storage to specified path."""
        assert path.exists()
        if not self._storage_enabled:
            return
        src_path = self._storage_path / str(key)
        dst_path = path
        shutil.copy2(src_path, dst_path)

    def _load_from_file(
        self,
        json_file: Path,
    ) -> Tuple[Dict[str, Dict[str, str]], Dict[str, List[str]]]:
        uuid_mapping: Dict[str, Dict[str, str]] = {}
        hash_mapping: Dict[str, List[str]] = {}
        if json_file.exists():
            with open(json_file) as f:
                j = json.load(f)
                for an_artifact in j:
                    the_uuid = an_artifact["_id"]
                    the_hash = an_artifact["hash"]
                    uuid_mapping[the_uuid] = an_artifact
                    if not the_hash in hash_mapping:
                        hash_mapping[the_hash] = []
                    hash_mapping[the_hash].append(the_uuid)
        return uuid_mapping, hash_mapping

    def _save_to_file(self, json_file: Path) -> None:
        content = list(self._uuid_artifact_map.values())
        with open(json_file, "w") as f:
            json.dump(content, f, indent=4, cls=ArtifactFileDB.ArtifactEncoder)

    def has_uuid(self, the_uuid: UUID) -> bool:
        return str(the_uuid) in self._uuid_artifact_map

    def has_hash(self, the_hash: str) -> bool:
        return the_hash in self._hash_uuid_map

    def get_artifact_by_uuid(self, the_uuid: UUID) -> Iterable[Dict[str, str]]:
        uuid_str = str(the_uuid)
        if not uuid_str in self._uuid_artifact_map:
            return
        yield self._uuid_artifact_map[uuid_str]

    def get_artifact_by_hash(self, the_hash: str) -> Iterable[Dict[str, str]]:
        if not the_hash in self._hash_uuid_map:
            return
        for the_uuid in self._hash_uuid_map[the_hash]:
            yield self._uuid_artifact_map[the_uuid]

    def insert_artifact(
        self,
        the_uuid: UUID,
        the_hash: str,
        the_artifact: Dict[str, Union[str, UUID]],
    ) -> bool:
        """
        Put the artifact to the database.

        Return True if the artifact uuid does not exist in the database prior
        to calling this function; return False otherwise.
        """
        uuid_str = str(the_uuid)
        if uuid_str in self._uuid_artifact_map:
            return False
        artifact_copy = copy.deepcopy(the_artifact)
        artifact_copy["_id"] = str(artifact_copy["_id"])
        self._uuid_artifact_map[uuid_str] = artifact_copy  # type: ignore
        if not the_hash in self._hash_uuid_map:
            self._hash_uuid_map[the_hash] = []
        self._hash_uuid_map[the_hash].append(uuid_str)
        self._save_to_file(self._json_file)
        return True

    def find_exact(
        self,
        attr: Dict[str, str],
        limit: int,
    ) -> Iterable[Dict[str, Any]]:
        """
        Return all artifacts such that, for every yielded artifact,
        and for every (k,v) in attr, the attribute `k` of the artifact has
        the value of `v`.
        """
        count = 0
        if count >= limit:
            return
        for artifact in self._uuid_artifact_map.values():
            # https://docs.python.org/3/library/stdtypes.html#frozenset.issubset
            if attr.items() <= artifact.items():
                yield artifact


_db = None

if MONGO_SUPPORT:
    _default_uri = "mongodb://localhost:27017"
else:
    _default_uri = "file://db.json"

_db_schemes: Dict[str, Type[ArtifactDB]] = {"file": ArtifactFileDB}
if MONGO_SUPPORT:
    _db_schemes["mongodb"] = ArtifactMongoDB


def _getDBType(uri: str) -> Type[ArtifactDB]:
    """Internal function to take a URI and return a class that can be
    constructed with that URI. For instance "mongodb://localhost" will return
    an ArtifactMongoDB. More types will be added in the future.

    Supported types:
        **ArtifactMongoDB**: mongodb://...
            See http://dochub.mongodb.org/core/connections for details.
        **ArtifactFileDB**: file://...
            A simple flat file database with optional storage for the binary
            artifacts. The filepath is where the json file is stored and the
            data storage can be specified with GEM5ART_STORAGE
    """
    result = urlparse(uri)
    if result.scheme in _db_schemes:
        return _db_schemes[result.scheme]
    else:
        raise Exception(f"Cannot find DB type for {uri}")


def getDBConnection(uri: str = "") -> ArtifactDB:
    """Returns the database connection

    uri: a string representing the URI of the database. See _getDBType for
        details. If no URI is given we use the default
        (mongodb://localhost:27017) or the value in the GEM5ART_DB environment
        variable.

    If the connection has not been established, this will create a new
    connection. If the connection has been established, this will replace the
    connection if the uri input is non-empy.
    """
    global _db

    # mypy bug: https://github.com/python/mypy/issues/5423
    if _db is not None and not uri:  # type: ignore[unreachable]
        # If we have already established a connection, use that
        return _db  # type: ignore[unreachable]

    if not uri:
        uri = os.environ.get("GEM5ART_DB", _default_uri)

    typ = _getDBType(uri)
    _db = typ(uri)

    return _db

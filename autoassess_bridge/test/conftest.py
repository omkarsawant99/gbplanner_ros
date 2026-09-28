# SPDX-License-Identifier: BSD-3-Clause
"""Test setup: import the package from src/ and provide a read-only fake CDF client."""

from __future__ import annotations

import os
import sys
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))

from cognite.client.data_classes.data_modeling.instances import Properties  # noqa: E402

SPACE = "autoassess"


class FakeNode:
    """The subset of a cognite-sdk Node the bridge reads."""

    instance_type = "node"

    def __init__(
        self,
        external_id: str,
        view: Tuple[str, str],
        props: Dict[str, Any],
        created_time: int = 0,
        last_updated_time: int = 0,
        space: str = SPACE,
    ) -> None:
        self.space = space
        self.external_id = external_id
        self.created_time = created_time
        self.last_updated_time = last_updated_time
        self.properties = Properties.load({space: {"{}/{}".format(*view): props}})


class FakeInstances:
    """Exposes only list() and retrieve(): any write API would raise AttributeError.

    list() returns the nodes registered for the requested view (tests assert the filter);
    retrieve() returns the registered nodes whose (space, externalId) was requested.
    """

    def __init__(self) -> None:
        self.by_view: Dict[str, List[FakeNode]] = {}
        self.retrievable: Dict[Tuple[str, str], FakeNode] = {}
        self.calls: List[Tuple[str, Dict[str, Any]]] = []
        self.fail_with: Optional[Exception] = None

    def add(self, view: Tuple[str, str], node: FakeNode, listable: bool = True) -> FakeNode:
        if listable:
            self.by_view.setdefault(view[0], []).append(node)
        self.retrievable[(node.space, node.external_id)] = node
        return node

    def list(self, **kwargs: Any) -> List[FakeNode]:
        self.calls.append(("list", kwargs))
        if self.fail_with is not None:
            raise self.fail_with
        return list(self.by_view.get(kwargs["sources"][0].external_id, []))

    def retrieve(self, **kwargs: Any) -> SimpleNamespace:
        self.calls.append(("retrieve", kwargs))
        if self.fail_with is not None:
            raise self.fail_with
        found = [self.retrievable[tuple(n)] for n in kwargs["nodes"] if tuple(n) in self.retrievable]
        return SimpleNamespace(nodes=found)

    def calls_to(self, method: str) -> List[Dict[str, Any]]:
        return [kwargs for name, kwargs in self.calls if name == method]


class FakeClient:
    def __init__(self) -> None:
        self.instances = FakeInstances()
        self.data_modeling = SimpleNamespace(instances=self.instances)


class RecordingLog:
    def __init__(self) -> None:
        self.records: List[Tuple[str, str]] = []

    def info(self, msg: str) -> None:
        self.records.append(("info", msg))

    def warning(self, msg: str) -> None:
        self.records.append(("warning", msg))

    def error(self, msg: str) -> None:
        self.records.append(("error", msg))

    def messages(self, level: str) -> List[str]:
        return [m for lvl, m in self.records if lvl == level]


class FakeFiles:
    """files.retrieve / files.upload_content over an in-memory store keyed by instance id.

    A CogniteFile node applied through FakeWriteInstances becomes retrievable with
    uploaded=False; upload_content marks it uploaded. Paths in `fail_paths` raise.
    """

    def __init__(self) -> None:
        self.store: Dict[Tuple[str, str], SimpleNamespace] = {}
        self.uploads: List[Tuple[str, Tuple[str, str]]] = []
        self.fail_paths: set = set()
        self._next_id = 1000

    def register(self, space: str, external_id: str, uploaded: bool) -> SimpleNamespace:
        self._next_id += 1
        meta = SimpleNamespace(id=self._next_id, uploaded=uploaded)
        self.store[(space, external_id)] = meta
        return meta

    def retrieve(self, id: Any = None, external_id: Any = None, instance_id: Any = None) -> Any:
        return self.store.get((instance_id.space, instance_id.external_id))

    def upload_content(self, path: str, external_id: Any = None, instance_id: Any = None) -> Any:
        key = (instance_id.space, instance_id.external_id)
        if path in self.fail_paths:
            raise RuntimeError("upload of {} failed".format(path))
        if key not in self.store:
            raise RuntimeError("no CogniteFile node {}".format(key))
        self.uploads.append((path, key))
        meta = self.store[key]
        meta.uploaded = True
        return SimpleNamespace(id=meta.id)


class FakeWriteInstances(FakeInstances):
    """FakeInstances plus apply(); applied CogniteFile nodes are registered in FakeFiles."""

    def __init__(self, files: FakeFiles) -> None:
        super().__init__()
        self._files = files
        self.applied: List[Dict[str, Any]] = []

    def apply(self, nodes: Any = None, **kwargs: Any) -> None:
        self.calls.append(("apply", dict(kwargs, nodes=nodes)))
        for node in nodes:
            dumped = node.dump()
            self.applied.append(dumped)
            source = dumped["sources"][0]["source"]
            if source["externalId"] == "CogniteFile":
                self._files.register(dumped["space"], dumped["externalId"], uploaded=False)


class FakeWriteClient:
    def __init__(self) -> None:
        self.files = FakeFiles()
        self.instances = FakeWriteInstances(self.files)
        self.data_modeling = SimpleNamespace(instances=self.instances)

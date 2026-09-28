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

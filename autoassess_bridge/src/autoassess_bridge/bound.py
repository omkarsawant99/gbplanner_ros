# SPDX-License-Identifier: BSD-3-Clause
"""Send the area box to the planner's global-bound service, retrying while it is not up yet."""

from __future__ import annotations

from typing import Any, Callable, Optional, Tuple

from autoassess_bridge.plan import Bounds


class ServiceUnavailable(Exception):
    """The service is not advertised (yet); the same bounds are retried on the next tick."""


class GlobalBoundSetter:
    """`call(bounds) -> (success, detail)` sends the bounds; it raises ServiceUnavailable when
    the service is not there. A rejection (success=False) or any other error is logged and not
    retried: resending the same box would be rejected again. `log` needs info() and warning().
    """

    def __init__(self, call: Callable[[Bounds], Tuple[bool, str]], log: Any) -> None:
        self._call = call
        self._log = log
        self._pending: Optional[Bounds] = None
        self._warned_unavailable = False

    @property
    def pending(self) -> bool:
        return self._pending is not None

    def set_pending(self, bounds: Bounds) -> None:
        self._pending = bounds
        self._warned_unavailable = False

    def tick(self) -> None:
        if self._pending is None:
            return
        bounds = self._pending
        try:
            success, detail = self._call(bounds)
        except ServiceUnavailable as exc:
            if not self._warned_unavailable:
                self._log.warning("Global bound service not available, will retry: {}".format(exc))
                self._warned_unavailable = True
            return
        except Exception as exc:  # noqa: BLE001 - report and give up on this box
            self._pending = None
            self._log.warning("Global bound call failed: {}: {}".format(type(exc).__name__, exc))
            return
        self._pending = None
        if success:
            self._log.info("Global bound set: {}".format(detail))
        else:
            self._log.warning(
                "Global bound rejected by the planner (robot outside the box?); "
                "planner bound: {}".format(detail)
            )

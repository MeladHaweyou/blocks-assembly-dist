"""Threaded background execution helper for Streamlit callbacks."""
from __future__ import annotations

from dataclasses import dataclass, field
from queue import Queue, Empty
from threading import Event, Lock, Thread
from typing import Any, Callable, Optional, Tuple


RunPayload = Tuple[str, int, Any, bool]


@dataclass
class BackgroundRunner:
    """Manage a single background worker thread.

    The worker reports completion or errors via an internal queue so the
    Streamlit thread can poll without blocking.
    """

    _thread: Optional[Thread] = None
    _stop_event: Event = field(default_factory=Event)
    _lock: Lock = field(default_factory=Lock)
    _queue: Queue[RunPayload] = field(default_factory=Queue)
    _token: int = 0

    def is_running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def request_stop(self) -> None:
        self._stop_event.set()

    def start(
        self,
        target: Callable[..., Any],
        *,
        args: Tuple[Any, ...] = (),
        kwargs: Optional[dict[str, Any]] = None,
    ) -> int:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("BackgroundRunner is already executing a task")
            self._stop_event = Event()
            self._token += 1
            token = self._token
            queue = self._queue

            def worker() -> None:
                try:
                    result = target(*args, stop_event=self._stop_event, **(kwargs or {}))
                except Exception as exc:  # pragma: no cover - defensive
                    queue.put(("error", token, f"{type(exc).__name__}: {exc}", False))
                else:
                    queue.put(("result", token, result, self._stop_event.is_set()))

            thread = Thread(target=worker, daemon=True)
            self._thread = thread
            thread.start()
            return token

    def poll(self) -> Optional[RunPayload]:
        try:
            return self._queue.get_nowait()
        except Empty:
            return None


__all__ = ["BackgroundRunner", "RunPayload"]

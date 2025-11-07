"""Thread controller for managing background simulation runs."""
from __future__ import annotations

from dataclasses import dataclass, field
from queue import Empty, Queue
import threading
from typing import Any, Callable, Optional, Tuple
import copy


RunPayload = Tuple[str, int, Any]


@dataclass
class RunController:
    """Own a worker thread and cooperative stop event for Streamlit UIs."""

    thread: Optional[threading.Thread] = None
    stop_evt: Optional[threading.Event] = None
    _queue: Queue[RunPayload] = field(default_factory=Queue)
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _token: int = 0

    def is_running(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def _drain_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except Empty:
                break

    def start(
        self,
        target: Callable[..., Any],
        *,
        args: Tuple[Any, ...] = (),
        kwargs: Optional[dict[str, Any]] = None,
    ) -> Optional[int]:
        with self._lock:
            if self.is_running():
                return None

            self._token += 1
            token = self._token

            # Fresh stop event per run and clear any stale queue payloads.
            stop_evt = threading.Event()
            self.stop_evt = stop_evt
            self._drain_queue()

            immutable_args = tuple(copy.deepcopy(arg) for arg in args)
            immutable_kwargs = copy.deepcopy(kwargs) if kwargs else {}
            queue = self._queue

            def worker() -> None:
                try:
                    result = target(*immutable_args, stop_evt=stop_evt, **immutable_kwargs)
                except Exception as exc:  # pragma: no cover - defensive
                    queue.put(("error", token, exc))
                else:
                    queue.put(("result", token, result))
                finally:
                    queue.put(("finished", token, stop_evt.is_set()))
                    with self._lock:
                        self.thread = None
                        self.stop_evt = None

            thread = threading.Thread(target=worker, daemon=True)
            self.thread = thread
            thread.start()
            return token

    def stop(self) -> None:
        if self.stop_evt is not None:
            self.stop_evt.set()

    def poll(self) -> Optional[RunPayload]:
        try:
            return self._queue.get_nowait()
        except Empty:
            return None


__all__ = ["RunController", "RunPayload"]


import itertools
import time
from collections import deque
from dataclasses import dataclass, field
from threading import Lock
from typing import Callable, Deque, Dict, Iterable, List, Optional

PRIORITY_LEVELS = ("high", "medium", "low")
DEFAULT_PRIORITY = "medium"

STATUS_QUEUED = "queued"
STATUS_RUNNING = "running"
STATUS_DONE = "done"
STATUS_FAILED = "failed"


@dataclass
class TaskItem:
    task_id: int
    origin: str
    destination: str
    priority: str
    status: str = STATUS_QUEUED
    created_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    finished_at: Optional[float] = None

    def to_dict(self) -> Dict[str, object]:
        return {
            "id": self.task_id,
            "origin": self.origin,
            "destination": self.destination,
            "priority": self.priority,
            "status": self.status,
            "created_at": round(self.created_at, 3),
            "started_at": None if self.started_at is None else round(self.started_at, 3),
            "finished_at": None if self.finished_at is None else round(self.finished_at, 3),
        }


class TaskQueue:
    def __init__(
        self,
        initial_tasks: Optional[Iterable[object]] = None,
        on_change: Optional[Callable[[Dict[str, object]], None]] = None,
        history_limit: int = 80,
    ) -> None:
        self._queues: Dict[str, Deque[TaskItem]] = {
            "high": deque(),
            "medium": deque(),
            "low": deque(),
        }
        self._tasks: Dict[int, TaskItem] = {}
        self._history: Deque[TaskItem] = deque(maxlen=history_limit)
        self._lock = Lock()
        self._counter = itertools.count(1)
        self._running: Optional[TaskItem] = None
        self._on_change = on_change

        if initial_tasks:
            self.add_tasks(initial_tasks, emit=False)
            self._emit_snapshot()

    def set_event_emitter(self, on_change: Optional[Callable[[Dict[str, object]], None]]) -> None:
        self._on_change = on_change

    def add_task(
        self,
        origin: str,
        destination: str,
        priority: str = DEFAULT_PRIORITY,
        emit: bool = True,
    ) -> TaskItem:
        if priority not in PRIORITY_LEVELS:
            raise ValueError(f"Prioridad invalida: {priority}")

        with self._lock:
            task_id = next(self._counter)
            task = TaskItem(
                task_id=task_id,
                origin=origin,
                destination=destination,
                priority=priority,
            )
            self._queues[priority].append(task)
            self._tasks[task_id] = task

        if emit:
            self._emit_snapshot()

        return task

    def add_tasks(self, tasks: Iterable[object], emit: bool = True) -> List[TaskItem]:
        created: List[TaskItem] = []
        for item in tasks:
            if isinstance(item, (list, tuple)) and len(item) == 3:
                origin, destination, priority = item
            elif isinstance(item, (list, tuple)) and len(item) == 2:
                origin, destination = item
                priority = DEFAULT_PRIORITY
            else:
                raise ValueError("Formato de tarea invalido")
            created.append(self.add_task(origin, destination, priority, emit=False))

        if emit and created:
            self._emit_snapshot()

        return created

    def peek_next(self) -> Optional[TaskItem]:
        with self._lock:
            for priority in PRIORITY_LEVELS:
                if self._queues[priority]:
                    return self._queues[priority][0]
        return None

    def pop_next(self) -> Optional[TaskItem]:
        task: Optional[TaskItem] = None
        with self._lock:
            if self._running is not None:
                return None
            for priority in PRIORITY_LEVELS:
                if self._queues[priority]:
                    task = self._queues[priority].popleft()
                    task.status = STATUS_RUNNING
                    task.started_at = time.time()
                    self._running = task
                    break

        if task is not None:
            self._emit_snapshot()

        return task

    def complete(self, task_id: int, success: bool = True) -> bool:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None or self._running is None or task.task_id != self._running.task_id:
                return False

            task.status = STATUS_DONE if success else STATUS_FAILED
            task.finished_at = time.time()
            self._running = None
            self._history.appendleft(task)

        self._emit_snapshot()
        return True

    def cancel(self, task_id: int) -> bool:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None or task.status != STATUS_QUEUED:
                return False

            removed = False
            for queue in self._queues.values():
                for item in list(queue):
                    if item.task_id == task_id:
                        queue.remove(item)
                        removed = True
                        break
                if removed:
                    break

            if not removed:
                return False

            self._tasks.pop(task_id, None)

        self._emit_snapshot()
        return True

    def get_snapshot(self) -> Dict[str, object]:
        with self._lock:
            queued = {
                priority: [task.to_dict() for task in list(self._queues[priority])]
                for priority in PRIORITY_LEVELS
            }
            running = None if self._running is None else self._running.to_dict()
            history = [task.to_dict() for task in list(self._history)]

        return {
            "queued": queued,
            "running": running,
            "history": history,
        }

    def is_empty(self) -> bool:
        with self._lock:
            return self._running is None and all(not q for q in self._queues.values())

    def _emit_snapshot(self) -> None:
        if self._on_change is None:
            return
        payload = self.get_snapshot()
        self._on_change({"type": "queue_updated", "payload": payload})

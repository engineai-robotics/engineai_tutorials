"""Task-local state. Wall-clock timeouts never overwrite action-server results."""
from dataclasses import dataclass, field
from typing import Any
import uuid

TERMINAL = frozenset(('succeeded', 'canceled', 'aborted', 'rejected'))

@dataclass
class NavigationTask:
    uuid: str = field(default_factory=lambda: uuid.uuid4().hex)
    state: str = 'waiting_for_acceptance'
    feedback: Any = None
    result: Any = None
    error: str = ''
    cancel_requested: bool = False
    cancel_response: Any = None
    goal_handle: Any = None
    send_future: Any = None
    result_future: Any = None
    cancel_future: Any = None

    @property
    def done(self):
        return self.state in TERMINAL

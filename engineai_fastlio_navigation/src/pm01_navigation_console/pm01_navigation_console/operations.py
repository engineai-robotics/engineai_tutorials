"""UI requests are bounded; output release never queues behind a long operation."""
from concurrent.futures import ThreadPoolExecutor
import queue
import threading


class Operations:
    def __init__(self):
        self.normal = ThreadPoolExecutor(max_workers=1, thread_name_prefix='pm01-operation')
        self.priority = ThreadPoolExecutor(max_workers=1, thread_name_prefix='pm01-release')
        self.results = queue.Queue()
        self.lock = threading.Lock()
        self.pending = set()

    def submit(self, name, function):
        with self.lock:
            if name in self.pending or (name != 'release' and self.pending - {'release'}):
                return False
            self.pending.add(name)
        future = (self.priority if name == 'release' else self.normal).submit(function)
        def done(result):
            try:
                value = result.result()
                self.results.put((name, True, value))
            except Exception as error:
                self.results.put((name, False, str(error)))
            finally:
                with self.lock:
                    self.pending.discard(name)
        future.add_done_callback(done)
        return True

    def close(self):
        self.normal.shutdown(wait=False, cancel_futures=True)
        self.priority.shutdown(wait=False, cancel_futures=True)

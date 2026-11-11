"""Own only a directly spawned process group; never discover/kill by name."""
import os
import signal
import subprocess
import time

class OutputProcess:
    def __init__(self):
        self.process = None

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, argv, log=None):
        if self.running:
            raise RuntimeError('Output process already running')
        self.process = subprocess.Popen(argv, start_new_session=True,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        return self.process.pid

    def stop(self, budget=2.):
        """SIGINT -> TERM -> KILL. Return exit confirmation within budget."""
        p = self.process
        if p is None or p.poll() is not None:
            return True
        deadline = time.monotonic()+budget
        for sig, fraction in ((signal.SIGINT,.25),(signal.SIGTERM,.25),(signal.SIGKILL,.5)):
            if p.poll() is not None:
                break
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                pass
            try:
                p.wait(timeout=max(0., min(budget*fraction, deadline-time.monotonic())))
            except subprocess.TimeoutExpired:
                pass
        return p.poll() is not None

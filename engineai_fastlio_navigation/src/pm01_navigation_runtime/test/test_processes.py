import sys
import time
from pm01_navigation_runtime.system.processes import OutputProcess

def test_stubborn_owned_process_killed_within_budget():
    worker=OutputProcess()
    worker.start([sys.executable,'-c','import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(30)'])
    time.sleep(.15)
    start=time.monotonic()
    assert worker.stop(2.)
    assert time.monotonic()-start<2.1
    assert not worker.running
    assert worker.stop(2.)

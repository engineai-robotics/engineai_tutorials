"""Read-only SSH clock comparison; no ROS participant or clock mutation."""
import argparse
import json
import os
from pathlib import Path
import re
import selectors
import shlex
import subprocess
import tempfile
import time
from .clock_sampling import estimate, SampleError


def check(host, expected_machine_id='', samples=8):
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*', host):
        raise ValueError('Use a configured SSH host alias')
    if not isinstance(samples, int) or not 1 <= samples <= 100:
        raise ValueError('samples must be between 1 and 100')
    source = Path(__file__).with_name('clock_sampling.py').read_text()
    command = 'python3 -u -c '+shlex.quote(source+'\nserve('+repr(expected_machine_id)+')\n')
    observations = []; pending = bytearray()
    # Authenticate and start Python once. Neither cost belongs in a clock
    # sample; only exchanges after the ready handshake are timed.
    with tempfile.TemporaryFile() as errors:
        process = subprocess.Popen(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                                    '-o', 'ConnectTimeout=4', host, command],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=errors, bufsize=0)
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        def read_line(deadline):
            while b'\n' not in pending:
                remaining = deadline-time.monotonic()
                if remaining <= 0 or not selector.select(remaining):
                    raise TimeoutError('Clock sample timed out')
                data = os.read(process.stdout.fileno(), 4096)
                if not data:
                    errors.seek(0)
                    raise RuntimeError('Clock channel ended: '+errors.read().decode(errors='replace')[-1000:])
                pending.extend(data)
                if len(pending) > 16384:
                    raise RuntimeError('Invalid clock channel response')
            line, _, rest = pending.partition(b'\n')
            pending[:] = rest
            return line
        try:
            ready = json.loads(read_line(time.monotonic()+6))
            if not ready.get('ready') or (expected_machine_id and ready.get('machine_id') != expected_machine_id):
                raise RuntimeError('Robot identity mismatch')
            for sequence in range(samples):
                payload = (json.dumps(dict(sequence=sequence))+'\n').encode()
                sent, start = time.time(), time.monotonic()
                process.stdin.write(payload); process.stdin.flush()
                line = read_line(start+2)
                received, elapsed = time.time(), time.monotonic()-start
                remote = json.loads(line)
                if remote.get('sequence') != sequence:
                    raise RuntimeError('Clock sequence mismatch')
                try:
                    observations.append(estimate(sent, received, remote, elapsed))
                except SampleError:
                    continue
                if sequence+1 < samples:
                    time.sleep(.05)
        finally:
            selector.close()
            process.stdin.close()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=1)
            process.stdout.close()
    if not observations:
        raise RuntimeError('No valid clock samples; clocks may have changed during measurement')
    best = min(observations, key=lambda value: value['round_trip_sec'])
    return dict(robot_minus_developer_sec=best['offset_sec'], round_trip_sec=best['round_trip_sec'],
                uncertainty_sec=best['uncertainty_sec'], error_bound_sec=abs(best['offset_sec'])+best['uncertainty_sec'],
                samples=len(observations), method='four_timestamp_persistent_ssh')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='Developer deployment.yaml')
    parser.add_argument('--ssh-host')
    parser.add_argument('--machine-id', default='')
    parser.add_argument('--max-skew', type=float, default=.2)
    args = parser.parse_args()
    from pm01_navigation_runtime.deployment.configuration import clock_connection
    try:
        host, identity = clock_connection(args.config, args.ssh_host, args.machine_id)
    except ValueError as error:
        parser.error(str(error))
    value = check(host, identity)
    print(json.dumps(value, indent=2))
    return 0 if abs(value['robot_minus_developer_sec'])+value.get('uncertainty_sec', 0) <= args.max_skew else 2


if __name__ == '__main__':
    raise SystemExit(main())

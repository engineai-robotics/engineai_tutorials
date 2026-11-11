"""Bounded clock sampling shared by SSH checks and pre-start calibration.

Uses the four-timestamp offset/delay calculation from RFC 5905. This is a
temporary application-level measurement, not an NTP daemon or frequency servo.
The source is sent through verified SSH so the robot needs only Python's stdlib.
"""
import json
import math
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request


class SampleError(ValueError):
    pass


def estimate(sent, received, remote, elapsed):
    t2, t3 = remote['received'], remote['sent']
    processing = remote['elapsed']
    values = (sent, received, t2, t3, processing, elapsed)
    if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
        raise SampleError('Non-finite clock sample')
    if min(sent, received, t2, t3) < 1600000000 or min(processing, elapsed) < 0:
        raise SampleError('Invalid clock sample')
    # Reject samples spanning a wall-clock step on either end. Deadlines use
    # monotonic time, so correcting CLOCK_REALTIME cannot extend the operation.
    if abs((received-sent)-elapsed) > .01 or abs((t3-t2)-processing) > .01:
        raise SampleError('Clock changed during measurement; sample discarded')
    delay = elapsed-processing
    if delay < -.002:
        raise SampleError('Inconsistent clock sample')
    delay = max(0., delay)
    return dict(offset_sec=((t2-sent)+(t3-received))/2,
                round_trip_sec=delay, uncertainty_sec=delay/2+.001)


def serve(expected):
    identity = Path('/etc/machine-id').read_text().strip()
    if expected and identity != expected:
        raise RuntimeError('Robot identity mismatch')
    print(json.dumps(dict(ready=True, machine_id=identity)), flush=True)
    for line in sys.stdin:
        received, started = time.time(), time.monotonic()
        request = json.loads(line)
        sent = time.time()
        print(json.dumps(dict(sequence=request['sequence'], received=received,
                              sent=sent, elapsed=time.monotonic()-started)), flush=True)


def converge(sample, adjust, *, timeout=60., tolerance=.05, stable_required=3,
             round_samples=5, monotonic=time.monotonic, sleep=time.sleep, progress=None):
    """Use fresh low-delay windows; verify after each correction, never before.

    sample returns peer-minus-local offset. An uncertain sample cannot satisfy
    convergence or trigger a correction. The uncertainty bound assumes
    non-negative network delays; it is not a guarantee of UTC accuracy.
    """
    if not (math.isfinite(timeout) and timeout > 0 and math.isfinite(tolerance) and tolerance > 0
            and stable_required >= 1 and round_samples >= 3):
        raise ValueError('Invalid clock convergence limits')
    started = monotonic(); deadline = started+timeout
    stable = steps = count = rounds = 0
    best = None; last_error = None
    while monotonic() < deadline:
        observations = []
        for _ in range(round_samples):
            remaining = deadline-monotonic()
            if remaining <= 0:
                break
            try:
                observations.append(sample(min(2., remaining)))
                count += 1
            except (SampleError, OSError, TimeoutError, urllib.error.URLError) as error:
                last_error = str(error)
            sleep(min(.05, max(0., deadline-monotonic())))
        rounds += 1
        if len(observations) < 3 or monotonic() >= deadline:
            stable = 0
        else:
            best = min(observations, key=lambda value: value['round_trip_sec'])
            offset, uncertainty = best['offset_sec'], best['uncertainty_sec']
            if abs(offset)+uncertainty <= tolerance:
                stable += 1
            else:
                stable = 0
                # Do not chase congestion. A sufficiently precise observation
                # is required before changing the robot's clock.
                if uncertainty <= tolerance/2 and abs(offset) > tolerance/2:
                    adjust(offset)
                    steps += 1
        result = dict(aligned=stable >= stable_required, stable_rounds=stable,
                      required_stable_rounds=stable_required, samples=count, rounds=rounds,
                      corrections=steps, elapsed_sec=monotonic()-started,
                      tolerance_sec=tolerance, method='four_timestamp_ssh',
                      ntp_configuration_changed=False)
        if best is not None:
            result.update(robot_minus_developer_sec=-best['offset_sec'],
                          round_trip_sec=best['round_trip_sec'], uncertainty_sec=best['uncertainty_sec'],
                          error_bound_sec=abs(best['offset_sec'])+best['uncertainty_sec'])
        if progress:
            progress(result)
        if result['aligned']:
            return result
        sleep(min(.2, max(0., deadline-monotonic())))
    result.update(aligned=False, error='Clock synchronisation timed out before convergence')
    if last_error:
        result['last_sample_error'] = last_error
    return result


def synchronise_remote(url, expected, timeout, tolerance):
    """Run only after sudo authentication, without installing a background job."""
    result = None
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        if Path('/etc/machine-id').read_text().strip() != expected:
            raise RuntimeError('Robot identity mismatch; time unchanged')
        def sample(remaining):
            sent, started = time.time(), time.monotonic()
            with opener.open(url, timeout=remaining) as response:
                body = response.read(4096)
            received, elapsed = time.time(), time.monotonic()-started
            try:
                remote = json.loads(body)
                return estimate(sent, received, remote, elapsed)
            except (ValueError, KeyError, TypeError) as error:
                raise SampleError('Invalid timestamp response') from error
        def adjust(offset):
            time.clock_settime(time.CLOCK_REALTIME, time.time()+offset)
        def progress(value):
            print(json.dumps(dict(clock_progress=value)), flush=True)
        result = converge(sample, adjust, timeout=timeout,
                          tolerance=tolerance, progress=progress)
    except Exception as error:
        result = dict(aligned=False, error=str(error), ntp_configuration_changed=False)
    result['machine_id'] = expected
    # Return the verified result to the calling wizard as well as its terminal.
    request = urllib.request.Request(url+'/result', json.dumps(result).encode(),
                                    {'Content-Type': 'application/json'}, method='POST')
    with opener.open(request, timeout=3) as response:
        response.read()
    print(json.dumps(result), flush=True)
    return 0 if result['aligned'] else 1

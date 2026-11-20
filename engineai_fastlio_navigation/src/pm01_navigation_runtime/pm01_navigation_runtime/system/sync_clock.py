#!/usr/bin/env python3
"""Explicitly confirmed LAN clock alignment without managing robot services.

Uses four-timestamp sampling through temporary verified SSH forwarding. Returns
only after consecutive verified windows converge, or the operation times out.
No internet, saved password, NTP configuration change or persistent service.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import sys
import re
import secrets
import shlex
import subprocess
import threading
import time

from pm01_navigation_runtime.deployment.configuration import clock_connection


def synchronise(host, identity, *, timeout=60., tolerance=.05):
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*', host):
        raise ValueError('Use a configured SSH alias')
    if not re.fullmatch(r'[0-9a-f]{32}', identity):
        raise ValueError('Supply the verified machine ID')
    if not math.isfinite(timeout) or not 1 <= timeout <= 300:
        raise ValueError('timeout must be between 1 and 300 seconds')
    if not math.isfinite(tolerance) or not .005 <= tolerance <= .2:
        raise ValueError('tolerance must be between 0.005 and 0.2 seconds')
    nonce = secrets.token_hex(20)
    results = []
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            received, started = time.time(), time.monotonic()
            if self.path != '/'+nonce:
                self.send_error(404); return
            body = json.dumps(dict(received=received, sent=time.time(),
                                   elapsed=time.monotonic()-started)).encode()
            self.send_response(200); self.send_header('Content-Length', str(len(body)))
            self.end_headers(); self.wfile.write(body)

        def do_POST(self):
            if self.path != '/'+nonce+'/result':
                self.send_error(404); return
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 16384:
                self.send_error(400); return
            value = json.loads(self.rfile.read(length))
            if value.get('machine_id') != identity:
                self.send_error(400); return
            results.append(value)
            self.send_response(200); self.send_header('Content-Length', '0'); self.end_headers()

        def log_message(self, *unused):
            pass
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    remote_port = 40000+secrets.randbelow(15000)
    source = Path(__file__).with_name('clock_sampling.py')
    url = 'http://127.0.0.1:'+str(remote_port)+'/'+nonce
    program = source.read_text()+'\nraise SystemExit(synchronise_remote('+repr(url)+','+repr(identity)+','+repr(timeout)+','+repr(tolerance)+'))\n'
    try:
        # Sampling starts after sudo authentication. Allow separate, bounded
        # time for entering the credential; never save or pipe the password.
        command = 'sudo python3 -u -c '+shlex.quote(program)
        outcome = subprocess.run(['ssh', '-tt', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ExitOnForwardFailure=yes', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=3',
            '-o', 'ServerAliveCountMax=2', '-R', '127.0.0.1:'+str(remote_port)+':127.0.0.1:'+str(server.server_port),
            host, command], timeout=timeout+120)
        if not results:
            raise RuntimeError('未收到校时最终结果；请检查 SSH／sudo 输出，不能确认校时成功')
        result = results[-1]
        if outcome.returncode or not result.get('aligned'):
            raise RuntimeError('校时未收敛：'+json.dumps(result, ensure_ascii=False))
        if (result.get('stable_rounds', 0) < 3 or result.get('error_bound_sec', math.inf) > tolerance
                or result.get('method') != 'four_timestamp_ssh'):
            raise RuntimeError('校时验证结果不完整，不能确认成功')
        return result
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='Developer deployment.yaml')
    parser.add_argument('--ssh-host')
    parser.add_argument('--machine-id')
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--timeout', type=float, default=60., help='Maximum sampling time after sudo authentication')
    parser.add_argument('--tolerance', type=float, default=.05, help='Maximum offset plus uncertainty, in seconds')
    args = parser.parse_args()
    try:
        host, identity = clock_connection(args.config, args.ssh_host, args.machine_id)
        if not args.apply:
            parser.error('Supply explicit --apply to confirm adjusting the robot system time')
        print('将以开发机时间校准机器人系统时间，可能发生时间跳变；现有服务不会停止或重启。', file=sys.stderr)
        result = synchronise(host, identity, timeout=args.timeout, tolerance=args.tolerance)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        print(json.dumps(dict(aligned=False, error=str(error)), ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())

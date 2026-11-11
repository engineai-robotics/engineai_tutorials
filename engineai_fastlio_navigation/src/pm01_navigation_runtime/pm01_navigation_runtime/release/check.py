#!/usr/bin/env python3
"""Check a source tree for workstation-only artifacts before public release."""
import argparse
import ipaddress
import os
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit

SKIP = {'.git', '__pycache__', '.pytest_cache', '.DS_Store'}
ROOT_SKIP = {'.deps', 'build', 'install', 'log'}
PERSONAL = re.compile(r'/(?:Users|home)/[A-Za-z0-9_.-]+(?:/|\b)')
INTERNAL_HOST = re.compile('code' + r'\.engineai\.com\.cn', re.I)
IPV4 = re.compile(r'(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])')


def documentation_targets(content):
    """Local Markdown links and HTML assets, including upstream documentation."""
    targets = re.findall(r'\]\((<[^>]+>|[^\s)]+)(?:\s+[^)]*)?\)', content)
    targets += re.findall(r"(?:src|href)=[\"']([^\"']+)[\"']", content, re.I)
    targets += re.findall(r'^\s{0,3}\[[^]]+\]:\s*(<[^>]+>|\S+)', content, re.M)
    return targets


def inspect(root):
    errors = []
    count = 0
    paths = []
    for directory, folders, files in os.walk(root, followlinks=False):
        folders[:] = [name for name in folders if name not in SKIP and
                      not (Path(directory) == root and name in ROOT_SKIP)]
        paths.extend(Path(directory) / name for name in folders if (Path(directory) / name).is_symlink())
        paths.extend(Path(directory) / name for name in files if name not in SKIP)
    for path in paths:
        relative = path.relative_to(root)
        if path.is_symlink():
            if not path.resolve().is_relative_to(root.resolve()) or not path.exists():
                errors.append(f'{relative}: broken or external symlink')
            continue
        if not path.is_file():
            continue
        count += 1
        if relative.parts[0] == 'reports' or '.vscode' in relative.parts:
            errors.append(f'{relative}: internal archive/editor artifact')
        if path.suffix in {'.pcd', '.bag', '.db3', '.mcap', '.pyc'}:
            errors.append(f'{relative}: generated sensor/cache artifact')
        try:
            content = path.read_text()
        except UnicodeError:
            continue
        if path.suffix.lower() == '.md':
            for target in documentation_targets(content):
                parsed = urlsplit(target.strip('<>'))
                if parsed.scheme or parsed.netloc or not parsed.path:
                    continue
                decoded = unquote(parsed.path)
                linked = (root / decoded.lstrip('/')) if decoded.startswith('/') else path.parent / decoded
                if not linked.exists():
                    errors.append(f'{relative}: broken documentation link: {target}')
        for number, line in enumerate(content.splitlines(), 1):
            # Public sources must not contain workstation or internal host links.
            checked = line
            checked = re.sub(r'/(?:home)/ubuntu/\.config/pm01_navigation(?=/|["\s]|$)',
                             'robot-config', checked)
            if PERSONAL.search(checked) or INTERNAL_HOST.search(checked):
                errors.append(f'{relative}:{number}: personal path or internal repository host')
            for candidate in IPV4.findall(line):
                try:
                    address = ipaddress.IPv4Address(candidate)
                except ipaddress.AddressValueError:
                    continue
                # PM01 and sensor LAN examples are intentionally retained.
                if address.packed[0] == 10 or (address.packed[0] == 172 and 16 <= address.packed[1] <= 31):
                    errors.append(f'{relative}:{number}: workstation LAN address')
    return count, errors


def main(workspace):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', nargs='?', type=Path, default=workspace)
    args = parser.parse_args()
    count, errors = inspect(args.root.resolve())
    for error in errors:
        print(error)
    print(f'Checked {count} source files; {len(errors)} release findings.')
    raise SystemExit(bool(errors))



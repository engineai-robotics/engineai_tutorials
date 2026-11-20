#!/usr/bin/env python3
"""Export a checked source snapshot with vendored sources, without Git history or downloaded assets."""
import argparse
from pathlib import Path
import subprocess
import tarfile
import tempfile
from pm01_navigation_runtime.release.check import inspect


def export_snapshot(root, output):
    root, output = root.resolve(), output.expanduser().resolve()
    if output.exists() or output.is_relative_to(root):
        raise ValueError('output must be a new directory outside this workspace')
    dirty = subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'], text=True)
    if dirty:
        raise ValueError('commit the reviewed source changes before exporting')
    count, errors = inspect(root)
    if errors:
        raise ValueError('\n'.join(errors))
    revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    with tempfile.TemporaryDirectory(prefix='navigation-release-') as directory:
        archive = Path(directory) / 'source.tar'
        subprocess.run(['git', '-C', str(root), 'archive', '--format=tar', '-o', str(archive), 'HEAD'], check=True)
        output.mkdir(parents=True)
        with tarfile.open(archive) as stream:
            # git archive cannot contain .git or parent-directory paths. Symlink
            # destinations were checked against the source tree before export.
            stream.extractall(output)
    (output / 'SOURCE_VERSION').write_text(revision + '\n')
    _, errors = inspect(output)
    if errors:
        raise RuntimeError('\n'.join(errors))
    return revision, count



def main(workspace):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New directory outside the source workspace')
    args = parser.parse_args()
    try:
        revision, count = export_snapshot(workspace, args.output)
    except (ValueError, RuntimeError) as error:
        parser.error(str(error))
    print(f'Exported {count} source files from {revision} to {args.output.resolve()}')
    print('The snapshot contains vendored sources; no .git history, downloaded assets or build output.')



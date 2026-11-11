"""Published documentation includes vendored package instructions."""
from pm01_navigation_runtime.release.check import inspect


def test_vendored_markdown_and_html_links(tmp_path):
    folder = tmp_path/'src'/'upstream'
    folder.mkdir(parents=True)
    (tmp_path/'LICENSE').write_text('license')
    (folder/'asset.svg').write_text('<svg/>')
    (folder/'README.md').write_text(
        '[license](/LICENSE) ![asset](asset.svg) '
        '<img src="missing.png"> [bad](gone.md)\n'
        '[ref]: other.md\n[remote](https://example.com/doc)')
    _, errors = inspect(tmp_path)
    assert len(errors) == 3
    assert all(any(name in error for error in errors) for name in ('missing.png', 'gone.md', 'other.md'))


def test_encoded_paths_and_query_fragments(tmp_path):
    (tmp_path/'a b.svg').write_text('<svg/>')
    (tmp_path/'README.md').write_text(
        '<img src="a%20b.svg?raw=true"> [file](<a b.svg>) [part](#section)')
    assert inspect(tmp_path)[1] == []

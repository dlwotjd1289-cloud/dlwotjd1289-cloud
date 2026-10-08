"""Offline integration tests against real Git repositories, not mocked commands."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts/fetch_hyundai_refs.sh'
REPOS = ('hdr_client_driver', 'hdr_description', 'hdr_ros2_driver', 'hdr_simulation_gz')


def run(*args, cwd, check=True):
    env = dict(os.environ, GIT_ALLOW_PROTOCOL='file',
               GIT_AUTHOR_NAME='Test', GIT_AUTHOR_EMAIL='test@example.invalid',
               GIT_COMMITTER_NAME='Test', GIT_COMMITTER_EMAIL='test@example.invalid')
    return subprocess.run(args, cwd=cwd, env=env, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=check)


def git(cwd, *args):
    return run('git', *args, cwd=cwd).stdout.strip()


@pytest.fixture
def checkout(tmp_path):
    upstream = tmp_path / 'upstream'
    upstream.mkdir()
    git(upstream, 'init', '-b', 'humble')
    nested = tmp_path / 'nested'
    nested.mkdir()
    git(nested, 'init', '-b', 'main')
    (nested / 'asset.txt').write_text('nested pin\n')
    git(nested, 'add', '.')
    git(nested, 'commit', '-m', 'nested fixture')
    git(upstream, 'submodule', 'add', str(nested), 'nested')
    (upstream / 'package.xml').write_text('pinned fixture\n')
    git(upstream, 'add', '.')
    git(upstream, 'commit', '-m', 'pinned')
    pin = git(upstream, 'rev-parse', 'HEAD')
    parent = tmp_path / 'parent'
    parent.mkdir()
    git(parent, 'init', '-b', 'main')
    (parent / 'scripts').mkdir()
    shutil.copy2(SCRIPT, parent / 'scripts' / SCRIPT.name)
    (parent / 'ros2_ws/src').mkdir(parents=True)
    for name in REPOS:
        path = f'external/hyundai_robotics/{name}'
        git(parent, 'submodule', 'add', str(upstream), path)
        (parent / 'ros2_ws/src' / name).symlink_to(f'../../{path}')
    git(parent, 'add', '.')
    git(parent, 'commit', '-m', 'fixture superproject')
    # Advance humble so installing its tip would violate the recorded pins.
    (upstream / 'package.xml').write_text('new upstream tip\n')
    git(upstream, 'add', '.')
    git(upstream, 'commit', '-m', 'advance humble')
    clone = tmp_path / 'clone with spaces'
    git(tmp_path, 'clone', str(parent), str(clone))
    return clone, upstream, pin


def install(clone, check=True):
    return run('bash', str(clone / 'scripts' / SCRIPT.name), cwd=clone.parent, check=check)


def assert_pinned(clone, pin):
    for name in REPOS:
        path = clone / 'external/hyundai_robotics' / name
        assert git(path, 'rev-parse', 'HEAD') == pin
        assert (path / 'nested/asset.txt').read_text() == 'nested pin\n'
        assert git(clone, 'ls-files', '-s', '--', f'ros2_ws/src/{name}').startswith('120000 ')
        assert (clone / 'ros2_ws/src' / name / 'package.xml').read_text() == 'pinned fixture\n'
    assert git(clone, 'status', '--porcelain') == ''


def test_fresh_clone_and_repeat(checkout):
    clone, _, pin = checkout
    install(clone)
    assert_pinned(clone, pin)
    assert all((clone / 'external/hyundai_robotics' / n / '.git').is_file() for n in REPOS)
    install(clone)
    assert_pinned(clone, pin)


def test_recursive_clone_and_local_update_strategy(checkout):
    clone, _, pin = checkout
    git(clone, 'submodule', 'update', '--init', '--recursive')
    for name in REPOS:
        path = f'external/hyundai_robotics/{name}'
        git(clone / path, 'checkout', 'humble')
        git(clone, 'config', f'submodule.{path}.update', 'merge')
    install(clone)
    assert_pinned(clone, pin)


def test_old_standalone_clones(checkout):
    clone, upstream, pin = checkout
    for name in REPOS:
        path = clone / 'external/hyundai_robotics' / name
        git(clone, 'clone', str(upstream), str(path))
        assert (path / '.git').is_dir()
    install(clone)
    assert_pinned(clone, pin)
    install(clone)
    assert_pinned(clone, pin)


@pytest.mark.parametrize('kind', ['tracked', 'untracked', 'staged'])
def test_dirty_checkout_preserved(checkout, kind):
    clone, _, pin = checkout
    install(clone)
    target = clone / 'external/hyundai_robotics' / REPOS[-1]
    file = target / ('local.txt' if kind == 'untracked' else 'package.xml')
    file.write_text('precious local work\n')
    if kind == 'staged':
        git(target, 'add', file.name)
    before = git(target, 'status', '--porcelain')
    result = install(clone, check=False)
    assert result.returncode != 0
    assert 'Local changes' in result.stderr
    assert file.read_text() == 'precious local work\n'
    assert git(target, 'status', '--porcelain') == before
    assert git(target, 'rev-parse', 'HEAD') == pin


@pytest.mark.parametrize('kind', ['missing', 'file', 'wrong'])
def test_invalid_symlink_fails_before_init(checkout, kind):
    clone, _, _ = checkout
    link = clone / 'ros2_ws/src' / REPOS[-1]
    link.unlink()
    if kind == 'file':
        link.write_text('local replacement')
    elif kind == 'wrong':
        link.symlink_to('../../wrong')
    result = install(clone, check=False)
    assert result.returncode != 0
    assert 'tracked relative symlink' in result.stderr
    assert not (clone / 'external/hyundai_robotics' / REPOS[0] / '.git').exists()


def test_non_repository_directory_preserved(checkout):
    clone, _, _ = checkout
    path = clone / 'external/hyundai_robotics' / REPOS[-1] / 'local.txt'
    path.write_text('keep me')
    result = install(clone, check=False)
    assert result.returncode != 0
    assert 'nonempty' in result.stderr
    assert path.read_text() == 'keep me'
    assert not (clone / 'external/hyundai_robotics' / REPOS[0] / '.git').exists()


def test_dirty_nested_submodule_preserved(checkout):
    clone, _, _ = checkout
    install(clone)
    file = clone / 'external/hyundai_robotics' / REPOS[-1] / 'nested/asset.txt'
    file.write_text('local nested work')
    result = install(clone, check=False)
    assert result.returncode != 0
    assert file.read_text() == 'local nested work'


def test_failed_fetch_can_be_retried(checkout):
    clone, upstream, pin = checkout
    unavailable = upstream.with_name('temporarily unavailable')
    upstream.rename(unavailable)
    try:
        assert install(clone, check=False).returncode != 0
    finally:
        unavailable.rename(upstream)
    install(clone)
    assert_pinned(clone, pin)

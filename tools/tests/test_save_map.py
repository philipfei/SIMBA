"""Offline failure-path checks for the live-map save helper."""
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


spec = importlib.util.spec_from_file_location('save_map', Path(__file__).parents[1] / 'save_map.py')
save_map = importlib.util.module_from_spec(spec)
spec.loader.exec_module(save_map)


def write_image(command):
    prefix = Path(command[command.index('-f') + 1])
    prefix.with_suffix('.yaml').write_text('image: map.pgm\n')
    prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\xff')


def test_first_save_creates_missing_parent_and_only_saves_image(tmp_path, monkeypatch):
    calls = []

    def run(command, timeout):
        calls.append(command)
        write_image(command)
        return 0

    monkeypatch.setattr(save_map, 'run', run)
    directory = tmp_path / 'maps' / 'room'
    assert save_map.main(['--directory', str(directory)]) == 0
    assert (directory / 'map.pgm').is_file()
    assert len(calls) == 1
    assert 'save_map_timeout:=20.0' in calls[0]


def test_failed_map_save_never_calls_serialization(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(save_map, 'run', lambda command, timeout: calls.append(command) or 1)
    assert save_map.main(['--directory', str(tmp_path), '--pose-graph']) == 1
    assert len(calls) == 1


def test_cli_success_without_files_is_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(save_map, 'run', lambda command, timeout: 0)
    assert save_map.main(['--directory', str(tmp_path)]) == 1


def test_missing_service_times_out_but_preserves_image(tmp_path, monkeypatch):
    def run(command, timeout):
        if 'map_saver_cli' in command:
            write_image(command)
            return 0
        raise subprocess.TimeoutExpired(command, timeout)

    monkeypatch.setattr(save_map, 'run', run)
    assert save_map.main(['--directory', str(tmp_path), '--pose-graph']) == 1
    assert (tmp_path / 'map.yaml').is_file()
    assert (tmp_path / 'map.pgm').is_file()


def test_existing_map_is_not_overwritten(tmp_path, monkeypatch):
    (tmp_path / 'map.yaml').write_text('original')
    monkeypatch.setattr(save_map, 'run', lambda *args: pytest.fail('must not run ROS'))
    assert save_map.main(['--directory', str(tmp_path)]) == 1
    assert (tmp_path / 'map.yaml').read_text() == 'original'


def test_real_command_timeout_reaps_process():
    with pytest.raises(subprocess.TimeoutExpired):
        save_map.run([sys.executable, '-c', 'import time; time.sleep(60)'], 0.1)


def test_service_success_without_graph_files_is_failure(tmp_path, monkeypatch):
    def run(command, timeout):
        if 'map_saver_cli' in command:
            write_image(command)
        return 0

    monkeypatch.setattr(save_map, 'run', run)
    assert save_map.main(['--directory', str(tmp_path), '--pose-graph']) == 1
    assert (tmp_path / 'map.pgm').is_file()

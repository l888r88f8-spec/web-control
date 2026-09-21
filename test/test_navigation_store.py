"""Tests for the shared atomic writer and navigation persistence."""
import os
from pathlib import Path

from web_control.atomic_io import atomic_write_text


def test_atomic_write_text_replaces_target_with_written_sibling(tmp_path, monkeypatch):
    path = tmp_path / "config" / "web_control" / "paused_navigation.json"
    replacements = []
    real_replace = os.replace

    def spying_replace(source, destination):
        source_path = Path(source)
        assert source_path.read_text(encoding="utf-8") == '{"a": 1}\n'
        replacements.append((source_path, Path(destination)))
        real_replace(source, destination)

    monkeypatch.setattr(os, "replace", spying_replace)
    atomic_write_text(path, '{"a": 1}\n')

    assert len(replacements) == 1
    temporary, destination = replacements[0]
    assert temporary.parent == path.parent
    assert destination == path
    assert path.read_text(encoding="utf-8") == '{"a": 1}\n'
    assert [entry.name for entry in path.parent.iterdir()] == ["paused_navigation.json"]

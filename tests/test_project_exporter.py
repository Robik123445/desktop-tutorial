import json
from project_exporter import collect_files, main


def test_collect_files(tmp_path):
    """Test gathering of project files by extension."""
    (tmp_path / "foo.py").write_text("print('hi')\n")
    (tmp_path / "bar.json").write_text('{"a":1}')
    files = collect_files(tmp_path)
    names = [f['filename'] for f in files]
    assert str((tmp_path / 'foo.py').as_posix()) in names
    assert str((tmp_path / 'bar.json').as_posix()) in names
    assert all('checksum' in f and 'full_content' in f for f in files)


def test_main(tmp_path, capsys):
    """Test writing the export JSON file via CLI."""
    (tmp_path / "a.py").write_text("print('hi')\n")
    main(['--root', str(tmp_path), '--output', str(tmp_path / 'out.json')])
    out = capsys.readouterr().out
    assert "Saved" in out
    data = json.loads((tmp_path / 'out.json').read_text())
    assert isinstance(data, list)
    assert 'build_info' in data[-1]
    assert len(data) == 2

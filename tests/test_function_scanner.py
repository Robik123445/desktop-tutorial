import json
from function_scanner import collect_functions, print_functions_list, main


def test_collect_functions(tmp_path):
    """Test collecting functions from Python files."""
    py_file = tmp_path / "sample.py"
    py_file.write_text(
        """\
"""
        "def foo():\n    \"\"\"One liner doc\n    more\"\"\"\n    pass\n\n"
        "def bar():\n    pass\n"
    )
    result = collect_functions(tmp_path)
    expected = [
        {'file': 'sample.py', 'function': 'foo', 'doc': 'One liner doc'},
        {'file': 'sample.py', 'function': 'bar', 'doc': ''},
    ]
    assert result == expected


def test_print_functions_list(capsys):
    """Test JSON printing of collected function data."""
    data = [{'file': 'a.py', 'function': 'f', 'doc': 'd'}]
    json_str = print_functions_list(data)
    captured = capsys.readouterr().out
    assert json.loads(json_str) == data
    assert json_str in captured


def test_main_cli(tmp_path, capsys):
    """Test CLI invocation of function scanner."""
    py = tmp_path / "a.py"
    py.write_text("def foo():\n    pass\n")
    main([str(tmp_path)])
    out = capsys.readouterr().out
    assert 'foo' in out

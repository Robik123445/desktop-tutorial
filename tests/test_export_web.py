import json
from pathlib import Path
from export_web import export_web


def test_export_web(tmp_path):
    # Vytvor testovaciu web štruktúru
    web = tmp_path / "web"
    web.mkdir()
    (web / "a.txt").write_text("hello", encoding="utf-8")

    sub = web / "sub"
    sub.mkdir()
    (sub / "b.txt").write_text("world", encoding="utf-8")

    # Ignorovaný adresár
    ignored = web / "node_modules"
    ignored.mkdir()
    (ignored / "c.txt").write_text("ignore", encoding="utf-8")

    # Exportuj obsah
    out = tmp_path / "out.json"
    data = export_web(web, out)

    # Over obsah súboru
    with out.open(encoding="utf-8") as fh:
        exported = json.load(fh)

    assert len(exported) == 2
    assert data == exported

    filenames = {item["filename"] for item in exported}
    assert filenames == {"a.txt", "sub/b.txt"}

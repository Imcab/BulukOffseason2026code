"""El replay visual se genera con datos coherentes y un HTML valido."""

import json
import re

from core.replay import build_replay, write_html


def test_replay_data_and_html(cfg, tmp_path):
    data = build_replay(cfg, [("bajo", 4.0, 4.0), ("alto", 30.0, 30.0)], n_robots=18, var_name="actual_openloop")
    assert len(data["robots"]) == 18 and len(data["scenarios"]) == 2
    for sc in data["scenarios"]:
        for run in sc["runs"]:
            assert len(run["x"]) == len(run["y"]) == len(run["th"]) > 10
            assert run["state"] in ("ok", "aborto", "inestable")
    states = lambda i: [r["state"] for r in data["scenarios"][i]["runs"]]  # noqa: E731
    assert states(0).count("inestable") == 0
    assert states(1).count("inestable") > 0  # kP 30/30 debe verse inestable en la prueba mantener_pose
    # Todos los escenarios usan los mismos robots: misma pose inicial.
    a, b = data["scenarios"][0]["runs"][5], data["scenarios"][1]["runs"][5]
    assert (a["x"][0], a["y"][0]) == (b["x"][0], b["y"][0])

    out = write_html(data, tmp_path / "replay.html")
    html = out.read_text(encoding="utf-8")
    assert "/*__REPLAY_DATA__*/" not in html
    blob = re.search(r"const DATA = (\{.*?\});\n", html, re.S).group(1)
    assert json.loads(blob)["variant"] == "actual_openloop"
    frag = write_html(data, tmp_path / "frag.html", fragment=True).read_text(encoding="utf-8")
    assert not frag.lstrip().lower().startswith("<!doctype") and frag.lstrip().startswith("<title>")

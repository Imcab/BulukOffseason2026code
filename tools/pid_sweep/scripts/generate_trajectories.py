"""Regenera trajectories/*.json a partir de path_constraints en params.yaml.

Uso (desde tools/pid_sweep):  python scripts/generate_trajectories.py
Las trayectorias ya vienen generadas; solo hace falta correr esto si cambian los constraints.
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.params import load_config  # noqa: E402
from core.trajectory import TRAJ_DIR  # noqa: E402
from core.trajgen import standard_set  # noqa: E402


def main():
    cfg = load_config()
    TRAJ_DIR.mkdir(exist_ok=True)
    for traj in standard_set(cfg["path_constraints"]):
        path = TRAJ_DIR / f"{traj['name']}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(traj, f, indent=1)
        print(f"{traj['name']:>20}: {traj['total_time']:.3f} s -> {path.name}")


if __name__ == "__main__":
    main()

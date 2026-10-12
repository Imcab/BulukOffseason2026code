import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.params import load_config  # noqa: E402
from core.trajectory import list_trajectories  # noqa: E402


@pytest.fixture(scope="session")
def cfg():
    return load_config()


@pytest.fixture(scope="session")
def trajs():
    return {t.name: t for t in list_trajectories()}

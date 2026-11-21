"""Shared helpers: the generator module, loaded by path (it is a tool, not a package)."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="session")
def generator():
    spec = importlib.util.spec_from_file_location("generate_interfaces", ROOT / "tools" / "generate_interfaces.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

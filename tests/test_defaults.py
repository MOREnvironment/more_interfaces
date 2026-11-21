"""Defaults of the library in the interface: NaN and inf are NOT representable (finding of the job A1), everything else is and arrives unchanged.

The rosidl generators of the C++ and Python typesupport read a .msg default with ``ast.literal_eval``, which has no ``nan`` or ``inf``: a .msg that carries
one does not build (shown 2026-11-21 on jazzy). The generator therefore writes no default for them and lists them in ``doc/unrepresentable_defaults.txt``.
(1) Text: every NaN / inf default of the library is in that list and in no .msg default. (2) Built messages (skipped only when the package is not
importable, i.e. outside colcon): every other default equals the library's; every listed field is 0 when unset, which is the stated limitation: a sender
that means "unchanged" (SetLimits, SetStationRadii) must set NaN itself.
"""
import dataclasses
import importlib
import math

import numpy as np
import pytest


def _same(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    return a.shape == b.shape and bool(np.all((a == b) | (np.isnan(a) & np.isnan(b))))


def _classes(generator):
    mods, base, plan, surf = generator.load_library()
    orders, reports, maneuvers = generator.collect(mods, base, plan, surf)
    out = {n: c for n, c in {**orders, **maneuvers, **reports}.items() if n != "ValidateOrder"}
    out["PartAction"] = generator.find_class(mods, "PartAction")
    return out


def _library_default(f):
    if f.default is not dataclasses.MISSING:
        return f.default
    if f.default_factory is not dataclasses.MISSING:
        return f.default_factory()
    return None


def _nonfinite(generator):
    for name, cls in _classes(generator).items():
        for f in dataclasses.fields(cls):
            d = _library_default(f)
            if isinstance(d, float) and not math.isfinite(d) or (isinstance(d, np.ndarray) and not np.all(np.isfinite(d))):
                yield name, f, d


def test_every_nan_and_inf_default_is_listed_and_none_is_in_a_msg(generator):
    listed = (generator.ROOT / "doc" / "unrepresentable_defaults.txt").read_text()
    kinds = set()
    for name, f, d in _nonfinite(generator):
        assert f"{name}.{f.name}: library default" in listed, f"{name}.{f.name} is not in the list"
        line = next(ln for ln in (generator.ROOT / "msg" / f"{name}.msg").read_text().splitlines() if ln.split("#")[0].split()[1:2] == [f.name])
        assert len(line.split("#")[0].split()) == 2, f"{name}.{f.name}: a nan / inf default was written into the .msg: {line}"
        kinds.add("nan" if np.any(np.isnan(d)) else "inf")
    assert kinds == {"nan", "inf"}


def test_built_messages_carry_the_representable_defaults_and_zero_for_the_listed(generator):
    try:
        msgmod = importlib.import_module("vehicle_api_interfaces.msg")
    except ImportError:
        pytest.skip("vehicle_api_interfaces is not built and sourced (run under colcon test)")
    nonfinite = {(n, f.name) for n, f, _ in _nonfinite(generator)}
    checked = 0
    for name, cls in _classes(generator).items():
        msg = getattr(msgmod, name)()
        for f in dataclasses.fields(cls):
            if f.name == "t" and name.endswith(("Report", "Reply")):
                continue
            d = _library_default(f)
            got = getattr(msg, f.name, None)
            if (name, f.name) in nonfinite:
                assert _same(got, np.zeros_like(np.asarray(d, dtype=float))), f"{name}.{f.name}: expected 0 when unset, got {got!r}"
                checked += 1
            elif f.type in ("float", "int") and d is not None:
                assert _same(got, d), f"{name}.{f.name}: {got!r} != library default {d!r}"
                checked += 1
            elif f.type == "str" and d is not None:
                assert got == d, f"{name}.{f.name}"
                checked += 1
            elif f.type == "bool" and d is not None:
                assert got is d, f"{name}.{f.name}"
                checked += 1
    assert checked > 100

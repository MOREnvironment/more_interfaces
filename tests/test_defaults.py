"""Defaults of the library in the interface. A NaN or inf default cannot be a .msg default (rosidl reads it with ``ast.literal_eval``: shown 2026-11-21 on
jazzy), so such a field is an OPTIONAL field (owner decision 2026-11-22): ``float64[<=1] name``, empty = the library's default; an array whose default is all
NaN is ``float64[<=N]``, empty = all NaN, else exactly N values. The comment of the field states the default and what empty means.

(1) Text: every NaN / inf default of the library is such a field, listed in ``doc/unrepresentable_defaults.txt``, its comment states the library's default,
and no .msg carries a nan / inf literal. (2) Built messages (skipped only when the package is not importable, i.e. outside colcon): every optional field is
empty when unset and every other default equals the library's.
"""
import dataclasses
import importlib
import math
import re

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


def _msg_line(generator, name, field):
    for ln in (generator.ROOT / "msg" / f"{name}.msg").read_text().splitlines():
        decl = ln.split("#")[0].split()
        if decl[1:2] == [field]:
            return ln
    raise AssertionError(f"{name}.{field}: no declaration")


def _text(d):
    arr = np.atleast_1d(np.asarray(d, dtype=float))
    one = "nan" if np.isnan(arr[0]) else ("inf" if arr[0] > 0 else "-inf")
    return one if np.ndim(d) == 0 else "[" + ", ".join(one for _ in arr) + "]"


def test_every_nan_and_inf_default_is_an_optional_field_with_the_libraries_default_stated(generator):
    listed = (generator.ROOT / "doc" / "unrepresentable_defaults.txt").read_text()
    kinds, n = set(), 0
    for name, f, d in _nonfinite(generator):
        line = _msg_line(generator, name, f.name)
        decl, _, note = line.partition("#")
        n += 1
        if np.ndim(d) == 0:
            assert decl.split()[0] == "float64[<=1]", f"{name}.{f.name} is not optional: {line}"
        else:
            assert decl.split()[0] == f"float64[<={len(d)}]", f"{name}.{f.name}: {line}"
        assert len(decl.split()) == 2, f"{name}.{f.name}: a default was written: {line}"
        assert "optional: empty = " in note, f"{name}.{f.name}: the comment does not say what empty means"
        stated = re.search(r"MGM default (\S.*?)(;|$)", note)
        assert stated and stated.group(1).strip() == _text(d), f"{name}.{f.name}: documented default {stated and stated.group(1)!r} != MGM's {_text(d)!r}"
        assert f"{name}.{f.name}: {decl.split()[0]}, MGM default {_text(d)}," in listed, f"{name}.{f.name} is not in the list"
        kinds.add("nan" if np.any(np.isnan(d)) else "inf")
    assert kinds == {"nan", "inf"} and n >= 61
    assert listed.count("MGM default") == n, "the list has entries the library no longer has"


def test_no_msg_carries_a_nan_or_inf_literal(generator):
    bad = [(p.name, ln) for p in sorted(generator.ROOT.glob("msg/*.msg")) + sorted(generator.ROOT.glob("srv/*.srv")) + sorted(generator.ROOT.glob("action/*.action"))
           for ln in p.read_text().splitlines() if re.search(r"\b-?(nan|inf)\b", ln.split("#")[0], re.I)]
    assert not bad, bad


def test_built_messages_carry_the_representable_defaults_and_empty_for_the_optional(generator):
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
                assert len(got) == 0, f"{name}.{f.name}: expected empty (= MGM's default) when unset, got {got!r}"
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

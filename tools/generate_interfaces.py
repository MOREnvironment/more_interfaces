#!/usr/bin/env python3
"""Generate the ROS 2 interface files of the vehicle API from the simulation library's order and report classes.

The library (``more_generic_models``) is imported as an installed package; its version is stated in every generated file. The output is
deterministic: the same classes give the same bytes. ``python tools/generate_interfaces.py`` writes ``msg/``, ``srv/``, ``action/`` and
``doc/metadata_gaps.txt`` and the stamp ``GENERATED_FROM``; ``--check`` writes nothing and fails when a committed file differs.

Field types are read from the library's declaration alone: the field's annotation (``float``, ``int``, ``bool``, ``str``, ``tuple[str, ...]``,
``tuple[tuple[str, float], ...]``, ``dict[str, float]``, a dataclass) or, where Python cannot say it, the ``elements`` entry of the field's metadata
(``vector``, ``matrix``, ``square_matrix``, ``records``, ``any_order``, ``open_mapping``); orders come from ``base.order_types()``. A field that can be
mapped by neither stops the generator (an MGM gap is reported, not worked around). A tuple entry gets a generated message named by its entry types
(``StrFloat``) with fields ``item0``, ``item1``: the library names no entry field. An ``int`` is ``uint32`` when its unit is ``id``, else ``int64``.
A field whose default is NaN or inf becomes an optional field (see ``OPTIONAL``): ``float64[<=1]``, empty = the library's default; an array whose
default is all NaN, or None, is ``float64[<=N]``, empty = that default, otherwise the full array. What an empty field means is stated in ``OPTIONAL``
(the library does not declare it per field).
Units, frames and notes are read from the field's ``metadata`` (``unit``, ``frame``, ``doc``) when the library gives them and go into the field comment;
a value the library states as ``unknown`` / ``mixed`` / ``record`` is written as given and listed in ``doc/metadata_gaps.txt`` as stated unknown by the
library. Where the library gives no metadata at all, the comment carries the marker ``GAP`` and the field is listed there as missing.
The version of the library is read from the ``pyproject.toml`` next to the imported package, its commit from the environment variable ``MGM_COMMIT``
(or from git when the package sits in a checkout).
"""
from __future__ import annotations

import argparse
import dataclasses
import importlib
import importlib.metadata
import math
import re
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
PACKAGE = "vehicle_api_interfaces"

# What an empty optional field means, per (class, field), then per class, then per field name. A nan / inf default with no entry stops the generator.
_UNCHANGED = "unchanged (the limit is left as it is)"
OPTIONAL = {
    ("StationKeeping", "z"): "not given (a vehicle that needs it refuses the order and names the field; one with no use for it takes the order without)",
    ("StationKeeping", "radius"): "not given (a vehicle that needs it refuses the order and names the field; one whose station radii are set by SetStationRadii takes the order without)",
    ("StationKeeping", "speed"): "not given (a vehicle that needs it refuses the order and names the field)",
    ("SpiralProfile", "start_z"): "the depth the vehicle has",
    ("CapabilitiesReport", "deadman_timeout"): "none (the vehicle has no dead-man timeout)",
    ("CapabilitiesReport", "origin_lat"): "the scenario states none", ("CapabilitiesReport", "origin_lon"): "the scenario states none",
    ("CapabilitiesReport", "origin_alt"): "the scenario states none",
    ("EnergyReport", "confidence"): "the sensor gives none",
    ("EstimatedStateReport", "altitude"): "no valid measurement",
    ("EstimatedStateReport", "velocity_ned"): "unknown",
    ("PartStateReport", "value"): "none (the monitor judged no number)",
    ("PlanStateReport", "plan_time_to_go"): "unknown", ("PlanStateReport", "maneuver_time_to_go"): "unknown",
    ("VehicleModeReport", "last_error_t"): "none (no breach yet)", ("VehicleModeReport", "maneuver_start_t"): "none (no manoeuvre runs)",
    ("VehicleModeReport", "maneuver_time_to_go"): "unknown",
}
OPTIONAL_CLASS = {
    "SetLimits": _UNCHANGED, "SetStationRadii": "unchanged (the radius is left as it is)",
    "FixReport": "the fix gives none", "LimitsReport": "none (no monitor enforces this limit)", "LinkReport": "the link has none",
    "NavigationUncertaintyReport": "the navigation gives none",
}
OPTIONAL_NAME = {"timeout": "no timeout", "duration": "never ends by time"}

FRAME_NOTE = ("Frames (owner decision): positions NED about the scenario origin (x north, y east), body quantities FRD, angles in rad from north "
              "clockwise, time in s on the scenario clock.")
STAMP_FIELDS = (("std_msgs/Header", "header"), ("uint64", "stream_seq"), ("float64", "t"))
SCALARS = {float: "float64", int: "int64", bool: "bool", str: "string"}
DTYPES = {"float": "float64", "bool": "bool", "str": "string"}
MGM_MODULES = ("more_generic_models.vehicle_api.messages", "more_generic_models.vehicle_api.plan_reference.messages",
               "more_generic_models.vehicle_api.external_control.messages")

_ASCII = {"×": "x", "→": "->", "—": "-", "–": "-", "≤": "<=", "≥": ">=", "π": "pi", "σ": "sigma", "°": " deg", "²": "^2", "·": "*", "≈": "~", "’": "'"}


def snake(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def ascii_text(s: str) -> str:
    for k, v in _ASCII.items():
        s = s.replace(k, v)
    return s.encode("ascii", "replace").decode()


def comment(text: str, indent: str = "") -> list:
    text = ascii_text(" ".join(text.split()))
    return [f"{indent}# {ln}" for ln in textwrap.wrap(text, 128 - len(indent))] if text else []


def fnum(v: float) -> str:
    if math.isnan(v):
        return "nan"
    if math.isinf(v):
        return "inf" if v > 0 else "-inf"
    return repr(float(v))


# --- reading the library -----------------------------------------------------------------------------------------------------------------------
def load_library():
    mods = [importlib.import_module(m) for m in MGM_MODULES]
    base = importlib.import_module("more_generic_models.vehicle_api.base")
    plan = importlib.import_module("more_generic_models.vehicle_api.plan_reference.base")
    surf = importlib.import_module("more_generic_models.vehicle_api.external_control.base")
    return mods, base, plan, surf


def find_class(mods, name):
    for m in mods:
        c = getattr(m, name, None)
        if dataclasses.is_dataclass(c):
            return c
    raise KeyError(f"no dataclass {name!r} in {[m.__name__ for m in mods]}")


def library_dir() -> Path:
    return Path(importlib.import_module("more_generic_models").__file__).resolve().parent


def library_version() -> str:
    """The version in the ``pyproject.toml`` beside the imported package (an unpacked tree), else the installed distribution's."""
    pp = library_dir().parent / "pyproject.toml"
    if pp.exists():
        m = re.search(r'^version\s*=\s*"([^"]+)"', pp.read_text(), re.M)
        if m:
            return m.group(1)
    return importlib.metadata.version("more_generic_models")


def library_commit() -> str:
    """``MGM_COMMIT`` when set (a tree unpacked with ``git archive`` has no git data), else the commit of the checkout the library is read from."""
    import os
    if os.environ.get("MGM_COMMIT"):
        return os.environ["MGM_COMMIT"]
    try:
        d = library_dir()
        out = subprocess.run(["git", "-C", str(d), "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(d), "status", "--porcelain", "--untracked-files=no"], capture_output=True, text=True).stdout.strip()
        return out + (" (modified tree)" if dirty else "")
    except Exception:       # noqa: BLE001
        return "unknown"


def collect(mods, base, plan, surf):
    """Orders (``base.order_types()``), reports (the families' ``REPORT_TYPES``) and manoeuvres (the classes a ``records`` entry of a field names)."""
    orders = {c.__name__: c for c in base.order_types()}
    report_names = set()
    for api in (plan.PlanReferenceApiBase, surf.ExternalControlApiBase):
        report_names.update(api.REPORT_TYPES)
    reports = {n: find_class(mods, n) for n in sorted(report_names)}
    maneuvers = {}
    for cls in [*orders.values(), *reports.values()]:
        for f in dataclasses.fields(cls):
            el = f.metadata.get("elements")
            if el and el["kind"] == "records":
                maneuvers.update({n: find_class(mods, n) for n in el["of"]})
    return dict(sorted(orders.items())), reports, dict(sorted(maneuvers.items()))


# --- field mapping -----------------------------------------------------------------------------------------------------------------------------
class Gaps(list):
    pass


UNKNOWN = []        # fields whose unit or frame the library states as unknown / mixed / record
OPTIONAL_FIELDS = []


def _base_of(names, classes):
    """The name of the union message of a set of classes: their common base class, minus a ``Fields`` suffix."""
    bases = set.intersection(*[set(c.__mro__) for c in classes]) - {object}
    bases = [b for b in classes[0].__mro__ if b in bases and dataclasses.is_dataclass(b)]
    if not bases:
        raise TypeError(f"no common base dataclass for {names}")
    return bases[0].__name__.removesuffix("Fields")


def _entry(t, where):
    """The ros type of one scalar of a tuple entry, named by its Python type."""
    if t in SCALARS:
        return SCALARS[t], t.__name__.capitalize()
    raise TypeError(f"{where}: tuple entry type {t!r} cannot be mapped")


def _pair(types, where):
    """A generated entry message for ``(t0, t1, ...)``: named by the entry types (``StrFloat``), fields ``item0``, ``item1``: the library names no entry."""
    parts = [_entry(t, where) for t in types]
    name = "".join(n for _, n in parts)
    ENTRIES[name] = [(ros, f"item{i}", None, "") for i, (ros, _) in enumerate(parts)]
    return name


def map_type(cls, f, hint, mods, where):
    """(ros type, optional-default marker) from the declaration alone: the annotation, else the ``elements`` entry; anything else raises."""
    import typing
    el = f.metadata.get("elements")
    origin, args = typing.get_origin(hint), typing.get_args(hint)
    if origin is typing.Union and type(None) in args:         # Optional[X]: absence (None) is the field's default
        hint = next(a for a in args if a is not type(None))
        origin, args = typing.get_origin(hint), typing.get_args(hint)
    if el:
        k = el["kind"]
        if k == "vector":
            return f"{DTYPES[el['dtype']]}[{el['length']}]", None
        if k == "matrix":
            n = None if el["rows"] is None else el["rows"] * el["cols"]
            return (f"{DTYPES[el['dtype']]}[]" if n is None else f"{DTYPES[el['dtype']]}[{n}]"), f"matrix {el['rows'] or 'any number of'} x {el['cols']}, row-major"
        if k == "square_matrix":
            sizes = el["size"] if isinstance(el["size"], tuple) else (el["size"],)
            top = max(sizes) ** 2
            return (f"{DTYPES[el['dtype']]}[<={top}]" if len(sizes) > 1 else f"{DTYPES[el['dtype']]}[{top}]"), \
                f"square matrix, row-major, size {' or '.join(map(str, sizes))}"
        if k == "records":
            classes = [find_class(mods, n) for n in el["of"]]
            return _base_of(el["of"], classes) + "[]", None
        if k == "any_order":
            return "Order", None
        if k == "open_mapping":
            vals = el["values"]
            if vals == ["any"] or vals == ("any",):
                return _pair((str, str), where) + "[]", "open mapping: values of any type, carried as text (their type is lost)"
            if all(v == "float" for v in vals):
                return _pair((str, float), where) + "[]", None
        raise TypeError(f"{where}: elements {el!r} cannot be mapped")
    if hint in SCALARS:
        if hint is int:
            return ("uint32" if f.metadata.get("unit") == "id" else "int64"), None
        return SCALARS[hint], None
    if origin is tuple and len(args) == 2 and args[1] is Ellipsis:
        a = args[0]
        if a in SCALARS:
            return SCALARS[a] + "[]", None
        if typing.get_origin(a) is tuple:
            return _pair(typing.get_args(a), where) + "[]", None
        if dataclasses.is_dataclass(a):
            REFERENCED.setdefault(a.__name__, a)
            return a.__name__ + "[]", None
    if origin is tuple and args and Ellipsis not in args and all(a is float for a in args):
        return f"float64[{len(args)}]", None
    if origin is dict and args == (str, float):
        return _pair((str, float), where) + "[]", None
    raise TypeError(f"{where}: annotation {hint!r} has no mapping and no `elements` entry in the library (an MGM gap, not to be worked around here)")


ENTRIES = {}
REFERENCED = {}


def field_lines(cls, gaps, skip=("t",), owner=None, unknown=None):
    import typing
    unknown = UNKNOWN if unknown is None else unknown
    owner_name = owner or cls.__name__
    """(ros type, name, default text, comment) per field of ``cls``."""
    out = []
    hints = typing.get_type_hints(cls)
    mods = [importlib.import_module(m) for m in MGM_MODULES]
    for f in dataclasses.fields(cls):
        if f.name in skip:
            continue
        key = (cls.__name__, f.name)
        where = f"{cls.__name__}.{f.name}"
        hint = hints[f.name]
        ros, shape_note = map_type(cls, f, hint, mods, where)
        default = None
        dval = f.default if f.default is not dataclasses.MISSING else (f.default_factory() if f.default_factory is not dataclasses.MISSING else dataclasses.MISSING)
        scalar = hint in SCALARS
        if scalar and f.default is not dataclasses.MISSING:
            d = f.default
            default = fnum(d) if hint is float else ("true" if d is True else "false") if hint is bool else (repr(int(d)) if hint is int else f'"{d}"')
            if (hint is str and d == "") or (hint is int and d == 0) or (hint is float and d == 0.0 and not math.copysign(1, d) < 0):
                default = None
        elif scalar and f.default_factory is not dataclasses.MISSING:
            raise TypeError(f"{where}: a default factory on a scalar field")
        notes = []
        optional_array = (dval is None) or (isinstance(dval, np.ndarray) and not np.all(np.isfinite(dval)))
        if optional_array:                    # None, or an all-NaN vector: empty = the library's default (the ros array is bounded)
            n = ros[ros.index("[") + 1:-1].lstrip("<=") if "[" in ros and not ros.endswith("[]") else None
            if n is not None:
                ros = f"float64[<={n}]"
            shown = "None" if dval is None else "[" + ", ".join("nan" if np.isnan(x) else fnum(x) for x in dval) + "]"
            meaning = OPTIONAL.get(key) or OPTIONAL_CLASS.get(cls.__name__) or ("none (the library's None)" if dval is None else None)
            if meaning is None:
                raise TypeError(f"{where}: library default {shown} has no entry in OPTIONAL (state what an empty field means)")
            notes.append(f"optional: empty = {meaning}; otherwise the full array. MGM default {shown}")
            OPTIONAL_FIELDS.append(f"{owner_name}.{f.name}: {ros}, MGM default {shown}, empty = {meaning}")
        if shape_note:
            notes.append(shape_note)
        if scalar and default is not None and ("nan" in default or "inf" in default):
            meaning = OPTIONAL.get(key) or OPTIONAL_CLASS.get(cls.__name__) or OPTIONAL_NAME.get(f.name)
            if meaning is None:
                raise TypeError(f"{where}: library default {default} has no entry in OPTIONAL (state what an empty field means)")
            ros = "float64[<=1]"
            notes.append(f"optional: empty = {meaning}; send one value to set it. MGM default {default}")
            OPTIONAL_FIELDS.append(f"{owner_name}.{f.name}: {ros}, MGM default {default}, empty = {meaning}")
            default = None
        meta = f.metadata
        unit, frame, mdoc = meta.get("unit"), meta.get("frame"), meta.get("doc")
        if unit:
            notes.append(f"unit: {unit}")
        if frame and frame != "none":
            notes.append(f"frame: {frame}")
        if mdoc:
            notes.append(" ".join(ascii_text(mdoc).split()))
        for what, val in (("unit", unit), ("frame", frame)):
            if val in ("unknown", "mixed", "record"):
                unknown.append(f"{owner_name}.{f.name}: {what} {val}")
        if "unit" not in meta:
            notes.append("GAP: unit and frame not given by the library")
            gaps.append(f"{owner_name}.{f.name}: unit and frame")
        out.append((ros, f.name, default, "; ".join(notes)))
    return out


def render_fields(fields, with_defaults=True):
    lines = []
    for ros, name, default, note in fields:
        decl = f"{ros} {name}" + (f" {default}" if default is not None and with_defaults else "")
        lines.append(decl + (f"  # {note}" if note else ""))
    return lines


def msg_text(name, doc, fields, library, preamble=(), helper=False):
    lines = [f"# {name}: generated by tools/generate_interfaces.py from more_generic_models {library}. Do not edit."]
    lines += comment(doc) + (["#"] + comment(FRAME_NOTE) if not helper else []) + list(preamble)
    lines += render_fields(fields)
    return "\n".join(lines) + "\n"


NO_DOC = []


def doc_of(cls):
    import inspect
    doc = inspect.getdoc(cls) or ""
    if not doc or doc.startswith(cls.__name__ + "("):          # a dataclass with no docstring gets its signature as __doc__
        NO_DOC.append(cls.__name__)
        return "GAP: the library gives no description of this class."
    return doc


def generate() -> dict:
    mods, base, plan, surf = load_library()
    orders, reports, maneuvers = collect(mods, base, plan, surf)
    version = library_version()
    library = version
    gaps = Gaps()
    files = {}
    NO_DOC.clear()
    UNKNOWN.clear()
    OPTIONAL_FIELDS.clear()

    ENTRIES.clear()
    REFERENCED.clear()
    order_fields = {}
    for name, cls in {**orders, **maneuvers}.items():
        if any(f.metadata.get("elements", {}).get("kind") == "any_order" for f in dataclasses.fields(cls)):
            continue                      # an order that holds any order is a service (ValidateOrder), not a member of Order
        order_fields[name] = field_lines(cls, gaps, skip=())
        files[f"msg/{name}.msg"] = msg_text(name, doc_of(cls), order_fields[name], library)

    for name, cls in reports.items():
        flds = field_lines(cls, gaps, skip=("t",))
        if dataclasses.fields(cls)[0].name != "t":
            raise TypeError(f"{name}: first field is not t")
        extra = [("string", "child_frame_id", None, "frame of the body vectors, <vehicle>/base_link_fsd (FRD), set by the bridge")] if name == "EstimatedStateReport" else []
        stamp = [("std_msgs/Header", "header", None, "stamp = t rounded to ns; frame_id map_ned for positions"),
                 ("uint64", "stream_seq", None, "publication counter per vehicle across all report types (restores the drain order)"),
                 ("float64", "t", None, "unit: s; the sample instant on the scenario clock, exactly")]
        files[f"msg/{name}.msg"] = msg_text(name, doc_of(cls), stamp + extra + flds, library)

    done = set(files)
    while any(f"msg/{n}.msg" not in done for n in REFERENCED):          # records the fields hold (PartAction)
        for n, c in sorted(REFERENCED.items()):
            if f"msg/{n}.msg" not in done:
                files[f"msg/{n}.msg"] = msg_text(n, doc_of(c), field_lines(c, gaps, skip=()), library)
                done.add(f"msg/{n}.msg")
    for n, flds in sorted(ENTRIES.items()):
        files[f"msg/{n}.msg"] = msg_text(n, "An entry of a library tuple or mapping; the library names no entry field: item0, item1, ... in its order.", flds, library, helper=True)

    def union(uname, doc, members):
        flds = [("string", "kind", None, "the class name of the member that is set")] + [(f"{m}[<=1]", snake(m), None, "") for m in members]
        files[f"msg/{uname}.msg"] = msg_text(uname, doc, flds, library)

    union(_base_of(list(maneuvers), list(maneuvers.values())), "One manoeuvre of a plan: exactly one member is set, named by kind (ROS interfaces cannot be recursive or polymorphic).", list(maneuvers))
    union("Order", "One order: exactly one member is set, named by kind. ValidateOrder is a service of its own (an order that holds any order cannot be a member).",
          list(order_fields))

    srv_head = "# {n}: generated by tools/generate_interfaces.py from more_generic_models {v}. Do not edit.\n"
    files["srv/SendOrder.srv"] = (srv_head.format(n="SendOrder", v=library) +
        "# Hand an order to the vehicle. The response comes at once; the vehicle's answer is the OrderReply on the order_reply topic.\n"
        "# A malformed order gives order_id = 0 and the library's text in error (no id is used).\n"
        "Order order\n---\nuint32 order_id\nstring error\nstring code  # a key of the library's refusal codes; empty when none\n")
    files["srv/CheckOrder.srv"] = (srv_head.format(n="CheckOrder", v=library) +
        "# Would the vehicle take this order at send time. Nothing is queued and no id is used.\n"
        "Order order\n---\nbool takes\nstring reason\nstring code  # a key of the library's refusal codes; empty when the order is taken\n")
    files["srv/ValidateOrder.srv"] = (srv_head.format(n="ValidateOrder", v=library) +
        "# Send a ValidateOrder: asks whether the vehicle would take the order as it is now, nothing is applied. Answered by an OrderReply.\n"
        "Order order\n---\nuint32 order_id\nstring error\nstring code\n")
    files["srv/Describe.srv"] = (srv_head.format(n="Describe", v=library) +
        "# What the vehicle's API offers. Reads only; never sends a QueryCapabilities.\n"
        "---\nstring vehicle\nstring model\nstring[] topics  # the library's names of the reports and channels\n"
        "string[] report_topics  # the same as topic names\nbool has_capabilities\nCapabilitiesReport capabilities\n")
    files["action/ExecutePlan.action"] = (
        "# ExecutePlan: generated by tools/generate_interfaces.py from more_generic_models " + library + ". Do not edit.\n"
        "# Run a plan: built only from StartPlan, PlanStateReport and StopPlan. The definition is here; the server is a later stage.\n"
        "StartPlan start\n---\nuint32 order_id\n"
        "string outcome  # SUCCEEDED, FAILED, ABORTED, CANCELLED, REPLACED, REFUSED or RESET\nstring reason\nPlanStateReport last\n---\nPlanStateReport state\n")

    gaps.extend(f"{n}: class docstring" for n in sorted(set(NO_DOC)))
    unknown = sorted(set(UNKNOWN))
    gap_text = ("# Fields whose unit or frame the library does not state (marked GAP in the .msg comments), and fields whose unit or frame the library itself\n"
                "# states as unknown, mixed or record (written as given in the .msg comments). Generated.\n"
                f"# more_generic_models {library}: {len(set(gaps))} missing, {len(unknown)} stated unknown by the library.\n"
                "# missing (the library gives no metadata for the field):\n" + "\n".join(sorted(set(gaps))) + "\n"
                "# stated unknown by the library (unit or frame is unknown, mixed or record):\n" + "\n".join(unknown) + "\n")
    files["doc/metadata_gaps.txt"] = gap_text
    files["doc/unrepresentable_defaults.txt"] = (
        "# Library defaults that a rosidl .msg cannot carry (NaN, inf): each such field is an optional field (a bounded array of at most one value, or at most\n"
        "# N for a vector), empty = the library's default. Generated.\n"
        f"# more_generic_models {library}: {len(set(OPTIONAL_FIELDS))} entries.\n" + "\n".join(sorted(set(OPTIONAL_FIELDS))) + "\n")
    lines = ["# Generated by tools/generate_interfaces.py. Do not edit."]
    for d in ("msg", "srv", "action"):
        lines += [f'list(APPEND INTERFACE_FILES "{p}")' for p in sorted(files) if p.startswith(d + "/")]
    files["cmake/interface_files.cmake"] = "\n".join(lines) + "\n"
    return files


def stamp() -> str:
    return f"more_generic_models {library_version()}\ncommit {library_commit()}\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="write nothing; exit 1 when a committed file differs from a fresh generation")
    a = ap.parse_args(argv)
    files = generate()
    existing = {str(p.relative_to(ROOT)) for d in ("msg", "srv", "action") for p in (ROOT / d).glob("*") if p.is_file()} | {"doc/metadata_gaps.txt", "doc/unrepresentable_defaults.txt", "cmake/interface_files.cmake"}
    stale = sorted(existing - set(files))
    changed = sorted(p for p, t in files.items() if not (ROOT / p).exists() or (ROOT / p).read_text() != t)
    if a.check:
        for p in changed + [f"{p} (stale)" for p in stale]:
            print("differs:", p)
        return 1 if changed or stale else 0
    for p in stale:
        (ROOT / p).unlink()
    for p, t in files.items():
        (ROOT / p).parent.mkdir(exist_ok=True)
        (ROOT / p).write_text(t)
    (ROOT / "GENERATED_FROM").write_text(stamp())
    print(f"wrote {len(files)} files from more_generic_models {library_version()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

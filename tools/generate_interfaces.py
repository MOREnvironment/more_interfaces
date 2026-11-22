#!/usr/bin/env python3
"""Generate the ROS 2 interface files of the vehicle API from the simulation library's order and report classes.

The library (``more_generic_models``) is imported as an installed package; its version is stated in every generated file. The output is
deterministic: the same classes give the same bytes. ``python tools/generate_interfaces.py`` writes ``msg/``, ``srv/``, ``action/`` and
``doc/metadata_gaps.txt`` and the stamp ``GENERATED_FROM``; ``--check`` writes nothing and fails when a committed file differs.

Field types: ``float`` -> float64, ``int`` -> int32 / uint32 / int64 (by name, ``INT_TYPES``), ``bool`` -> bool, ``str`` -> string, each with the
library's default, except that a field whose default is NaN or inf becomes an optional field (see ``OPTIONAL``): ``float64[<=1]``, empty = the
library's default, or for an array whose default is all NaN ``float64[<=N]``, empty = all NaN, otherwise exactly N values. Every other field (arrays, tuples, dicts, nested records, ``object``) is named in ``SPECIAL``; a field of
a reachable class that is neither a scalar nor in ``SPECIAL`` stops the generator, so a field the library appends later breaks the build, not the run.
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

# --- the interface's own helper records -------------------------------------------------------------------------------------------------------
HELPERS = {
    "KeyValue": ("A name and a number (a setting, a period).", [("string", "key", None, ""), ("float64", "value", None, "")]),
    "Waypoint": ("One point of a path: north and east in m (NED, about the scenario origin), z in the unit of the manoeuvre's z_mode, speed in m/s.",
                 [("float64", n, None, "") for n in ("x", "y", "z", "speed")]),
    "Transition": ("An ordered pair of manoeuvre ids: the destination starts when the source is done.",
                   [("string", "source", None, ""), ("string", "destination", None, "")]),
    "PlaneVertex": ("A vertex of a polygon: north and east in m (NED, about the scenario origin).", [("float64", "x", None, ""), ("float64", "y", None, "")]),
    "LimitBound": ("One operating limit and the bounds within which it can be set (same unit as the limit).",
                   [("string", "field", None, ""), ("float64", "minimum", None, ""), ("float64", "maximum", None, "")]),
    "Matrix": ("A matrix, row-major. rows = 0 means there is none (the library's None).",
               [("uint32", "rows", None, ""), ("uint32", "cols", None, ""), ("float64[]", "data", None, "")]),
}

# Helper records emitted only when a field uses them (the library may not have the field yet).
OPTIONAL_HELPERS = {
    "EntryLevelOrder": ("A pair: an entry level and the name of an order that commands it.", [("string", "level", None, ""), ("string", "order", None, "")]),
}

# (class, field) -> (ros type, default or None). Everything that is not a plain float / int / bool / str.
NAN3, NAN6 = "[nan, nan, nan]", "[nan, nan, nan, nan, nan, nan]"
SPECIAL = {
    ("EstimatedStateReport", "eta"): ("float64[6]", None), ("EstimatedStateReport", "nu"): ("float64[6]", None),
    ("EstimatedStateReport", "velocity_ned"): ("float64[3]", NAN3),
    ("NavigationUncertaintyReport", "covariance"): ("Matrix", None),
    ("VehicleModeReport", "parts_in_error"): ("string[]", None),
    ("PathTrackingReport", "leg_start"): ("float64[3]", None), ("PathTrackingReport", "leg_end"): ("float64[3]", None),
    ("ControlOutputReport", "requested_wrench"): ("float64[6]", None), ("ControlOutputReport", "realized_wrench"): ("float64[6]", None),
    ("ControlOutputReport", "enabled"): ("bool[6]", None),
    ("AidingEventReport", "innovation"): ("float64[]", None),
    ("SensorReadingReport", "values"): ("KeyValue[]", None), ("SensorReadingReport", "covariance"): ("Matrix", None),
    ("PayloadStateReport", "settings"): ("KeyValue[]", None),
    ("LimitsReport", "region"): ("PlaneVertex[]", None), ("SetLimits", "region"): ("PlaneVertex[]", None),
    ("CapabilitiesReport", "orders"): ("string[]", None), ("CapabilitiesReport", "reports"): ("string[]", None),
    ("CapabilitiesReport", "entry_levels"): ("string[]", None), ("CapabilitiesReport", "periods"): ("KeyValue[]", None),
    ("CapabilitiesReport", "limits"): ("LimitBound[]", None), ("CapabilitiesReport", "entry_level_orders"): ("EntryLevelOrder[]", None),
    ("PlanQueryReply", "maneuver_ids"): ("string[]", None), ("PlanQueryReply", "maneuvers"): ("Maneuver[]", None),
    ("PlanQueryReply", "transitions"): ("Transition[]", None),
    ("PlanListReply", "plan_ids"): ("string[]", None), ("PlanListReply", "checksums"): ("string[]", None),
    ("PartAction", "settings"): ("KeyValue[]", None), ("SetPayload", "settings"): ("KeyValue[]", None),
    ("FollowPath", "points"): ("Waypoint[]", None),
    ("StorePlan", "maneuvers"): ("Maneuver[]", None), ("StorePlan", "transitions"): ("Transition[]", None),
    ("StartPlan", "maneuvers"): ("Maneuver[]", None),
}
for _m in ("ManeuverFields",):
    SPECIAL[(_m, "start_actions")] = ("PartAction[]", None)
    SPECIAL[(_m, "end_actions")] = ("PartAction[]", None)

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

INT_TYPES = {"order_id": "uint32", "index": "int32", "count": "int32"}      # any other int field: int64
FRAME_NOTE = ("Frames (owner decision): positions NED about the scenario origin (x north, y east), body quantities FRD, angles in rad from north "
              "clockwise, time in s on the scenario clock.")
STAMP_FIELDS = (("std_msgs/Header", "header"), ("uint64", "stream_seq"), ("float64", "t"))
SCALARS = {"float": "float64", "int": "int64", "bool": "bool", "str": "string"}
NUMERIC = ("float64", "float64[]", "float64[3]", "float64[6]", "Matrix", "KeyValue[]", "LimitBound[]", "PlaneVertex[]", "Waypoint[]")
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
    orders = {}
    for api in (plan.PlanReferenceApiBase, surf.ExternalControlApiBase):
        orders.update(api.ORDERS)
    for c in base.CATEGORY_ORDERS:
        orders[c.__name__] = c
    report_names = set()
    for api in (plan.PlanReferenceApiBase, surf.ExternalControlApiBase):
        report_names.update(api.REPORT_TYPES)
    reports = {n: find_class(mods, n) for n in sorted(report_names)}
    maneuvers = {c.__name__: c for c in plan.messages.MANEUVERS}
    return dict(sorted(orders.items())), reports, dict(sorted(maneuvers.items()))


# --- field mapping -----------------------------------------------------------------------------------------------------------------------------
class Gaps(list):
    pass


UNKNOWN = []        # fields whose unit or frame the library states as unknown / mixed / record
OPTIONAL_FIELDS = []


def field_lines(cls, gaps, skip=("t",), owner=None, unknown=None):
    unknown = UNKNOWN if unknown is None else unknown
    owner_name = owner or cls.__name__
    """(ros type, name, default text, comment) per field of ``cls``; ``owner`` names the msg for the gap list."""
    out = []
    for f in dataclasses.fields(cls):
        if f.name in skip:
            continue
        ann = f.type if isinstance(f.type, str) else getattr(f.type, "__name__", str(f.type))
        key = (cls.__name__, f.name)
        spec = SPECIAL.get(key) or SPECIAL.get(("ManeuverFields", f.name) if f.name in ("start_actions", "end_actions") else key)
        if spec:
            ros, default = spec
        elif ann in SCALARS:
            ros = SCALARS[ann]
            if ann == "int":
                ros = INT_TYPES.get(f.name, "int64")
            default = None
            if f.default is not dataclasses.MISSING:
                d = f.default
                default = fnum(d) if ann == "float" else ("true" if d is True else "false") if ann == "bool" else (repr(int(d)) if ann == "int" else f'"{d}"')
                if ann == "str" and d == "":
                    default = None
                if ann == "int" and d == 0:
                    default = None
                if ann == "float" and f.default == 0.0 and not math.copysign(1, f.default) < 0:
                    default = None
            elif f.default_factory is not dataclasses.MISSING:
                raise TypeError(f"{cls.__name__}.{f.name}: a default factory on a scalar field")
        else:
            raise TypeError(f"{cls.__name__}.{f.name}: type {ann!r} has no entry in SPECIAL (add one to the generator)")
        if ann == "float" and not spec and f.default is not dataclasses.MISSING and f.default == 0.0:
            default = None
        notes = []
        if default is not None and ("nan" in default or "inf" in default):
            # rosidl (C++ and Python typesupport) reads a default with ast.literal_eval, which has no nan or inf: the field becomes optional, empty = the library's default
            meaning = OPTIONAL.get(key) or OPTIONAL_CLASS.get(cls.__name__) or (OPTIONAL_NAME.get(f.name) if cls.__name__ != "SetLimits" else None)
            if meaning is None:
                raise TypeError(f"{cls.__name__}.{f.name}: library default {default} has no entry in OPTIONAL (state what an empty field means)")
            if ros == "float64":
                ros = "float64[<=1]"
                notes.append(f"optional: empty = {meaning}; send one value to set it. MGM default {default}")
            elif ros.startswith("float64[") and default.startswith("[nan"):
                n = default.count("nan")
                ros = f"float64[<={n}]"
                notes.append(f"optional: empty = {meaning}, the library's all-NaN vector; otherwise exactly {n} values (NaN entries allowed). MGM default {default}")
            else:
                raise TypeError(f"{cls.__name__}.{f.name}: no optional form for type {ros} with default {default}")
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
        if ros in NUMERIC or ros.startswith("float64[<=") :
            if not unit and not frame and "unit" not in meta:
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


def msg_text(name, doc, fields, library, preamble=()):
    lines = [f"# {name}: generated by tools/generate_interfaces.py from more_generic_models {library}. Do not edit."]
    lines += comment(doc) + (["#"] + comment(FRAME_NOTE) if name not in HELPERS else []) + list(preamble)
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

    for name, (doc, fields) in HELPERS.items():
        files[f"msg/{name}.msg"] = msg_text(name, doc, fields, library)

    # PartAction (a record of the shared order base, not an order itself)
    part_action = find_class(mods, "PartAction")
    files["msg/PartAction.msg"] = msg_text("PartAction", doc_of(part_action), field_lines(part_action, gaps, skip=()), library)

    order_fields = {}
    for name, cls in {**orders, **maneuvers}.items():
        if name == "ValidateOrder":
            continue
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

    def union(uname, doc, members):
        flds = [("string", "kind", None, "the class name of the member that is set")] + [(f"{m}[<=1]", snake(m), None, "") for m in members]
        files[f"msg/{uname}.msg"] = msg_text(uname, doc, flds, library)

    union("Maneuver", "One manoeuvre of a plan: exactly one member is set, named by kind (ROS interfaces cannot be recursive or polymorphic).", list(maneuvers))
    union("Order", "One order: exactly one member is set, named by kind. ValidateOrder is a service of its own (an order that holds any order cannot be a member).",
          [n for n in order_fields if n != "ValidateOrder"])

    for name, (doc, fields) in OPTIONAL_HELPERS.items():
        if any(ln.startswith(name + "[]") for txt in files.values() for ln in txt.splitlines()):
            files[f"msg/{name}.msg"] = msg_text(name, doc, fields, library)

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

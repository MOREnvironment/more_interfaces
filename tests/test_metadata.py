"""Field metadata of the library (unit, frame, doc) in the field comments, and the list of what the library states as unknown (works on any library
version: the class under test is built here, not read from the library)."""
import dataclasses
import math

import numpy as np


def _cls():
    def f(unit, frame="none", default=dataclasses.MISSING, doc="", elements=None):
        meta = {"unit": unit, "frame": frame, "doc": doc, **({"elements": elements} if elements else {})}
        return dataclasses.field(metadata=meta) if default is dataclasses.MISSING else dataclasses.field(default=default, metadata=meta)

    @dataclasses.dataclass
    class Probe:
        t: float = f("s")
        pos: np.ndarray = f("mixed", "ned", doc="[north, east (m), yaw (rad)]", elements={"kind": "vector", "length": 3, "dtype": "float"})
        speed: float = f("m/s", "unknown", 1.0, doc="over ground or through water")
        origin: float = f("m", "geodetic", 2.0)
        level: str = f("enum", default="a")
        bare: float = 0.0
    return Probe


def test_unit_frame_and_doc_go_into_the_comment_and_unknowns_are_counted(generator):
    generator.UNKNOWN.clear()
    gaps = generator.Gaps()
    lines = {name: (ros, default, note) for ros, name, default, note in generator.field_lines(_cls(), gaps, skip=("t",))}
    assert lines["pos"][2] == "unit: mixed; frame: ned; [north, east (m), yaw (rad)]"
    assert "unit: m/s" in lines["speed"][2] and "frame: unknown" in lines["speed"][2] and "over ground or through water" in lines["speed"][2]
    assert "frame: none" not in lines["level"][2] and lines["level"][2] == "unit: enum"
    assert [g for g in gaps] == ["Probe.bare: unit and frame"] and "GAP" in lines["bare"][2]
    assert sorted(generator.UNKNOWN) == ["Probe.pos: unit mixed", "Probe.speed: frame unknown"]


def test_a_field_the_library_cannot_say_stops_the_generator(generator):
    import pytest

    @dataclasses.dataclass
    class Bare:
        a: np.ndarray = dataclasses.field(metadata={"unit": "mixed", "frame": "none", "doc": ""})

    with pytest.raises(TypeError, match="no mapping and no `elements`"):
        generator.field_lines(Bare, generator.Gaps(), skip=())

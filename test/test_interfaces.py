"""The built ``more_interfaces``: every message and service (both halves) round-trips through the middleware's serialisation, NaN and
infinity survive, and the interface files carry no source, vendor, vehicle or protocol words and say in a leading comment what they are.
Needs the built package."""
import math
import re
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from rosidl_runtime_py.convert import message_to_ordereddict  # noqa: E402
from rosidl_runtime_py.utilities import get_message, get_service  # noqa: E402

SHARE = Path(get_package_share_directory("more_interfaces"))
ALL_TYPES = sorted([f"more_interfaces/msg/{p.stem}" for p in (SHARE / "msg").glob("*.msg")]
                   + [f"more_interfaces/srv/{p.stem}" for p in (SHARE / "srv").glob("*.srv")]
                   + [f"more_interfaces/action/{p.stem}" for p in (SHARE / "action").glob("*.action")])
# The interfaces older than the vehicle contract are not under the naming and comment tests.
OLDER = {"AllocatorResponse", "FinsResponse", "IMU", "QuaternionNed", "ThrusterResponse", "CalibrateIMU"}


def resolve(type_name):
    kind = type_name.split("/")[1]
    return get_service(type_name) if kind == "srv" else get_message(type_name)


def test_every_interface_file_resolves():
    for t in ALL_TYPES:
        if "/action/" not in t:
            assert resolve(t) is not None, t
    assert len(ALL_TYPES) >= 44


# -- round trip ------------------------------------------------------------------------------------------------------------------
_LIMIT = {"uint8": 200, "int8": 100, "uint16": 60000, "int16": 30000, "uint32": 2 ** 31, "int32": 2 ** 30, "uint64": 2 ** 40, "int64": 2 ** 40}


def make(t, i, counter):
    """A distinct value of the IDL type ``t`` (a nested message is filled, field by field)."""
    if "/" in t:
        pkg, name = t.split("/")
        msg = get_message(f"{pkg}/msg/{name}")()
        fill(msg, counter)
        return msg
    if t == "boolean":
        return bool(i % 2)
    if t == "string":
        return f"text{i}"
    if t in ("float", "double"):
        return i + 0.25
    if t == "octet":
        return bytes([i % 200 + 1])
    return i % _LIMIT[t] + 1


def fill(msg, counter):
    """Set every field of ``msg`` to a distinct value: nested messages, sequences (two items) and arrays included."""
    for name, t in msg.get_fields_and_field_types().items():
        counter[0] += 1
        i = counter[0]
        if t.startswith("sequence<"):
            setattr(msg, name, [make(t[9:-1], i + k, counter) for k in range(2)])
        elif t.endswith("]"):
            setattr(msg, name, [make(t[:t.index("[")], i + k, counter) for k in range(len(getattr(msg, name)))])
        else:
            setattr(msg, name, make(t, i, counter))


def same(a, b):
    if isinstance(a, float) and isinstance(b, float):
        return (math.isnan(a) and math.isnan(b)) or a == b
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    return a == b


@pytest.mark.parametrize("type_name", [t for t in ALL_TYPES if "/action/" not in t])
def test_round_trip(type_name):
    kind = type_name.split("/")[1]
    classes = [resolve(type_name)] if kind == "msg" else [resolve(type_name).Request, resolve(type_name).Response]
    for cls in classes:
        msg = cls()
        fill(msg, [0])
        back = deserialize_message(serialize_message(msg), cls)
        assert same(message_to_ordereddict(msg), message_to_ordereddict(back)), cls.__name__


def test_round_trip_keeps_nan_and_infinity():
    m = get_message("more_interfaces/msg/Maneuver")()
    m.start_z, m.duration, m.timeout = math.nan, math.inf, math.inf
    m.position.altitude = math.nan
    back = deserialize_message(serialize_message(m), type(m))
    assert math.isnan(back.start_z) and math.isinf(back.duration) and math.isinf(back.timeout) and math.isnan(back.position.altitude)



def test_range_measurement_keeps_validity_pose_beam_and_range():
    cls = get_message("more_interfaces/msg/RangeMeasurement")
    m = cls()
    m.header.frame_id = "sonar_link"
    m.target_id = "7"
    m.validity = 1
    m.pose.position.x, m.pose.orientation.w = 1.5, 1.0
    m.beam_width, m.beam_height, m.range = 0.5, 0.25, 12.5
    back = deserialize_message(serialize_message(m), cls)
    assert (back.header.frame_id, back.target_id, back.validity, back.range, back.beam_width, back.beam_height) == \
        ("sonar_link", "7", 1, 12.5, 0.5, 0.25)
    assert back.pose.position.x == 1.5 and back.pose.orientation.w == 1.0


def test_range_measurement_has_one_frame_field_and_an_identifier_field():
    # The frame is header.frame_id only; what the range was measured to is an identifier, not a frame.
    fields = get_message("more_interfaces/msg/RangeMeasurement").get_fields_and_field_types()
    assert "frame_id" not in fields and fields["target_id"] == "string"


def test_range_measurement_beam_nan_means_not_reported():
    cls = get_message("more_interfaces/msg/RangeMeasurement")
    m = cls()
    m.beam_width = m.beam_height = math.nan
    back = deserialize_message(serialize_message(m), cls)
    assert math.isnan(back.beam_width) and math.isnan(back.beam_height)


def test_range_measurement_validity_has_three_values():
    # 0 invalid, 1 valid, 2 not reported: an input without validity is not made valid or invalid.
    cls = get_message("more_interfaces/msg/RangeMeasurement")
    assert (cls.VALIDITY_INVALID, cls.VALIDITY_VALID, cls.VALIDITY_NOT_REPORTED) == (0, 1, 2)
    m = cls()
    m.validity = cls.VALIDITY_NOT_REPORTED
    m.range = 120.0
    back = deserialize_message(serialize_message(m), cls)
    assert back.validity == cls.VALIDITY_NOT_REPORTED and back.range == 120.0


def test_water_column_altitude_nan_means_no_measurement():
    cls = get_message("more_interfaces/msg/WaterColumn")
    m = cls()
    m.depth, m.altitude = 20.0, math.nan
    back = deserialize_message(serialize_message(m), cls)
    assert back.depth == 20.0 and math.isnan(back.altitude)


# -- names and comments ------------------------------------------------------------------------------------------------------------
FORBIDDEN = re.compile(r"imc|dune|neptus|vsim|lsts|mss|fossen|evologics|marie|grethe|otter|remus", re.I)


def new_files():
    return [p for p in list((SHARE / "msg").glob("*.msg")) + list((SHARE / "srv").glob("*.srv")) if p.stem not in OLDER]


def test_names_and_contents_carry_no_source_vendor_vehicle_or_protocol_words():
    bad = [t for t in ALL_TYPES if FORBIDDEN.search(t.split("/")[-1])]
    for p in new_files():
        bad += [f"{p.name}: {w}" for w in FORBIDDEN.findall(p.read_text())]
    assert bad == []


def test_every_new_interface_says_what_it_is_in_a_comment():
    for p in new_files():
        assert p.read_text().lstrip().startswith("#"), p.name

# vehicle_api_interfaces

ROS 2 messages, services and one action for the order-and-report API of a vehicle (ament_cmake, rosidl). The package is named by function: a
simulation bridge and a bridge to a real vehicle use the same types, so autonomy code runs unchanged on either.

## Contents (61 messages, 4 services, 1 action)

- 27 order messages (11 manoeuvres, 16 other orders) and 25 report messages, one per class of the library, field names unchanged.
  Reports carry `std_msgs/Header header`, `uint64 stream_seq` (the bridge's counter across all report types), `float64 t` (the exact sample instant), then
  the library's fields. `EstimatedStateReport` adds `child_frame_id`.
- Helper records: `KeyValue`, `Waypoint`, `Transition`, `PlaneVertex`, `LimitBound`, `Matrix` (rows = 0: none), `PartAction`.
- Unions: `Maneuver` and `Order` (a `kind` string and one `<Type>[<=1]` member per class, because rosidl has no polymorphism or recursion).
  `ValidateOrder` holds any order, so it is a service, not a member of `Order`.
- Services: `SendOrder`, `CheckOrder`, `ValidateOrder` (all `Order order`), `Describe`. Action: `ExecutePlan` (definition only).
- Orders common to both families are one class each (`StationKeeping`, `Abort`); a family refuses the fields it cannot honour.

## Namespace

Entities of a vehicle `<v>` sit under `/<v>/api/` (neutral, no simulation or vendor word): services `send`, `check`, `validate`, `describe`, action
`execute_plan`, reports `report/<snake_case of the class>` (e.g. `/<v>/api/report/order_reply`). A bridge takes the root as a parameter; autonomy code
uses relative names. The package only defines types; it publishes nothing.

## Frames and units

By definition the messages carry the library's values: positions NED about the scenario origin, body vectors FRD, angles in rad, time in s.
Where the library states a unit or frame in the class text it is copied into the comment; machine-readable `unit` / `frame` metadata is read from
the field when the library has it, and where it does not, the field comment says `GAP` and the field is in `doc/metadata_gaps.txt`.

## Generation

`tools/generate_interfaces.py` imports the installed `more_generic_models` (version in `tools/requirements.txt`) and writes `msg/`, `srv/`, `action/`,
`cmake/interface_files.cmake` and `doc/*.txt` deterministically; generated files are committed. `GENERATED_FROM` records the library version and commit.

    python tools/generate_interfaces.py           # write
    python tools/generate_interfaces.py --check   # exit 1 when a committed file differs

Type mapping: float -> float64, int -> int32 (`index`, `count`) / uint32 (`order_id`) / int64, bool -> bool, str -> string, with the library's default.
Arrays, tuples, dicts and nested records are mapped by an explicit table in the generator (`SPECIAL`); a field of a reachable class that is neither a
scalar nor in the table stops the generator.

Generated from: more_generic_models 0.1.0, develop at commit 29c46184 (before the branches `api-plan-identity` and `api-field-metadata`).

## NaN and inf defaults are not representable

rosidl (jazzy) reads a `.msg` default with `ast.literal_eval`: a default `nan` or `inf` makes the C++ and Python generators fail. Such fields get
no default (an unset field is 0) and a comment; `doc/unrepresentable_defaults.txt` lists them (63). A sender that means "unchanged" in `SetLimits` or
`SetStationRadii` must set NaN itself.

## Tests (`colcon test`, or `pytest tests/` with the library installed)

- `test_consistency.py`: committed files equal a fresh generation; every order and report has a message with all its fields; no vendor or protocol names.
- `test_defaults.py`: every NaN / inf default is listed and in no `.msg`; in the built messages every other default equals the library's, listed ones are 0.

## Not covered yet

The bridge, the action server, QoS, the `describe` content, the frame parameter, a codec between dataclasses and messages, the library's list of
real-vehicle messages. Service responses carry `code` (library refusal codes) as added by the library after the design was written.

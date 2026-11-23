# vehicle_api_interfaces

ROS 2 messages, services and one action for the order-and-report API of a vehicle (ament_cmake, rosidl). The package is named by function: a
simulation bridge and a bridge to a real vehicle use the same types, so autonomy code runs unchanged on either.

## Contents (63 messages, 4 services, 1 action)

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
Where the library states a unit or frame in the class text it is copied into the comment. The library's machine-readable field metadata (`unit`, `frame`, `doc`)
goes into the field comment when it has it (frame `none` is left out); a value the library states as `unknown`, `mixed` or `record` is written as given and
counted in `doc/metadata_gaps.txt` as "stated unknown by the library". Where the library gives no metadata for a field, the comment says `GAP` and the field is
listed there as missing.

## Generation

`tools/generate_interfaces.py` imports the installed `more_generic_models` (version in `tools/requirements.txt`; the version is read from the `pyproject.toml` beside the imported package, the commit from `MGM_COMMIT` or git) and writes `msg/`, `srv/`, `action/`,
`cmake/interface_files.cmake` and `doc/*.txt` deterministically; generated files are committed. `GENERATED_FROM` records the library version and commit.

    python tools/generate_interfaces.py           # write
    python tools/generate_interfaces.py --check   # exit 1 when a committed file differs

Type mapping: float -> float64, int -> int32 (`index`, `count`) / uint32 (`order_id`) / int64, bool -> bool, str -> string, with the library's default
(a NaN or inf default makes the field optional, see below).
Arrays, tuples, dicts and nested records are mapped by an explicit table in the generator (`SPECIAL`); a field of a reachable class that is neither a
scalar nor in the table stops the generator.

Generated from: more_generic_models 0.1.0, develop at commit a00c281c (field metadata, capabilities additions, `KeepAlive`, plan identity). The generator is run with an unpacked `git archive` of that commit first
on the path, so the checkout and the environment of the library are untouched:

    git -C <more_generic_models> archive <commit> | tar -x -C <dir>
    MGM_COMMIT=<commit> PYTHONPATH=<dir> python tools/generate_interfaces.py

## Optional fields: NaN and inf defaults

rosidl (jazzy) reads a `.msg` default with `ast.literal_eval`: a default `nan` or `inf` makes the C++ and Python generators fail. A field whose library default
is NaN or inf (61 of them, `doc/unrepresentable_defaults.txt`) is therefore an **optional field**: a bounded array of at most one value, `float64[<=1] name`.
Empty means the library's default; the comment on the field states that default and its meaning ("empty = unchanged", "empty = no timeout", "empty = unknown").
A vector whose default is all NaN (`EstimatedStateReport.velocity_ned`) is `float64[<=3]`: empty = all NaN, otherwise exactly 3 values (NaN entries allowed).
A new NaN / inf default of the library with no stated meaning in the generator (`OPTIONAL`) stops the generator.

For a ROS 2 client (Python), `SetLimits` with an unchanged depth limit and a new speed limit:

    msg = SetLimits()
    msg.max_speed = [2.5]      # set it
    # msg.max_depth stays [] = unchanged (the library's NaN)

and `StationKeeping(x, y)` with no `timeout` is `timeout = []` (no timeout); `timeout = [120.0]` stops it after 120 s. A bridge reads empty as the library's
default and one element as that value; a report field that is empty means NaN (unknown).

## Hand table the generator still carries

MGM gives no machine-readable shape for tuple fields, so `SPECIAL` in the generator names them (`KeyValue`, `LimitBound`, `EntryLevelOrder`, `Waypoint`, `Transition`, `PlaneVertex`, `Matrix`): `CapabilitiesReport.periods`, `.limits`, `.entry_level_orders`, the `settings` and `covariance` fields, `FollowPath.points`, `parts_in_error` and others. Only the `doc` text describes their entries.

## Tests (`colcon test`, or `pytest tests/` with the library installed)

- `test_consistency.py`: committed files equal a fresh generation; every order and report has a message with all its fields; no vendor or protocol names.
- `test_defaults.py`: every NaN / inf default is an optional field whose comment states the library's default, is listed, and is in no `.msg` as a literal; in the built messages optional fields are empty and every other default equals the library's.
- `test_metadata.py`: unit / frame / doc reach the comment; unknown / mixed / record are counted.

## Not covered yet

The bridge, the action server, QoS, the `describe` content, the frame parameter, a codec between dataclasses and messages, the library's list of
real-vehicle messages. Service responses carry `code` (library refusal codes) as added by the library after the design was written.

"""The committed interface files equal a fresh generation from the installed simulation library (the consistency test).

A failure means the library changed (or a generated file was edited by hand): run ``python tools/generate_interfaces.py`` and commit the result.
The library is imported as an installed package; if it cannot be imported this test fails, it does not skip.
"""
import re

from conftest import ROOT


def test_committed_files_equal_a_fresh_generation(generator):
    files = generator.generate()
    differing = [p for p, text in sorted(files.items()) if not (ROOT / p).exists() or (ROOT / p).read_text() != text]
    on_disk = {str(p.relative_to(ROOT)) for d in ("msg", "srv", "action") for p in (ROOT / d).glob("*") if p.is_file()}
    stale = sorted(on_disk - set(files))
    assert not differing and not stale, f"differ from a fresh generation: {differing}; committed but no longer generated: {stale}"


def test_every_order_and_report_has_a_message(generator):
    mods, base, plan, surf = generator.load_library()
    orders, reports, maneuvers = generator.collect(mods, base, plan, surf)
    names = {p.stem for p in (ROOT / "msg").glob("*.msg")}
    wanted = (set(orders) - {"ValidateOrder"}) | set(reports) | set(maneuvers)
    assert wanted <= names, f"no message for: {sorted(wanted - names)}"
    for cls_name in set(orders) - {"ValidateOrder"}:
        text = (ROOT / "msg" / f"{cls_name}.msg").read_text()
        cls = {**orders}[cls_name]
        import dataclasses
        for f in dataclasses.fields(cls):
            assert re.search(rf"^\S+ {f.name}( |$)", text, re.M), f"{cls_name}.{f.name} missing in its .msg"
    # the unions name every order and every manoeuvre
    order_msg = (ROOT / "msg" / "Order.msg").read_text()
    for n in set(orders) - {"ValidateOrder"}:
        assert f"{n}[<=1] {generator.snake(n)}" in order_msg
    maneuver_msg = (ROOT / "msg" / "Maneuver.msg").read_text()
    for n in maneuvers:
        assert f"{n}[<=1] {generator.snake(n)}" in maneuver_msg


def test_no_names_the_interface_must_not_carry():
    banned = re.compile(r"\b(dune|imc|nmea|capnp|marie|grethe|remus|otter|ros1)\b", re.I)
    hits = [(p.name, m.group(0)) for p in sorted(ROOT.glob("msg/*")) + sorted(ROOT.glob("srv/*")) + sorted(ROOT.glob("action/*"))
            for m in [banned.search(p.read_text())] if m]
    assert not hits, hits

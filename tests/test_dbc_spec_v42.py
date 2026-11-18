"""Strukturprüfung der DBC gegen die Soll-Spezifikation (Roadmap v4.2, Phase 2.3).

Jede Botschaft und jedes Signal wird einzeln geprüft, damit ein roter Test
direkt zeigt, was in der DBC fehlt oder abweicht.
"""
import math

import cantools
import pytest

from can_guard_spec import (BITRATE, BUSLOAD_LIMIT_NORMAL, BUSLOAD_LIMIT_WORST, DBC_PATH,
                            MESSAGES, NODES, RESERVED_IDS)

SIGNALS = [(m, s) for m in MESSAGES for s in m.signals]


@pytest.fixture(scope="module")
def db():
    assert DBC_PATH.is_file(), f"DBC-Datei fehlt: {DBC_PATH}"
    # strict=True: überlappende Signale oder Signale außerhalb des DLC brechen das Laden ab
    return cantools.database.load_file(str(DBC_PATH), strict=True)


def _message(db, spec):
    try:
        return db.get_message_by_frame_id(spec.frame_id)
    except KeyError:
        pytest.fail(f"Botschaft 0x{spec.frame_id:03X} ({spec.name}) fehlt in der DBC")


def test_dbc_laedt_fehlerfrei(db):
    assert len(db.messages) > 0


def test_alle_knoten_definiert(db):
    vorhanden = {node.name for node in db.nodes}
    fehlend = [n for n in NODES if n not in vorhanden]
    assert not fehlend, f"BU_ fehlt: {fehlend}"


def test_reservierte_ids_unbenutzt(db):
    ids = {m.frame_id for m in db.messages}
    belegt = [hex(i) for i in RESERVED_IDS if i in ids]
    assert not belegt, f"reservierte ID belegt: {belegt}"


def test_keine_unbekannten_botschaften(db):
    erlaubt = {m.frame_id for m in MESSAGES}
    fremd = [f"0x{m.frame_id:03X} {m.name}" for m in db.messages if m.frame_id not in erlaubt]
    assert not fremd, f"Botschaften ohne Spezifikation: {fremd}"


@pytest.mark.parametrize("spec", MESSAGES, ids=lambda m: f"0x{m.frame_id:03X}")
def test_botschaft(db, spec):
    msg = _message(db, spec)
    assert msg.name == spec.name, "Name"
    assert not msg.is_extended_frame, "muss 11-bit Standard-ID sein"
    assert msg.length == spec.dlc, f"DLC: erwartet {spec.dlc}, DBC {msg.length}"
    assert msg.senders == [spec.sender], f"Sender: erwartet {spec.sender}, DBC {msg.senders}"
    assert {s.name for s in msg.signals} == {s.name for s in spec.signals}, "Signalnamen"


@pytest.mark.parametrize("spec,sig", SIGNALS, ids=lambda x: getattr(x, "name", ""))
def test_signal(db, spec, sig):
    msg = _message(db, spec)
    try:
        s = msg.get_signal_by_name(sig.name)
    except KeyError:
        pytest.fail(f"Signal {sig.name} fehlt in {spec.name}")
    assert s.byte_order == "little_endian", "Byte-Reihenfolge muss Intel (@1) sein"
    assert s.start == sig.start, f"Startbit: erwartet {sig.start}, DBC {s.start}"
    assert s.length == sig.length, f"Länge: erwartet {sig.length}, DBC {s.length}"
    assert s.is_signed == sig.signed, "Vorzeichen"
    assert math.isclose(s.scale, sig.scale), f"Faktor: erwartet {sig.scale}, DBC {s.scale}"
    assert s.offset == 0, "Offset muss 0 sein"
    assert (s.minimum, s.maximum) == (sig.minimum, sig.maximum), \
        f"Wertebereich: erwartet [{sig.minimum}|{sig.maximum}], DBC [{s.minimum}|{s.maximum}]"
    fehlend = [r for r in spec.receivers if r not in s.receivers]
    assert not fehlend, f"Empfänger fehlen: {fehlend}"


def _frame_bits_worst_case(dlc: int) -> int:
    # Standard-Frame: 47 Bit Overhead + Daten + maximales Bit-Stuffing über 34 + 8*DLC Bit
    return 47 + 8 * dlc + (34 + 8 * dlc - 1) // 4


def test_buslast():
    zyklisch = sum(_frame_bits_worst_case(m.dlc) * 1000 / m.cycle_ms for m in MESSAGES if m.cycle_ms)
    worst = sum(_frame_bits_worst_case(m.dlc) * m.max_rate_hz for m in MESSAGES)
    assert zyklisch / BITRATE < BUSLOAD_LIMIT_NORMAL, f"zyklische Buslast {zyklisch / BITRATE:.2%}"
    assert worst / BITRATE < BUSLOAD_LIMIT_WORST, f"Worst-Case-Buslast {worst / BITRATE:.2%}"

"""Soll-Spezifikation der CAN-Kommunikation nach Roadmap v4.2.

Einzige Quelle der Wahrheit für die Tests: Ändert sich die Roadmap, wird nur
diese Datei angepasst. Alle uint16/int16-Signale sind Little Endian (Intel),
passend zur Firmware (Low-Byte zuerst).
"""
from dataclasses import dataclass, field
from pathlib import Path

DBC_PATH = Path(__file__).resolve().parent.parent / "dbc" / "can_guard_network.dbc"

GATEWAY = "Gateway"          # Node 5 (ESP32-S3); Name aus der bestehenden DBC
PC = "PC_Analyzer"           # SavvyCAN / CANable 2.0
BITRATE = 500_000            # bit/s
BUSLOAD_LIMIT_NORMAL = 0.01  # Roadmap: rechnerisch < 1 %
BUSLOAD_LIMIT_WORST = 0.05   # Roadmap: Grenzwert im Test 5 %
RESERVED_IDS = (0x1FF,)      # darf in der DBC nicht vorkommen


@dataclass(frozen=True)
class Sig:
    name: str
    start: int               # Startbit (Intel, LSB)
    length: int
    signed: bool = False
    scale: float = 1
    minimum: float = 0
    maximum: float = 1
    unit: str = ""


@dataclass(frozen=True)
class Msg:
    name: str
    frame_id: int
    dlc: int
    sender: str
    receivers: tuple          # muss jeder Signal-Empfängerliste enthalten sein
    signals: tuple
    cycle_ms: int             # zyklisches Sendeintervall (0 = nur ereignisgesteuert)
    max_rate_hz: float        # Worst Case inkl. Ereignissen, für die Buslast
    example: dict = field(default_factory=dict)   # physikalische Werte
    example_hex: str = ""                         # erwartete Bytes (unabhängig hergeleitet)


def _dht(n: int, frame_id: int) -> Msg:
    return Msg(
        name=f"SENSOR_NODE_3_S{n}", frame_id=frame_id, dlc=5, sender="Node3",
        receivers=(GATEWAY, PC),
        signals=(
            Sig(f"DHT11_{n}_Temp", 0, 16, signed=True, scale=0.1, minimum=-20, maximum=60),
            Sig(f"DHT11_{n}_Hum", 16, 16, scale=0.1, minimum=0, maximum=100),
            Sig(f"DHT11_{n}_Status", 32, 8, minimum=0, maximum=1),
        ),
        cycle_ms=1000, max_rate_hz=1,
        # 20,0 °C -> 200 = 0x00C8, 60,0 % -> 600 = 0x0258, Status 0
        example={f"DHT11_{n}_Temp": 20.0, f"DHT11_{n}_Hum": 60.0, f"DHT11_{n}_Status": 0},
        example_hex="C8 00 58 02 00",
    )


def _diag(n: int, max_sensor_id: int) -> Msg:
    p = f"N{n}_Diag_"
    return Msg(
        name=f"DIAG_NODE_{n}", frame_id=0x1F0 + n, dlc=4, sender=f"Node{n}",
        receivers=(GATEWAY, PC),
        signals=(
            Sig(p + "Sensor_ID", 0, 8, minimum=1, maximum=max_sensor_id),
            Sig(p + "Event", 8, 8, minimum=0, maximum=1),        # 1 aktiv, 0 behoben
            Sig(p + "Error_Code", 16, 8, minimum=0, maximum=3),  # 1 ungültig, 2 Timeout, 3 Bereich
            Sig(p + "Counter", 24, 8, minimum=0, maximum=255),
        ),
        cycle_ms=0, max_rate_hz=5,   # Mindestabstand 200 ms
        example={p + "Sensor_ID": 1, p + "Event": 1, p + "Error_Code": 2, p + "Counter": 255},
        example_hex="01 01 02 FF",
    )


MESSAGES = (
    Msg(
        name="SENSOR_NODE_1", frame_id=0x101, dlc=3, sender="Node1",
        receivers=("Node3", GATEWAY, PC),   # Node 3 steuert damit den LED-Ring
        signals=(
            Sig("Switch_Status", 0, 8, minimum=0, maximum=1),
            Sig("LDR_Brightness", 8, 16, minimum=0, maximum=1023),
        ),
        cycle_ms=200, max_rate_hz=5,
        # LDR 750 = 0x02EE -> EE 02
        example={"Switch_Status": 1, "LDR_Brightness": 750},
        example_hex="01 EE 02",
    ),
    Msg(
        name="SENSOR_NODE_2", frame_id=0x102, dlc=2, sender="Node2",
        receivers=(GATEWAY, PC),
        signals=(Sig("Ultrasonic_Distance", 0, 16, minimum=0, maximum=400, unit="cm"),),
        cycle_ms=250, max_rate_hz=4,
        example={"Ultrasonic_Distance": 145},
        example_hex="91 00",
    ),
    _dht(1, 0x103),
    _dht(2, 0x104),
    _dht(3, 0x105),
    Msg(
        name="SENSOR_NODE_4", frame_id=0x106, dlc=2, sender="Node4",
        receivers=(GATEWAY, PC),
        signals=(
            Sig("Motion_Detected", 0, 8, minimum=0, maximum=1),
            Sig("SR602_Status", 8, 8, minimum=0, maximum=1),     # 1 = Anlaufphase
        ),
        cycle_ms=1000, max_rate_hz=5,    # 1 Hz + Flanken
        example={"Motion_Detected": 1, "SR602_Status": 0},
        example_hex="01 00",
    ),
    _diag(1, 2),   # 1 Schalter (reserviert), 2 LDR
    _diag(2, 1),   # 1 HC-SR04
    _diag(3, 4),   # 1-3 DHT11, 4 fehlende 0x101
    _diag(4, 1),   # 1 SR602
)

NODES = ("Node1", "Node2", "Node3", "Node4", GATEWAY, PC)

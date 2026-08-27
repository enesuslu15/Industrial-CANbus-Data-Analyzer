# 🏭 Industrial CANbus Data Analyzer — Project Report

---

## 1. Project Summary

This project demonstrates the development of an industrial automation communication analyzer built entirely in Python. It simulates a real CAN bus network, captures raw hexadecimal CAN frames, and decodes them into human-readable engineering values in real time.

Without any physical hardware, a **UDP Multicast** based virtual CAN network is established between two Python nodes — a motor drive emulator (Node A) and a data analyzer with live dashboard (Node B).

---

## 2. Engineering Motivation

```
Raw bus traffic (what a machine outputs):
  Motor → [0x100]  05 DC
  Motor → [0x101]  02 F3
  Motor → [0x102]  0C 0C

This project's output (what an engineer reads):
  0x100 | 05 DC | Motor RPM    →  1500.00 RPM  ✓
  0x101 | 02 F3 | Temperature  →  75.50 °C     ✓
  0x102 | 0C 0C | Torque       →  47.85 N·m    ✓
```

> *"Turning noisy industrial bus traffic into actionable engineering data."*

An engineer who can read and decode raw CAN traffic directly from a motor drive is invaluable for fault analysis, performance optimization, and predictive maintenance. This project proves that skill in software with zero hardware cost.

---

## 3. System Architecture

```
┌─────────────────────────────────────────────────────────┐
│               Virtual Industrial Network                │
│              (UDP Multicast · 239.0.0.1)                │
├──────────────────────┬──────────────────────────────────┤
│      NODE A          │           NODE B                 │
│  Motor Simulator     │    Analyzer + Live Dashboard     │
│                      │                                  │
│  • Sine-wave RPM     │  ┌──────────┬─────────────────┐ │
│  • Thermal model     │  │ HEX LOG  │  LIVE GRAPHS    │ │
│  • Power → Torque    │  │ TERMINAL │  (4 subplots)   │ │
│  • Fault flags       │  └──────────┴─────────────────┘ │
│                      │  Thread-safe DataStore           │
│  10 frames / sec     │  200 ms GUI refresh rate         │
└──────────────────────┴──────────────────────────────────┘
```

### 3.1 Layered Architecture

| Layer | File | Responsibility |
|-------|------|----------------|
| **Protocol** | `can_protocol.py` | CAN ID map, encode / decode engine |
| **Simulation** | `node_a_sender.py` | Motor physics model, frame generation |
| **Analysis + UI** | `node_b_analyzer_gui.py` | CAN listener, data store, GUI |

---

## 4. CAN Protocol Design

### 4.1 Signal Map

| CAN ID | Signal | Data Type | Scale | Unit | Example Hex | Decoded Value |
|--------|--------|-----------|-------|------|-------------|---------------|
| `0x100` | Motor RPM | uint16 BE | ×1 | RPM | `05 DC` | 1500 RPM |
| `0x101` | Temperature | int16 BE | ÷10 | °C | `02 F3` | 75.5 °C |
| `0x102` | Torque | int16 BE | ÷100 | N·m | `12 5E` | 47.50 N·m |
| `0x103` | DC Bus Voltage | uint16 BE | ÷10 | V | `15 8E` | 554.2 V |
| `0x104` | Phase Current | int16 BE | ÷100 | A | `08 34` | 20.84 A |
| `0x1FF` | Fault Flags | bitmask | — | — | `03` | OVERHEAT + OVERCURRENT |

> **Why Big-Endian?** Industrial protocols (CANopen, J1939, EtherCAT) all use big-endian byte ordering. This project follows the same convention for authenticity.

### 4.2 Fault Bitmask Design

```
Bit 4    Bit 3    Bit 2    Bit 1    Bit 0
  │        │        │        │        └── Overheat      (T > 80 °C)
  │        │        │        └─────────── Overcurrent   (I > 5 A)
  │        │        └──────────────────── Undervoltage  (V < 520 V)
  │        └───────────────────────────── Comm Loss
  └────────────────────────────────────── Encoder Fault
```

---

## 5. Motor Physics Model (Node A)

### 5.1 RPM Profile
```python
# Overlapping sine waves produce realistic load variation
wave = sin(t × 0.3) × 0.5 + sin(t × 0.07) × 0.3
RPM  = 1500 + wave × 850 + Gaussian(0, 15)
# → Dynamic swing between 800 and 2500 RPM
```

### 5.2 Temperature Model
```python
# Heat rise driven by load factor + thermal oscillation
T = 25 + (RPM / 2500) × 55 + sin(t × 0.05) × 3 + Gaussian(0, 0.5)
# → 35 °C (idle) to 85 °C (full load)
```

### 5.3 Torque and Current
```python
# Constant power theorem  (P = 7.5 kW)
Torque  = (P × 60) / (2π × RPM)   # ≈ 47 N·m @ 1500 RPM
Current = Torque / Kt               # Kt = 2.1 (motor constant)
```

---

## 6. Software Design Decisions

### 6.1 Why `udp_multicast` Instead of `virtual`?
`python-can`'s `virtual` bus is **process-scoped** — two separate terminal windows are two separate processes and cannot share it. `udp_multicast` provides real socket communication between independent processes, exactly as a physical CAN adapter would.

### 6.2 Thread Architecture
```
Main Thread  ──┬── tkinter GUI event loop
               │   ├── _update_log()     every 200 ms
               │   ├── _update_graphs()  every 200 ms
               │   └── _update_kpis()    every 500 ms
               │
Daemon Thread ─└── can_listener_thread()
                    └── bus.recv() → DataStore → log_queue
```
`threading.Lock()` keeps `DataStore` thread-safe at all times.

### 6.3 `try / finally` Guarantee
Every GUI update function is wrapped in `try / finally`. Any drawing error will not freeze the interface — `root.after()` is called unconditionally.

---

## 7. Encode / Decode Verification

| Test | Input | Raw Hex | Output | Result |
|------|-------|---------|--------|--------|
| RPM round-trip | 1500 | `05 DC` | 1500.00 RPM | ✅ Pass |
| Temperature round-trip | 75.5 °C | `02 F3` | 75.50 °C | ✅ Pass |
| Torque round-trip | 47.85 N·m | `12 5E` | 47.85 N·m | ✅ Pass |
| Single fault flag | OVERHEAT | `01` | "OVERHEAT" | ✅ Pass |
| Combined fault flags | OVERHEAT + OVERCURRENT | `03` | 2 faults listed | ✅ Pass |

---

## 8. Tech Stack

| Library | Version | Purpose |
|---------|---------|---------|
| `python-can` | 4.6.1 | CAN bus abstraction layer |
| `tkinter` | stdlib | Main GUI window |
| `matplotlib` | 3.8+ | Real-time subplots (TkAgg backend) |
| `rich` | 13.7+ | Colored terminal output — Matrix effect |
| `numpy` | 1.26+ | Array operations & time-window masking |
| `msgpack` | 1.1.2 | UDP multicast frame serialization |
| `struct` | stdlib | Byte-level encode / decode |
| `threading` | stdlib | CAN listener daemon thread |
| `collections.deque` | stdlib | Fixed-size circular data buffer |

---

## 9. Getting Started

### Installation
```bash
pip install -r requirements.txt
```

### Running (two terminals required)

**Terminal 1 — launch the analyzer first:**
```bash
python node_b_analyzer_gui.py
```

**Terminal 2 — launch the simulator:**
```bash
python node_a_sender.py
```

**Expected output — Terminal 2 (Node A):**
```
[15:08:21] ID:0x100 | 05 DC                  | Motor RPM       →  1500 RPM
[15:08:21] ID:0x101 | 02 F3                  | Temperature     →  75.5 °C
[15:08:21] ID:0x102 | 12 5C                  | Torque          →  47.00 N·m
[15:08:21] ID:0x103 | 15 8A                  | DC Voltage      →  554.2 V
[15:08:21] ID:0x104 | 08 34                  | Phase Current   →  20.84 A
```

**Expected output — GUI (Node B):**
- Left panel: colored hex log stream with decoded values
- Right panel: four live-updating subplots (RPM, Temperature, Torque, Current)
- Top bar: real-time KPI cards showing the latest values

---

## 10. Project File Structure

```
CANbus_Analyzer/
├── can_protocol.py          (~120 lines)
│   ├── CAN ID constants
│   ├── encode_*() functions  (5 signals)
│   ├── decode_frame() dispatcher
│   └── DecodedFrame dataclass
│
├── node_a_sender.py         (~195 lines)
│   ├── MotorSimulator class
│   │   ├── rpm()         → sine-wave profile
│   │   ├── temperature() → RPM-dependent thermal model
│   │   ├── torque()      → P = 7.5 kW constant power
│   │   ├── voltage()     → DC bus model
│   │   └── current()     → Kt motor constant
│   └── Rich terminal formatter
│
├── node_b_analyzer_gui.py   (~530 lines)
│   ├── DataStore           → thread-safe deque buffer
│   ├── can_listener_thread → daemon thread
│   └── CANAnalyzerApp
│       ├── KPI bar         → live numeric cards
│       ├── Log panel       → colored hex terminal
│       └── Graph panel     → 4-subplot matplotlib canvas
│
├── requirements.txt
├── README.md
└── PROJE_RAPORU.md          ← this file
```

---

## 11. GitHub & LinkedIn Presentation Tips

### Suggested Repository Description
```
Zero-hardware industrial CAN bus simulator & real-time decoder.
Two Python nodes communicate over a virtual network — hex frames decoded live into RPM, temperature, torque, and current.
```

### Screenshot Strategy
Ideal side-by-side screenshot:
- **Left half:** Node A terminal — scrolling green hex stream (Matrix effect)
- **Right half:** Node B GUI — 4 live graphs + KPI bar

### LinkedIn Post Template
> *Do you know what a motor drive is actually saying?*  
> `[0x100] 05 DC | [0x101] 02 F3` — that's the raw data.
>  
> I built a virtual CANbus network in Python where two separate processes exchange industrial-format packets over UDP multicast — just like real hardware. Every hex frame is decoded live and rendered as engineering data.
>  
> RPM: 1500 | Temperature: 75.5 °C | Torque: 47 N·m  
> Hardware cost: $0. Engineering depth: priceless.

### Technical Talking Points
- **Data Link Layer** level data processing
- **Big-Endian byte ordering** (J1939 / CANopen standard)
- **Thread-safe** producer–consumer data pipeline
- **Real-time** visualization at 200 ms latency
- **Physics-based** motor telemetry simulation

---

## 12. Roadmap

| Priority | Feature | Approach |
|----------|---------|---------|
| 🔴 High | Real USB-CAN adapter support | `interface='socketcan'` / `'kvaser'` / `'pcan'` |
| 🟡 Medium | Log recording & playback | ASC / CSV via `python-can` built-ins |
| 🟡 Medium | DBC file parsing | [`cantools`](https://github.com/eerimoq/cantools) |
| 🟢 Low | Web-based dashboard | FastAPI + WebSocket + Chart.js |
| 🟢 Low | Anomaly detection | `IsolationForest` (scikit-learn) |

---

## 13. License

MIT — free to use, modify, and distribute with attribution.

---

*"Turning noisy industrial bus traffic into actionable engineering data."*

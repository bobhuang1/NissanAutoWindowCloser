# NissanAutoWindowCloser — PCB design (integrated board)

Single-board version of the Arduino Nano + MCP2515 module prototype: everything
on one PCB that plugs into the car's **OBD-II port**.

Uses the *same silicon* as the prototype — ATmega328P (the chips used on the
Arduino UNO), MCP2515, and MCP2551 — so the firmware in the repo root compiles
and runs unchanged. Only the crystal note differs: the integrated board gives
the MCP2515 its own 16 MHz oscillator, so set (`NissanAutoWindowCloser.ino`):

```cpp
#define CAN_CLOCK  MCP_16MHZ    // integrated PCB (this hardware)
```

(If you still use the plug-in MCP2515 module with an 8 MHz crystal, wire K1 and
pick `MCP_8MHZ` instead.)

```
┌────────────────────────────┐         OBD-II harness
│  Car 12 V battery (16)     │────────┐
│  GND        (4, 5)         │────────┤
│  CAN-H      (6)            │────────┤    J1 (J1962, male)
│  CAN-L      (14)           │────────┘
│  [ACC via optional pigtail]│────────── J4 (JST-PH 2-pin)  ──▸ 50 mV divider → A0
└────────────────────────────┘
          │
      TVS / CM-choke / fuse
          ▼
   Regulator 12 V → 5 V (LM2940-5.0) ── 5 V rail
          ▼
   ┌──────────────┐   SPI   ┌─────────────┐  CAN TX/RX  ┌────────────┐
   │  ATmega328P  │◄───────►│   MCP2515   │◄───────────►│  MCP2551   │── CANH/CANL
   │  16 MHz osc  │         │  16 MHz osc │             │ transceiver│── (+120 Ω opt.)
   └──────────────┘         └─────────────┘             └────────────┘
```
The firmware (root of this repo) needs no change; only `CAN_CLOCK` as above.

## Board features

- **Plug-in OBD-II**: male J1962 (ELM327-dongle style) or a 2×8 0.1" header
  for a flying-lead OBD cable — switch with K2.
- **Power** from OBD pin 16 (always-on battery). Reverse-polarity diode, 500 mA
  resettable fuse, transient TVS, then LM2940-5.0 LDO → clean 5 V for the MCU,
  MCP2515 and MCP2551.
- **ACC sensing**: OBD does not expose ACC. Optional 2-pin pigtail (J4) brings
  ACC into the onboard 10 k / 4.7 k divider → A0, or you can run the firmware's
  "door-lock CAN detection" only and skip the pigtail completely.
- **CAN interface**: CM-choke + PESD1CAN TVS + optional split termination
  (K1 / 60 Ω + 60 Ω + 4.7 nF). See `layout.md` regarding termination at an OBD
  tap.
- **Programming**: standard 6-pin ICSP header + 6-pin UART header (FTDI), LED
  on ACT, reset button.

## Design files in this folder

| File | Contents |
| --- | --- |
| `schematic.md` | Human-readable schematic (ASCII) with net labels |
| `netlist.md`  | Complete connection table per net |
| `bom.md`      | Bill of materials with footprints and notes |
| `layout.md`   | 2-layer layout guidance, placement, EMI notes |

The schematic is net-exact: it can be transcribed 1:1 into KiCad/EasyEDA.
There is intentionally no vendor-specific CAD file — the netlist and BOM are
portable to any tool you already use.

## Build order

1. Transcribe `schematic.md` + `netlist.md` into your EDA.
2. Place per `layout.md`, route, emit Gerbers, order from a fab.
3. Assemble from `bom.md`.
4. Flash the firmware (ICSP or UART header) — same `.ino` as the breadboard
   version.
5. Re-engineer the constants (`LOCK_FRAME_*`, `DOOR_FRAME_*`, `HAZARD_FRAME_*`,
   `WINDOWS_UP[]`/`WINDOWS_DOWN[]`) for the vehicle before trusting it on an
   actual car. See root README.

> ⚠️ Same safety warnings as the firmware README apply — especially pinch
> protection and correct CAN termination. Tapping OBD-II pin 6/14 is the
> standard diagnostic pair on this car, but always verify with a sniffer
> first.
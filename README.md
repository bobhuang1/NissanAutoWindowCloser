# NissanAutoWindowCloser

Automatic window closer and lock convenience module for a **Nissan** (built for a
**Sylphy**) on an **Arduino Nano** with an **MCP2515 CAN bus module**.

> ⚠️ **Read this before plugging it into a car.** The *receive* values come from a
> published Nissan database and the *commands* go through the BCM's standard
> diagnostic service, but they were measured on **other** Nissan models. The one
> thing nobody publishes is the window motor command frame, so that part is
> deliberately disabled (`WINDOW_FRAMES_VERIFIED = 0`) — the sketch will not put
> invented frames on your bus. Everything vehicle-specific sits in the CONFIG
> block at the top of the sketch.

## Where the values come from

| Purpose | Source |
| --- | --- |
| Door / lock / ignition status, headlight status, vehicle speed | [dalathegreat/leaf_can_bus_messages](https://github.com/dalathegreat/leaf_can_bus_messages) — `CAR-can_AZE0.dbc` |
| BCM diagnostic request / response IDs (0x745 / 0x765, IPDM-E/R 0x74D / 0x76D) | poll table in the same repository |
| BCM `$30` local-ID map used to lock, unlock, beep, drive the beams and hazards | [balrog-kun/nissan-qashqai-can-info](https://github.com/balrog-kun/nissan-qashqai-can-info), [projectbytes (370Z CAN control)](https://projectbytes.wordpress.com/2015/07/08/nissan-370z-can-control/) |
| MCP2515 driver | [`coryjfowler/mcp_can`](https://github.com/coryjfowler/MCP_CAN_lib), **1.5.x** API |

### Received frames

| ID | Byte | Bits | Meaning |
| --- | --- | --- | --- |
| `0x60D` `x60D_BCM_GeneralStatus7` | 0 | `DOOR_MASK` = `0x1F` | 0x01 trunk, 0x02 rear right, 0x04 rear left, 0x08 driver, 0x10 passenger |
| | 1 | `0x60` >> 5 | ignition: 0 off, 1 ACC, 2 start, 3 on |
| | 2 | `LOCKSTAT_MASK` = `0x30` | 0x10 driver locked, 0x20 other doors locked |
| `0x284` (ABS) | 4/5 | little-endian | vehicle speed, 0.01 km/h per count |
| `0x625` `x625_USM_GeneralStatus` | 1 | `0x20`+`0x40` | headlight read-back (`0x60` beams, `0x68` + fog) |

The LEAF database and the Qashqai measurements agree on the front-door bits
(`0x08` / `0x10`) but disagree about their neighbours. If `DOOR_MASK = 0x1F`
ever misfires, narrow it to `0x18` (front doors only) — the comment in the sketch
says so at the definition.

If the ignition field does not track your car, the Qashqai/Sentra/370Z encoding
for the same byte is `0x0C` >> 2. Both alternatives are documented at `IGN_MASK`.

### Transmitted frames

Nissan body modules do not expose a bare "do this" command. The sketch uses the
standard UDS services inside the extended diagnostic session, on `BCM_CMD_ID`:

```
02 10 C0 00 00 00 00 00              enter diagnostic session
04 30 <LID> <func> <value> 00 00     InputOutputControlByIdentifier
02 10 81 00 00 00 00 00              back to default session
```

| Service | LID | func | value | Used for |
| --- | --- | --- | --- | --- |
| `0x30` | `0x07` | `0x00` | 1 / 2 | lock / unlock all doors |
| `0x30` | `0x11` | `0x20` | 1 / 0 | key beep (the horn the fob uses), on / off |
| `0x30` | `0x3B` | `0x20` | 1 / 0 | low beam on / off |
| `0x30` | `0x4C` | `0x20` | 0 / 1 / 2 | indicators off / right / left |

`func 0x20` is a latched state that the BCM times out after about five seconds,
which is what the horn and hazard state machines rely on. LID `0x4C` drives one
side at a time, so hazards are produced by alternating 0x01 and 0x02 every
`HAZARD_TOGGLE_MS`.

The session is opened when needed and reused for `BCM_REUSE_MS`, so a burst (the
double honk, the hazard alternation) does not re-send `$10 C0` per frame. The
BCM falls back to the default session on its own after ~5 s; serial `x` returns it
immediately.

## Features

Each behaviour has its own `TRIGGER_*` switch in the CONFIG block.

1. **Close windows when the car is switched off** — `TRIGGER_ACC_OFF`. Ignition
   comes from the BCM's own field on `0x60D` (`ACC_SOURCE_CAN`), debounced by
   `CAN_IGN_DEBOUNCE_MS`, with the ACC divider on `A0` as the fallback until the
   first `0x60D` arrives. After `CLOSE_DELAY_MS` one frame per window leaves the
   `WINDOWS_UP[]` table, `CLOSE_SPACING_MS` apart.
2. **Close windows on a lock event** — `TRIGGER_DOOR_LOCK`: the lock level on
   `0x60D` byte 2 rises while ACC is off.
3. **Horn confirmation** — `TRIGGER_HORN_ON_LOCK` honks once for
   `HORN_PULSE_MS`; `TRIGGER_HORN_ON_UNLOCK` honks `HORN_PULSES_UNLOCK` (2) times
   with `HORN_GAP_MS` between. Non-blocking, and the horn is forced off if the
   master switch opens. The first lock sample after boot is adopted without
   reacting, so powering the module up in a locked car stays quiet.
4. **Automatic re-lock** — `TRIGGER_AUTO_RELOCK`: an unlock arms a
   `RELOCK_DELAY_MS` (60 s) timer. Opening a door, turning ACC on, locking by
   hand, or flipping the master switch cancels it. `RELOCK_ARM_AT_BOOT = 1` also
   arms it when the car is already unlocked when the module boots.
5. **Hazards on with any open door** — `TRIGGER_HAZARD_ON_DOOR`, debounced by
   `DOOR_DEBOUNCE_MS`, `HAZARD_ONLY_WHEN_PARKED` keeps it to a parked car.
6. **Roll windows down on a triple unlock** — `TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK`,
   **off by default**: three unlock events inside `TRIPLE_UNLOCK_WINDOW_MS` (3 s)
   while ACC is off. It counts lock-status *edges*, and pressing unlock repeatedly
   on the fob gives only one edge (the car is already unlocked; Nissan's two-stage
   unlock still reports "locked" after the first press), so it cannot fire until it
   counts the fob's unlock button frame instead. Sniff that frame first.
7. **Hazards on a hard deceleration** — `TRIGGER_BRAKE_WARNING_LIGHTS`, **off by
   default** (see Safety): speed from
   `0x284`; if the drop rate reaches `DECEL_THRESHOLD_KMPHS_X10` (25 km/h per
   second) above `MIN_SPEED_KMH_X10` (20 km/h), the hazards flash for
   `BRAKE_HAZARD_MS`. Sample gaps outside `DECEL_MIN_DT_MS`..`DECEL_MAX_DT_MS` are
   ignored and `BRAKE_COOLDOWN_MS` stops echo storms.
8. **Headlights while auto-closing** — `LIGHTS_WHILE_CLOSE` beams on, window
   frames, then beams off after `HEADLIGHTS_HOLD_MS`. Only active once
   `WINDOW_FRAMES_VERIFIED = 1`.
9. **Cooldown** — `EVENT_COOLDOWN_MS` stops a close/roll-down echo from repeating.
   Hazards are demand-driven (door / brake / manual bits OR'd together) and
   reconcile every `loop()`, so sources never fight over the indicator command.

### Serial commands (115200 baud)

`c` close · `o` roll down · `l` lock · `u` unlock · `p` one honk · `n` two honks ·
`d`/`f` hazards on/off · `r` force a re-lock now · `x` close the diagnostic
session · `a` print live ACC millivolts · `m` CAN monitor (needs
`TRIGGER_CAN_MONITOR = 1`) · `s` dump state · `?` help.

The status LED on D9 ticks on every frame the sketch transmits.

## Wiring (MCP2515 v1 module ⇄ Nano)

| MCP2515 | Arduino Nano |
| --- | --- |
| VCC | 5V |
| GND | GND |
| CS | D10 |
| SCK | D13 |
| MOSI | D11 |
| MISO | D12 |
| INT | D2 (optional — receive is polled) |

ACC sense: ACC wire (up to ~14.4 V) through a resistive divider to `A0`, e.g.
**10 kΩ (bus side) + 4.7 kΩ (to GND)** gives ≈ 3.8–4.4 V — safe for the Nano.
Set `ACC_OFF_MV` between the "car off" and "car on" readings. The integrated
PCB's J5 shorts `A1` to GND and is the master disable
(`MASTER_ENABLE_PIN`): every automatic feature pauses, an active horn is silenced,
manual serial commands still work.

## Build & upload

**Arduino IDE** — install the **mcp_can** library (Library Manager → "mcp_can",
coryjfowler). The sketch targets the **1.5.x** API, which `platformio.ini` pins.
Reference tutorials: how2electronics.com MCP2515 module walkthrough and
lastminuteengineers.com MCP2515 + Arduino tutorial. Board = Arduino Nano, upload.

**PlatformIO**

```powershell
pio run -e nanoatmega328 -t upload
```

> Do not bump `coryjfowler/mcp_can` to 2.x without editing the sketch: 2.x removed
> `readMsgBufID()` in favour of `readMsgBuf()` and renamed the mode constants to
> `MCP_MODE_*`, plus `begin()` now always takes the ID mode first.

## CAN bus settings

| Setting | Value | Where |
| --- | --- | --- |
| Bit rate | `CAN_500KBPS` | `CAN_SPEED` — Nissan body CAN, **verify with a sniffer** |
| ID mode | `MCP_STDEXT` | `begin()` first argument — accept 11 and 29 bit |
| Controller clock | `MCP_16MHZ` | `CAN_CLOCK` — integrated PCB uses 16 MHz; plug-in modules are often 8 MHz → `MCP_8MHZ` |
| Transceiver | CANH/CANL to the car's body bus, plus a common GND | hardware |

## Find the real window frame (do this first)

The window motors hang off a **LIN** bus with the BCM as master, so a command has
to reach the BCM — and no public database lists it. Guessing is genuinely
dangerous: `0x180` is a live VCM torque message on real Nissans, which is exactly
why the tables are disabled.

1. Flash an MCP2515 **sniffer** onto the Nano (mcp_can ships examples) instead of
   this sketch.
2. Operate each window switch **up** and **down** and log which IDs appear and
   which payload bytes change. Press the driver-door lock, open each door and the
   tailgate, press the hazard and headlight stalks, and log a hard brake for the
   speed frame.
3. Fill in `WINDOWS_UP[]` / `WINDOWS_DOWN[]` with the ID, extended flag and
   payload, then set `WINDOW_FRAMES_VERIFIED = 1`.
4. Re-verify the status values in the same log: if the door or lock bits do not
   behave as the table above says, adjust `DOOR_MASK` / `LOCKSTAT_MASK` (or the
   ignition `IGN_MASK` / `IGN_SHIFT`) to what you actually saw.

## Safety

- **Brake hazards act while the car is moving.** They open the BCM's extended
  diagnostic session and drive the indicators through `$30` IO-control at speed;
  while that override is active the BCM may ignore the driver's own stalk. LID
  `0x4C` drives one side at a time, so following traffic sees left/right
  alternating turn signals, not a hazard flash. This is why the feature ships
  disabled - only enable it after verifying both effects on your car.

- **Auto-close is the dangerous one**: a hand, arm or child in the way is worse
  with a motor, not better. Keep the master kill-switch within reach and add a
  physical inhibit (window pinch sensor or the J5 disable) before relying on it.
- **Auto roll-down** (triple unlock) opens the car to rain and theft. The three
  unlocks are counted from a lock-status *edge*, so verify the burst gate on your
  fob before leaving it enabled.
- **The horn** is a real $30 command to the BCM. If your car beeps for other
  reasons, set `TRIGGER_HORN_ON_LOCK` / `TRIGGER_HORN_ON_UNLOCK` to 0 — the rest
  of the sketch keeps working.
- **Auto re-lock** can lock a car with a bag or a passenger's keys in it. It is
  cancelled by a door opening and by ACC going on, but confirm that behaviour on
  your car; `RELOCK_ARM_AT_BOOT = 0` (the default) means it never arms just
  because the module booted.
- Tapping the CAN bus: prefer a T-junction and leave terminations alone; never
  cut or loop the bus wires.
- Verify the divider and the transceiver before powering anything from the car's
  rails.


## License

This project is free software, released under the **GNU General Public License v3.0**. You may redistribute and/or modify it under those terms; see [LICENSE.md](LICENSE.md) for the full text.

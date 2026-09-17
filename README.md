# NissanAutoWindowCloser

Automatic window closer for a **Nissan Sylphy** on an **Arduino Nano** with an
**MCP2515 CAN bus module**.

> ⚠️ **Sample / proof-of-concept code.** CAN frame IDs and payload bytes are
> **placeholders**. Find the real values on *your* car before trusting this on
> the actual bus. Everything vehicle-specific is isolated at the top of the
> sketch (CONFIG section) so retuning is a constant edit, not a rewrite.

## How it works

Four configurable behaviors, each with its own `TRIGGER_*` switch:

1. **Close windows when the car switches off** — two detection modes:
   - `TRIGGER_ACC_OFF`: ACC voltage (12 V → divider → A0) drops and stays below
     `ACC_OFF_MV` for `ACC_OFF_DEBOUNCE_MS`.
   - `TRIGGER_DOOR_LOCK`: a CAN frame matching `LOCK_FRAME_ID` + bit pattern
     `LOCK_BYTE / LOCK_MASK / LOCK_VALUE` arrives **while ACC is off**.
   After `CLOSE_DELAY_MS`, one "close" frame per window shoots off the
   `WINDOWS_UP[]` table (`SEND_RETRIES` each, `CLOSE_SPACING_MS` apart).
2. **Hazards on with any open door** — `TRIGGER_HAZARD_ON_DOOR` watches the
   door/tailgate status frame (`DOOR_FRAME_ID`, bitfield `DOOR_BITS` across
   `DOOR_BYTE0/1`, `DOOR_DEBOUNCE_MS`). Any door open →
   `setHazards(true)` sends the `HAZARD_FRAME_ID` "on" frame; all closed sends
   "off". While `HAZARD_ONLY_WHEN_PARKED = 1` it only reacts with ACC off, and
   catches doors already open when ACC drops.
3. **Roll windows down on a triple unlock** — `TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK`:
   three `UNLOCK_VALUE` events on the lock frame inside
   `TRIPLE_UNLOCK_WINDOW_MS` (3 s, while ACC is off) roll every window down via
   `WINDOWS_DOWN[]` after `ROLL_DOWN_DELAY_MS`. A fresh lock event or an ACC-on
   resets the burst counter.
4. **Cooldown** — `EVENT_COOLDOWN_MS` stops a close/roll-down echo from
   repeating every identical bus frame. Hazard toggling is independent.

USB serial doubles as a test rig before the car install: `c` closes windows,
`o` rolls them down, `d`/`f` hazards on/off, `a` prints live ACC millivolts,
`s` dumps the state machine, `?` prints help.

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

ACC sense: ACC wire (up to ~14.4 V) through a resistive divider to A0, e.g.
**10 kΩ (bus side) + 4.7 kΩ (to GND)** gives ≈ 3.8–4.4 V — safe for the Nano.
Set `ACC_OFF_MV` to sit between the "car off" and "car on" divider readings.

## Build & upload

**Arduino IDE**
1. Install the **mcp_can** library (Library Manager → search "mcp_can",
   coryjfowler/MCP_CAN_lib). The two reference tutorials used for this sample:
   - how2electronics.com/interfacing-mcp2515-can-bus-module-with-arduino
   - lastminuteengineers.com/mcp2515-can-module-arduino-tutorial
2. Open `NissanAutoWindowCloser.ino`, board = Arduino Nano, upload.

**PlatformIO**
```powershell
pio run -e nanoatmega328 -t upload
```
(`platformio.ini` already pins the `mcp_can` dependency.)

## CAN bus settings

| Setting | Value | Where |
| --- | --- | --- |
| Bit rate | `CAN_500KBPS` | `CAN_SPEED` — Nissan body CAN, **verify with a sniffer** |
| Controller clock | `MCP_16MHZ` | `CAN_CLOCK` — integrated PCB uses 16 MHz; the plug-in module often 8 MHz → `MCP_8MHZ` |
| Transceiver wiring | CANH/CANL of the module to the car's body bus + a common GND | hardware |

## Find the real frames (do this first)

1. Flash a **MCP2515 CAN sniffer** (mcp_can ships `CAN_SendReceive`/sniffer
   examples) onto the Nano instead of this sketch.
2. With the car running, operate each window switch **up** and watch which IDs
   appear on the bus; do the same for the driver-door lock / key-fob lock. Then
   open each door and the tailgate to log the door-status frame bits.
3. Note the ID, extended bit, and which payload bytes change.
4. Fill in `LOCK_FRAME_*`, `DOOR_FRAME_*`, `HAZARD_FRAME_*` and the
   `WINDOWS_UP[]` / `WINDOWS_DOWN[]` tables accordingly.

On many Nissans the windows are actually driven over **LIN** with the body
computer (BCM) as master — in that case send the *BCM's* "roll windows up"
request frame on the **body** CAN instead of talking to a window motor
directly. A CAN prototype of this kind proves the trigger logic; make it road
safe by swapping to the genuine command once sniffed.

## Safety

- Auto-close: a hand/arm/child in the way is worse with a motor, not better.
  Add a physical interrupt (window pinch sensor or a "disable" switch) before
  relying on it.
- Auto roll-down (triple unlock): it also opens the car to rain/theft — keep
  `TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK` off until the unlock frame is verified
  and confirm the burst gate behaves on *your* key fob.
- Tapping the CAN bus of a car: prefer a T-junction and keep terminations as
  they are; never loop or cut the bus wires.
- Verify both divider and CAN transceiver before powering anything from the
  car's rails.
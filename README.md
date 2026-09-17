# NissanAutoWindowCloser

Automatic window closer for a **Nissan Sylphy** on an **Arduino Nano** with an
**MCP2515 CAN bus module**.

> ⚠️ **Sample / proof-of-concept code.** CAN frame IDs and payload bytes are
> **placeholders**. Find the real values on *your* car before trusting this on
> the actual bus. Everything vehicle-specific is isolated at the top of the
> sketch (CONFIG section) so retuning is a constant edit, not a rewrite.

## How it works

1. **Detect "car just switched off"** — two modes, both configurable:
   - `TRIGGER_ACC_OFF`: ACC voltage (12 V → divider → A0) drops and stays below
     `ACC_OFF_MV` for `ACC_OFF_DEBOUNCE_MS`.
   - `TRIGGER_DOOR_LOCK`: a CAN frame matching `LOCK_FRAME_ID` + bit pattern
     `LOCK_BYTE/ LOCK_MASK / LOCK_VALUE` arrives **while ACC is off**.
2. After `CLOSE_DELAY_MS`, the sketch sends one "close" frame per window from
   the `WINDOWS[]` table (4 x `sendMsgBuf`, `SEND_RETRIES` each, spaced by
   `CLOSE_SPACING_MS`).
3. A cooldown (`EVENT_COOLDOWN_MS`) stops repeated lock frames from re-firing.

USB serial doubles as a test rig before the car install: `c` closes all
windows, `a` prints live ACC millivolts, `?` prints help.

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
| Controller clock | `MCP_8MHZ` | `CAN_CLOCK` — match the crystal on *your* module (or `MCP_16MHZ`) |
| Transceiver wiring | CANH/CANL of the module to the car's body bus + a common GND | hardware |

## Find the real frames (do this first)

1. Flash a **MCP2515 CAN sniffer** (mcp_can ships `CAN_SendReceive`/sniffer
   examples) onto the Nano instead of this sketch.
2. With the car running, operate each window switch **up** and watch which IDs
   appear on the bus; then do the same for the driver-door lock or key-fob lock.
3. Note the ID, extended bit, and which payload bytes change.
4. Fill in `LOCK_FRAME_*` and the `WINDOWS[]` table accordingly.

On many Nissans the windows are actually driven over **LIN** with the body
computer (BCM) as master — in that case send the *BCM's* "roll windows up"
request frame on the **body** CAN instead of talking to a window motor
directly. A CAN prototype of this kind proves the trigger logic; make it road
safe by swapping to the genuine command once sniffed.

## Safety

- This closes windows automatically — a hand/arm/child in the way is worse with
  a motor, not better. Add a physical interrupt (window pinch sensor or a
  "disable" switch) before relying on it.
- Tapping the CAN bus of a car: prefer a T-junction and keep terminations as
  they are; never loop or cut the bus wires.
- Verify both divider and CAN transceiver before powering anything from the
  car's rails.
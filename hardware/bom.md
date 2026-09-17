# BOM — NissanAutoWindowCloser PCB

Cost-optimized, hobby-friendly, mostly **through-hole** (UNO heritage). All
parts are generic and interchangeable. Prices approximate.

| Ref | Part | Footprint | Qty | Purpose / note |
| --- | --- | --- | --- | --- |
| U1 | ATMEGA328P-PU | DIP-28 | 1 | MCU, same as Arduino UNO (optiboot optional — ICSP flash works bare) |
| U2 | MCP2515-I/P | DIP-18 | 1 | CAN controller, SPI |
| U3 | MCP2551-I/P | DIP-8 | 1 | CAN transceiver 5 V (MCP2562 acceptable) |
| U4 | LM2940-5.0 | TO-220 | 1 | 12→5 V LDO, 1 A, low dropout; add TO-220 heatsink pad |
| Y1 | 16 MHz HC-49/S crystal | HC49S | 1 | MCU clock (UNO uses 16 MHz) |
| Y2 | 16 MHz HC-49/S crystal | HC49S | 1 | MCP2515 clock — must match `CAN_CLOCK = MCP_16MHZ` |
| D1 | SS34 Schottky | DO-214/SMA | 1 | reverse-polarity protection on 12 V |
| D2 | SMBJ24A TVS | SMB / DO-214AA | 1 | clamp VBAT transients (load-dump) |
| D3 | PESD1CAN | SOT-23 | 1 | ±24 kV CAN bus ESD clamp CANH/CANL→GND |
| L1 | ACT45B-101-2P | 1812 | 1 | common-mode choke on CAN (ACM1211/ACM3225 ok) |
| L2 | Ferrite bead 1 A (e.g. BLM31AG102) | 1206 | 1 | 12 V input filtering |
| F1 | PTC 500 mA 16 V (e.g. MF-R050) | radial | 1 | resettable input fuse |
| R1 | 10 kΩ 1/4 W | 0207 | 1 | MCU RESET pull-up |
| R2 | 1 kΩ 1/4 W | 0207 | 1 | LED1 (PWR) current limit |
| R3 | 330 Ω 1/4 W | 0207 | 1 | LED2 (ACT) current limit |
| R4 | 120 Ω 1/4 W 1% | 0207 | 1 | CAN termination (option A, K1) |
| R5 | 60 Ω 1/4 W 1% | 0207 | 1 | split termination CANH side (opt B, K1) |
| R6 | 60 Ω 1/4 W 1% | 0207 | 1 | split termination CANL side (opt B, K1) |
| R7 | 10 kΩ 1/4 W | 0207 | 1 | U2 RESET pull-up |
| R8 | 10 kΩ 1/4 W | 0207 | 1 | MCP2551 RS → GND (high-speed) |
| R9 | 10 kΩ 1/4 W | 0207 | 1 | ACC divider, bus side ~12 V |
| R10 | 4.7 kΩ 1/4 W | 0207 | 1 | ACC divider, to GND (A0 ≈ 3.8–4.4 V with 12–14.4 V in) |
| C1 | 0.22 µF/50 V ceramic | 1206 | 1 | LM2940 input |
| C2 | 22 µF/16 V electrolytic | 5 mm | 1 | LM2940 output (ESR ≤ 3 Ω — short-lead elec) |
| C3,C4,C5,C6,C7 | 100 nF/50 V ceramic | 0805 | 5 | rail + each IC decoupling |
| C8,C9 | 22 pF/50 V C0G | 0805 | 2 | MCU crystal load |
| C10,C11 | 22 pF/50 V C0G | 0805 | 2 | MCP2515 crystal load |
| C12 | 100 nF/50 V ceramic | 0805 | 1 | MCU RESET filter |
| C14 | 4.7 nF/50 V ceramic | 0805 | 1 | split-termination midpoint to GND (opt B) |
| C20 | 100 µF/25 V electrolytic | 5 mm | 1 | VBAT bulk before LDO |
| C60 | 10 µF/16 V ceramic/elec | 1206 | 1 | +5 V bulk near MCU |
| C61 | 10 µF/16 V ceramic/elec | 1206 | 1 | +5 V bulk near MCP2515 |
| J1 | J1962 male 16-way (e.g. auto aftermarket OBD pigtail) | 2×8 | 1 | board-side OBD connector (or through K2) |
| J2 | Header 2×3 0.1" (ICSP) | 2×3 | 1 | AVR ISP programming |
| J3 | Header 1×6 0.1" (FTDI row) | 1×6 | 1 | UART: DTR RX TX VCC CTS GND |
| J4 | JST-PH 2-pin (B2B-PH) | 2-pin | 1 | optional ACC pigtail |
| K1 | Jumper 2×2 0.1" + shunt | 2×2 | 1 | select/kill CAN termination (default OPEN) |
| K2 | 0 Ω (tie on same net w/ J1) | 1206 | 1 | selects J1 vs flying-lead header (leave 0 Ω) |
| LED1 | Red 5 mm LED | 5 mm | 1 | power indicator |
| LED2 | Green 5 mm LED | 5 mm | 1 | ACT indicator (active low) |
| SW1 | Tactile switch 6×6 mm | — | 1 | reset |

## Notes / variants

- **MCP2551 vs MCP2562**: both fine at 5 V / 500 kbps; MCP2562 needs no RS
  pull (RS tied low internally), R8 then DNP.
- **LM2940 vs buck**: LDO wastes ~7 V×0.05 A ≈ 0.35 W at idle — fine for a
  dongle. For less quiescent draw (always-on OBD), a buck like TPS54331 drops
  idle current; footprint (SOT-23) is a board diff, not a BOM swap.
- **LC vs LDO input**: C1 is required for stability; keep it and C2 close to
  U4.
- **Fuse**: F1 guards the 12 V rail; keep it outside the LDO loop.
- Numbers are per one board; buy the usual breadboard extras.
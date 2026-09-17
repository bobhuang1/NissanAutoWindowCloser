# Schematic — NissanAutoWindowCloser PCB

Net-exact ASCII schematic. Transcribe straight into any EDA (KiCad, EasyEDA).
Net names in `` are transport labels; every node text in the figures must be a
real net. All logic is 5 V (ATmega328P, MCP2515, MCP2551 share one rail).

## Conventions

- Signal polarity of LEDs: anode → 5 V, cathode → resistor → pin ⇒ **LED ON = pin LOW**.
- Unused MCP2515 GPIOs (RX0BF, RX1BF, TX0RTS..TX2RTS) → NC.
- MCP2551 `RS` → 10 kΩ to GND (high-speed mode).
- Jumper K1 closes = **120 Ω termination fitted**; open = not fitted.
- U4 (LM2940-5.0) is TO-220: 1=IN, 2=GND, 3=OUT.

---

## Block 1 — Power input & OBD-II connector (J1)

```
    12V_RAW ──► SS34 ────► F1(500mA PTC) ──► VBAT ──► SMBJ24A(TVS) ──► GND
   (J1-16)    D1 reverse                     │         D2
              protect                       L2 ferrite bead
                                             │         C20 100uF/25V ──► GND
                                             ▼
                                    ┌────────┴─────────┐
                               IN(1)│    U4 LM2940-5   │OUT(3)
                                    └────────┬─────────┘
                                        C1 0.22uF│          C2 22uF/16V
                                        IN→GND   │          OUT→GND (required ESR cap)
                                             ▼
                                            +5V ──► C3 100n ──► GND
```

J1 pinout (J1962, male — plugs into the car's OBD-II socket or into a cable):

| Pin | Net | Pin | Net |
| --- | --- | --- | --- |
| 1   | — (reserved)  | 9  | — |
| 2   | —             | 10 | — |
| 3   | —             | 11 | — |
| 4   | GND (chassis) | 12 | — |
| 5   | GND (signal)  | 13 | — |
| 6   | CANH          | 14 | CANL |
| 7   | —             | 15 | — |
| 8   | —             | 16 | 12V_RAW |

---

## Block 2 — CAN transceiver (U3 MCP2551) & bus protection

```
  J1-6  CANH ─┬── D3 PESD1CAN ───┬──► L1 (CM-choke, ACT45B/ACM1211) ──► U3/7 CANH
  J1-14 CANL ─┴──────────────────┴──► L1 ──────────────────────────────► U3/6 CANL
                          D3 TVS: CANH→GND + CANL→GND

  Optional bus termination, selected by jumper K1 (default OPEN — the car bus is
  usually already end-terminated by the ECU; fit only if your tap is far from
  both ends):
    A) 120R R4 across CANH–CANL   (through K1)
    B) split: R5 60R CANH→SER  +  R6 60R CANL→SER  +  C14 4.7nF SER→GND

  U3 = MCP2551-I/P (PDIP-8):
    1 TXD  ◄── U2/1 TXCAN (CAN_TX)
    2 VSS  ──► GND
    3 VDD  ──► +5V          (C7 100n → GND at the pin)
    4 RXD  ──► U2/2 RXCAN (CAN_RX)
    5 VREF     NC
    6 CANL ──► L1 (above)
    7 CANH ──► L1 (above)
    8 RS   ──► R8 10k ──► GND   (slope-control: high-speed mode)
```

---

## Block 3 — CAN controller (U2 MCP2515-I/P, PDIP-18)

```
  OSC:  U2/8 OSC1 ── Y2 16 MHz ── U2/7 OSC2
            ├── C10 22pF ──► GND
            └── C11 22pF ──► GND

  U2 = MCP2515-I/P (PDIP-18):
    1  TXCAN  ────► CAN_TX  (→ U3/1 TXD)
    2  RXCAN  ◄─── CAN_RX   (← U3/4 RXD)
    3  CLKOUT    NC
    4  TX0RTS    NC          5  TX1RTS    NC      6  TX2RTS    NC
    7  OSC2      (above)     8  OSC1      (above) 9  VSS  ──► GND
    10 RX0BF     NC          11 RX1BF     NC      12 VDD  ──► +5V (C6 100n + C61 10µF → GND)
    13 CS    ◄──── D10/PB2  (MCU SS)
    14 SCK   ◄──── D13/PB5
    15 SI    ◄──── D11/PB3  (MOSI → controller SI)
    16 SO    ────► D12/PB4  (SO → MCU MISO)
    17 INT   ────► D2/PD2   (optional; firmware polls)
    18 RESET ◄──── R7 10k ──► +5V  (pull-up, behaves like MCU reset pin)
```

---

## Block 4 — MCU (U1 ATmega328P-PU, DIP-28)

```
                                +5V
                                 │
                          R1 10k │   C12 100n ──► GND
                                 ├── 1 PC6/RESET ── SW1 (to GND)
   Y1 16MHz:  9 PB6/XTAL1 ─ C8 22pF ─► GND
              10 PB7/XTAL2 ─ C9 22pF ─► GND

   Serial  :  2 PD0/RXD ◄── J3 TXD  / UART header
              3 PD1/TXD ──► J3 RXD
   CAN_INT :  4 PD2/INT0 ◄── U2/17 INT      (optional; firmware polls)
   SPI     : 16 PB2/PB3/PB4 → see Block 3 (D10 CS, D11 MOSI, D12 MISO, D13 SCK)
   LED_ACT : 15 PB1 ◄─ R3 330R ◄─ cathode, anode → +5V
   ACC     : 23 PC0/A0 ◄── R9(10k) ◄── J4 ACC  ──┬
                     │                          R10 4.7k
                     └────────────────────────────┴── GND
              (divider node = ACC_SENSE; 12 V drove ~3.8–4.4 V → A0)

   Power   :  7,20 VCC (+5V) ── C4,C5 100n + C60 10uF
              8,22 GND; 21 AREF (NC); 24–28 PC1..PC5 (spare)
```

---

## Block 5 — Programming & status

```
   J2 ICSP-6 (2×3):  1 MISO  2 +5V         J3 UART-6 (row):   1 DTR
                     3 SCK   4 MOSI                           2 RXD(firm→PC)
                     5 RESET 6 GND                            3 TXD(firm→PC)
                                                               4 +5V
   LED1 PWR : +5V ── 1k R2 ── LED ── GND                      5 CTS
   LED2 ACT : +5V ── 330 R3 ── LED ── D9/PB1                   6 GND
```

---

## Parts reference (see bom.md for full detail)

U1 ATmega328P-PU · U2 MCP2515-I/P · U3 MCP2551-I/P · U4 LM2940-5.0 · Y1/Y2 16 MHz
D1 SS34 · D2 SMBJ24A · D3 PESD1CAN · L1 CM choke · L2 ferrite · F1 500 mA PTC
J1 J1962 · J2 ICSP · J3 UART · J4 ACC (JST-PH 2) · K1 terminate jumper · SW1
LED1/LED2 · Caps C1..C20 + C60/C61 · Resistors R1..R10 — full value list in
`bom.md`, connection-by-connection table in `netlist.md`.
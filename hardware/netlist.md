# Netlist — NissanAutoWindowCloser PCB

Every connection on the board, grouped by net. `►` marks a net origin; each row
after it is a joined node. All logic is 5 V. Reference = part/pin of the
schematic in `schematic.md`.

## Power

| Net | Nodes |
| --- | --- |
| `12V_RAW` | ► J1-16 · D1-A |
| `VBAT`    | ► D1-K · F1-1 · D2-K · L2-1 · C20+ |
| `VBAT_LC` | ► L2-2 · U4-1 (IN) · C1 |
| `+5V`     | ► U4-3 (OUT) · C2+ · C3 · C4 · C5 · C60+ · C6 · C61+ · C7 · R1 · R2 · R3 · R7 · U1-7 · U1-20 · U2-12 · U3-3 · J3-4 · J2-2 |
| `GND`     | ► J1-4 · J1-5 · U4-2 · D2-A · D3 · U3-2 · U1-8 · U1-22 · U2-9 · SW1 · R8 · R10 · C1− · C2− · C3 · C4 · C5 · C6 · C7 · C8 · C9 · C10 · C11 · C12 · C14− · C20− · C60− · C61− · J3-6 · J2-6 |

## Oscillators & RESET

| Net | Nodes |
| --- | --- |
| `XTAL1_MCU` | ► U1-9 · Y1-1 · C8 |
| `XTAL2_MCU` | ► U1-10 · Y1-2 · C9 |
| `OSC1_CAN`  | ► U2-8 · Y2-1 · C10 |
| `OSC2_CAN`  | ► U2-7 · Y2-2 · C11 |
| `RESET`     | ► U1-1 · R1 · C12 · SW1 · J2-5 |

## SPI (MCU ⇄ MCP2515)

| Net | Nodes |
| --- | --- |
| `SPI_CS`   | ► U1-16 (PB2/D10) · U2-13 |
| `SPI_MOSI` | ► U1-17 (PB3/D11) · U2-15 (SI) |
| `SPI_MISO` | ► U1-18 (PB4/D12) · U2-16 (SO) |
| `SPI_SCK`  | ► U1-19 (PB5/D13) · U2-14 |

## CAN digital

| Net | Nodes |
| --- | --- |
| `CAN_TX`  | ► U2-1 (TXCAN) · U3-1 (TXD) |
| `CAN_RX`  | ► U2-2 (RXCAN) · U3-4 (RXD) |
| `CAN_INT` | ► U2-17 · U1-4 (PD2/D2) — optional; firmware polls |
| `CAN_RS`  | ► U3-8 · R8 · GND |

## CAN bus (to J1962)

| Net | Nodes |
| --- | --- |
| `CANH`     | ► U3-7 · L1-1 · D3 · R4-1 · R5-1 · J1-6 |
| `CANL`     | ► U3-6 · L1-2 · D3 · R4-2 · R6-1 · J1-14 |
| `CAN_SER`  | ► R5-2 · R6-2 · C14 — split-termination midpoint (option B, K1) |

## ACC sense

| Net | Nodes |
| --- | --- |
| `ACC_IN`    | ► J4-1 · R9 |
| `ACC_SENSE` | ► R9 · R10 · U1-23 (PC0/A0) |

## Programming & status

| Net | Nodes |
| --- | --- |
| `UART_RX` | ► J3-2 · U1-3 (PD1/TXD) — FTDI row labels are *adapter* perspective, wires cross |
| `UART_TX` | ► J3-3 · U1-2 (PD0/RXD) — same note as above |
| `LED_PWR` | ► +5V · R2 · LED1 · GND |
| `LED_ACT` | ► +5V · R3 · LED2 · U1-15 (PB1/D9) — **LED ON = pin LOW** |

## Not connected (expected, check at DRC)

U2-3 CLKOUT · U2-4/5/6 (TX0/1/2RTS) · U2-10/11 (RX0/1BF) · U3-5 VREF ·
U1-21 AREF · U1-24..28 (PC1..PC5) · J1-1/2/3/7/8/9/10/11/12/13/15
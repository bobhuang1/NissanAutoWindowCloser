# PCB layout — NissanAutoWindowCloser

2-layer, 100 × 40 mm card with a **J1962 OBD plug on the short edge** (ELM327
dongle style). Layer stack: signal/tracks top, GND fills both, copper pour top
and bottom stitched with vias.

## Board outline & mounting

- 100 × 40 mm, 1.6 mm FR-4, 2 oz copper.
- J1 (J1962 male) hangs off the short edge so the whole board plugs into the
  car's OBD socket directly; K2 pads let you instead route a flying-lead OBD
  cable to a 2×8 header on the board.
- ICSP (J2) and UART (J3) on the same edge, accessible with the board plugged
  in via a short adapter cable.

## Placement order (EMI-aware)

1. **Bus side first**: J1 → D3 TVS → L1 CM-choke → U3 — this chain is the EMI
   containment zone; keep it as one straight line, no components in the middle.
2. **Power**: D1/F1/D2/L2 cluster near J1-16; U4 with C1, C2 as close as leads
   allow; put the LDO at the opposite end of the short axis from the CAN chain
   so 12 V noise doesn't couple into CAN.
3. **Controller**: U2 + Y2 + C10/C11; SPI traces short to U1.
4. **MCU**: U1 + Y1 + C8/C9 + decoupling; keep RESET/R1/C12 path tight.

## Routing rules

- **CANH/CANL**: differential pair to J1 pins 6/14, 120 Ω impedance target,
  edge-coupled, length-matched, keep 10 mm clear of 12 V traces and the LDO.
  Route through L1 in the official sequence J1→D3→L1→U3 (transceiver pins
  6/7 feed the choke, not the reverse).
- **All SPI** (CS/MOSI/MISO/SCK): short, ≤ 20 mm, no vias if possible; group
  them.
- Every IC VDD pin → 100 nF cap → via to GND pour, cap within 2 mm of the pin.
- Crystal traces: tadpole loop from XTAL pin → crystal → cap → GND via; keep
  the ground on the via side; nothing routed under the crystals.
- Analog ACC_SENSE: R9+R10 right at U1-23, keep away from D2's clamped 12 V
  spikes; add 100 nF (footprint C13 optional) at A0 if noise shows up.
- Master-enable J5: short trace PC1(A1) → R11 → +5V rail; place J5 on the same
  edge as J3/ICSP so it can be flicked with the board installed. Keep R11's GND
  via clear of the ACC divider.
- **GND**: unbroken pour both layers; vias every 5 mm near the bus/transceiver
  chain. Star the LDO GND back to the OBD GND pins 4/5.

## Termination decision (important)

A car bus is normally terminated **at the ECU** (both physical ends of the
trunk). A dongle hanging near the middle of the bus *should not add
termination*. Therefore:

- K1 default **open / not fitted**.
- Fit R4 (120 Ω) only for bench testing without a car attached (then the two
  OBD pins are your "bus").
- R5/R6/C14 split option only if your car genuinely lacks end termination — a
  sniffer/lab check will tell you.

## Silkscreen

- Net names next to each pad (matches `netlist.md`).
- `CANH` / `CANL` labels at J1.
- `J5=OFF` silkscreen marker at the disable jumper (shunt in = features off).
- LED polarity dots; U1 pin-1 dot; J2/J3 pin-order numbers.
- "NissanAutoWindowCloser v1" + fw date + safety note sticker area.

## Manufacturing note

Order from the fab with:

- 2 layers, 1.6 mm, ENIG or immersion-Sn (better for solderability of L1/TVS).
- Min trace/space 6/6 mil is plenty; keep 8/8 for hand soldering.
- Panelize if ordering < 5; remove V-scores if plugging into OBD stresses the
  connector edge.

## Verification checklist before sending to fab

- [ ] DRC: no unconnected pins except the NC list in `netlist.md`.
- [ ] J1 pin 6 → CANH chain, pin 14 → CANL chain (order verified at D3/L1).
- [ ] Reverse-polarity protection direction (D1 cathode toward F1).
- [ ] K1 shunt absent by default.
- [ ] LDO I/O caps fit within 5 mm of U4.
- [ ] LED2 active-low note matches firmware (`LED_INVERTED` style).
- [ ] 5 V rail curves from U4-3 reach U3-3 without crossing the CAN pair.
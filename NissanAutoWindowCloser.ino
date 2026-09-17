/*
 * NissanAutoWindowCloser  —  Arduino Nano + MCP2515 CAN module
 * ---------------------------------------------------------------------------------
 * Automatic window closer for a Nissan Sylphy (sample code).
 *
 * Mission:
 *   1. Detect "ACC power off"  (12V signal on a voltage divider, pin A0)
 *      and/or a "door lock" event sniffed on the CAN bus.
 *   2. When either happens, send CAN frames that command all 4 windows to close.
 *
 * WARNING — sample / proof-of-concept only.
 *   The CAN frame IDs and payload bytes below are PLACEHOLDERS. They are NOT
 *   verified Nissan Sylphy messages. Real per-model needs are found by
 *   sniffing the car's own CAN bus (see README "Find the real frames").
 *   All vehicle specifics are isolated in the CONFIG section so the sketch can
 *   be retuned without touching the logic.
 *
 * Library:  MCP_CAN (coryjfowler / Seeed lineage) — install from Library
 *           Manager ("mcp_can"), or PlatformIO dependency.
 *
 * Wiring (MCP2515 v1 module):
 *   module   Nano
 *   VCC   -> 5V
 *   GND   -> GND
 *   CS    -> D10
 *   SCK   -> D13
 *   MOSI  -> D11
 *   MISO  -> D12
 *   INT   -> D2    (optional; receive is polled, no interrupt needed)
 *
 *   ACC sense: ACC wire (12 V) through a divider (e.g. 10 k + 4.7 k) to A0.
 * ---------------------------------------------------------------------------------
 */

#include <SPI.h>
#include "mcp_can.h"

/* ============================== CONFIG =====================================
 * Everything you might need to change for another car / module lives here.
 * ==========================================================================*/

/* --- MCP2515 CAN controller ------------------------------------------------ */
#define SPI_CS_PIN        10
#define CAN_SPEED         CAN_500KBPS          // Nissan body CAN is 500 kbit/s (verify!)
#define CAN_CLOCK         MCP_16MHZ            // integrated PCB uses a 16 MHz crystal; the common plug-in module uses 8 MHz -> MCP_8MHZ

/* --- ACC / ignition sensing ------------------------------------------------- */
#define ACC_SENSE_PIN     A0                   // ACC 12 V via divider -> 0..5 V
#define ACC_OFF_MV        1200UL               // below this value ACC is considered OFF
#define ACC_OFF_DEBOUNCE_MS 400UL              // consecutive time below threshold -> OFF
#define CLOSE_DELAY_MS    1500UL               // delay AFTER the trigger before closing
#define CLOSE_SPACING_MS  250UL                // pause between window frames
#define SEND_RETRIES      3                    // retries per frame if transmission fails
#define EVENT_COOLDOWN_MS 10000UL              // minimum gap between automatic closes
#define AUTO_CLOSE_AT_BOOT 0                   // 1 = close even if ACC already OFF at boot

/* --- Event sources ---------------------------------------------------------- */
#define TRIGGER_ACC_OFF   1                    // close when ACC switches OFF
#define TRIGGER_DOOR_LOCK 1                    // close when a lock frame arrives while ACC OFF

/* --- Door lock event sniffed on the bus (placeholder values!) ----------------- */
#define LOCK_FRAME_ID     0x2A2UL              // placeholder: BCM door lock frame
#define LOCK_FRAME_EXT    0                    // 0 = standard ID, 1 = extended
#define LOCK_BYTE         1                    // byte index to inspect
#define LOCK_MASK         0x03                 // bit mask on that byte
#define LOCK_VALUE        0x01                 // 0x01 = "locked" (placeholder)

/* --- STATUS / debug ---------------------------------------------------------- */
#define STATUS_LED        9                    // integrated PCB LED2 -> PB1/D9 (NOT D13, that is SPI SCK!)
#define LED_ACTIVE_HIGH   0                    // PCB: LED anode to +5V -> shines when pin is LOW. UNO built-in LED on D13 would be 1.
#define ledOn()           digitalWrite(STATUS_LED, LED_ACTIVE_HIGH ? HIGH : LOW)
#define ledOff()          digitalWrite(STATUS_LED, LED_ACTIVE_HIGH ? LOW  : HIGH)

/* ==================== WINDOW "CLOSE" FRAME TABLE =============================
 * PLACEHOLDERS! For a real Sylphy these must be replaced with frames recorded
 * from the car's own bus while the windows actually close (see README).
 * `ext` marks an extended (29-bit) ID: 1 = extended, 0 = standard.
 * ============================================================================*/
typedef struct {
  const char *name;                 // window name for logging
  uint32_t    id;                   // CAN frame ID
  uint8_t     ext;                  // 0 standard / 1 extended
  uint8_t     dlc;                  // payload length (<= 8)
  uint8_t     data[8];              // payload bytes
} WinClose;

static const WinClose WINDOWS[] = {
  { "FL", 0x180UL, 0, 8, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00} }, // driver
  { "FR", 0x181UL, 0, 8, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00} }, // passenger
  { "RL", 0x182UL, 0, 8, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00} }, // rear left
  { "RR", 0x183UL, 0, 8, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00} }, // rear right
};
#define NUM_WINDOWS  (sizeof(WINDOWS) / sizeof(WINDOWS[0]))

/* ============================== STATE ======================================== */
MCP_CAN CAN0(SPI_CS_PIN);

static unsigned long _nextEventMs = 0;          // millis() gate for EVENT_COOLDOWN_MS
static unsigned long _closeAtMs   = 0;          // pending close timer
static const char  *_closeReason  = nullptr;    // reason for the pending close

static bool _accOn          = false;            // last stable ACC state
static unsigned long _accLowSinceMs = 0;        // when the ACC reading first went low

static uint8_t  _rxLen;
static uint8_t  _rxData[8];
static uint32_t _rxFrameId;

/* ================================ SETUP ====================================== */
void setup() {
  Serial.begin(115200);
  Serial.println(F("\r\n[NissanAutoWindowCloser] boot"));

  pinMode(STATUS_LED, OUTPUT);
  pinMode(ACC_SENSE_PIN, INPUT);

  // CAN controller on the bus (start-loopback is the library default -> force normal).
  if (CAN0.begin(CAN_SPEED, CAN_CLOCK) == CAN_OK) {
    CAN0.setMode(MCP_MODE_NORMAL);              // newer library versions default loopback
    Serial.println(F("CAN controller OK (normal mode)"));
  } else {
    Serial.println(F("FATAL: CAN controller init failed"));
    Serial.println(F("Hint: check CAN_CLOCK (8 vs 16 MHz) and wiring."));
    while (1) { blink(SEND_RETRIES, 150); }     // hard stop, blink pattern
  }

  Serial.print(F("ACC idle threshold: ")); Serial.print(ACC_OFF_MV); Serial.println(F(" mV"));
  Serial.print(F("Lock frame: 0x"));
  Serial.println(LOCK_FRAME_ID, HEX);
  Serial.print(F("Window frames: ")); Serial.println(NUM_WINDOWS);

  // Initial state, so we can detect a falling edge later.
  _accOn = (readAccMv() >= ACC_OFF_MV);
  if (_accOn) Serial.println(F("ACC: ON"));
  else {
    Serial.println(F("ACC: OFF at boot"));
#if AUTO_CLOSE_AT_BOOT
    scheduleClose(F("ACC off at boot"));
#endif
  }
}

/* ================================= LOOP ====================================== */
void loop() {
  readCanPipe();        // keep ears open (door-lock events)
  pollAcc();            // ACC power-off events
  runPendingClose();    // fire the delayed close if it is due
  handleSerial();       // manual test commands over USB
}

/* ============================= EVENT SOURCES ================================= */

/* ACC monitored on the analog pin. Debounced, falls once per edge. */
static void pollAcc() {
  bool on = (readAccMv() >= ACC_OFF_MV);
  if (on != _accOn) {
    if (!on) {
      // Low for ACC_OFF_DEBOUNCE_MS in a row and this is a real transition.
      if (_accLowSinceMs == 0) _accLowSinceMs = millis();
      if (millis() - _accLowSinceMs >= ACC_OFF_DEBOUNCE_MS) {
        _accOn = false;
        Serial.println(F("ACC: OFF (debounced)"));
#if TRIGGER_ACC_OFF
        scheduleClose(F("ACC off"));
#endif
      }
    } else {
      _accLowSinceMs = 0;
      _accOn = true;
      Serial.println(F("ACC: ON"));
    }
  } else if (on) {
    _accLowSinceMs = 0;                          // stay primed for the next drop
  }
}

/* CAN receive pipe: inspect every frame for the door-lock event signature. */
static void readCanPipe() {
  if (CAN0.checkReceive() != CAN_MSGAVAIL) return;

  CAN0.readMsgBufID(&_rxFrameId, &_rxLen, _rxData);

#if TRIGGER_DOOR_LOCK
  // readMsgBufID(3-arg) flags an extended ID in bit31; normalize both to compare.
  uint32_t frameId = _rxFrameId & 0x1FFFFFFFUL;
  bool frameIsExt  = (_rxFrameId & 0x80000000UL) != 0;

  bool matchesId  = (frameId == LOCK_FRAME_ID) && (frameIsExt == (LOCK_FRAME_EXT != 0));
  bool isLocked   = (matchesId && _rxLen > LOCK_BYTE && ((_rxData[LOCK_BYTE] & LOCK_MASK) == LOCK_VALUE));

  // Only react while the car is off: a lock event while driving shouldn't close windows.
  if (isLocked && !_accOn) {
    Serial.println(F("Door-lock event while ACC off -> close"));
    scheduleClose(F("door lock"));
  }
#endif
}

/* ================================= ACTIONS =================================== */

/* Schedule one close sequence EVENT_COOLDOWN_MS after the previous one. */
static void scheduleClose(const char *reason) {
  unsigned long now = millis();
  if (now < _nextEventMs) {
    Serial.println(F("(cooldown active, skipping)"));
    return;
  }
  _nextEventMs = now + EVENT_COOLDOWN_MS;
  _closeAtMs   = now + CLOSE_DELAY_MS;
  _closeReason = reason;
  Serial.print(F("Will close windows in "));
  Serial.print(CLOSE_DELAY_MS);
  Serial.print(F(" ms  ("));
  Serial.print(reason);
  Serial.println(F(")"));
}

static void runPendingClose() {
  if (_closeAtMs == 0) return;
  if ((long)(millis() - _closeAtMs) < 0) return;   // not yet (millis wrap-safe)
  _closeAtMs = 0;
  closeAllWindows(_closeReason);
}

/* Send every "close" frame in WINDOWS[]. All windows even if some fail. */
static void closeAllWindows(const char *reason) {
  Serial.print(F("Closing all windows ("));
  Serial.print(reason ? reason : "manual");
  Serial.println(F(")"));

  for (uint8_t w = 0; w < NUM_WINDOWS; w++) {
    sendWindowFrame(w);
    delay(CLOSE_SPACING_MS);
  }
  Serial.println(F("Close sequence done"));
}

static void sendWindowFrame(uint8_t w) {
  const WinClose &wc = WINDOWS[w];

  for (uint8_t attempt = 1; attempt <= SEND_RETRIES; attempt++) {
    uint32_t frameId = wc.ext ? (wc.id | 0x80000000UL) : wc.id;
    byte result = CAN0.sendMsgBuf(frameId, wc.ext, wc.dlc, wc.data);

    if (result == CAN_OK) {
      Serial.print(F("  sent ")); Serial.print(wc.name);
      Serial.print(F("  ID=0x"));  Serial.print(wc.id, HEX);
      Serial.print(F(" data="));
      for (uint8_t i = 0; i < wc.dlc; i++) { Serial.print(wc.data[i], HEX); Serial.print(F(" ")); }
      Serial.println();
      blink(2, 60);                              // status LED confirmation
      return;
    }
    Serial.print(F("  tx failed ")); Serial.print(wc.name);
    Serial.print(F(" (attempt ")); Serial.print(attempt); Serial.println(F(")"));
    delay(50);
  }
  Serial.print(F("  giving up on ")); Serial.println(wc.name);
}

/* ================================ HELPERS ==================================== */

/* ACC voltage in millivolts (5 V reference, 10-bit ADC). */
static unsigned long readAccMv() {
  return (unsigned long)analogRead(ACC_SENSE_PIN) * 5000UL / 1023UL;
}

static void blink(uint8_t times, uint16_t halfPeriodMs) {
  for (uint8_t i = 0; i < times; i++) {
    ledOn();  delay(halfPeriodMs);
    ledOff(); delay(halfPeriodMs);
  }
}

/* USB serial test harness (handy before wiring the car): send 'c' to close. */
static void handleSerial() {
  if (!Serial.available()) return;

  char c = (char)Serial.read();
  switch (c) {
    case 'c':
    case 'C':
      closeAllWindows("serial-cmd");
      break;
    case 'a':
    case 'A':
      Serial.print(F("ACC mV: ")); Serial.println(readAccMv());
      break;
    case 'h':
    case 'H':
    case '?':
      Serial.println(F("commands: c=close all windows, a=print ACC mV, ?=help"));
      break;
    default:
      break;
  }
}
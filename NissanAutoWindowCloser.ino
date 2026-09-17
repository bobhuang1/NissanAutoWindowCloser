/*
 * NissanAutoWindowCloser  —  Arduino Nano + MCP2515 CAN module
 * ---------------------------------------------------------------------------------
 * Automatic window closer for a Nissan Sylphy (sample code).
 *
 * Mission:
 *   1. Detect "ACC power off"  (12V signal on a voltage divider, pin A0)
 *      and/or a "door lock" event sniffed on the CAN bus.
 *   2. When either happens, send CAN frames that command all 4 windows to close.
 *   3. Detect any door / trunk / rear-hatch open -> switch on the hazard lights,
 *      and switch them back off once everything is closed.
 *   4. If 3 "unlock" signals arrive within 3 s while the car is parked/locked,
 *      roll all 4 windows down automatically.
 *   5. While the car is running, a hard deceleration (speed drop above the
 *      DECEL threshold) flashes the hazards for 10 s to warn cars behind.
 *   6. During every automatic window-close, the headlights come on and go back
 *      off when the sequence is fully done.
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
#define EVENT_COOLDOWN_MS 10000UL              // minimum gap between automatic close/roll-downs
#define AUTO_CLOSE_AT_BOOT 0                   // 1 = close even if ACC already OFF at boot

/* --- Event sources ---------------------------------------------------------- */
#define TRIGGER_ACC_OFF   1                    // close when ACC switches OFF
#define TRIGGER_DOOR_LOCK 1                    // close when a lock frame arrives while ACC OFF
#define TRIGGER_HAZARD_ON_DOOR 1               // hazards while any door/trunk is open (parked)
#define TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK 1   // 3 unlocks in <3 s -> all windows down

/* --- Door lock / unlock event (placeholder values!) --------------------------- */
#define LOCK_FRAME_ID     0x2A2UL              // placeholder: BCM door lock frame
#define LOCK_FRAME_EXT    0                    // 0 = standard ID, 1 = extended
#define LOCK_BYTE         1                    // byte index to inspect
#define LOCK_MASK         0x03                 // bit mask on that byte
#define LOCK_VALUE        0x01                 // 0x01 = "locked"   (placeholder)
#define UNLOCK_VALUE      0x02                 // 0x02 = "unlocked" (placeholder)

/* --- Roll windows down after a triple unlock -------------------------------- */
#define TRIPLE_UNLOCK_WINDOW_MS 3000UL         // 3 unlocks inside this window -> roll down
#define ROLL_DOWN_DELAY_MS    1000UL           // delay AFTER the 3rd unlock
#define ROLL_DOWN_SPACING_MS  250UL            // pause between window-down frames

/* --- Door / tailgate open sensing (placeholder values!) ---------------------- */
#define DOOR_FRAME_ID     0x1B9UL              // placeholder: BCM door/tailgate status frame
#define DOOR_FRAME_EXT    0
#define DOOR_BYTE0        0                    // low byte  of the door bitfield
#define DOOR_BYTE1        1                    // high byte of the door bitfield
#define DOOR_BITS         0x000F               // FL|FR|RL|RR, add a bit for the tailgate (placeholder)
#define DOOR_DEBOUNCE_MS  150UL                // door-switch debounce
#define HAZARD_ONLY_WHEN_PARKED 1              // 1 = hazard only while ACC is OFF

/* --- Hazard lights command (placeholder values!) ----------------------------- */
#define HAZARD_FRAME_ID   0x4B2UL              // placeholder: BCM hazard request ID
#define HAZARD_FRAME_EXT  0
#define HAZARD_DLC        8
#define HAZARD_ON_DATA    { 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 }  // placeholder
#define HAZARD_OFF_DATA   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }  // placeholder

/* --- Brake-hazard: flash the hazards on a hard deceleration ------------------- */
#define TRIGGER_BRAKE_WARNING_LIGHTS 1         // car running + fast decel -> 10 s hazards
#define SPEED_FRAME_ID     0x1D0UL             // placeholder: vehicle speed frame
#define SPEED_FRAME_EXT    0
#define SPEED_BYTE         1                   // byte that holds vehicle speed
#define SPEED_SCALE        1                   // km/h per raw unit (placeholder)
#define MIN_SPEED_KMH      20                  // below this speed no brake warning
#define DECEL_THRESHOLD_KMPHS 25UL             // km/h lost per second to trigger
#define DECEL_MIN_DT_MS    100UL               // ignore sub-100 ms glitches
#define DECEL_MAX_DT_MS    2000UL              // ignore long gaps / edge cases
#define BRAKE_HAZARD_MS    10000UL             // how long the hazards flash
#define BRAKE_COOLDOWN_MS  20000UL             // min pause before re-triggering

/* --- Headlights during the automatic window-close ----------------------------- */
#define LIGHTS_WHILE_CLOSE 1                   // turn on headlights while closing
#define HEADLIGHT_FRAME_ID  0x2C0UL            // placeholder: BCM headlamp request
#define HEADLIGHT_FRAME_EXT 0
#define HEADLIGHT_DLC       8
#define HEADLIGHT_ON_DATA   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }  // placeholder
#define HEADLIGHT_OFF_DATA  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }  // placeholder
#define HEADLIGHTS_HOLD_MS  1500UL             // keep on briefly after the last frame

/* --- STATUS / debug ---------------------------------------------------------- */
#define STATUS_LED        9                    // integrated PCB LED2 -> PB1/D9 (NOT D13, that is SPI SCK!)
#define LED_ACTIVE_HIGH   0                    // PCB: LED anode to +5V -> shines when pin is LOW. UNO built-in LED on D13 would be 1.
#define ledOn()           digitalWrite(STATUS_LED, LED_ACTIVE_HIGH ? HIGH : LOW)
#define ledOff()          digitalWrite(STATUS_LED, LED_ACTIVE_HIGH ? LOW  : HIGH)

/* ==================== WINDOW FRAME TABLES ====================================
 * PLACEHOLDERS! For a real Sylphy these must be replaced with frames recorded
 * from the car's own bus while the windows actually move (see README).
 * `ext` marks an extended (29-bit) ID: 1 = extended, 0 = standard.
 * ============================================================================*/
typedef struct {
  const char *name;                 // window name for logging
  uint32_t    id;                   // CAN frame ID
  uint8_t     ext;                  // 0 standard / 1 extended
  uint8_t     dlc;                  // payload length (<= 8)
  uint8_t     data[8];              // payload bytes
} WinFrame;

/* Windows UP ("close") — used by the auto-close triggers. */
static const WinFrame WINDOWS_UP[] = {
  { "FL", 0x180UL, 0, 8, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00} }, // driver
  { "FR", 0x181UL, 0, 8, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00} }, // passenger
  { "RL", 0x182UL, 0, 8, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00} }, // rear left
  { "RR", 0x183UL, 0, 8, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00} }, // rear right
};
#define NUM_WINDOWS_UP  (sizeof(WINDOWS_UP) / sizeof(WINDOWS_UP[0]))

/* Windows DOWN ("open/roll down") — used by the triple-unlock trigger. */
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static const WinFrame WINDOWS_DOWN[] = {
  { "FL", 0x180UL, 0, 8, {0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00} }, // driver
  { "FR", 0x181UL, 0, 8, {0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00} }, // passenger
  { "RL", 0x182UL, 0, 8, {0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00} }, // rear left
  { "RR", 0x183UL, 0, 8, {0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00} }, // rear right
};
#define NUM_WINDOWS_DOWN (sizeof(WINDOWS_DOWN) / sizeof(WINDOWS_DOWN[0]))
#endif

/* ============================== STATE ======================================== */
MCP_CAN CAN0(SPI_CS_PIN);

static unsigned long _nextEventMs = 0;          // millis() gate for EVENT_COOLDOWN_MS
static unsigned long _closeAtMs   = 0;          // pending close timer
static const char  *_closeReason  = nullptr;    // reason for the pending close

static bool _accOn          = false;            // last stable ACC state
static unsigned long _accLowSinceMs = 0;        // when the ACC reading first went low

#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static unsigned long _openAtMs   = 0;           // pending roll-down timer
static const char  *_openReason  = nullptr;     // reason for the pending roll-down
#endif

#if TRIGGER_DOOR_LOCK || TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static bool _carLocked          = false;        // last lock/unlock on the bus
static uint8_t   _unlockCount   = 0;            // unlock burst counter
static unsigned long _unlockWindowStartMs = 0;  // when the first unlock landed
#endif

#if TRIGGER_HAZARD_ON_DOOR
static bool _doorsOpen            = false;      // debounced door/tailgate state
static bool _doorDebouncePending  = false;      // edge seen, debounce running
static unsigned long _doorsChangeAtMs = 0;
#endif

#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
static bool    _hazardsOn        = false;      // last hazard command we TX'd
static uint8_t _hazardDemands    = 0;          // OR of hazard requests
#define HAZARD_REQ_DOOR    0x01
#define HAZARD_REQ_BRAKE   0x02
#define HAZARD_REQ_MANUAL  0x04
#endif

#if TRIGGER_BRAKE_WARNING_LIGHTS
static bool         _brakeHazardsActive = false;
static unsigned long _brakeHazardUntilMs  = 0;
static unsigned long _brakeCooldownUntilMs = 0;
static unsigned long _lastSpeedKmh = 0;        // previous speed sample
static unsigned long _lastSpeedAtMs = 0;       // when it was sampled
#endif

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
  Serial.print(F("Lock frame: 0x")); Serial.println(LOCK_FRAME_ID, HEX);
  Serial.print(F("Window up frames: "));   Serial.println(NUM_WINDOWS_UP);
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
  Serial.print(F("Window down frames: ")); Serial.println(NUM_WINDOWS_DOWN);
#endif
#if TRIGGER_HAZARD_ON_DOOR
  Serial.print(F("Door status frame: 0x")); Serial.println(DOOR_FRAME_ID, HEX);
  Serial.print(F("Hazard frame: 0x"));      Serial.println(HAZARD_FRAME_ID, HEX);
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
  Serial.print(F("Speed frame: 0x"));       Serial.println(SPEED_FRAME_ID, HEX);
  Serial.print(F("Decel threshold: "));     Serial.print(DECEL_THRESHOLD_KMPHS); Serial.println(F(" km/h per s"));
#endif
#if LIGHTS_WHILE_CLOSE
  Serial.print(F("Headlight frame: 0x"));   Serial.println(HEADLIGHT_FRAME_ID, HEX);
#endif

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
  readCanPipe();        // keep ears open (lock / door-lock events)
  pollAcc();            // ACC power-off events
  runPendingClose();    // fire the delayed close if it is due
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
  runPendingRollDown(); // fire the delayed roll-down if it is due
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
  pollBrakeWarning();   // count down an active brake-hazard burst
#endif
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
  updateHazards();      // reconcile all hazard requests -> one TX on change
#endif
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
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
        _hazardDemands = 0;                        // fresh car-off state
#endif
#if TRIGGER_HAZARD_ON_DOOR
        // Doors may already be open from when ACC was running — light up now.
        if (_doorsOpen) _hazardDemands |= HAZARD_REQ_DOOR;
#endif
      }
    } else {
      _accLowSinceMs = 0;
      _accOn = true;
      Serial.println(F("ACC: ON"));
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
      _hazardDemands = 0;                          // driving resets every hazard request
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
      _brakeHazardsActive = false;
      _lastSpeedAtMs = 0;                          // re-arm the speed tracker
#endif
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
      _unlockCount = 0;                         // a fresh drive resets the unlock burst
#endif
    }
  } else if (on) {
    _accLowSinceMs = 0;                          // stay primed for the next drop
  }
}

/* CAN receive pipe: dispatch every frame to the event handlers above. */
static void readCanPipe() {
  if (CAN0.checkReceive() != CAN_MSGAVAIL) return;

  CAN0.readMsgBufID(&_rxFrameId, &_rxLen, _rxData);

#if TRIGGER_DOOR_LOCK || TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
  handleLockFrame();
#endif
#if TRIGGER_HAZARD_ON_DOOR
  handleDoorFrame();
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
  handleSpeedFrame();
#endif
}

/* --- Door lock / unlock frames ------------------------------------------------ */
#if TRIGGER_DOOR_LOCK || TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static void handleLockFrame() {
  if (!frameIdMatches(LOCK_FRAME_ID, LOCK_FRAME_EXT)) return;
  if (_rxLen <= LOCK_BYTE) return;

  uint8_t code = _rxData[LOCK_BYTE] & LOCK_MASK;

  if (code == LOCK_VALUE) {
    _carLocked = true;
    _unlockCount = 0;                            // fresh lock resets the burst counter
    Serial.println(F("Door lock event"));

#if TRIGGER_DOOR_LOCK
    // Only react while the car is off: a lock event while driving shouldn't close windows.
    if (!_accOn) {
      Serial.println(F("Door-lock event while ACC off -> close"));
      scheduleClose(F("door lock"));
    }
#endif
  }
  else if (code == UNLOCK_VALUE) {
    _carLocked = false;                          // latch unlocked
    Serial.println(F("Door unlock event"));

#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
    handleUnlockBurst();
#endif
  }
#endif
}

/* 3 unlocks inside TRIPLE_UNLOCK_WINDOW_MS -> roll down the windows. */
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static void handleUnlockBurst() {
  if (_accOn) return;                            // parked / off only

  unsigned long now = millis();
  if (_unlockCount == 0 || now - _unlockWindowStartMs > TRIPLE_UNLOCK_WINDOW_MS) {
    _unlockWindowStartMs = now;                  // start a fresh window with this event
    _unlockCount = 1;
  } else {
    _unlockCount++;
  }

  Serial.print(F("Unlock burst: ")); Serial.println(_unlockCount);

  if (_unlockCount >= 3) {
    _unlockCount = 0;
    Serial.println(F("Triple unlock detected -> roll windows down"));
    scheduleRollDown(F("triple unlock"));
  }
}
#endif

/* --- Door / tailgate status frames ---------------------------------------------- */
#if TRIGGER_HAZARD_ON_DOOR
static void handleDoorFrame() {
  if (!frameIdMatches(DOOR_FRAME_ID, DOOR_FRAME_EXT)) return;
  if (_rxLen <= DOOR_BYTE1) return;

  uint16_t openMask = (uint16_t)_rxData[DOOR_BYTE0]
                    | ((uint16_t)_rxData[DOOR_BYTE1] << 8);
  bool anyOpen = (openMask & DOOR_BITS) != 0;

  // Same value as the debounced state -> done, restart any running debounce.
  if (anyOpen == _doorsOpen) {
    _doorDebouncePending = false;
    return;
  }

  if (!_doorDebouncePending) {
    _doorsChangeAtMs = millis();
    _doorDebouncePending = true;
    return;
  }
  if (millis() - _doorsChangeAtMs < DOOR_DEBOUNCE_MS) return;

  _doorDebouncePending = false;
  _doorsOpen = anyOpen;
  Serial.println(_doorsOpen ? F("Door/tailgate OPEN") : F("Door/tailgate CLOSED"));

  if (_doorsOpen) {
    if (HAZARD_ONLY_WHEN_PARKED && _accOn) {
      Serial.println(F("(parked-only: hazards skipped while driving)"));
    } else {
      _hazardDemands |= HAZARD_REQ_DOOR;
    }
  } else {
    _hazardDemands &= ~HAZARD_REQ_DOOR;
  }
}
#endif

/* --- Vehicle speed frames: detect a hard deceleration -> brake hazard ------ */
#if TRIGGER_BRAKE_WARNING_LIGHTS
static void handleSpeedFrame() {
  if (!frameIdMatches(SPEED_FRAME_ID, SPEED_FRAME_EXT)) return;
  if (_rxLen <= SPEED_BYTE) return;
  if (!_accOn) return;                             // car not running -> no brake warning

  unsigned long now  = millis();
  unsigned long spd  = (unsigned long)_rxData[SPEED_BYTE] * SPEED_SCALE;

  if (_lastSpeedAtMs != 0) {
    unsigned long dt = now - _lastSpeedAtMs;     // unsigned diff is wrap-safe
    if (dt >= DECEL_MIN_DT_MS && dt <= DECEL_MAX_DT_MS && spd < _lastSpeedKmh) {
      unsigned long dropPerSec = (_lastSpeedKmh - spd) * 1000UL / dt;

      if (dropPerSec >= DECEL_THRESHOLD_KMPHS && spd >= MIN_SPEED_KMH
          && !_brakeHazardsActive && now >= _brakeCooldownUntilMs) {
        Serial.print(F("Hard deceleration: lost "));
        Serial.print(_lastSpeedKmh - spd);
        Serial.println(F(" km/h across the sample window"));
        startBrakeWarning(now);
      }
    }
  }

  _lastSpeedKmh    = spd;
  _lastSpeedAtMs   = now;
}

static void startBrakeWarning(unsigned long now) {
  _brakeHazardsActive   = true;
  _brakeHazardUntilMs   = now + BRAKE_HAZARD_MS;
  _brakeCooldownUntilMs = now + BRAKE_HAZARD_MS + BRAKE_COOLDOWN_MS;
  _hazardDemands |= HAZARD_REQ_BRAKE;
  Serial.print(F("Brake hazard ON for "));
  Serial.print(BRAKE_HAZARD_MS);
  Serial.println(F(" ms"));
}

/* Count down the brake burst; clearing the demand lets updateHazards turn off. */
static void pollBrakeWarning() {
  if (!_brakeHazardsActive) return;
  unsigned long now = millis();
  if ((long)(now - _brakeHazardUntilMs) >= 0) {   // wrap-safe timer
    _brakeHazardsActive = false;
    _hazardDemands &= ~HAZARD_REQ_BRAKE;
    Serial.println(F("Brake hazard OFF"));
  }
}
#endif

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

/* Schedule one roll-down, also gated by the shared EVENT_COOLDOWN_MS. */
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static void scheduleRollDown(const char *reason) {
  unsigned long now = millis();
  if (now < _nextEventMs) {
    Serial.println(F("(cooldown active, skipping)"));
    return;
  }
  _nextEventMs = now + EVENT_COOLDOWN_MS;
  _openAtMs    = now + ROLL_DOWN_DELAY_MS;
  _openReason  = reason;
  Serial.print(F("Will roll windows down in "));
  Serial.print(ROLL_DOWN_DELAY_MS);
  Serial.print(F(" ms  ("));
  Serial.print(reason);
  Serial.println(F(")"));
}

static void runPendingRollDown() {
  if (_openAtMs == 0) return;
  if ((long)(millis() - _openAtMs) < 0) return;   // not yet (millis wrap-safe)
  _openAtMs = 0;
  openAllWindows(_openReason);
}
#endif

static void runPendingClose() {
  if (_closeAtMs == 0) return;
  if ((long)(millis() - _closeAtMs) < 0) return;   // not yet (millis wrap-safe)
  _closeAtMs = 0;
  closeAllWindows(_closeReason);
}

/* Send every "up" frame in WINDOWS_UP[]. Headlights guard the whole sequence. */
static void closeAllWindows(const char *reason) {
  Serial.print(F("Closing all windows ("));
  Serial.print(reason ? reason : "manual");
  Serial.println(F(")"));

#if LIGHTS_WHILE_CLOSE
  static const uint8_t hlOnData[HEADLIGHT_DLC]  = HEADLIGHT_ON_DATA;
  static const uint8_t hlOffData[HEADLIGHT_DLC] = HEADLIGHT_OFF_DATA;
  sendFrame(HEADLIGHT_FRAME_ID, HEADLIGHT_FRAME_EXT, HEADLIGHT_DLC,
            hlOnData, "headlights ON");
#endif

  for (uint8_t w = 0; w < NUM_WINDOWS_UP; w++) {
    const WinFrame &wf = WINDOWS_UP[w];
    sendFrame(wf.id, wf.ext, wf.dlc, wf.data, wf.name);
    delay(CLOSE_SPACING_MS);
  }

#if LIGHTS_WHILE_CLOSE
  delay(HEADLIGHTS_HOLD_MS);                     // let the last frame finish the roll
  sendFrame(HEADLIGHT_FRAME_ID, HEADLIGHT_FRAME_EXT, HEADLIGHT_DLC,
            hlOffData, "headlights OFF");
#endif

  Serial.println(F("Close sequence done"));
}

/* Send every "down" frame in WINDOWS_DOWN[]. */
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static void openAllWindows(const char *reason) {
  Serial.print(F("Rolling all windows down ("));
  Serial.print(reason ? reason : "manual");
  Serial.println(F(")"));

  for (uint8_t w = 0; w < NUM_WINDOWS_DOWN; w++) {
    const WinFrame &wf = WINDOWS_DOWN[w];
    sendFrame(wf.id, wf.ext, wf.dlc, wf.data, wf.name);
    delay(ROLL_DOWN_SPACING_MS);
  }
  Serial.println(F("Roll-down sequence done"));
}
#endif

/* Generic CAN frame sender with retries + status LED confirmation. */
static bool sendFrame(uint32_t id, uint8_t ext, uint8_t dlc,
                      const uint8_t *data, const char *what) {
  uint32_t frameId = ext ? (id | 0x80000000UL) : id;

  for (uint8_t attempt = 1; attempt <= SEND_RETRIES; attempt++) {
    byte result = CAN0.sendMsgBuf(frameId, ext, dlc, (INT8U *)data);

    if (result == CAN_OK) {
      Serial.print(F("  sent ")); Serial.print(what);
      Serial.print(F("  ID=0x")); Serial.print(id, HEX);
      Serial.print(F(" data="));
      for (uint8_t i = 0; i < dlc; i++) { Serial.print(data[i], HEX); Serial.print(F(" ")); }
      Serial.println();
      blink(2, 60);                              // status LED confirmation
      return true;
    }
    Serial.print(F("  tx failed ")); Serial.print(what);
    Serial.print(F(" (attempt ")); Serial.print(attempt); Serial.println(F(")"));
    delay(50);
  }
  Serial.print(F("  giving up on ")); Serial.println(what);
  return false;
}

/* Reconcile all hazard requests (door / brake / manual) -> one TX on change. */
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
static void updateHazards() {
  bool want = (_hazardDemands != 0);
  if (want == _hazardsOn) return;

  static const uint8_t hazOnData[HAZARD_DLC]  = HAZARD_ON_DATA;
  static const uint8_t hazOffData[HAZARD_DLC] = HAZARD_OFF_DATA;

  sendFrame(HAZARD_FRAME_ID, HAZARD_FRAME_EXT, HAZARD_DLC,
            want ? hazOnData : hazOffData,
            want ? "hazards ON" : "hazards OFF");
  _hazardsOn = want;
}
#endif

/* ================================ HELPERS ==================================== */

/* Does the received frame (in _rxFrameId) match an ID/ext pair? */
static bool frameIdMatches(uint32_t id, uint8_t ext) {
  uint32_t normId = _rxFrameId & 0x1FFFFFFFUL;          // strip the ext flag bit31
  bool normExt   = (_rxFrameId & 0x80000000UL) != 0;
  return (normId == id) && (normExt == (ext != 0));
}

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

/* USB serial test harness (handy before wiring the car). */
static void handleSerial() {
  if (!Serial.available()) return;

  char c = (char)Serial.read();
  switch (c) {
    case 'c':
    case 'C':
      closeAllWindows("serial-cmd");
      break;
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
    case 'o':
    case 'O':
      openAllWindows("serial-cmd");
      break;
#endif
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
    case 'd':
    case 'D':
      _hazardDemands |= HAZARD_REQ_MANUAL;       // reconciled by loop()'s updateHazards()
      break;
    case 'f':
    case 'F':
      _hazardDemands &= ~HAZARD_REQ_MANUAL;
      break;
#endif
    case 'a':
    case 'A':
      Serial.print(F("ACC mV: ")); Serial.println(readAccMv());
      break;
    case 's':
    case 'S':
      Serial.print(F("ACC: ")); Serial.println(_accOn ? F("ON") : F("OFF"));
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
      Serial.print(F("Locked: ")); Serial.println(_carLocked ? F("yes") : F("no"));
      Serial.print(F("Unlock burst: ")); Serial.println(_unlockCount);
#endif
#if TRIGGER_HAZARD_ON_DOOR
      Serial.print(F("Door/tailgate open: ")); Serial.println(_doorsOpen ? F("yes") : F("no"));
#endif
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
      Serial.print(F("Hazard demands: 0x"));    Serial.println(_hazardDemands, HEX);
      Serial.print(F("Hazards TX: "));          Serial.println(_hazardsOn ? F("on") : F("off"));
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
      Serial.print(F("Last speed km/h: "));     Serial.println(_lastSpeedKmh);
      Serial.print(F("Brake hazard active: ")); Serial.println(_brakeHazardsActive ? F("yes") : F("no"));
#endif
      break;
    case 'h':
    case 'H':
    case '?':
      Serial.println(F("commands: c=windows up (close), o=windows down (roll), "
                       "d=hazards on, f=hazards off, a=ACC mV, s=status, ?=help"));
      break;
    default:
      break;
  }
}